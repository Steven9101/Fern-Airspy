// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Runs the session loop in a thread with real pipes in place of fds 0 to 3
// and plays FernSDR's part, against fake Airspys that stream a tone.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "fake_backend.h"
#include "json.h"
#include "r820t.h"
#include "receiver.h"
#include "session.h"
#include "test.h"

using fern::json::Value;
using Clock = std::chrono::steady_clock;

namespace {

struct Pipe {
    int r = -1;
    int w = -1;
    Pipe() {
        int fds[2];
        if (::pipe2(fds, O_CLOEXEC) == 0) {
            r = fds[0];
            w = fds[1];
        }
    }
    ~Pipe() {
        close_r();
        close_w();
    }
    void close_r() {
        if (r >= 0)
            ::close(r);
        r = -1;
    }
    void close_w() {
        if (w >= 0)
            ::close(w);
        w = -1;
    }
};

fern::SessionOptions quick_options() {
    fern::SessionOptions o;
    o.stats_interval = std::chrono::milliseconds(100);
    o.stall_timeout = std::chrono::milliseconds(800);
    o.shutdown_timeout = std::chrono::milliseconds(1500);
    o.ring_bytes = 1 << 20;
    return o;
}

const char* const open_line =
    "{\"type\":\"open\",\"sample_rate\":2500000,\"center\":100000000,\"signal\":\"iq\",\"settings\":{}}";
const char* const hf_open_line =
    "{\"type\":\"open\",\"sample_rate\":768000,\"center\":7100000,\"signal\":\"iq\",\"settings\":{}}";

class Harness {
public:
    // With nonblocking, the module's ends of fds 0, 1 and 3 are handed over
    // with O_NONBLOCK set, as a host that uses pipe2(O_NONBLOCK) would.
    explicit Harness(fake::Backend& backend, fern::SessionOptions options = quick_options(),
                     bool nonblocking = false) {
        REQUIRE(commands_.r >= 0 && samples_.r >= 0 && events_.r >= 0 && stop_.r >= 0);
        if (nonblocking)
            for (int fd : {commands_.r, samples_.w, events_.w})
                REQUIRE(::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK) == 0);
        fern::SessionIo io;
        io.commands = commands_.r;
        io.samples = samples_.w;
        io.events = events_.w;
        io.stop = stop_.r;
        thread_ = std::thread([this, &backend, io, options] {
            result_ = fern::run_session(backend, io, options);
            done_.store(true);
        });
    }

    ~Harness() {
        // Whatever happened, make the session end so that the thread can be
        // joined: this is what FernSDR's leaving looks like.
        commands_.close_w();
        samples_.close_r();
        events_.close_r();
        if (thread_.joinable())
            thread_.join();
    }

    void send(const std::string& line) {
        const std::string text = line + "\n";
        REQUIRE(::write(commands_.w, text.data(), text.size()) == static_cast<ssize_t>(text.size()));
    }
    void close_commands() { commands_.close_w(); }
    void close_samples() { samples_.close_r(); }
    void signal_stop() { REQUIRE(::write(stop_.w, "x", 1) == 1); }
    // Makes the events pipe hold only one page, so that a few unread events
    // fill it.
    void shrink_events() { REQUIRE(::fcntl(events_.w, F_SETPIPE_SZ, 4096) >= 0); }

    // The next event on fd 3, or null after the timeout.
    Value event(int timeout_ms = 3000) {
        const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
        for (;;) {
            const size_t nl = events_buf_.find('\n');
            if (nl != std::string::npos) {
                const std::string line = events_buf_.substr(0, nl);
                events_buf_.erase(0, nl + 1);
                Value v;
                std::string error;
                if (!fern::json::parse(line, v, error))
                    test::report(__FILE__, __LINE__, "event is not valid JSON: " + line + ": " + error);
                if (line.size() > 65536)
                    test::report(__FILE__, __LINE__, "event longer than 64 KiB");
                lines_.push_back(line);
                return v;
            }
            const int left = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count());
            if (left <= 0)
                return Value();
            struct pollfd pfd = {events_.r, POLLIN, 0};
            if (::poll(&pfd, 1, left) <= 0)
                continue;
            char buf[4096];
            const ssize_t n = ::read(events_.r, buf, sizeof buf);
            if (n <= 0)
                return Value();
            events_buf_.append(buf, static_cast<size_t>(n));
        }
    }

    // The next event of the given type, skipping stats. The timeout covers
    // all of them, so a stream of stats cannot keep a test waiting.
    Value expect(const std::string& type, int timeout_ms = 3000) {
        const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
        for (;;) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
            Value v = left > 0 ? event(static_cast<int>(left)) : Value();
            if (v.is_null()) {
                test::report(__FILE__, __LINE__, "no " + type + " event");
                throw test::Stop{};
            }
            const std::string t = v.find("type") ? v.find("type")->as_string() : "";
            if (t == type)
                return v;
            if (t != "stats") {
                test::report(__FILE__, __LINE__, "expected " + type + ", got " + lines_.back());
                throw test::Stop{};
            }
        }
    }

    // Reads exactly n sample bytes.
    std::vector<uint8_t> samples(size_t n, int timeout_ms = 3000) {
        std::vector<uint8_t> out;
        const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
        while (out.size() < n && Clock::now() < deadline) {
            struct pollfd pfd = {samples_.r, POLLIN, 0};
            if (::poll(&pfd, 1, 50) <= 0)
                continue;
            std::vector<uint8_t> buf(n - out.size());
            const ssize_t got = ::read(samples_.r, buf.data(), buf.size());
            if (got <= 0)
                break;
            out.insert(out.end(), buf.begin(), buf.begin() + got);
        }
        return out;
    }

    // Reads and discards sample bytes for a while; returns how many.
    size_t drain_samples(int ms) {
        size_t total = 0;
        const auto deadline = Clock::now() + std::chrono::milliseconds(ms);
        std::vector<uint8_t> buf(1 << 16);
        while (Clock::now() < deadline) {
            struct pollfd pfd = {samples_.r, POLLIN, 0};
            if (::poll(&pfd, 1, 20) <= 0)
                continue;
            const ssize_t got = ::read(samples_.r, buf.data(), buf.size());
            if (got <= 0)
                break;
            total += static_cast<size_t>(got);
        }
        return total;
    }

    bool samples_pending() {
        struct pollfd pfd = {samples_.r, POLLIN, 0};
        return ::poll(&pfd, 1, 0) > 0;
    }

    // Waits for the session to end and returns its exit status.
    int exit_status(int timeout_ms = 4000, bool clean = true) {
        const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
        while (!done_.load() && Clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        REQUIRE(done_.load());
        thread_.join();
        CHECK_EQ(result_.clean, clean);
        return result_.status;
    }

    bool done() const { return done_.load(); }
    const std::vector<std::string>& lines() const { return lines_; }

private:
    Pipe commands_;
    Pipe samples_;
    Pipe events_;
    Pipe stop_;
    std::thread thread_;
    std::atomic<bool> done_{false};
    fern::SessionResult result_;
    std::string events_buf_;
    std::vector<std::string> lines_;
};

void check_hello(Harness& h) {
    const Value hello = h.event();
    REQUIRE(hello.is_object());
    CHECK_EQ(fern::json::serialize(hello),
             std::string("{\"type\":\"hello\",\"api\":1,\"id\":\"airspy\",\"version\":\"") + FERN_AIRSPY_VERSION +
                 "\",\"kind\":\"input\"}");
}

std::string text_of(const Value& v, const char* key) {
    const Value* f = v.find(key);
    return f && f->is_string() ? f->as_string() : "";
}

constexpr double two_pi = 6.283185307179586476925286766559;

// The frequency of the strongest tone in I/Q pairs: roughly from the mean
// phase step, then exactly from how far the phase moved between a stretch
// at the start and one at the end.
template <typename T>
double frequency_of(const T* iq, size_t pairs, double rate) {
    std::complex<double> sum = 0;
    for (size_t n = 1; n < pairs; ++n) {
        const std::complex<double> a(iq[2 * n - 2], iq[2 * n - 1]);
        const std::complex<double> b(iq[2 * n], iq[2 * n + 1]);
        sum += b * std::conj(a);
    }
    const double coarse = std::arg(sum) * rate / two_pi;
    const size_t len = 4096;
    auto stretch = [&](size_t from) {
        std::complex<double> s = 0;
        for (size_t n = from; n < from + len; ++n)
            s += std::complex<double>(iq[2 * n], iq[2 * n + 1]) * std::polar(1.0, -two_pi * coarse * n / rate);
        return s;
    };
    const size_t last = pairs - len;
    return coarse + std::arg(stretch(last) * std::conj(stretch(0))) * rate / (two_pi * last);
}

std::vector<int16_t> as_s16(const std::vector<uint8_t>& bytes) {
    std::vector<int16_t> v(bytes.size() / 2);
    std::memcpy(v.data(), bytes.data(), v.size() * 2);
    return v;
}

std::vector<float> as_f32(const std::vector<uint8_t>& bytes) {
    std::vector<float> v(bytes.size() / 4);
    std::memcpy(v.data(), bytes.data(), v.size() * 4);
    return v;
}

// The newest stats event: those the module sent while the test read
// samples wait in the pipe, oldest first. Stats keep coming every 100 ms, so
// it reads until the pipe has been quiet for 20 ms, and gives up waiting
// for that after a hundred events.
Value latest_stats(Harness& h) {
    Value newest;
    for (int i = 0; i < 100; ++i) {
        Value v = h.event(20);
        if (v.is_null())
            break;
        if (text_of(v, "type") == "stats")
            newest = v;
    }
    return newest.is_null() ? h.expect("stats") : newest;
}

int exit_after_open(fake::Backend& backend, const std::string& line, Value& error) {
    Harness h(backend);
    check_hello(h);
    h.send(line);
    error = h.expect("error");
    return h.exit_status();
}

}  // namespace

TEST(session_full_exchange_with_an_r2) {
    fake::Backend backend({fake::Spec{}});
    Harness h(backend);
    check_hello(h);
    h.send("{\"type\":\"open\",\"sample_rate\":2500000,\"center\":100000000,\"signal\":\"iq\","
           "\"settings\":{\"gain\":\"auto\",\"bias_tee\":false}}");
    const Value ready = h.expect("ready");
    CHECK_EQ(text_of(ready, "format"), std::string("s16"));
    CHECK_EQ(text_of(ready, "signal"), std::string("iq"));
    CHECK_EQ(ready.find("sample_rate")->as_number(), 2500000.0);
    // The PLL's remainder is shifted out, so the band is where it was asked
    // to be.
    CHECK_EQ(ready.find("center")->as_number(), 100000000.0);
    CHECK_EQ(text_of(*ready.find("device"), "name"), std::string("Airspy R2"));
    CHECK_EQ(text_of(*ready.find("settings"), "gain"), std::string("auto"));
    CHECK_EQ(text_of(*ready.find("settings"), "packing"), std::string("off"));

    // The fake's tone, 100 kHz above the centre, is at 100 kHz to a small
    // fraction of a hertz: the R820T's PLL landed some hertz off.
    const std::vector<uint8_t> bytes = h.samples(4 * 500000);
    REQUIRE(bytes.size() == 4 * 500000);
    const std::vector<int16_t> iq = as_s16(bytes);
    CHECK(std::fabs(frequency_of(iq.data() + 2000, iq.size() / 2 - 1000, 2500000) - 100000) < 0.05);
    const auto offset = fern::one_tuning(25000000, 100000000, 2500000)->offset;
    CHECK(std::fabs(offset) > 1);

    const Value stats = h.expect("stats");
    CHECK(stats.find("samples")->as_number() > 0);
    CHECK_EQ(stats.find("dropped")->as_number(), 0.0);
    CHECK_EQ(stats.find("clipping")->as_number(), 0.0);

    h.send("{\"type\":\"set\",\"id\":7,\"settings\":{\"gain\":7}}");
    CHECK_EQ(fern::json::serialize(h.expect("applied")),
             std::string("{\"type\":\"applied\",\"id\":7,\"settings\":{\"gain\":7}}"));
    CHECK_EQ(backend.last()->gain, std::string("linearity 7"));

    h.send("{\"type\":\"set\",\"id\":\"eight\",\"settings\":{\"bias_tee\":true}}");
    CHECK_EQ(fern::json::serialize(h.expect("applied")),
             std::string("{\"type\":\"applied\",\"id\":\"eight\",\"settings\":{\"bias_tee\":true}}"));
    CHECK(backend.last()->bias_tee);

    // Refused sets answer with a non-fatal error carrying the id, and the
    // stream goes on.
    h.send("{\"type\":\"set\",\"id\":9,\"settings\":{\"packing\":\"on\"}}");
    const Value refused = h.expect("error");
    CHECK_EQ(refused.find("id")->as_number(), 9.0);
    CHECK_EQ(text_of(refused, "code"), std::string("invalid"));
    CHECK_HAS(text_of(refused, "message"), "module.packing cannot change while the band runs");
    CHECK(!refused.find("fatal")->as_bool());
    h.send("{\"type\":\"set\",\"id\":10,\"settings\":{\"attenuation\":6}}");
    const Value other = h.expect("error");
    CHECK_EQ(other.find("id")->as_number(), 10.0);
    CHECK_HAS(text_of(other, "message"), "module.attenuation is for the Airspy HF+ only");
    h.send("{\"type\":\"set\",\"id\":11,\"settings\":{\"volume\":1}}");
    CHECK_HAS(text_of(h.expect("error"), "message"), "unknown setting module.volume");
    CHECK(h.drain_samples(200) > 0);

    // stats only grow.
    const double before = h.expect("stats").find("samples")->as_number();
    h.drain_samples(250);
    const double after = h.expect("stats").find("samples")->as_number();
    CHECK(after > before);

    h.send("{\"type\":\"stop\"}");
    CHECK_EQ(h.exit_status(), 0);
    const auto st = backend.last();
    CHECK(st->closed);
    CHECK(!st->bias_tee);  // switched off again before closing
    CHECK_EQ(st->stops, 1);
}

TEST(session_takes_76_hz_out_at_1_ghz) {
    fake::Spec spec;
    spec.tone_offset = -300000;
    fake::Backend backend({spec});
    Harness h(backend);
    check_hello(h);
    h.send("{\"type\":\"open\",\"sample_rate\":2500000,\"center\":1000000000,\"signal\":\"iq\"}");
    const Value ready = h.expect("ready");
    CHECK_EQ(ready.find("center")->as_number(), 1000000000.0);
    const double offset = ready.find("settings")->find("pll_offset")->as_number();
    CHECK(std::fabs(offset) > 20);
    const std::vector<int16_t> iq = as_s16(h.samples(4 * 500000));
    REQUIRE(iq.size() == 2 * 500000);
    CHECK(std::fabs(frequency_of(iq.data() + 2000, iq.size() / 2 - 1000, 2500000) - (-300000)) < 0.05);
    h.close_commands();
    CHECK_EQ(h.exit_status(), 0);
}

TEST(session_streams_floats_from_an_hf_plus) {
    fake::Spec spec;
    spec.family = fern::Family::hf;
    spec.tone_offset = 12345;
    fake::Backend backend({spec});
    Harness h(backend);
    check_hello(h);
    h.send(hf_open_line);
    const Value ready = h.expect("ready");
    CHECK_EQ(text_of(ready, "format"), std::string("f32"));
    CHECK_EQ(ready.find("center")->as_number(), 7100000.0);
    CHECK_EQ(text_of(*ready.find("device"), "name"), std::string("Airspy HF+ Discovery"));
    const std::vector<float> iq = as_f32(h.samples(8 * 100000));
    REQUIRE(iq.size() == 2 * 100000);
    CHECK(std::fabs(iq[0] - 0.5f) < 1e-6f);
    CHECK(std::fabs(frequency_of(iq.data(), iq.size() / 2, 768000) - 12345) < 0.01);
    // No clipping figure: the HF+ does not show its converter.
    const Value stats = h.expect("stats");
    CHECK(!stats.find("clipping"));
    h.send("{\"type\":\"set\",\"id\":1,\"settings\":{\"gain\":\"manual\",\"attenuation\":7}}");
    CHECK_EQ(fern::json::serialize(h.expect("applied")),
             std::string("{\"type\":\"applied\",\"id\":1,\"settings\":{\"gain\":\"manual\",\"attenuation\":12}}"));
    h.send("{\"type\":\"stop\"}");
    CHECK_EQ(h.exit_status(), 0);
    CHECK(backend.last()->closed);
}

TEST(session_brings_the_gain_down_when_the_converter_clips) {
    fake::Spec spec;
    // At linearity 10, where gain = auto starts, the tone swings twice as
    // far as the converter reaches.
    spec.amplitude = 4000;
    spec.follow_gain = true;
    fake::Backend backend({spec});
    fern::SessionOptions options = quick_options();
    options.gain_timing.settle = std::chrono::milliseconds(100);
    Harness h(backend, options);
    check_hello(h);
    h.send(open_line);
    h.expect("ready");
    // 6 dB down at once, two presets, to 8, where the tone peaks at 2005 of
    // 2048 and nothing clips.
    double most_clipping = 0;
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    std::string gain;
    while (Clock::now() < deadline) {
        h.drain_samples(50);
        const Value stats = h.expect("stats");
        REQUIRE(stats.find("clipping"));
        most_clipping = std::max(most_clipping, stats.find("clipping")->as_number());
        {
            std::lock_guard<std::mutex> lock(backend.last()->mutex);
            gain = backend.last()->gain;
        }
        if (gain == "linearity 8")
            break;
    }
    CHECK_EQ(gain, std::string("linearity 8"));
    CHECK(most_clipping > 0.1);
    // The module says it sets the gain itself: preset 8 at the ladder's
    // nominal 3 dB a step.
    REQUIRE(h.expect("stats").find("gain"));
    h.drain_samples(600);
    const Value settled = latest_stats(h);
    CHECK_EQ(settled.find("clipping")->as_number(), 0.0);
    CHECK_EQ(settled.find("gain")->as_number(), 24.0);
    {
        std::lock_guard<std::mutex> lock(backend.last()->mutex);
        CHECK_EQ(backend.last()->gain, std::string("linearity 8"));
    }
    // A preset of the operator's: no gain in the stats.
    h.send("{\"type\":\"set\",\"id\":1,\"settings\":{\"gain\":5}}");
    h.expect("applied");
    CHECK(!latest_stats(h).find("gain"));
    h.send("{\"type\":\"stop\"}");
    CHECK_EQ(h.exit_status(), 0);
}

TEST(session_reports_an_unplugged_airspy) {
    fake::Spec spec;
    spec.unplug_after = 300000;
    fake::Backend backend({spec});
    Harness h(backend);
    check_hello(h);
    h.send(open_line);
    h.expect("ready");
    h.drain_samples(100);
    const Value e = h.expect("error");
    CHECK_EQ(text_of(e, "code"), std::string("lost"));
    CHECK(e.find("fatal")->as_bool());
    CHECK_HAS(text_of(e, "message"), "the Airspy stopped streaming: it was unplugged or its USB connection failed");
    CHECK_HAS(text_of(e, "message"), "USB 2 or USB 3 port");
    CHECK_EQ(h.exit_status(), 5);
    CHECK(backend.last()->closed);
}

TEST(session_reports_an_airspy_that_goes_quiet) {
    fake::Spec spec;
    spec.family = fern::Family::hf;
    spec.stall_after = 50000;
    fake::Backend backend({spec});
    Harness h(backend);
    check_hello(h);
    h.send(hf_open_line);
    h.expect("ready");
    h.drain_samples(100);
    const Value e = h.expect("error", 3000);
    CHECK_EQ(text_of(e, "code"), std::string("lost"));
    CHECK_HAS(text_of(e, "message"), "the Airspy sent no samples for 0.8 seconds");
    CHECK_EQ(h.exit_status(), 5);
}

TEST(session_reports_a_start_that_fails) {
    fake::Spec spec;
    spec.fail_start = true;
    fake::Backend backend({spec});
    Harness h(backend);
    check_hello(h);
    h.send(open_line);
    // ready goes first: samples may follow it only.
    h.expect("ready");
    const Value e = h.expect("error");
    CHECK_EQ(text_of(e, "code"), std::string("usb"));
    CHECK_HAS(text_of(e, "message"), "the Airspy did not start streaming");
    CHECK_EQ(h.exit_status(), 5);
    CHECK(!h.samples_pending());
    CHECK(backend.last()->closed);
}

TEST(session_exits_with_the_status_of_each_open_failure) {
    Value e;
    {
        fake::Backend none;
        CHECK_EQ(exit_after_open(none, open_line, e), 3);
        CHECK_EQ(text_of(e, "code"), std::string("no-device"));
    }
    {
        fake::Spec locked;
        locked.open_error = fern::usb_error::access;
        fake::Backend backend({locked});
        CHECK_EQ(exit_after_open(backend, open_line, e), 5);
        CHECK_EQ(text_of(e, "message"), fern::permission_advice());
    }
    {
        fake::Spec held;
        held.open_error = fern::usb_error::busy;
        fake::Backend backend({held});
        CHECK_EQ(exit_after_open(backend, open_line, e), 4);
        CHECK_EQ(text_of(e, "code"), std::string("busy"));
    }
    {
        fake::Backend backend({fake::Spec{}});
        CHECK_EQ(exit_after_open(backend,
                                 "{\"type\":\"open\",\"sample_rate\":2500000,\"center\":100000000,\"signal\":\"iq\","
                                 "\"settings\":{\"squelch\":1}}",
                                 e),
                 6);
        CHECK_EQ(text_of(e, "code"), std::string("invalid"));
        CHECK_EQ(backend.opens(), 0);
    }
    {
        fake::Backend backend({fake::Spec{}});
        CHECK_EQ(exit_after_open(backend,
                                 "{\"type\":\"open\",\"sample_rate\":3000000,\"center\":100000000,\"signal\":\"iq\"}",
                                 e),
                 6);
        CHECK_HAS(text_of(e, "message"), "it runs at 10000000 and 2500000 samples/s");
        CHECK(backend.last()->closed);
    }
}

TEST(session_stops_on_a_signal_while_streaming) {
    fake::Backend backend({fake::Spec{}});
    Harness h(backend);
    check_hello(h);
    h.send(open_line);
    h.expect("ready");
    h.drain_samples(100);
    h.signal_stop();
    CHECK_EQ(h.exit_status(), 0);
    CHECK(backend.last()->closed);
}

TEST(session_stops_when_fernsdr_goes) {
    {
        fake::Backend backend({fake::Spec{}});
        Harness h(backend);
        check_hello(h);
        h.send(open_line);
        h.expect("ready");
        h.drain_samples(100);
        h.close_commands();
        CHECK_EQ(h.exit_status(), 0);
        CHECK(backend.last()->closed);
    }
    {
        fake::Backend backend({fake::Spec{}});
        Harness h(backend);
        check_hello(h);
        h.send(open_line);
        h.expect("ready");
        h.drain_samples(100);
        h.close_samples();
        CHECK_EQ(h.exit_status(), 0);
        CHECK(backend.last()->closed);
    }
}

TEST(session_works_with_nonblocking_fds) {
    fake::Spec fast;
    fast.realtime = false;
    fake::Backend backend({fast});
    fern::SessionOptions o = quick_options();
    o.ring_bytes = 1 << 18;
    Harness h(backend, o, true);
    check_hello(h);
    h.send(open_line);
    h.expect("ready");
    // Not reading fd 1 fills the pipe; the module must wait, not fail.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const std::vector<uint8_t> s = h.samples(400000);
    REQUIRE(s.size() == 400000);
    h.send("{\"type\":\"set\",\"id\":1,\"settings\":{\"gain\":20}}");
    CHECK_EQ(fern::json::serialize(h.expect("applied")),
             std::string("{\"type\":\"applied\",\"id\":1,\"settings\":{\"gain\":20}}"));
    CHECK(h.expect("stats").find("dropped")->as_number() > 0);
    const auto start = Clock::now();
    h.send("{\"type\":\"stop\"}");
    CHECK_EQ(h.exit_status(), 0);
    CHECK(Clock::now() - start < std::chrono::milliseconds(800));
}

TEST(session_ignores_what_it_does_not_know) {
    fake::Backend backend({fake::Spec{}});
    Harness h(backend);
    check_hello(h);
    h.send("not json at all");
    h.send("[1,2,3]");
    h.send("{\"no\":\"type\"}");
    h.send("{\"type\":\"future-command\",\"payload\":{\"x\":1}}");
    h.send("   ");
    h.send(std::string(70000, 'x'));
    h.send("{\"type\":\"set\",\"id\":0,\"settings\":{\"gain\":3}}");
    CHECK_HAS(text_of(h.expect("error"), "message"), "set arrived before the Airspy was opened");
    h.send("{\"type\":\"open\",\"sample_rate\":2500000,\"center\":100000000,\"signal\":\"iq\",\"settings\":{},"
           "\"future_field\":[1]}");
    h.expect("ready");
    h.send(open_line);
    h.send("{\"type\":\"set\",\"id\":1,\"settings\":{\"gain\":\"auto\"},\"extra\":true}");
    CHECK_EQ(h.expect("applied").find("id")->as_number(), 1.0);
    CHECK_EQ(backend.opens(), 1);
    h.close_commands();
    CHECK_EQ(h.exit_status(), 0);
}
