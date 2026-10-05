// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include <atomic>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "stream.h"
#include "test.h"

using Clock = std::chrono::steady_clock;

TEST(ring_rounds_capacity_to_a_power_of_two) {
    CHECK_EQ(fern::RingBuffer(1000).capacity(), size_t(1024));
    CHECK_EQ(fern::RingBuffer(4096).capacity(), size_t(4096));
    CHECK_EQ(fern::RingBuffer(0).capacity(), size_t(2));
}

TEST(ring_wraps_around) {
    fern::RingBuffer ring(16);
    std::vector<uint8_t> data(12);
    for (size_t i = 0; i < data.size(); ++i)
        data[i] = static_cast<uint8_t>(i + 1);
    REQUIRE(ring.push(data.data(), 12));
    CHECK_EQ(ring.size(), size_t(12));
    const uint8_t* p = nullptr;
    REQUIRE(ring.peek(&p) == 12);
    CHECK_EQ(p[0], uint8_t(1));
    ring.consume(10);
    CHECK_EQ(ring.size(), size_t(2));

    // 12 more bytes: 4 fit before the end, 8 wrap to the start.
    REQUIRE(ring.push(data.data(), 12));
    CHECK_EQ(ring.size(), size_t(14));
    std::vector<uint8_t> out;
    while (ring.size() > 0) {
        const size_t n = ring.peek(&p);
        REQUIRE(n > 0);
        out.insert(out.end(), p, p + n);
        ring.consume(n);
    }
    REQUIRE(out.size() == 14);
    CHECK_EQ(out[0], uint8_t(11));
    CHECK_EQ(out[1], uint8_t(12));
    for (size_t i = 0; i < 12; ++i)
        CHECK_EQ(out[2 + i], uint8_t(i + 1));
    CHECK_EQ(ring.peek(&p), size_t(0));
}

TEST(ring_drops_whole_blocks_when_full) {
    fern::RingBuffer ring(16);
    const uint8_t block[6] = {1, 2, 3, 4, 5, 6};
    CHECK(ring.push(block, 6));
    CHECK(ring.push(block, 6));
    CHECK(!ring.push(block, 6));  // 12 used, 4 free: nothing is stored
    CHECK_EQ(ring.size(), size_t(12));
    CHECK(ring.push(block, 4));
    CHECK_EQ(ring.size(), size_t(16));
    CHECK(!ring.push(block, 1));
    ring.consume(6);
    CHECK(ring.push(block, 6));
    CHECK(!ring.push(block, 17));
}

// One producer pushes numbered blocks as fast as it can while one consumer
// reads slowly. Every byte read must belong to a block that was pushed
// whole, in order, and pushed plus dropped must account for every block.
// The consumer starts only once the producer has tried four times what the
// ring holds, so some blocks are always dropped: left to the scheduler, a
// fast machine's consumer sometimes kept up and nothing was.
TEST(ring_keeps_order_under_concurrency) {
    fern::RingBuffer ring(1 << 12);
    constexpr uint32_t blocks = 20000;
    constexpr size_t block_size = 256;
    std::atomic<bool> done{false};
    std::atomic<uint32_t> tried{0};
    uint32_t pushed = 0;
    uint32_t dropped = 0;

    std::thread producer([&] {
        std::vector<uint8_t> block(block_size);
        for (uint32_t n = 0; n < blocks; ++n, tried.store(n)) {
            std::memcpy(block.data(), &n, sizeof n);
            for (size_t i = sizeof n; i < block_size; ++i)
                block[i] = static_cast<uint8_t>(n + i);
            if (ring.push(block.data(), block.size()))
                ++pushed;
            else
                ++dropped;
        }
        done.store(true);
    });

    while (tried.load() < 4 * (1 << 12) / block_size && !done.load()) std::this_thread::yield();
    std::vector<uint8_t> partial;
    uint32_t received = 0;
    int64_t last = -1;
    bool in_order = true;
    bool intact = true;
    for (;;) {
        const uint8_t* p = nullptr;
        const size_t n = ring.peek(&p);
        if (n == 0) {
            if (done.load() && ring.size() == 0)
                break;
            std::this_thread::yield();
            continue;
        }
        const size_t take = n < 100 ? n : 100;
        partial.insert(partial.end(), p, p + take);
        ring.consume(take);
        while (partial.size() >= block_size) {
            uint32_t id;
            std::memcpy(&id, partial.data(), sizeof id);
            for (size_t i = sizeof id; i < block_size; ++i)
                if (partial[i] != static_cast<uint8_t>(id + i))
                    intact = false;
            if (static_cast<int64_t>(id) <= last)
                in_order = false;
            last = id;
            ++received;
            partial.erase(partial.begin(), partial.begin() + block_size);
        }
    }
    producer.join();
    CHECK(intact);
    CHECK(in_order);
    CHECK(partial.empty());
    CHECK_EQ(received, pushed);
    CHECK_EQ(pushed + dropped, blocks);
    CHECK(dropped > 0);  // the consumer is slower on purpose
}

namespace {

// A source that hands over numbered bytes in blocks from a thread of its
// own, as a vendor library does, until stopped or until it fails.
class ScriptedSource : public fern::Source {
public:
    size_t block = 4096;
    size_t blocks_before_failing = 0;  // 0: never fails
    int start_error = 0;
    unsigned stop_delay_ms = 0;
    uint64_t lost_per_block = 0;
    std::atomic<int> stops{0};

    ~ScriptedSource() override { halt(); }

    int start(fern::Stream& stream) override {
        if (start_error != 0)
            return start_error;
        running_.store(true);
        thread_ = std::thread([this, &stream] {
            std::vector<uint8_t> data(block);
            uint64_t sent = 0;
            for (size_t n = 0; running_.load(); ++n) {
                if (blocks_before_failing && n >= blocks_before_failing) {
                    failed_.store(true);
                    return;
                }
                for (size_t i = 0; i < block; ++i)
                    data[i] = static_cast<uint8_t>((sent + i) % 251);
                sent += block;
                stream.deliver(data.data(), block, lost_per_block);
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
        });
        return 0;
    }
    int stop() override {
        ++stops;
        if (stop_delay_ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(stop_delay_ms));
        halt();
        return 0;
    }
    bool streaming() override { return running_.load() && !failed_.load(); }

private:
    void halt() {
        running_.store(false);
        if (thread_.joinable())
            thread_.join();
    }
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> failed_{false};
};

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
        if (r >= 0)
            ::close(r);
        if (w >= 0)
            ::close(w);
    }
};

std::vector<uint8_t> read_bytes(int fd, size_t n, int timeout_ms = 3000) {
    std::vector<uint8_t> out;
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    while (out.size() < n && Clock::now() < deadline) {
        struct pollfd pfd = {fd, POLLIN, 0};
        if (::poll(&pfd, 1, 20) <= 0)
            continue;
        std::vector<uint8_t> buf(n - out.size());
        const ssize_t got = ::read(fd, buf.data(), buf.size());
        if (got <= 0)
            break;
        out.insert(out.end(), buf.begin(), buf.begin() + got);
    }
    return out;
}

}  // namespace

TEST(stream_delivers_every_byte_in_order) {
    ScriptedSource source;
    Pipe pipe;
    const int notify = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    {
        fern::Stream stream(source, pipe.w, notify, 1 << 20, 4);
        REQUIRE(stream.start() == 0);
        const std::vector<uint8_t> got = read_bytes(pipe.r, 200000);
        REQUIRE(got.size() == 200000);
        bool exact = true;
        for (size_t i = 0; i < got.size(); ++i)
            exact = exact && got[i] == i % 251;
        CHECK(exact);
        CHECK(!stream.source_failed());
        stream.request_stop();
        CHECK(stream.wait(Clock::now() + std::chrono::seconds(2)));
        CHECK_EQ(source.stops.load(), 1);
        CHECK(stream.samples_delivered() * 4 >= 200000);
        // What is still in the ring at the stop is neither.
        CHECK(stream.samples_received() >= stream.samples_delivered() + stream.samples_dropped());
    }
    ::close(notify);
}

TEST(stream_counts_what_the_ring_drops_and_the_library_lost) {
    ScriptedSource source;
    source.lost_per_block = 3;
    Pipe pipe;
    fern::Stream stream(source, pipe.w, -1, 1 << 14, 4);
    REQUIRE(stream.start() == 0);
    // Nobody reads: the pipe and then the ring fill, and blocks are dropped
    // whole.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    stream.request_stop();
    // The writer is stuck in write() on a full pipe until nudged.
    CHECK(stream.wait(Clock::now() + std::chrono::seconds(2)));
    // The ring's drops, and on top three samples for every block the
    // library said it lost.
    const uint64_t blocks = stream.samples_received() * 4 / 4096;
    CHECK(blocks > 0);
    CHECK(stream.samples_dropped() > 3 * blocks);
    CHECK(stream.samples_delivered() < stream.samples_received());
}

TEST(stream_reports_a_source_that_fails) {
    ScriptedSource source;
    source.blocks_before_failing = 5;
    Pipe pipe;
    fern::Stream stream(source, pipe.w, -1, 1 << 20, 4);
    REQUIRE(stream.start() == 0);
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    while (!stream.source_failed() && Clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    CHECK(stream.source_failed());
    CHECK_EQ(stream.samples_received(), uint64_t(5 * 4096 / 4));
    stream.request_stop();
    // Once a stop is asked for, a source that ended is no failure.
    CHECK(!stream.source_failed());
    CHECK(stream.wait(Clock::now() + std::chrono::seconds(2)));
}

TEST(stream_start_failure_leaves_nothing_running) {
    ScriptedSource source;
    source.start_error = -1000;
    Pipe pipe;
    fern::Stream stream(source, pipe.w, -1, 1 << 16, 4);
    CHECK_EQ(stream.start(), -1000);
    // The library may have started a thread before failing: stopped.
    CHECK_EQ(source.stops.load(), 1);
    CHECK(stream.wait(Clock::now() + std::chrono::milliseconds(100)));
}

TEST(stream_gives_up_waiting_for_a_slow_stop) {
    ScriptedSource source;
    source.stop_delay_ms = 400;
    Pipe pipe;
    fern::Stream stream(source, pipe.w, -1, 1 << 20, 4);
    REQUIRE(stream.start() == 0);
    read_bytes(pipe.r, 10000);
    stream.request_stop();
    CHECK(!stream.wait(Clock::now() + std::chrono::milliseconds(100)));
    // The stop goes on in its own thread and finishes.
    CHECK(stream.wait(Clock::now() + std::chrono::seconds(2)));
}

TEST(stream_ends_when_fd_1_is_closed) {
    ScriptedSource source;
    Pipe pipe;
    const int notify = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    fern::Stream stream(source, pipe.w, notify, 1 << 20, 4);
    REQUIRE(stream.start() == 0);
    read_bytes(pipe.r, 4096);
    ::close(pipe.r);
    pipe.r = -1;
    struct pollfd pfd = {notify, POLLIN, 0};
    CHECK(::poll(&pfd, 1, 2000) == 1);
    CHECK(stream.writer_end() == fern::Stream::WriterEnd::host_gone);
    stream.request_stop();
    CHECK(stream.wait(Clock::now() + std::chrono::seconds(2)));
    ::close(notify);
}

TEST(stream_notes_levels) {
    ScriptedSource source;
    Pipe pipe;
    fern::Stream stream(source, pipe.w, -1, 1 << 16, 4);
    stream.note_levels(1000, 3, 700);
    stream.note_levels(1000, 0, 2048);
    stream.note_levels(1000, 1, 5);
    CHECK_EQ(stream.real_samples(), uint64_t(3000));
    CHECK_EQ(stream.samples_clipped(), uint64_t(4));
    CHECK_EQ(stream.take_peak(), 2048u);
    CHECK_EQ(stream.take_peak(), 0u);
}
