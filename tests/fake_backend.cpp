// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include "fake_backend.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

namespace fake {

namespace {

constexpr double two_pi = 6.283185307179586476925286766559;
constexpr int one_error = -1000;  // AIRSPY_ERROR_LIBUSB
constexpr int hf_error = -1;      // AIRSPYHF_ERROR

uint32_t xtal_of(const Spec& spec) {
    const uint32_t nominal = spec.version.find("MINI") != std::string::npos ? 24000000 : 25000000;
    const bool mini = nominal == 24000000;
    if (!spec.calibration || (!mini && !spec.clock_lost))
        return nominal;
    // The firmware's own rounding, sys_calib_r820t() in airspyone_firmware.
    const int64_t billion = 1000000000;
    return static_cast<uint32_t>(nominal + (static_cast<int64_t>(nominal) * spec.calibration_ppb + billion / 2) / billion);
}

using Clock = std::chrono::steady_clock;

// Paces a producer at `rate` items a second.
void pace(Clock::time_point start, uint64_t sent, double rate) {
    const auto due = start + std::chrono::nanoseconds(static_cast<int64_t>(static_cast<double>(sent) / rate * 1e9));
    std::this_thread::sleep_until(due);
}

}  // namespace

double landed_lo(uint32_t xtal, uint32_t lo) {
    for (unsigned shift = 0; shift <= 5; ++shift) {
        const double vco = static_cast<double>(lo) * (2u << shift);
        if (vco >= 1770000000.0 && vco <= 3900000000.0) {
            const double steps = std::llround(vco * 65536.0 / xtal);
            return xtal * steps / 65536.0 / (2u << shift);
        }
    }
    return 0;
}

class OneDevice : public fern::OneDevice {
public:
    OneDevice(Backend& owner, const Spec& spec, std::shared_ptr<State> state)
        : owner_(owner), spec_(spec), state_(std::move(state)) {
        std::lock_guard<std::mutex> lock(owner_.mutex_);
        owner_.live_one_.insert(this);
    }

    ~OneDevice() override {
        {
            std::lock_guard<std::mutex> lock(owner_.mutex_);
            owner_.live_one_.erase(this);
        }
        halt();
        // A slow close may outlive the backend, so only state_ is used from
        // here on.
        if (spec_.close_delay_ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(spec_.close_delay_ms));
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->closed = true;
        state_->calls.push_back("close");
    }

    int version(std::string& out) override {
        if (failing("version"))
            return one_error;
        out = spec_.version;
        return 0;
    }
    int samplerates(std::vector<uint32_t>& out) override {
        out = spec_.one_rates;
        return 0;
    }
    int read_calibration(bool& valid, int32_t& ppb) override {
        if (spec_.fail_call == "read_calibration")
            return one_error;
        valid = spec_.calibration;
        ppb = spec_.calibration_ppb;
        return 0;
    }
    int clock_input_lost(bool& lost) override {
        lost = spec_.clock_lost;
        return 0;
    }
    int set_samplerate_index(uint32_t index) override {
        if (failing("set_samplerate") || index >= spec_.one_rates.size())
            return one_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->rate_index = index;
        state_->rate = spec_.one_rates[index];
        state_->calls.push_back("set_samplerate " + std::to_string(index));
        return 0;
    }
    int set_packing(bool on) override {
        if (failing("set_packing"))
            return one_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->packing = on;
        state_->calls.push_back(std::string("set_packing ") + (on ? "1" : "0"));
        return 0;
    }
    int set_freq(uint32_t hz) override {
        if (failing("set_freq"))
            return one_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->freq = hz;
        state_->calls.push_back("set_freq " + std::to_string(hz));
        return 0;
    }
    int set_linearity_gain(uint8_t value) override { return preset("linearity", value); }
    int set_sensitivity_gain(uint8_t value) override { return preset("sensitivity", value); }
    int set_lna_gain(uint8_t value) override { return stage("lna", value, &State::lna); }
    int set_mixer_gain(uint8_t value) override { return stage("mixer", value, &State::mixer); }
    int set_vga_gain(uint8_t value) override { return stage("vga", value, &State::vga); }
    int set_lna_agc(bool on) override {
        if (failing("set_lna_agc"))
            return one_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->lna_agc = on;
        state_->calls.push_back(std::string("set_lna_agc ") + (on ? "1" : "0"));
        return 0;
    }
    int set_mixer_agc(bool on) override {
        if (failing("set_mixer_agc"))
            return one_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->mixer_agc = on;
        state_->calls.push_back(std::string("set_mixer_agc ") + (on ? "1" : "0"));
        return 0;
    }
    int set_bias_tee(bool on) override {
        if (failing("set_bias_tee") || (on && failing("set_bias_tee on")) || gone_.load())
            return one_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->bias_tee = on;
        state_->calls.push_back(std::string("set_bias_tee ") + (on ? "1" : "0"));
        return 0;
    }

    int start(fern::RealCallback cb, void* ctx) override {
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            ++state_->starts;
            state_->calls.push_back("start");
        }
        if (spec_.fail_start)
            return one_error;
        cb_ = cb;
        ctx_ = ctx;
        stop_.store(false);
        streaming_.store(true);
        thread_ = std::thread([this] { produce(); });
        return 0;
    }

    int stop() override {
        halt();
        std::lock_guard<std::mutex> lock(state_->mutex);
        ++state_->stops;
        state_->calls.push_back("stop");
        return gone_.load() ? one_error : 0;
    }

    bool streaming() override { return streaming_.load(); }

private:
    bool failing(const std::string& call) const { return spec_.fail_call == call; }

    int preset(const char* ladder, uint8_t value) {
        if (failing(std::string("set_") + ladder + "_gain") || gone_.load())
            return one_error;
        const int v = std::min<int>(value, 21);
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->gain = std::string(ladder) + " " + std::to_string(v);
        state_->lna_agc = false;
        state_->mixer_agc = false;
        state_->calls.push_back("set_" + std::string(ladder) + "_gain " + std::to_string(v));
        gain_db_.store(3.0 * (v - 10));
        return 0;
    }

    int stage(const char* name, uint8_t value, int State::*field) {
        if (failing(std::string("set_") + name + "_gain"))
            return one_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        (*state_).*field = value;
        state_->gain = "manual " + std::to_string(state_->lna) + "/" + std::to_string(state_->mixer) + "/" +
                       std::to_string(state_->vga);
        state_->calls.push_back("set_" + std::string(name) + "_gain " + std::to_string(value));
        return 0;
    }

    void halt() {
        stop_.store(true);
        if (thread_.joinable()) {
            if (spec_.stop_delay_ms)
                std::this_thread::sleep_for(std::chrono::milliseconds(spec_.stop_delay_ms));
            thread_.join();
        }
        streaming_.store(false);
    }

    // What the library's consumer thread does: hand over blocks of real
    // samples until stopped, or until the device goes.
    void produce() {
        uint32_t rate;
        uint32_t freq;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            rate = state_->rate;
            freq = state_->freq;
        }
        const double fs = 2.0 * rate;
        const uint32_t lo_request = freq + rate / 2;
        const double lo = landed_lo(xtal_of(spec_), lo_request);
        // High-side injection: the IF is the LO less the signal.
        const double f_if = lo - (static_cast<double>(freq) + spec_.tone_offset);
        const double step = two_pi * f_if / fs;
        constexpr size_t block = 16384;
        std::vector<uint16_t> buf(block);
        uint64_t n = 0;
        const Clock::time_point start = Clock::now();
        while (!stop_.load()) {
            if (spec_.unplug_after && n / 2 >= spec_.unplug_after) {
                gone_.store(true);
                streaming_.store(false);
                return;
            }
            if (spec_.stall_after && n / 2 >= spec_.stall_after) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            const double amplitude = spec_.amplitude * (spec_.follow_gain ? std::pow(10.0, gain_db_.load() / 20) : 1.0);
            // A phasor stepped sample by sample, set exactly at each block.
            const double phase = std::fmod(step * static_cast<double>(n), two_pi);
            double c = std::cos(phase);
            double s = std::sin(phase);
            const double dc = std::cos(step);
            const double ds = std::sin(step);
            for (size_t i = 0; i < block; ++i) {
                const double v = 2048 + amplitude * c;
                buf[i] = static_cast<uint16_t>(std::clamp<long>(std::lround(v), 0, 4095));
                const double next = c * dc - s * ds;
                s = s * dc + c * ds;
                c = next;
            }
            n += block;
            cb_(buf.data(), block, 0, ctx_);
            if (spec_.realtime)
                pace(start, n, fs);
        }
    }

    Backend& owner_;
    const Spec spec_;
    std::shared_ptr<State> state_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> gone_{false};
    std::atomic<double> gain_db_{0};
    fern::RealCallback cb_ = nullptr;
    void* ctx_ = nullptr;
};

class HfDevice : public fern::HfDevice {
public:
    HfDevice(Backend& owner, const Spec& spec, std::shared_ptr<State> state)
        : owner_(owner), spec_(spec), state_(std::move(state)) {
        std::lock_guard<std::mutex> lock(owner_.mutex_);
        owner_.live_hf_.insert(this);
    }

    ~HfDevice() override {
        {
            std::lock_guard<std::mutex> lock(owner_.mutex_);
            owner_.live_hf_.erase(this);
        }
        halt();
        if (spec_.close_delay_ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(spec_.close_delay_ms));
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->closed = true;
        state_->calls.push_back("close");
    }

    int version(std::string& out) override {
        out = spec_.hf_version;
        return 0;
    }
    int board_id(uint32_t& out) override {
        out = spec_.board_id;
        return 0;
    }
    int samplerates(std::vector<uint32_t>& out) override {
        out = spec_.hf_rates;
        return 0;
    }
    int att_steps(std::vector<float>& out) override {
        out = spec_.att_steps;
        return 0;
    }
    int bias_tee_count(int32_t& out) override {
        out = spec_.bias_tees;
        return 0;
    }
    int set_samplerate(uint32_t hz) override {
        if (failing("set_samplerate") ||
            std::find(spec_.hf_rates.begin(), spec_.hf_rates.end(), hz) == spec_.hf_rates.end())
            return hf_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->rate = hz;
        state_->calls.push_back("set_samplerate " + std::to_string(hz));
        return 0;
    }
    int set_freq(double hz) override {
        if (failing("set_freq"))
            return hf_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->hf_freq = hz;
        state_->calls.push_back("set_freq " + std::to_string(static_cast<long long>(hz)));
        return 0;
    }
    int set_hf_agc(bool on) override {
        if (failing("set_hf_agc"))
            return hf_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->hf_agc = on;
        state_->calls.push_back(std::string("set_hf_agc ") + (on ? "1" : "0"));
        return 0;
    }
    int set_hf_agc_threshold(bool high) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->hf_agc_high = high;
        state_->calls.push_back(std::string("set_hf_agc_threshold ") + (high ? "1" : "0"));
        return 0;
    }
    int set_att(float db) override {
        if (failing("set_att"))
            return hf_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->att = db;
        state_->calls.push_back("set_att " + std::to_string(static_cast<int>(db)));
        return 0;
    }
    int set_hf_lna(bool on) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->hf_lna = on;
        state_->calls.push_back(std::string("set_hf_lna ") + (on ? "1" : "0"));
        return 0;
    }
    int set_bias_tee(bool on) override {
        // Firmware without a bias tee does not know the request.
        if (spec_.bias_tees <= 0 || gone_.load())
            return hf_error;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->bias_tee = on;
        state_->calls.push_back(std::string("set_bias_tee ") + (on ? "1" : "0"));
        return 0;
    }

    int start(fern::IqCallback cb, void* ctx) override {
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            ++state_->starts;
            state_->calls.push_back("start");
        }
        if (spec_.fail_start)
            return hf_error;
        cb_ = cb;
        ctx_ = ctx;
        stop_.store(false);
        streaming_.store(true);
        thread_ = std::thread([this] { produce(); });
        return 0;
    }

    int stop() override {
        halt();
        std::lock_guard<std::mutex> lock(state_->mutex);
        ++state_->stops;
        state_->calls.push_back("stop");
        return 0;
    }

    bool streaming() override { return streaming_.load(); }

private:
    bool failing(const std::string& call) const { return spec_.fail_call == call; }

    void halt() {
        stop_.store(true);
        if (thread_.joinable()) {
            if (spec_.stop_delay_ms)
                std::this_thread::sleep_for(std::chrono::milliseconds(spec_.stop_delay_ms));
            thread_.join();
        }
        streaming_.store(false);
    }

    void produce() {
        uint32_t rate;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            rate = state_->rate;
        }
        const double step = two_pi * spec_.tone_offset / rate;
        constexpr size_t block = 4096;
        std::vector<float> buf(2 * block);
        uint64_t n = 0;
        const Clock::time_point start = Clock::now();
        while (!stop_.load()) {
            if (spec_.unplug_after && n >= spec_.unplug_after) {
                gone_.store(true);
                streaming_.store(false);
                return;
            }
            if (spec_.stall_after && n >= spec_.stall_after) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            const double phase = std::fmod(step * static_cast<double>(n), two_pi);
            double c = std::cos(phase);
            double s = std::sin(phase);
            const double dc = std::cos(step);
            const double ds = std::sin(step);
            for (size_t i = 0; i < block; ++i) {
                buf[2 * i] = static_cast<float>(0.5 * c);
                buf[2 * i + 1] = static_cast<float>(0.5 * s);
                const double next = c * dc - s * ds;
                s = s * dc + c * ds;
                c = next;
            }
            n += block;
            cb_(buf.data(), block, 0, ctx_);
            if (spec_.realtime)
                pace(start, n, rate);
        }
    }

    Backend& owner_;
    const Spec spec_;
    std::shared_ptr<State> state_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> gone_{false};
    fern::IqCallback cb_ = nullptr;
    void* ctx_ = nullptr;
};

Backend::~Backend() {
    std::vector<fern::OneDevice*> ones;
    std::vector<fern::HfDevice*> hfs;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ones.assign(live_one_.begin(), live_one_.end());
        hfs.assign(live_hf_.begin(), live_hf_.end());
    }
    for (fern::OneDevice* d : ones)
        delete d;
    for (fern::HfDevice* d : hfs)
        delete d;
}

fern::Enumeration Backend::enumerate() {
    fern::Enumeration e;
    e.error = enumeration_error;
    e.diagnostic = enumeration_diagnostic;
    if (e.error != 0)
        return e;
    for (const Spec& s : devices) {
        fern::Found f;
        f.family = s.family;
        f.port = s.port;
        f.open_error = s.open_error;
        // A device that cannot be opened cannot tell its serial.
        f.has_serial = s.has_serial && s.open_error != fern::usb_error::access;
        f.serial = f.has_serial ? s.serial : 0;
        e.devices.push_back(f);
    }
    return e;
}

int Backend::open_one(uint64_t serial, std::unique_ptr<fern::OneDevice>& out) {
    for (const Spec& s : devices) {
        if (s.family != fern::Family::one || s.serial != serial)
            continue;
        if (s.open_delay_ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(s.open_delay_ms));
        if (s.vendor_open != 0)
            return s.vendor_open;
        auto state = std::make_shared<State>();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            states_.push_back(state);
        }
        out = std::make_unique<OneDevice>(*this, s, state);
        return 0;
    }
    return -5;  // AIRSPY_ERROR_NOT_FOUND
}

int Backend::open_hf(uint64_t serial, std::unique_ptr<fern::HfDevice>& out) {
    for (const Spec& s : devices) {
        if (s.family != fern::Family::hf || s.serial != serial)
            continue;
        if (s.open_delay_ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(s.open_delay_ms));
        if (s.vendor_open != 0)
            return s.vendor_open;
        auto state = std::make_shared<State>();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            states_.push_back(state);
        }
        out = std::make_unique<HfDevice>(*this, s, state);
        return 0;
    }
    return hf_error;
}

std::shared_ptr<State> Backend::last() {
    std::lock_guard<std::mutex> lock(mutex_);
    return states_.empty() ? nullptr : states_.back();
}

int Backend::opens() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(states_.size());
}

}  // namespace fake
