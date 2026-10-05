// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// A fake backend with fake Airspys of both families. They follow the vendor
// libraries' return conventions and stream the way the real radios do: an
// R2 or Mini sends real 12-bit samples of a tone, received through an R820T
// whose PLL lands where the firmware puts it, an HF+ float I/Q of a tone.
// The tests then check what reaches fd 1 against where the tone really is.
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "device.h"

namespace fake {

struct Spec {
    fern::Family family = fern::Family::one;
    std::string port = "1-2";
    uint64_t serial = 0x0000000012345678ull;
    bool has_serial = true;
    int open_error = 0;    // what the USB scan finds opening or claiming it
    int vendor_open = 0;   // what Backend::open_one/open_hf returns

    // R2/Mini.
    std::string version = "AirSpy NOS v1.0.0-rc10-6-g4008185 2020-05-08";
    std::vector<uint32_t> one_rates = {10000000, 2500000};
    bool calibration = false;
    int32_t calibration_ppb = 0;
    bool clock_lost = true;  // no external 10 MHz reference

    // HF+.
    uint32_t board_id = 2;
    std::string hf_version = "R3.0.7-CD";
    std::vector<uint32_t> hf_rates = {912000, 768000, 456000, 384000, 256000, 192000};
    std::vector<float> att_steps = {0, 6, 12, 18, 24, 30, 36, 42, 48};
    int32_t bias_tees = 0;

    // The tone, this far above the centre asked for, Hz.
    double tone_offset = 100000;
    // R2/Mini: the tone's amplitude in ADC steps at linearity 10; with
    // follow_gain it changes 3 dB a preset, clipped at the converter's
    // limits as a real one clips.
    double amplitude = 800;
    bool follow_gain = false;

    uint64_t unplug_after = 0;  // samples sent before the device vanishes; 0 never
    uint64_t stall_after = 0;   // samples sent before the device goes quiet; 0 never
    bool fail_start = false;    // start() fails
    std::string fail_call;      // this call returns an error, e.g. "set_freq"
    bool realtime = true;       // pace the samples at the sample rate
    unsigned open_delay_ms = 0;  // how long the library's open takes
    unsigned stop_delay_ms = 0;  // how long stop() takes
    unsigned close_delay_ms = 0;
};

// What a fake device was told, for the tests to inspect.
struct State {
    std::mutex mutex;
    bool closed = false;
    uint32_t rate_index = 0;
    uint32_t rate = 0;
    bool packing = false;
    uint32_t freq = 0;
    double hf_freq = 0;
    std::string gain;  // "linearity 10", "sensitivity 3", "manual 1/5/5", "agc vga 5"
    bool lna_agc = false;
    bool mixer_agc = false;
    int lna = 0;
    int mixer = 0;
    int vga = 0;
    bool hf_agc = true;
    bool hf_agc_high = false;
    float att = 0;
    bool hf_lna = false;
    bool bias_tee = false;
    int starts = 0;
    int stops = 0;
    std::vector<std::string> calls;
};

// Where the fake R820T's LO lands: the nearest step of its PLL, by plain
// rounding of the exact ratio, not by the firmware's bit by bit division
// that the module mirrors. The two agree but for rare ties; the tests use
// frequencies where they do.
double landed_lo(uint32_t xtal, uint32_t lo);

class Backend : public fern::Backend {
public:
    Backend() = default;
    explicit Backend(std::vector<Spec> specs) : devices(std::move(specs)) {}
    ~Backend() override;

    std::vector<Spec> devices;
    int enumeration_error = 0;
    std::string enumeration_diagnostic;

    fern::Enumeration enumerate() override;
    int open_one(uint64_t serial, std::unique_ptr<fern::OneDevice>& out) override;
    int open_hf(uint64_t serial, std::unique_ptr<fern::HfDevice>& out) override;

    // State of the device opened last; null when none was opened.
    std::shared_ptr<State> last();
    int opens() const;

private:
    friend class OneDevice;
    friend class HfDevice;
    mutable std::mutex mutex_;
    std::vector<std::shared_ptr<State>> states_;
    // Devices the module abandoned are deleted when the backend goes, so
    // that leak checks stay meaningful.
    std::set<fern::OneDevice*> live_one_;
    std::set<fern::HfDevice*> live_hf_;
};

}  // namespace fake
