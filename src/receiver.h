// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Opens the Airspy that the settings select, applies the settings in an
// order the vendor libraries handle correctly, and checks what the radio
// reports before anything is announced to FernSDR.
#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "device.h"
#include "failure.h"
#include "json.h"
#include "settings.h"
#include "stream.h"

namespace fern {

enum class Model { r2, mini, hf_plus, hf_discovery, ranger, hf_other };
const char* model_name(Model m);

// What to tell an operator whose receiver may not open the Airspy: USB
// permissions come from a udev rule that FernSDR's installer writes.
std::string permission_advice();

struct DeviceInfo {
    uint32_t index = 0;
    Family family = Family::one;
    Model model = Model::r2;
    uint64_t serial = 0;
    std::string port;
    std::string firmware;
    std::vector<uint32_t> rates;
    // R2 and Mini: the crystal the firmware tunes with, after a stored
    // correction when the firmware applies one.
    uint32_t xtal = 0;
    bool calibration_stored = false;
    int32_t calibration_ppb = 0;
    bool calibration_applied = false;
    // HF+: the attenuator's steps in dB and the number of bias tees.
    std::vector<float> att_steps;
    int32_t bias_tees = 0;
};

// The model of an R2 or Mini from its firmware's version string.
Model one_model(const std::string& version);
// The model of an HF+ from its part id.
Model hf_model(uint32_t board_id);

// What the hardware was actually set to.
struct Effective {
    uint32_t sample_rate = 0;
    uint32_t center = 0;
    GainSetting gain;
    GainLadder gain_mode = GainLadder::linearity;
    int preset_now = 0;  // R2/Mini with gain = auto or a preset
    int lna_gain = 1;
    int mixer_gain = 5;
    int vga_gain = 5;
    double attenuation = 0;
    bool agc_threshold_high = false;
    bool preamp = false;
    bool bias_tee = false;
    bool packing = false;
    // R2/Mini: how far the R820T's PLL landed from the frequency asked for,
    // Hz, which the module shifts out of the samples.
    double pll_offset = 0;
};

// Where gain = auto starts on an R2 or Mini: the middle of the ladder.
constexpr int auto_start_preset = 10;

class Receiver {
public:
    explicit Receiver(Backend& backend) : backend_(backend) {}
    ~Receiver();
    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;

    // Selects and opens the device, applies every setting and verifies the
    // result. On failure the device is closed again.
    std::optional<Failure> open(const OpenRequest& request);

    // Applies a set. Validation failures change nothing; a failure with
    // ErrorCode::usb may leave the change half done.
    std::optional<Failure> apply(const LiveChange& change);

    bool is_open() const { return source_ != nullptr; }
    const DeviceInfo& info() const { return info_; }
    const Effective& effective() const { return effective_; }

    // The source the stream starts; valid while open.
    Source& source() { return *source_; }
    // "s16" for an R2 or Mini, "f32" for an HF+, and the bytes of one I/Q
    // pair in it.
    const char* format() const;
    size_t sample_bytes() const;

    // R2/Mini with gain = auto: the module moves along the ladder itself.
    bool controls_gain() const;
    // The ladder for the gain control: nominal steps in tenths of a dB and
    // their names.
    std::vector<int> ladder_steps() const;
    std::vector<std::string> ladder_labels() const;
    std::optional<Failure> set_preset(int preset);

    json::Value device_json() const;
    json::Value settings_json() const;
    json::Value settings_json(const LiveChange& change) const;

    // Closes the device. When it still answers, a bias tee the module
    // switched on is switched off first.
    void close(bool device_answers = true);
    // The same, but gives up waiting at the deadline, leaving the close to
    // finish (or hang) in a thread of its own; the process must then leave
    // with _exit(). Returns whether the close finished.
    bool close_by(std::chrono::steady_clock::time_point deadline, bool device_answers = true);
    // Drops the device without closing it, for when another thread may still
    // be inside the vendor library. The process must exit soon after.
    void abandon();

private:
    Backend& backend_;
    std::unique_ptr<OneDevice> one_;
    std::unique_ptr<HfDevice> hf_;
    std::unique_ptr<Source> source_;
    DeviceInfo info_;
    Effective effective_;
    bool bias_on_by_module_ = false;

    std::optional<Failure> select(const DeviceSelector& selector, Found& found, uint32_t& index);
    std::optional<Failure> open_one(const OpenRequest& request, const Found& found);
    std::optional<Failure> open_hf(const OpenRequest& request, const Found& found);
    std::optional<Failure> check_family(const ModuleSettings& s) const;
    std::optional<Failure> check_family(const LiveChange& c) const;
    std::optional<Failure> set_one_gain(const GainSetting& gain, GainLadder ladder, int lna, int mixer, int vga);
    std::optional<Failure> set_hf_gain(const GainSetting& gain, bool threshold_high, double attenuation);
    std::optional<Failure> set_bias(bool on);
    std::string which() const;
};

}  // namespace fern
