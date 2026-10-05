// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// The settings the module declares (printed by --describe and copied into
// the package manifest) and the validation of open and set against them.
// Validation here needs no hardware; checks that depend on the radio happen
// in receiver.cpp.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "failure.h"
#include "json.h"

namespace fern {

constexpr const char* module_id = "airspy";
constexpr const char* module_name = "Airspy";
constexpr int module_api = 1;
const char* module_version();

struct DeviceSelector {
    enum class Kind { only, serial, index };
    Kind kind = Kind::only;
    uint64_t serial = 0;
    uint32_t index = 0;
};

// The R2's and Mini's gain: the R820T tuner's LNA, mixer and VGA, set
// together along one of libairspy's two ladders of 22 presets, or one by
// one. On an HF+ only automatic and manual exist.
struct GainSetting {
    enum class Mode {
        automatic,  // R2/Mini: the module's own control along the ladder; HF+: its AGC
        tuner,      // R2/Mini: the R820T's own LNA and mixer AGC
        manual,     // R2/Mini: lna_gain, mixer_gain, vga_gain; HF+: AGC off, attenuation
        preset,     // R2/Mini: one preset of the ladder, 0 to 21
    };
    Mode mode = Mode::automatic;
    int preset = 0;

    static GainSetting of_preset(int p) { return GainSetting{Mode::preset, p}; }
    static GainSetting of(Mode m) { return GainSetting{m, 0}; }
    bool operator==(const GainSetting& o) const { return mode == o.mode && (mode != Mode::preset || preset == o.preset); }
    bool operator!=(const GainSetting& o) const { return !(*this == o); }
};

enum class GainLadder { linearity, sensitivity };
const char* ladder_name(GainLadder l);

enum class Packing { automatic, on, off };
const char* packing_name(Packing p);

constexpr int presets = 22;
constexpr int max_lna_gain = 14;
constexpr int max_mixer_gain = 15;
constexpr int max_vga_gain = 15;
constexpr double max_attenuation = 48;

// The defaults are those of Airspy's own airspy_rx for the three stages.
struct ModuleSettings {
    DeviceSelector device;
    GainSetting gain;
    GainLadder gain_mode = GainLadder::linearity;
    int lna_gain = 1;
    int mixer_gain = 5;
    int vga_gain = 5;
    double attenuation = 0;
    bool agc_threshold_high = false;
    bool preamp = false;
    bool bias_tee = false;
    Packing packing = Packing::automatic;
};

struct OpenRequest {
    uint32_t sample_rate = 0;
    uint32_t center = 0;
    ModuleSettings settings;
};

// The settings that may change while samples flow.
struct LiveChange {
    std::optional<GainSetting> gain;
    std::optional<GainLadder> gain_mode;
    std::optional<int> lna_gain;
    std::optional<int> mixer_gain;
    std::optional<int> vga_gain;
    std::optional<double> attenuation;
    std::optional<bool> agc_threshold_high;
    std::optional<bool> preamp;
    std::optional<bool> bias_tee;
};

// Validates an open message. Fails with ErrorCode::invalid.
std::optional<Failure> parse_open(const json::Value& message, OpenRequest& out);

// Validates the settings object of a set message: only live settings may
// appear. Fails with ErrorCode::invalid.
std::optional<Failure> parse_set(const json::Value& settings, LiveChange& out);

// "auto", "tuner", "manual" or the preset as a number.
json::Value gain_json(const GainSetting& g);

// The settings list, identical in --describe and in the package manifest.
json::Value settings_schema();
// The complete --describe object.
json::Value describe_module();

}  // namespace fern
