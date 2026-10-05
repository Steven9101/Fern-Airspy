// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include "settings.h"

#include <cctype>
#include <cmath>
#include <string_view>
#include <vector>

#ifndef FERN_AIRSPY_VERSION
#error "FERN_AIRSPY_VERSION must be defined by the build"
#endif

namespace fern {

const char* module_version() { return FERN_AIRSPY_VERSION; }

const char* ladder_name(GainLadder l) { return l == GainLadder::sensitivity ? "sensitivity" : "linearity"; }

const char* packing_name(Packing p) {
    switch (p) {
    case Packing::on: return "on";
    case Packing::off: return "off";
    case Packing::automatic: break;
    }
    return "auto";
}

json::Value gain_json(const GainSetting& g) {
    switch (g.mode) {
    case GainSetting::Mode::tuner: return json::Value("tuner");
    case GainSetting::Mode::manual: return json::Value("manual");
    case GainSetting::Mode::preset: return json::Value(g.preset);
    case GainSetting::Mode::automatic: break;
    }
    return json::Value("auto");
}

namespace {

enum class Key {
    device,
    gain,
    gain_mode,
    lna_gain,
    mixer_gain,
    vga_gain,
    attenuation,
    agc_threshold,
    preamp,
    bias_tee,
    packing,
};

struct KeyInfo {
    Key key;
    const char* name;
    bool live;
};

// The order here is the order of --describe.
constexpr KeyInfo all_keys[] = {
    {Key::device, "device", false},
    {Key::gain, "gain", true},
    {Key::gain_mode, "gain_mode", true},
    {Key::lna_gain, "lna_gain", true},
    {Key::mixer_gain, "mixer_gain", true},
    {Key::vga_gain, "vga_gain", true},
    {Key::attenuation, "attenuation", true},
    {Key::agc_threshold, "agc_threshold", true},
    {Key::preamp, "preamp", true},
    {Key::bias_tee, "bias_tee", true},
    {Key::packing, "packing", false},
};

const KeyInfo* find_key(std::string_view name) {
    for (const KeyInfo& k : all_keys)
        if (name == k.name)
            return &k;
    return nullptr;
}

std::string known_keys() {
    std::string s;
    const size_t n = sizeof all_keys / sizeof all_keys[0];
    for (size_t i = 0; i < n; ++i) {
        if (i > 0)
            s += i + 1 == n ? " and " : ", ";
        s += all_keys[i].name;
    }
    return s;
}

// A value as the operator would recognise it in a message, kept short.
std::string shown(const json::Value& v) {
    std::string s = json::serialize(v);
    if (s.size() > 60)
        s = s.substr(0, 57) + "...";
    return s;
}

std::optional<Failure> invalid(std::string message) { return Failure{ErrorCode::invalid, std::move(message)}; }

bool whole_in_range(const json::Value& v, double lo, double hi, double& out) {
    if (!v.is_number() || !json::is_whole(v.as_number()))
        return false;
    const double d = v.as_number();
    if (d < lo || d > hi)
        return false;
    out = d;
    return true;
}

bool same_word(const std::string& s, const char* word) {
    size_t i = 0;
    for (; word[i] != '\0'; ++i)
        if (i >= s.size() || std::tolower(static_cast<unsigned char>(s[i])) != word[i])
            return false;
    return i == s.size();
}

bool starts_without_case(std::string_view text, std::string_view prefix) {
    if (text.size() < prefix.size())
        return false;
    for (size_t i = 0; i < prefix.size(); ++i)
        if (std::tolower(static_cast<unsigned char>(text[i])) != std::tolower(static_cast<unsigned char>(prefix[i])))
            return false;
    return true;
}

int hex_digit(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

std::optional<Failure> parse_device(const json::Value& v, DeviceSelector& out) {
    const std::string wrong =
        "module.device must be serial:<serial> (up to 16 hexadecimal digits, as fern-airspy --list-devices prints "
        "them), index:<n> or empty, not ";
    if (!v.is_string())
        return invalid(wrong + shown(v));
    const std::string& s = v.as_string();
    DeviceSelector sel;
    if (s.empty()) {
        sel.kind = DeviceSelector::Kind::only;
    } else if (s.rfind("serial:", 0) == 0) {
        std::string_view digits(s.data() + 7, s.size() - 7);
        // FernSDR's own USB scan reads the serial string descriptor as it
        // stands, "AIRSPY SN:" or "AIRSPYHF SN:" and the digits, and its
        // setup writes that.
        for (const std::string_view prefix : {std::string_view("AIRSPY SN:"), std::string_view("AIRSPYHF SN:")}) {
            if (starts_without_case(digits, prefix)) {
                digits.remove_prefix(prefix.size());
                break;
            }
        }
        if (digits.size() > 2 && digits[0] == '0' && (digits[1] == 'x' || digits[1] == 'X'))
            digits.remove_prefix(2);
        if (digits.empty() || digits.size() > 16)
            return invalid(wrong + shown(v));
        uint64_t n = 0;
        for (char c : digits) {
            const int d = hex_digit(c);
            if (d < 0)
                return invalid(wrong + shown(v));
            n = n << 4 | static_cast<uint64_t>(d);
        }
        sel.kind = DeviceSelector::Kind::serial;
        sel.serial = n;
    } else if (s.rfind("index:", 0) == 0 && s.size() > 6 && s.size() <= 16) {
        uint64_t n = 0;
        for (size_t i = 6; i < s.size(); ++i) {
            if (s[i] < '0' || s[i] > '9')
                return invalid(wrong + shown(v));
            n = n * 10 + static_cast<uint64_t>(s[i] - '0');
        }
        if (n > UINT32_MAX)
            return invalid(wrong + shown(v));
        sel.kind = DeviceSelector::Kind::index;
        sel.index = static_cast<uint32_t>(n);
    } else {
        return invalid(wrong + shown(v));
    }
    out = sel;
    return std::nullopt;
}

// The schema declares gain as a string; a JSON number is accepted as well,
// because a set may carry one.
std::optional<Failure> parse_gain(const json::Value& v, GainSetting& out) {
    const std::string wrong =
        "module.gain must be auto, tuner, manual or a preset from 0 to 21 (Airspy R2 and Mini), or auto or "
        "manual (Airspy HF+), not ";
    double d;
    if (v.is_string()) {
        const std::string& s = v.as_string();
        if (same_word(s, "auto")) {
            out = GainSetting::of(GainSetting::Mode::automatic);
            return std::nullopt;
        }
        if (same_word(s, "tuner")) {
            out = GainSetting::of(GainSetting::Mode::tuner);
            return std::nullopt;
        }
        if (same_word(s, "manual")) {
            out = GainSetting::of(GainSetting::Mode::manual);
            return std::nullopt;
        }
        if (s.empty() || s.size() > 2)
            return invalid(wrong + shown(v));
        int n = 0;
        for (char c : s) {
            if (c < '0' || c > '9')
                return invalid(wrong + shown(v));
            n = n * 10 + (c - '0');
        }
        if (n >= presets)
            return invalid(wrong + shown(v));
        out = GainSetting::of_preset(n);
        return std::nullopt;
    }
    if (!whole_in_range(v, 0, presets - 1, d))
        return invalid(wrong + shown(v));
    out = GainSetting::of_preset(static_cast<int>(d));
    return std::nullopt;
}

std::optional<Failure> parse_bool(const char* key, const json::Value& v, bool& out) {
    if (!v.is_bool())
        return invalid(std::string("module.") + key + " must be yes or no (a JSON boolean), not " + shown(v));
    out = v.as_bool();
    return std::nullopt;
}

std::optional<Failure> parse_stage(const char* key, const json::Value& v, int most, int& out) {
    double d;
    if (!whole_in_range(v, 0, most, d))
        return invalid(std::string("module.") + key + " must be a whole number from 0 to " + std::to_string(most) +
                       ", not " + shown(v));
    out = static_cast<int>(d);
    return std::nullopt;
}

std::optional<Failure> parse_value(const KeyInfo& k, const json::Value& v, ModuleSettings& s) {
    switch (k.key) {
    case Key::device:
        return parse_device(v, s.device);
    case Key::gain:
        return parse_gain(v, s.gain);
    case Key::gain_mode:
        if (v.is_string() && v.as_string() == "linearity")
            s.gain_mode = GainLadder::linearity;
        else if (v.is_string() && v.as_string() == "sensitivity")
            s.gain_mode = GainLadder::sensitivity;
        else
            return invalid("module.gain_mode must be linearity or sensitivity, not " + shown(v));
        return std::nullopt;
    case Key::lna_gain:
        return parse_stage(k.name, v, max_lna_gain, s.lna_gain);
    case Key::mixer_gain:
        return parse_stage(k.name, v, max_mixer_gain, s.mixer_gain);
    case Key::vga_gain:
        return parse_stage(k.name, v, max_vga_gain, s.vga_gain);
    case Key::attenuation:
        if (!v.is_number() || !std::isfinite(v.as_number()) || v.as_number() < 0 ||
            v.as_number() > max_attenuation)
            return invalid("module.attenuation must be a number of dB from 0 to 48, not " + shown(v));
        s.attenuation = v.as_number();
        return std::nullopt;
    case Key::agc_threshold:
        if (v.is_string() && v.as_string() == "low")
            s.agc_threshold_high = false;
        else if (v.is_string() && v.as_string() == "high")
            s.agc_threshold_high = true;
        else
            return invalid("module.agc_threshold must be low or high, not " + shown(v));
        return std::nullopt;
    case Key::preamp:
        return parse_bool(k.name, v, s.preamp);
    case Key::bias_tee:
        return parse_bool(k.name, v, s.bias_tee);
    case Key::packing:
        if (v.is_string() && v.as_string() == "auto")
            s.packing = Packing::automatic;
        else if (v.is_string() && v.as_string() == "on")
            s.packing = Packing::on;
        else if (v.is_string() && v.as_string() == "off")
            s.packing = Packing::off;
        else
            return invalid("module.packing must be auto, on or off, not " + shown(v));
        return std::nullopt;
    }
    return invalid("internal: unhandled setting");
}

// Unknown keys are refused all at once, so that the operator can fix every
// one of them in one go.
std::optional<Failure> check_keys(const json::Value& settings) {
    std::vector<std::string> unknown;
    for (const json::Member& m : settings.members())
        if (!find_key(m.key))
            unknown.push_back(m.key);
    if (unknown.empty())
        return std::nullopt;
    std::string names;
    for (size_t i = 0; i < unknown.size() && i < 8; ++i) {
        if (i > 0)
            names += ", ";
        const std::string& key = unknown[i];
        names += "module." + (key.size() > 40 ? key.substr(0, 37) + "..." : key);
    }
    if (unknown.size() > 8)
        names += " and " + std::to_string(unknown.size() - 8) + " more";
    return invalid(std::string(unknown.size() == 1 ? "unknown setting " : "unknown settings ") + names +
                   "; this module takes " + known_keys());
}

}  // namespace

std::optional<Failure> parse_open(const json::Value& message, OpenRequest& out) {
    OpenRequest req;
    double d;

    const json::Value* rate = message.find("sample_rate");
    if (!rate)
        return invalid("open carries no sample_rate");
    if (!whole_in_range(*rate, 1000, 100000000, d))
        return invalid("sample_rate must be a whole number of Hz that the Airspy offers, such as 10000000 for an "
                       "R2, 6000000 for a Mini or 768000 for an HF+, not " +
                       shown(*rate));
    req.sample_rate = static_cast<uint32_t>(d);

    const json::Value* center = message.find("center");
    if (!center)
        return invalid("open carries no center");
    if (!whole_in_range(*center, 0, UINT32_MAX, d))
        return invalid("center must be a whole number of Hz from 0 to 4294967295, not " + shown(*center));
    req.center = static_cast<uint32_t>(d);

    const json::Value* signal = message.find("signal");
    if (!signal)
        return invalid("open carries no signal");
    if (!signal->is_string() || (signal->as_string() != "iq" && signal->as_string() != "real"))
        return invalid("signal must be iq or real, not " + shown(*signal));
    if (signal->as_string() != "iq")
        return invalid("an Airspy delivers I/Q samples; set signal = iq for this band");

    const json::Value* settings = message.find("settings");
    if (settings) {
        if (!settings->is_object())
            return invalid("settings in open must be an object, not " + shown(*settings));
        if (auto f = check_keys(*settings))
            return f;
        for (const json::Member& m : settings->members())
            if (auto f = parse_value(*find_key(m.key), m.value, req.settings))
                return f;
    }
    out = req;
    return std::nullopt;
}

std::optional<Failure> parse_set(const json::Value& settings, LiveChange& out) {
    if (!settings.is_object())
        return invalid("settings in set must be an object, not " + shown(settings));
    if (auto f = check_keys(settings))
        return f;
    LiveChange change;
    ModuleSettings scratch;
    for (const json::Member& m : settings.members()) {
        const KeyInfo& k = *find_key(m.key);
        if (!k.live)
            return invalid("module." + m.key + " cannot change while the band runs; restart the band to apply it");
        if (auto f = parse_value(k, m.value, scratch))
            return f;
        switch (k.key) {
        case Key::gain: change.gain = scratch.gain; break;
        case Key::gain_mode: change.gain_mode = scratch.gain_mode; break;
        case Key::lna_gain: change.lna_gain = scratch.lna_gain; break;
        case Key::mixer_gain: change.mixer_gain = scratch.mixer_gain; break;
        case Key::vga_gain: change.vga_gain = scratch.vga_gain; break;
        case Key::attenuation: change.attenuation = scratch.attenuation; break;
        case Key::agc_threshold: change.agc_threshold_high = scratch.agc_threshold_high; break;
        case Key::preamp: change.preamp = scratch.preamp; break;
        case Key::bias_tee: change.bias_tee = scratch.bias_tee; break;
        case Key::device:
        case Key::packing: break;
        }
    }
    out = change;
    return std::nullopt;
}

json::Value settings_schema() {
    const ModuleSettings defaults;
    json::Value list = json::Value::array();
    for (const KeyInfo& k : all_keys) {
        json::Value s = json::Value::object();
        s.set("key", k.name);
        bool advanced = false;
        switch (k.key) {
        case Key::device:
            s.set("type", "string");
            s.set("label", "Device");
            s.set("default", "");
            s.set("help",
                  "Which Airspy to use: serial:<serial> or index:<n>. Leave empty when exactly one is plugged "
                  "in. fern-airspy --list-devices shows the serials.");
            break;
        case Key::gain:
            s.set("type", "string");
            s.set("label", "Gain");
            s.set("default", "auto");
            s.set("help",
                  "auto: on an R2 or Mini, the highest gain that keeps the converter out of clipping with 6 dB "
                  "to spare, lowered at once when something clips; on an HF+, its own AGC. On an R2 or Mini "
                  "also a preset from 0 to 21 along gain_mode's ladder, tuner for the R820T's own AGC, or "
                  "manual for lna_gain, mixer_gain and vga_gain. On an HF+ also manual, for attenuation.");
            break;
        case Key::gain_mode:
            s.set("type", "choice");
            s.set("label", "Gain presets");
            s.set("choices", json::Value::array().push("linearity").push("sensitivity"));
            s.set("default", ladder_name(defaults.gain_mode));
            s.set("help",
                  "Airspy R2 and Mini. The ladder that a preset and gain = auto walk: linearity keeps strong "
                  "signals clean, sensitivity hears weak ones a little better where the band is quiet.");
            break;
        case Key::lna_gain:
            s.set("type", "number");
            s.set("label", "LNA gain");
            s.set("default", defaults.lna_gain);
            s.set("min", 0);
            s.set("max", max_lna_gain);
            s.set("help", "Airspy R2 and Mini, with gain = manual: the R820T's LNA, 0 to 14.");
            advanced = true;
            break;
        case Key::mixer_gain:
            s.set("type", "number");
            s.set("label", "Mixer gain");
            s.set("default", defaults.mixer_gain);
            s.set("min", 0);
            s.set("max", max_mixer_gain);
            s.set("help", "Airspy R2 and Mini, with gain = manual: the R820T's mixer, 0 to 15.");
            advanced = true;
            break;
        case Key::vga_gain:
            s.set("type", "number");
            s.set("label", "VGA gain");
            s.set("default", defaults.vga_gain);
            s.set("min", 0);
            s.set("max", max_vga_gain);
            s.set("help", "Airspy R2 and Mini, with gain = manual or tuner: the R820T's IF amplifier, 0 to 15.");
            advanced = true;
            break;
        case Key::attenuation:
            s.set("type", "number");
            s.set("label", "Attenuation");
            s.set("default", defaults.attenuation);
            s.set("min", 0);
            s.set("max", max_attenuation);
            s.set("unit", "dB");
            s.set("help",
                  "Airspy HF+, with gain = manual: the input attenuator, 0 to 48 dB in the steps the radio "
                  "offers (6 dB on the HF+ Discovery); the nearest step at or above is used and reported.");
            break;
        case Key::agc_threshold:
            s.set("type", "choice");
            s.set("label", "AGC threshold");
            s.set("choices", json::Value::array().push("low").push("high"));
            s.set("default", defaults.agc_threshold_high ? "high" : "low");
            s.set("help", "Airspy HF+, with gain = auto: where its AGC starts to attenuate.");
            advanced = true;
            break;
        case Key::preamp:
            s.set("type", "boolean");
            s.set("label", "Preamplifier");
            s.set("default", defaults.preamp);
            s.set("help", "Airspy HF+: 6 dB more gain in front, for a quiet antenna.");
            break;
        case Key::bias_tee:
            s.set("type", "boolean");
            s.set("label", "Bias tee");
            s.set("default", defaults.bias_tee);
            s.set("help",
                  "Puts DC on the antenna input to power an active antenna or an LNA (Airspy R2 and Mini, and "
                  "HF+ models that have one).");
            break;
        case Key::packing:
            s.set("type", "choice");
            s.set("label", "USB packing");
            s.set("choices", json::Value::array().push("auto").push("on").push("off"));
            s.set("default", packing_name(defaults.packing));
            s.set("help",
                  "Airspy R2 and Mini: sends 12-bit samples packed, a quarter less USB traffic. auto packs "
                  "above 5 Msps, where a shared or slow USB port starts to lose samples.");
            advanced = true;
            break;
        }
        s.set("live", k.live);
        if (advanced)
            s.set("advanced", true);
        list.push(std::move(s));
    }
    return list;
}

json::Value describe_module() {
    json::Value d = json::Value::object();
    d.set("api", module_api);
    d.set("id", module_id);
    d.set("name", module_name);
    d.set("version", module_version());
    d.set("kind", "input");
    d.set("settings", settings_schema());
    // One block for two families of radio: the HF+ from 9 kHz to 31 MHz (and
    // 60 to 260 MHz, inside the second range), the R2 and Mini from 24 MHz to
    // 1.8 GHz. The rates are the R2's and the Mini's, then the HF+'s widest.
    json::Value tuning = json::Value::object();
    tuning.set("ranges", json::Value::array()
                             .push(json::Value::array().push(9000.0).push(31000000.0))
                             .push(json::Value::array().push(24000000.0).push(1800000000.0)));
    json::Value rates = json::Value::array();
    for (double r : {10000000.0, 6000000.0, 3000000.0, 2500000.0, 912000.0, 768000.0, 384000.0, 192000.0})
        rates.push(r);
    tuning.set("rates", std::move(rates));
    tuning.set("signal", "iq");
    d.set("tuning", tuning);
    return d;
}

}  // namespace fern
