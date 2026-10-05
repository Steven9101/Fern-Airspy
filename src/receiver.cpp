// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include "receiver.h"

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <system_error>
#include <thread>

#include "convert.h"
#include "log.h"
#include "r820t.h"

namespace fern {

const char* usb_error_text(int code) {
    switch (code) {
    case -1: return "input/output error";
    case -2: return "invalid parameter";
    case -3: return "permission denied";
    case -4: return "no such device";
    case -5: return "not found";
    case -6: return "busy";
    case -7: return "timeout";
    case -8: return "overflow";
    case -9: return "pipe error";
    case -10: return "interrupted";
    case -11: return "out of memory";
    case -12: return "not supported";
    default: return "error";
    }
}

std::string serial_text(uint64_t serial) {
    char text[24];
    std::snprintf(text, sizeof text, "%016" PRIX64, serial);
    return text;
}

const char* model_name(Model m) {
    switch (m) {
    case Model::r2: return "Airspy R2";
    case Model::mini: return "Airspy Mini";
    case Model::hf_plus: return "Airspy HF+";
    case Model::hf_discovery: return "Airspy HF+ Discovery";
    case Model::ranger: return "Airspy Ranger";
    case Model::hf_other: break;
    }
    return "Airspy HF+";
}

std::string permission_advice() {
    return "The receiver may not open the Airspy yet. Run FernSDR's install command once more (it keeps your "
           "settings and adds the rule for Airspys), then unplug the Airspy and plug it in again.";
}

Model one_model(const std::string& version) {
    // The firmware names the board first: "AirSpy NOS" for the R0 and R2,
    // "AirSpy MINI" for the Mini. Firmware from before the Mini says neither
    // and runs only on the R0 and R2.
    return version.find("MINI") != std::string::npos ? Model::mini : Model::r2;
}

Model hf_model(uint32_t board_id) {
    switch (board_id) {
    case 1: return Model::hf_plus;
    case 2: return Model::hf_discovery;
    case 3: return Model::ranger;
    default: return Model::hf_other;
    }
}

namespace {

Failure invalid(std::string m) { return Failure{ErrorCode::invalid, std::move(m)}; }

const char* one_error_text(int code) {
    switch (code) {
    case -2: return "invalid parameter";
    case -5: return "not found";
    case -6: return "busy";
    case -11: return "out of memory";
    case -12: return "not supported";
    case -1000: return "libusb error";
    case -1001: return "thread error";
    case -1002: return "streaming thread error";
    case -1003: return "streaming stopped";
    default: return "error";
    }
}

Failure usb_failure(const std::string& what, int code, bool hf) {
    return Failure{ErrorCode::usb, what + " failed (" + (hf ? "libairspyhf error" : one_error_text(code)) +
                                       "); check the USB cable, and plug the Airspy straight into the computer "
                                       "rather than into a hub"};
}

std::string rates_text(const std::vector<uint32_t>& rates) {
    std::string s;
    for (size_t i = 0; i < rates.size(); ++i) {
        if (i > 0)
            s += i + 1 == rates.size() ? " and " : ", ";
        s += std::to_string(rates[i]);
    }
    return s;
}

std::string hz_text(double hz) { return json::serialize(json::Value(std::round(hz * 100) / 100)); }

std::string family_text(Family f) { return f == Family::one ? "Airspy R2 or Mini" : "Airspy HF+"; }

// The libairspy ladder the presets walk.
const char* ladder_label(GainLadder l) { return ladder_name(l); }

// R2 and Mini: real samples from the library, converted here.
class OneSource : public Source {
public:
    OneSource(OneDevice& device, double shift_hz, double iq_rate) : device_(device), converter_(shift_hz, iq_rate) {}

    int start(Stream& stream) override {
        stream_ = &stream;
        return device_.start(&OneSource::on_block, this);
    }
    int stop() override { return device_.stop(); }
    bool streaming() override { return device_.streaming(); }

private:
    static void on_block(const uint16_t* samples, size_t count, uint64_t dropped, void* ctx) {
        OneSource* self = static_cast<OneSource*>(ctx);
        // The converter works on groups of four real samples; libairspy's
        // blocks always hold whole groups.
        count -= count % 4;
        if (self->out_.size() < count)
            self->out_.resize(count);
        uint64_t clipped = 0;
        unsigned peak = 0;
        self->converter_.process(samples, count, self->out_.data(), clipped, peak);
        self->stream_->note_levels(count, clipped, peak);
        // Two real samples make one I/Q pair, also for those lost.
        self->stream_->deliver(self->out_.data(), count * sizeof(int16_t), dropped / 2);
    }

    OneDevice& device_;
    OneConverter converter_;
    std::vector<int16_t> out_;
    Stream* stream_ = nullptr;
};

// HF+: the library delivers float I/Q, already corrected and fine-tuned.
class HfSource : public Source {
public:
    explicit HfSource(HfDevice& device) : device_(device) {}

    int start(Stream& stream) override {
        stream_ = &stream;
        return device_.start(&HfSource::on_block, this);
    }
    int stop() override { return device_.stop(); }
    bool streaming() override { return device_.streaming(); }

private:
    static void on_block(const float* samples, size_t count, uint64_t dropped, void* ctx) {
        HfSource* self = static_cast<HfSource*>(ctx);
        self->stream_->deliver(samples, count * 2 * sizeof(float), dropped);
    }

    HfDevice& device_;
    Stream* stream_ = nullptr;
};

}  // namespace

Receiver::~Receiver() { close(); }

std::string Receiver::which() const {
    return std::string("the ") + model_name(info_.model) + " with serial " + serial_text(info_.serial);
}

const char* Receiver::format() const { return info_.family == Family::one ? "s16" : "f32"; }

size_t Receiver::sample_bytes() const { return info_.family == Family::one ? 2 * sizeof(int16_t) : 2 * sizeof(float); }

std::optional<Failure> Receiver::select(const DeviceSelector& selector, Found& found, uint32_t& index) {
    const Enumeration e = backend_.enumerate();
    if (e.error != 0)
        return Failure{ErrorCode::usb, e.diagnostic};

    const std::string check_cable =
        "Check the USB cable; lsusb lists an Airspy R2 or Mini as 1d50:60a1 and an Airspy HF+ as 03eb:800c.";
    const size_t count = e.devices.size();
    if (count == 0) {
        if (selector.kind == DeviceSelector::Kind::serial)
            return Failure{ErrorCode::no_device, "no Airspy with serial " + serial_text(selector.serial) +
                                                     " is plugged in, nor any other. " + check_cable};
        return Failure{ErrorCode::no_device, "no Airspy is plugged in. " + check_cable};
    }

    std::string inventory;
    bool unreadable = false;
    for (size_t i = 0; i < count && i < 8; ++i) {
        const Found& d = e.devices[i];
        inventory += (i > 0 ? "; " : "") + std::string("index ") + std::to_string(i) + ", " + family_text(d.family);
        if (d.has_serial)
            inventory += " with serial " + serial_text(d.serial);
        if (!d.port.empty())
            inventory += " at USB port " + d.port;
        if (!d.has_serial && d.open_error != 0) {
            inventory += std::string(" that could not be opened (") + usb_error_text(d.open_error) + ")";
            unreadable = true;
        }
    }
    if (count > 8)
        inventory += "; and " + std::to_string(count - 8) + " more";

    size_t chosen = 0;
    switch (selector.kind) {
    case DeviceSelector::Kind::only:
        if (count > 1)
            return invalid(std::to_string(count) + " Airspys are plugged in (" + inventory +
                           "); set module.device = serial:<serial> to choose one");
        chosen = 0;
        break;
    case DeviceSelector::Kind::index:
        if (selector.index >= count)
            return Failure{ErrorCode::no_device, "there is no Airspy at index " + std::to_string(selector.index) +
                                                     "; plugged in: " + inventory};
        chosen = selector.index;
        break;
    case DeviceSelector::Kind::serial: {
        std::vector<size_t> matches;
        for (size_t i = 0; i < count; ++i)
            if (e.devices[i].has_serial && e.devices[i].serial == selector.serial)
                matches.push_back(i);
        if (matches.empty()) {
            std::string text = "no Airspy with serial " + serial_text(selector.serial) + " can be seen; plugged in: " +
                               inventory;
            if (unreadable)
                return Failure{ErrorCode::usb, text + ". " + permission_advice()};
            return Failure{ErrorCode::no_device, text};
        }
        if (matches.size() > 1)
            return invalid(std::to_string(matches.size()) + " Airspys have the serial " +
                           serial_text(selector.serial) + "; choose one with module.device = index:<n>");
        chosen = matches[0];
        break;
    }
    }

    found = e.devices[chosen];
    index = static_cast<uint32_t>(chosen);
    const std::string what = std::string("the ") + family_text(found.family) +
                             (found.port.empty() ? std::string() : " at USB port " + found.port);
    switch (found.open_error) {
    case 0:
        break;
    case usb_error::access:
        return Failure{ErrorCode::usb, permission_advice()};
    case usb_error::busy:
        return Failure{ErrorCode::busy, what +
                                            " is in use by another program, such as spyserver, SDR++ or SDR# "
                                            "server, airspy_rx or another FernSDR band. Stop that program, or "
                                            "choose another Airspy with module.device"};
    case usb_error::no_device:
        return Failure{ErrorCode::no_device, what + " disappeared while it was being opened"};
    default:
        return Failure{ErrorCode::usb, "could not open " + what + " (" + usb_error_text(found.open_error) +
                                           "); check the USB cable, and plug the Airspy straight into the "
                                           "computer rather than into a hub"};
    }
    if (!found.has_serial)
        return Failure{ErrorCode::usb, what + " reports no serial number that the Airspy library can open it by; "
                                              "unplug it and plug it in again"};
    return std::nullopt;
}

std::optional<Failure> Receiver::check_family(const ModuleSettings& s) const {
    const ModuleSettings defaults;
    const std::string here = std::string("; this is an ") + model_name(info_.model) + ". Remove it from the band";
    if (info_.family == Family::one) {
        if (s.attenuation != defaults.attenuation)
            return invalid("module.attenuation is for the Airspy HF+ only" + here);
        if (s.agc_threshold_high != defaults.agc_threshold_high)
            return invalid("module.agc_threshold is for the Airspy HF+ only" + here);
        if (s.preamp != defaults.preamp)
            return invalid("module.preamp is for the Airspy HF+ only" + here);
        return std::nullopt;
    }
    if (s.gain.mode == GainSetting::Mode::tuner || s.gain.mode == GainSetting::Mode::preset)
        return invalid("module.gain = " + json::serialize(gain_json(s.gain)) +
                       " is for the Airspy R2 and Mini; an HF+ takes auto (its AGC) or manual (attenuation)");
    if (s.gain_mode != defaults.gain_mode)
        return invalid("module.gain_mode is for the Airspy R2 and Mini only" + here);
    if (s.lna_gain != defaults.lna_gain || s.mixer_gain != defaults.mixer_gain || s.vga_gain != defaults.vga_gain)
        return invalid("module.lna_gain, mixer_gain and vga_gain are for the Airspy R2 and Mini only" + here);
    if (s.packing != defaults.packing)
        return invalid("module.packing is for the Airspy R2 and Mini only" + here);
    if (s.bias_tee && info_.bias_tees <= 0)
        return invalid("this " + std::string(model_name(info_.model)) +
                       " has no bias tee its firmware can switch; remove module.bias_tee");
    return std::nullopt;
}

std::optional<Failure> Receiver::check_family(const LiveChange& c) const {
    ModuleSettings s;
    if (c.gain)
        s.gain = *c.gain;
    if (c.gain_mode)
        s.gain_mode = *c.gain_mode;
    if (c.lna_gain)
        s.lna_gain = *c.lna_gain;
    if (c.mixer_gain)
        s.mixer_gain = *c.mixer_gain;
    if (c.vga_gain)
        s.vga_gain = *c.vga_gain;
    if (c.attenuation)
        s.attenuation = *c.attenuation;
    if (c.agc_threshold_high)
        s.agc_threshold_high = *c.agc_threshold_high;
    if (c.preamp)
        s.preamp = *c.preamp;
    if (c.bias_tee)
        s.bias_tee = *c.bias_tee;
    return check_family(s);
}

std::optional<Failure> Receiver::open(const OpenRequest& request) {
    close();
    Found found;
    uint32_t index = 0;
    if (auto f = select(request.settings.device, found, index))
        return f;
    info_ = DeviceInfo{};
    info_.index = index;
    info_.family = found.family;
    info_.serial = found.serial;
    info_.port = found.port;
    effective_ = Effective{};
    std::optional<Failure> f =
        found.family == Family::one ? open_one(request, found) : open_hf(request, found);
    if (f)
        close();
    return f;
}

std::optional<Failure> Receiver::open_one(const OpenRequest& request, const Found& found) {
    const ModuleSettings& s = request.settings;
    std::unique_ptr<OneDevice> device;
    int r = backend_.open_one(found.serial, device);
    if (r == 0 && !device)
        return Failure{ErrorCode::internal, "opening the Airspy returned no device"};
    if (r != 0) {
        const std::string what = "the Airspy R2 or Mini with serial " + serial_text(found.serial);
        if (r == -5)
            return Failure{ErrorCode::busy, what +
                                                " could not be opened: another program may have taken it just now, "
                                                "or it was unplugged. Stop other Airspy programs and try again"};
        return Failure{ErrorCode::usb, "could not open " + what + " (" + one_error_text(r) +
                                           "); check the USB cable, and plug the Airspy straight into the "
                                           "computer rather than into a hub"};
    }
    one_ = std::move(device);

    if (one_->version(info_.firmware) != 0)
        info_.firmware.clear();
    info_.model = one_model(info_.firmware);
    if ((r = one_->samplerates(info_.rates)) != 0 || info_.rates.empty())
        return usb_failure("reading the sample rates", r, false);
    log_line("opened the %s with serial %s at USB port %s, firmware %s", model_name(info_.model),
             serial_text(info_.serial).c_str(), info_.port.empty() ? "(unknown)" : info_.port.c_str(),
             info_.firmware.empty() ? "(unknown)" : info_.firmware.c_str());

    const auto rate = std::find(info_.rates.begin(), info_.rates.end(), request.sample_rate);
    if (rate == info_.rates.end())
        return invalid("sample_rate " + std::to_string(request.sample_rate) + " is not one the " +
                       model_name(info_.model) + " offers; it runs at " + rates_text(info_.rates) +
                       " samples/s. Set sample_rate to one of them");
    const uint32_t rate_index = static_cast<uint32_t>(rate - info_.rates.begin());

    if (auto f = check_family(s))
        return f;

    // The firmware tunes with a stored correction of its crystal, when it
    // runs from that crystal; so must the module, or it would predict
    // another PLL step than the firmware takes.
    info_.xtal = info_.model == Model::mini ? mini_xtal_hz : r2_xtal_hz;
    bool stored = false;
    int32_t ppb = 0;
    if (one_->read_calibration(stored, ppb) != 0) {
        log_line("could not read the frequency correction stored in the Airspy; assuming there is none");
        stored = false;
    }
    info_.calibration_stored = stored;
    info_.calibration_ppb = stored ? ppb : 0;
    if (stored) {
        bool own_crystal = true;
        bool lost = true;
        if (info_.model == Model::r2) {
            if (one_->clock_input_lost(lost) == 0)
                own_crystal = lost;
            else
                log_line("could not read whether an external clock drives the Airspy; assuming it does not");
        }
        info_.calibration_applied = own_crystal;
        if (own_crystal)
            info_.xtal = calibrated_xtal(info_.xtal, ppb);
        log_line("the Airspy carries a frequency correction of %d ppb from airspy_calibrate, which its firmware %s; "
                 "FernSDR's ppm corrects the whole band better, see the README",
                 static_cast<int>(ppb), own_crystal ? "applies to the tuner" : "ignores with an external clock");
    }

    const auto tuning = one_tuning(info_.xtal, request.center, request.sample_rate);
    if (!tuning) {
        const uint32_t half = request.sample_rate / 2;
        return invalid("at " + std::to_string(request.sample_rate) + " samples/s the " + model_name(info_.model) +
                       " tunes centres from " + std::to_string(r820t_lo_min - half) + " to " +
                       std::to_string(r820t_lo_max - half) + " Hz, and center is " +
                       std::to_string(request.center) + " Hz. Choose a centre in that range");
    }
    if (request.center < 24000000 || request.center > 1800000000)
        log_line("%u Hz is outside the 24 MHz to 1.8 GHz the %s is made for; expect it to hear less there",
                 request.center, model_name(info_.model));

    if ((r = one_->set_samplerate_index(rate_index)) != 0)
        return usb_failure("setting the sample rate", r, false);
    const bool packing = s.packing == Packing::on || (s.packing == Packing::automatic && request.sample_rate > 5000000);
    if ((r = one_->set_packing(packing)) != 0)
        return usb_failure(packing ? "switching packing on" : "switching packing off", r, false);
    // The firmware adds the IF of the rate set above.
    if ((r = one_->set_freq(request.center)) != 0)
        return usb_failure("tuning to " + std::to_string(request.center) + " Hz", r, false);
    if (auto f = set_one_gain(s.gain, s.gain_mode, s.lna_gain, s.mixer_gain, s.vga_gain))
        return f;
    if (auto f = set_bias(s.bias_tee))
        return f;

    effective_.sample_rate = request.sample_rate;
    effective_.center = request.center;
    effective_.packing = packing;
    effective_.pll_offset = tuning->offset;
    source_ = std::make_unique<OneSource>(*one_, tuning->offset, request.sample_rate);
    log_line("tuned to %u Hz at %u samples/s; the R820T's PLL landed %s Hz from it, which the module shifts out; "
             "packing %s, gain %s",
             request.center, request.sample_rate, hz_text(tuning->offset).c_str(), packing ? "on" : "off",
             effective_.gain.mode == GainSetting::Mode::automatic
                 ? ("auto from " + std::string(ladder_label(effective_.gain_mode)) + " " +
                    std::to_string(effective_.preset_now))
                       .c_str()
                 : json::serialize(gain_json(effective_.gain)).c_str());
    return std::nullopt;
}

std::optional<Failure> Receiver::open_hf(const OpenRequest& request, const Found& found) {
    const ModuleSettings& s = request.settings;
    std::unique_ptr<HfDevice> device;
    int r = backend_.open_hf(found.serial, device);
    if (r == 0 && !device)
        return Failure{ErrorCode::internal, "opening the Airspy HF+ returned no device"};
    if (r != 0)
        return Failure{ErrorCode::busy, "the Airspy HF+ with serial " + serial_text(found.serial) +
                                            " could not be opened: another program may have taken it just now, or "
                                            "it was unplugged. Stop other Airspy programs and try again"};
    hf_ = std::move(device);

    uint32_t board = 0;
    if (hf_->board_id(board) != 0)
        board = 0;
    info_.model = hf_model(board);
    if (hf_->version(info_.firmware) != 0)
        info_.firmware.clear();
    if ((r = hf_->samplerates(info_.rates)) != 0 || info_.rates.empty())
        return usb_failure("reading the sample rates", r, true);
    if (hf_->att_steps(info_.att_steps) != 0)
        info_.att_steps.clear();
    std::sort(info_.att_steps.begin(), info_.att_steps.end());
    if (hf_->bias_tee_count(info_.bias_tees) != 0)
        info_.bias_tees = 0;
    log_line("opened the %s with serial %s at USB port %s, firmware %s", model_name(info_.model),
             serial_text(info_.serial).c_str(), info_.port.empty() ? "(unknown)" : info_.port.c_str(),
             info_.firmware.empty() ? "(unknown)" : info_.firmware.c_str());

    if (std::find(info_.rates.begin(), info_.rates.end(), request.sample_rate) == info_.rates.end())
        return invalid("sample_rate " + std::to_string(request.sample_rate) + " is not one the " +
                       model_name(info_.model) + " offers; it runs at " + rates_text(info_.rates) +
                       " samples/s. Set sample_rate to one of them");
    if (request.center > 260000000)
        return invalid("the " + std::string(model_name(info_.model)) +
                       " receives up to 31 MHz and from 60 to 260 MHz, and center is " +
                       std::to_string(request.center) + " Hz. Choose a centre in those ranges");
    if (request.center > 31000000 && request.center < 60000000)
        log_line("%u Hz is between the HF+'s two inputs, 31 and 60 MHz; expect it to hear less there",
                 request.center);
    if (auto f = check_family(s))
        return f;

    if ((r = hf_->set_samplerate(request.sample_rate)) != 0)
        return usb_failure("setting the sample rate", r, true);
    if ((r = hf_->set_freq(static_cast<double>(request.center))) != 0)
        return usb_failure("tuning to " + std::to_string(request.center) + " Hz", r, true);
    if (auto f = set_hf_gain(s.gain, s.agc_threshold_high, s.attenuation))
        return f;
    if ((r = hf_->set_hf_lna(s.preamp)) != 0)
        return usb_failure(s.preamp ? "switching the preamplifier on" : "switching the preamplifier off", r, true);
    effective_.preamp = s.preamp;
    if (auto f = set_bias(s.bias_tee))
        return f;

    effective_.sample_rate = request.sample_rate;
    effective_.center = request.center;
    source_ = std::make_unique<HfSource>(*hf_);
    log_line("tuned to %u Hz at %u samples/s, gain %s", request.center, request.sample_rate,
             effective_.gain.mode == GainSetting::Mode::automatic ? "auto (the HF+'s AGC)"
                                                                  : ("manual, attenuation " +
                                                                     hz_text(effective_.attenuation) + " dB")
                                                                        .c_str());
    return std::nullopt;
}

std::optional<Failure> Receiver::set_one_gain(const GainSetting& gain, GainLadder ladder, int lna, int mixer,
                                              int vga) {
    int r;
    switch (gain.mode) {
    case GainSetting::Mode::automatic:
    case GainSetting::Mode::preset: {
        // Already under the module's control: the preset it has found stays,
        // also on the other ladder, whose presets are about as loud.
        int preset = gain.mode == GainSetting::Mode::preset ? gain.preset : auto_start_preset;
        if (gain.mode == GainSetting::Mode::automatic && effective_.gain.mode == GainSetting::Mode::automatic &&
            source_)
            preset = effective_.preset_now;
        r = ladder == GainLadder::sensitivity ? one_->set_sensitivity_gain(static_cast<uint8_t>(preset))
                                              : one_->set_linearity_gain(static_cast<uint8_t>(preset));
        if (r != 0)
            return usb_failure("setting the gain to " + std::string(ladder_label(ladder)) + " " +
                                   std::to_string(preset),
                               r, false);
        effective_.preset_now = preset;
        break;
    }
    case GainSetting::Mode::tuner:
        if ((r = one_->set_lna_agc(true)) != 0 || (r = one_->set_mixer_agc(true)) != 0)
            return usb_failure("switching the R820T's AGC on", r, false);
        if ((r = one_->set_vga_gain(static_cast<uint8_t>(vga))) != 0)
            return usb_failure("setting the VGA gain", r, false);
        break;
    case GainSetting::Mode::manual:
        if ((r = one_->set_lna_agc(false)) != 0 || (r = one_->set_mixer_agc(false)) != 0)
            return usb_failure("switching the R820T's AGC off", r, false);
        if ((r = one_->set_lna_gain(static_cast<uint8_t>(lna))) != 0)
            return usb_failure("setting the LNA gain", r, false);
        if ((r = one_->set_mixer_gain(static_cast<uint8_t>(mixer))) != 0)
            return usb_failure("setting the mixer gain", r, false);
        if ((r = one_->set_vga_gain(static_cast<uint8_t>(vga))) != 0)
            return usb_failure("setting the VGA gain", r, false);
        break;
    }
    effective_.gain = gain;
    effective_.gain_mode = ladder;
    effective_.lna_gain = lna;
    effective_.mixer_gain = mixer;
    effective_.vga_gain = vga;
    return std::nullopt;
}

std::optional<Failure> Receiver::set_hf_gain(const GainSetting& gain, bool threshold_high, double attenuation) {
    // The step at or above what was asked for, as libairspyhf picks it;
    // kept while the AGC runs, for when it is switched off.
    double step = attenuation;
    if (!info_.att_steps.empty()) {
        step = info_.att_steps.back();
        for (float s : info_.att_steps)
            if (s >= attenuation - 1e-6) {
                step = s;
                break;
            }
    }
    int r;
    if (gain.mode == GainSetting::Mode::automatic) {
        if ((r = hf_->set_hf_agc(true)) != 0)
            return usb_failure("switching the AGC on", r, true);
        if ((r = hf_->set_hf_agc_threshold(threshold_high)) != 0)
            return usb_failure("setting the AGC threshold", r, true);
        effective_.attenuation = step;
    } else {
        if ((r = hf_->set_hf_agc(false)) != 0)
            return usb_failure("switching the AGC off", r, true);
        if ((r = hf_->set_att(static_cast<float>(step))) != 0)
            return usb_failure("setting the attenuator", r, true);
        effective_.attenuation = step;
    }
    effective_.gain = gain;
    effective_.agc_threshold_high = threshold_high;
    return std::nullopt;
}

std::optional<Failure> Receiver::set_bias(bool on) {
    int r = 0;
    if (info_.family == Family::one) {
        r = one_->set_bias_tee(on);
    } else if (info_.bias_tees > 0) {
        r = hf_->set_bias_tee(on);
    } else if (on) {
        return invalid("this " + std::string(model_name(info_.model)) +
                       " has no bias tee its firmware can switch; remove module.bias_tee");
    }
    if (r != 0)
        return usb_failure(on ? "switching the bias tee on" : "switching the bias tee off", r,
                           info_.family == Family::hf);
    effective_.bias_tee = on;
    bias_on_by_module_ = on;
    return std::nullopt;
}

std::optional<Failure> Receiver::apply(const LiveChange& change) {
    if (!source_)
        return Failure{ErrorCode::internal, "no Airspy is open"};
    if (auto f = check_family(change))
        return f;
    if (info_.family == Family::one) {
        const GainSetting gain = change.gain.value_or(effective_.gain);
        const GainLadder ladder = change.gain_mode.value_or(effective_.gain_mode);
        const int lna = change.lna_gain.value_or(effective_.lna_gain);
        const int mixer = change.mixer_gain.value_or(effective_.mixer_gain);
        const int vga = change.vga_gain.value_or(effective_.vga_gain);
        const bool stages = change.lna_gain || change.mixer_gain || change.vga_gain;
        if (change.gain || change.gain_mode || (stages && gain.mode == GainSetting::Mode::manual) ||
            (change.vga_gain && gain.mode == GainSetting::Mode::tuner)) {
            if (auto f = set_one_gain(gain, ladder, lna, mixer, vga))
                return f;
        } else {
            // Kept for when gain = manual is chosen.
            effective_.lna_gain = lna;
            effective_.mixer_gain = mixer;
            effective_.vga_gain = vga;
        }
    } else {
        const GainSetting gain = change.gain.value_or(effective_.gain);
        const bool high = change.agc_threshold_high.value_or(effective_.agc_threshold_high);
        const double attenuation = change.attenuation.value_or(effective_.attenuation);
        if (change.gain || change.agc_threshold_high || change.attenuation)
            if (auto f = set_hf_gain(gain, high, attenuation))
                return f;
        if (change.preamp) {
            const int r = hf_->set_hf_lna(*change.preamp);
            if (r != 0)
                return usb_failure(*change.preamp ? "switching the preamplifier on" : "switching the preamplifier off",
                                   r, true);
            effective_.preamp = *change.preamp;
        }
    }
    if (change.bias_tee)
        if (auto f = set_bias(*change.bias_tee))
            return f;
    return std::nullopt;
}

bool Receiver::controls_gain() const {
    return source_ && info_.family == Family::one && effective_.gain.mode == GainSetting::Mode::automatic;
}

std::vector<int> Receiver::ladder_steps() const {
    // libairspy publishes no gain in dB for its presets. The control needs a
    // figure only to judge whether the next preset leaves room: 3 dB a step,
    // about what the R820T's stages add up to over the ladder, errs on the
    // side of a step too few.
    std::vector<int> steps;
    for (int i = 0; i < presets; ++i)
        steps.push_back(30 * i);
    return steps;
}

std::vector<std::string> Receiver::ladder_labels() const {
    std::vector<std::string> labels;
    for (int i = 0; i < presets; ++i)
        labels.push_back(std::string(ladder_label(effective_.gain_mode)) + " " + std::to_string(i));
    return labels;
}

std::optional<Failure> Receiver::set_preset(int preset) {
    if (!controls_gain() || preset < 0 || preset >= presets)
        return Failure{ErrorCode::internal, "the gain preset cannot be changed now"};
    const int r = effective_.gain_mode == GainLadder::sensitivity
                      ? one_->set_sensitivity_gain(static_cast<uint8_t>(preset))
                      : one_->set_linearity_gain(static_cast<uint8_t>(preset));
    if (r != 0)
        return usb_failure("setting the gain to " + std::string(ladder_label(effective_.gain_mode)) + " " +
                               std::to_string(preset),
                           r, false);
    effective_.preset_now = preset;
    return std::nullopt;
}

json::Value Receiver::device_json() const {
    json::Value d = json::Value::object();
    d.set("name", model_name(info_.model));
    d.set("family", family_id(info_.family));
    d.set("serial", serial_text(info_.serial));
    if (!info_.firmware.empty())
        d.set("firmware", info_.firmware);
    if (!info_.port.empty())
        d.set("port", info_.port);
    d.set("index", info_.index);
    return d;
}

namespace {

json::Value one_gain_now(const Effective& e) {
    return json::Value(std::string(ladder_name(e.gain_mode)) + " " + std::to_string(e.preset_now));
}

}  // namespace

json::Value Receiver::settings_json() const {
    json::Value s = json::Value::object();
    s.set("gain", gain_json(effective_.gain));
    if (info_.family == Family::one) {
        if (effective_.gain.mode == GainSetting::Mode::automatic)
            s.set("gain_now", one_gain_now(effective_));
        s.set("gain_mode", ladder_name(effective_.gain_mode));
        s.set("lna_gain", effective_.lna_gain);
        s.set("mixer_gain", effective_.mixer_gain);
        s.set("vga_gain", effective_.vga_gain);
        s.set("packing", effective_.packing ? "on" : "off");
        s.set("pll_offset", std::round(effective_.pll_offset * 100) / 100);
    } else {
        s.set("attenuation", effective_.attenuation);
        s.set("agc_threshold", effective_.agc_threshold_high ? "high" : "low");
        s.set("preamp", effective_.preamp);
    }
    s.set("bias_tee", effective_.bias_tee);
    return s;
}

json::Value Receiver::settings_json(const LiveChange& change) const {
    json::Value s = json::Value::object();
    if (change.gain) {
        s.set("gain", gain_json(effective_.gain));
        if (info_.family == Family::one && effective_.gain.mode == GainSetting::Mode::automatic)
            s.set("gain_now", one_gain_now(effective_));
    }
    if (change.gain_mode)
        s.set("gain_mode", ladder_name(effective_.gain_mode));
    if (change.lna_gain)
        s.set("lna_gain", effective_.lna_gain);
    if (change.mixer_gain)
        s.set("mixer_gain", effective_.mixer_gain);
    if (change.vga_gain)
        s.set("vga_gain", effective_.vga_gain);
    if (change.attenuation)
        s.set("attenuation", effective_.attenuation);
    if (change.agc_threshold_high)
        s.set("agc_threshold", effective_.agc_threshold_high ? "high" : "low");
    if (change.preamp)
        s.set("preamp", effective_.preamp);
    if (change.bias_tee)
        s.set("bias_tee", effective_.bias_tee);
    return s;
}

namespace {

struct CloseJob {
    std::unique_ptr<OneDevice> one;
    std::unique_ptr<HfDevice> hf;
    bool switch_bias_off = false;
    std::atomic<bool> done{false};
};

void run_close(CloseJob& job) {
    if (job.switch_bias_off) {
        const int r = job.one ? job.one->set_bias_tee(false) : job.hf ? job.hf->set_bias_tee(false) : 0;
        if (r != 0)
            log_line("could not switch the bias tee off before closing the Airspy");
    }
    job.one.reset();
    job.hf.reset();
    job.done.store(true);
}

}  // namespace

void Receiver::close(bool device_answers) {
    source_.reset();
    if (!one_ && !hf_)
        return;
    CloseJob job;
    job.one = std::move(one_);
    job.hf = std::move(hf_);
    job.switch_bias_off = device_answers && bias_on_by_module_;
    bias_on_by_module_ = false;
    run_close(job);
}

bool Receiver::close_by(std::chrono::steady_clock::time_point deadline, bool device_answers) {
    source_.reset();
    if (!one_ && !hf_)
        return true;
    // A device that no longer answers costs the vendor library its 500 ms
    // timeout for each control transfer. The job is shared so that it
    // outlives this receiver if the close does not finish in time.
    auto job = std::make_shared<CloseJob>();
    job->one = std::move(one_);
    job->hf = std::move(hf_);
    job->switch_bias_off = device_answers && bias_on_by_module_;
    bias_on_by_module_ = false;
    std::thread worker;
    try {
        worker = std::thread([job] { run_close(*job); });
    } catch (const std::system_error&) {
        run_close(*job);
        return true;
    }
    while (!job->done.load() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (job->done.load()) {
        worker.join();
        return true;
    }
    worker.detach();
    return false;
}

void Receiver::abandon() {
    source_.reset();
    (void)one_.release();
    (void)hf_.release();
    bias_on_by_module_ = false;
}

}  // namespace fern
