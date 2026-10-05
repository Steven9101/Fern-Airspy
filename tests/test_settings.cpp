// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include <string>

#include "json.h"
#include "settings.h"
#include "test.h"

using fern::GainLadder;
using fern::GainSetting;
using fern::OpenRequest;
using fern::json::Value;

namespace {

Value parsed(const std::string& text) {
    Value v;
    std::string error;
    if (!fern::json::parse(text, v, error))
        test::report(__FILE__, __LINE__, "bad JSON in test: " + error);
    return v;
}

Value open_with(const std::string& settings, const std::string& rate = "10000000",
                const std::string& center = "100000000") {
    return parsed("{\"type\":\"open\",\"sample_rate\":" + rate + ",\"center\":" + center +
                  ",\"signal\":\"iq\",\"settings\":" + settings + "}");
}

std::string refusal(const Value& message) {
    OpenRequest r;
    const auto f = fern::parse_open(message, r);
    if (!f)
        return "";
    CHECK(f->code == fern::ErrorCode::invalid);
    return f->message;
}

}  // namespace

TEST(settings_open_defaults) {
    OpenRequest r;
    REQUIRE(!fern::parse_open(open_with("{}"), r));
    CHECK_EQ(r.sample_rate, 10000000u);
    CHECK_EQ(r.center, 100000000u);
    CHECK(r.settings.device.kind == fern::DeviceSelector::Kind::only);
    CHECK(r.settings.gain == GainSetting::of(GainSetting::Mode::automatic));
    CHECK(r.settings.gain_mode == GainLadder::linearity);
    CHECK_EQ(r.settings.lna_gain, 1);
    CHECK_EQ(r.settings.mixer_gain, 5);
    CHECK_EQ(r.settings.vga_gain, 5);
    CHECK_EQ(r.settings.attenuation, 0.0);
    CHECK(!r.settings.bias_tee);
    CHECK(r.settings.packing == fern::Packing::automatic);
    // No settings at all is the same.
    OpenRequest bare;
    REQUIRE(!fern::parse_open(parsed("{\"type\":\"open\",\"sample_rate\":768000,\"center\":7100000,"
                                     "\"signal\":\"iq\"}"),
                              bare));
    CHECK_EQ(bare.sample_rate, 768000u);
}

TEST(settings_open_reads_every_key) {
    OpenRequest r;
    REQUIRE(!fern::parse_open(open_with("{\"device\":\"serial:0x26D464DC2A8E7A2B\",\"gain\":\"13\","
                                        "\"gain_mode\":\"sensitivity\",\"lna_gain\":14,\"mixer_gain\":15,"
                                        "\"vga_gain\":0,\"attenuation\":12,\"agc_threshold\":\"high\","
                                        "\"preamp\":true,\"bias_tee\":true,\"packing\":\"off\"}"),
                              r));
    CHECK(r.settings.device.kind == fern::DeviceSelector::Kind::serial);
    CHECK_EQ(r.settings.device.serial, 0x26D464DC2A8E7A2Bull);
    CHECK(r.settings.gain == GainSetting::of_preset(13));
    CHECK(r.settings.gain_mode == GainLadder::sensitivity);
    CHECK_EQ(r.settings.lna_gain, 14);
    CHECK_EQ(r.settings.mixer_gain, 15);
    CHECK_EQ(r.settings.vga_gain, 0);
    CHECK_EQ(r.settings.attenuation, 12.0);
    CHECK(r.settings.agc_threshold_high);
    CHECK(r.settings.preamp);
    CHECK(r.settings.bias_tee);
    CHECK(r.settings.packing == fern::Packing::off);
}

TEST(settings_device_forms) {
    OpenRequest r;
    REQUIRE(!fern::parse_open(open_with("{\"device\":\"serial:3652a98081830f1e\"}"), r));
    CHECK_EQ(r.settings.device.serial, 0x3652A98081830F1Eull);
    REQUIRE(!fern::parse_open(open_with("{\"device\":\"serial:12AB\"}"), r));
    CHECK_EQ(r.settings.device.serial, 0x12ABull);
    REQUIRE(!fern::parse_open(open_with("{\"device\":\"index:2\"}"), r));
    CHECK(r.settings.device.kind == fern::DeviceSelector::Kind::index);
    CHECK_EQ(r.settings.device.index, 2u);
    REQUIRE(!fern::parse_open(open_with("{\"device\":\"\"}"), r));
    CHECK(r.settings.device.kind == fern::DeviceSelector::Kind::only);
    for (const char* bad : {"\"serial:\"", "\"serial:0x\"", "\"serial:12345678901234567\"", "\"serial:12G4\"",
                            "\"index:\"", "\"index:-1\"", "\"index:99999999999\"", "\"port:1-2\"", "3"}) {
        CHECK_HAS(refusal(open_with(std::string("{\"device\":") + bad + "}")), "module.device must be");
    }
}

TEST(settings_gain_forms) {
    OpenRequest r;
    REQUIRE(!fern::parse_open(open_with("{\"gain\":\"AUTO\"}"), r));
    CHECK(r.settings.gain == GainSetting::of(GainSetting::Mode::automatic));
    REQUIRE(!fern::parse_open(open_with("{\"gain\":\"tuner\"}"), r));
    CHECK(r.settings.gain == GainSetting::of(GainSetting::Mode::tuner));
    REQUIRE(!fern::parse_open(open_with("{\"gain\":\"manual\"}"), r));
    CHECK(r.settings.gain == GainSetting::of(GainSetting::Mode::manual));
    REQUIRE(!fern::parse_open(open_with("{\"gain\":\"0\"}"), r));
    CHECK(r.settings.gain == GainSetting::of_preset(0));
    REQUIRE(!fern::parse_open(open_with("{\"gain\":21}"), r));
    CHECK(r.settings.gain == GainSetting::of_preset(21));
    for (const char* bad : {"\"22\"", "22", "-1", "10.5", "\"10.5\"", "\"\"", "\"loud\"", "\"1 2\"", "true", "null"})
        CHECK_HAS(refusal(open_with(std::string("{\"gain\":") + bad + "}")), "module.gain must be auto, tuner");
}

TEST(settings_refuse_bad_values) {
    CHECK_HAS(refusal(open_with("{\"gain_mode\":\"loud\"}")), "module.gain_mode must be linearity or sensitivity");
    CHECK_HAS(refusal(open_with("{\"lna_gain\":15}")), "module.lna_gain must be a whole number from 0 to 14");
    CHECK_HAS(refusal(open_with("{\"mixer_gain\":16}")), "module.mixer_gain must be a whole number from 0 to 15");
    CHECK_HAS(refusal(open_with("{\"vga_gain\":2.5}")), "module.vga_gain must be a whole number from 0 to 15");
    CHECK_HAS(refusal(open_with("{\"attenuation\":49}")), "module.attenuation must be a number of dB from 0 to 48");
    CHECK_HAS(refusal(open_with("{\"attenuation\":\"6\"}")), "module.attenuation must be");
    CHECK_HAS(refusal(open_with("{\"agc_threshold\":\"medium\"}")), "module.agc_threshold must be low or high");
    CHECK_HAS(refusal(open_with("{\"preamp\":\"yes\"}")), "module.preamp must be yes or no");
    CHECK_HAS(refusal(open_with("{\"bias_tee\":1}")), "module.bias_tee must be yes or no");
    CHECK_HAS(refusal(open_with("{\"packing\":true}")), "module.packing must be auto, on or off");
}

TEST(settings_refuse_unknown_keys_all_at_once) {
    const std::string m = refusal(open_with("{\"ppm\":1,\"gain\":\"auto\",\"squelch\":3}"));
    CHECK_HAS(m, "unknown settings module.ppm, module.squelch");
    CHECK_HAS(m, "this module takes device, gain, gain_mode");
}

TEST(settings_refuse_bad_open_fields) {
    CHECK_HAS(refusal(parsed("{\"type\":\"open\",\"center\":1,\"signal\":\"iq\"}")), "open carries no sample_rate");
    CHECK_EQ(refusal(open_with("{}", "2.5e6")), std::string());
    CHECK_HAS(refusal(open_with("{}", "999")), "sample_rate must be a whole number of Hz that the Airspy offers");
    CHECK_HAS(refusal(open_with("{}", "\"10000000\"")), "sample_rate must be");
    CHECK_HAS(refusal(open_with("{}", "10000000", "-1")), "center must be a whole number of Hz");
    CHECK_HAS(refusal(open_with("{}", "10000000", "100.5")), "center must be a whole number of Hz");
    CHECK_HAS(refusal(parsed("{\"type\":\"open\",\"sample_rate\":768000,\"center\":7100000}")),
              "open carries no signal");
    CHECK_HAS(refusal(parsed("{\"type\":\"open\",\"sample_rate\":768000,\"center\":0,\"signal\":\"real\"}")),
              "an Airspy delivers I/Q samples; set signal = iq");
    CHECK_HAS(refusal(parsed("{\"type\":\"open\",\"sample_rate\":768000,\"center\":0,\"signal\":\"iq\","
                             "\"settings\":[]}")),
              "settings in open must be an object");
}

TEST(settings_set_takes_only_live_keys) {
    fern::LiveChange c;
    REQUIRE(!fern::parse_set(parsed("{\"gain\":7,\"bias_tee\":true,\"attenuation\":6,\"preamp\":false,"
                                    "\"gain_mode\":\"sensitivity\",\"lna_gain\":3,\"mixer_gain\":4,\"vga_gain\":9,"
                                    "\"agc_threshold\":\"high\"}"),
                             c));
    CHECK(*c.gain == GainSetting::of_preset(7));
    CHECK(*c.bias_tee);
    CHECK_EQ(*c.attenuation, 6.0);
    CHECK(!*c.preamp);
    CHECK(*c.gain_mode == GainLadder::sensitivity);
    CHECK_EQ(*c.lna_gain, 3);
    CHECK_EQ(*c.mixer_gain, 4);
    CHECK_EQ(*c.vga_gain, 9);
    CHECK(*c.agc_threshold_high);
    fern::LiveChange d;
    REQUIRE(!fern::parse_set(parsed("{}"), d));
    CHECK(!d.gain && !d.bias_tee);
    auto f = fern::parse_set(parsed("{\"packing\":\"on\"}"), d);
    REQUIRE(f);
    CHECK_HAS(f->message, "module.packing cannot change while the band runs");
    f = fern::parse_set(parsed("{\"device\":\"index:0\"}"), d);
    REQUIRE(f);
    CHECK_HAS(f->message, "module.device cannot change while the band runs");
    f = fern::parse_set(parsed("[]"), d);
    REQUIRE(f);
    CHECK_HAS(f->message, "settings in set must be an object");
}

TEST(settings_describe_matches_the_contract) {
    const Value d = fern::describe_module();
    CHECK_EQ(d.find("id")->as_string(), std::string("airspy"));
    CHECK_EQ(d.find("kind")->as_string(), std::string("input"));
    CHECK_EQ(d.find("api")->as_number(), 1.0);
    const Value& settings = *d.find("settings");
    std::string keys;
    for (const Value& s : settings.items()) {
        keys += s.find("key")->as_string() + " ";
        // Every setting says what it is and whether it is live.
        CHECK(s.find("type") && s.find("label") && s.find("live") && s.find("default"));
        const std::string help = s.find("help") ? s.find("help")->as_string() : "";
        CHECK(!help.empty() && help.size() <= 400);
        // No em dashes in interface text.
        CHECK(help.find("\xe2\x80\x94") == std::string::npos);
    }
    CHECK_EQ(keys, std::string("device gain gain_mode lna_gain mixer_gain vga_gain attenuation agc_threshold preamp "
                               "bias_tee packing "));
    // A default of every setting passes the module's own validation.
    Value defaults = Value::object();
    for (const Value& s : settings.items())
        defaults.set(s.find("key")->as_string(), *s.find("default"));
    Value open = open_with("{}");
    open.set("settings", defaults);
    OpenRequest r;
    CHECK(!fern::parse_open(open, r));
    const Value& tuning = *d.find("tuning");
    CHECK_EQ(tuning.find("signal")->as_string(), std::string("iq"));
    CHECK(tuning.find("ranges")->items().size() <= 8);
    CHECK(tuning.find("rates")->items().size() <= 8);
    CHECK_EQ(tuning.find("rates")->items()[0].as_number(), 10000000.0);
}
