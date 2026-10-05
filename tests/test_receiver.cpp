// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "fake_backend.h"
#include "r820t.h"
#include "receiver.h"
#include "test.h"

using fern::ErrorCode;
using fern::Family;
using fern::OpenRequest;

namespace {

fake::Spec r2(uint64_t serial = 0x1111) {
    fake::Spec s;
    s.serial = serial;
    return s;
}

fake::Spec mini(uint64_t serial = 0x2222) {
    fake::Spec s;
    s.serial = serial;
    s.version = "AirSpy MINI v1.0.0-rc10-6-g4008185 2020-05-08";
    s.one_rates = {6000000, 3000000};
    return s;
}

fake::Spec hf(uint64_t serial = 0x3333) {
    fake::Spec s;
    s.family = Family::hf;
    s.serial = serial;
    s.port = "1-3";
    return s;
}

OpenRequest request(uint32_t rate = 10000000, uint32_t center = 100000000) {
    OpenRequest r;
    r.sample_rate = rate;
    r.center = center;
    return r;
}

bool called(const std::shared_ptr<fake::State>& st, const std::string& call) {
    std::lock_guard<std::mutex> lock(st->mutex);
    return std::find(st->calls.begin(), st->calls.end(), call) != st->calls.end();
}

size_t position(const std::shared_ptr<fake::State>& st, const std::string& call) {
    std::lock_guard<std::mutex> lock(st->mutex);
    return static_cast<size_t>(std::find(st->calls.begin(), st->calls.end(), call) - st->calls.begin());
}

}  // namespace

TEST(receiver_says_when_no_airspy_is_plugged_in) {
    fake::Backend backend;
    fern::Receiver rx(backend);
    auto f = rx.open(request());
    REQUIRE(f);
    CHECK(f->code == ErrorCode::no_device);
    CHECK_HAS(f->message, "no Airspy is plugged in");
    CHECK_HAS(f->message, "1d50:60a1");
    CHECK_HAS(f->message, "03eb:800c");
    OpenRequest r = request();
    r.settings.device.kind = fern::DeviceSelector::Kind::serial;
    r.settings.device.serial = 0xABC;
    f = rx.open(r);
    REQUIRE(f);
    CHECK_HAS(f->message, "no Airspy with serial 0000000000000ABC is plugged in, nor any other");
}

TEST(receiver_reports_usb_that_does_not_start) {
    fake::Backend backend({r2()});
    backend.enumeration_error = -99;
    backend.enumeration_diagnostic = "USB devices are not visible to this process";
    fern::Receiver rx(backend);
    const auto f = rx.open(request());
    REQUIRE(f);
    CHECK(f->code == ErrorCode::usb);
    CHECK_EQ(f->message, std::string("USB devices are not visible to this process"));
}

TEST(receiver_picks_the_only_one_and_asks_among_several) {
    fake::Backend one({hf()});
    fern::Receiver rx(one);
    REQUIRE(!rx.open(request(768000, 7100000)));
    CHECK(rx.info().family == Family::hf);
    rx.close();

    fake::Backend two({r2(0x26D464DC2A8E7A2Bull), hf(0x3652A98081830F1Eull)});
    fern::Receiver rx2(two);
    const auto f = rx2.open(request());
    REQUIRE(f);
    CHECK(f->code == ErrorCode::invalid);
    CHECK_HAS(f->message, "2 Airspys are plugged in (index 0, Airspy R2 or Mini with serial 26D464DC2A8E7A2B at "
                          "USB port 1-2; index 1, Airspy HF+ with serial 3652A98081830F1E at USB port 1-3)");
    CHECK_HAS(f->message, "set module.device = serial:<serial>");

    OpenRequest r = request(768000, 7100000);
    r.settings.device.kind = fern::DeviceSelector::Kind::serial;
    r.settings.device.serial = 0x3652A98081830F1Eull;
    REQUIRE(!rx2.open(r));
    CHECK(rx2.info().family == Family::hf);
    CHECK_EQ(two.last()->hf_freq, 7100000.0);

    r = request();
    r.settings.device.kind = fern::DeviceSelector::Kind::index;
    r.settings.device.index = 0;
    REQUIRE(!rx2.open(r));
    CHECK(rx2.info().family == Family::one);
    r.settings.device.index = 2;
    const auto g = rx2.open(r);
    REQUIRE(g);
    CHECK(g->code == ErrorCode::no_device);
    CHECK_HAS(g->message, "there is no Airspy at index 2");
}

TEST(receiver_tells_a_beginner_about_permissions) {
    fake::Spec locked = r2();
    locked.open_error = fern::usb_error::access;
    fake::Backend backend({locked});
    fern::Receiver rx(backend);
    auto f = rx.open(request());
    REQUIRE(f);
    CHECK(f->code == ErrorCode::usb);
    CHECK_EQ(f->message, fern::permission_advice());
    CHECK_HAS(f->message, "Run FernSDR's install command once more");
    CHECK(f->message.find("\xe2\x80\x94") == std::string::npos);
    // Asked for by serial, the locked one cannot say its serial; that is
    // the likely reason, so the advice comes with the inventory.
    OpenRequest r = request();
    r.settings.device.kind = fern::DeviceSelector::Kind::serial;
    r.settings.device.serial = 0x1111;
    f = rx.open(r);
    REQUIRE(f);
    CHECK(f->code == ErrorCode::usb);
    CHECK_HAS(f->message, "that could not be opened (permission denied)");
    CHECK_HAS(f->message, fern::permission_advice());
}

TEST(receiver_says_who_else_may_hold_it) {
    fake::Spec held = hf();
    held.open_error = fern::usb_error::busy;
    fake::Backend backend({held});
    fern::Receiver rx(backend);
    auto f = rx.open(request(768000, 7100000));
    REQUIRE(f);
    CHECK(f->code == ErrorCode::busy);
    CHECK_HAS(f->message, "the Airspy HF+ at USB port 1-3 is in use by another program, such as spyserver");
    // Taken between the scan and the library's open.
    fake::Spec raced = r2();
    raced.vendor_open = -5;
    fake::Backend backend2({raced});
    fern::Receiver rx2(backend2);
    f = rx2.open(request());
    REQUIRE(f);
    CHECK(f->code == ErrorCode::busy);
    CHECK_HAS(f->message, "another program may have taken it just now");
}

TEST(receiver_opens_an_r2) {
    fake::Backend backend({r2()});
    fern::Receiver rx(backend);
    REQUIRE(!rx.open(request()));
    const auto st = backend.last();
    CHECK(rx.info().model == fern::Model::r2);
    CHECK_EQ(rx.info().xtal, 25000000u);
    CHECK_EQ(std::string(rx.format()), std::string("s16"));
    CHECK_EQ(rx.sample_bytes(), size_t(4));
    CHECK_EQ(st->rate_index, 0u);
    CHECK_EQ(st->freq, 100000000u);
    // The rate first: the firmware adds the IF of the rate set.
    CHECK(position(st, "set_samplerate 0") < position(st, "set_freq 100000000"));
    // 10 Msps packs, to spare the USB port a quarter.
    CHECK(st->packing);
    CHECK(rx.effective().packing);
    CHECK_EQ(st->gain, std::string("linearity 10"));
    CHECK(rx.controls_gain());
    CHECK(std::fabs(rx.effective().pll_offset - (-4.76837158203125)) < 1e-6);
    const std::string settings = fern::json::serialize(rx.settings_json());
    CHECK_EQ(settings, std::string("{\"gain\":\"auto\",\"gain_now\":\"linearity 10\",\"gain_mode\":\"linearity\","
                                   "\"lna_gain\":1,\"mixer_gain\":5,\"vga_gain\":5,\"packing\":\"on\","
                                   "\"pll_offset\":-4.77,\"bias_tee\":false}"));
    CHECK_EQ(fern::json::serialize(rx.device_json()),
             std::string("{\"name\":\"Airspy R2\",\"family\":\"airspy\",\"serial\":\"0000000000001111\","
                         "\"firmware\":\"AirSpy NOS v1.0.0-rc10-6-g4008185 2020-05-08\",\"port\":\"1-2\","
                         "\"index\":0}"));
    rx.close();
    CHECK(st->closed);
    // 2.5 Msps does not pack unless asked.
    REQUIRE(!rx.open(request(2500000, 145000000)));
    CHECK(!backend.last()->packing);
    CHECK_EQ(backend.last()->rate_index, 1u);
    OpenRequest r = request(2500000, 145000000);
    r.settings.packing = fern::Packing::on;
    REQUIRE(!rx.open(r));
    CHECK(backend.last()->packing);
    r = request();
    r.settings.packing = fern::Packing::off;
    REQUIRE(!rx.open(r));
    CHECK(!backend.last()->packing);
}

TEST(receiver_opens_a_mini_with_its_own_crystal) {
    fake::Backend backend({mini()});
    fern::Receiver rx(backend);
    REQUIRE(!rx.open(request(6000000, 100000000)));
    CHECK(rx.info().model == fern::Model::mini);
    CHECK_EQ(rx.info().xtal, 24000000u);
    const auto t = fern::one_tuning(24000000, 100000000, 6000000);
    REQUIRE(t);
    CHECK_EQ(rx.effective().pll_offset, t->offset);
    CHECK(backend.last()->packing);
    CHECK_EQ(fern::json::serialize(rx.device_json()).find("\"name\":\"Airspy Mini\"") != std::string::npos, true);
    // The R2's rates are not the Mini's.
    const auto f = rx.open(request(10000000, 100000000));
    REQUIRE(f);
    CHECK(f->code == ErrorCode::invalid);
    CHECK_HAS(f->message, "sample_rate 10000000 is not one the Airspy Mini offers; it runs at 6000000 and 3000000 "
                          "samples/s");
}

TEST(receiver_refuses_centres_the_pll_cannot_reach) {
    fake::Backend backend({r2()});
    fern::Receiver rx(backend);
    auto f = rx.open(request(2500000, 24000000));
    REQUIRE(f);
    CHECK(f->code == ErrorCode::invalid);
    CHECK_HAS(f->message, "at 2500000 samples/s the Airspy R2 tunes centres from 26406250 to 1948750000 Hz, and "
                          "center is 24000000 Hz");
    // Nothing stays open after a refusal.
    CHECK(backend.last()->closed);
    CHECK(!rx.is_open());
    f = rx.open(request(10000000, 1946000000));
    REQUIRE(f);
    CHECK_HAS(f->message, "from 22656250 to 1945000000 Hz");
    CHECK(!rx.open(request(10000000, 23000000)));
}

TEST(receiver_tunes_with_a_stored_correction_as_the_firmware_does) {
    fake::Spec calibrated = r2();
    calibrated.calibration = true;
    calibrated.calibration_ppb = 1200;
    fake::Backend backend({calibrated});
    fern::Receiver rx(backend);
    REQUIRE(!rx.open(request(10000000, 1000000000)));
    CHECK(rx.info().calibration_stored);
    CHECK(rx.info().calibration_applied);
    CHECK_EQ(rx.info().xtal, 25000030u);
    const auto with = fern::one_tuning(25000030, 1000000000, 10000000);
    REQUIRE(with);
    CHECK_EQ(rx.effective().pll_offset, with->offset);
    const auto without = fern::one_tuning(25000000, 1000000000, 10000000);
    REQUIRE(without);
    CHECK(std::fabs(with->offset - without->offset) > 1);

    // With an external clock the firmware leaves the correction alone.
    calibrated.clock_lost = false;
    fake::Backend backend2({calibrated});
    fern::Receiver rx2(backend2);
    REQUIRE(!rx2.open(request(10000000, 1000000000)));
    CHECK(rx2.info().calibration_stored);
    CHECK(!rx2.info().calibration_applied);
    CHECK_EQ(rx2.info().xtal, 25000000u);
    CHECK_EQ(rx2.effective().pll_offset, without->offset);

    // A Mini has no external clock input.
    fake::Spec mini_calibrated = mini();
    mini_calibrated.calibration = true;
    mini_calibrated.calibration_ppb = -500;
    mini_calibrated.clock_lost = false;
    fake::Backend backend3({mini_calibrated});
    fern::Receiver rx3(backend3);
    REQUIRE(!rx3.open(request(6000000, 433000000)));
    CHECK(rx3.info().calibration_applied);
    // -11.5 Hz, truncated toward zero after adding the half.
    CHECK_EQ(rx3.info().xtal, 23999989u);

    // A flash that cannot be read counts as none.
    fake::Spec unreadable = r2();
    unreadable.calibration = true;
    unreadable.calibration_ppb = 1200;
    unreadable.fail_call = "read_calibration";
    fake::Backend backend4({unreadable});
    fern::Receiver rx4(backend4);
    REQUIRE(!rx4.open(request(10000000, 1000000000)));
    CHECK(!rx4.info().calibration_stored);
    CHECK_EQ(rx4.info().xtal, 25000000u);
}

TEST(receiver_sets_every_r2_gain_mode) {
    fake::Backend backend({r2()});
    fern::Receiver rx(backend);
    OpenRequest r = request();
    r.settings.gain = fern::GainSetting::of_preset(13);
    r.settings.gain_mode = fern::GainLadder::sensitivity;
    REQUIRE(!rx.open(r));
    CHECK_EQ(backend.last()->gain, std::string("sensitivity 13"));
    CHECK(!rx.controls_gain());
    CHECK_EQ(fern::json::serialize(fern::gain_json(rx.effective().gain)), std::string("13"));

    r.settings.gain = fern::GainSetting::of(fern::GainSetting::Mode::tuner);
    r.settings.vga_gain = 9;
    REQUIRE(!rx.open(r));
    auto st = backend.last();
    CHECK(st->lna_agc && st->mixer_agc);
    CHECK_EQ(st->vga, 9);

    r.settings.gain = fern::GainSetting::of(fern::GainSetting::Mode::manual);
    r.settings.lna_gain = 12;
    r.settings.mixer_gain = 3;
    REQUIRE(!rx.open(r));
    st = backend.last();
    CHECK(!st->lna_agc && !st->mixer_agc);
    CHECK_EQ(st->gain, std::string("manual 12/3/9"));
}

TEST(receiver_refuses_settings_of_the_other_family) {
    fake::Backend backend({r2(), hf()});
    fern::Receiver rx(backend);
    OpenRequest r = request();
    r.settings.device.kind = fern::DeviceSelector::Kind::index;
    r.settings.device.index = 0;
    r.settings.attenuation = 6;
    auto f = rx.open(r);
    REQUIRE(f);
    CHECK(f->code == ErrorCode::invalid);
    CHECK_HAS(f->message, "module.attenuation is for the Airspy HF+ only; this is an Airspy R2. Remove it");
    r.settings.attenuation = 0;
    r.settings.preamp = true;
    CHECK_HAS(rx.open(r)->message, "module.preamp is for the Airspy HF+ only");
    r.settings.preamp = false;
    r.settings.agc_threshold_high = true;
    CHECK_HAS(rx.open(r)->message, "module.agc_threshold is for the Airspy HF+ only");
    // Values at their defaults are no trouble: a form may write them all.
    r.settings.agc_threshold_high = false;
    CHECK(!rx.open(r));

    OpenRequest h = request(768000, 7100000);
    h.settings.device.kind = fern::DeviceSelector::Kind::index;
    h.settings.device.index = 1;
    h.settings.gain = fern::GainSetting::of_preset(5);
    CHECK_HAS(rx.open(h)->message, "module.gain = 5 is for the Airspy R2 and Mini; an HF+ takes auto (its AGC) or "
                                   "manual (attenuation)");
    h.settings.gain = fern::GainSetting::of(fern::GainSetting::Mode::tuner);
    CHECK_HAS(rx.open(h)->message, "module.gain = \"tuner\" is for the Airspy R2 and Mini");
    h.settings.gain = fern::GainSetting::of(fern::GainSetting::Mode::automatic);
    h.settings.gain_mode = fern::GainLadder::sensitivity;
    CHECK_HAS(rx.open(h)->message, "module.gain_mode is for the Airspy R2 and Mini only");
    h.settings.gain_mode = fern::GainLadder::linearity;
    h.settings.vga_gain = 7;
    CHECK_HAS(rx.open(h)->message, "module.lna_gain, mixer_gain and vga_gain are for the Airspy R2 and Mini only");
    h.settings.vga_gain = 5;
    h.settings.packing = fern::Packing::on;
    CHECK_HAS(rx.open(h)->message, "module.packing is for the Airspy R2 and Mini only");
    h.settings.packing = fern::Packing::automatic;
    CHECK(!rx.open(h));
}

TEST(receiver_opens_an_hf_plus) {
    fake::Backend backend({hf()});
    fern::Receiver rx(backend);
    REQUIRE(!rx.open(request(768000, 7100000)));
    const auto st = backend.last();
    CHECK(rx.info().model == fern::Model::hf_discovery);
    CHECK_EQ(std::string(rx.format()), std::string("f32"));
    CHECK_EQ(rx.sample_bytes(), size_t(8));
    CHECK_EQ(st->rate, 768000u);
    CHECK_EQ(st->hf_freq, 7100000.0);
    CHECK(position(st, "set_samplerate 768000") < position(st, "set_freq 7100000"));
    CHECK(st->hf_agc);
    CHECK(!st->hf_agc_high);
    CHECK(!st->hf_lna);
    CHECK(!rx.controls_gain());
    // No bias tee on this one: nothing is sent to it.
    CHECK(!called(st, "set_bias_tee 0"));
    CHECK_EQ(fern::json::serialize(rx.settings_json()),
             std::string("{\"gain\":\"auto\",\"attenuation\":0,\"agc_threshold\":\"low\",\"preamp\":false,"
                         "\"bias_tee\":false}"));
    CHECK_EQ(fern::json::serialize(rx.device_json()),
             std::string("{\"name\":\"Airspy HF+ Discovery\",\"family\":\"airspyhf\",\"serial\":\"0000000000003333\","
                         "\"firmware\":\"R3.0.7-CD\",\"port\":\"1-3\",\"index\":0}"));

    OpenRequest r = request(912000, 145000000);
    r.settings.gain = fern::GainSetting::of(fern::GainSetting::Mode::manual);
    r.settings.attenuation = 10;
    r.settings.preamp = true;
    REQUIRE(!rx.open(r));
    const auto st2 = backend.last();
    CHECK(!st2->hf_agc);
    CHECK_EQ(st2->att, 12.0f);
    CHECK_EQ(rx.effective().attenuation, 12.0);
    CHECK(st2->hf_lna);
}

TEST(receiver_refuses_what_an_hf_plus_cannot_do) {
    fake::Backend backend({hf()});
    fern::Receiver rx(backend);
    auto f = rx.open(request(10000000, 7100000));
    REQUIRE(f);
    CHECK_HAS(f->message, "sample_rate 10000000 is not one the Airspy HF+ Discovery offers; it runs at 912000, "
                          "768000, 456000, 384000, 256000 and 192000 samples/s");
    f = rx.open(request(768000, 433000000));
    REQUIRE(f);
    CHECK_HAS(f->message, "receives up to 31 MHz and from 60 to 260 MHz");
    OpenRequest r = request(768000, 7100000);
    r.settings.bias_tee = true;
    f = rx.open(r);
    REQUIRE(f);
    CHECK_HAS(f->message, "has no bias tee its firmware can switch");
    // One with a bias tee switches it, and off again on closing.
    fake::Spec with_tee = hf();
    with_tee.bias_tees = 1;
    with_tee.board_id = 1;
    fake::Backend backend2({with_tee});
    fern::Receiver rx2(backend2);
    REQUIRE(!rx2.open(r));
    CHECK(rx2.info().model == fern::Model::hf_plus);
    CHECK(backend2.last()->bias_tee);
    rx2.close();
    CHECK(!backend2.last()->bias_tee);
}

TEST(receiver_applies_live_changes_to_an_r2) {
    fake::Backend backend({r2()});
    fern::Receiver rx(backend);
    REQUIRE(!rx.open(request()));
    const auto st = backend.last();
    fern::LiveChange c;
    c.gain = fern::GainSetting::of_preset(4);
    REQUIRE(!rx.apply(c));
    CHECK_EQ(st->gain, std::string("linearity 4"));
    CHECK_EQ(fern::json::serialize(rx.settings_json(c)), std::string("{\"gain\":4}"));
    // Stages without gain = manual are kept for later.
    fern::LiveChange d;
    d.lna_gain = 11;
    REQUIRE(!rx.apply(d));
    CHECK_EQ(st->gain, std::string("linearity 4"));
    CHECK_EQ(fern::json::serialize(rx.settings_json(d)), std::string("{\"lna_gain\":11}"));
    fern::LiveChange e;
    e.gain = fern::GainSetting::of(fern::GainSetting::Mode::manual);
    REQUIRE(!rx.apply(e));
    CHECK_EQ(st->gain, std::string("manual 11/5/5"));
    fern::LiveChange g;
    g.vga_gain = 12;
    REQUIRE(!rx.apply(g));
    CHECK_EQ(st->gain, std::string("manual 11/5/12"));
    // Back to auto: from the middle of the ladder again.
    fern::LiveChange h;
    h.gain = fern::GainSetting::of(fern::GainSetting::Mode::automatic);
    h.gain_mode = fern::GainLadder::sensitivity;
    REQUIRE(!rx.apply(h));
    CHECK_EQ(st->gain, std::string("sensitivity 10"));
    CHECK_EQ(fern::json::serialize(rx.settings_json(h)),
             std::string("{\"gain\":\"auto\",\"gain_now\":\"sensitivity 10\",\"gain_mode\":\"sensitivity\"}"));
    REQUIRE(!rx.set_preset(14));
    CHECK_EQ(st->gain, std::string("sensitivity 14"));
    // Asking for auto again keeps what the control found.
    fern::LiveChange again;
    again.gain = fern::GainSetting::of(fern::GainSetting::Mode::automatic);
    REQUIRE(!rx.apply(again));
    CHECK_EQ(st->gain, std::string("sensitivity 14"));
    fern::LiveChange bias;
    bias.bias_tee = true;
    REQUIRE(!rx.apply(bias));
    CHECK(st->bias_tee);
    fern::LiveChange wrong;
    wrong.attenuation = 6;
    const auto f = rx.apply(wrong);
    REQUIRE(f);
    CHECK_HAS(f->message, "module.attenuation is for the Airspy HF+ only");
    rx.close();
    CHECK(!st->bias_tee);
    CHECK(st->closed);
}

TEST(receiver_applies_live_changes_to_an_hf_plus) {
    fake::Backend backend({hf()});
    fern::Receiver rx(backend);
    REQUIRE(!rx.open(request(768000, 7100000)));
    const auto st = backend.last();
    fern::LiveChange c;
    c.gain = fern::GainSetting::of(fern::GainSetting::Mode::manual);
    c.attenuation = 18;
    REQUIRE(!rx.apply(c));
    CHECK(!st->hf_agc);
    CHECK_EQ(st->att, 18.0f);
    CHECK_EQ(fern::json::serialize(rx.settings_json(c)), std::string("{\"gain\":\"manual\",\"attenuation\":18}"));
    fern::LiveChange d;
    d.attenuation = 47;
    REQUIRE(!rx.apply(d));
    CHECK_EQ(st->att, 48.0f);
    fern::LiveChange e;
    e.gain = fern::GainSetting::of(fern::GainSetting::Mode::automatic);
    e.agc_threshold_high = true;
    e.preamp = true;
    REQUIRE(!rx.apply(e));
    CHECK(st->hf_agc);
    CHECK(st->hf_agc_high);
    CHECK(st->hf_lna);
    fern::LiveChange wrong;
    wrong.gain = fern::GainSetting::of_preset(3);
    CHECK(rx.apply(wrong));
    CHECK(st->hf_agc);
}

TEST(receiver_reports_a_failing_usb_call) {
    fake::Spec spec = r2();
    spec.fail_call = "set_freq";
    fake::Backend backend({spec});
    fern::Receiver rx(backend);
    const auto f = rx.open(request());
    REQUIRE(f);
    CHECK(f->code == ErrorCode::usb);
    CHECK_HAS(f->message, "tuning to 100000000 Hz failed (libusb error); check the USB cable");
    CHECK(backend.last()->closed);
}

TEST(receiver_close_by_gives_up_on_a_slow_close) {
    fake::Spec slow = r2();
    slow.close_delay_ms = 400;
    fake::Backend backend({slow});
    fern::Receiver rx(backend);
    REQUIRE(!rx.open(request()));
    const auto st = backend.last();
    CHECK(!rx.close_by(std::chrono::steady_clock::now() + std::chrono::milliseconds(50)));
    // It finishes in its own thread.
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    std::lock_guard<std::mutex> lock(st->mutex);
    CHECK(st->closed);
}
