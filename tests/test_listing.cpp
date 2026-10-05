// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include <chrono>
#include <string>
#include <thread>

#include "fake_backend.h"
#include "json.h"
#include "listing.h"
#include "receiver.h"
#include "test.h"

using fern::json::Value;

TEST(listing_without_devices) {
    fake::Backend backend;
    const fern::Listing l = fern::list_devices(backend, std::chrono::milliseconds(2000));
    CHECK(l.complete);
    CHECK_EQ(fern::report_text(l.report), std::string("{\"devices\":[]}\n"));
}

TEST(listing_describes_every_airspy) {
    fake::Spec r2;
    r2.serial = 0x26D464DC2A8E7A2Bull;
    fake::Spec mini;
    mini.serial = 0x2;
    mini.port = "2-1.4";
    mini.version = "AirSpy MINI v1.0.0-rc10-6-g4008185 2020-05-08";
    mini.one_rates = {6000000, 3000000};
    fake::Spec hf;
    hf.family = fern::Family::hf;
    hf.serial = 0x3652A98081830F1Eull;
    hf.port = "1-3";
    fake::Spec locked;
    locked.family = fern::Family::hf;
    locked.serial = 0x4;
    locked.port = "1-4";
    locked.open_error = fern::usb_error::access;
    fake::Spec busy;
    busy.serial = 0x5;
    busy.port = "1-5";
    busy.open_error = fern::usb_error::busy;
    fake::Spec raced;
    raced.serial = 0x6;
    raced.port = "1-6";
    raced.vendor_open = -5;
    fake::Backend backend({r2, mini, hf, locked, busy, raced});

    const fern::Listing l = fern::list_devices(backend, std::chrono::milliseconds(2000));
    CHECK(l.complete);
    const std::string text = fern::report_text(l.report);
    Value parsed;
    std::string error;
    REQUIRE(fern::json::parse(text, parsed, error));
    const Value* devices = parsed.find("devices");
    REQUIRE(devices && devices->items().size() == 6);
    CHECK_EQ(fern::json::serialize(devices->items()[0]),
             std::string("{\"index\":0,\"family\":\"airspy\",\"name\":\"Airspy R2\",\"serial\":\"26D464DC2A8E7A2B\","
                         "\"port\":\"1-2\",\"firmware\":\"AirSpy NOS v1.0.0-rc10-6-g4008185 2020-05-08\","
                         "\"rates\":[10000000,2500000],\"usable\":true}"));
    CHECK_EQ(fern::json::serialize(devices->items()[1]),
             std::string("{\"index\":1,\"family\":\"airspy\",\"name\":\"Airspy Mini\",\"serial\":\"0000000000000002\","
                         "\"port\":\"2-1.4\",\"firmware\":\"AirSpy MINI v1.0.0-rc10-6-g4008185 2020-05-08\","
                         "\"rates\":[6000000,3000000],\"usable\":true}"));
    CHECK_EQ(fern::json::serialize(devices->items()[2]),
             std::string("{\"index\":2,\"family\":\"airspyhf\",\"name\":\"Airspy HF+ Discovery\","
                         "\"serial\":\"3652A98081830F1E\",\"port\":\"1-3\",\"firmware\":\"R3.0.7-CD\","
                         "\"rates\":[912000,768000,456000,384000,256000,192000],\"usable\":true}"));
    // One the receiver may not open cannot tell its serial, and says what to do.
    CHECK_EQ(fern::json::serialize(devices->items()[3]),
             std::string("{\"index\":3,\"family\":\"airspyhf\",\"name\":\"Airspy HF+\",\"port\":\"1-4\","
                         "\"usable\":false,\"error\":\"") +
                 fern::permission_advice() + "\"}");
    CHECK_HAS(fern::json::serialize(devices->items()[4]),
              "\"usable\":false,\"error\":\"in use by another program, such as spyserver");
    CHECK_HAS(fern::json::serialize(devices->items()[5]),
              "\"usable\":false,\"error\":\"could not be opened: another program may hold it\"");
    // Every opened device was closed again.
    CHECK(backend.last()->closed);
}

TEST(listing_reports_usb_trouble) {
    fake::Backend backend;
    backend.enumeration_error = -99;
    backend.enumeration_diagnostic = "/dev/bus/usb does not exist";
    const fern::Listing l = fern::list_devices(backend, std::chrono::milliseconds(2000));
    CHECK_EQ(fern::report_text(l.report),
             std::string("{\"devices\":[],\"error\":\"/dev/bus/usb does not exist\"}\n"));
}

TEST(listing_keeps_its_time_limit) {
    fake::Spec quick;
    quick.serial = 0x2;
    fake::Spec slow;
    slow.serial = 0x3;
    slow.open_delay_ms = 600;
    fake::Backend backend({quick, slow});
    const auto start = std::chrono::steady_clock::now();
    const fern::Listing l = fern::list_devices(backend, std::chrono::milliseconds(200));
    const auto took = std::chrono::steady_clock::now() - start;
    CHECK(took < std::chrono::milliseconds(500));
    CHECK(!l.complete);
    const std::string text = fern::report_text(l.report);
    CHECK_HAS(text, "\"serial\":\"0000000000000002\"");
    CHECK_HAS(text, "{\"index\":1,\"usable\":false,\"error\":\"did not answer within the time limit\"}");
    // The worker still uses the backend; let it finish before the backend goes.
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
}

TEST(listing_fits_into_64_kib) {
    // About 250 bytes per device with firmware and rates and 120 without:
    // first those go, then the devices that still do not fit.
    std::vector<fake::Spec> many(800);
    for (size_t i = 0; i < many.size(); ++i)
        many[i].serial = 0x1000 + i;
    fake::Backend backend(many);
    const fern::Listing l = fern::list_devices(backend, std::chrono::milliseconds(5000));
    CHECK(l.complete);
    const std::string text = fern::report_text(l.report);
    CHECK(text.size() <= fern::max_report_bytes);
    Value parsed;
    std::string error;
    REQUIRE(fern::json::parse(text, parsed, error));
    const size_t kept = parsed.find("devices")->items().size();
    CHECK(kept > 400);
    CHECK(kept < 800);
    CHECK(parsed.find("devices")->items()[0].find("rates") == nullptr);
    CHECK_EQ(parsed.find("devices")->items()[kept - 1].find("index")->as_number(), double(kept - 1));
}
