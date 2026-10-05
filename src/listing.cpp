// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include "listing.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "log.h"
#include "receiver.h"

namespace fern {

namespace {

struct Shared {
    std::mutex mutex;
    std::condition_variable changed;
    bool enumerated = false;
    Enumeration enumeration;
    std::vector<json::Value> devices;
    bool finished = false;
};

std::string open_error(int r) {
    switch (r) {
    case usb_error::busy:
        return "in use by another program, such as spyserver, SDR++ or SDR# server, airspy_rx or another FernSDR "
               "band";
    case usb_error::access:
        return permission_advice();
    case usb_error::no_device:
        return "it disappeared while it was being opened";
    default:
        return std::string("could not be opened (") + usb_error_text(r) + ")";
    }
}

json::Value examine(Backend& backend, const Found& found, uint32_t index, size_t same_family) {
    json::Value d = json::Value::object();
    d.set("index", index);
    d.set("family", family_id(found.family));
    d.set("name", found.family == Family::one ? "Airspy R2 or Mini" : "Airspy HF+");
    if (found.has_serial)
        d.set("serial", serial_text(found.serial));
    if (!found.port.empty())
        d.set("port", found.port);
    if (found.open_error != 0 || !found.has_serial) {
        d.set("usable", false);
        d.set("error", found.open_error != 0 ? open_error(found.open_error)
                                             : std::string("reports no serial number; unplug it and plug it in again"));
        return d;
    }
    // The vendor libraries take serial 0 to mean the first device they find,
    // which among several may be another one.
    if (found.serial == 0 && same_family > 1) {
        d.set("usable", false);
        d.set("error", "reports the serial number 0, by which it cannot be told from the other Airspys");
        return d;
    }
    std::string firmware;
    std::vector<uint32_t> rates;
    if (found.family == Family::one) {
        std::unique_ptr<OneDevice> device;
        const int r = backend.open_one(found.serial, device);
        if (r != 0 || !device) {
            d.set("usable", false);
            d.set("error", "could not be opened: another program may hold it");
            return d;
        }
        if (device->version(firmware) != 0)
            firmware.clear();
        if (device->samplerates(rates) != 0)
            rates.clear();
        d.set("name", model_name(one_model(firmware)));
    } else {
        std::unique_ptr<HfDevice> device;
        const int r = backend.open_hf(found.serial, device);
        if (r != 0 || !device) {
            d.set("usable", false);
            d.set("error", "could not be opened: another program may hold it");
            return d;
        }
        uint32_t board = 0;
        if (device->board_id(board) != 0)
            board = 0;
        if (device->version(firmware) != 0)
            firmware.clear();
        if (device->samplerates(rates) != 0)
            rates.clear();
        d.set("name", model_name(hf_model(board)));
    }
    if (!firmware.empty())
        d.set("firmware", firmware);
    json::Value list = json::Value::array();
    for (uint32_t r : rates)
        list.push(r);
    // What this one radio tunes and at which rates, for FernSDR's setup: the
    // manifest's block covers both families at once, and an R2 and a Mini
    // share a USB id but not their rates.
    if (!rates.empty()) {
        json::Value ranges = json::Value::array();
        if (found.family == Family::one) {
            ranges.push(json::Value::array().push(24000000.0).push(1800000000.0));
        } else {
            ranges.push(json::Value::array().push(9000.0).push(31000000.0));
            ranges.push(json::Value::array().push(60000000.0).push(260000000.0));
        }
        json::Value tuning = json::Value::object();
        tuning.set("ranges", std::move(ranges));
        tuning.set("rates", list);
        tuning.set("signal", "iq");
        d.set("tuning", std::move(tuning));
    }
    d.set("rates", std::move(list));
    d.set("usable", true);
    return d;
}

void list_into(Backend& backend, Shared& shared) {
    Enumeration e = backend.enumerate();
    order_by_port(e.devices);
    {
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.enumeration = e;
        shared.enumerated = true;
    }
    shared.changed.notify_all();
    if (e.error == 0) {
        for (uint32_t i = 0; i < e.devices.size(); ++i) {
            size_t same_family = 0;
            for (const Found& f : e.devices)
                same_family += f.family == e.devices[i].family;
            json::Value d = examine(backend, e.devices[i], i, same_family);
            {
                std::lock_guard<std::mutex> lock(shared.mutex);
                shared.devices.push_back(std::move(d));
            }
            shared.changed.notify_all();
        }
    }
    {
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.finished = true;
    }
    shared.changed.notify_all();
}

}  // namespace

Listing list_devices(Backend& backend, std::chrono::milliseconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    auto shared = std::make_shared<Shared>();
    std::thread worker;
    try {
        worker = std::thread([shared, &backend] { list_into(backend, *shared); });
    } catch (const std::system_error&) {
        list_into(backend, *shared);
    }

    std::unique_lock<std::mutex> lock(shared->mutex);
    const bool finished = shared->changed.wait_until(lock, deadline, [&] { return shared->finished; });
    json::Value devices = json::Value::array();
    for (const json::Value& d : shared->devices)
        devices.push(d);
    json::Value report = json::Value::object();
    if (!shared->enumerated) {
        report.set("devices", std::move(devices));
        report.set("error", "the list of USB devices could not be read within the time limit");
    } else {
        for (size_t i = shared->devices.size(); i < shared->enumeration.devices.size(); ++i) {
            json::Value d = json::Value::object();
            d.set("index", i);
            d.set("usable", false);
            d.set("error", "did not answer within the time limit");
            devices.push(std::move(d));
        }
        report.set("devices", std::move(devices));
        if (shared->enumeration.error != 0)
            report.set("error", shared->enumeration.diagnostic);
    }
    lock.unlock();

    if (worker.joinable()) {
        if (finished)
            worker.join();
        else
            worker.detach();
    }
    return Listing{std::move(report), finished};
}

std::string report_text(const json::Value& report) {
    std::string text = json::serialize(report) + "\n";
    if (text.size() <= max_report_bytes)
        return text;

    std::vector<json::Value> kept;
    if (const json::Value* list = report.find("devices")) {
        for (const json::Value& d : list->items()) {
            json::Value slim = json::Value::object();
            for (const json::Member& m : d.members())
                if (m.key != "rates" && m.key != "firmware" && m.key != "tuning")
                    slim.set(m.key, m.value);
            kept.push_back(std::move(slim));
        }
    }
    for (;;) {
        json::Value out = json::Value::object();
        json::Value list = json::Value::array();
        for (const json::Value& d : kept)
            list.push(d);
        out.set("devices", std::move(list));
        for (const json::Member& m : report.members())
            if (m.key != "devices")
                out.set(m.key, m.value);
        text = json::serialize(out) + "\n";
        if (text.size() <= max_report_bytes || kept.empty())
            break;
        kept.pop_back();
    }
    log_line("the device list was shortened to fit into %zu bytes", max_report_bytes);
    return text;
}

}  // namespace fern
