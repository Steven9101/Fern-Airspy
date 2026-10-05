// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <string>

#include "device.h"

namespace fern {

// The Backend on top of libusb, libairspy and libairspyhf.
class AirspyBackend : public Backend {
public:
    Enumeration enumerate() override;
    int open_one(uint64_t serial, std::unique_ptr<OneDevice>& out) override;
    int open_hf(uint64_t serial, std::unique_ptr<HfDevice>& out) override;
};

// e.g. "libairspy 1.0.12, libairspyhf 1.8.1 and libusb 1.0.30"
std::string library_versions();

}  // namespace fern
