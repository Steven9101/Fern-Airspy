// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Where an Airspy R2 or Mini really tunes.
//
// The firmware tunes the R820T's local oscillator to the frequency asked
// for plus the IF, half the I/Q rate, and libairspy shifts the IF down to
// 0 Hz exactly. The oscillator is a fractional-N PLL whose 16-bit fraction
// steps by the crystal over 65536 at the VCO, divided by 2 to 64 down to the
// LO: 381 Hz / 2 at 1 GHz with the R2's 25 MHz crystal, 381 Hz / 32 at
// 100 MHz. The firmware rounds to the nearest step and says nothing about
// the rest, so a band would sit up to 95 Hz beside the frequency asked for,
// further than FernSDR's 10 Hz allows.
//
// The module works out the step the firmware chooses, by the same integer
// arithmetic (common/r820t.c, r820t_set_pll(), in airspyone_firmware), and
// shifts the remainder out of the samples (convert.h). The firmware computes
// with its crystal frequency corrected by the calibration airspy_calibrate
// stores in the flash, when it runs from its own crystal; so does the
// module (calibrated_xtal()).
#pragma once

#include <cstdint>
#include <optional>

namespace fern {

// The VCO's range in the firmware; outside it the firmware leaves the PLL
// where it was, and the band would be received elsewhere.
constexpr uint64_t r820t_vco_min = 1770000000;
constexpr uint64_t r820t_vco_max = 3900000000;
// The LO frequencies a divider of 2 to 64 reaches from that VCO range.
constexpr uint32_t r820t_lo_min = 27656250;  // vco_min / 64, rounded up
constexpr uint32_t r820t_lo_max = 1950000000;

constexpr uint32_t r2_xtal_hz = 25000000;
constexpr uint32_t mini_xtal_hz = 24000000;

// The crystal frequency the firmware computes with after a stored
// correction, rounded as the firmware rounds it.
uint32_t calibrated_xtal(uint32_t xtal_hz, int32_t correction_ppb);

struct PllSetting {
    unsigned divider_shift = 0;  // the LO is the VCO over 2 << divider_shift
    uint32_t ratio = 0;          // VCO = xtal * ratio / 65536: 8 integer and 16 fraction bits
    double lo_hz = 0;            // where the LO lands
};

// The setting the firmware writes for an LO frequency, or nothing when the
// LO is outside what the PLL reaches.
std::optional<PllSetting> r820t_pll(uint32_t xtal_hz, uint32_t lo_hz);

struct OneTuning {
    uint32_t lo_request = 0;  // center + IF, what the firmware is asked for
    double lo_landed = 0;
    // Where the band's 0 Hz really is, less where it was asked to be:
    // what the samples must be shifted by, up.
    double offset = 0;
};

// For a centre and an I/Q rate of an R2 or Mini, whose IF is half the rate.
std::optional<OneTuning> one_tuning(uint32_t xtal_hz, uint32_t center_hz, uint32_t iq_rate);

}  // namespace fern
