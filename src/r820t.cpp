// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include "r820t.h"

namespace fern {

uint32_t calibrated_xtal(uint32_t xtal_hz, int32_t correction_ppb) {
    // C's division truncates toward zero, so the half added for rounding
    // rounds negative corrections differently from positive ones; the
    // firmware does exactly this.
    const int64_t billion = 1000000000;
    const int64_t delta = (static_cast<int64_t>(xtal_hz) * correction_ppb + billion / 2) / billion;
    return static_cast<uint32_t>(static_cast<int64_t>(xtal_hz) + delta);
}

std::optional<PllSetting> r820t_pll(uint32_t xtal_hz, uint32_t lo_hz) {
    // The smallest divider that puts the VCO in its range.
    unsigned shift = 0;
    uint64_t vco = 0;
    bool found = false;
    for (; shift <= 5; ++shift) {
        vco = static_cast<uint64_t>(lo_hz) << (shift + 1);
        if (vco >= r820t_vco_min && vco <= r820t_vco_max) {
            found = true;
            break;
        }
    }
    if (!found)
        return std::nullopt;

    // The ratio VCO / xtal to 16 fraction bits, rounded by adding half a
    // step first, then found bit by bit against the halved crystal
    // frequency, shifted down one bit per step with the bits it loses.
    const uint32_t half = xtal_hz >> 1;
    uint64_t rest = vco + (half >> 16);
    uint64_t divisor = static_cast<uint64_t>(half) << 8;
    uint32_t ratio = 0;
    for (int bit = 23; bit >= 0 && rest > 0; --bit) {
        if (rest >= divisor) {
            ratio |= 1u << bit;
            rest -= divisor;
        }
        divisor >>= 1;
    }

    PllSetting s;
    s.divider_shift = shift;
    s.ratio = ratio;
    s.lo_hz = static_cast<double>(xtal_hz) * ratio / 65536.0 / static_cast<double>(2u << shift);
    return s;
}

std::optional<OneTuning> one_tuning(uint32_t xtal_hz, uint32_t center_hz, uint32_t iq_rate) {
    const uint64_t lo = static_cast<uint64_t>(center_hz) + iq_rate / 2;
    if (lo < r820t_lo_min || lo > r820t_lo_max)
        return std::nullopt;
    const auto pll = r820t_pll(xtal_hz, static_cast<uint32_t>(lo));
    if (!pll)
        return std::nullopt;
    OneTuning t;
    t.lo_request = static_cast<uint32_t>(lo);
    t.lo_landed = pll->lo_hz;
    t.offset = pll->lo_hz - static_cast<double>(lo);
    return t;
}

}  // namespace fern
