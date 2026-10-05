// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// The R820T PLL as the Airspy firmware sets it, against landings worked out
// with exact fractions (the nearest of the PLL's steps) and by hand.
#include <cmath>
#include <cstdint>
#include <random>

#include "fake_backend.h"
#include "r820t.h"
#include "test.h"

using fern::r820t_pll;

TEST(r820t_lands_on_the_nearest_step) {
    // 100 MHz at 10 Msps: the LO goes to 105 MHz, the VCO to 32 times that,
    // 3.36 GHz, which is 134.4 times the 25 MHz crystal; 0.4 of 65536 is
    // 26214.4, so the fraction is 0x6666 and the LO 4.77 Hz low.
    auto pll = r820t_pll(25000000, 105000000);
    REQUIRE(pll);
    CHECK_EQ(pll->divider_shift, 4u);
    CHECK_EQ(pll->ratio, 0x866666u);
    CHECK(std::fabs(pll->lo_hz - 104999995.23162842) < 1e-6);

    // At 1 GHz the divider is 2 and a step of the fraction is 191 Hz at the
    // LO: 76 Hz off, which the module must shift out.
    pll = r820t_pll(25000000, 1005000000);
    REQUIRE(pll);
    CHECK_EQ(pll->divider_shift, 0u);
    CHECK_EQ(pll->ratio, 0x506666u);
    CHECK(std::fabs(pll->lo_hz - 1004999923.7060547) < 1e-6);

    // The Mini's 24 MHz crystal.
    pll = r820t_pll(24000000, 103000000);
    REQUIRE(pll);
    CHECK_EQ(pll->ratio, 0x895555u);
    CHECK(std::fabs(pll->lo_hz - 102999996.18530273) < 1e-6);

    // A whole multiple of the crystal needs no fraction.
    pll = r820t_pll(25000000, 100000000);
    REQUIRE(pll);
    CHECK_EQ(pll->ratio, 128u << 16);
    CHECK_EQ(pll->lo_hz, 100000000.0);
}

TEST(r820t_refuses_what_the_vco_cannot_reach) {
    CHECK(!r820t_pll(25000000, fern::r820t_lo_min - 1));
    CHECK(r820t_pll(25000000, fern::r820t_lo_min));
    CHECK(r820t_pll(25000000, fern::r820t_lo_max));
    CHECK(!r820t_pll(25000000, fern::r820t_lo_max + 1));
    CHECK(!r820t_pll(25000000, 0));
    CHECK(!r820t_pll(25000000, 4000000000u));
    // Between dividers there is no gap: 3.9 GHz / 2^n reaches past 1.77 GHz
    // / 2^(n-1) for every n.
    for (unsigned shift = 1; shift <= 5; ++shift) {
        const uint32_t edge = static_cast<uint32_t>(fern::r820t_vco_min >> shift);
        CHECK(r820t_pll(25000000, edge));
        CHECK(r820t_pll(25000000, edge - 1));
    }
}

// The firmware divides bit by bit with a divisor it halves each time,
// dropping a bit now and then; that lands on the nearest step nearly always
// and never more than one step off.
TEST(r820t_bitwise_division_matches_rounding) {
    std::mt19937 random(12345);
    std::uniform_int_distribution<uint32_t> lo(fern::r820t_lo_min, fern::r820t_lo_max);
    int differ = 0;
    const int tries = 20000;
    for (int i = 0; i < tries; ++i) {
        const uint32_t xtal = i % 2 ? 25000000 : 24000000;
        const uint32_t f = lo(random);
        const auto pll = r820t_pll(xtal, f);
        REQUIRE(pll);
        const double exact = fake::landed_lo(xtal, f);
        const double step = xtal / 65536.0 / (2u << pll->divider_shift);
        CHECK(std::fabs(pll->lo_hz - exact) <= step * 1.0001);
        CHECK(std::fabs(pll->lo_hz - f) <= step * 1.5);
        differ += pll->lo_hz != exact;
    }
    CHECK(differ < tries / 100);
}

TEST(r820t_calibration_rounds_as_the_firmware_does) {
    CHECK_EQ(fern::calibrated_xtal(25000000, 0), 25000000u);
    CHECK_EQ(fern::calibrated_xtal(25000000, 1000), 25000025u);
    // -25 Hz and a half, truncated toward zero after adding the half.
    CHECK_EQ(fern::calibrated_xtal(25000000, -1000), 24999976u);
    CHECK_EQ(fern::calibrated_xtal(24000000, 1500), 24000036u);
    CHECK_EQ(fern::calibrated_xtal(24000000, -20000), 23999521u);
}

TEST(r820t_one_tuning_adds_half_the_rate) {
    auto t = fern::one_tuning(25000000, 100000000, 10000000);
    REQUIRE(t);
    CHECK_EQ(t->lo_request, 105000000u);
    CHECK(std::fabs(t->offset - (-4.76837158203125)) < 1e-6);
    t = fern::one_tuning(25000000, 1000000000, 10000000);
    REQUIRE(t);
    CHECK(std::fabs(t->offset - (-76.2939453125)) < 1e-6);
    // The lowest centres: the LO must reach 27.66 MHz.
    CHECK(fern::one_tuning(25000000, fern::r820t_lo_min - 5000000, 10000000));
    CHECK(!fern::one_tuning(25000000, fern::r820t_lo_min - 5000001, 10000000));
    CHECK(!fern::one_tuning(25000000, 24000000, 2500000));
    CHECK(fern::one_tuning(25000000, 24700000, 6000000));
    CHECK(!fern::one_tuning(25000000, 1946000000, 10000000));
    CHECK(fern::one_tuning(25000000, 1800000000, 10000000));
}
