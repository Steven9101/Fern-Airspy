// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// The R2's and Mini's sample path: real samples through libairspy's own
// converter, then the PLL's remainder shifted out.
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

#include "convert.h"
#include "test.h"

namespace {

constexpr double two_pi = 6.283185307179586476925286766559;

// Real 12-bit samples of a cosine at `hz`, sampled at `fs`.
std::vector<uint16_t> real_tone(double hz, double fs, size_t count, double amplitude = 1000) {
    std::vector<uint16_t> v(count);
    for (size_t n = 0; n < count; ++n)
        v[n] = static_cast<uint16_t>(std::lround(2048 + amplitude * std::cos(two_pi * hz * n / fs)));
    return v;
}

// The power of the I/Q samples at `hz`, by correlation, skipping the
// converter's first samples.
double power_at(const std::vector<int16_t>& iq, double hz, double rate, size_t skip = 256) {
    std::complex<double> sum = 0;
    const size_t pairs = iq.size() / 2;
    for (size_t n = skip; n < pairs; ++n) {
        const std::complex<double> z(iq[2 * n], iq[2 * n + 1]);
        sum += z * std::polar(1.0, -two_pi * hz * n / rate);
    }
    return std::norm(sum / static_cast<double>(pairs - skip));
}

// The frequency of the strongest tone: roughly from the mean phase step,
// then exactly from how far the phase moved between a stretch at the start
// and one at the end. The mean step alone is pulled a little by the
// converter's faint image.
double frequency_of(const std::vector<int16_t>& iq, double rate, size_t skip = 256) {
    const size_t pairs = iq.size() / 2;
    std::complex<double> sum = 0;
    for (size_t n = skip + 1; n < pairs; ++n) {
        const std::complex<double> a(iq[2 * n - 2], iq[2 * n - 1]);
        const std::complex<double> b(iq[2 * n], iq[2 * n + 1]);
        sum += b * std::conj(a);
    }
    const double coarse = std::arg(sum) * rate / two_pi;
    const size_t len = 4096;
    auto stretch = [&](size_t from) {
        std::complex<double> s = 0;
        for (size_t n = from; n < from + len; ++n)
            s += std::complex<double>(iq[2 * n], iq[2 * n + 1]) * std::polar(1.0, -two_pi * coarse * n / rate);
        return s;
    };
    const size_t first = skip;
    const size_t last = pairs - len;
    return coarse + std::arg(stretch(last) * std::conj(stretch(first))) * rate / (two_pi * (last - first));
}

std::vector<int16_t> convert(fern::OneConverter& c, const std::vector<uint16_t>& real, size_t block = 16384) {
    std::vector<int16_t> out(real.size());
    for (size_t at = 0; at < real.size(); at += block) {
        uint64_t clipped = 0;
        unsigned peak = 0;
        c.process(real.data() + at, std::min(block, real.size() - at), out.data() + at, clipped, peak);
    }
    return out;
}

}  // namespace

// The R820T's IF sits at a quarter of the real rate, and with the LO above
// the signal a signal above the centre arrives below the IF. libairspy's
// converter must turn that into a tone above 0 Hz, and the image away.
TEST(convert_puts_a_signal_above_the_centre_above_0_hz) {
    const double iq_rate = 2500000;
    const double fs = 2 * iq_rate;
    fern::OneConverter c(0, iq_rate);
    const std::vector<int16_t> iq = convert(c, real_tone(fs / 4 - 100000, fs, 1 << 18));
    const double wanted = power_at(iq, 100000, iq_rate);
    const double image = power_at(iq, -100000, iq_rate);
    CHECK(wanted > 1e5);
    CHECK(10 * std::log10(wanted / image) > 60);
    CHECK(std::fabs(frequency_of(iq, iq_rate) - 100000) < 0.01);
}

TEST(convert_shifts_out_the_pll_remainder) {
    const double iq_rate = 10000000;
    const double fs = 2 * iq_rate;
    // The PLL landed 76.29 Hz low (1 GHz on an R2): every signal arrives
    // 76.29 Hz high, and the shift takes it back.
    fern::OneConverter plain(0, iq_rate);
    fern::OneConverter shifted(-76.2939453125, iq_rate);
    CHECK(shifted.shift().active());
    CHECK(!plain.shift().active());
    const std::vector<uint16_t> real = real_tone(fs / 4 - 1000076.2939453125, fs, 1 << 21);
    CHECK(std::fabs(frequency_of(convert(plain, real), iq_rate) - 1000076.2939453125) < 0.05);
    CHECK(std::fabs(frequency_of(convert(shifted, real), iq_rate) - 1000000) < 0.05);
}

TEST(convert_counts_clipping_and_the_peak) {
    fern::OneConverter c(0, 2500000);
    std::vector<uint16_t> real(1024, 2048);
    real[3] = 0;
    real[100] = 4095;
    real[200] = 4095;
    real[300] = 3000;
    // Bits above the twelve are not the converter's.
    real[400] = 0xF800;
    std::vector<int16_t> out(real.size());
    uint64_t clipped = 0;
    unsigned peak = 0;
    c.process(real.data(), real.size(), out.data(), clipped, peak);
    CHECK_EQ(clipped, uint64_t(3));
    CHECK_EQ(peak, 2048u);
    std::vector<uint16_t> quiet(1024, 2048);
    quiet[5] = 2148;
    c.process(quiet.data(), quiet.size(), out.data(), clipped, peak);
    CHECK_EQ(clipped, uint64_t(0));
    CHECK_EQ(peak, 100u);
}

// The first samples after start are zeros through the filter, not
// whatever the allocator left in its delay line.
TEST(convert_starts_from_silence) {
    for (int round = 0; round < 4; ++round) {
        // Leave something in freed memory for the converter to pick up.
        std::vector<int16_t> junk(4096, 12345);
        junk.clear();
        fern::OneConverter c(0, 2500000);
        std::vector<uint16_t> real(64, 2048);
        std::vector<int16_t> out(real.size());
        uint64_t clipped = 0;
        unsigned peak = 0;
        c.process(real.data(), real.size(), out.data(), clipped, peak);
        bool silent = true;
        for (int16_t v : out)
            silent = silent && v == 0;
        CHECK(silent);
    }
}

TEST(frequency_shift_is_seamless_across_calls) {
    const double rate = 2500000;
    fern::FrequencyShift whole(1234.5, rate);
    fern::FrequencyShift pieces(1234.5, rate);
    const size_t pairs = 10000;
    std::vector<float> a(2 * pairs);
    for (size_t n = 0; n < pairs; ++n) {
        a[2 * n] = 20000;
        a[2 * n + 1] = -5000;
    }
    std::vector<float> b = a;
    whole.process(a.data(), pairs);
    size_t at = 0;
    for (size_t len : {1u, 999u, 1024u, 3000u, 7u}) {
        pieces.process(b.data() + 2 * at, len);
        at += len;
    }
    pieces.process(b.data() + 2 * at, pairs - at);
    double worst = 0;
    for (size_t i = 0; i < a.size(); ++i)
        worst = std::max(worst, static_cast<double>(std::fabs(a[i] - b[i])));
    CHECK(worst < 0.05);
    // A constant becomes a tone at the shift.
    std::vector<int16_t> rounded(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        rounded[i] = static_cast<int16_t>(std::lround(a[i]));
    CHECK(std::fabs(frequency_of(rounded, rate, 0) - 1234.5) < 0.01);
}

// A tone that fills the converter comes out about half of 16 bits, and the
// rounding to 16 bits saturates rather than wraps.
TEST(convert_output_scale_and_saturation) {
    const double iq_rate = 2500000;
    const double fs = 2 * iq_rate;
    fern::OneConverter c(0, iq_rate);
    const std::vector<int16_t> iq = convert(c, real_tone(fs / 4 - 100000, fs, 1 << 16, 2047));
    int top = 0;
    for (size_t i = 4096; i < iq.size(); ++i)
        top = std::max(top, std::abs(static_cast<int>(iq[i])));
    CHECK(top > 15000);
    CHECK(top < 17500);
    // A converter slammed from limit to limit, a square wave at an eighth of
    // its rate whose fundamental is louder than any sine it can take: still
    // no sign flips.
    fern::OneConverter d(0, iq_rate);
    std::vector<uint16_t> square(1 << 14);
    for (size_t n = 0; n < square.size(); ++n)
        square[n] = (n / 4) % 2 ? 4095 : 0;
    const std::vector<int16_t> out = convert(d, square);
    // Its I swings evenly about 0: a value that wrapped would land on the
    // wrong side of it.
    int highest = 0;
    int lowest = 0;
    for (size_t i = 1024; i < out.size(); i += 2) {
        highest = std::max(highest, static_cast<int>(out[i]));
        lowest = std::min(lowest, static_cast<int>(out[i]));
    }
    CHECK(highest > 16000);
    CHECK(lowest < -16000);
}
