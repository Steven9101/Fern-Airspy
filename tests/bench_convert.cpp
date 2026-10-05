// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// CPU time of the R2's and Mini's sample path, per second of samples: the
// clip count, libairspy's converter and the PLL correction, on blocks as
// libairspy hands them over. The vendor library would run the converter
// itself; the clip count and the correction are the module's own cost.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <vector>

#include "convert.h"

namespace {

double thread_seconds() {
    timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Milliseconds of CPU for one second of samples at iq_rate.
double run(double iq_rate, double shift_hz, size_t block) {
    const size_t real_rate = static_cast<size_t>(2 * iq_rate);
    std::vector<uint16_t> real(block);
    for (size_t n = 0; n < block; ++n)
        real[n] = static_cast<uint16_t>(std::lround(2048 + 900 * std::cos(0.37 * static_cast<double>(n))));
    std::vector<int16_t> out(block);
    fern::OneConverter converter(shift_hz, iq_rate);
    uint64_t clipped = 0;
    unsigned peak = 0;
    // Once to warm the caches, then timed over three seconds of samples.
    converter.process(real.data(), block, out.data(), clipped, peak);
    const size_t blocks = 3 * real_rate / block;
    const double start = thread_seconds();
    for (size_t i = 0; i < blocks; ++i)
        converter.process(real.data(), block, out.data(), clipped, peak);
    const double seconds = thread_seconds() - start;
    return seconds * 1000 / (static_cast<double>(blocks * block) / static_cast<double>(real_rate));
}

}  // namespace

int main() {
    // libairspy's blocks: 131072 real samples unpacked, 98304 packed.
    std::printf("CPU per second of samples, one core:\n");
    std::printf("  10 Msps, PLL correction on:   %6.1f ms\n", run(10e6, -76.29, 131072));
    std::printf("  10 Msps, no correction:       %6.1f ms\n", run(10e6, 0, 131072));
    std::printf("  10 Msps packed, correction:   %6.1f ms\n", run(10e6, -76.29, 98304));
    std::printf("  6 Msps (Mini), correction:    %6.1f ms\n", run(6e6, -3.81, 131072));
    std::printf("  2.5 Msps, correction:         %6.1f ms\n", run(2.5e6, 4.77, 131072));
    return 0;
}
