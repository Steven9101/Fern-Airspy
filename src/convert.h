// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// The R2's and Mini's sample path. The radio sends real 12-bit samples at
// twice the I/Q rate with the band around a quarter of that rate; libairspy
// would turn them into I/Q itself, but then nothing would tell when the
// converter clips. The module takes the real samples, counts the clipped
// ones, and runs libairspy's own converter on them (iqconverter_float, the
// code libairspy uses for AIRSPY_SAMPLE_FLOAT32_IQ), then shifts out what
// the R820T's PLL could not tune (r820t.h).
//
// The float converter folds its symmetric filter and costs a third less
// than libairspy's int16 one (make bench). Its output is rounded to 16-bit
// I/Q: that holds the ADC's 12 bits and the filter's gain with headroom to
// spare, so 32-bit floats would carry nothing more, at twice the bytes
// through FernSDR's pipe: 40 MB/s rather than 80 at 10 Msps.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

extern "C" {
#include "iqconverter_float.h"
}

namespace fern {

// The converter's limits: a real sample of 0 or 4095 clipped.
constexpr uint16_t adc_min = 0;
constexpr uint16_t adc_max = 4095;
constexpr unsigned adc_half = 2048;

// Rotates I/Q samples by a constant frequency. Phase is kept in double
// precision and the rotation computed per sample from a table of one block,
// so that it neither drifts nor costs a sine per sample.
class FrequencyShift {
public:
    FrequencyShift(double shift_hz, double sample_rate);

    // Below this the shift is not worth the time: a hundredth of a hertz.
    bool active() const { return active_; }
    double shift_hz() const { return shift_hz_; }
    // pairs I/Q pairs, in place.
    void process(float* iq, size_t pairs);

private:
    static constexpr size_t block = 1024;
    double shift_hz_;
    double step_;   // radians per sample
    double phase_ = 0;
    bool active_;
    std::vector<float> cos_;
    std::vector<float> sin_;
};

class OneConverter {
public:
    OneConverter(double shift_hz, double iq_rate);
    ~OneConverter();
    OneConverter(const OneConverter&) = delete;
    OneConverter& operator=(const OneConverter&) = delete;

    // count real samples, a multiple of 4, become count / 2 I/Q pairs, that
    // is count int16 values, in out, a full-scale tone about 16384 from 0.
    // clipped counts the real samples at the converter's limits; peak is the
    // largest distance of one from the midpoint, 0 to 2048.
    void process(const uint16_t* real, size_t count, int16_t* out, uint64_t& clipped, unsigned& peak);

    const FrequencyShift& shift() const { return shift_; }

private:
    iqconverter_float_t* converter_;
    FrequencyShift shift_;
    std::vector<float> work_;
};

}  // namespace fern
