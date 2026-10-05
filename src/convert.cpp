// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include "convert.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

// The half-band filter libairspy builds its converter with. In C++ these
// constants have internal linkage, so this copy does not clash with
// libairspy's own.
#include "filters.h"

namespace fern {

namespace {

constexpr double two_pi = 6.283185307179586476925286766559;

int16_t saturate(float v) {
    const float r = v >= 0 ? v + 0.5f : v - 0.5f;
    if (r >= 32767.0f)
        return 32767;
    if (r <= -32768.0f)
        return -32768;
    return static_cast<int16_t>(r);
}

}  // namespace

FrequencyShift::FrequencyShift(double shift_hz, double sample_rate)
    : shift_hz_(shift_hz),
      step_(sample_rate > 0 ? two_pi * shift_hz / sample_rate : 0),
      active_(sample_rate > 0 && std::fabs(shift_hz) >= 0.01) {
    if (!active_)
        return;
    cos_.resize(block);
    sin_.resize(block);
    for (size_t n = 0; n < block; ++n) {
        cos_[n] = static_cast<float>(std::cos(step_ * static_cast<double>(n)));
        sin_[n] = static_cast<float>(std::sin(step_ * static_cast<double>(n)));
    }
}

void FrequencyShift::process(float* iq, size_t pairs) {
    if (!active_)
        return;
    for (size_t start = 0; start < pairs; start += block) {
        const size_t n = std::min(block, pairs - start);
        const float bc = static_cast<float>(std::cos(phase_));
        const float bs = static_cast<float>(std::sin(phase_));
        float* p = iq + 2 * start;
        for (size_t k = 0; k < n; ++k) {
            // e^(j phase) e^(j step k), then the sample times that.
            const float c = bc * cos_[k] - bs * sin_[k];
            const float s = bs * cos_[k] + bc * sin_[k];
            const float i = p[2 * k];
            const float q = p[2 * k + 1];
            p[2 * k] = i * c - q * s;
            p[2 * k + 1] = i * s + q * c;
        }
        phase_ = std::fmod(phase_ + step_ * static_cast<double>(n), two_pi);
    }
}

OneConverter::OneConverter(double shift_hz, double iq_rate)
    : converter_(iqconverter_float_create(HB_KERNEL_FLOAT, HB_KERNEL_FLOAT_LEN)), shift_(shift_hz, iq_rate) {
    if (!converter_)
        throw std::bad_alloc();
}

OneConverter::~OneConverter() { iqconverter_float_free(converter_); }

void OneConverter::process(const uint16_t* real, size_t count, int16_t* out, uint64_t& clipped, unsigned& peak) {
    if (work_.size() < count)
        work_.resize(count);
    float* w = work_.data();
    uint64_t cut = 0;
    unsigned top = 0;
    for (size_t i = 0; i < count; ++i) {
        const unsigned v = real[i] & 0x0fff;
        cut += v == adc_min || v == adc_max;
        const unsigned d = v >= adc_half ? v - adc_half : adc_half - v;
        top = d > top ? d : top;
        // 12 bits to the top of 16, as libairspy's int16 path scales them,
        // so that the converter's output needs no further scaling.
        w[i] = static_cast<float>((static_cast<int>(v) - static_cast<int>(adc_half)) * 16);
    }
    clipped = cut;
    peak = top;
    iqconverter_float_process(converter_, w, static_cast<int>(count));
    shift_.process(w, count / 2);
    for (size_t i = 0; i < count; ++i)
        out[i] = saturate(w[i]);
}

}  // namespace fern
