// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// The narrow interface between the module and the vendor libraries:
// libairspy for the Airspy R2 and Mini, libairspyhf for the HF+ family.
// Methods mirror the library calls one to one, with the same return
// conventions, so that the real implementation stays a thin wrapper and all
// decisions live in code the tests drive with a fake backend.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fern {

// Two USB ids, two libraries, two kinds of radio.
enum class Family {
    one,  // Airspy R2 and Mini, 1d50:60a1, libairspy
    hf,   // Airspy HF+, HF+ Discovery and Ranger, 03eb:800c, libairspyhf
};

inline const char* family_id(Family f) { return f == Family::one ? "airspy" : "airspyhf"; }

// libusb error codes, repeated here so that the module logic does not depend
// on libusb headers.
namespace usb_error {
constexpr int io = -1;
constexpr int access = -3;
constexpr int no_device = -4;
constexpr int not_found = -5;
constexpr int busy = -6;
constexpr int timeout = -7;
}  // namespace usb_error

const char* usb_error_text(int code);

// One Airspy as a USB scan finds it, before a vendor library opens it. The
// vendor libraries skip a device they cannot open or claim and say only
// that none was found; the scan says why.
struct Found {
    Family family = Family::one;
    std::string port;     // bus-port.port, e.g. "1-1.3"
    int open_error = 0;   // 0, or the usb_error that opening or claiming it gave
    bool has_serial = false;
    uint64_t serial = 0;
};

struct Enumeration {
    int error = 0;           // 0, or a usb_error when USB is unusable here
    std::string diagnostic;  // what to do about error, for the operator
    std::vector<Found> devices;
};

// Puts the devices in the order of their USB ports, as FernSDR's own USB
// scan lists them, so that index:<n> means the same radio in its setup and
// here, whatever order libusb found them in.
void order_by_port(std::vector<Found>& devices);

// Blocks of samples arrive on a thread of the vendor library. dropped
// counts what the library itself lost before this block, in the same unit
// as count.
//
// R2 and Mini: count unsigned 12-bit real samples, 0 to 4095, at twice the
// I/Q rate, with the band at a quarter of that real rate.
using RealCallback = void (*)(const uint16_t* samples, size_t count, uint64_t dropped, void* ctx);
// HF+: count complex samples, I then Q, as 32-bit floats.
using IqCallback = void (*)(const float* samples, size_t count, uint64_t dropped, void* ctx);

// An opened Airspy R2 or Mini. Destroying it closes the device; that must
// not happen while start() is in effect.
class OneDevice {
public:
    virtual ~OneDevice() = default;

    // airspy_version_string_read(): "AirSpy NOS v1.0.0-rc10-..." for the R2,
    // "AirSpy MINI ..." for the Mini.
    virtual int version(std::string& out) = 0;
    // The I/Q rates the firmware offers, in its order.
    virtual int samplerates(std::vector<uint32_t>& out) = 0;
    // The correction airspy_calibrate wrote to the flash, if any (valid is
    // false when none was written).
    virtual int read_calibration(bool& valid, int32_t& ppb) = 0;
    // The Si5351C's CLKIN loss-of-signal flag: true when no external 10 MHz
    // reference drives the clock. R2 only; the Mini has no Si5351C.
    virtual int clock_input_lost(bool& lost) = 0;
    virtual int set_samplerate_index(uint32_t index) = 0;
    virtual int set_packing(bool on) = 0;
    virtual int set_freq(uint32_t hz) = 0;
    virtual int set_linearity_gain(uint8_t value) = 0;
    virtual int set_sensitivity_gain(uint8_t value) = 0;
    virtual int set_lna_gain(uint8_t value) = 0;
    virtual int set_mixer_gain(uint8_t value) = 0;
    virtual int set_vga_gain(uint8_t value) = 0;
    virtual int set_lna_agc(bool on) = 0;
    virtual int set_mixer_agc(bool on) = 0;
    virtual int set_bias_tee(bool on) = 0;
    // Starts the library's threads; cb runs on one of them until stop().
    virtual int start(RealCallback cb, void* ctx) = 0;
    // Stops and joins the library's threads; no callback runs afterwards.
    virtual int stop() = 0;
    // False once the library saw USB fail, a callback refused or stop() ran.
    virtual bool streaming() = 0;
};

// An opened Airspy HF+, HF+ Discovery or Ranger.
class HfDevice {
public:
    virtual ~HfDevice() = default;

    virtual int version(std::string& out) = 0;
    // The part id, which tells the boards apart: 1 HF+, 2 HF+ Discovery,
    // 3 Ranger; 0 for firmware that does not say.
    virtual int board_id(uint32_t& out) = 0;
    virtual int samplerates(std::vector<uint32_t>& out) = 0;
    // The attenuator's steps in dB, ascending, as the firmware lists them.
    virtual int att_steps(std::vector<float>& out) = 0;
    virtual int bias_tee_count(int32_t& out) = 0;
    virtual int set_samplerate(uint32_t hz) = 0;
    // Exact: libairspyhf tunes in whole kHz and shifts the rest out of the
    // samples itself.
    virtual int set_freq(double hz) = 0;
    virtual int set_hf_agc(bool on) = 0;
    virtual int set_hf_agc_threshold(bool high) = 0;
    virtual int set_att(float db) = 0;
    virtual int set_hf_lna(bool on) = 0;
    virtual int set_bias_tee(bool on) = 0;
    virtual int start(IqCallback cb, void* ctx) = 0;
    virtual int stop() = 0;
    virtual bool streaming() = 0;
};

class Backend {
public:
    virtual ~Backend() = default;
    virtual Enumeration enumerate() = 0;
    // 0 with a device, or a negative code of the vendor library.
    virtual int open_one(uint64_t serial, std::unique_ptr<OneDevice>& out) = 0;
    virtual int open_hf(uint64_t serial, std::unique_ptr<HfDevice>& out) = 0;
};

// Serials print as 16 hexadecimal digits in capitals, as Airspy's own tools
// and SDR# show them.
std::string serial_text(uint64_t serial);

}  // namespace fern
