// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include "airspy_backend.h"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <linux/netlink.h>
#include <sys/socket.h>
#include <unistd.h>

#include <airspy.h>
#include <airspyhf.h>
#include <libusb.h>

namespace fern {

namespace {

constexpr uint16_t one_vid = 0x1d50;
constexpr uint16_t one_pid = 0x60a1;
constexpr uint16_t hf_vid = 0x03eb;
constexpr uint16_t hf_pid = 0x800c;

// The serial string descriptors, as the vendor libraries read them:
// "AIRSPY SN:" and "AIRSPYHF SN:", then 16 hexadecimal digits.
constexpr size_t one_serial_length = 26;
constexpr size_t one_serial_prefix = 10;
constexpr size_t hf_serial_length = 28;
const char hf_prefix[] = "AIRSPYHF SN:";

// Where airspy_calibrate keeps its correction in the R2's and Mini's SPI
// flash, and the header that marks it valid (airspyone_host,
// airspy-tools/src/airspy_calibrate.c).
constexpr uint32_t calibration_offset = 0x20000;
constexpr uint32_t calibration_header = 0xCA1B0001;

// The Si5351C's status register and its CLKIN loss-of-signal bit.
constexpr uint8_t si5351c_status = 0;
constexpr uint8_t si5351c_clkin_los = 1 << 4;

uint32_t little_endian_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 | static_cast<uint32_t>(p[2]) << 16 |
           static_cast<uint32_t>(p[3]) << 24;
}

class AirspyOne : public OneDevice {
public:
    explicit AirspyOne(airspy_device* dev) : dev_(dev) {}
    ~AirspyOne() override { airspy_close(dev_); }
    AirspyOne(const AirspyOne&) = delete;
    AirspyOne& operator=(const AirspyOne&) = delete;

    int version(std::string& out) override {
        char text[128] = {};
        const int r = airspy_version_string_read(dev_, text, sizeof text);
        if (r != AIRSPY_SUCCESS)
            return r;
        text[sizeof text - 1] = '\0';
        out = text;
        return 0;
    }

    int samplerates(std::vector<uint32_t>& out) override {
        uint32_t count = 0;
        int r = airspy_get_samplerates(dev_, &count, 0);
        if (r != AIRSPY_SUCCESS)
            return r;
        if (count == 0 || count > 64)
            return AIRSPY_ERROR_OTHER;
        std::vector<uint32_t> rates(count);
        r = airspy_get_samplerates(dev_, rates.data(), count);
        if (r != AIRSPY_SUCCESS)
            return r;
        out = rates;
        return 0;
    }

    int read_calibration(bool& valid, int32_t& ppb) override {
        uint8_t data[12] = {};
        const int r = airspy_spiflash_read(dev_, calibration_offset, sizeof data, data);
        if (r != AIRSPY_SUCCESS)
            return r;
        valid = little_endian_u32(data) == calibration_header;
        ppb = static_cast<int32_t>(little_endian_u32(data + 8));
        return 0;
    }

    int clock_input_lost(bool& lost) override {
        uint8_t value = 0;
        const int r = airspy_si5351c_read(dev_, si5351c_status, &value);
        if (r != AIRSPY_SUCCESS)
            return r;
        lost = (value & si5351c_clkin_los) != 0;
        return 0;
    }

    int set_samplerate_index(uint32_t index) override { return airspy_set_samplerate(dev_, index); }
    int set_packing(bool on) override { return airspy_set_packing(dev_, on ? 1 : 0); }
    int set_freq(uint32_t hz) override { return airspy_set_freq(dev_, hz); }
    int set_linearity_gain(uint8_t value) override { return airspy_set_linearity_gain(dev_, value); }
    int set_sensitivity_gain(uint8_t value) override { return airspy_set_sensitivity_gain(dev_, value); }
    int set_lna_gain(uint8_t value) override { return airspy_set_lna_gain(dev_, value); }
    int set_mixer_gain(uint8_t value) override { return airspy_set_mixer_gain(dev_, value); }
    int set_vga_gain(uint8_t value) override { return airspy_set_vga_gain(dev_, value); }
    int set_lna_agc(bool on) override { return airspy_set_lna_agc(dev_, on ? 1 : 0); }
    int set_mixer_agc(bool on) override { return airspy_set_mixer_agc(dev_, on ? 1 : 0); }
    int set_bias_tee(bool on) override { return airspy_set_rf_bias(dev_, on ? 1 : 0); }

    int start(RealCallback cb, void* ctx) override {
        cb_ = cb;
        ctx_ = ctx;
        // Real samples, which the module converts itself (convert.h).
        const int r = airspy_set_sample_type(dev_, AIRSPY_SAMPLE_UINT16_REAL);
        if (r != AIRSPY_SUCCESS)
            return r;
        return airspy_start_rx(dev_, &AirspyOne::on_transfer, this);
    }
    int stop() override { return airspy_stop_rx(dev_); }
    bool streaming() override { return airspy_is_streaming(dev_) == AIRSPY_TRUE; }

private:
    static int on_transfer(airspy_transfer* t) {
        AirspyOne* self = static_cast<AirspyOne*>(t->ctx);
        if (t->sample_count > 0)
            self->cb_(static_cast<const uint16_t*>(t->samples), static_cast<size_t>(t->sample_count),
                      t->dropped_samples, self->ctx_);
        return 0;
    }

    airspy_device* dev_;
    RealCallback cb_ = nullptr;
    void* ctx_ = nullptr;
};

class AirspyHf : public HfDevice {
public:
    explicit AirspyHf(airspyhf_device_t* dev) : dev_(dev) {}
    ~AirspyHf() override { airspyhf_close(dev_); }
    AirspyHf(const AirspyHf&) = delete;
    AirspyHf& operator=(const AirspyHf&) = delete;

    int version(std::string& out) override {
        // libairspyhf copies `length` bytes out of a buffer of
        // MAX_VERSION_STRING_SIZE, so no more may be asked for.
        char text[MAX_VERSION_STRING_SIZE] = {};
        const int r = airspyhf_version_string_read(dev_, text, sizeof text);
        if (r != AIRSPYHF_SUCCESS)
            return r;
        text[sizeof text - 1] = '\0';
        out = text;
        return 0;
    }

    int board_id(uint32_t& out) override {
        airspyhf_read_partid_serialno_t id = {};
        const int r = airspyhf_board_partid_serialno_read(dev_, &id);
        if (r != AIRSPYHF_SUCCESS)
            return r;
        out = id.part_id;
        return 0;
    }

    int samplerates(std::vector<uint32_t>& out) override {
        uint32_t count = 0;
        int r = airspyhf_get_samplerates(dev_, &count, 0);
        if (r != AIRSPYHF_SUCCESS)
            return r;
        if (count == 0 || count > 100)
            return AIRSPYHF_ERROR;
        std::vector<uint32_t> rates(count);
        r = airspyhf_get_samplerates(dev_, rates.data(), count);
        if (r != AIRSPYHF_SUCCESS)
            return r;
        out = rates;
        return 0;
    }

    int att_steps(std::vector<float>& out) override {
        uint32_t count = 0;
        int r = airspyhf_get_att_steps(dev_, &count, 0);
        if (r != AIRSPYHF_SUCCESS)
            return r;
        if (count == 0 || count > 64)
            return AIRSPYHF_ERROR;
        std::vector<float> steps(count);
        r = airspyhf_get_att_steps(dev_, steps.data(), count);
        if (r != AIRSPYHF_SUCCESS)
            return r;
        out = steps;
        return 0;
    }

    int bias_tee_count(int32_t& out) override { return airspyhf_get_bias_tee_count(dev_, &out); }
    int set_samplerate(uint32_t hz) override { return airspyhf_set_samplerate(dev_, hz); }
    int set_freq(double hz) override { return airspyhf_set_freq_double(dev_, hz); }
    int set_hf_agc(bool on) override { return airspyhf_set_hf_agc(dev_, on ? 1 : 0); }
    int set_hf_agc_threshold(bool high) override { return airspyhf_set_hf_agc_threshold(dev_, high ? 1 : 0); }
    int set_att(float db) override { return airspyhf_set_att(dev_, db); }
    int set_hf_lna(bool on) override { return airspyhf_set_hf_lna(dev_, on ? 1 : 0); }
    int set_bias_tee(bool on) override { return airspyhf_set_bias_tee(dev_, on ? 1 : 0); }

    int start(IqCallback cb, void* ctx) override {
        cb_ = cb;
        ctx_ = ctx;
        return airspyhf_start(dev_, &AirspyHf::on_transfer, this);
    }
    int stop() override { return airspyhf_stop(dev_); }
    bool streaming() override { return airspyhf_is_streaming(dev_) != 0; }

private:
    static int on_transfer(airspyhf_transfer_t* t) {
        AirspyHf* self = static_cast<AirspyHf*>(t->ctx);
        // airspyhf_complex_float_t is two floats, re then im.
        if (t->sample_count > 0)
            self->cb_(reinterpret_cast<const float*>(t->samples), static_cast<size_t>(t->sample_count),
                      t->dropped_samples, self->ctx_);
        return 0;
    }

    airspyhf_device_t* dev_;
    IqCallback cb_ = nullptr;
    void* ctx_ = nullptr;
};

static_assert(sizeof(airspyhf_complex_float_t) == 2 * sizeof(float), "an HF+ sample is two floats");

// libusb fails to start in a sandbox that hides /dev/bus/usb or forbids the
// netlink socket it watches for hotplug events on. Say which, since both
// look like "no Airspy plugged in" otherwise.
std::string usb_diagnostic(int code) {
    if (::access("/dev/bus/usb", F_OK) != 0)
        return "USB devices are not visible to this process: /dev/bus/usb does not exist. If FernSDR runs under "
               "systemd, its unit must not set PrivateDevices=yes; in Docker, pass the USB bus with "
               "--device /dev/bus/usb";
    const int s = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_KOBJECT_UEVENT);
    if (s < 0)
        return std::string("libusb needs a netlink socket to watch for USB devices, and this process may not "
                           "open one (") +
               std::strerror(errno) +
               "). If FernSDR runs under systemd, add AF_NETLINK to RestrictAddressFamilies in its unit";
    ::close(s);
    return std::string("libusb could not start (") + libusb_error_name(code) + ")";
}

std::string port_of(libusb_device* dev) {
    uint8_t ports[8];
    const int n = libusb_get_port_numbers(dev, ports, sizeof ports);
    std::string text = std::to_string(libusb_get_bus_number(dev));
    for (int i = 0; i < n; ++i)
        text += (i == 0 ? "-" : ".") + std::to_string(ports[i]);
    return text;
}

bool parse_serial(Family family, const unsigned char* text, int length, uint64_t& out) {
    size_t skip;
    if (family == Family::one) {
        if (length != static_cast<int>(one_serial_length))
            return false;
        skip = one_serial_prefix;
    } else {
        if (length != static_cast<int>(hf_serial_length) ||
            std::memcmp(text, hf_prefix, sizeof hf_prefix - 1) != 0)
            return false;
        skip = sizeof hf_prefix - 1;
    }
    char digits[20] = {};
    std::memcpy(digits, text + skip, static_cast<size_t>(length) - skip);
    char* end = nullptr;
    errno = 0;
    const unsigned long long v = std::strtoull(digits, &end, 16);
    if (end == digits || errno != 0)
        return false;
    out = v;
    return true;
}

}  // namespace

Enumeration AirspyBackend::enumerate() {
    Enumeration e;
    libusb_context* ctx = nullptr;
    int r = libusb_init(&ctx);
    if (r < 0) {
        e.error = r;
        e.diagnostic = usb_diagnostic(r);
        return e;
    }
    libusb_device** list = nullptr;
    const ssize_t n = libusb_get_device_list(ctx, &list);
    if (n < 0) {
        e.error = static_cast<int>(n);
        e.diagnostic = usb_diagnostic(static_cast<int>(n));
        libusb_exit(ctx);
        return e;
    }
    for (ssize_t i = 0; i < n; ++i) {
        libusb_device* dev = list[i];
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev, &desc) != 0)
            continue;
        Found f;
        if (desc.idVendor == one_vid && desc.idProduct == one_pid)
            f.family = Family::one;
        else if (desc.idVendor == hf_vid && desc.idProduct == hf_pid)
            f.family = Family::hf;
        else
            continue;
        f.port = port_of(dev);
        libusb_device_handle* handle = nullptr;
        r = libusb_open(dev, &handle);
        if (r != 0) {
            f.open_error = r;
            e.devices.push_back(f);
            continue;
        }
        unsigned char text[64] = {};
        if (desc.iSerialNumber != 0) {
            const int length = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, text, sizeof text - 1);
            if (length > 0)
                f.has_serial = parse_serial(f.family, text, length, f.serial);
        }
        // The vendor libraries skip a device whose interface another
        // program holds and report only that none was found; claiming it
        // here for a moment tells which.
        r = libusb_claim_interface(handle, 0);
        if (r == 0)
            libusb_release_interface(handle, 0);
        else if (r == LIBUSB_ERROR_BUSY)
            f.open_error = usb_error::busy;
        libusb_close(handle);
        e.devices.push_back(f);
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return e;
}

int AirspyBackend::open_one(uint64_t serial, std::unique_ptr<OneDevice>& out) {
    airspy_device* dev = nullptr;
    const int r = airspy_open_sn(&dev, serial);
    if (r != AIRSPY_SUCCESS)
        return r;
    if (!dev)
        return AIRSPY_ERROR_OTHER;
    out = std::make_unique<AirspyOne>(dev);
    return 0;
}

int AirspyBackend::open_hf(uint64_t serial, std::unique_ptr<HfDevice>& out) {
    airspyhf_device_t* dev = nullptr;
    const int r = airspyhf_open_sn(&dev, serial);
    if (r != AIRSPYHF_SUCCESS)
        return r < 0 ? r : AIRSPYHF_ERROR;
    if (!dev)
        return AIRSPYHF_ERROR;
    out = std::make_unique<AirspyHf>(dev);
    return 0;
}

std::string library_versions() {
    airspy_lib_version_t one;
    airspy_lib_version(&one);
    airspyhf_lib_version_t hf;
    airspyhf_lib_version(&hf);
    const struct libusb_version* usb = libusb_get_version();
    return "libairspy " + std::to_string(one.major_version) + "." + std::to_string(one.minor_version) + "." +
           std::to_string(one.revision) + ", libairspyhf " + std::to_string(hf.major_version) + "." +
           std::to_string(hf.minor_version) + "." + std::to_string(hf.revision) + " and libusb " +
           std::to_string(usb->major) + "." + std::to_string(usb->minor) + "." + std::to_string(usb->micro);
}

}  // namespace fern
