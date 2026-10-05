# Third-party code

Three libraries are copied unmodified from their upstream repositories, with
only the files the build compiles and their licences. The Makefile compiles
them directly; their own build systems are not used.

## libairspy

- Upstream: https://github.com/airspy/airspyone_host
- Commit `43570b63de90dacd2c5fc6500a538617fdb406ff` (2024-03-01), library
  version 1.0.12, the last commit before the licence change described below
- License: `airspy.c`, `airspy.h` and `airspy_commands.h` under the BSD
  3-clause licence, `filters.h` and the two `iqconverter_*` files under the
  MIT licence; see `libairspy/LICENSE.md` and the comment at the top of
  each file
- Files: `libairspy/src/airspy.c`, `airspy.h`, `airspy_commands.h`,
  `filters.h`, `iqconverter_float.c`, `iqconverter_float.h`,
  `iqconverter_int16.c`, `iqconverter_int16.h`, and `libairspy/LICENSE.md`

Do not update past this commit without reading the licences again. On
2025-06-11 (commits `126b0a1`, `0ff938f`, `2fed7ac`, `d892755`, `25f234e`,
`a379373`) the four `iqconverter_*` files were relicensed to permit use
"exclusively as part of the Airspy ecosystem" and to forbid redistribution
outside it without written consent. That is not a free licence and cannot be
combined with the GPL; libairspy does not build without those files. The
files here are the earlier MIT versions, which stay under the MIT licence.
Nothing after the pinned commit changes `airspy.c` or `airspy.h`.

The module also calls `iqconverter_float_create()`, `_process()` and
`_free()` itself, with the `HB_KERNEL_FLOAT` filter from `filters.h`
(`src/convert.cpp`): libairspy's own conversion of the R2's and Mini's real
samples to I/Q, run by the module so that it can count clipped samples
first.

## libairspyhf

- Upstream: https://github.com/airspy/airspyhf
- Commit `24fe8ffcb00b14f827268bbad89ae1392de055e5` (2026-07-22), library
  version 1.8.1
- License: BSD 3-clause, see `libairspyhf/LICENSE.BSD` and the comment at the
  top of each file. (The repository's tools are under the GPL; none of them
  is copied.)
- Files: `libairspyhf/src/airspyhf.c`, `airspyhf.h`, `airspyhf_commands.h`,
  `iqbalancer.c`, `iqbalancer.h`, and `LICENSE.BSD`

## libusb

- Upstream: https://github.com/libusb/libusb
- Tag: `v1.0.30`, commit `87a55632db62c9bdc58cd31d3ccfa673f1bb017f`, the same as
  Fern-RTLSDR's and Fern-RX888's
- License: LGPL-2.1-or-later, see `libusb/COPYING`; authors in `libusb/AUTHORS`
- Files: `libusb/core.c`, `descriptor.c`, `hotplug.c`, `io.c`, `sync.c`,
  `strerror.c`, `os/linux_usbfs.c`, `os/linux_netlink.c`,
  `os/events_posix.c`, `os/threads_posix.c` and the headers they include

Used only by `make static` and `make package`; the native `make` links the
system libusb. Built without udev: devices are found through sysfs and
usbfs, hotplug events arrive on a netlink socket. `libusb-config/config.h`
is written for this project in place of the header that libusb's configure
script generates; it defines what configure would define on glibc Linux with
`--disable-udev`.

## Updating

Clone the new commit with `git clone <url>` and `git checkout <commit>`,
check the licence of every file to be copied (see libairspy above), copy
the same files over the ones here, update the commits above and in
`notices()` in `src/main.cpp`, and run `make test`, `make test-asan`,
`make static ARCH=aarch64` and `make static ARCH=armhf`. For libairspy, also
check that `airspy_open_sn()`, `airspy_start_rx()` with
`AIRSPY_SAMPLE_UINT16_REAL`, `airspy_stop_rx()`, `airspy_set_samplerate()`
by index and `airspy_set_freq()` still behave as `src/airspy_backend.cpp`
expects, and that the firmware's `r820t_set_pll()` (airspyone_firmware,
`common/r820t.c`) still computes what `src/r820t.cpp` mirrors.
