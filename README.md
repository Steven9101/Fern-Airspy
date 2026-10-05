# Fern-Airspy

Fern-Airspy lets [FernSDR](https://github.com/Steven9101/WebSDR) receive with
an Airspy. It is an input module: a separate program that FernSDR starts for
a band and talks to over pipes, as FernSDR's `docs/MODULES.md` describes.
FernSDR sends the band's settings; the module opens the Airspy, tunes it,
sets its gain and writes its I/Q samples to FernSDR. It reports statistics
once a second and takes gain, attenuator, preamplifier and bias tee changes
while it runs.

The module is one statically linked executable for each of x86_64, aarch64
and armhf (Raspberry Pi 2 and later). libairspy, libairspyhf and libusb are
built into it, so nothing needs installing besides FernSDR, on any Linux
distribution and in Docker.

## Hardware

| Radio | USB id | Covers | Sample rates | Samples |
|---|---|---|---|---|
| Airspy R2 | 1d50:60a1 | 24 MHz to 1.8 GHz | 10 and 2.5 Msps | 16-bit I/Q |
| Airspy Mini | 1d50:60a1 | 24 MHz to 1.8 GHz | 6 and 3 Msps | 16-bit I/Q |
| Airspy HF+ (Dual Port) | 03eb:800c | 9 kHz to 31 MHz, 60 to 260 MHz | those its firmware lists | 32-bit float I/Q |
| Airspy HF+ Discovery | 03eb:800c | 0.5 kHz to 31 MHz, 60 to 260 MHz | those its firmware lists, up to 912 ksps | 32-bit float I/Q |

The module offers each radio the sample rates its firmware reports, and
`fern-airspy --list-devices` shows them. A band whose rate the radio does not
have is refused with the list.

No Airspy was attached while this version was written. The module was built
and tested against fake radios that behave as the vendor libraries and the
firmware do (`tests/fake_backend.cpp`); how it works with real hardware
follows the vendor sources, not a test on a radio.

## Using it with FernSDR

Install the package for your machine from the admin panel's Modules page,
which offers the releases of this repository, then add a band from the
suggestions there or by hand. From a shell:

```sh
fernsdr --install-module airspy-0.1.0-linux-x86_64.fernmod fernsdr.conf
```

A band for an R2, all of the 2 m band and more:

```ini
[band:2m]
name        = 2 m
source      = module
module      = airspy
sample_rate = 2500000
signal      = iq
center      = 145000000
module.gain = auto
```

A band for an HF+ Discovery:

```ini
[band:40m]
name        = 40 m
source      = module
module      = airspy
sample_rate = 768000
signal      = iq
center      = 7100000
```

With one Airspy plugged in, `module.device` can be left out. With several,
give each band `module.device = serial:<serial>`, as `fern-airspy
--list-devices` prints the serials.

### USB permissions

FernSDR's installer adds udev rules that let the receiver's user open
Airspys (1d50:60a1 and 03eb:800c). A receiver installed before FernSDR 0.1.2
does not have them: run FernSDR's install command once more, which keeps
your settings and adds the rule, then unplug the Airspy and plug it in
again. Without FernSDR's installer, put

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="60a1", MODE:="0660", GROUP:="fernsdr"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="800c", MODE:="0660", GROUP:="fernsdr"
```

in `/etc/udev/rules.d/61-fernsdr-usb.rules`, with the receiver's group in
place of `fernsdr`, and replug the Airspy. In Docker, pass the USB bus with
`--device /dev/bus/usb`.

The module tells which problem it meets: no Airspy plugged in, one it may
not open, one another program holds (spyserver, SDR++, airspy_rx, another
band), or a USB connection that fails while it runs.

## Settings

`fern-airspy --describe` lists them with their help texts; FernSDR's admin
panel shows the same. A setting for the other kind of Airspy is refused
unless it is at its default, so that a band cannot quietly ask for
something its radio does not do.

| Setting | Default | Live | For | |
|---|---|---|---|---|
| `device` | empty | no | all | `serial:<serial>` or `index:<n>` |
| `gain` | `auto` | yes | all | R2 and Mini: `auto`, a preset `0` to `21`, `tuner` or `manual`; HF+: `auto` or `manual` |
| `gain_mode` | `linearity` | yes | R2, Mini | the preset ladder: `linearity` or `sensitivity` |
| `lna_gain` | 1 | yes | R2, Mini | with `gain = manual`, 0 to 14 |
| `mixer_gain` | 5 | yes | R2, Mini | with `gain = manual`, 0 to 15 |
| `vga_gain` | 5 | yes | R2, Mini | with `gain = manual` or `tuner`, 0 to 15 |
| `attenuation` | 0 | yes | HF+ | with `gain = manual`, 0 to 48 dB in the radio's steps |
| `agc_threshold` | `low` | yes | HF+ | with `gain = auto`: `low` or `high` |
| `preamp` | no | yes | HF+ | 6 dB more gain in front |
| `bias_tee` | no | yes | all | DC on the antenna input; only HF+ models that have one |
| `packing` | `auto` | no | R2, Mini | 12-bit USB packing: `auto` (above 5 Msps), `on`, `off` |

### Gain

On an R2 or Mini, `gain = auto` is the module's own control, the same as
Fern-RTLSDR's: it walks libairspy's ladder of 22 presets (linearity, or
sensitivity with `gain_mode`), choosing the highest that keeps the 12-bit
converter out of clipping with 6 dB to spare, and steps down at once when
something keeps clipping. It starts at preset 10. FernSDR's admin panel shows
the share of clipped samples. A number holds one preset; `tuner` lets the
R820T's own LNA and mixer AGC work, which watches the tuner and not the
converter; `manual` sets the three stages from `lna_gain`, `mixer_gain` and
`vga_gain`.

On an HF+, `gain = auto` is the radio's own AGC, and `manual` switches it off
for the attenuator in `attenuation`.

### Tuning accuracy

The R2's and Mini's R820T tuner sets its oscillator in steps of the crystal
over 65536, divided down: a few hertz apart at 100 MHz, 190 Hz apart at
1 GHz. The firmware takes the nearest step and does not say which, so a
band would lie up to 95 Hz beside the frequency asked for. The module works
out the step the firmware takes and shifts the difference out of the
samples, so the band is where it was asked to be. The HF+ library does the
same for the HF+.

What neither can know is the crystal's own error. Measure it and set the
band's `ppm` in FernSDR. A correction stored in the Airspy with Airspy's
`airspy_calibrate` moves only the tuner and not the sample clock, which
FernSDR's `ppm` cannot follow; the module takes it into account for the
tuning, and logs that it is there, but `airspy_calibrate -w 0` and
FernSDR's `ppm` correct the whole band better.

### Bandwidth and CPU

At 10 Msps an R2 sends 30 MB/s over USB with packing (40 MB/s without),
and the module writes 40 MB/s of 16-bit I/Q to FernSDR. Plug it straight
into a USB 2 or USB 3 port of the computer, not into a hub. On a Raspberry
Pi, start with 2.5 Msps. The HF+ at 912 ksps sends 3.6 MB/s and
the module writes 7.3 MB/s of floats.

`make bench` measures the module's own work on the R2's samples, the
conversion to I/Q and the PLL correction, per second of samples.

## Building

```sh
make                 # build/fern-airspy with the system libusb
make test            # unit, protocol, command line and package tests
make test-asan       # the unit and protocol tests under ASan and UBSan
make test-tsan       # the same under TSan
make package         # dist/airspy-0.1.0-linux-<arch>.fernmod, statically linked
make package ARCH=aarch64
make package ARCH=armhf
```

The native build needs the libusb-1.0 development files; `make static` and
`make package` build libusb from `third_party/` and need none. Cross builds
need `aarch64-linux-gnu-g++` or `arm-linux-gnueabihf-g++`.

## Licence

GPL-2.0-or-later, see `LICENSE`. libairspy and libairspyhf are under the BSD
3-clause licence, libairspy's converter under the MIT licence, and libusb
under LGPL-2.1-or-later; `third_party/README.md` names the versions and why
libairspy is pinned where it is. `fern-airspy --notices` prints all of them.
