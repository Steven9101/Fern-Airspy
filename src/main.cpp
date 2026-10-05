// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <string>
#include <string_view>
#include <sys/signalfd.h>
#include <unistd.h>

#include "airspy_backend.h"
#include "failure.h"
#include "io.h"
#include "json.h"
#include "listing.h"
#include "log.h"
#include "session.h"
#include "settings.h"

using namespace fern;

namespace fern {
// build/gen/notices.cpp, from LICENSE and third_party/.
extern const char module_licence[];
extern const char libairspy_notices[];
extern const char libairspyhf_notices[];
extern const char libusb_notices[];
}  // namespace fern

namespace {

const char usage_text[] =
    "usage: fern-airspy --describe          print the module and its settings as JSON\n"
    "       fern-airspy --list-devices      print the Airspys this machine can see as JSON\n"
    "       fern-airspy --fernsdr-module 1  run as a FernSDR input module (FernSDR starts it so)\n"
    "       fern-airspy --notices           print the licences of this module and of the libraries in it\n"
    "       fern-airspy --version\n";

int print(const std::string& text) {
    return write_all(STDOUT_FILENO, text.data(), text.size()) == 0 ? 0 : exit_status::internal;
}

int describe() {
    const std::string text = json::serialize(describe_module()) + "\n";
    if (text.size() > max_report_bytes) {
        std::fprintf(stderr, "fern-airspy: the description is larger than %zu bytes\n", max_report_bytes);
        return exit_status::internal;
    }
    return print(text);
}

int list() {
    AirspyBackend backend;
    // FernSDR allows 10 s; leave a margin for starting up and printing.
    const Listing listing = list_devices(backend, std::chrono::milliseconds(8500));
    const int status = print(report_text(listing.report));
    if (!listing.complete)
        _exit(status);  // a worker thread is still inside a vendor library
    return status;
}

int notices() {
    return print(std::string("fern-airspy ") + module_version() +
                 " is under the GNU General Public License, version 2 or later, printed at the end; its source "
                 "is at https://github.com/Steven9101/Fern-Airspy.\n\n"
                 "It contains libairspy from https://github.com/airspy/airspyone_host at commit "
                 "43570b63de90dacd2c5fc6500a538617fdb406ff (BSD-3-Clause, its converter and filter MIT), "
                 "libairspyhf from https://github.com/airspy/airspyhf at commit "
                 "24fe8ffcb00b14f827268bbad89ae1392de055e5 (BSD-3-Clause), both unmodified, and in its static "
                 "builds libusb 1.0.30 from https://github.com/libusb/libusb, unmodified, under the GNU Lesser "
                 "General Public License, version 2.1 or later. The module's source carries the copies it is "
                 "built from in third_party/. Their copyright notices and licences:\n\n"
                 "==== libairspy ====\n\n" +
                 libairspy_notices + "\n\n==== libairspyhf ====\n\n" + libairspyhf_notices +
                 "\n\n==== libusb ====\n\n" + libusb_notices + "\n\n==== fern-airspy ====\n\n" + module_licence);
}

int module() {
    for (int fd : {STDIN_FILENO, STDOUT_FILENO, 3}) {
        if (::fcntl(fd, F_GETFD) < 0) {
            std::fprintf(stderr,
                         "fern-airspy: fd %d is not open. --fernsdr-module is how FernSDR runs this program, "
                         "with commands on fd 0, samples on fd 1 and events on fd 3.\n",
                         fd);
            return exit_status::usage;
        }
    }
    std::signal(SIGPIPE, SIG_IGN);

    // SIGTERM and friends arrive through a signalfd, so that the session
    // loop sees them like any other input. Threads started later, the vendor
    // libraries' too, inherit the mask.
    sigset_t stop_signals;
    sigemptyset(&stop_signals);
    sigaddset(&stop_signals, SIGTERM);
    sigaddset(&stop_signals, SIGINT);
    sigaddset(&stop_signals, SIGHUP);
    if (pthread_sigmask(SIG_BLOCK, &stop_signals, nullptr) != 0) {
        log_line("could not block the stop signals");
        return exit_status::internal;
    }
    const int signal_fd = signalfd(-1, &stop_signals, SFD_CLOEXEC);
    if (signal_fd < 0) {
        log_line("could not create a signalfd: %s", std::strerror(errno));
        return exit_status::internal;
    }

    log_line("fern-airspy %s with %s", module_version(), library_versions().c_str());
    AirspyBackend backend;
    SessionIo io;
    io.stop = signal_fd;
    const SessionResult result = run_session(backend, io);
    if (!result.clean)
        _exit(result.status);
    ::close(signal_fd);
    return result.status;
}

}  // namespace

int main(int argc, char** argv) {
    const int n = argc - 1;
    const std::string_view first = n >= 1 ? argv[1] : "";
    if (n == 1 && first == "--describe")
        return describe();
    if (n == 1 && first == "--list-devices")
        return list();
    if (n == 1 && first == "--notices")
        return notices();
    if (n == 2 && first == "--fernsdr-module") {
        if (std::string_view(argv[2]) == "1")
            return module();
        std::fprintf(stderr, "fern-airspy: this module speaks module API 1, not %s\n", argv[2]);
        return exit_status::usage;
    }
    if (n == 1 && first == "--version")
        return print(std::string("fern-airspy ") + module_version() + "\n");
    if (n == 1 && (first == "--help" || first == "-h"))
        return print(usage_text);
    std::fputs(usage_text, stderr);
    return exit_status::usage;
}
