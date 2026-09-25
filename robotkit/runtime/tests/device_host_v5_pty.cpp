#include "robotkit_device_host_v5.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <limits>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); std::abort(); } } while (false)

namespace v5 = robotkit::v5;

int main(int argc, char **argv) {
    CHECK(argc == 2);
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    CHECK(master >= 0);
    CHECK(::grantpt(master) == 0 && ::unlockpt(master) == 0);
    const char *slave = ::ptsname(master);
    CHECK(slave != nullptr);
    int control[2]{};
    CHECK(::pipe(control) == 0);
    CHECK(::fcntl(control[0], F_SETFL, O_NONBLOCK) == 0);
    const pid_t child = ::fork();
    CHECK(child >= 0);
    if (child == 0) {
        ::close(control[1]);
        const auto master_arg = std::to_string(master);
        const auto control_arg = std::to_string(control[0]);
        ::execl(argv[1], argv[1], master_arg.c_str(), control_arg.c_str(), nullptr);
        std::perror("exec Rust device simulator");
        _exit(127);
    }
    ::close(control[0]);
    std::array<std::uint8_t, 16> fingerprint{};
    for (std::size_t i = 0; i < fingerprint.size(); ++i)
        fingerprint[i] = static_cast<std::uint8_t>(i);
    {
        auto link = v5::HostLink::open(slave, 115200, fingerprint, 1);
        CHECK(link && link->ready() && link->last_session_status() == 1);
        CHECK(link->send_command(4));
        v5::HostState state{};
        CHECK(link->read_state(state));
        CHECK(state.header.safety == 0 && state.header.accepted_sequence == 1);
        const std::array<v5::HostTarget, 1> lossy{{{0, 2, 1.1}}};
        CHECK(!link->send_targets(lossy, 1e-9));
        CHECK(!link->send_targets(lossy, -1.0));
        const std::array<v5::HostTarget, 1> too_large{{{0, 2, std::numeric_limits<double>::max()}}};
        CHECK(!link->send_targets(too_large, 1.0));
        const std::array<v5::HostTarget, 1> not_finite{{{0, 2, std::numeric_limits<double>::quiet_NaN()}}};
        CHECK(!link->send_targets(not_finite, 1.0));
        const std::array<v5::HostTarget, 1> too_small{{{0, 2, std::numeric_limits<double>::denorm_min()}}};
        CHECK(!link->send_targets(too_small, 1.0));
        CHECK(link->last_sent_sequence() == 1);
        CHECK(link->send_targets(lossy, 1e-6));
        CHECK(link->read_state(state));
        CHECK(state.header.accepted_sequence == 2 && state.joints[0].velocity == -2.0f);
        CHECK(::write(control[1], "W", 1) == 1);
        CHECK(link->read_state(state)); // Device-local watchdog expires without more commands.
        CHECK(state.header.safety == 2 && state.header.accepted_sequence == 2);
        const auto previous_session = link->session_id();
        CHECK(previous_session != 0 && link->last_sent_sequence() == 2);
        CHECK(link->begin_session());
        CHECK(link->session_id() != previous_session && link->last_sent_sequence() == 0);
        CHECK(link->send_command(4) && link->read_state(state));
        CHECK(state.header.accepted_sequence == 1 && state.header.session == link->session_id());
    }
    auto wrong = fingerprint;
    wrong[0] ^= 1;
    const int rejected_fd = ::open(slave, O_RDWR | O_NOCTTY | O_NONBLOCK);
    CHECK(rejected_fd >= 0);
    v5::HostLink rejected(rejected_fd, true, 115200, wrong, 1);
    CHECK(!rejected.begin_session() && rejected.last_session_status() == 2);
    ::close(control[1]);
    int status = 0;
    CHECK(::waitpid(child, &status, 0) == child);
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    ::close(master);
}
