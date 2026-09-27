#include "robotkit_device_serial_endpoint.hpp"
#include "rkd6_endpoint.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

using namespace robotkit;
using namespace std::chrono_literals;

static std::uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main(int argc, char **argv) {
    assert(argc == 2);
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0 && ::grantpt(master) == 0 && ::unlockpt(master) == 0);
    const char *slave = ::ptsname(master);
    assert(slave);
    int control[2]{};
    assert(::pipe(control) == 0);
    assert(::fcntl(control[0], F_SETFL, O_NONBLOCK) == 0);
    const auto child = ::fork();
    assert(child >= 0);
    if (child == 0) {
        ::close(control[1]);
        const auto m = std::to_string(master), c = std::to_string(control[0]);
        ::execl(argv[1], argv[1], m.c_str(), c.c_str(), nullptr);
        _exit(127);
    }
    ::close(control[0]);
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = 10;
    blueprint.joints[0].max_acceleration = 10;
    std::array<std::uint8_t, 16> fingerprint{};
    for (std::size_t i = 0; i < fingerprint.size(); ++i) fingerprint[i] = i;
    auto endpoint = DeviceSerialEndpoint::open(slave, 921'600, blueprint,
        fingerprint, 1e-5, 40'000, 500'000'000, 30'000'000);
    assert(endpoint);
    rk_robot_state state{};
    for (int i = 0; i < 40; ++i) {
        endpoint->sample(now_ns(), state);
        std::this_thread::sleep_for(10ms);
    }
    rk_plan_submission plan{};
    plan.struct_size = sizeof(plan);
    plan.plan_id = 1;
    plan.ends_at_rest = 1;
    plan.segments.segment_count = 1;
    auto &segment = plan.segments.segments[0];
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 0.1;
    assert(endpoint->submit_device_plan(plan, 0, now_ns(), 20'000'000, blueprint) == RK_OK);
    bool moved = false;
    for (int i = 0; i < 180; ++i) {
        if (endpoint->sample(now_ns(), state) == RK_OK && state.position[0] > 0.05)
            moved = true;
        std::this_thread::sleep_for(10ms);
    }
    if (!moved || state.safety != RK_SAFETY_READY)
        std::fprintf(stderr, "serial smoke: moved=%d position=%f safety=%u diagnostic=%d\n",
            moved, state.position[0], state.safety, endpoint->diagnostic_code());
    assert(moved && state.safety == RK_SAFETY_READY);
    endpoint.reset();
    ::close(control[1]);
    ::close(master);
    int status = 0;
    assert(::waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0);
}
