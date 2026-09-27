#include "robotkit_device_host.hpp"
#include "robotkit_device_serial_endpoint.hpp"
#include "motionkit.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <limits>
#include <regex>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); std::abort(); } } while (false)

namespace device = robotkit::device;

std::uint64_t deployment_integer(const std::string &source, const char *field) {
    const std::regex pattern(std::string("\"") + field + "\"\\s*:\\s*([0-9]+)");
    std::smatch match;
    CHECK(std::regex_search(source, match, pattern));
    return std::stoull(match[1].str());
}

rk_robot_runtime_blueprint blueprint() {
    rk_robot_runtime_blueprint value{};
    value.struct_size = sizeof(value);
    value.revision = 1;
    value.joint_count = 1;
    value.link_count = 2;
    value.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (auto &link : value.links) {
        link.mass = 1.0;
        link.inertia_tensor[0] = 1.0;
        link.inertia_tensor[4] = 1.0;
        link.inertia_tensor[8] = 1.0;
    }
    value.joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -10.0, 10.0, 10.0};
    value.joints[0].parent_frame_rotation[3] = 1.0;
    value.joints[0].child_frame_rotation[3] = 1.0;
    value.joints[0].axis[2] = 1.0;
    value.joints[0].max_acceleration = 5.0;
    value.joints[0].max_velocity = 1.0;
    return value;
}

int main(int argc, char **argv) {
    CHECK(argc == 3);
    std::ifstream bench_file(argv[2]);
    CHECK(bench_file.good());
    const std::string bench((std::istreambuf_iterator<char>(bench_file)),
        std::istreambuf_iterator<char>());
    CHECK(deployment_integer(bench, "schemaVersion") == 2);
    const auto bench_baud = static_cast<unsigned>(deployment_integer(bench, "baud"));
    const auto bench_period = std::chrono::nanoseconds(
        deployment_integer(bench, "owner_period_ns"));
    const auto bench_allowance = std::chrono::nanoseconds(
        deployment_integer(bench, "processing_allowance_ns"));
    // The bench model has two drive joints; qualify the deployed joint count.
    CHECK(robotkit::DeviceSerialEndpoint::minimum_owner_period_ns(bench_baud, 2,
        bench_allowance) <= static_cast<std::uint64_t>(bench_period.count()));
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
    CHECK(!device::HostLink::open(slave, 115200, fingerprint, 1));
    for (std::size_t i = 0; i < fingerprint.size(); ++i)
        fingerprint[i] = static_cast<std::uint8_t>(i);
    {
        auto link = device::HostLink::open(slave, 115200, fingerprint, 1);
        CHECK(link && link->ready() && link->last_session_status() == 1);
        device::HostState state{};
        CHECK(link->read_state(state));
        CHECK(state.header.safety == 2 && state.header.accepted_sequence == 0 && state.header.timestamp_ns == 0);
        CHECK(link->send_command(4));
        CHECK(link->read_state(state));
        CHECK(state.header.safety == 0 && state.header.accepted_sequence == 1);
        const std::array<device::HostTarget, 1> lossy{{{0, 2, 1.1}}};
        CHECK(!link->send_targets(lossy, 1e-9));
        CHECK(!link->send_targets(lossy, -1.0));
        const std::array<device::HostTarget, 1> too_large{{{0, 2, std::numeric_limits<double>::max()}}};
        CHECK(!link->send_targets(too_large, 1.0));
        const std::array<device::HostTarget, 1> not_finite{{{0, 2, std::numeric_limits<double>::quiet_NaN()}}};
        CHECK(!link->send_targets(not_finite, 1.0));
        const std::array<device::HostTarget, 1> too_small{{{0, 2, std::numeric_limits<double>::denorm_min()}}};
        CHECK(!link->send_targets(too_small, 1.0));
        CHECK(link->last_sent_sequence() == 1);
        CHECK(link->send_targets(lossy, 1e-6));
        CHECK(link->read_state(state));
        CHECK(state.header.accepted_sequence == 2 && state.joints[0].velocity == -2.0f);
        const std::array<device::HostTarget, 1> rejected_target{{{0, 2, 2.0}}};
        CHECK(link->send_targets(rejected_target, 0.0));
        CHECK(::write(control[1], "F", 1) == 1);
        CHECK(link->read_state(state));
        CHECK(state.header.fault == 1 && state.header.accepted_sequence == 2);
        CHECK(::write(control[1], "W", 1) == 1);
        CHECK(link->read_state(state)); // Device-local watchdog expires without more commands.
        CHECK(state.header.safety == 2 && state.header.accepted_sequence == 2);
        const auto previous_session = link->session_id();
        CHECK(previous_session != 0 && link->last_sent_sequence() == 3);
        CHECK(link->begin_session());
        CHECK(link->session_id() != previous_session && link->last_sent_sequence() == 0);
        auto endpoint = robotkit::DeviceSerialEndpoint::attach(std::move(link), 1, 1e-6,
            std::chrono::milliseconds(20));
        CHECK(endpoint && endpoint->initial_safety_state() == RK_SAFETY_EMERGENCY_STOP);
        CHECK(endpoint->supports_trajectory_queue());
        robotkit::RobotRuntime runtime(blueprint(), endpoint, std::chrono::milliseconds(20));
        rk_robot_state runtime_state{};
        CHECK(runtime.publish_sample(0) == RK_OK);
        CHECK(runtime.snapshot(runtime_state) == RK_OK);
        CHECK(runtime_state.safety == RK_SAFETY_EMERGENCY_STOP && runtime_state.joint_count == 1);
        rk_robot_command command{};
        command.struct_size = sizeof(command);
        command.sequence = 1;
        command.kind = RK_COMMAND_RESET_SAFETY;
        CHECK(runtime.submit(command) == RK_OK);
        CHECK(runtime.apply_pending_commands() == RK_OK);
        CHECK(runtime.publish_sample(0) == RK_OK);
        CHECK(runtime.snapshot(runtime_state) == RK_OK && runtime_state.safety == RK_SAFETY_READY);
        command.sequence = 2;
        command.kind = RK_COMMAND_JOINT_TARGETS;
        command.target_count = 1;
        command.targets[0] = {0, RK_TARGET_VELOCITY, 1.1, 0.0, 0.0};
        CHECK(runtime.submit(command) == RK_OK);
        CHECK(runtime.apply_pending_commands() == RK_OK);
        CHECK(runtime.publish_sample(0) == RK_OK);
        CHECK(runtime.snapshot(runtime_state) == RK_OK);
        CHECK(runtime_state.source_timestamp_ns > 0 && runtime_state.velocity[0] == -2.0);
        CHECK(endpoint->apply(command) == RK_ERROR_STALE_COMMAND);
        command.sequence = 3;
        command.kind = RK_COMMAND_NONE;
        command.target_count = 0;
        CHECK(endpoint->apply(command) == RK_OK);
        CHECK(endpoint->sample(0, runtime_state) == RK_OK);
        CHECK(runtime_state.safety == RK_SAFETY_READY);
    }
    {
        auto link = device::HostLink::open(slave, bench_baud, fingerprint, 1);
        CHECK(link && link->ready());
        auto endpoint = robotkit::DeviceSerialEndpoint::attach(std::move(link), 1, 1e-6,
            bench_period, bench_allowance);
        CHECK(endpoint && endpoint->supports_trajectory_queue());
        robotkit::RobotRuntime runtime(blueprint(), endpoint, bench_period);
        CHECK(runtime.publish_sample(0) == RK_OK);
        rk_robot_command reset{};
        reset.struct_size = sizeof(reset);
        reset.sequence = 1;
        reset.kind = RK_COMMAND_RESET_SAFETY;
        CHECK(runtime.submit(reset) == RK_OK);
        CHECK(runtime.apply_pending_commands() == RK_OK);
        CHECK(runtime.publish_sample(bench_period.count()) == RK_OK);

        mk_state_to_state_request request{};
        request.struct_size = sizeof(request);
        request.joint_count = 1;
        request.synchronization = MK_SYNCHRONIZATION_TIME;
        request.target_position[0] = 0.1;
        request.max_velocity[0] = 1.0;
        request.max_acceleration[0] = 5.0;
        request.max_jerk[0] = 50.0;
        mk_trajectory_handle generated{};
        int32_t ruckig_result = 0;
        CHECK(mk_generate_state_to_state(&request, &generated, &ruckig_result) == MK_OK);
        uint32_t segment_count = 0;
        CHECK(mk_trajectory_segment_count(generated, &segment_count) == MK_OK);
        CHECK(segment_count > 0 && segment_count <= RK_MAX_TRAJECTORY_SEGMENTS);
        rk_plan_submission plan{};
        plan.struct_size = sizeof(plan);
        plan.sequence = 2;
        plan.plan_id = 1;
        plan.model_revision = 1;
        plan.required_capabilities = RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE;
        plan.ends_at_rest = 1;
        plan.segments.struct_size = sizeof(plan.segments);
        plan.segments.segment_count = segment_count;
        for (uint32_t index = 0; index < segment_count; ++index) {
            mk_segment source{};
            source.struct_size = sizeof(source);
            CHECK(mk_trajectory_get_segment(generated, index, &source) == MK_OK);
            auto &target = plan.segments.segments[index];
            target.time_from_start_ns = static_cast<uint64_t>(source.t0_ns);
            target.duration_ns = static_cast<uint64_t>(source.duration_ns);
            target.degree = source.degree;
            target.joint_count = source.joint_count;
            for (uint32_t degree = 0; degree <= source.degree; ++degree)
                target.coefficients[0].value[degree] = source.coefficients[0].value[degree];
        }
        CHECK(runtime.submit_plan(plan) == RK_OK);
        uint64_t timestamp = static_cast<uint64_t>(bench_period.count() * 2);
        rk_robot_state state{};
        auto on_path = [&] {
            CHECK(runtime.snapshot(state) == RK_OK);
            CHECK(state.trajectory_active == 1);
            mk_trajectory_state reference{};
            reference.struct_size = sizeof(reference);
            CHECK(mk_trajectory_evaluate(generated,
                static_cast<int64_t>(state.trajectory_time_ns), &reference) == MK_OK);
            CHECK(std::abs(state.position[0] - reference.position[0]) < 1e-6);
        };
        for (int cycle = 0; cycle < 4; ++cycle) {
            CHECK(runtime.apply_pending_commands() == RK_OK);
            CHECK(runtime.publish_sample(timestamp) == RK_OK);
            on_path();
            timestamp += static_cast<uint64_t>(bench_period.count());
        }
        CHECK(state.trajectory_active == 1 && state.position[0] >= 0.0 && state.position[0] <= 0.1);
        rk_robot_command lifecycle{};
        lifecycle.struct_size = sizeof(lifecycle);
        lifecycle.sequence = 3;
        lifecycle.kind = RK_COMMAND_HOLD;
        CHECK(runtime.submit(lifecycle) == RK_OK);
        for (int cycle = 0; cycle < 3; ++cycle) {
            CHECK(runtime.apply_pending_commands() == RK_OK);
            CHECK(runtime.publish_sample(timestamp) == RK_OK);
            on_path();
            timestamp += static_cast<uint64_t>(bench_period.count());
        }
        CHECK(state.trajectory_active == 1 && state.position[0] >= 0.0 && state.position[0] <= 0.1);
        lifecycle.sequence = 4;
        lifecycle.kind = RK_COMMAND_RESUME;
        CHECK(runtime.submit(lifecycle) == RK_OK);
        for (int cycle = 0; cycle < 4; ++cycle) {
            CHECK(runtime.apply_pending_commands() == RK_OK);
            CHECK(runtime.publish_sample(timestamp) == RK_OK);
            on_path();
            timestamp += static_cast<uint64_t>(bench_period.count());
        }
        CHECK(state.trajectory_active == 1 && state.position[0] >= 0.0 && state.position[0] <= 0.1);
        mk_trajectory_destroy(generated);
    }
    constexpr char fingerprint_hex[] = "000102030405060708090a0b0c0d0e0f";
    auto layout = blueprint();
    rk_robot_runtime handle = RK_INVALID_ROBOT_RUNTIME;
    CHECK(rk_robot_runtime_create_serial(&layout, slave, 115200,
        "00000000000000000000000000000000", 1e-6, &handle) == RK_ERROR_INVALID_ARGUMENT);
    CHECK(rk_robot_runtime_create_serial(&layout, slave, 115200,
        "not-a-fingerprint", 1e-6, &handle) == RK_ERROR_INVALID_ARGUMENT);
    CHECK(rk_robot_runtime_create_serial(&layout, slave, 115200,
        fingerprint_hex, -1.0, &handle) == RK_ERROR_INVALID_ARGUMENT);
    CHECK(rk_robot_runtime_create_serial(&layout, slave, 115200,
        fingerprint_hex, 1e-6, &handle) == RK_OK);
    CHECK(handle != RK_INVALID_ROBOT_RUNTIME);
    rk_robot_capabilities capabilities{};
    capabilities.struct_size = sizeof(capabilities);
    CHECK(rk_robot_runtime_capabilities(handle, &capabilities) == RK_OK);
    CHECK(capabilities.supports_trajectory_queue == 1);
    rk_robot_runtime_destroy(handle);
    handle = RK_INVALID_ROBOT_RUNTIME;
    layout.owner_period_ns = 2'000'000;
    CHECK(rk_robot_runtime_create_serial(&layout, slave, 115200,
        fingerprint_hex, 1e-6, &handle) == RK_OK);
    CHECK(rk_robot_runtime_capabilities(handle, &capabilities) == RK_OK);
    CHECK(capabilities.supports_trajectory_queue == 0);
    rk_robot_runtime_destroy(handle);
    handle = RK_INVALID_ROBOT_RUNTIME;
    layout.owner_period_ns = 5'000'000;
    CHECK(rk_robot_runtime_create_serial(&layout, slave, 921600,
        fingerprint_hex, 1e-6, &handle) == RK_OK);
    CHECK(rk_robot_runtime_capabilities(handle, &capabilities) == RK_OK);
    CHECK(capabilities.supports_trajectory_queue == 1);
    rk_robot_runtime_destroy(handle);
    handle = RK_INVALID_ROBOT_RUNTIME;
    layout.owner_period_ns = 20'000'000;
    CHECK(rk_robot_runtime_create_serial(&layout, slave, 115200,
        fingerprint_hex, 1e-6, &handle) == RK_OK);
    CHECK(rk_robot_runtime_capabilities(handle, &capabilities) == RK_OK);
    CHECK(capabilities.supports_trajectory_queue == 1);
    rk_robot_runtime_destroy(handle);
    handle = RK_INVALID_ROBOT_RUNTIME;
    CHECK(rk_robot_runtime_create_serial(&layout, slave, 115200,
        "010102030405060708090a0b0c0d0e0f", 1e-6, &handle) == RK_ERROR_MODEL_MISMATCH);
    CHECK(handle == RK_INVALID_ROBOT_RUNTIME);
    ::close(control[1]);
    int status = 0;
    CHECK(::waitpid(child, &status, 0) == child);
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    ::close(master);
}
