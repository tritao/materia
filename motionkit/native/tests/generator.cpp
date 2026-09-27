#include "motionkit.h"
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

void near(double actual, double expected, double tolerance = 1e-9) {
    if (std::abs(actual - expected) > tolerance)
        std::fprintf(stderr, "mismatch: actual %.17g expected %.17g tolerance %.3g\n",
            actual, expected, tolerance);
    assert(std::abs(actual - expected) <= tolerance);
}

mk_state_to_state_request standard(uint32_t joints = 1) {
    mk_state_to_state_request request{};
    request.struct_size = sizeof(request);
    request.joint_count = joints;
    request.synchronization = MK_SYNCHRONIZATION_TIME;
    for (uint32_t joint = 0; joint < joints; ++joint) {
        request.target_position[joint] = 1.0;
        request.max_velocity[joint] = 1.0;
        request.max_acceleration[joint] = 2.0;
        request.max_jerk[joint] = 5.0;
    }
    return request;
}

ruckig::Trajectory<ruckig::DynamicDOFs> reference_for(const mk_state_to_state_request &request) {
    const auto joints = request.joint_count;
    ruckig::InputParameter<ruckig::DynamicDOFs> input(joints);
    input.control_interface = request.control_mode == MK_CONTROL_VELOCITY_STOP ?
        ruckig::ControlInterface::Velocity : ruckig::ControlInterface::Position;
    input.synchronization = ruckig::Synchronization::Time;
    for (uint32_t joint = 0; joint < joints; ++joint) {
        input.current_position[joint] = request.current_position[joint];
        input.current_velocity[joint] = request.current_velocity[joint];
        input.current_acceleration[joint] = request.current_acceleration[joint];
        input.target_position[joint] = request.target_position[joint];
        input.target_velocity[joint] = request.target_velocity[joint];
        input.target_acceleration[joint] = request.target_acceleration[joint];
        input.max_velocity[joint] = request.max_velocity[joint];
        input.max_acceleration[joint] = request.max_acceleration[joint];
        input.max_jerk[joint] = request.max_jerk[joint];
    }
    ruckig::Ruckig<ruckig::DynamicDOFs> calculator(joints);
    ruckig::Trajectory<ruckig::DynamicDOFs> reference(joints);
    assert(calculator.calculate(input, reference) == ruckig::Result::Working);
    return reference;
}

mk_validation_report validate_all(mk_trajectory_handle trajectory,
                                  const mk_state_to_state_request &request,
                                  double lower = -10.0, double upper = 10.0) {
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = request.joint_count;
    for (uint32_t joint = 0; joint < request.joint_count; ++joint) {
        limits.position_claimed[joint] = 1;
        limits.position_lower[joint] = lower;
        limits.position_upper[joint] = upper;
        limits.max_velocity[joint] = request.max_velocity[joint];
        limits.max_acceleration[joint] = request.max_acceleration[joint];
        limits.max_jerk[joint] = request.max_jerk[joint];
    }
    for (auto &jump : limits.max_continuity_jump) jump = 1e-5;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    for (size_t index = 0; index < MK_CHECK_COUNT; ++index)
        assert(index == MK_CHECK_TASK_SPACE
            ? report.checks[index].status == MK_CHECK_UNCHECKED
            : report.checks[index].status != MK_CHECK_UNCHECKED);
    return report;
}

mk_trajectory_handle compare_with_ruckig(const mk_state_to_state_request &request) {
    auto reference = reference_for(request);
    mk_trajectory_handle trajectory{};
    int32_t result = -999;
    assert(mk_generate_state_to_state(&request, &trajectory, &result) == MK_OK);
    assert(result == ruckig::Result::Working && trajectory.id != 0);
    int64_t duration_ns = 0;
    assert(mk_trajectory_duration_ns(trajectory, &duration_ns) == MK_OK);
    near(static_cast<double>(duration_ns) * 1e-9, reference.get_duration(), 1.1e-9);
    uint32_t segment_count = 0;
    assert(mk_trajectory_segment_count(trajectory, &segment_count) == MK_OK);
    assert(segment_count > 0);
    std::vector<double> p(request.joint_count), v(request.joint_count), a(request.joint_count);
    const auto last_tick = static_cast<int>(std::floor(reference.get_duration() * 1000.0));
    for (int tick = 0; tick <= last_tick; ++tick) {
        const double seconds = tick * 0.001;
        reference.at_time(seconds, p, v, a);
        mk_trajectory_state state{};
        state.struct_size = sizeof(state);
        assert(mk_trajectory_evaluate(trajectory, std::llround(seconds * 1e9), &state) == MK_OK);
        for (uint32_t joint = 0; joint < request.joint_count; ++joint) {
            near(state.position[joint], p[joint]);
            near(state.velocity[joint], v[joint]);
            near(state.acceleration[joint], a[joint]);
        }
    }
    const auto profiles = reference.get_profiles();
    for (uint32_t joint = 0; joint < request.joint_count; ++joint) {
        const auto &profile = profiles.front()[joint];
        std::vector<double> boundaries{profile.brake.t[0], profile.brake.duration,
            reference.get_duration()};
        for (double phase_end : profile.t_sum)
            boundaries.push_back(profile.brake.duration + phase_end);
        for (double boundary : boundaries) {
            const auto rounded = std::llround(boundary * 1e9);
            for (int offset = -1; offset <= 1; ++offset) {
                const auto time_ns = rounded + offset;
                if (time_ns < 0 || time_ns > duration_ns) continue;
                const double seconds = static_cast<double>(time_ns) * 1e-9;
                reference.at_time(seconds, p, v, a);
                mk_trajectory_state state{};
                state.struct_size = sizeof(state);
                assert(mk_trajectory_evaluate(trajectory, time_ns, &state) == MK_OK);
                const double velocity_bound = std::max(request.max_velocity[joint],
                    std::abs(request.current_velocity[joint])) * 0.5e-9 + 1e-12;
                const double acceleration_bound = std::max(request.max_acceleration[joint],
                    std::abs(request.current_acceleration[joint])) * 0.5e-9 + 1e-12;
                const double jerk_bound = request.max_jerk[joint] * 0.5e-9 + 1e-12;
                near(state.position[joint], p[joint], velocity_bound);
                near(state.velocity[joint], v[joint], acceleration_bound);
                near(state.acceleration[joint], a[joint], jerk_bound);
            }
        }
    }
    return trajectory;
}

void expect_all_pass(mk_trajectory_handle trajectory, const mk_state_to_state_request &request) {
    const auto report = validate_all(trajectory, request);
    for (size_t index = 0; index < MK_CHECK_TASK_SPACE; ++index) {
        const auto &check = report.checks[index];
        if (check.status != MK_CHECK_PASSED)
            std::fprintf(stderr, "check %zu status %u value %.17g limit %.17g time %.17g\n",
                index, check.status, check.value, check.limit, check.time_seconds);
        assert(check.status == MK_CHECK_PASSED);
    }
}

void baseline_and_multiaxis() {
    auto request = standard(2);
    request.target_position[1] = -0.3;
    request.max_velocity[1] = 0.6;
    request.max_acceleration[1] = 1.5;
    request.max_jerk[1] = 4.0;
    const auto trajectory = compare_with_ruckig(request);
    expect_all_pass(trajectory, request);
    mk_trajectory_destroy(trajectory);
}

void moving_start_and_reversal() {
    auto request = standard();
    request.current_velocity[0] = 0.35;
    request.current_acceleration[0] = 0.4;
    auto trajectory = compare_with_ruckig(request);
    const auto rounded_boundary_report = validate_all(trajectory, request);
    const auto &acceleration_check = rounded_boundary_report.checks[MK_CHECK_ACCELERATION];
    assert(acceleration_check.margin < 0.0);
    assert(-acceleration_check.margin <= acceleration_check.tolerance);
    expect_all_pass(trajectory, request);
    mk_trajectory_destroy(trajectory);
    request.current_velocity[0] = -0.3;
    request.current_acceleration[0] = 0.0;
    trajectory = compare_with_ruckig(request);
    expect_all_pass(trajectory, request);
    mk_trajectory_destroy(trajectory);
}

void retarget_and_stop() {
    auto first = standard();
    first.target_position[0] = 2.0;
    auto trajectory = compare_with_ruckig(first);
    expect_all_pass(trajectory, first);
    mk_trajectory_state moving{};
    moving.struct_size = sizeof(moving);
    assert(mk_trajectory_evaluate(trajectory, 250'000'000, &moving) == MK_OK);
    mk_trajectory_destroy(trajectory);
    auto retarget = standard();
    retarget.current_position[0] = moving.position[0];
    retarget.current_velocity[0] = moving.velocity[0];
    retarget.current_acceleration[0] = moving.acceleration[0];
    retarget.target_position[0] = -0.2;
    trajectory = compare_with_ruckig(retarget);
    expect_all_pass(trajectory, retarget);
    mk_trajectory_destroy(trajectory);
    auto stop = retarget;
    stop.control_mode = MK_CONTROL_VELOCITY_STOP;
    stop.target_position[0] = 1000.0; // Position is deliberately free.
    trajectory = compare_with_ruckig(stop);
    expect_all_pass(trajectory, stop);
    int64_t duration_ns = 0;
    assert(mk_trajectory_duration_ns(trajectory, &duration_ns) == MK_OK);
    mk_trajectory_state final_state{};
    final_state.struct_size = sizeof(final_state);
    assert(mk_trajectory_evaluate(trajectory, duration_ns, &final_state) == MK_OK);
    near(final_state.velocity[0], 0.0, 1e-8);
    near(final_state.acceleration[0], 0.0, 1e-8);
    assert(std::abs(final_state.position[0] - 1000.0) > 1.0);
    mk_trajectory_destroy(trajectory);
}

void brake_and_travel_limit() {
    auto request = standard();
    request.current_velocity[0] = 1.5;
    assert(reference_for(request).get_profiles().front().front().brake.duration > 0.0);
    auto trajectory = compare_with_ruckig(request);
    const auto brake_report = validate_all(trajectory, request);
    assert(brake_report.checks[MK_CHECK_VELOCITY].status == MK_CHECK_FAILED);
    mk_trajectory_destroy(trajectory);
    request = standard();
    request.current_position[0] = 0.95;
    request.current_velocity[0] = 0.5;
    request.target_position[0] = 0.9;
    trajectory = compare_with_ruckig(request);
    expect_all_pass(trajectory, request);
    const auto travel_report = validate_all(trajectory, request, -1.0, 1.0);
    assert(travel_report.checks[MK_CHECK_POSITION].status == MK_CHECK_FAILED);
    assert(travel_report.checks[MK_CHECK_POSITION].time_seconds > 0.0);
    mk_trajectory_destroy(trajectory);
}

void invalid_input_maps_result() {
    auto request = standard();
    request.max_jerk[0] = 0.0;
    mk_trajectory_handle trajectory{};
    int32_t result = 0;
    assert(mk_generate_state_to_state(&request, &trajectory, &result) == MK_ERROR_INVALID_ARGUMENT);
    assert(result == ruckig::Result::ErrorInvalidInput && trajectory.id == 0);
    request = standard();
    request.target_velocity[0] = 2.0;
    assert(mk_generate_state_to_state(&request, &trajectory, &result) == MK_ERROR_INVALID_ARGUMENT);
    assert(result == ruckig::Result::ErrorInvalidInput && trajectory.id == 0);
}

void unchanged_state_has_one_nanosecond_segment() {
    auto request = standard();
    request.target_position[0] = 0.0;
    mk_trajectory_handle trajectory{};
    int32_t result = -999;
    assert(mk_generate_state_to_state(&request, &trajectory, &result) == MK_OK);
    int64_t duration = 0;
    assert(mk_trajectory_duration_ns(trajectory, &duration) == MK_OK);
    assert(duration == 1);
    const auto report = validate_all(trajectory, request);
    for (size_t index = 0; index < MK_CHECK_COUNT; ++index)
        assert(report.checks[index].status == (index == MK_CHECK_TASK_SPACE
            ? MK_CHECK_UNCHECKED : MK_CHECK_PASSED));
    mk_trajectory_destroy(trajectory);
}

void zero_origin_travel_limit_is_translation_invariant() {
    double reference_tolerance = 0.0;
    for (double offset : {0.0, 1000.0}) {
        auto request = standard();
        request.current_position[0] = offset + 1.0;
        request.target_position[0] = offset;
        mk_trajectory_handle trajectory{};
        int32_t result = -999;
        assert(mk_generate_state_to_state(&request, &trajectory, &result) == MK_OK);
        mk_limits limits{};
        limits.struct_size = sizeof(limits);
        limits.joint_count = 1;
        limits.position_claimed[0] = 1;
        limits.position_lower[0] = offset;
        limits.position_upper[0] = offset + 2.0;
        mk_validation_report report{};
        report.struct_size = sizeof(report);
        assert(mk_validate(trajectory, &limits, &report) == MK_OK);
        const auto &check = report.checks[MK_CHECK_POSITION];
        assert(check.status == MK_CHECK_PASSED);
        near(check.limit, offset);
        assert(check.tolerance > 0.0);
        if (offset == 0.0) reference_tolerance = check.tolerance;
        else near(check.tolerance, reference_tolerance, 1e-15);
        mk_trajectory_destroy(trajectory);
    }
}

} // namespace

int main() {
    baseline_and_multiaxis();
    moving_start_and_reversal();
    retarget_and_stop();
    brake_and_travel_limit();
    invalid_input_maps_result();
    unchanged_state_has_one_nanosecond_segment();
    zero_origin_travel_limit_is_translation_invariant();
}
