#include "motionkit.hpp"

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace motionkit {
namespace {

bool valid_request(const mk_state_to_state_request &request) {
    if (request.struct_size < sizeof(request) || request.joint_count == 0 ||
        request.joint_count > MK_MAX_JOINTS ||
        request.synchronization != MK_SYNCHRONIZATION_TIME ||
        (request.control_mode != MK_CONTROL_POSITION &&
         request.control_mode != MK_CONTROL_VELOCITY_STOP))
        return false;
    for (uint32_t joint = 0; joint < request.joint_count; ++joint) {
        if (!std::isfinite(request.current_position[joint]) ||
            !std::isfinite(request.current_velocity[joint]) ||
            !std::isfinite(request.current_acceleration[joint]) ||
            (request.control_mode == MK_CONTROL_POSITION &&
             !std::isfinite(request.target_position[joint])) ||
            !std::isfinite(request.target_velocity[joint]) ||
            !std::isfinite(request.target_acceleration[joint]) ||
            !std::isfinite(request.max_velocity[joint]) ||
            !std::isfinite(request.max_acceleration[joint]) ||
            !std::isfinite(request.max_jerk[joint]) ||
            request.max_velocity[joint] <= 0.0 ||
            request.max_acceleration[joint] <= 0.0 ||
            request.max_jerk[joint] <= 0.0)
            return false;
        if (request.control_mode == MK_CONTROL_VELOCITY_STOP &&
            (request.target_velocity[joint] != 0.0 ||
             request.target_acceleration[joint] != 0.0))
            return false;
    }
    return true;
}

mk_result map_result(ruckig::Result result) {
    switch (result) {
    case ruckig::Result::Working:
    case ruckig::Result::Finished: return MK_OK;
    case ruckig::Result::ErrorInvalidInput:
    case ruckig::Result::ErrorZeroLimits: return MK_ERROR_INVALID_ARGUMENT;
    case ruckig::Result::ErrorPositionalLimits: return MK_ERROR_LIMIT;
    default: return MK_ERROR_GENERATION;
    }
}

} // namespace

mk_result generate(const mk_state_to_state_request &request, Trajectory &trajectory,
                   int32_t &ruckig_result) {
    if (!valid_request(request)) {
        ruckig_result = ruckig::Result::ErrorInvalidInput;
        return MK_ERROR_INVALID_ARGUMENT;
    }
    const auto dofs = request.joint_count;
    ruckig::InputParameter<ruckig::DynamicDOFs> input(dofs);
    input.synchronization = ruckig::Synchronization::Time;
    input.control_interface = request.control_mode == MK_CONTROL_VELOCITY_STOP ?
        ruckig::ControlInterface::Velocity : ruckig::ControlInterface::Position;
    for (uint32_t joint = 0; joint < dofs; ++joint) {
        input.current_position[joint] = request.current_position[joint];
        input.current_velocity[joint] = request.current_velocity[joint];
        input.current_acceleration[joint] = request.current_acceleration[joint];
        input.target_position[joint] = request.control_mode == MK_CONTROL_VELOCITY_STOP ?
            request.current_position[joint] : request.target_position[joint];
        input.target_velocity[joint] = request.target_velocity[joint];
        input.target_acceleration[joint] = request.target_acceleration[joint];
        input.max_velocity[joint] = request.max_velocity[joint];
        input.max_acceleration[joint] = request.max_acceleration[joint];
        input.max_jerk[joint] = request.max_jerk[joint];
    }
    ruckig::Ruckig<ruckig::DynamicDOFs> calculator(dofs);
    ruckig::Trajectory<ruckig::DynamicDOFs> reference(dofs);
    const auto result = calculator.calculate(input, reference);
    ruckig_result = static_cast<int32_t>(result);
    if (const auto mapped = map_result(result); mapped != MK_OK) return mapped;

    const double duration = reference.get_duration();
    if (!std::isfinite(duration) || duration < 0.0 ||
        duration >= static_cast<double>(INT64_MAX - 1024) / 1e9) return MK_ERROR_GENERATION;
    // An unchanged state still needs one positive-duration native segment.
    const int64_t final_ns = std::max<int64_t>(1, std::llround(duration * 1e9));
    std::vector<int64_t> boundaries{0, final_ns};
    const auto profiles = reference.get_profiles();
    if (profiles.size() != 1 || profiles.front().size() != dofs)
        return MK_ERROR_GENERATION; // Waypoint/cloud profiles are intentionally unsupported.
    for (const auto &profile : profiles.front()) {
        auto add_boundary = [&](double seconds) {
            if (std::isfinite(seconds) && seconds > 0.0 && seconds < duration) {
                const auto rounded = std::llround(seconds * 1e9);
                if (rounded > 0 && rounded < final_ns) boundaries.push_back(rounded);
            }
        };
        add_boundary(profile.brake.t[0]);
        add_boundary(profile.brake.duration);
        for (double end : profile.t_sum)
            add_boundary(profile.brake.duration + end);
    }
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());

    std::vector<double> midpoint_position(dofs), midpoint_velocity(dofs),
        midpoint_acceleration(dofs), midpoint_jerk(dofs);
    size_t section = 0;
    for (size_t index = 1; index < boundaries.size(); ++index) {
        const int64_t begin = boundaries[index - 1], end = boundaries[index];
        if (end <= begin) continue; // Sub-nanosecond phases collapse deterministically.
        const double begin_seconds = static_cast<double>(begin) / 1e9;
        const double midpoint_seconds = static_cast<double>(begin + (end - begin) / 2.0) / 1e9;
        reference.at_time(midpoint_seconds, midpoint_position, midpoint_velocity,
            midpoint_acceleration, midpoint_jerk, section);
        mk_segment segment{};
        segment.struct_size = sizeof(segment);
        segment.t0_ns = begin;
        segment.duration_ns = end - begin;
        segment.degree = 3;
        segment.joint_count = dofs;
        for (uint32_t joint = 0; joint < dofs; ++joint) {
            // Anchor inside the active Ruckig phase, then extrapolate to the
            // nanosecond boundary. Sampling at the rounded boundary itself
            // would retain an O(jerk * rounding error) acceleration offset.
            const auto [position, velocity, acceleration] = ruckig::integrate(
                begin_seconds - midpoint_seconds, midpoint_position[joint],
                midpoint_velocity[joint], midpoint_acceleration[joint], midpoint_jerk[joint]);
            segment.coefficients[joint].value[0] = position;
            segment.coefficients[joint].value[1] = velocity;
            segment.coefficients[joint].value[2] = acceleration / 2.0;
            segment.coefficients[joint].value[3] = midpoint_jerk[joint] / 6.0;
        }
        if (!trajectory.append(segment)) return MK_ERROR_GENERATION;
    }
    return MK_OK;
}

} // namespace motionkit
