#include "motionkit.hpp"

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include <memory>
#include <new>

namespace motionkit {
namespace {

bool valid_request(const mk_state_to_state_request &request) {
    if (request.struct_size < sizeof(request) || request.joint_count == 0 ||
        request.joint_count > MK_MAX_JOINTS ||
        (request.synchronization != MK_SYNCHRONIZATION_TIME &&
         request.synchronization != MK_SYNCHRONIZATION_PHASE) ||
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
    // Ruckig's Phase mode may fall back to independently synchronized axes
    // when the fastest individual profile cannot be shared under every limit.
    // For stopped straight moves, solve one progress coordinate instead: its
    // limits are the intersection of every scaled joint limit. This guarantees
    // the requested phase relationship even when no individual axis supplies
    // the limiting velocity, acceleration and jerk simultaneously.
    bool stopped_phase = request.synchronization == MK_SYNCHRONIZATION_PHASE &&
        request.control_mode == MK_CONTROL_POSITION;
    for (uint32_t joint = 0; joint < request.joint_count; ++joint)
        stopped_phase = stopped_phase && request.current_velocity[joint] == 0.0 &&
            request.current_acceleration[joint] == 0.0 &&
            request.target_velocity[joint] == 0.0 && request.target_acceleration[joint] == 0.0;
    if (stopped_phase) {
        auto progress_request = request;
        progress_request.joint_count = 1;
        progress_request.synchronization = MK_SYNCHRONIZATION_TIME;
        progress_request.current_position[0] = 0.0;
        progress_request.target_position[0] = 1.0;
        double speed = std::numeric_limits<double>::infinity();
        double acceleration = speed, jerk = speed;
        bool moving = false;
        for (uint32_t joint = 0; joint < request.joint_count; ++joint) {
            const double distance = std::abs(request.target_position[joint] -
                request.current_position[joint]);
            if (distance == 0.0) continue;
            moving = true;
            speed = std::min(speed, request.max_velocity[joint] / distance);
            acceleration = std::min(acceleration, request.max_acceleration[joint] / distance);
            jerk = std::min(jerk, request.max_jerk[joint] / distance);
        }
        if (moving) {
            progress_request.max_velocity[0] = speed;
            progress_request.max_acceleration[0] = acceleration;
            progress_request.max_jerk[0] = jerk;
            Trajectory progress(1);
            const auto status = generate(progress_request, progress, ruckig_result);
            if (status != MK_OK) return status;
            for (uint32_t index = 0; index < progress.segment_count(); ++index) {
                const auto source = progress.segment(index);
                auto segment = source;
                segment.joint_count = request.joint_count;
                for (uint32_t joint = 0; joint < request.joint_count; ++joint) {
                    const double delta = request.target_position[joint] - request.current_position[joint];
                    for (uint32_t power = 0; power <= source.degree; ++power)
                        segment.coefficients[joint].value[power] =
                            (power == 0 ? request.current_position[joint] : 0.0) +
                            delta * source.coefficients[0].value[power];
                }
                if (!trajectory.append(segment)) return MK_ERROR_GENERATION;
            }
            return MK_OK;
        }
    }
    const auto dofs = request.joint_count;
    ruckig::InputParameter<ruckig::DynamicDOFs> input(dofs);
    input.synchronization = request.synchronization == MK_SYNCHRONIZATION_PHASE ?
        ruckig::Synchronization::Phase : ruckig::Synchronization::Time;
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
    auto result = calculator.calculate(input, reference);
    double time_scale = 1.0;
    if (map_result(result) != MK_OK && dofs == 1 && request.control_mode == MK_CONTROL_POSITION &&
        request.current_velocity[0] == 0.0 && request.current_acceleration[0] == 0.0 &&
        request.target_velocity[0] == 0.0 && request.target_acceleration[0] == 0.0) {
        // Unit progress on a microscopic move has enormous derivative limits.
        // Solve the identical problem in its natural jerk time unit instead.
        // Keep the coordinate: shrinking it can trigger Ruckig's no-move cutoff.
        const double distance = std::abs(input.target_position[0] - input.current_position[0]);
        const double scale = std::cbrt(distance) / std::cbrt(input.max_jerk[0]);
        auto conditioned = input;
        conditioned.max_velocity[0] *= scale;
        conditioned.max_acceleration[0] *= scale * scale;
        conditioned.max_jerk[0] *= scale * scale * scale;
        if (std::isfinite(scale) && scale > 0.0 && scale != 1.0 &&
            std::isfinite(conditioned.max_velocity[0]) && conditioned.max_velocity[0] > 0.0 &&
            std::isfinite(conditioned.max_acceleration[0]) && conditioned.max_acceleration[0] > 0.0 &&
            std::isfinite(conditioned.max_jerk[0]) && conditioned.max_jerk[0] > 0.0) {
            const auto retried = calculator.calculate(conditioned, reference);
            if (map_result(retried) == MK_OK && reference.get_duration() > 0.0) {
                result = retried;
                time_scale = scale;
            }
        }
    }
    ruckig_result = static_cast<int32_t>(result);
    if (const auto mapped = map_result(result); mapped != MK_OK) return mapped;

    const double duration = reference.get_duration() * time_scale;
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
            seconds *= time_scale;
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
        reference.at_time(midpoint_seconds / time_scale, midpoint_position, midpoint_velocity,
            midpoint_acceleration, midpoint_jerk, section);
        if (time_scale != 1.0) for (uint32_t joint = 0; joint < dofs; ++joint) {
            midpoint_velocity[joint] /= time_scale;
            midpoint_acceleration[joint] /= time_scale * time_scale;
            midpoint_jerk[joint] /= time_scale * time_scale * time_scale;
        }
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
            if (begin == 0) {
                // A sub-nanosecond initial phase can collapse into the next
                // phase. Keep the requested start state exact at time zero;
                // its quantization belongs to the subsequent phase seam.
                segment.coefficients[joint].value[0] = request.current_position[joint];
                segment.coefficients[joint].value[1] = request.current_velocity[joint];
                segment.coefficients[joint].value[2] = request.current_acceleration[joint] / 2.0;
            }
        }
        if (!trajectory.append(segment)) return MK_ERROR_GENERATION;
    }
    return MK_OK;
}

} // namespace motionkit

mk_result MK_CALL mk_generate_state_to_state(const mk_state_to_state_request *request,
    mk_trajectory_handle *out_trajectory, int32_t *out_ruckig_result) {
    if (out_trajectory != nullptr) out_trajectory->id = 0;
    if (out_ruckig_result != nullptr) *out_ruckig_result = -100; // Ruckig ErrorInvalidInput.
    if (request == nullptr || out_trajectory == nullptr || out_ruckig_result == nullptr ||
        request->struct_size < sizeof(mk_state_to_state_request) ||
        request->joint_count == 0 || request->joint_count > MK_MAX_JOINTS)
        return MK_ERROR_INVALID_ARGUMENT;
    *out_ruckig_result = 0;
    try {
        auto generated = std::make_unique<motionkit::Trajectory>(request->joint_count);
        const auto result = motionkit::generate(*request, *generated, *out_ruckig_result);
        if (result != MK_OK) return result;
        auto status = mk_trajectory_create(request->joint_count, out_trajectory);
        if (status != MK_OK) return status;
        for (uint32_t index = 0; index < generated->segment_count(); ++index) {
            const mk_segment segment = generated->segment(index);
            status = mk_trajectory_append_segment(*out_trajectory, &segment);
            if (status != MK_OK) {
                mk_trajectory_destroy(*out_trajectory);
                out_trajectory->id = 0;
                return status;
            }
        }
        return MK_OK;
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_GENERATION;
    }
}
