#include "robotkit_runtime.hpp"

#include <cmath>
#include <algorithm>
#include <cstddef>
#include <cstring>

namespace {

template <typename T>
bool has_full_struct(const T *value) {
    return value != nullptr && value->struct_size >= sizeof(T);
}

bool is_finite(double value) {
    return std::isfinite(value);
}

bool valid_target_mode(rk_joint_target_mode mode) {
    return mode >= RK_TARGET_POSITION && mode <= RK_TARGET_SERVO;
}

bool valid_trajectory_status(uint32_t depth, uint32_t active, uint64_t time_ns,
                             uint64_t duration_ns) {
    if (depth > RK_MAX_TRAJECTORY_QUEUE_POINTS || active > 1 || time_ns > duration_ns)
        return false;
    return active != 0 || (depth == 0 && time_ns == 0 && duration_ns == 0);
}

bool has_couplings(const rk_robot_runtime_blueprint *blueprint) {
    return blueprint->struct_size >= sizeof(*blueprint);
}

bool coupled_values(const rk_robot_runtime_blueprint *blueprint, const double *values,
                    double tolerance, bool include_offset) {
    if (!has_couplings(blueprint)) return true;
    for (uint32_t i = 0; i < blueprint->coupling_count; ++i) {
        const auto &c = blueprint->couplings[i];
        const double expected = values[c.leader] * c.ratio + (include_offset ? c.offset : 0.0);
        if (!is_finite(values[c.follower]) ||
            std::abs(values[c.follower] - expected) > tolerance)
            return false;
    }
    return true;
}

template <typename T> bool valid_sensors(const T &value) {
    if (value.sensor_count > RK_MAX_SENSORS) return false;
    for (uint32_t i = 0; i < value.sensor_count; ++i) {
        const auto &sample = value.sensors[i];
        if (sample.value_count > RK_MAX_SENSOR_VALUES || (!sample.sequence && sample.value_count) ||
            sample.value_offset > RK_SENSOR_VALUE_POOL ||
            sample.value_count > RK_SENSOR_VALUE_POOL - sample.value_offset) return false;
        for (uint32_t j = 0; j < sample.value_count; ++j)
            if (!is_finite(value.sensor_values[sample.value_offset + j])) return false;
    }
    return true;
}

} // namespace

extern "C" {

static bool valid_event_id(const char *value, size_t capacity) {
    return value[0] != '\0' && std::memchr(value, '\0', capacity) != nullptr;
}

static bool valid_event_value(const rk_event_value &value) {
    if (value.kind == RK_EVENT_DIGITAL) return value.digital <= 1;
    if (value.kind == RK_EVENT_ANALOG) return is_finite(value.analog);
    if (value.kind == RK_EVENT_PROCESS)
        return valid_event_id(value.command, RK_PROCESS_COMMAND_BYTES) &&
            is_finite(value.argument);
    return false;
}

rk_result RK_CALL rk_robot_runtime_blueprint_validate(const rk_robot_runtime_blueprint *blueprint) {
    if (blueprint == nullptr ||
        blueprint->struct_size < offsetof(rk_robot_runtime_blueprint, calibration_revision) ||
        blueprint->joint_count > RK_MAX_JOINTS ||
        blueprint->link_count == 0 || blueprint->link_count > RK_MAX_LINKS ||
        blueprint->sensor_count > RK_MAX_SENSORS ||
        blueprint->collision_approximation > RK_COLLISION_APPROXIMATION_BOUNDS_BOX ||
        blueprint->self_collision > RK_SELF_COLLISION_DISABLED ||
        blueprint->floating_base > 1)
        return RK_ERROR_INVALID_ARGUMENT;
    if (blueprint->struct_size >=
        offsetof(rk_robot_runtime_blueprint, following_error_bound) +
            sizeof(blueprint->following_error_bound))
        for (uint32_t joint = 0; joint < blueprint->joint_count &&
                joint < RK_MAX_TRAJECTORY_JOINTS; ++joint)
            if (!is_finite(blueprint->following_error_bound[joint]) ||
                blueprint->following_error_bound[joint] < 0.0)
                return RK_ERROR_INVALID_ARGUMENT;
    if (blueprint->struct_size >= offsetof(rk_robot_runtime_blueprint, owner_period_ns) +
            sizeof(blueprint->owner_period_ns) &&
        blueprint->owner_period_ns > static_cast<uint64_t>(INT64_MAX))
        return RK_ERROR_INVALID_ARGUMENT;
    if (blueprint->struct_size >= offsetof(rk_robot_runtime_blueprint, serial_processing_allowance_ns) +
            sizeof(blueprint->serial_processing_allowance_ns) &&
        blueprint->serial_processing_allowance_ns > static_cast<uint64_t>(INT64_MAX))
        return RK_ERROR_INVALID_ARGUMENT;
    if (blueprint->struct_size >= offsetof(rk_robot_runtime_blueprint, joint_dynamics) +
            sizeof(blueprint->joint_dynamics))
        for (uint32_t joint = 0; joint < blueprint->joint_count; ++joint) {
            const auto &dynamics = blueprint->joint_dynamics[joint];
            if (!is_finite(dynamics.armature) || !is_finite(dynamics.damping) ||
                !is_finite(dynamics.friction_loss) || dynamics.armature < 0.0 ||
                dynamics.damping < 0.0 || dynamics.friction_loss < 0.0 ||
                !is_finite(dynamics.limit_time_constant) || dynamics.limit_time_constant < 0.0 ||
                !is_finite(dynamics.limit_damping_ratio) || dynamics.limit_damping_ratio < 0.0)
                return RK_ERROR_INVALID_ARGUMENT;
            for (const double value : dynamics.limit_impedance)
                if (!is_finite(value)) return RK_ERROR_INVALID_ARGUMENT;
        }
    if (blueprint->struct_size >= offsetof(rk_robot_runtime_blueprint, observed_limit_tolerance) +
            sizeof(blueprint->observed_limit_tolerance) &&
        (!is_finite(blueprint->observed_limit_tolerance) || blueprint->observed_limit_tolerance < 0.0))
        return RK_ERROR_INVALID_ARGUMENT;
    if (blueprint->struct_size >= offsetof(rk_robot_runtime_blueprint, joint_overtravel) +
            sizeof(blueprint->joint_overtravel))
        for (uint32_t joint = 0; joint < blueprint->joint_count; ++joint)
            if (!is_finite(blueprint->joint_overtravel[joint]) || blueprint->joint_overtravel[joint] < 0.0)
                return RK_ERROR_INVALID_ARGUMENT;
    constexpr auto channels_size = offsetof(rk_robot_runtime_blueprint, coupling_count);
    if (blueprint->struct_size > offsetof(rk_robot_runtime_blueprint, channel_count) &&
        blueprint->struct_size < channels_size) return RK_ERROR_INVALID_ARGUMENT;
    if (blueprint->struct_size > channels_size &&
        blueprint->struct_size < sizeof(*blueprint)) return RK_ERROR_INVALID_ARGUMENT;
    if (blueprint->struct_size >= channels_size) {
        if (blueprint->channel_count > RK_MAX_PROCESS_CHANNELS) return RK_ERROR_INVALID_ARGUMENT;
        for (uint32_t i = 0; i < blueprint->channel_count; ++i) {
            const auto &channel = blueprint->channels[i];
            if (!valid_event_id(channel.id, sizeof(channel.id)) ||
                channel.kind != channel.safe_value.kind || !valid_event_value(channel.safe_value) ||
                (channel.stop_policy != RK_CHANNEL_SAFE_ON_STOP && channel.stop_policy != RK_CHANNEL_KEEP_ON_STOP))
                return RK_ERROR_INVALID_ARGUMENT;
            for (uint32_t j = 0; j < i; ++j)
                if (std::strcmp(channel.id, blueprint->channels[j].id) == 0)
                    return RK_ERROR_INVALID_ARGUMENT;
        }
    }
    if (has_couplings(blueprint)) {
        if (blueprint->coupling_count > RK_MAX_JOINT_COUPLINGS) return RK_ERROR_INVALID_ARGUMENT;
        for (uint32_t i = 0; i < blueprint->coupling_count; ++i) {
            const auto &c = blueprint->couplings[i];
            if (c.leader >= blueprint->joint_count || c.follower >= blueprint->joint_count ||
                c.leader == c.follower || !is_finite(c.ratio) || c.ratio == 0.0 ||
                !is_finite(c.offset) ||
                blueprint->joints[c.leader].type == RK_RUNTIME_JOINT_FIXED ||
                blueprint->joints[c.follower].type == RK_RUNTIME_JOINT_FIXED)
                return RK_ERROR_INVALID_ARGUMENT;
            for (uint32_t j = 0; j < i; ++j)
                if (c.follower == blueprint->couplings[j].follower)
                    return RK_ERROR_INVALID_ARGUMENT;
        }
    }
    for (uint32_t i = 0; i < blueprint->link_count; ++i) {
        const auto &link = blueprint->links[i];
        if (!is_finite(link.mass) || link.mass <= 0.0) return RK_ERROR_INVALID_ARGUMENT;
        for (double value : link.center_of_mass) if (!is_finite(value)) return RK_ERROR_INVALID_ARGUMENT;
        for (double value : link.inertia_tensor) if (!is_finite(value)) return RK_ERROR_INVALID_ARGUMENT;
        const auto *m = link.inertia_tensor;
        const double scale = std::max({1.0, std::abs(m[0]), std::abs(m[4]), std::abs(m[8])});
        const double eps = scale * 1e-10;
        if (std::abs(m[1]-m[3]) > eps || std::abs(m[2]-m[6]) > eps || std::abs(m[5]-m[7]) > eps ||
            m[0] <= eps || m[0]*m[4]-m[1]*m[3] <= eps*eps ||
            m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6]) <= eps*eps*eps)
            return RK_ERROR_INVALID_ARGUMENT;
    }
    // The sensors' values share one pool in every state, so what they report together must fit it.
    uint64_t pooled_values = 0;
    for (uint32_t i = 0; i < blueprint->sensor_count; ++i) {
        const auto &sensor = blueprint->sensors[i];
        if (sensor.kind < RK_SENSOR_ENCODER || sensor.kind > RK_SENSOR_LIDAR ||
            sensor.link >= blueprint->link_count || !is_finite(sensor.update_rate) || sensor.update_rate < 0.0 ||
            !is_finite(sensor.noise_stddev) || sensor.noise_stddev < 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
        double norm = 0.0;
        for (double v : sensor.position) if (!is_finite(v)) return RK_ERROR_INVALID_ARGUMENT;
        for (double v : sensor.rotation) { if (!is_finite(v)) return RK_ERROR_INVALID_ARGUMENT; norm += v*v; }
        if (std::abs(norm - 1.0) > 1e-6) return RK_ERROR_INVALID_ARGUMENT;
        if (sensor.kind == RK_SENSOR_LIDAR && (!sensor.ray_count || sensor.ray_count > RK_MAX_SENSOR_VALUES ||
            !is_finite(sensor.max_range) || sensor.max_range <= 0.0 ||
            !is_finite(sensor.start_angle) || !is_finite(sensor.field_of_view) ||
            sensor.field_of_view < 0.0 || sensor.field_of_view > 6.283185307179586))
            return RK_ERROR_INVALID_ARGUMENT;
        pooled_values += sensor.kind == RK_SENSOR_LIDAR ? sensor.ray_count
            : sensor.kind == RK_SENSOR_IMU ? 6 : blueprint->joint_count;
    }
    if (pooled_values > RK_SENSOR_VALUE_POOL) return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < blueprint->joint_count; ++index) {
        const auto &joint = blueprint->joints[index];
        if (joint.joint != index || joint.type < RK_RUNTIME_JOINT_FIXED ||
            joint.type > RK_RUNTIME_JOINT_PRISMATIC || joint.parent_link >= blueprint->link_count ||
            joint.child_link >= blueprint->link_count || joint.parent_link == joint.child_link ||
            !is_finite(joint.lower_limit) || !is_finite(joint.upper_limit) ||
            !is_finite(joint.max_effort) || !is_finite(joint.max_acceleration) ||
            !is_finite(joint.max_velocity) ||
            joint.lower_limit > joint.upper_limit || joint.max_effort < 0.0 ||
            joint.max_acceleration < 0.0 || joint.max_velocity < 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
        for (double value : joint.parent_frame_position) if (!is_finite(value)) return RK_ERROR_INVALID_ARGUMENT;
        for (double value : joint.child_frame_position) if (!is_finite(value)) return RK_ERROR_INVALID_ARGUMENT;
        double parent_norm = 0.0, child_norm = 0.0, axis_norm = 0.0;
        for (double value : joint.parent_frame_rotation) { if (!is_finite(value)) return RK_ERROR_INVALID_ARGUMENT; parent_norm += value*value; }
        for (double value : joint.child_frame_rotation) { if (!is_finite(value)) return RK_ERROR_INVALID_ARGUMENT; child_norm += value*value; }
        for (double value : joint.axis) { if (!is_finite(value)) return RK_ERROR_INVALID_ARGUMENT; axis_norm += value*value; }
        if (std::abs(parent_norm-1.0)>1e-6 || std::abs(child_norm-1.0)>1e-6 || std::abs(axis_norm-1.0)>1e-6)
            return RK_ERROR_INVALID_ARGUMENT;
    }
    return RK_OK;
}

rk_result RK_CALL rk_robot_command_validate(const rk_robot_command *command) {
    if (!has_full_struct(command) || command->target_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    if (command->kind > RK_COMMAND_ABORT || command->kind == 5)
        return RK_ERROR_INVALID_ARGUMENT;
    if ((command->kind == RK_COMMAND_NONE || command->kind == RK_COMMAND_STOP ||
         command->kind == RK_COMMAND_EMERGENCY_STOP ||
         command->kind == RK_COMMAND_RESET_SAFETY ||
         command->kind == RK_COMMAND_HOLD || command->kind == RK_COMMAND_RESUME ||
         command->kind == RK_COMMAND_ABORT) &&
        command->target_count != 0)
        return RK_ERROR_INVALID_ARGUMENT;
    if (command->kind == RK_COMMAND_TRAJECTORY_SEGMENTS && command->target_count != 0)
        return RK_ERROR_INVALID_ARGUMENT;

    bool targeted[RK_MAX_JOINTS]{};
    for (uint32_t index = 0; index < command->target_count; ++index) {
        const auto &target = command->targets[index];
        if (target.joint >= RK_MAX_JOINTS || !valid_target_mode(target.mode) ||
            !is_finite(target.target) || !is_finite(target.max_rate) ||
            !is_finite(target.max_effort) || target.max_rate < 0.0 || target.max_effort < 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
        if (targeted[target.joint])
            return RK_ERROR_INVALID_ARGUMENT;
        targeted[target.joint] = true;
        if (target.mode == RK_TARGET_SERVO) {
            if (index >= RK_MAX_SERVO_JOINTS || target.joint >= RK_MAX_SERVO_JOINTS)
                return RK_ERROR_INVALID_ARGUMENT;
            const auto &servo = command->servos[index];
            if (!is_finite(servo.velocity) || !is_finite(servo.stiffness) ||
                !is_finite(servo.damping) || !is_finite(servo.feedforward) ||
                servo.stiffness < 0.0 || servo.damping < 0.0)
                return RK_ERROR_INVALID_ARGUMENT;
        }
    }
    return RK_OK;
}

rk_result RK_CALL rk_robot_command_validate_for_blueprint(
    const rk_robot_command *command, const rk_robot_runtime_blueprint *blueprint) {
    if (rk_robot_runtime_blueprint_validate(blueprint) != RK_OK ||
        rk_robot_command_validate(command) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < command->target_count; ++index) {
        if (command->targets[index].joint >= blueprint->joint_count)
            return RK_ERROR_INVALID_ARGUMENT;
    }
    if (has_couplings(blueprint) && command->kind == RK_COMMAND_JOINT_TARGETS) {
        for (uint32_t i = 0; i < blueprint->coupling_count; ++i) {
            const auto &c = blueprint->couplings[i];
            const rk_joint_target *leader = nullptr, *follower = nullptr;
            for (uint32_t j = 0; j < command->target_count; ++j) {
                if (command->targets[j].joint == c.leader) leader = &command->targets[j];
                if (command->targets[j].joint == c.follower) follower = &command->targets[j];
            }
            if (follower && follower->mode != RK_TARGET_EFFORT &&
                (!leader || follower->mode != leader->mode ||
                std::abs(follower->target - c.ratio * leader->target -
                    (follower->mode == RK_TARGET_POSITION || follower->mode == RK_TARGET_SERVO
                        ? c.offset : 0.0)) > 1e-6))
                return RK_ERROR_INVALID_ARGUMENT;
        }
    }
    return RK_OK;
}

rk_result RK_CALL rk_robot_state_validate(const rk_robot_state *state) {
    if (!has_full_struct(state) || state->joint_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    if (state->mode > RK_ROBOT_MODE_FAULT || state->safety > RK_SAFETY_FAULT ||
        !valid_trajectory_status(state->trajectory_queue_depth, state->trajectory_active,
            state->trajectory_time_ns, state->trajectory_duration_ns) || !valid_sensors(*state))
        return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < state->joint_count; ++index) {
        if (!is_finite(state->position[index]) || !is_finite(state->velocity[index]) ||
            !is_finite(state->effort[index]))
            return RK_ERROR_INVALID_ARGUMENT;
    }
    return RK_OK;
}

rk_result RK_CALL rk_robot_snapshot_validate(const rk_robot_snapshot *snapshot) {
    if (snapshot == nullptr ||
        snapshot->struct_size < offsetof(rk_robot_snapshot, calibration_revision) ||
        snapshot->joint_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    if (snapshot->mode > RK_ROBOT_MODE_FAULT || snapshot->safety > RK_SAFETY_FAULT ||
        snapshot->endpoint > RK_ENDPOINT_FAULT ||
        !valid_trajectory_status(snapshot->trajectory_queue_depth, snapshot->trajectory_active,
            snapshot->trajectory_time_ns, snapshot->trajectory_duration_ns))
        return RK_ERROR_INVALID_ARGUMENT;
    if (snapshot->fault_code < 0 || !valid_sensors(*snapshot))
        return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < snapshot->joint_count; ++index) {
        if (!is_finite(snapshot->position[index]) || !is_finite(snapshot->velocity[index]) ||
            !is_finite(snapshot->effort[index]))
            return RK_ERROR_INVALID_ARGUMENT;
    }
    return RK_OK;
}

rk_result RK_CALL rk_robot_capabilities_validate(const rk_robot_capabilities *capabilities) {
    if (!has_full_struct(capabilities) || capabilities->joint_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    if (capabilities->supports_position_targets > 1 ||
        capabilities->supports_velocity_targets > 1 ||
        capabilities->supports_effort_targets > 1 || capabilities->supports_prediction > 1 ||
        capabilities->supports_trajectory_queue > 1 ||
        capabilities->supports_execution_plans > 1)
        return RK_ERROR_INVALID_ARGUMENT;
    return RK_OK;
}

} // extern "C"

namespace robotkit {

rk_result validate_segments(const SegmentBatch &batch) {
    const auto &segments = batch.segments;
    if (segments.empty() || segments.size() > RK_MAX_TRAJECTORY_QUEUE_POINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    uint64_t expected_start = 0;
    const uint32_t joint_count = segments[0].joint_count;
    for (const auto &segment : segments) {
        if (segment.joint_count == 0 || segment.joint_count > RK_MAX_TRAJECTORY_JOINTS ||
            segment.joint_count != joint_count || segment.degree > 5 ||
            segment.duration_ns == 0 || segment.time_from_start_ns != expected_start ||
            segment.duration_ns > UINT64_MAX - expected_start)
            return RK_ERROR_INVALID_ARGUMENT;
        expected_start += segment.duration_ns;
        for (uint32_t joint = 0; joint < joint_count; ++joint)
            for (uint32_t degree = 0; degree <= segment.degree; ++degree)
                if (!is_finite(segment.coefficients[joint].value[degree]))
                    return RK_ERROR_INVALID_ARGUMENT;
    }
    return RK_OK;
}

rk_result validate_segments_for_blueprint(const SegmentBatch &batch,
    const rk_robot_runtime_blueprint &blueprint) {
    if (rk_robot_runtime_blueprint_validate(&blueprint) != RK_OK ||
        validate_segments(batch) != RK_OK ||
        blueprint.joint_count > RK_MAX_TRAJECTORY_JOINTS ||
        batch.segments[0].joint_count != blueprint.joint_count)
        return RK_ERROR_INVALID_ARGUMENT;
    if (has_couplings(&blueprint))
        for (const auto &segment : batch.segments)
            for (uint32_t i = 0; i < blueprint.coupling_count; ++i) {
                const auto &c = blueprint.couplings[i];
                for (uint32_t degree = 0; degree <= segment.degree; ++degree) {
                    const double expected = c.ratio * segment.coefficients[c.leader].value[degree] +
                        (degree == 0 ? c.offset : 0.0);
                    if (std::abs(segment.coefficients[c.follower].value[degree] - expected) > 1e-6)
                        return RK_ERROR_INVALID_ARGUMENT;
                }
            }
    return RK_OK;
}

rk_result validate_plan_for_blueprint(const PlanRequest &plan,
    const rk_robot_runtime_blueprint &blueprint) {
    if (plan.sequence == 0 || plan.plan_id == 0 ||
        (plan.flags & ~RK_PLAN_JERK_UNCHECKED) != 0 ||
        validate_segments_for_blueprint(plan.segments, blueprint) != RK_OK ||
        (plan.replace_after_plan_id == 0 && plan.replace_after_time_ns != 0) ||
        (plan.replace_after_plan_id != 0 && plan.replace_after_time_ns == 0))
        return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint)
        if (!is_finite(plan.start_position[joint]) ||
            !is_finite(plan.start_velocity[joint]) ||
            !is_finite(plan.start_acceleration[joint]) ||
            !is_finite(plan.position_tolerance[joint]) ||
            !is_finite(plan.velocity_tolerance[joint]) ||
            !is_finite(plan.acceleration_tolerance[joint]) ||
            plan.position_tolerance[joint] < 0.0 ||
            plan.velocity_tolerance[joint] < 0.0 ||
            plan.acceleration_tolerance[joint] < 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
    if (!coupled_values(&blueprint, plan.start_position, 1e-6, true) ||
        !coupled_values(&blueprint, plan.start_velocity, 1e-6, false) ||
        !coupled_values(&blueprint, plan.start_acceleration, 1e-6, false))
        return RK_ERROR_INVALID_ARGUMENT;
    if (plan.events.size() > RK_MAX_TRAJECTORY_QUEUE_POINTS ||
        (!plan.events.empty() && (plan.required_capabilities & RK_PLAN_CAPABILITY_EVENTS) == 0))
        return RK_ERROR_INVALID_ARGUMENT;
    uint64_t previous = 0;
    for (std::size_t i = 0; i < plan.events.size(); ++i) {
        const auto &event = plan.events[i];
        if (!valid_event_id(event.channel, sizeof(event.channel)) ||
            !valid_event_value(event.value) || event.hold_policy > RK_EVENT_RESTORE_ON_RESUME ||
            (i != 0 && event.time_ns < previous)) return RK_ERROR_INVALID_ARGUMENT;
        previous = event.time_ns;
        bool declared = false;
        for (uint32_t j = 0; j < blueprint.channel_count; ++j)
            if (std::strcmp(event.channel, blueprint.channels[j].id) == 0 &&
                event.value.kind == blueprint.channels[j].kind) declared = true;
        if (!declared) return RK_ERROR_INVALID_ARGUMENT;
    }
    return RK_OK;
}

} // namespace robotkit

