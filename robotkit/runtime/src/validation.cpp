#include "robotkit_runtime.h"

#include <cmath>
#include <algorithm>
#include <cstddef>

namespace {

template <typename T>
bool has_full_struct(const T *value) {
    return value != nullptr && value->struct_size >= sizeof(T);
}

bool is_finite(double value) {
    return std::isfinite(value);
}

bool valid_target_mode(rk_joint_target_mode mode) {
    return mode >= RK_TARGET_POSITION && mode <= RK_TARGET_EFFORT;
}

bool valid_trajectory_status(uint32_t depth, uint32_t active, uint64_t time_ns,
                             uint64_t duration_ns) {
    if (depth > RK_MAX_TRAJECTORY_POINTS || active > 1 || time_ns > duration_ns)
        return false;
    return active != 0 || (depth == 0 && time_ns == 0 && duration_ns == 0);
}

template <typename T> bool valid_sensors(const T &value) {
    if (value.sensor_count > RK_MAX_SENSORS) return false;
    for (uint32_t i = 0; i < value.sensor_count; ++i) {
        const auto &sample = value.sensors[i];
        if (sample.value_count > RK_MAX_SENSOR_VALUES || (!sample.sequence && sample.value_count)) return false;
        for (uint32_t j = 0; j < sample.value_count; ++j)
            if (!is_finite(sample.values[j])) return false;
    }
    return true;
}

} // namespace

extern "C" {

rk_result RK_CALL rk_robot_runtime_blueprint_validate(const rk_robot_runtime_blueprint *blueprint) {
    if (!has_full_struct(blueprint) || blueprint->joint_count > RK_MAX_JOINTS ||
        blueprint->link_count == 0 || blueprint->link_count > RK_MAX_LINKS ||
        blueprint->sensor_count > RK_MAX_SENSORS ||
        blueprint->collision_approximation > RK_COLLISION_APPROXIMATION_BOUNDS_BOX ||
        blueprint->self_collision > RK_SELF_COLLISION_DISABLED)
        return RK_ERROR_INVALID_ARGUMENT;
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
    }
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
    if (command->kind > RK_COMMAND_TRAJECTORY_CHUNK)
        return RK_ERROR_INVALID_ARGUMENT;
    if ((command->kind == RK_COMMAND_NONE || command->kind == RK_COMMAND_STOP ||
         command->kind == RK_COMMAND_EMERGENCY_STOP ||
         command->kind == RK_COMMAND_RESET_SAFETY) &&
        command->target_count != 0)
        return RK_ERROR_INVALID_ARGUMENT;
    if (command->kind == RK_COMMAND_TRAJECTORY_CHUNK && command->target_count != 0)
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
    }
    return RK_OK;
}

rk_result RK_CALL rk_trajectory_chunk_validate(const rk_trajectory_chunk *chunk) {
    if (!has_full_struct(chunk) || chunk->point_count == 0 ||
        chunk->point_count > RK_MAX_TRAJECTORY_POINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    uint64_t previous_time = 0;
    for (uint32_t index = 0; index < chunk->point_count; ++index) {
        const auto &point = chunk->points[index];
        if (point.joint_count == 0 || point.joint_count > RK_MAX_TRAJECTORY_JOINTS ||
            (index > 0 && point.time_from_start_ns < previous_time))
            return RK_ERROR_INVALID_ARGUMENT;
        previous_time = point.time_from_start_ns;
        for (uint32_t joint = 0; joint < point.joint_count; ++joint)
            if (!is_finite(point.positions[joint])) return RK_ERROR_INVALID_ARGUMENT;
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
    return RK_OK;
}

rk_result RK_CALL rk_trajectory_chunk_validate_for_blueprint(
    const rk_trajectory_chunk *chunk, const rk_robot_runtime_blueprint *blueprint) {
    if (rk_robot_runtime_blueprint_validate(blueprint) != RK_OK ||
        rk_trajectory_chunk_validate(chunk) != RK_OK ||
        blueprint->joint_count > RK_MAX_TRAJECTORY_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < chunk->point_count; ++index)
        if (chunk->points[index].joint_count != blueprint->joint_count)
            return RK_ERROR_INVALID_ARGUMENT;
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
    if (!has_full_struct(snapshot) || snapshot->joint_count > RK_MAX_JOINTS)
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
        capabilities->supports_trajectory_queue > 1)
        return RK_ERROR_INVALID_ARGUMENT;
    return RK_OK;
}

} // extern "C"
