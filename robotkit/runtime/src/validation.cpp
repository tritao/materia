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
        blueprint->collision_approximation > RK_COLLISION_APPROXIMATION_BOUNDS_BOX)
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
            !is_finite(sensor.max_range) || sensor.max_range <= 0.0)) return RK_ERROR_INVALID_ARGUMENT;
    }
    for (uint32_t index = 0; index < blueprint->joint_count; ++index) {
        const auto &joint = blueprint->joints[index];
        if (joint.joint != index || joint.type < RK_RUNTIME_JOINT_FIXED ||
            joint.type > RK_RUNTIME_JOINT_PRISMATIC || joint.parent_link >= blueprint->link_count ||
            joint.child_link >= blueprint->link_count || joint.parent_link == joint.child_link ||
            !is_finite(joint.lower_limit) || !is_finite(joint.upper_limit) ||
            !is_finite(joint.max_effort) || joint.lower_limit > joint.upper_limit ||
            joint.max_effort < 0.0)
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
    if (command->kind > RK_COMMAND_RESET_SAFETY)
        return RK_ERROR_INVALID_ARGUMENT;
    if ((command->kind == RK_COMMAND_NONE || command->kind == RK_COMMAND_STOP ||
         command->kind == RK_COMMAND_EMERGENCY_STOP ||
         command->kind == RK_COMMAND_RESET_SAFETY) && command->target_count != 0)
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

rk_result RK_CALL rk_robot_state_validate(const rk_robot_state *state) {
    if (!has_full_struct(state) || state->joint_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    if (state->mode > RK_ROBOT_MODE_FAULT || state->safety > RK_SAFETY_FAULT || !valid_sensors(*state))
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
        snapshot->endpoint > RK_ENDPOINT_FAULT)
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
        capabilities->supports_effort_targets > 1 || capabilities->supports_prediction > 1)
        return RK_ERROR_INVALID_ARGUMENT;
    return RK_OK;
}

} // extern "C"
