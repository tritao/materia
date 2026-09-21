#include "robotkit_runtime.h"

#include <cmath>
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

} // namespace

extern "C" {

rk_result RK_CALL rk_runtime_layout_validate(const rk_runtime_layout *layout) {
    if (!has_full_struct(layout) || layout->joint_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    return RK_OK;
}

rk_result RK_CALL rk_runtime_blueprint_validate(const rk_runtime_blueprint *blueprint) {
    if (!has_full_struct(blueprint) || blueprint->joint_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < blueprint->joint_count; ++index) {
        const auto &joint = blueprint->joints[index];
        if (joint.joint != index || joint.type < RK_RUNTIME_JOINT_FIXED ||
            joint.type > RK_RUNTIME_JOINT_PRISMATIC || joint.parent_link >= blueprint->link_count ||
            joint.child_link >= blueprint->link_count || joint.parent_link == joint.child_link ||
            !is_finite(joint.lower_limit) || !is_finite(joint.upper_limit) ||
            !is_finite(joint.max_effort) || joint.lower_limit > joint.upper_limit ||
            joint.max_effort < 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
    }
    return RK_OK;
}

rk_result RK_CALL rk_robot_command_validate(const rk_robot_command *command) {
    if (!has_full_struct(command) || command->target_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    if (command->kind > RK_COMMAND_EMERGENCY_STOP)
        return RK_ERROR_INVALID_ARGUMENT;
    if ((command->kind == RK_COMMAND_NONE || command->kind == RK_COMMAND_STOP ||
         command->kind == RK_COMMAND_EMERGENCY_STOP) && command->target_count != 0)
        return RK_ERROR_INVALID_ARGUMENT;

    for (uint32_t index = 0; index < command->target_count; ++index) {
        const auto &target = command->targets[index];
        if (target.joint >= RK_MAX_JOINTS || !valid_target_mode(target.mode) ||
            !is_finite(target.target) || !is_finite(target.max_rate) ||
            !is_finite(target.max_effort) || target.max_rate < 0.0 || target.max_effort < 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
    }
    return RK_OK;
}

rk_result RK_CALL rk_robot_command_validate_for_layout(const rk_robot_command *command,
                                                        const rk_runtime_layout *layout) {
    if (rk_runtime_layout_validate(layout) != RK_OK || rk_robot_command_validate(command) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < command->target_count; ++index) {
        if (command->targets[index].joint >= layout->joint_count)
            return RK_ERROR_INVALID_ARGUMENT;
    }
    return RK_OK;
}

rk_result RK_CALL rk_robot_state_validate(const rk_robot_state *state) {
    if (!has_full_struct(state) || state->joint_count > RK_MAX_JOINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    if (state->mode > RK_ROBOT_MODE_FAULT || state->safety > RK_SAFETY_FAULT)
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
    if (snapshot->fault_code < 0)
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
