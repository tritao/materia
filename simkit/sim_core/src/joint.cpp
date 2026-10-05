#include "internal.hpp"

extern "C" {

void NKSIM_CALL nksim_joint_destroy(nksim_world world, nksim_joint joint) {
    const auto value = nksim::resolve_world(world);
    if (value)
        value->destroy_joint(joint);
}

nksim_result NKSIM_CALL nksim_joint_create(nksim_world world, const nksim_joint_desc *desc,
                                           nksim_joint *out_joint) {
    if (!desc || !out_joint)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    return value ? value->create_joint(*desc, out_joint) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_joint_couple(nksim_world world,
                                           const nksim_joint_coupling_desc *desc) {
    if (!desc || desc->struct_size < sizeof(*desc)) return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    return value ? value->couple_joint(*desc) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_closure_create(nksim_world world,
                                             const nksim_closure_desc *desc) {
    if (!desc || desc->struct_size < sizeof(*desc)) return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    return value ? value->create_closure(*desc) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_joint_get_state(nksim_world world, nksim_joint joint,
                                              nksim_joint_state *out_state) {
    const auto value = nksim::resolve_world(world);
    return value ? value->get_joint_state(joint, out_state) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_joint_set_state(nksim_world world, nksim_joint joint,
                                              double position, double velocity) {
    const auto value = nksim::resolve_world(world);
    return value ? value->set_joint_state(joint, position, velocity) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_joint_configure_brake(nksim_world world, nksim_joint joint,
        double holding_effort, double release_delay, double application_delay) {
    const auto value = nksim::resolve_world(world);
    return value ? value->configure_joint_brake(joint, {holding_effort, release_delay, application_delay})
        : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_joint_set_brake_release(nksim_world world, nksim_joint joint,
        uint32_t energized) {
    if (energized > 1) return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    return value ? value->set_joint_brake_release(joint, energized != 0) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_joint_get_brake_released(nksim_world world, nksim_joint joint,
        uint32_t *out_released) {
    if (!out_released) return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    if (!value) return NKSIM_ERROR_INVALID_HANDLE;
    bool released = false;
    const auto result = value->get_joint_brake_released(joint, &released);
    if (result == NKSIM_OK) *out_released = released ? 1u : 0u;
    return result;
}

} // extern "C"
