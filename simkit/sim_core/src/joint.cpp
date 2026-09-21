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

nksim_result NKSIM_CALL nksim_joint_get_state(nksim_world world, nksim_joint joint,
                                              nksim_joint_state *out_state) {
    const auto value = nksim::resolve_world(world);
    return value ? value->get_joint_state(joint, out_state) : NKSIM_ERROR_INVALID_HANDLE;
}

} // extern "C"
