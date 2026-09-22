#include "internal.hpp"

extern "C" {

void NKSIM_CALL nksim_body_destroy(nksim_world world, nksim_body body) {
    const auto value = nksim::resolve_world(world);
    if (value)
        value->destroy_body(body);
}

nksim_result NKSIM_CALL nksim_body_create(nksim_world world, const nksim_body_desc *desc,
                                          nksim_body *out_body) {
    if (!desc || !out_body)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    return value ? value->create_body(*desc, out_body) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_body_get_state(nksim_world world, nksim_body body,
                                             nksim_body_state *out_state) {
    const auto value = nksim::resolve_world(world);
    return value ? value->get_body_state(body, out_state) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_body_set_state(nksim_world world, nksim_body body,
                                             const nksim_body_state *state) {
    if (!state)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    return value ? value->set_body_state(body, *state) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_world_reset(nksim_world world) {
    const auto value = nksim::resolve_world(world);
    return value ? value->reset() : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_body_reset(nksim_world world, nksim_body body) {
    const auto value = nksim::resolve_world(world);
    return value ? value->reset_body(body) : NKSIM_ERROR_INVALID_HANDLE;
}

} // extern "C"
