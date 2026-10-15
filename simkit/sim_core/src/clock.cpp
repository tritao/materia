#include "internal.hpp"

extern "C" {

nksim_result NKSIM_CALL nksim_world_get_clock(nksim_world world, nksim_clock *out_clock) {
    const auto value = nksim::resolve_world(world);
    return value ? value->get_clock(out_clock) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_world_get_features(nksim_world world, uint32_t *out_features) {
    if (!out_features) return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    if (!value) return NKSIM_ERROR_INVALID_HANDLE;
    *out_features = value->features();
    return NKSIM_OK;
}

nksim_result NKSIM_CALL nksim_world_get_position_precision(nksim_world world, double *out_precision) {
    if (!out_precision) return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    if (!value) return NKSIM_ERROR_INVALID_HANDLE;
    *out_precision = value->position_precision();
    return NKSIM_OK;
}

} // extern "C"
