#include "internal.hpp"

extern "C" {

nksim_result NKSIM_CALL nksim_world_get_clock(nksim_world world, nksim_clock *out_clock) {
    const auto value = nksim::resolve_world(world);
    return value ? value->get_clock(out_clock) : NKSIM_ERROR_INVALID_HANDLE;
}

} // extern "C"
