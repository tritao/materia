// The simulated device board's C ABI for builds without it (RK_BUILD_VIRTUAL_DEVICE off): no board can be created,
// so a simulation asking for one fails with RK_ERROR_UNSUPPORTED and the rest is never called.
#include "rkd_virtual.h"

extern "C" {
rkd_virtual_device *rkd_virtual_create(uint64_t, uint32_t, uint64_t, int32_t, uint32_t, const double *,
    const uint8_t *, uint8_t) { return nullptr; }
void rkd_virtual_destroy(rkd_virtual_device *) {}
int32_t rkd_virtual_step(rkd_virtual_device *, uint64_t) { return -1; }
int32_t rkd_virtual_link_host_to_device(rkd_virtual_device *, const uint8_t *, size_t) { return -1; }
size_t rkd_virtual_link_device_to_host(rkd_virtual_device *, uint8_t *, size_t) { return 0; }
size_t rkd_virtual_actuator_positions(const rkd_virtual_device *, double *, size_t) { return 0; }
size_t rkd_virtual_channel_values(const rkd_virtual_device *, float *, size_t) { return 0; }
size_t rkd_virtual_step_log(const rkd_virtual_device *, rkd_virtual_step_record *, size_t) { return 0; }
size_t rkd_virtual_event_log(const rkd_virtual_device *, rkd_virtual_event_record *, size_t) { return 0; }
int32_t rkd_virtual_miss_next_steps(rkd_virtual_device *, uint32_t, uint32_t) { return -1; }
}
