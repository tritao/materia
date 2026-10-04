#ifndef RKD_VIRTUAL_H
#define RKD_VIRTUAL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rkd_virtual_device rkd_virtual_device;
typedef struct rkd_virtual_step_record {
    uint64_t ticks;
    uint32_t actuator;
    uint8_t forward;
    uint8_t reserved[3];
} rkd_virtual_step_record;
typedef struct rkd_virtual_event_record {
    uint64_t plan_id;
    uint64_t scheduled_path_ticks;
    uint64_t applied_path_ticks;
    uint64_t device_ticks;
    uint32_t channel;
    uint8_t kind;
    uint8_t digital;
    uint8_t reserved[2];
} rkd_virtual_event_record;

/* host_ns is the deterministic owner clock; the board applies offset and drift. */
rkd_virtual_device *rkd_virtual_create(uint64_t tick_hz, uint32_t step_tick_hz,
    uint64_t offset_ticks, int32_t drift_ppm, uint32_t actuator_count,
    const double *steps_per_unit, const uint8_t *controller, uint8_t profile);
/* Configure before opening the session. Feedback is published through SENSOR6. */
int32_t rkd_virtual_configure_welder(rkd_virtual_device *device, uint8_t sensor_slot,
    uint32_t arc_channel, uint32_t wire_channel, uint32_t voltage_channel,
    double ignition_seconds, double no_arc_seconds, float efficiency);
int32_t rkd_virtual_set_welder_grounded(rkd_virtual_device *device, uint8_t grounded);
void rkd_virtual_destroy(rkd_virtual_device *device);
int32_t rkd_virtual_step(rkd_virtual_device *device, uint64_t host_ns);
int32_t rkd_virtual_link_host_to_device(rkd_virtual_device *device,
    const uint8_t *bytes, size_t length);
size_t rkd_virtual_link_device_to_host(rkd_virtual_device *device,
    uint8_t *bytes, size_t capacity);
size_t rkd_virtual_actuator_positions(const rkd_virtual_device *device,
    double *positions, size_t capacity);
size_t rkd_virtual_channel_values(const rkd_virtual_device *device,
    float *values, size_t capacity);
size_t rkd_virtual_step_log(const rkd_virtual_device *device,
    rkd_virtual_step_record *records, size_t capacity);
size_t rkd_virtual_event_log(const rkd_virtual_device *device,
    rkd_virtual_event_record *records, size_t capacity);
int32_t rkd_virtual_miss_next_steps(rkd_virtual_device *device,
    uint32_t actuator, uint32_t count);

#ifdef __cplusplus
}
#endif
#endif
