#ifndef NATIVEKIT_SIM_HOST_H
#define NATIVEKIT_SIM_HOST_H

#include "nativekit_sim.h"

#include <stdint.h>

#if defined(_WIN32)
#if defined(NKSIM_STATIC)
#define NKSIM_HOST_API
#elif defined(NKSIM_BUILDING_LIBRARY)
#define NKSIM_HOST_API __declspec(dllexport)
#else
#define NKSIM_HOST_API __declspec(dllimport)
#endif
#define NKSIM_HOST_CALL __cdecl
#else
#define NKSIM_HOST_API __attribute__((visibility("default")))
#define NKSIM_HOST_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** A dedicated owner thread for a synchronous nksim_world. */
typedef uint32_t nksim_host NK_HANDLE NK_HANDLE_DESTROY(nksim_host_destroy);

#define NKSIM_INVALID_HOST ((nksim_host)0)

enum {
    NKSIM_HOST_MODE_REALTIME = 1,
    NKSIM_HOST_MODE_UNBOUNDED = 2,
    NKSIM_HOST_MODE_EXTERNAL = 3
};

typedef struct nksim_host_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    nksim_world world;
    uint32_t mode;
    /** Wall-clock scheduling multiplier. Zero selects 1.0. */
    double time_scale;
    /** Maximum queued command packets. Zero selects the default. */
    uint32_t command_capacity;
    uint64_t reserved[4];
} nksim_host_desc;

typedef struct nksim_host_status {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t running NK_BOOL32;
    uint32_t paused NK_BOOL32;
    uint32_t mode;
    uint32_t reserved0;
    uint64_t step_index;
    double simulation_time;
    nksim_result last_error;
    uint32_t queued_commands;
    uint64_t reserved[3];
} nksim_host_status;

NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_create(
    const nksim_host_desc *desc, nksim_host *out_host NK_OUT NK_OWNED);
NKSIM_HOST_API void NKSIM_HOST_CALL nksim_host_destroy(nksim_host host);

NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_start(nksim_host host);
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_stop(nksim_host host);
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_pause(nksim_host host);
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_resume(nksim_host host);

/** Queue one fixed simulation tick and wait for its result. */
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_step(
    nksim_host host, nksim_step_result *out_result NK_INOUT);

NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_submit_forces(
    nksim_host host, const nksim_body_force *forces NK_IN_ARRAY(count), uint32_t count);
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_submit_joint_targets(
    nksim_host host, const nksim_joint_target *targets NK_IN_ARRAY(count), uint32_t count);
/**
 * Queue body state writes applied on the owner thread before the next tick,
 * with the same semantics as nksim_body_set_state(). For a kinematic body the
 * write is a discontinuity: the next tick does not infer a velocity from the
 * pose jump and carries the supplied twist instead, then later ticks resume
 * inferring the twist from the body's scene-node motion.
 */
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_submit_body_states(
    nksim_host host, const nksim_body_state *states NK_IN_ARRAY(count), uint32_t count);
/**
 * Queue kinematic body drives applied on the owner thread before the next
 * tick, with the same semantics as nksim_body_drive(): continuous motion to
 * the supplied pose with exactly the supplied twist.
 */
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_submit_body_drives(
    nksim_host host, const nksim_body_state *states NK_IN_ARRAY(count), uint32_t count);

/** Return the latest immutable snapshot published by the owner thread. */
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_get_snapshot(
    nksim_host host, nksim_snapshot *out_snapshot NK_OUT NK_OWNED);
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_get_clock(
    nksim_host host, nksim_clock *out_clock NK_INOUT);
NKSIM_HOST_API nksim_result NKSIM_HOST_CALL nksim_host_get_status(
    nksim_host host, nksim_host_status *out_status NK_INOUT);

#ifdef __cplusplus
}
#endif

#endif /* NATIVEKIT_SIM_HOST_H */
