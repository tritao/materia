#ifndef ROBOTKIT_RUNTIME_H
#define ROBOTKIT_RUNTIME_H

#include "robotkit_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t rk_runtime RK_HANDLE RK_HANDLE_DESTROY(rk_runtime_destroy);

#define RK_INVALID_RUNTIME ((rk_runtime)0)

/** Creates the initial deterministic in-memory runtime endpoint. */
RK_API rk_result RK_CALL rk_runtime_create(const rk_runtime_layout *layout,
                                           rk_runtime *out_runtime RK_OUT RK_OWNED);
RK_API void RK_CALL rk_runtime_destroy(rk_runtime runtime);

RK_API rk_result RK_CALL rk_runtime_start(rk_runtime runtime);
RK_API rk_result RK_CALL rk_runtime_stop(rk_runtime runtime);

/** Submits one complete command batch to the runtime mailbox. */
RK_API rk_result RK_CALL rk_runtime_submit(rk_runtime runtime,
                                           const rk_robot_command *command);

/** Advances a stopped runtime by one deterministic owner-thread tick. */
RK_API rk_result RK_CALL rk_runtime_step(rk_runtime runtime, uint64_t timestamp_ns);

/** Copies the latest immutable state into the caller-provided value. */
RK_API rk_result RK_CALL rk_runtime_snapshot(rk_runtime runtime,
                                             rk_robot_state *out_state RK_INOUT);
/** Copies the latest immutable published snapshot, including endpoint metadata. */
RK_API rk_result RK_CALL rk_runtime_snapshot_full(
    rk_runtime runtime, rk_robot_snapshot *out_snapshot RK_INOUT);
RK_API rk_result RK_CALL rk_runtime_capabilities(
    rk_runtime runtime, rk_robot_capabilities *out_capabilities RK_INOUT);

#ifdef __cplusplus
}
#endif

#endif
