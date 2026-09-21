#ifndef ROBOTKIT_SIMKIT_H
#define ROBOTKIT_SIMKIT_H

#include "robotkit_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t rk_simulation RK_HANDLE RK_HANDLE_DESTROY(rk_simulation_destroy);
#define RK_INVALID_SIMULATION ((rk_simulation)0)

/** Construction parameters for one shared Simulation owner. */
typedef struct rk_simulation_desc {
    uint32_t struct_size RK_STRUCT_SIZE;
    double fixed_timestep;
    uint32_t physics_substeps;
    uint32_t reserved0;
    uint64_t reserved[4];
} rk_simulation_desc;

/** Read-only clock values published by a Simulation tick. */
typedef struct rk_simulation_clock {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t step_index;
    double simulation_time;
    uint64_t reserved[2];
} rk_simulation_clock;

/** Creates one shared simulated universe and its fixed-step clock. */
RK_API rk_result RK_CALL rk_simulation_create(
    const rk_simulation_desc *desc, rk_simulation *out_simulation RK_OUT RK_OWNED);
/** Destroys the simulation and all runtime handles it created. */
RK_API void RK_CALL rk_simulation_destroy(rk_simulation simulation);
/** Adds one robot before the simulation is started or stepped. */
RK_API rk_result RK_CALL rk_simulation_add_robot(
    rk_simulation simulation, const rk_robot_runtime_blueprint *blueprint,
    rk_robot_runtime *out_runtime RK_OUT RK_OWNED);
/** Applies all attached robot commands and advances the world exactly once. */
RK_API rk_result RK_CALL rk_simulation_step(rk_simulation simulation,
                                            uint64_t timestamp_ns);
/** Starts the simulation's realtime owner thread. */
RK_API rk_result RK_CALL rk_simulation_start(rk_simulation simulation);
/** Stops the realtime owner thread without destroying the simulation. */
RK_API rk_result RK_CALL rk_simulation_stop(rk_simulation simulation);
/** Reads the shared simulation clock. */
RK_API rk_result RK_CALL rk_simulation_get_clock(
    rk_simulation simulation, rk_simulation_clock *out_clock RK_INOUT);

#ifdef __cplusplus
}
#endif

#endif
