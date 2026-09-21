#ifndef ROBOTKIT_SIMKIT_H
#define ROBOTKIT_SIMKIT_H

#include "robotkit_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t rk_simulation RK_HANDLE RK_HANDLE_DESTROY(rk_simulation_destroy);
#define RK_INVALID_SIMULATION ((rk_simulation)0)

typedef struct rk_simulation_desc {
    uint32_t struct_size RK_STRUCT_SIZE;
    double fixed_timestep;
    uint32_t physics_substeps;
    uint32_t reserved0;
    uint64_t reserved[4];
} rk_simulation_desc;

typedef struct rk_simulation_clock {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t step_index;
    double simulation_time;
    uint64_t reserved[2];
} rk_simulation_clock;

RK_API rk_result RK_CALL rk_simulation_create(
    const rk_simulation_desc *desc, rk_simulation *out_simulation RK_OUT RK_OWNED);
RK_API void RK_CALL rk_simulation_destroy(rk_simulation simulation);
RK_API rk_result RK_CALL rk_simulation_add_robot(
    rk_simulation simulation, const rk_runtime_blueprint *blueprint,
    rk_runtime *out_runtime RK_OUT RK_OWNED);
RK_API rk_result RK_CALL rk_simulation_step(rk_simulation simulation,
                                            uint64_t timestamp_ns);
RK_API rk_result RK_CALL rk_simulation_start(rk_simulation simulation);
RK_API rk_result RK_CALL rk_simulation_stop(rk_simulation simulation);
RK_API rk_result RK_CALL rk_simulation_get_clock(
    rk_simulation simulation, rk_simulation_clock *out_clock RK_INOUT);

/** Compatibility helper creating an implicit one-robot simulation. */
RK_API rk_result RK_CALL rk_runtime_create_sim(
    const rk_runtime_blueprint *blueprint,
    rk_runtime *out_runtime RK_OUT RK_OWNED);

#ifdef __cplusplus
}
#endif

#endif
