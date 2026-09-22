#ifndef ROBOTKIT_SIMKIT_H
#define ROBOTKIT_SIMKIT_H

/**
 * @file robotkit_simkit.h
 * @brief Shared-physics simulation API for RobotKit.
 *
 * A rk_simulation owns one SceneKit/SimKit universe and its clock. Robot
 * runtimes returned by rk_simulation_add_robot() are bindings into that
 * universe; they do not own a physics world and they must not be advanced
 * independently. A manual tick follows this order:
 *
 * 1. drain every attached runtime mailbox;
 * 2. apply every robot command;
 * 3. advance the common physics host exactly once; and
 * 4. publish one snapshot for every runtime.
 *
 * Use rk_simulation_step() for deterministic or externally scheduled ticks.
 * Use rk_simulation_start() and rk_simulation_stop() when the simulation
 * should own a realtime worker thread. Robot topology must be complete before
 * the first start or step, because the first tick seals the world layout.
 */

#include "robotkit_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle for one shared simulation universe and clock. */
typedef uint32_t rk_simulation RK_HANDLE RK_HANDLE_DESTROY(rk_simulation_destroy);
#define RK_INVALID_SIMULATION ((rk_simulation)0)
/** Opaque ID for an environment object owned by a Simulation. */
typedef uint32_t rk_simulation_object;
#define RK_INVALID_SIMULATION_OBJECT ((rk_simulation_object)0)

/**
 * Construction parameters for one shared Simulation owner.
 *
 * @note struct_size must be initialized to sizeof(rk_simulation_desc). The
 * fixed timestep is expressed in seconds and must be positive. Physics
 * substeps controls the backend solver subdivisions within one RobotKit tick.
 */
typedef struct rk_simulation_desc {
    uint32_t struct_size RK_STRUCT_SIZE;
    double fixed_timestep;
    uint32_t physics_substeps;
    uint32_t backend; /**< 0: deterministic test backend; 1: MuJoCo (must be built). */
    uint64_t reserved[4];
} rk_simulation_desc;

/**
 * Read-only clock values published by a Simulation tick.
 *
 * step_index counts successfully completed physics advances. simulation_time
 * is the corresponding fixed-step time in seconds; it is independent of the
 * legacy owner-clock hint passed to rk_simulation_step().
 */
typedef struct rk_simulation_clock {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t step_index;
    double simulation_time;
    uint64_t reserved[2];
} rk_simulation_clock;

/** Editable-scene description for one simulation-owned environment object. */
typedef struct rk_simulation_object_desc {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t motion_type; /**< 0 static, 1 kinematic, 2 dynamic. */
    double position[3];
    double rotation[4]; /**< Quaternion in x, y, z, w order. */
    double half_extents[3]; /**< Box dimensions used by the default object shape. */
    double mass;
    uint64_t reserved[2];
} rk_simulation_object_desc;

typedef struct rk_simulation_pose {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t reserved0;
    double position[3];
    double rotation[4];
} rk_simulation_pose;

/**
 * Creates one shared simulated universe and its fixed-step clock.
 *
 * The returned handle owns the SceneKit scene, SimKit world, physics host,
 * clock, and all native resources created for attached robots. No robot
 * runtime exists until rk_simulation_add_robot() is called.
 *
 * @param desc Simulation timing and solver configuration.
 * @param out_simulation Receives an owned simulation handle.
 * @return RK_OK on success, or an argument/backend/allocation error.
 */
RK_API rk_result RK_CALL rk_simulation_create(
    const rk_simulation_desc *desc, rk_simulation *out_simulation RK_OUT RK_OWNED);
/**
 * Destroys a simulation and releases every runtime handle it created.
 *
 * Callers should stop a realtime simulation first. Destroying an invalid or
 * already-destroyed handle is harmless. Runtime handles returned by this
 * simulation become invalid after this call.
 */
RK_API void RK_CALL rk_simulation_destroy(rk_simulation simulation);
/**
 * Adds one robot binding before the simulation is started or stepped.
 *
 * The blueprint becomes immutable simulation topology. The returned runtime
 * accepts commands and publishes snapshots, but it cannot be started or
 * ticked as an independent runtime; advance the owning simulation instead.
 * All robots should be added before the first call to rk_simulation_step() or
 * rk_simulation_start().
 *
 * @param simulation Shared simulation owner.
 * @param blueprint Compiled robot topology and joint metadata.
 * @param out_runtime Receives an owned runtime handle bound to this simulation.
 * @return RK_OK on success, or an invalid-state/argument/backend error.
 */
RK_API rk_result RK_CALL rk_simulation_add_robot(
    rk_simulation simulation, const rk_robot_runtime_blueprint *blueprint,
    rk_robot_runtime *out_runtime RK_OUT RK_OWNED);
/**
 * Applies all attached robot commands and advances the world exactly once.
 *
 * timestamp_ns is a compatibility owner-clock hint. Simulation samples use
 * fixed simulation source time and actual local monotonic receipt time,
 * neither derived from this argument. If any command fails, the world is not
 * advanced and staged commands are discarded so robots cannot observe a
 * partially committed tick.
 *
 * @param simulation Shared simulation owner.
 * @param timestamp_ns Legacy owner-clock hint, ignored by simulated sensors.
 * @return RK_OK after one complete world advance, or an error with no advance.
 */
RK_API rk_result RK_CALL rk_simulation_step(rk_simulation simulation,
                                            uint64_t timestamp_ns);
/**
 * Starts the simulation's realtime owner thread.
 *
 * Topology is sealed on the first successful start. While running, callers
 * submit commands through runtime handles and read snapshots; manual calls to
 * rk_simulation_step() are rejected until rk_simulation_stop() completes.
 */
RK_API rk_result RK_CALL rk_simulation_start(rk_simulation simulation);
/**
 * Stops the realtime owner thread without destroying the simulation.
 *
 * The simulation handle and its latest clock/snapshot remain available for
 * inspection. Use rk_simulation_start() again only if the backend and caller
 * lifecycle permit restarting the shared clock.
 */
RK_API rk_result RK_CALL rk_simulation_stop(rk_simulation simulation);
/**
 * Reads the shared simulation clock.
 *
 * The values are synchronized with completed ticks. This function reports
 * fixed-step simulation time, not wall-clock time and not the observation
 * timestamp supplied by the caller.
 */
RK_API rk_result RK_CALL rk_simulation_get_clock(
    rk_simulation simulation, rk_simulation_clock *out_clock RK_INOUT);
/** Stops the owner, restores all bodies, and resets the shared fixed-step clock. */
RK_API rk_result RK_CALL rk_simulation_reset(rk_simulation simulation);
/** Restores one attached robot's bodies and clears its runtime state. */
RK_API rk_result RK_CALL rk_simulation_reset_robot(rk_simulation simulation,
                                                    uint32_t robot_index);
/** Teleports one attached robot's base while the simulation is stopped. */
RK_API rk_result RK_CALL rk_simulation_teleport_robot(
    rk_simulation simulation, uint32_t robot_index,
    const rk_simulation_pose *pose);
/** Adds one environment body from the editable scene while stopped. */
RK_API rk_result RK_CALL rk_simulation_spawn_object(
    rk_simulation simulation, const rk_simulation_object_desc *desc,
    rk_simulation_object *out_object RK_OUT);
/** Removes one environment body while stopped. */
RK_API rk_result RK_CALL rk_simulation_remove_object(
    rk_simulation simulation, rk_simulation_object object);
/** Teleports one environment body while stopped. */
RK_API rk_result RK_CALL rk_simulation_teleport_object(
    rk_simulation simulation, rk_simulation_object object,
    const rk_simulation_pose *pose);

#ifdef __cplusplus
}
#endif

#endif
