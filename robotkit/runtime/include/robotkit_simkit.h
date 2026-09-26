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

/** Optional initial pose for one robot added to a Simulation. */
typedef struct rk_simulation_robot_desc {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t reserved0;
    rk_simulation_pose initial_pose;
    uint64_t reserved[2];
} rk_simulation_robot_desc;

/**
 * Ideal rolling differential-drive coupling for one robot's kinematic base.
 *
 * The wheel joints are robot joint indices of actuated wheel joints. Lengths
 * are metres and must be positive; track_width is the distance between the
 * wheel contact points.
 */
typedef struct rk_simulation_differential_drive_desc {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t left_wheel_joint;
    uint32_t right_wheel_joint;
    uint32_t reserved0;
    double wheel_radius;
    double track_width;
    uint64_t reserved[2];
} rk_simulation_differential_drive_desc;

/**
 * Differential-drive plant state after the latest completed tick.
 *
 * x, y, and height are the base position written to the kinematic base in
 * metres; yaw is its heading in radians (the direction of the base's x axis
 * projected on the floor), unwrapped so it stays continuous across turns. The
 * base's rotation is the yaw about world Z composed with the roll and pitch it
 * had when the plant was seeded. The wheel rates are the velocity targets, in rad/s, the robot
 * applied for that tick after runtime clamping: zero after a stop, a reset, or
 * while a wheel is idle or held by a non-velocity target.
 */
typedef struct rk_simulation_differential_drive_state {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t enabled; /**< Nonzero while the coupling is active. */
    double x;
    double y;
    double yaw;
    double height;
    double left_wheel_rate;
    double right_wheel_rate;
    uint64_t reserved[2];
} rk_simulation_differential_drive_state;

/**
 * Ideal rolling omni-wheel coupling for one robot's kinematic base.
 *
 * Three omni wheels, each at mount angle wheel_angles[i] (radians about +Z
 * from the base's x axis) and base_radius metres from the base centre, roll
 * tangentially: for a body twist (vx, vy, omega) in the base frame, wheel i's
 * rim speed is -sin(a_i) vx + cos(a_i) vy + base_radius omega, and its joint
 * rate is that speed over wheel_radius. A kiwi drive uses angles
 * pi/2 + i * 2pi/3. The wheel joints are robot joint indices of actuated
 * wheel joints; the wheels must span every planar motion.
 */
typedef struct rk_simulation_omni_drive_desc {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t wheel_joints[3];
    double wheel_angles[3];
    double wheel_radius;
    double base_radius;
    uint64_t reserved[2];
} rk_simulation_omni_drive_desc;

/**
 * Omni-wheel plant state after the latest completed tick, with the same
 * x, y, yaw, and height meaning as rk_simulation_differential_drive_state.
 * The wheel rates are the velocity targets, in rad/s, the robot applied for
 * that tick after runtime clamping.
 */
typedef struct rk_simulation_omni_drive_state {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t enabled; /**< Nonzero while the coupling is active. */
    double x;
    double y;
    double yaw;
    double height;
    double wheel_rates[3];
    uint64_t reserved[2];
} rk_simulation_omni_drive_state;

/** Immutable copied presentation snapshot captured under one simulation lock. */
typedef uint32_t rk_simulation_presentation RK_HANDLE RK_HANDLE_DESTROY(rk_simulation_presentation_destroy);
#define RK_INVALID_SIMULATION_PRESENTATION ((rk_simulation_presentation)0)
typedef struct rk_simulation_presentation_info {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t step_index;
    double simulation_time;
    uint32_t pose_count;
    uint32_t reserved[3];
} rk_simulation_presentation_info;
typedef enum rk_simulation_presentation_pose_kind {
    RK_SIMULATION_PRESENTATION_ROBOT_BASE = 1,
    RK_SIMULATION_PRESENTATION_ROBOT_LINK = 2,
    RK_SIMULATION_PRESENTATION_ENVIRONMENT = 3
} rk_simulation_presentation_pose_kind;
typedef struct rk_simulation_presentation_pose {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t kind;
    uint32_t robot_index;
    uint32_t link_index;
    uint32_t object_id;
    uint32_t reserved[3];
    double position[3];
    double rotation[4];
} rk_simulation_presentation_pose;

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
 * @param robot_desc Optional versioned initial-pose descriptor. A null pointer
 * uses the existing default pose `(robot_index, 0, 0)` with identity yaw.
 * @param out_runtime Receives an owned runtime handle bound to this simulation.
 * @return RK_OK on success, or an invalid-state/argument/backend error.
 */
RK_API rk_result RK_CALL rk_simulation_add_robot(
    rk_simulation simulation, const rk_robot_runtime_blueprint *blueprint,
    const rk_simulation_robot_desc *robot_desc,
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
/** Teleports one attached robot's base while the simulation is stopped.
 * This does not change the pose restored by reset or resetRobot. */
RK_API rk_result RK_CALL rk_simulation_teleport_robot(
    rk_simulation simulation, uint32_t robot_index,
    const rk_simulation_pose *pose);
/**
 * Drives one attached robot's kinematic base to a pose for the next tick.
 *
 * Unlike rk_simulation_teleport_robot(), this is accepted while the shared
 * clock is running or stepped externally: it does not stop the owner, reset
 * sensors, or change the pose restored by reset. The pose becomes the base
 * body's physics state by the end of the next completed tick, and the base's
 * velocity over that tick is the motion from the pose it held at the end of
 * the previous tick, differenced in double precision, so derivative sensors
 * such as the IMU measure consecutive drives as continuous motion without
 * single-precision scene rounding, however far from the origin. A tick with no
 * drive holds the base at rest. Use rk_simulation_place_robot_base() for a
 * jump that must not read as motion.
 *
 * @param simulation Shared simulation owner.
 * @param robot_index Index of the robot in attachment order.
 * @param pose Target base pose with a unit quaternion rotation.
 * @return RK_OK, or an invalid-argument/invalid-state/backend error.
 */
RK_API rk_result RK_CALL rk_simulation_drive_robot_base(
    rk_simulation simulation, uint32_t robot_index,
    const rk_simulation_pose *pose);
/**
 * Jumps one attached robot's base to a pose for the next tick.
 *
 * Like rk_simulation_drive_robot_base(), this is accepted while the shared
 * clock is running or stepped externally, and it does not reset sensors or
 * change the pose restored by reset. Unlike a drive, the jump is a teleport:
 * the base keeps its body-frame velocity through it instead of the pose change
 * reading as motion, so derivative sensors such as the IMU see no spike. A
 * differential-drive plant continues from the new pose.
 *
 * @param simulation Shared simulation owner.
 * @param robot_index Index of the robot in attachment order.
 * @param pose Target base pose with a unit quaternion rotation.
 * @return RK_OK, or an invalid-argument/backend error.
 */
RK_API rk_result RK_CALL rk_simulation_place_robot_base(
    rk_simulation simulation, uint32_t robot_index,
    const rk_simulation_pose *pose);
/**
 * Couples one robot's wheel velocity targets to its kinematic base.
 *
 * Every tick, after the robot applies its commands and before physics
 * advances, the base rolls along a constant-curvature arc by the wheel targets
 * the robot applied for that tick (rate-clamped by the runtime, zero after a
 * normal or emergency stop). The plant starts from the base's current pose.
 * The wheels roll on a level floor: the base translates in the world XY plane
 * at its current height and turns about world Z, and keeps the roll and pitch
 * it had when seeded (they turn with the heading), so a tilted chassis stays
 * tilted. The base's velocity is the plant's exact double-precision twist.
 * Whatever submits the targets, the base follows with no added latency.
 * Replaces any previous coupling for the robot; accepted while running.
 *
 * @param simulation Shared simulation owner.
 * @param robot_index Index of the robot in attachment order.
 * @param desc Wheel joints and geometry.
 * @return RK_OK, or RK_ERROR_INVALID_ARGUMENT for an unknown robot, a fixed or
 * missing wheel joint, identical wheels, or non-positive geometry.
 */
RK_API rk_result RK_CALL rk_simulation_set_differential_drive(
    rk_simulation simulation, uint32_t robot_index,
    const rk_simulation_differential_drive_desc *desc);
/** Removes a robot's differential-drive coupling, if it has one; its base stays where it is. */
RK_API rk_result RK_CALL rk_simulation_clear_differential_drive(
    rk_simulation simulation, uint32_t robot_index);
/** Reads one robot's differential-drive plant state. */
RK_API rk_result RK_CALL rk_simulation_get_differential_drive_state(
    rk_simulation simulation, uint32_t robot_index,
    rk_simulation_differential_drive_state *out_state RK_INOUT);
/**
 * Couples one robot's three omni-wheel velocity targets to its kinematic base.
 *
 * Behaves like rk_simulation_set_differential_drive, including its floor,
 * tilt, and zero-latency rules, but the body twist decoded from the applied
 * wheel rates may include a lateral component, so the base can strafe. A
 * robot has one drive coupling: this replaces a differential one and vice
 * versa.
 *
 * @return RK_OK, or RK_ERROR_INVALID_ARGUMENT for an unknown robot, a fixed,
 * missing, or repeated wheel joint, non-positive or non-finite geometry, or
 * wheel angles that cannot span every planar motion.
 */
RK_API rk_result RK_CALL rk_simulation_set_omni_drive(
    rk_simulation simulation, uint32_t robot_index,
    const rk_simulation_omni_drive_desc *desc);
/** Removes a robot's omni-wheel coupling, if it has one; its base stays where it is. */
RK_API rk_result RK_CALL rk_simulation_clear_omni_drive(
    rk_simulation simulation, uint32_t robot_index);
/** Reads one robot's omni-wheel plant state. */
RK_API rk_result RK_CALL rk_simulation_get_omni_drive_state(
    rk_simulation simulation, uint32_t robot_index,
    rk_simulation_omni_drive_state *out_state RK_INOUT);
/** Reads one robot base pose from the latest physics state. */
RK_API rk_result RK_CALL rk_simulation_get_robot_pose(
    rk_simulation simulation, uint32_t robot_index,
    rk_simulation_pose *out_pose RK_INOUT);
/** Reads one robot link pose from the latest physics state. */
RK_API rk_result RK_CALL rk_simulation_get_link_pose(
    rk_simulation simulation, uint32_t robot_index, uint32_t link_index,
    rk_simulation_pose *out_pose RK_INOUT);
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
/** Reads one environment object's pose from the latest physics state. */
RK_API rk_result RK_CALL rk_simulation_get_object_pose(
    rk_simulation simulation, rk_simulation_object object,
    rk_simulation_pose *out_pose RK_INOUT);
/** Copies the complete clock and every presentation pose under one simulation lock. */
RK_API rk_result RK_CALL rk_simulation_capture_presentation(
    rk_simulation simulation,
    rk_simulation_presentation *out_presentation RK_OUT RK_OWNED);
/** Reads immutable metadata from a captured presentation snapshot. */
RK_API rk_result RK_CALL rk_simulation_presentation_get_info(
    rk_simulation_presentation presentation,
    rk_simulation_presentation_info *out_info RK_INOUT);
/** Reads one pose from the immutable presentation snapshot, without locking the simulation. */
RK_API rk_result RK_CALL rk_simulation_presentation_get_pose(
    rk_simulation_presentation presentation, uint32_t index,
    rk_simulation_presentation_pose *out_pose RK_INOUT);
/** Destroys a captured presentation snapshot. */
RK_API void RK_CALL rk_simulation_presentation_destroy(
    rk_simulation_presentation presentation);

#ifdef __cplusplus
}
#endif

#endif
