#ifndef ROBOTKIT_SIMKIT_H
#define ROBOTKIT_SIMKIT_H

/**
 * @file robotkit_simkit.h
 * @brief Shared-physics simulation API for RobotKit.
 *
 * A rk_simulation is the set of robots taking part in one SimKit session
 * (nativekit_sim_session.h), which owns the shared world, clock, and the
 * environment. Robot runtimes returned by rk_simulation_add_robot() are
 * bindings into that session; they do not own a physics world and they must
 * not be advanced independently. Every session tick follows this order:
 *
 * 1. drain every attached runtime mailbox;
 * 2. apply every robot command (all or none);
 * 3. advance the common physics host exactly once; and
 * 4. publish one snapshot for every runtime.
 *
 * rk_simulation_create_in_session() attaches robots to a session the caller
 * owns, steps, starts, stops, and resets, and whose environment (objects and
 * actors) the caller edits directly through the session API. Robot topology
 * must be complete before the session's first start or step, because the
 * first tick seals the world layout.
 */

#include "robotkit_runtime.h"
#include "nativekit_sim_session.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle for one shared simulation universe and clock. */
typedef uint32_t rk_simulation RK_HANDLE RK_HANDLE_DESTROY(rk_simulation_destroy);
#define RK_INVALID_SIMULATION ((rk_simulation)0)

/**
 * Read-only clock values published by a Simulation tick.
 *
 * step_index counts successfully completed physics advances. simulation_time
 * is the corresponding fixed-step time in seconds.
 */
typedef struct rk_simulation_clock {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t step_index;
    double simulation_time;
    uint64_t reserved[2];
} rk_simulation_clock;

typedef struct rk_simulation_pose {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t reserved0;
    double position[3];
    double rotation[4];
} rk_simulation_pose;

typedef struct rk_simulation_closure_desc {
    uint32_t parent_link;
    uint32_t child_link;
    uint32_t type; /**< RK_RUNTIME_JOINT_FIXED or RK_RUNTIME_JOINT_REVOLUTE. */
    uint32_t reserved0;
    double anchor_parent[3];
    double axis_parent[3];
} rk_simulation_closure_desc;

/** Optional initial pose for one robot added to a Simulation. */
typedef struct rk_simulation_robot_desc {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t reserved0;
    rk_simulation_pose initial_pose;
    uint64_t reserved[2];
    /* Optional when struct_size includes this tail. 0 keeps the direct
       simulation endpoint; 1 runs RKD6 through the virtual device. */
    uint32_t virtual_device_enabled;
    uint32_t virtual_device_step_tick_hz;
    uint64_t virtual_device_tick_hz;
    uint64_t virtual_device_offset_ticks;
    int32_t virtual_device_drift_ppm;
    uint32_t virtual_device_baud;
    uint64_t virtual_device_latency_ns;
    uint64_t virtual_device_jitter_ns;
    double virtual_device_drop_rate;
    double virtual_device_corruption_rate;
    uint64_t virtual_device_seed;
    double virtual_device_steps_per_unit[64];
    uint8_t virtual_device_fingerprint[16];
    double virtual_device_target_error;
    uint64_t virtual_device_clock_bound_ns;
    uint64_t virtual_device_link_loss_timeout_ns;
    /* Optional RKD6 actuator layout tail. Zero count keeps the v1 identity map. */
    uint32_t virtual_device_actuator_count;
    uint8_t virtual_device_actuator_joint[64];
    double virtual_device_actuator_ratio[64];
    double virtual_device_actuator_offset[64];
    double virtual_device_actuator_steps_per_unit[64];
    double virtual_device_actuator_max_rate[64];
    uint16_t virtual_device_actuator_direction_setup_ticks[64];
    double virtual_device_actuator_skew_bound[64];
    uint8_t virtual_device_actuator_ids[4096]; /**< 64 NUL-terminated ASCII IDs, 64 bytes each. */
    /** Optional origin-centred link boxes; zero extents preserve legacy shape policy. */
    double collision_half_extents[RK_MAX_LINKS * 3];
    /** Optional physical-part convex hulls; 4..64 local XYZ vertices per link. */
    uint32_t collision_hull_count[RK_MAX_LINKS];
    double collision_hull_vertices[RK_MAX_LINKS * 64 * 3];
    uint32_t closure_count;
    rk_simulation_closure_desc closures[64];
    uint32_t virtual_device_profile; /**< 1 full, 2 minimal; zero defaults to full. */
    /** Optional tool collision geoms, expressed in the flange link frame. */
    uint32_t tool_link_index;
    uint32_t tool_piece_count;
    uint32_t tool_piece_vertex_count[16];
    double tool_piece_vertices[16 * 64 * 3];
    double tool_margin;
    double tool_gap;
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
    RK_SIMULATION_PRESENTATION_ROBOT_LINK = 2
} rk_simulation_presentation_pose_kind;
typedef struct rk_simulation_presentation_pose {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t kind;
    uint32_t robot_index;
    uint32_t link_index;
    uint32_t reserved[4];
    double position[3];
    double rotation[4];
} rk_simulation_presentation_pose;

/**
 * Attaches a new, empty set of robots to a session the caller owns.
 *
 * The session must be stopped and must outlive the returned handle.
 * Destroying the handle while the session is stopped removes its robots from
 * the world.
 *
 * @param session A stopped SimKit session.
 * @param out_simulation Receives an owned simulation handle.
 * @return RK_OK on success, or an invalid-state/handle/backend error.
 */
RK_API rk_result RK_CALL rk_simulation_create_in_session(
    nksim_session session, rk_simulation *out_simulation RK_OUT RK_OWNED);
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
 * ticked as an independent runtime; advance the session instead. All robots
 * should be added before the session's first step or start.
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
/** Disconnects or reconnects one virtual RKD6 device in a simulation. */
RK_API rk_result RK_CALL rk_simulation_cut_virtual_device_link(
    rk_simulation simulation, uint32_t robot_index, uint32_t cut);
/**
 * Reads the shared simulation clock.
 *
 * The values are synchronized with completed ticks. This function reports
 * fixed-step simulation time, not wall-clock time and not the observation
 * timestamp supplied by the caller.
 */
RK_API rk_result RK_CALL rk_simulation_get_clock(
    rk_simulation simulation, rk_simulation_clock *out_clock RK_INOUT);
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
/**
 * Contact involving one robot link. tool_piece_index is -1 for link geometry.
 * other_object is the session object (nksim_object) the caller created that
 * the link touches, or zero for another robot link or an unowned body.
 */
typedef struct rk_robot_contact {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t link_index;
    int32_t tool_piece_index;
    nksim_object other_object;
    double distance;
    double position[3];
    double normal[3];
    uint32_t active;
    uint32_t reserved;
} rk_robot_contact;
RK_API rk_result RK_CALL rk_simulation_get_robot_contacts(
    rk_simulation simulation, rk_robot_runtime runtime, rk_robot_contact *out,
    uint32_t capacity, uint32_t *out_count RK_OUT);
RK_API rk_result RK_CALL rk_simulation_get_robot_contact(
    rk_simulation simulation, rk_robot_runtime runtime, uint32_t index,
    rk_robot_contact *out_contact RK_INOUT);
/** Copies the complete clock and every presentation pose under one simulation lock. */
RK_API rk_result RK_CALL rk_simulation_capture_presentation(
    rk_simulation simulation,
    rk_simulation_presentation *out_presentation RK_OUT RK_OWNED);
/**
 * Copies every robot pose from a frame captured from its session, so robots
 * and the session's other participants drawn from one frame agree. Read an
 * environment object's pose directly from the frame (nksim_frame_get_object_pose).
 */
RK_API rk_result RK_CALL rk_simulation_present_frame(
    rk_simulation simulation, nksim_frame frame,
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
