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

/** Kinds of primitive link collision shape. Sizes are in metres. */
typedef uint32_t rk_simulation_link_shape_type;
enum {
    RK_LINK_SHAPE_BOX = 1,      /**< size: half extents. */
    RK_LINK_SHAPE_SPHERE = 2,   /**< size[0]: radius. */
    /** size[0]: radius; size[1]: half-length of the straight part, along local Z. */
    RK_LINK_SHAPE_CAPSULE = 3,
    /** size[0]: radius; size[1]: half-length along local Z. */
    RK_LINK_SHAPE_CYLINDER = 4
};

enum { RK_MAX_LINK_SHAPES = 256 };
enum { RK_MAX_LINK_HULLS = 512 };

/**
 * One convex hull of a link built from several rigid parts, as XYZ vertices in
 * the link frame; 4..64 vertices.
 */
typedef struct rk_simulation_link_hull {
    uint32_t link;
    uint32_t vertex_count;
    double vertices[64 * 3];
} rk_simulation_link_hull;

/** One primitive collision shape attached to a robot link, posed in the link frame. */
typedef struct rk_simulation_link_shape {
    uint32_t link;
    uint32_t type; /**< rk_simulation_link_shape_type. */
    double size[3];
    double position[3];
    double rotation[4]; /**< Unit quaternion in x, y, z, w order. */
    /**
     * Contact surface; zero fields keep the backend default. friction is
     * sliding, torsional and rolling; friction_dimensions is 1, 3, 4 or 6;
     * contact_time_constant (s) and contact_damping_ratio set soft-contact
     * stiffness and damping.
     */
    double friction[3];
    double contact_time_constant;
    double contact_damping_ratio;
    uint32_t friction_dimensions;
    /**
     * NKSIM_CONTACT_* from nativekit_sim.h: 0 collides by layers like other
     * links; 1 only through contact pairs; 2 through pairs and with every
     * environment object, using this shape's surface.
     */
    uint32_t contact_filter;
} rk_simulation_link_shape;

enum { RK_MAX_CONTACT_PAIRS = 128 };

/**
 * An explicit contact between two of a robot's link shapes, given as indices
 * into rk_simulation_robot_desc.link_shapes, with its own surface (see
 * rk_simulation_link_shape; zero fields keep the backend default).
 */
typedef struct rk_simulation_contact_pair {
    uint32_t shape_a;
    uint32_t shape_b;
    double friction[3];
    double contact_time_constant;
    double contact_damping_ratio;
    uint32_t friction_dimensions;
    uint32_t reserved0;
} rk_simulation_contact_pair;

/**
 * Optional settings for one robot added to a Simulation. An initial_pose whose
 * struct_size is zero keeps the default placement.
 */
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
    uint8_t virtual_device_controller[16];
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
    /**
     * Optional primitive collision shapes. A link that has any collides
     * through them together with its hull or box above, if given, and its
     * tool pieces; otherwise the link keeps the legacy shape policy.
     */
    uint32_t link_shape_count;
    rk_simulation_link_shape link_shapes[RK_MAX_LINK_SHAPES];
    /** Optional explicit contacts between link shapes. */
    uint32_t contact_pair_count;
    rk_simulation_contact_pair contact_pairs[RK_MAX_CONTACT_PAIRS];
    /**
     * Optional convex hulls of links built from several rigid parts, one per
     * part. A link collides through a compound of its hull or box above, these
     * hulls, its primitives and its tool pieces.
     */
    uint32_t link_hull_count;
    rk_simulation_link_hull link_hulls[RK_MAX_LINK_HULLS];
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
/**
 * Reads the RobotKit error behind the latest failed tick, such as
 * RK_ERROR_LIMIT for a command beyond a joint's limits, or RK_OK. The session
 * that stepped reports only that a participant failed.
 */
RK_API rk_result RK_CALL rk_simulation_get_rejection(
    rk_simulation simulation, rk_result *out_result RK_OUT);
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
 * jump that must not read as motion. A floating base moves only under
 * physics, so driving one returns RK_ERROR_INVALID_STATE.
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
 * missing wheel joint, identical wheels, or non-positive geometry, or
 * RK_ERROR_INVALID_STATE for a floating-base robot.
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
 * wheel angles that cannot span every planar motion, or
 * RK_ERROR_INVALID_STATE for a floating-base robot.
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
/**
 * Places one attached robot's joints at positions, in joint order, as a pose
 * to start from, such as a humanoid's standing keyframe. Velocities become
 * zero. Accepted only while the simulation is stopped, like
 * rk_simulation_teleport_robot(); reset returns joints to zero. Fixed joints
 * must be given 0.
 */
RK_API rk_result RK_CALL rk_simulation_set_joint_positions(
    rk_simulation simulation, uint32_t robot_index,
    const double *positions RK_IN_ARRAY(count), uint32_t count);
/** Reads one robot base pose from the latest physics state. */
RK_API rk_result RK_CALL rk_simulation_get_robot_pose(
    rk_simulation simulation, uint32_t robot_index,
    rk_simulation_pose *out_pose RK_INOUT);
/** World-frame twist of one body: metres per second and radians per second. */
typedef struct rk_simulation_twist {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t reserved0;
    double linear[3];
    double angular[3];
} rk_simulation_twist;
/**
 * Reads one robot base's world-frame twist from the latest physics state.
 * A floating base (rk_robot_runtime_blueprint.floating_base) reports its free
 * motion; a kinematic base reports the twist it was driven with.
 */
RK_API rk_result RK_CALL rk_simulation_get_robot_base_velocity(
    rk_simulation simulation, uint32_t robot_index,
    rk_simulation_twist *out_twist RK_INOUT);
/** A force (N) and torque (N m), world frame, at a body's centre of mass. */
typedef struct rk_simulation_wrench {
    uint32_t struct_size RK_STRUCT_SIZE;
    double force[3];
    double torque[3];
} rk_simulation_wrench;
/**
 * Pushes one robot's base for the next tick, on top of any already applied for
 * that tick. Repeat every tick to push for longer. A kinematic base ignores
 * it, as it ignores every force.
 */
RK_API rk_result RK_CALL rk_simulation_apply_robot_force(
    rk_simulation simulation, uint32_t robot_index, const rk_simulation_wrench *wrench);
/** Reads one robot link pose from the latest physics state. */
RK_API rk_result RK_CALL rk_simulation_get_link_pose(
    rk_simulation simulation, uint32_t robot_index, uint32_t link_index,
    rk_simulation_pose *out_pose RK_INOUT);
/**
 * The physics body that carries one robot link. Pass it to nksim_session_hold_object() as the
 * carrier to attach a session object to that link, for instance a workpiece held by a suction cup.
 */
RK_API rk_result RK_CALL rk_simulation_get_link_body(
    rk_simulation simulation, uint32_t robot_index, uint32_t link_index,
    nksim_body *out_body RK_OUT);
/**
 * Contact involving one robot link. tool_piece_index is -1 for link geometry.
 * other_object is nonzero only for Object contacts. RobotLink contacts carry
 * zero-based other_robot/other_link; World covers unowned geometry.
 */
typedef enum rk_contact_other_kind {
    RK_CONTACT_OTHER_WORLD = 0,
    RK_CONTACT_OTHER_OBJECT = 1,
    RK_CONTACT_OTHER_ROBOT_LINK = 2
} rk_contact_other_kind;
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
    rk_contact_other_kind other_kind;
    uint32_t other_robot;
    uint32_t other_link;
    uint32_t reserved2;
} rk_robot_contact;
typedef uint32_t rk_robot_contact_list RK_HANDLE RK_HANDLE_DESTROY(rk_robot_contact_list_destroy);
#define RK_INVALID_ROBOT_CONTACT_LIST ((rk_robot_contact_list)0)
RK_API rk_result RK_CALL rk_simulation_capture_robot_contacts(
    rk_simulation simulation, rk_robot_runtime runtime,
    rk_robot_contact_list *out_list RK_OUT RK_OWNED);
RK_API rk_result RK_CALL rk_robot_contact_list_count(
    rk_robot_contact_list list, uint32_t *out_count RK_OUT);
RK_API rk_result RK_CALL rk_robot_contact_list_get(
    rk_robot_contact_list list, uint32_t index, rk_robot_contact *out_contact RK_INOUT);
RK_API rk_result RK_CALL rk_robot_contact_list_step_index(
    rk_robot_contact_list list, uint64_t *out_step_index RK_OUT);
RK_API void RK_CALL rk_robot_contact_list_destroy(rk_robot_contact_list list);
/** A zero out[0].struct_size uses sizeof(rk_robot_contact). Otherwise it
 * supplies the stride and copied prefix size for each output entry. */
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
