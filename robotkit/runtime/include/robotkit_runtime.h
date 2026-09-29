#ifndef ROBOTKIT_RUNTIME_H
#define ROBOTKIT_RUNTIME_H

/**
 * @file robotkit_runtime.h
 * @brief C ABI for RobotKit command mailboxes and robot state publication.
 *
 * This header defines the backend-neutral contract shared by simulation,
 * physical, replay, and test endpoints. A RobotRuntime owns one command
 * mailbox and publishes immutable observations; it does not expose native
 * backend handles. All extensible structs begin with struct_size so callers
 * and libraries can validate ABI compatibility before reading fields.
 */

/* ------------------------------------------------------------------------- */
/* Dependencies                                                              */
/* ------------------------------------------------------------------------- */

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* API annotations                                                           */
/* ------------------------------------------------------------------------- */

/* Haxeon FFI annotations are inert for ordinary C/C++ consumers. */
#if defined(__clang__)
#define RK_OUT __attribute__((annotate("hxi:out")))
#define RK_INOUT __attribute__((annotate("hxi:inout")))
#define RK_HANDLE __attribute__((annotate("hxi:handle")))
#define RK_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#define RK_OWNED __attribute__((annotate("hxi:owned")))
#define RK_UTF8 __attribute__((annotate("hxi:utf8")))
#define RK_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#define RK_OUT_BUFFER(size) __attribute__((annotate("hxi:out_buffer")))
#define RK_STRUCT_SIZE __attribute__((annotate("hxi:struct_size")))
#else
#define RK_OUT
#define RK_INOUT
#define RK_HANDLE
#define RK_HANDLE_DESTROY(symbol)
#define RK_OWNED
#define RK_UTF8
#define RK_IN_ARRAY(count)
#define RK_OUT_BUFFER(size)
#define RK_STRUCT_SIZE
#endif

#if defined(_WIN32)
#if defined(RK_STATIC)
#define RK_API
#elif defined(RK_BUILDING_LIBRARY)
#define RK_API __declspec(dllexport)
#else
#define RK_API __declspec(dllimport)
#endif
#define RK_CALL __cdecl
#else
#define RK_API __attribute__((visibility("default")))
#define RK_CALL
#endif

/* ------------------------------------------------------------------------- */
/* C linkage                                                                 */
/* ------------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Limits and result codes                                                   */
/* ------------------------------------------------------------------------- */

/** API-wide limits and version identifiers. */
enum {
    RK_MAX_JOINTS = 512, /**< Maximum joints carried by one fixed-size ABI value. */
    RK_MAX_LINKS = 1024,
    RK_MAX_SERIAL_JOINTS = 64, /**< Capacity of the current serial wire protocol. */
    RK_MAX_TRAJECTORY_JOINTS = 64, /**< Maximum joints represented by one trajectory chunk. */
    RK_MAX_TRAJECTORY_SEGMENTS = 128, /**< Maximum polynomial segments per chunk. */
    RK_MAX_TRAJECTORY_COEFFICIENTS = 4096, /**< Maximum used scalar coefficients per segment chunk. */
    RK_MAX_TRAJECTORY_QUEUE_POINTS = 4096, /**< Maximum queued segment-start knots. */
    RK_MAX_SENSORS = 8,
    RK_MAX_SENSOR_VALUES = 64,
    RK_MAX_PROCESS_CHANNELS = 32,
    RK_MAX_PLAN_EVENTS = 256,
    RK_MAX_EVENT_RECORDS = 64,
    RK_PROCESS_CHANNEL_ID_BYTES = 48,
    RK_PROCESS_COMMAND_BYTES = 48,
    RK_MAX_JOINT_COUPLINGS = 512,
    RK_API_VERSION = 21 /**< Adds simulation virtual-device link-loss injection. */
};

/** Result returned by RobotKit C ABI functions. */
typedef int32_t rk_result;
enum {
    RK_OK = 0, /**< Operation completed successfully. */
    RK_ERROR_INVALID_ARGUMENT = -1, /**< A pointer, value, or struct size is invalid. */
    RK_ERROR_INVALID_STATE = -2, /**< The operation is incompatible with lifecycle state. */
    RK_ERROR_QUEUE_FULL = -3, /**< The command mailbox cannot accept another batch. */
    RK_ERROR_UNSUPPORTED = -4, /**< The endpoint or backend does not support the request. */
    RK_ERROR_SAFETY_STOPPED = -5, /**< Safety state rejects the requested command. */
    RK_ERROR_INVALID_HANDLE = -6, /**< The opaque handle is null, stale, or foreign. */
    RK_ERROR_OUT_OF_MEMORY = -7, /**< Native allocation failed. */
    RK_ERROR_BACKEND = -8, /**< The underlying endpoint or physics backend failed. */
    RK_ERROR_STALE_COMMAND = -9, /**< Command sequence is not newer than the last accepted command. */
    RK_ERROR_LIMIT = -10, /**< Command violates a compiled joint or actuator limit. */
    RK_ERROR_STALE_STATE = -11, /**< The endpoint only supplied an old observation. */
    RK_ERROR_MODEL_MISMATCH = -12, /**< Device layout fingerprint differs from the host deployment. */
    RK_ERROR_FOLLOWING_ERROR = -13 /**< Measured state exceeds the configured distance from the commanded setpoint. */
};

/* ------------------------------------------------------------------------- */
/* Persistent recording                                                      */
/* ------------------------------------------------------------------------- */

typedef uint32_t rk_recording_compression;
enum { RK_RECORDING_COMPRESSION_NONE = 0, RK_RECORDING_COMPRESSION_LZ4 = 1 };

typedef uint32_t rk_recording_state;
enum {
    RK_RECORDING_OPEN = 1,
    RK_RECORDING_CLOSING = 2,
    RK_RECORDING_CLOSED = 3,
    RK_RECORDING_FAILED = 4
};

typedef struct rk_recording_writer_handle { uint32_t id; } rk_recording_writer_handle RK_HANDLE RK_HANDLE_DESTROY(rk_recording_writer_destroy);
typedef struct rk_recording_reader_handle { uint32_t id; } rk_recording_reader_handle RK_HANDLE RK_HANDLE_DESTROY(rk_recording_reader_destroy);

typedef struct rk_recording_writer_status {
    uint32_t struct_size RK_STRUCT_SIZE;
    rk_recording_state state;
    uint64_t accepted;
    uint64_t written;
    uint64_t dropped;
    uint64_t queued;
    uint64_t queued_bytes;
    char error[256];
} rk_recording_writer_status;

typedef struct rk_recording_message {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t schema_version;
    uint32_t payload_size;
    uint64_t ordinal;
    uint64_t recording_timestamp_ns;
    char topic[128];
} rk_recording_message;

RK_API rk_result RK_CALL rk_recording_writer_create(const char *path RK_UTF8,
    uint64_t queue_capacity_bytes, rk_recording_compression compression,
    rk_recording_writer_handle *out_writer RK_OUT RK_OWNED);
RK_API rk_result RK_CALL rk_recording_writer_register_channel(rk_recording_writer_handle writer,
    const char *topic RK_UTF8, const char *schema_name RK_UTF8,
    const char *schema_encoding RK_UTF8, const uint8_t *schema_data RK_IN_ARRAY(schema_data_len),
    uint32_t schema_data_len, const char *message_encoding RK_UTF8,
    uint32_t *out_channel_id RK_OUT);
RK_API rk_result RK_CALL rk_recording_writer_enqueue(rk_recording_writer_handle writer,
    uint32_t channel_id, uint64_t ordinal,
    uint64_t recording_timestamp_ns,
    const uint8_t *payload RK_IN_ARRAY(payload_size), uint32_t payload_size);
RK_API rk_result RK_CALL rk_recording_writer_get_status(rk_recording_writer_handle writer,
    rk_recording_writer_status *status);
RK_API rk_result RK_CALL rk_recording_writer_finish(rk_recording_writer_handle writer);
RK_API void RK_CALL rk_recording_writer_destroy(rk_recording_writer_handle writer);

RK_API rk_result RK_CALL rk_recording_reader_open(const char *path RK_UTF8,
    rk_recording_reader_handle *out_reader RK_OUT RK_OWNED);
/** Returns RK_ERROR_STALE_STATE at clean end of file. */
RK_API rk_result RK_CALL rk_recording_reader_next(rk_recording_reader_handle reader,
    rk_recording_message *message, uint8_t *payload RK_OUT_BUFFER(inout_payload_size),
    uint32_t *inout_payload_size RK_INOUT);
RK_API void RK_CALL rk_recording_reader_destroy(rk_recording_reader_handle reader);

/* ------------------------------------------------------------------------- */
/* Identifiers and states                                                    */
/* ------------------------------------------------------------------------- */

/** Index of a joint in a compiled RobotRuntime layout. */
typedef uint32_t rk_joint_id;
/** Index of a link in a compiled RobotRuntime layout. */
typedef uint32_t rk_link_id;
/** Index of a reference frame in a compiled RobotRuntime layout. */
typedef uint32_t rk_frame_id;

/** High-level operating mode published by a robot runtime. */
typedef uint32_t rk_robot_mode;
enum {
    RK_ROBOT_MODE_IDLE = 0, /**< No active target or motion request. */
    RK_ROBOT_MODE_TRACKING = 1, /**< Following one or more accepted targets. */
    RK_ROBOT_MODE_STOPPING = 2, /**< Performing a normal stop. */
    RK_ROBOT_MODE_FAULT = 3 /**< Endpoint or command processing fault. */
};

/** Safety condition governing whether commands may be applied. */
typedef uint32_t rk_safety_state;
enum {
    RK_SAFETY_READY = 0, /**< Commands may be accepted. */
    RK_SAFETY_STOPPING = 1, /**< Normal stop is in progress. */
    RK_SAFETY_EMERGENCY_STOP = 2, /**< Emergency stop latched by the endpoint. */
    RK_SAFETY_FAULT = 3 /**< Fault requires endpoint/application handling. */
};

/** Availability of the backend represented by a published snapshot. */
typedef uint32_t rk_endpoint_state;
enum {
    RK_ENDPOINT_DISCONNECTED = 0, /**< No backend connection is available. */
    RK_ENDPOINT_CONNECTED = 1, /**< Backend is available for commands and state. */
    RK_ENDPOINT_FAULT = 2 /**< Backend reported a persistent fault. */
};

/** Kind of command batch submitted to a runtime mailbox. */
typedef uint32_t rk_command_kind;
enum {
    RK_COMMAND_NONE = 0, /**< No-op batch, useful for an explicit heartbeat. */
    RK_COMMAND_JOINT_TARGETS = 1, /**< Apply the included joint target array. */
    RK_COMMAND_STOP = 2, /**< Request a normal stop and clear active targets. */
    RK_COMMAND_EMERGENCY_STOP = 3, /**< Request an emergency stop. */
    RK_COMMAND_RESET_SAFETY = 4, /**< Clear a latched stop after application acknowledgement. */
    RK_COMMAND_TRAJECTORY_SEGMENTS = 6, /**< Append polynomial trajectory segments. */
    RK_COMMAND_HOLD = 7, /**< Decelerate the path clock to zero, retaining the queue. */
    RK_COMMAND_RESUME = 8, /**< Accelerate a held path clock back to normal rate. */
    RK_COMMAND_ABORT = 9 /**< Straight-line controlled stop, clearing the queue. */
};

/** Control interpretation of one joint target value. */
typedef uint32_t rk_joint_target_mode;
enum {
    RK_TARGET_POSITION = 1, /**< Target joint position. */
    RK_TARGET_VELOCITY = 2, /**< Target joint velocity. */
    RK_TARGET_EFFORT = 3, /**< Target joint effort or torque. */
    /**
     * Joint servo: target is a position, and the command's parallel servos
     * entry adds a velocity target, stiffness, damping and feedforward effort.
     * The endpoint applies effort = stiffness * (position error) +
     * damping * (velocity error) + feedforward every physics step, clamped to
     * the joint's effort limit (or max_effort when smaller).
     */
    RK_TARGET_SERVO = 4
};

/** Joint topology type understood by the runtime compiler/backend. */
typedef uint32_t rk_robot_runtime_joint_type;
enum {
    RK_RUNTIME_JOINT_FIXED = 1, /**< Fixed relationship between two links. */
    RK_RUNTIME_JOINT_REVOLUTE = 2, /**< Rotational degree of freedom. */
    RK_RUNTIME_JOINT_PRISMATIC = 3 /**< Translational degree of freedom. */
};

/* ------------------------------------------------------------------------- */
/* RobotRuntime descriptions                                                 */
/* ------------------------------------------------------------------------- */

/** One compiled joint and its limits in a RobotRuntimeBlueprint. */
typedef struct rk_robot_runtime_joint {
    rk_joint_id joint; /**< Stable index of this joint within the blueprint. */
    rk_robot_runtime_joint_type type; /**< Joint kinematic type. */
    rk_link_id parent_link; /**< Link on the parent side of the joint. */
    rk_link_id child_link; /**< Link on the child side of the joint. */
    double lower_limit; /**< Inclusive lower position limit, in SI units. */
    double upper_limit; /**< Inclusive upper position limit, in SI units. */
    double max_effort; /**< Maximum supported effort, in SI units. */
    double parent_frame_position[3];
    double parent_frame_rotation[4]; /**< Unit quaternion xyzw. */
    double child_frame_position[3];
    double child_frame_rotation[4]; /**< Unit quaternion xyzw. */
    double axis[3]; /**< Unit vector in the parent joint frame. */
    double max_acceleration; /**< Maximum joint acceleration, or zero when unspecified. */
    double max_velocity; /**< Maximum joint speed, or zero when unspecified. */
} rk_robot_runtime_joint;

enum { RK_COLLISION_APPROXIMATION_NONE = 0, RK_COLLISION_APPROXIMATION_BOUNDS_BOX = 1 };
/** Self-collision policy for one compiled RobotRuntime blueprint. */
enum {
    RK_SELF_COLLISION_DEFAULT = 0, /**< Preserve the enabled default. */
    RK_SELF_COLLISION_ENABLED = 1,
    RK_SELF_COLLISION_DISABLED = 2
};
/** Passive joint dynamics; zero is none. */
typedef struct rk_robot_joint_dynamics {
    double armature; /**< Reflected rotor inertia: kg m^2, or kg for a prismatic joint. */
    double damping; /**< Viscous effort per unit joint velocity. */
    double friction_loss; /**< Dry friction effort. */
    /** Limit softness (see nksim_joint_desc); zeros keep the backend default. */
    double limit_time_constant;
    double limit_damping_ratio;
    double limit_impedance[5];
} rk_robot_joint_dynamics;

typedef struct rk_robot_runtime_link {
    double mass;
    double center_of_mass[3];
    double inertia_tensor[9]; /**< Row-major, symmetric kg m² tensor. */
} rk_robot_runtime_link;

enum { RK_SENSOR_ENCODER = 1, RK_SENSOR_IMU = 2, RK_SENSOR_LIDAR = 3 };

/** Compiled sensor slot. Semantic IDs live in the immutable host mapping. */
typedef struct rk_sensor_config {
    uint32_t kind;
    rk_link_id link;
    double position[3]; /**< link_T_sensor translation in meters. */
    double rotation[4]; /**< link_T_sensor unit quaternion xyzw. */
    double update_rate; /**< Hz; zero means every shared tick. */
    uint32_t ray_count; /**< LiDAR resolution, 1..RK_MAX_SENSOR_VALUES. */
    uint32_t noise_seed; /**< Deterministic per-sensor PRNG seed. */
    double max_range; /**< LiDAR maximum range in meters. */
    double noise_stddev; /**< Independent Gaussian noise, SI units. Zero disables it. */
    double start_angle; /**< Bearing of the first LiDAR ray in the sensor frame. */
    double field_of_view; /**< LiDAR angular coverage in radians; zero selects 2*pi. */
} rk_sensor_config;

/** Latest acquisition for one compiled sensor slot; zero sequence means absent. */
typedef struct rk_sensor_sample {
    uint64_t sequence;
    uint64_t source_timestamp_ns;
    uint64_t received_timestamp_ns;
    uint32_t value_count;
    double values[RK_MAX_SENSOR_VALUES];
} rk_sensor_sample;

typedef uint32_t rk_event_kind;
enum { RK_EVENT_DIGITAL = 1, RK_EVENT_ANALOG = 2, RK_EVENT_PROCESS = 3 };
typedef uint32_t rk_event_hold_policy;
enum { RK_EVENT_KEEP = 0, RK_EVENT_SAFE_WHILE_HELD = 1,
    RK_EVENT_RESTORE_ON_RESUME = 2 };

/** One typed output value; digital is 0 or 1, analog and process arguments use SI units. */
typedef struct rk_event_value {
    rk_event_kind kind;
    uint32_t digital;
    double analog;
    char command[RK_PROCESS_COMMAND_BYTES];
    double argument;
} rk_event_value;

typedef struct rk_channel_declaration {
    char id[RK_PROCESS_CHANNEL_ID_BYTES];
    rk_event_kind kind;
    rk_event_value safe_value;
} rk_channel_declaration;

typedef struct rk_timed_event {
    uint64_t time_ns; /**< Plan-relative trajectory time. */
    char channel[RK_PROCESS_CHANNEL_ID_BYTES];
    rk_event_value value;
    rk_event_hold_policy hold_policy;
} rk_timed_event;

typedef uint32_t rk_event_cause;
enum { RK_EVENT_SCHEDULED = 1, RK_EVENT_HOLD_SAFE = 2,
    RK_EVENT_RESUME_RESTORE = 3, RK_EVENT_STOP_SAFE = 4 };
typedef struct rk_event_record {
    uint64_t plan_id;
    uint64_t scheduled_time_ns;
    uint64_t applied_owner_time_ns;
    char channel[RK_PROCESS_CHANNEL_ID_BYTES];
    rk_event_value value;
    rk_event_cause cause;
} rk_event_record;
typedef struct rk_event_record_batch {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t count;
    uint32_t overflow;
    rk_event_record records[RK_MAX_EVENT_RECORDS];
} rk_event_record_batch;

/**
 * Bulk compiled robot description consumed when a RobotRuntime is created.
 *
 * Keeping this value separate from commands makes topology immutable while
 * realtime code is running and lets Simulation build several runtimes before
 * its first shared tick.
 */
typedef struct rk_robot_joint_coupling {
    rk_joint_id leader;
    rk_joint_id follower;
    double ratio;
    double offset;
} rk_robot_joint_coupling;

typedef struct rk_robot_runtime_blueprint {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t revision; /**< Compiled model revision. */
    uint32_t joint_count; /**< Number of valid entries in joints. */
    uint32_t link_count; /**< Number of links referenced by the joints. */
    uint32_t frame_count; /**< Number of compiled reference frames. */
    uint32_t collision_approximation;
    uint32_t self_collision; /**< RK_SELF_COLLISION_*; zero preserves enabled-by-default. */
    /**
     * Nonzero makes the root link a free six-DOF body in Simulation instead of
     * a kinematic base. Runtime joints stay one-DOF; the root's pose and twist
     * are read through rk_simulation_get_robot_pose() and
     * rk_simulation_get_robot_base_velocity(). Must be 0 or 1.
     */
    uint32_t floating_base;
    uint64_t reserved[1];
    rk_robot_runtime_joint joints[RK_MAX_JOINTS];
    rk_robot_runtime_link links[RK_MAX_LINKS];
    uint32_t sensor_count; /**< Zero selects the default base-mounted simulation sensors. */
    rk_sensor_config sensors[RK_MAX_SENSORS];
    uint64_t calibration_revision; /**< Optional compiled calibration identity; zero is unspecified. */
    uint64_t commit_lead_ns; /**< Zero selects two owner periods. */
    double following_error_bound[RK_MAX_TRAJECTORY_JOINTS]; /**< Per-joint SI-unit bound; zero disables this check. */
    uint64_t owner_period_ns; /**< Zero selects the 10 ms default; versioned by struct_size. */
    uint64_t serial_processing_allowance_ns; /**< Zero selects 2 ms; one-way target-streaming budget. */
    uint32_t channel_count; /**< Versioned: absent means no process channels. */
    rk_channel_declaration channels[RK_MAX_PROCESS_CHANNELS];
    uint32_t coupling_count; /**< Joint relations compiled from RobotModel v5. */
    rk_robot_joint_coupling couplings[RK_MAX_JOINT_COUPLINGS];
    /** Versioned: absent means no passive dynamics. Indexed by joint. */
    rk_robot_joint_dynamics joint_dynamics[RK_MAX_JOINTS];
    /**
     * Versioned: how far, in joint units, an observed position may pass a
     * joint limit before the runtime faults. Zero keeps limits exact. A robot
     * that rests on its mechanical stops, such as a legged robot on its knee
     * stops, needs room for a simulator's or joint's compliant stops.
     * Commanded positions must still lie within the limits.
     */
    double observed_limit_tolerance;
} rk_robot_runtime_blueprint;

/* ------------------------------------------------------------------------- */
/* Commands and observations                                                 */
/* ------------------------------------------------------------------------- */

/** One requested joint target inside a RobotCommand batch. */
typedef struct rk_joint_target {
    rk_joint_id joint; /**< Target joint index. */
    rk_joint_target_mode mode; /**< Position, velocity, or effort interpretation. */
    double target; /**< Desired value in the mode's SI units. */
    double max_rate; /**< Optional rate limit; zero means backend default. */
    double max_effort; /**< Optional effort limit; zero means backend default. */
} rk_joint_target;

/** Servo control covers joints 0 .. RK_MAX_SERVO_JOINTS - 1: enough for a humanoid with hands. */
enum { RK_MAX_SERVO_JOINTS = 64 };

/** Servo terms for one RK_TARGET_SERVO target, in the target's SI units. */
typedef struct rk_joint_servo {
    double velocity; /**< Velocity target. */
    double stiffness; /**< Effort per unit position error; non-negative. */
    double damping; /**< Effort per unit velocity error; non-negative. */
    double feedforward; /**< Effort added to the feedback terms. */
} rk_joint_servo;

/** Coefficients for one joint, in powers of seconds from segment start. */
typedef struct rk_trajectory_coefficients {
    double value[6]; /**< Degree zero through five. */
} rk_trajectory_coefficients;

/** A polynomial segment whose start time is relative to its chunk. */
typedef struct rk_trajectory_segment {
    uint64_t time_from_start_ns;
    uint64_t duration_ns;
    uint32_t degree;
    uint32_t joint_count;
    rk_trajectory_coefficients coefficients[RK_MAX_TRAJECTORY_JOINTS];
} rk_trajectory_segment;

/** Bounded polynomial payload; segment starts are contiguous from time zero. */
typedef struct rk_trajectory_segment_chunk {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t segment_count;
    rk_trajectory_segment segments[RK_MAX_TRAJECTORY_SEGMENTS];
    uint64_t tag;
} rk_trajectory_segment_chunk;

/** Capabilities required by a plan; unknown or reserved bits are unsupported. */
enum {
    RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE = 1u,
    /** Path-time process events; see motionkit/plans/CONTRACTS.md C1. */
    RK_PLAN_CAPABILITY_EVENTS = 2u
};

/** Validation property carried from MotionKit's plan report. */
enum { RK_PLAN_JERK_UNCHECKED = 1u };

/** Bounded plan. Replacement time is in the active plan's trajectory clock. */
typedef struct rk_plan_submission {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t sequence; /**< Monotonic runtime command sequence. */
    uint64_t plan_id; /**< Nonzero caller-selected identity. */
    uint64_t model_revision;
    uint64_t calibration_revision;
    uint32_t required_capabilities;
    uint32_t reserved0; /**< RK_PLAN_JERK_UNCHECKED when jerk validation is unchecked. */
    uint64_t replace_after_plan_id; /**< Zero means append/start, not replace. */
    uint64_t replace_after_time_ns;
    double start_position[RK_MAX_TRAJECTORY_JOINTS];
    double start_velocity[RK_MAX_TRAJECTORY_JOINTS];
    double start_acceleration[RK_MAX_TRAJECTORY_JOINTS];
    rk_trajectory_segment_chunk segments;
    /** Per-joint nonnegative start-state tolerances; zero defaults to 1e-6. */
    double position_tolerance[RK_MAX_TRAJECTORY_JOINTS];
    double velocity_tolerance[RK_MAX_TRAJECTORY_JOINTS];
    double acceleration_tolerance[RK_MAX_TRAJECTORY_JOINTS];
    /** One for a final plan, zero if more motion is expected. Absent means one.
     * Full-size C callers must set this explicitly; Haxe defaults to final.
     */
    uint32_t ends_at_rest;
    uint32_t event_count; /**< Versioned: absent means no events. */
    rk_timed_event events[RK_MAX_PLAN_EVENTS];
} rk_plan_submission;

/** Non-latched runtime diagnostic; safety remains READY. */
enum { RK_FAULT_TRAJECTORY_UNDERFLOW = 2, RK_FAULT_RAMP_LIMIT = 3,
    RK_FAULT_CLOCK_SYNC_LOST = 4, RK_FAULT_DUAL_DRIVE_SKEW = 5,
    RK_FAULT_QUEUE_REVISION_MISMATCH = 6 };

typedef uint32_t rk_session_state;
enum {
    RK_SESSION_IDLE = 0,
    RK_SESSION_EXECUTING = 1,
    RK_SESSION_HOLDING = 2, /**< Transitioning toward a path-preserving hold. */
    RK_SESSION_HELD = 3, /**< Path clock paused with queued motion retained. */
    RK_SESSION_STOPPING = 4,
    RK_SESSION_FAULTED = 5
};

/** Command metadata and optional fixed-capacity joint-target payload. */
typedef struct rk_robot_command {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t sequence; /**< Monotonic command sequence chosen by the caller. */
    uint64_t timestamp_ns; /**< Optional issuer/source timestamp metadata, NOT a deadline. */
    rk_command_kind kind; /**< Operation represented by this batch. */
    uint32_t target_count; /**< Number of valid entries in targets. */
    rk_joint_target targets[RK_MAX_JOINTS]; /**< Fixed-capacity target payload. */
    /**
     * Parallel to the first RK_MAX_SERVO_JOINTS targets; read only for
     * RK_TARGET_SERVO entries, which are limited to joints below
     * RK_MAX_SERVO_JOINTS so the command stays small to copy.
     */
    rk_joint_servo servos[RK_MAX_SERVO_JOINTS];
} rk_robot_command;

/** Mutable native state used internally while a runtime publishes a snapshot. */
typedef struct rk_robot_state {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t sequence; /**< Monotonic published state sequence. */
    uint64_t source_timestamp_ns; /**< Timestamp generated by the robot/backend clock. */
    rk_robot_mode mode; /**< Current operating mode. */
    rk_safety_state safety; /**< Current safety state. */
    uint32_t joint_count; /**< Number of valid entries in each state array. */
    uint32_t reserved0;
    double position[RK_MAX_JOINTS];
    double velocity[RK_MAX_JOINTS];
    double effort[RK_MAX_JOINTS];
    uint64_t received_timestamp_ns; /**< Monotonic timestamp when Runtime accepted the sample. */
    uint32_t trajectory_queue_depth; /**< Queued knots not yet passed, including the active start knot. */
    uint32_t trajectory_active; /**< Non-zero while a timestamped trajectory is executing. */
    uint64_t trajectory_time_ns; /**< Runtime owner-clock position within the active trajectory. */
    uint64_t trajectory_duration_ns; /**< Timestamp of the active trajectory's final endpoint. */
    uint32_t sensor_count;
    rk_sensor_sample sensors[RK_MAX_SENSORS];
    uint64_t trajectory_tag; /**< Chunk identity currently running or last stopped. */
    uint64_t trajectory_tag_time_ns; /**< Time within trajectory_tag, in nanoseconds. */
    rk_session_state session_state;
    uint64_t active_plan_id;
    uint64_t committed_until_ns;
    uint64_t queue_end_time_ns;
} rk_robot_state;

/** Immutable published runtime snapshot; native handles never enter this ABI. */
typedef struct rk_robot_snapshot {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t revision; /**< Runtime blueprint revision. */
    uint64_t sequence; /**< Monotonic published state sequence. */
    uint64_t source_timestamp_ns; /**< Robot/backend timestamp in nanoseconds. */
    rk_robot_mode mode; /**< Current operating mode. */
    rk_safety_state safety; /**< Current safety state. */
    rk_endpoint_state endpoint; /**< Backend connection state. */
    uint32_t joint_count; /**< Number of valid entries in state arrays. */
    double position[RK_MAX_JOINTS]; /**< Joint positions in SI units. */
    double velocity[RK_MAX_JOINTS]; /**< Joint velocities in SI units. */
    double effort[RK_MAX_JOINTS]; /**< Joint efforts in SI units. */
    double base_position[3]; /**< Base translation, when provided by the endpoint. */
    double base_rotation[4]; /**< Base orientation quaternion, when provided. */
    double base_linear_velocity[3]; /**< Base linear velocity, when provided. */
    double base_angular_velocity[3]; /**< Base angular velocity, when provided. */
    int32_t fault_code; /**< Endpoint-specific diagnostic code, or zero. */
    uint32_t reserved0;
    uint64_t reserved[2];
    uint64_t received_timestamp_ns; /**< Runtime receive timestamp in nanoseconds. */
    uint32_t trajectory_queue_depth; /**< Queued knots not yet passed, including the active start knot. */
    uint32_t trajectory_active; /**< Non-zero while a timestamped trajectory is executing. */
    uint64_t trajectory_time_ns; /**< Runtime owner-clock position within the active trajectory. */
    uint64_t trajectory_duration_ns; /**< Timestamp of the active trajectory's final endpoint. */
    uint32_t sensor_count;
    rk_sensor_sample sensors[RK_MAX_SENSORS];
    uint64_t trajectory_tag; /**< Chunk identity currently running or last stopped. */
    uint64_t trajectory_tag_time_ns; /**< Time within trajectory_tag, in nanoseconds. */
    uint64_t calibration_revision; /**< Blueprint calibration identity; zero is unspecified. */
    rk_session_state session_state;
    uint64_t active_plan_id;
    uint64_t committed_until_ns;
    uint64_t queue_end_time_ns;
} rk_robot_snapshot;

/** Static control capabilities reported by a RobotRuntime endpoint. */
typedef struct rk_robot_capabilities {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint32_t joint_count; /**< Number of controllable joints. */
    uint32_t supports_position_targets; /**< Non-zero when position targets work. */
    uint32_t supports_velocity_targets; /**< Non-zero when velocity targets work. */
    uint32_t supports_effort_targets; /**< Non-zero when effort targets work. */
    uint32_t supports_prediction; /**< Non-zero when predictive state is available. */
    uint32_t supports_trajectory_queue; /**< Non-zero when timestamped trajectory chunks work. */
    uint32_t supports_execution_plans; /**< Non-zero when revision-bound plans work. */
    uint32_t reserved[1];
} rk_robot_capabilities;

/* ------------------------------------------------------------------------- */
/* Value validation                                                          */
/* ------------------------------------------------------------------------- */

/** Validates compiled topology, joint indices, limits, and struct size. */
RK_API rk_result RK_CALL rk_robot_runtime_blueprint_validate(
    const rk_robot_runtime_blueprint *blueprint);
/** Validates command kind, target count, indices, and target modes. */
RK_API rk_result RK_CALL rk_robot_command_validate(const rk_robot_command *command);
/** Validates a command against the joint count in a compiled blueprint. */
RK_API rk_result RK_CALL rk_robot_command_validate_for_blueprint(
    const rk_robot_command *command, const rk_robot_runtime_blueprint *blueprint);
/** Validates a bounded polynomial payload and its coefficient budget. */
RK_API rk_result RK_CALL rk_trajectory_segment_chunk_validate(
    const rk_trajectory_segment_chunk *chunk);
RK_API rk_result RK_CALL rk_trajectory_segment_chunk_validate_for_blueprint(
    const rk_trajectory_segment_chunk *chunk, const rk_robot_runtime_blueprint *blueprint);
RK_API rk_result RK_CALL rk_plan_submission_validate_for_blueprint(
    const rk_plan_submission *plan, const rk_robot_runtime_blueprint *blueprint);
/** Validates a mutable native state value and its array counts. */
RK_API rk_result RK_CALL rk_robot_state_validate(const rk_robot_state *state);
/** Validates an immutable published snapshot and its array counts. */
RK_API rk_result RK_CALL rk_robot_snapshot_validate(const rk_robot_snapshot *snapshot);
/** Validates capability flags and the advertised joint count. */
RK_API rk_result RK_CALL rk_robot_capabilities_validate(
    const rk_robot_capabilities *capabilities);

/* ------------------------------------------------------------------------- */
/* RobotRuntime lifecycle and data flow                                      */
/* ------------------------------------------------------------------------- */

/** Opaque handle for one RobotRuntime command/state boundary. */
typedef uint32_t rk_robot_runtime RK_HANDLE RK_HANDLE_DESTROY(rk_robot_runtime_destroy);

#define RK_INVALID_ROBOT_RUNTIME ((rk_robot_runtime)0)

/**
 * Creates a standalone in-memory RobotRuntime.
 *
 * The returned runtime owns its endpoint worker lifecycle. For shared physics,
 * use rk_simulation_add_robot instead so Simulation remains the one clock
 * owner. Standalone callers advance through start/stop; there is no public
 * per-runtime tick operation.
 *
 * This convenience constructor creates the loopback endpoint used for
 * standalone host bring-up. Shared simulation runtimes must instead be
 * created with rk_simulation_add_robot().
 *
 * @param blueprint Compiled execution topology for the standalone runtime.
 * @param out_runtime Receives an owned runtime handle.
 * @return RK_OK on success, or an argument/allocation error.
 */
RK_API rk_result RK_CALL rk_robot_runtime_create(const rk_robot_runtime_blueprint *blueprint,
                                           rk_robot_runtime *out_runtime RK_OUT RK_OWNED);
/** Creates a serial runtime; fingerprint is 32 hex digits and error is in target SI units. */
RK_API rk_result RK_CALL rk_robot_runtime_create_serial(
    const rk_robot_runtime_blueprint *blueprint, const char *device_path RK_UTF8,
    uint32_t baud, const char *fingerprint_hex RK_UTF8,
    double max_target_error, rk_robot_runtime *out_runtime RK_OUT RK_OWNED);
/** RKD6 serial constructor with deployment timing and capability inputs. */
RK_API rk_result RK_CALL rk_robot_runtime_create_serial6(
    const rk_robot_runtime_blueprint *blueprint, const char *device_path RK_UTF8,
    uint32_t baud, const char *fingerprint_hex RK_UTF8,
    double max_target_error, uint32_t step_tick_hz,
    uint64_t link_loss_timeout_ns, uint64_t clock_bound_ns,
    uint64_t link_latency_ns, rk_robot_runtime *out_runtime RK_OUT RK_OWNED);
/**
 * Stops and releases a standalone runtime handle.
 *
 * Passing an invalid handle is harmless. Handles created by Simulation are
 * normally released by rk_simulation_destroy().
 */
RK_API void RK_CALL rk_robot_runtime_destroy(rk_robot_runtime runtime);
/**
 * Starts a standalone runtime's owner worker.
 *
 * A runtime attached to a shared Simulation is externally driven and returns
 * RK_ERROR_INVALID_STATE from this function.
 */
RK_API rk_result RK_CALL rk_robot_runtime_start(rk_robot_runtime runtime);
/** Stops a standalone runtime's owner worker, if it is running. */
RK_API rk_result RK_CALL rk_robot_runtime_stop(rk_robot_runtime runtime);

/**
 * Submits one complete command batch to the runtime mailbox.
 *
 * Submission only queues the value. The endpoint applies it during its next
 * owner-thread phase; in a shared Simulation that phase is part of the next
 * rk_simulation_step() or realtime simulation tick.
 *
 * A joint-target batch is a partial update: joints it omits keep their current
 * targets. When several batches arrive before one owner phase, an emergency
 * stop among them wins; a newest stop or safety reset wins alone; otherwise
 * the joint-target batches since the latest stop or reset merge per joint, the
 * newer batch winning for a joint both set. So a drive and an arm may be
 * commanded with separate batches in the same cycle.
 *
 * @param runtime Runtime receiving the command.
 * @param command Complete command value to copy into the mailbox.
 * @return RK_OK when queued, or a validation/state/queue error.
 */
RK_API rk_result RK_CALL rk_robot_runtime_submit(rk_robot_runtime runtime,
                                           const rk_robot_command *command);

/** Submits a bounded polynomial segment chunk to the runtime queue. */
RK_API rk_result RK_CALL rk_robot_runtime_submit_segments(
    rk_robot_runtime runtime, const rk_robot_command *command,
    const rk_trajectory_segment_chunk *chunk);

/**
 * Atomically validates and accepts a plan or committed-horizon replacement.
 * A replacement before committed_until_ns returns RK_ERROR_INVALID_STATE with
 * no queue mutation.
 */
RK_API rk_result RK_CALL rk_robot_runtime_submit_plan(
    rk_robot_runtime runtime, const rk_plan_submission *plan);
/** Drains process-output changes. An overflow flag means earlier records were lost. */
RK_API rk_result RK_CALL rk_robot_runtime_poll_events(
    rk_robot_runtime runtime, rk_event_record_batch *out_batch);

/**
 * Copies the latest state into the caller-provided value.
 *
 * This is a read operation and never advances a runtime or simulation clock.
 * The caller must initialize out_state->struct_size before calling.
 */
RK_API rk_result RK_CALL rk_robot_runtime_snapshot(rk_robot_runtime runtime,
                                             rk_robot_state *out_state RK_INOUT);
/**
 * Copies the latest published snapshot, including revision, endpoint, and
 * fault metadata. This operation never advances time.
 */
RK_API rk_result RK_CALL rk_robot_runtime_snapshot_full(
    rk_robot_runtime runtime, rk_robot_snapshot *out_snapshot RK_INOUT);
/**
 * Copies the endpoint's static control capabilities.
 *
 * Capabilities describe what the endpoint can accept; they do not imply that
 * a command is safe or valid for the current lifecycle state.
 */
RK_API rk_result RK_CALL rk_robot_runtime_capabilities(
    rk_robot_runtime runtime, rk_robot_capabilities *out_capabilities RK_INOUT);

#ifdef __cplusplus
}
#endif

#endif
