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
#define RK_STRUCT_SIZE __attribute__((annotate("hxi:struct_size")))
#else
#define RK_OUT
#define RK_INOUT
#define RK_HANDLE
#define RK_HANDLE_DESTROY(symbol)
#define RK_OWNED
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
    RK_MAX_JOINTS = 64, /**< Maximum joints carried by one fixed-size ABI value. */
    RK_API_VERSION = 1 /**< Version of the RobotKit C data contract. */
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
    RK_ERROR_BACKEND = -8 /**< The underlying endpoint or physics backend failed. */
};

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
    RK_COMMAND_EMERGENCY_STOP = 3 /**< Request an emergency stop. */
};

/** Control interpretation of one joint target value. */
typedef uint32_t rk_joint_target_mode;
enum {
    RK_TARGET_POSITION = 1, /**< Target joint position. */
    RK_TARGET_VELOCITY = 2, /**< Target joint velocity. */
    RK_TARGET_EFFORT = 3 /**< Target joint effort or torque. */
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

/**
 * Native execution shape for one RobotRuntime.
 *
 * This is the compact ABI description used after a semantic Haxe robot model
 * has been compiled. It contains counts and revision metadata, not ownership
 * of any native resources.
 */
typedef struct rk_robot_runtime_layout {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t revision; /**< Compiled model revision represented by the layout. */
    uint32_t joint_count; /**< Number of valid joint entries/state values. */
    uint32_t link_count; /**< Number of links in the compiled topology. */
    uint32_t frame_count; /**< Number of reference frames, if supported. */
    uint32_t reserved0;
    uint64_t reserved[2];
} rk_robot_runtime_layout;

/** One compiled joint and its limits in a RobotRuntimeBlueprint. */
typedef struct rk_robot_runtime_joint {
    rk_joint_id joint; /**< Stable index of this joint within the blueprint. */
    rk_robot_runtime_joint_type type; /**< Joint kinematic type. */
    rk_link_id parent_link; /**< Link on the parent side of the joint. */
    rk_link_id child_link; /**< Link on the child side of the joint. */
    double lower_limit; /**< Inclusive lower position limit, in SI units. */
    double upper_limit; /**< Inclusive upper position limit, in SI units. */
    double max_effort; /**< Maximum supported effort, in SI units. */
} rk_robot_runtime_joint;

/**
 * Bulk compiled robot description consumed when a RobotRuntime is created.
 *
 * Keeping this value separate from commands makes topology immutable while
 * realtime code is running and lets Simulation build several runtimes before
 * its first shared tick.
 */
typedef struct rk_robot_runtime_blueprint {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t revision; /**< Compiled model revision. */
    uint32_t joint_count; /**< Number of valid entries in joints. */
    uint32_t link_count; /**< Number of links referenced by the joints. */
    uint32_t frame_count; /**< Number of compiled reference frames. */
    uint32_t reserved0;
    uint64_t reserved[2];
    rk_robot_runtime_joint joints[RK_MAX_JOINTS];
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

/** Complete command batch submitted atomically to one RobotRuntime mailbox. */
typedef struct rk_robot_command {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t sequence; /**< Monotonic command sequence chosen by the caller. */
    uint64_t timestamp_ns; /**< Optional command timestamp or deadline in nanoseconds. */
    rk_command_kind kind; /**< Operation represented by this batch. */
    uint32_t target_count; /**< Number of valid entries in targets. */
    rk_joint_target targets[RK_MAX_JOINTS]; /**< Fixed-capacity target payload. */
} rk_robot_command;

/** Mutable native state used internally while a runtime publishes a snapshot. */
typedef struct rk_robot_state {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t sequence; /**< Monotonic published state sequence. */
    uint64_t timestamp_ns; /**< Observation timestamp supplied by the owner. */
    rk_robot_mode mode; /**< Current operating mode. */
    rk_safety_state safety; /**< Current safety state. */
    uint32_t joint_count; /**< Number of valid entries in each state array. */
    uint32_t reserved0;
    double position[RK_MAX_JOINTS];
    double velocity[RK_MAX_JOINTS];
    double effort[RK_MAX_JOINTS];
} rk_robot_state;

/** Immutable published runtime snapshot; native handles never enter this ABI. */
typedef struct rk_robot_snapshot {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint64_t revision; /**< Runtime blueprint revision. */
    uint64_t sequence; /**< Monotonic published state sequence. */
    uint64_t timestamp_ns; /**< Observation timestamp in nanoseconds. */
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
} rk_robot_snapshot;

/** Static control capabilities reported by a RobotRuntime endpoint. */
typedef struct rk_robot_capabilities {
    uint32_t struct_size RK_STRUCT_SIZE; /**< Set to sizeof this struct. */
    uint32_t joint_count; /**< Number of controllable joints. */
    uint32_t supports_position_targets; /**< Non-zero when position targets work. */
    uint32_t supports_velocity_targets; /**< Non-zero when velocity targets work. */
    uint32_t supports_effort_targets; /**< Non-zero when effort targets work. */
    uint32_t supports_prediction; /**< Non-zero when predictive state is available. */
    uint32_t reserved[3];
} rk_robot_capabilities;

/* ------------------------------------------------------------------------- */
/* Value validation                                                          */
/* ------------------------------------------------------------------------- */

/** Validates layout size, counts, and reserved ABI fields. */
RK_API rk_result RK_CALL rk_robot_runtime_layout_validate(const rk_robot_runtime_layout *layout);
/** Validates compiled topology, joint indices, limits, and struct size. */
RK_API rk_result RK_CALL rk_robot_runtime_blueprint_validate(
    const rk_robot_runtime_blueprint *blueprint);
/** Validates command kind, target count, indices, and target modes. */
RK_API rk_result RK_CALL rk_robot_command_validate(const rk_robot_command *command);
/** Validates a command against the joint count in a runtime layout. */
RK_API rk_result RK_CALL rk_robot_command_validate_for_layout(
    const rk_robot_command *command, const rk_robot_runtime_layout *layout);
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
 * @param layout Compiled execution shape for the standalone runtime.
 * @param out_runtime Receives an owned runtime handle.
 * @return RK_OK on success, or an argument/allocation error.
 */
RK_API rk_result RK_CALL rk_robot_runtime_create(const rk_robot_runtime_layout *layout,
                                           rk_robot_runtime *out_runtime RK_OUT RK_OWNED);
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
 * @param runtime Runtime receiving the command.
 * @param command Complete command value to copy into the mailbox.
 * @return RK_OK when queued, or a validation/state/queue error.
 */
RK_API rk_result RK_CALL rk_robot_runtime_submit(rk_robot_runtime runtime,
                                           const rk_robot_command *command);

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
