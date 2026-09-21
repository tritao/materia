#ifndef ROBOTKIT_RUNTIME_H
#define ROBOTKIT_RUNTIME_H

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

enum {
    RK_MAX_JOINTS = 64,
    RK_API_VERSION = 1
};

typedef int32_t rk_result;
enum {
    RK_OK = 0,
    RK_ERROR_INVALID_ARGUMENT = -1,
    RK_ERROR_INVALID_STATE = -2,
    RK_ERROR_QUEUE_FULL = -3,
    RK_ERROR_UNSUPPORTED = -4,
    RK_ERROR_SAFETY_STOPPED = -5,
    RK_ERROR_INVALID_HANDLE = -6,
    RK_ERROR_OUT_OF_MEMORY = -7,
    RK_ERROR_BACKEND = -8
};

/* ------------------------------------------------------------------------- */
/* Identifiers and states                                                    */
/* ------------------------------------------------------------------------- */

typedef uint32_t rk_joint_id;
typedef uint32_t rk_link_id;
typedef uint32_t rk_frame_id;

typedef uint32_t rk_robot_mode;
enum {
    RK_ROBOT_MODE_IDLE = 0,
    RK_ROBOT_MODE_TRACKING = 1,
    RK_ROBOT_MODE_STOPPING = 2,
    RK_ROBOT_MODE_FAULT = 3
};

typedef uint32_t rk_safety_state;
enum {
    RK_SAFETY_READY = 0,
    RK_SAFETY_STOPPING = 1,
    RK_SAFETY_EMERGENCY_STOP = 2,
    RK_SAFETY_FAULT = 3
};

typedef uint32_t rk_endpoint_state;
enum {
    RK_ENDPOINT_DISCONNECTED = 0,
    RK_ENDPOINT_CONNECTED = 1,
    RK_ENDPOINT_FAULT = 2
};

typedef uint32_t rk_command_kind;
enum {
    RK_COMMAND_NONE = 0,
    RK_COMMAND_JOINT_TARGETS = 1,
    RK_COMMAND_STOP = 2,
    RK_COMMAND_EMERGENCY_STOP = 3
};

typedef uint32_t rk_joint_target_mode;
enum {
    RK_TARGET_POSITION = 1,
    RK_TARGET_VELOCITY = 2,
    RK_TARGET_EFFORT = 3
};

typedef uint32_t rk_runtime_joint_type;
enum {
    RK_RUNTIME_JOINT_FIXED = 1,
    RK_RUNTIME_JOINT_REVOLUTE = 2,
    RK_RUNTIME_JOINT_PRISMATIC = 3
};

/* ------------------------------------------------------------------------- */
/* Runtime descriptions                                                      */
/* ------------------------------------------------------------------------- */

/** Compiled runtime shape; semantic Robot models live in robotkit/robotd. */
typedef struct rk_runtime_layout {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t revision;
    uint32_t joint_count;
    uint32_t link_count;
    uint32_t frame_count;
    uint32_t reserved0;
    uint64_t reserved[2];
} rk_runtime_layout;

typedef struct rk_runtime_joint {
    rk_joint_id joint;
    rk_runtime_joint_type type;
    rk_link_id parent_link;
    rk_link_id child_link;
    double lower_limit;
    double upper_limit;
    double max_effort;
} rk_runtime_joint;

/** Bulk compiled robot description consumed once when a runtime is created. */
typedef struct rk_runtime_blueprint {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t revision;
    uint32_t joint_count;
    uint32_t link_count;
    uint32_t frame_count;
    uint32_t reserved0;
    uint64_t reserved[2];
    rk_runtime_joint joints[RK_MAX_JOINTS];
} rk_runtime_blueprint;

/* ------------------------------------------------------------------------- */
/* Commands and observations                                                 */
/* ------------------------------------------------------------------------- */

typedef struct rk_joint_target {
    rk_joint_id joint;
    rk_joint_target_mode mode;
    double target;
    double max_rate;
    double max_effort;
} rk_joint_target;

typedef struct rk_robot_command {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t sequence;
    uint64_t timestamp_ns;
    rk_command_kind kind;
    uint32_t target_count;
    rk_joint_target targets[RK_MAX_JOINTS];
} rk_robot_command;

typedef struct rk_robot_state {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t sequence;
    uint64_t timestamp_ns;
    rk_robot_mode mode;
    rk_safety_state safety;
    uint32_t joint_count;
    uint32_t reserved0;
    double position[RK_MAX_JOINTS];
    double velocity[RK_MAX_JOINTS];
    double effort[RK_MAX_JOINTS];
} rk_robot_state;

/** Immutable published runtime snapshot; native handles never enter this ABI. */
typedef struct rk_robot_snapshot {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t revision;
    uint64_t sequence;
    uint64_t timestamp_ns;
    rk_robot_mode mode;
    rk_safety_state safety;
    rk_endpoint_state endpoint;
    uint32_t joint_count;
    double position[RK_MAX_JOINTS];
    double velocity[RK_MAX_JOINTS];
    double effort[RK_MAX_JOINTS];
    double base_position[3];
    double base_rotation[4];
    double base_linear_velocity[3];
    double base_angular_velocity[3];
    int32_t fault_code;
    uint32_t reserved0;
    uint64_t reserved[2];
} rk_robot_snapshot;

typedef struct rk_robot_capabilities {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t joint_count;
    uint32_t supports_position_targets;
    uint32_t supports_velocity_targets;
    uint32_t supports_effort_targets;
    uint32_t supports_prediction;
    uint32_t reserved[3];
} rk_robot_capabilities;

/* ------------------------------------------------------------------------- */
/* Value validation                                                          */
/* ------------------------------------------------------------------------- */

RK_API rk_result RK_CALL rk_runtime_layout_validate(const rk_runtime_layout *layout);
RK_API rk_result RK_CALL rk_runtime_blueprint_validate(
    const rk_runtime_blueprint *blueprint);
RK_API rk_result RK_CALL rk_robot_command_validate(const rk_robot_command *command);
RK_API rk_result RK_CALL rk_robot_command_validate_for_layout(
    const rk_robot_command *command, const rk_runtime_layout *layout);
RK_API rk_result RK_CALL rk_robot_state_validate(const rk_robot_state *state);
RK_API rk_result RK_CALL rk_robot_snapshot_validate(const rk_robot_snapshot *snapshot);
RK_API rk_result RK_CALL rk_robot_capabilities_validate(
    const rk_robot_capabilities *capabilities);

/* ------------------------------------------------------------------------- */
/* Runtime lifecycle and data flow                                           */
/* ------------------------------------------------------------------------- */

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
