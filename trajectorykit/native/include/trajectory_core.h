#ifndef TRAJECTORY_CORE_H
#define TRAJECTORY_CORE_H

#include <stddef.h>
#include <stdint.h>

#if defined(__clang__)
#define MK_OUT __attribute__((annotate("hxi:out")))
#define MK_OWNED __attribute__((annotate("hxi:owned")))
#define MK_HANDLE __attribute__((annotate("hxi:handle")))
#define MK_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#define MK_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#define MK_OUT_ARRAY(count) __attribute__((annotate("hxi:out_array")))
#define MK_RETURNS_SPAN(count) __attribute__((annotate("hxi:returns_span")))
#define MK_STRUCT_SIZE __attribute__((annotate("hxi:struct_size")))
#else
#define MK_OUT
#define MK_OWNED
#define MK_HANDLE
#define MK_HANDLE_DESTROY(symbol)
#define MK_IN_ARRAY(count)
#define MK_OUT_ARRAY(count)
#define MK_RETURNS_SPAN(count)
#define MK_STRUCT_SIZE
#endif

#if defined(_WIN32)
#if defined(TC_STATIC)
#define TC_API
#elif defined(TC_BUILDING_LIBRARY)
#define TC_API __declspec(dllexport)
#else
#define TC_API __declspec(dllimport)
#endif
#define MK_CALL __cdecl
#else
#define TC_API __attribute__((visibility("default")))
#define MK_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum { MK_API_VERSION = 14, MK_MAX_JOINTS = 64, MK_MAX_DEGREE = 5,
    MK_MAX_PLAN_EVENTS = 256, MK_EVENT_CHANNEL_BYTES = 48, MK_EVENT_COMMAND_BYTES = 48,
    MK_MAX_ASSUMPTIONS = 320, MK_ASSUMPTION_LENGTH = 96 };
typedef int32_t mk_result;
enum {
    MK_OK = 0,
    MK_ERROR_INVALID_ARGUMENT = -1,
    MK_ERROR_INVALID_HANDLE = -2,
    MK_ERROR_OUT_OF_MEMORY = -3,
    MK_ERROR_LIMIT = -4,
    MK_ERROR_UNSUPPORTED = -5,
    MK_ERROR_GENERATION = -6
};

/*
 * Threads: any function may be called from any thread. A trajectory, path, or time law is used by
 * one thread at a time; a plan is immutable and may be read from several threads at once, and
 * destroying it waits for no other thread's planning or validation.
 */

/** Opaque registry identity; only trajectory_core may interpret id. */
typedef struct mk_trajectory_handle { uint32_t id; } mk_trajectory_handle
    MK_HANDLE MK_HANDLE_DESTROY(mk_trajectory_destroy);
typedef struct mk_plan_handle { uint32_t id; } mk_plan_handle
    MK_HANDLE MK_HANDLE_DESTROY(mk_plan_destroy);

typedef struct mk_joint_coefficients {
    double value[MK_MAX_DEGREE + 1];
} mk_joint_coefficients;

/** Coefficients are joint-major, with seconds from t0 as the polynomial variable. */
typedef struct mk_segment {
    uint32_t struct_size MK_STRUCT_SIZE;
    int64_t t0_ns;
    int64_t duration_ns;
    uint32_t degree;
    uint32_t joint_count;
    mk_joint_coefficients coefficients[MK_MAX_JOINTS];
} mk_segment;

typedef struct mk_sample {
    uint32_t struct_size MK_STRUCT_SIZE;
    int64_t time_ns;
    uint32_t joint_count;
    double position[MK_MAX_JOINTS];
} mk_sample;

typedef struct mk_trajectory_state {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    double position[MK_MAX_JOINTS];
    double velocity[MK_MAX_JOINTS];
    double acceleration[MK_MAX_JOINTS];
    double jerk[MK_MAX_JOINTS];
} mk_trajectory_state;

/** Shared STOP/hold-lead derivative estimate for one queued path region. */
typedef struct mk_path_derivative_estimate {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    uint32_t has_forward_acceleration;
    uint32_t has_recent_acceleration;
    double velocity[MK_MAX_JOINTS];
    double acceleration[MK_MAX_JOINTS];
    double recent_acceleration[MK_MAX_JOINTS];
} mk_path_derivative_estimate;

/** Maximum absolute jump for each derivative across one segment boundary. */
typedef struct mk_continuity {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t boundary_index;
    int64_t time_ns;
    double c0_jump;
    double c1_jump;
    double c2_jump;
    uint32_t c0_joint;
    uint32_t c1_joint;
    uint32_t c2_joint;
} mk_continuity;

enum { MK_CHECK_POSITION = 0, MK_CHECK_VELOCITY = 1,
    MK_CHECK_ACCELERATION = 2, MK_CHECK_JERK = 3, MK_CHECK_CONTINUITY = 4,
    MK_CHECK_TASK_SPACE = 5, /**< Reserved for Cartesian and tool-path tolerance. */
    MK_CHECK_COUNT = 6 };
enum { MK_CHECK_UNCHECKED = 0, MK_CHECK_PASSED = 1, MK_CHECK_FAILED = 2 };
enum { MK_CHECK_METHOD_EXACT = 0, MK_CHECK_METHOD_SAMPLED = 1, MK_CHECK_METHOD_BOUND = 2 };

/** Zero motion limits are unclaimed. Position limits use an explicit flag so zero is usable. */
typedef struct mk_limits {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    uint64_t model_revision;
    uint64_t calibration_revision;
    uint32_t position_claimed[MK_MAX_JOINTS];
    double position_lower[MK_MAX_JOINTS];
    double position_upper[MK_MAX_JOINTS];
    double max_velocity[MK_MAX_JOINTS];
    double max_acceleration[MK_MAX_JOINTS];
    double max_jerk[MK_MAX_JOINTS];
    uint32_t derivative_claimed[MK_MAX_JOINTS]; /**< Bits 1, 2, 4 claim velocity, acceleration, jerk, including zero. */
    double max_continuity_jump[3]; /**< Optional C0, C1, C2 jump claims. */
    /** Zero defaults to 1 ns. Round non-integer device tick periods up (e.g. 5.88 ns to 6 ns). */
    uint64_t executor_time_resolution_ns;
} mk_limits;

typedef struct mk_validation_check {
    uint32_t status; /**< MK_CHECK_* status. */
    uint32_t joint; /**< UINT32_MAX when no limit is claimed. */
    uint32_t derivative_order; /**< 0 for position, 1..3 for derivatives. */
    uint32_t method; /**< MK_CHECK_METHOD_*; BOUND stores a continuous upper bound. */
    double value; /**< Signed position or absolute derivative/jump value. */
    double time_seconds; /**< Time from trajectory clock epoch, including t0; zero for a whole-path bound. */
    double limit; /**< Boundary or absolute maximum corresponding to value. */
    double margin; /**< Signed room to limit: negative means measured value exceeds it. */
    double tolerance; /**< Explicit permitted comparison excess, in check units. */
    uint64_t resolution_ns; /**< Sampling resolution; zero for exact checks. */
} mk_validation_check;

typedef struct mk_assumption {
    char text[MK_ASSUMPTION_LENGTH];
} mk_assumption;

typedef struct mk_validation_report {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t assumption_count;
    uint64_t model_revision;
    uint64_t calibration_revision;
    uint64_t trajectory_revision;
    mk_validation_check checks[MK_CHECK_COUNT];
    mk_assumption assumptions[MK_MAX_ASSUMPTIONS];
    uint64_t executor_time_resolution_ns; /**< Effective validation time resolution. */
} mk_validation_report;

typedef struct mk_start_state {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    double position[MK_MAX_JOINTS];
    double velocity[MK_MAX_JOINTS];
    double acceleration[MK_MAX_JOINTS];
    double position_tolerance[MK_MAX_JOINTS];
    double velocity_tolerance[MK_MAX_JOINTS];
    double acceleration_tolerance[MK_MAX_JOINTS];
} mk_start_state;

enum { MK_CAP_TIMED_TRAJECTORY = 1, MK_CAP_EVENTS = 2 };
enum { MK_AUTHORITY_MATERIA = 1, MK_AUTHORITY_BACKEND = 2 };

typedef uint32_t mk_event_kind;
enum { MK_EVENT_DIGITAL = 1, MK_EVENT_ANALOG = 2, MK_EVENT_PROCESS = 3 };
typedef uint32_t mk_event_hold_policy;
enum { MK_EVENT_KEEP = 0, MK_EVENT_SAFE_WHILE_HELD = 1,
    MK_EVENT_RESTORE_ON_RESUME = 2 };
typedef struct mk_event_value {
    mk_event_kind kind;
    uint32_t digital;
    double analog;
    char command[MK_EVENT_COMMAND_BYTES];
    double argument;
} mk_event_value;
typedef struct mk_timed_event {
    uint64_t time_ns;
    char channel[MK_EVENT_CHANNEL_BYTES];
    mk_event_value value;
    mk_event_hold_policy hold_policy;
} mk_timed_event;

typedef struct mk_plan_spec {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint64_t plan_id;
    uint64_t model_revision;
    uint64_t calibration_revision;
    uint64_t required_capabilities;
    uint32_t planning_authority;
    mk_start_state start_state;
    uint32_t event_count; /**< Versioned: zero when absent. */
    mk_timed_event events[MK_MAX_PLAN_EVENTS];
} mk_plan_spec;

typedef struct mk_plan_info {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    uint64_t plan_id;
    uint64_t model_revision;
    uint64_t calibration_revision;
    uint64_t trajectory_revision;
    uint64_t required_capabilities;
    uint32_t planning_authority;
    int64_t duration_ns;
    uint32_t event_count;
} mk_plan_info;

/*
 * A plan's segments as arrays the plan owns, read in place: valid until the plan is destroyed, and
 * empty (NULL, count zero) for an invalid handle. Segment i starts at starts[i] from the plan's
 * start, lasts durations[i], has degree degrees[i], and has joint j's coefficient of power p at
 * coefficients[(i * joint_count + j) * (MK_MAX_DEGREE + 1) + p], zero above its degree.
 */

TC_API mk_result MK_CALL mk_trajectory_create(uint32_t joint_count,
    mk_trajectory_handle *out_trajectory MK_OUT MK_OWNED);
TC_API void MK_CALL mk_trajectory_destroy(mk_trajectory_handle trajectory);
TC_API mk_result MK_CALL mk_trajectory_append_segment(mk_trajectory_handle trajectory,
    const mk_segment *segment);
TC_API mk_result MK_CALL mk_trajectory_evaluate(mk_trajectory_handle trajectory,
    int64_t time_ns, mk_trajectory_state *out_state);
/** Degree-1 uses chord differences; degree 0 and degree >= 2 use analytic derivatives. */
TC_API mk_result MK_CALL mk_trajectory_estimate_path_derivatives(
    mk_trajectory_handle trajectory, int64_t time_ns, uint64_t window_ns,
    mk_path_derivative_estimate *out_estimate);
TC_API mk_result MK_CALL mk_trajectory_duration_ns(mk_trajectory_handle trajectory,
    int64_t *out_duration_ns MK_OUT);
TC_API mk_result MK_CALL mk_trajectory_joint_count(mk_trajectory_handle trajectory,
    uint32_t *out_joint_count MK_OUT);
TC_API mk_result MK_CALL mk_trajectory_segment_count(mk_trajectory_handle trajectory,
    uint32_t *out_segment_count MK_OUT);
TC_API mk_result MK_CALL mk_trajectory_get_segment(mk_trajectory_handle trajectory,
    uint32_t index, mk_segment *out_segment);
TC_API mk_result MK_CALL mk_trajectory_boundary_continuity(mk_trajectory_handle trajectory,
    uint32_t boundary_index, mk_continuity *out_continuity);
/** Requires at least two strictly increasing samples. Positions are interpolated linearly. */
TC_API mk_result MK_CALL mk_trajectory_from_samples(uint32_t joint_count,
    const mk_sample *samples MK_IN_ARRAY(sample_count), uint32_t sample_count,
    mk_trajectory_handle *out_trajectory MK_OUT MK_OWNED);
/** Reports exact polynomial extrema; failed checks return MK_OK with failed status. */
TC_API mk_result MK_CALL mk_validate(mk_trajectory_handle trajectory,
    const mk_limits *limits, mk_validation_report *out_report);
/** Fills the planner-owned sampled task-space check in a validation report. */
TC_API mk_result MK_CALL mk_report_set_task_space(mk_validation_report *report,
    uint32_t status, double worst, double time_seconds, double tolerance,
    uint64_t resolution_ns);
/** Records a planner-proven whole-path task-error upper bound. No sampled
 * peak time/resolution is claimed. The planner owns the geometric proof. */
TC_API mk_result MK_CALL mk_report_set_task_space_bound(mk_validation_report *report,
    double upper_bound, double tolerance);
/** Deep-copies the trajectory and refuses any failed validation check. */
TC_API mk_result MK_CALL mk_plan_create(mk_trajectory_handle trajectory,
    const mk_plan_spec *spec, const mk_limits *limits,
    mk_plan_handle *out_plan MK_OUT MK_OWNED, mk_validation_report *out_report);
TC_API void MK_CALL mk_plan_destroy(mk_plan_handle plan);
TC_API mk_result MK_CALL mk_plan_get_info(mk_plan_handle plan, mk_plan_info *out_info);
TC_API size_t MK_CALL mk_plan_segment_array_count(mk_plan_handle plan);
TC_API const int64_t *MK_CALL mk_plan_segment_starts(mk_plan_handle plan)
    MK_RETURNS_SPAN(mk_plan_segment_array_count);
TC_API const int64_t *MK_CALL mk_plan_segment_durations(mk_plan_handle plan)
    MK_RETURNS_SPAN(mk_plan_segment_array_count);
TC_API const int32_t *MK_CALL mk_plan_segment_degrees(mk_plan_handle plan)
    MK_RETURNS_SPAN(mk_plan_segment_array_count);
TC_API size_t MK_CALL mk_plan_coefficient_array_count(mk_plan_handle plan);
TC_API const double *MK_CALL mk_plan_segment_coefficients(mk_plan_handle plan)
    MK_RETURNS_SPAN(mk_plan_coefficient_array_count);
TC_API mk_result MK_CALL mk_plan_get_event(mk_plan_handle plan, uint32_t index,
    mk_timed_event *out_event);
TC_API mk_result MK_CALL mk_plan_get_start_state(mk_plan_handle plan,
    mk_start_state *out_start_state);
TC_API mk_result MK_CALL mk_plan_get_report(mk_plan_handle plan,
    mk_validation_report *out_report);
TC_API mk_result MK_CALL mk_plan_evaluate(mk_plan_handle plan, int64_t time_ns,
    mk_trajectory_state *out_state);

#ifdef __cplusplus
}
#endif

#endif
