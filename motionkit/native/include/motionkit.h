#ifndef MOTIONKIT_H
#define MOTIONKIT_H

#include "trajectory_core.h"

#if defined(_WIN32)
#if defined(MK_STATIC)
#define MK_API
#elif defined(MK_BUILDING_LIBRARY)
#define MK_API __declspec(dllexport)
#else
#define MK_API __declspec(dllimport)
#endif
#define MK_CALL __cdecl
#else
#define MK_API __attribute__((visibility("default")))
#define MK_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mk_path_handle { uint32_t id; } mk_path_handle
    MK_HANDLE MK_HANDLE_DESTROY(mk_path_destroy);
typedef struct mk_time_law_handle { uint32_t id; } mk_time_law_handle
    MK_HANDLE MK_HANDLE_DESTROY(mk_time_law_destroy);

/** OPW's seven geometric parameters and manufacturer joint conventions. */
typedef struct mk_opw_parameters {
    uint32_t struct_size MK_STRUCT_SIZE;
    double a1, a2, b, c1, c2, c3, c4;
    double offsets[6];
    int8_t sign_corrections[6];
} mk_opw_parameters;

typedef struct mk_opw_pose {
    uint32_t struct_size MK_STRUCT_SIZE;
    double position[3];
    double quaternion[4]; /**< x, y, z, w. */
} mk_opw_pose;

typedef struct mk_opw_solution {
    uint32_t struct_size MK_STRUCT_SIZE;
    double joints[6];
    uint8_t valid;
    uint8_t singular;
} mk_opw_solution;

/** Cartesian chain represented by its zero-pose space screws and TCP pose.
 * The three translation axes are columns of translation_axes. Optional C/A
 * axes and origins are in the same reference frame, in chain order. */
typedef struct mk_analytic_cartesian_model {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count; /**< 3 (XYZ), 4 (XYZ+C), or 5 (XYZ+C+A). */
    double translation_axes[9];
    double rotary_axes[6];
    double rotary_origins[6];
    double home_position[3];
    double home_quaternion[4];
} mk_analytic_cartesian_model;

typedef struct mk_analytic_solution {
    uint32_t struct_size MK_STRUCT_SIZE;
    double joints[6];
    uint32_t branch;
    uint32_t singular; /**< Family-specific flags; Cartesian C axis or UR wrist/elbow. */
} mk_analytic_solution;

/** UR DH family: alpha = [pi/2, 0, 0, pi/2, -pi/2, 0].
 * Canonical DH q = sign_corrections[i] * model_q - offsets[i]. */
typedef struct mk_ur_parameters {
    uint32_t struct_size MK_STRUCT_SIZE;
    double a2, a3, d1, d4, d5, d6;
    double offsets[6];
    int8_t sign_corrections[6];
} mk_ur_parameters;

/** Deterministic orientation lattice in the target TCP frame. */
typedef struct mk_orientation_lattice {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t mode; /**< 0 fixed, 1 free TCP spin, 2 cone plus free spin. */
    uint32_t roll_count;
    uint32_t tilt_rings; /**< Cone rings excluding its centre. */
    uint32_t azimuth_count;
    double half_angle; /**< Cone half-angle in radians, [0, pi]. */
} mk_orientation_lattice;

typedef struct mk_orientation_sample {
    uint32_t struct_size MK_STRUCT_SIZE;
    double position[3];
    double quaternion[4]; /**< x, y, z, w. */
    uint32_t roll_index;
    uint32_t tilt_index; /**< 0 is the cone centre. */
    uint32_t azimuth_index;
} mk_orientation_sample;

/** Finite planning limits used for complete periodic-lift enumeration. */
typedef struct mk_joint_lift_request {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    double lower[MK_MAX_JOINTS];
    double upper[MK_MAX_JOINTS];
    uint8_t periodic[MK_MAX_JOINTS];
} mk_joint_lift_request;

typedef struct mk_joint_lift {
    uint32_t struct_size MK_STRUCT_SIZE;
    double joints[MK_MAX_JOINTS];
    int32_t wraps[MK_MAX_JOINTS];
} mk_joint_lift;

/** External-axis Cartesian product. A rule is a one-point equal-bound range. */
typedef struct mk_external_lattice {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    uint32_t axis_count;
    uint32_t joint_indices[MK_MAX_JOINTS];
    uint32_t point_counts[MK_MAX_JOINTS];
    double lower[MK_MAX_JOINTS];
    double upper[MK_MAX_JOINTS];
} mk_external_lattice;

typedef struct mk_external_cell {
    uint32_t struct_size MK_STRUCT_SIZE;
    double joints[MK_MAX_JOINTS];
    uint32_t coordinates[MK_MAX_JOINTS]; /**< In external-axis descriptor order. */
} mk_external_cell;

/** Space screws of external joints, in a common root frame at q=0.
 * Entries follow each chain's root-to-tip order. Scope 0 moves the arm base,
 * scope 1 moves the work frame, scope 2 moves both. Kind 0 is prismatic, kind 1 is revolute. */
typedef struct mk_serial_cell_model {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    uint32_t external_count;
    uint32_t arm_joint_count; /**< 3–5 Cartesian joints or 6 serial-arm joints. */
    uint32_t arm_joint_indices[6];
    uint32_t external_joint_indices[MK_MAX_JOINTS];
    uint32_t external_scopes[MK_MAX_JOINTS];
    uint32_t external_kinds[MK_MAX_JOINTS];
    double external_axes[3*MK_MAX_JOINTS];
    double external_origins[3*MK_MAX_JOINTS];
    double base_position[3];
    double base_quaternion[4];
    double work_position[3];
    double work_quaternion[4];
    double tool_position[3];
    double tool_quaternion[4];
} mk_serial_cell_model;

typedef struct mk_lattice_candidate {
    uint32_t struct_size MK_STRUCT_SIZE;
    double joints[MK_MAX_JOINTS];
    int32_t wraps[MK_MAX_JOINTS];
    uint32_t external_coordinates[MK_MAX_JOINTS];
    uint32_t roll_index;
    uint32_t tilt_index;
    uint32_t azimuth_index;
    uint32_t branch;
    uint32_t singular;
} mk_lattice_candidate;

/** Structured ladder costs and lattice dimensions. External axes use their
 * process weights in weights[]; speeds normalize travel costs. */
typedef struct mk_ladder_request {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    uint32_t external_count;
    uint32_t roll_count;
    uint32_t tilt_count;
    uint32_t azimuth_count;
    double max_jump[MK_MAX_JOINTS];
    double weights[MK_MAX_JOINTS];
    double velocity[MK_MAX_JOINTS];
    double start_joints[MK_MAX_JOINTS];
    double roll_weight;
} mk_ladder_request;

typedef struct mk_ladder_result {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t failed_sample; /**< UINT32_MAX on success. */
    uint32_t failure_kind; /**< 0 success, 1 empty candidates, 2 no legal edges. */
    double failed_distance;
    double cost;
    uint64_t tested_edges;
} mk_ladder_result;

/** Candidate sets for one Descartes ladder-graph selection call. */
typedef struct mk_configuration_request {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    uint32_t threads; /**< 1 by default; multiple threads need OpenMP. */
    uint8_t has_preferred;
    double lower[MK_MAX_JOINTS];
    double upper[MK_MAX_JOINTS];
    double max_jump[MK_MAX_JOINTS];
    double weights[MK_MAX_JOINTS];
    double preferred[MK_MAX_JOINTS];
} mk_configuration_request;

typedef struct mk_configuration_sample {
    uint32_t struct_size MK_STRUCT_SIZE;
    double distance;
    uint32_t first_candidate;
    uint32_t candidate_count;
} mk_configuration_sample;

typedef struct mk_opw_path_sample {
    uint32_t struct_size MK_STRUCT_SIZE;
    double distance;
    double position[3];
    double quaternion[4]; /**< OPW flange pose in the solver's local frame. */
} mk_opw_path_sample;

typedef struct mk_configuration_candidate {
    uint32_t struct_size MK_STRUCT_SIZE;
    double joints[MK_MAX_JOINTS];
} mk_configuration_candidate;

typedef struct mk_configuration_solution {
    uint32_t struct_size MK_STRUCT_SIZE;
    double joints[MK_MAX_JOINTS];
    uint32_t failed_sample; /**< Only slot zero: UINT32_MAX on success. */
    double failed_distance;
    double cost;
    int8_t diagnostic[256];
} mk_configuration_solution;

/**
 * Joint path derivatives are with respect to path parameter s. `second` is the
 * second derivative leaving the sample and `second_before` the one arriving at
 * it; they differ where the path's curvature jumps, such as where a line meets
 * an arc, and are equal elsewhere.
 */
typedef struct mk_path_sample {
    uint32_t struct_size MK_STRUCT_SIZE;
    double s;
    uint32_t joint_count;
    double position[MK_MAX_JOINTS];
    double first[MK_MAX_JOINTS];
    double second[MK_MAX_JOINTS];
    double second_before[MK_MAX_JOINTS];
} mk_path_sample;

/** s(t) = start_s + speed*tau + acceleration*tau^2/2, tau in seconds. */
typedef struct mk_time_stage {
    uint32_t struct_size MK_STRUCT_SIZE;
    int64_t start_ns;
    int64_t duration_ns;
    double start_s;
    double speed;
    double acceleration;
} mk_time_stage;

enum { MK_TIMING_BINDING_JOINT_VELOCITY = 1,
    MK_TIMING_BINDING_JOINT_ACCELERATION = 2,
    MK_TIMING_BINDING_FEED_CAP = 3 };
typedef struct mk_timing_binding {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t stage_index;
    uint32_t kind;
    uint32_t joint; /**< UINT32_MAX for a feed cap. */
    double limit;
} mk_timing_binding;

enum { MK_SYNCHRONIZATION_TIME = 0 };
enum { MK_CONTROL_POSITION = 0, MK_CONTROL_VELOCITY_STOP = 1 };

/** Offline Ruckig request. Velocity-stop ignores target position and velocity limits. */
typedef struct mk_state_to_state_request {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    uint32_t synchronization; /**< Only MK_SYNCHRONIZATION_TIME is supported. */
    uint32_t control_mode; /**< Position target or velocity-control stop. */
    double current_position[MK_MAX_JOINTS];
    double current_velocity[MK_MAX_JOINTS];
    double current_acceleration[MK_MAX_JOINTS];
    double target_position[MK_MAX_JOINTS];
    double target_velocity[MK_MAX_JOINTS];
    double target_acceleration[MK_MAX_JOINTS];
    double max_velocity[MK_MAX_JOINTS];
    double max_acceleration[MK_MAX_JOINTS];
    double max_jerk[MK_MAX_JOINTS];
} mk_state_to_state_request;

/** All eight slots are written, including invalid solutions. */
MK_API mk_result MK_CALL mk_opw_forward(const mk_opw_parameters *parameters,
    const double *joints MK_IN_ARRAY(joint_count), uint32_t joint_count,
    mk_opw_pose *out_pose MK_OUT);
MK_API mk_result MK_CALL mk_opw_inverse(const mk_opw_parameters *parameters,
    const mk_opw_pose *pose, mk_opw_solution *out_solutions MK_OUT_ARRAY(solution_count),
    uint32_t solution_count);
MK_API mk_result MK_CALL mk_analytic_cartesian_forward(const mk_analytic_cartesian_model *model,
    const double *joints MK_IN_ARRAY(joint_count), uint32_t joint_count, mk_opw_pose *out_pose MK_OUT);
/** All geometric branches, before limits/wrap enumeration by the sampler.
 * axis_only frees TCP spin; fixed orientation filters branches by full rotation.
 * The seed selects C at its explicit axis singularity. */
MK_API mk_result MK_CALL mk_analytic_cartesian_inverse(const mk_analytic_cartesian_model *model,
    const mk_opw_pose *target, uint32_t axis_only, double singular_c_seed,
    mk_analytic_solution *out_solutions MK_OUT_ARRAY(solution_capacity), uint32_t solution_capacity,
    uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_analytic_ur_forward(const mk_ur_parameters *parameters,
    const double *joints MK_IN_ARRAY(joint_count), uint32_t joint_count, mk_opw_pose *out_pose MK_OUT);
/** Up to eight branches. Wrist-singular solutions preserve the supplied q6.
 * singular bits: 1 wrist, 2 elbow. Limits and periodic lifts belong to sampling. */
MK_API mk_result MK_CALL mk_analytic_ur_inverse(const mk_ur_parameters *parameters,
    const mk_opw_pose *target, double singular_q6_seed,
    mk_analytic_solution *out_solutions MK_OUT_ARRAY(solution_capacity), uint32_t solution_capacity,
    uint32_t *out_count MK_OUT);
/** Count first, then allocate exactly enough output. Invalid requests fail before writing. */
MK_API mk_result MK_CALL mk_orientation_lattice_count(const mk_orientation_lattice *lattice,
    uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_sample_orientations(const mk_orientation_lattice *lattice,
    const mk_opw_pose *target, mk_orientation_sample *out_samples MK_OUT_ARRAY(sample_capacity),
    uint32_t sample_capacity, uint32_t *out_count MK_OUT);
/** All legal lifts, in lexicographic wrap order; zero count means limits reject the branch.
 * Infinite planning ranges or a count/index overflow return INVALID_ARGUMENT. */
MK_API mk_result MK_CALL mk_joint_lift_count(const mk_joint_lift_request *request,
    const double *joints MK_IN_ARRAY(joint_count), uint32_t joint_count, uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_enumerate_joint_lifts(const mk_joint_lift_request *request,
    const double *joints MK_IN_ARRAY(joint_count), uint32_t joint_count,
    mk_joint_lift *out_lifts MK_OUT_ARRAY(lift_capacity), uint32_t lift_capacity, uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_external_lattice_count(const mk_external_lattice *lattice,
    uint32_t *out_count MK_OUT);
/** Axis endpoints are included; the last external coordinate varies fastest.
 * Non-external joints are copied from the seed. Zero external axes yields one cell. */
MK_API mk_result MK_CALL mk_sample_external_cells(const mk_external_lattice *lattice,
    const double *seed MK_IN_ARRAY(joint_count), uint32_t joint_count,
    mk_external_cell *out_cells MK_OUT_ARRAY(cell_capacity), uint32_t cell_capacity, uint32_t *out_count MK_OUT);
/** Complete UR external-cell x orientation x branch x legal-lift sampling.
 * Target poses are in the work frame. Tool/base conventions match UR analytic FK.
 * External coordinates are held and are never independently lifted. */
MK_API mk_result MK_CALL mk_ur_candidate_count(const mk_ur_parameters *parameters,
    const mk_serial_cell_model *model, const mk_external_lattice *external,
    const mk_orientation_lattice *orientation, const mk_joint_lift_request *limits,
    const mk_opw_pose *target, const double *seed MK_IN_ARRAY(joint_count), uint32_t joint_count,
    uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_sample_ur_candidates(const mk_ur_parameters *parameters,
    const mk_serial_cell_model *model, const mk_external_lattice *external,
    const mk_orientation_lattice *orientation, const mk_joint_lift_request *limits,
    const mk_opw_pose *target, const double *seed MK_IN_ARRAY(joint_count), uint32_t joint_count,
    mk_lattice_candidate *out_candidates MK_OUT_ARRAY(candidate_capacity), uint32_t candidate_capacity,
    uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_opw_candidate_count(const mk_opw_parameters *parameters,
    const mk_serial_cell_model *model, const mk_external_lattice *external,
    const mk_orientation_lattice *orientation, const mk_joint_lift_request *limits,
    const mk_opw_pose *target, const double *seed MK_IN_ARRAY(joint_count), uint32_t joint_count,
    uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_sample_opw_candidates(const mk_opw_parameters *parameters,
    const mk_serial_cell_model *model, const mk_external_lattice *external,
    const mk_orientation_lattice *orientation, const mk_joint_lift_request *limits,
    const mk_opw_pose *target, const double *seed MK_IN_ARRAY(joint_count), uint32_t joint_count,
    mk_lattice_candidate *out_candidates MK_OUT_ARRAY(candidate_capacity), uint32_t candidate_capacity,
    uint32_t *out_count MK_OUT);
/** Cartesian free-spin/cone cells solve the tool-axis constraint. Repeated
 * geometric configurations retain their first lattice coordinates. The Cartesian
 * descriptor includes its TCP, so model.tool must be identity and XYZ nonperiodic. */
MK_API mk_result MK_CALL mk_cartesian_candidate_count(const mk_analytic_cartesian_model *parameters,
    const mk_serial_cell_model *model, const mk_external_lattice *external,
    const mk_orientation_lattice *orientation, const mk_joint_lift_request *limits,
    const mk_opw_pose *target, const double *seed MK_IN_ARRAY(joint_count), uint32_t joint_count,
    uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_sample_cartesian_candidates(const mk_analytic_cartesian_model *parameters,
    const mk_serial_cell_model *model, const mk_external_lattice *external,
    const mk_orientation_lattice *orientation, const mk_joint_lift_request *limits,
    const mk_opw_pose *target, const double *seed MK_IN_ARRAY(joint_count), uint32_t joint_count,
    mk_lattice_candidate *out_candidates MK_OUT_ARRAY(candidate_capacity), uint32_t candidate_capacity,
    uint32_t *out_count MK_OUT);
/** Returns MK_ERROR_GENERATION with a sample-distance diagnostic if disconnected. */
/** No candidate copies: samples address slices of the candidate array.
 * Costs are nonnegative; +infinity disables a candidate. Selected indices are
 * global candidate-array offsets. Failure details remain in out_result. */
MK_API mk_result MK_CALL mk_search_ladder(const mk_ladder_request *request,
    const mk_configuration_sample *samples MK_IN_ARRAY(sample_count), uint32_t sample_count,
    const mk_lattice_candidate *candidates MK_IN_ARRAY(candidate_count),
    const double *state_costs MK_IN_ARRAY(candidate_count), uint32_t candidate_count,
    uint32_t *out_indices MK_OUT_ARRAY(sample_count), mk_ladder_result *out_result);

MK_API mk_result MK_CALL mk_select_configurations(const mk_configuration_request *request,
    const mk_configuration_sample *samples MK_IN_ARRAY(sample_count), uint32_t sample_count,
    const mk_configuration_candidate *candidates MK_IN_ARRAY(candidate_count), uint32_t candidate_count,
    mk_configuration_solution *out_sequence MK_OUT_ARRAY(sample_count));
/** Samples OPW's eight IK branches natively before the same Descartes search. */
MK_API mk_result MK_CALL mk_select_opw_configurations(
    const mk_configuration_request *request, const mk_opw_parameters *parameters,
    const mk_opw_path_sample *samples MK_IN_ARRAY(sample_count), uint32_t sample_count,
    const double *start_joints MK_IN_ARRAY(joint_count), uint32_t joint_count,
    mk_configuration_solution *out_sequence MK_OUT_ARRAY(sample_count));
MK_API mk_result MK_CALL mk_path_create(const mk_path_sample *samples MK_IN_ARRAY(sample_count),
    uint32_t sample_count, mk_path_handle *out_path MK_OUT MK_OWNED);
MK_API void MK_CALL mk_path_destroy(mk_path_handle path);
MK_API mk_result MK_CALL mk_time_law_create(const mk_time_stage *stages MK_IN_ARRAY(stage_count),
    uint32_t stage_count, mk_time_law_handle *out_law MK_OUT MK_OWNED);
MK_API void MK_CALL mk_time_law_destroy(mk_time_law_handle law);
/** Returns seconds from the time-law epoch; exact at stage boundaries. */
MK_API mk_result MK_CALL mk_path_distance_to_time(mk_time_law_handle law,
    double s, double *out_seconds MK_OUT);
/**
 * Returns the path distance reached at each of `count` times, in seconds from the time-law
 * epoch: the inverse of mk_path_distance_to_time, evaluated in closed form per stage. Times
 * outside the law clamp to its first and last distance. Non-finite times are rejected.
 * One call replaces a bisection of mk_path_distance_to_time per sample.
 */
MK_API mk_result MK_CALL mk_path_times_to_distances(mk_time_law_handle law,
    const double *seconds MK_IN_ARRAY(count), uint32_t count,
    double *out_distances MK_OUT_ARRAY(count));
/** Quintic Hermite lowering, with adaptive knots and exact polynomial-deviation extrema. */
MK_API mk_result MK_CALL mk_path_lower(mk_path_handle path, mk_time_law_handle law,
    double tolerance, mk_trajectory_handle *out_trajectory MK_OUT MK_OWNED);
/** Reachability-based TOPP-RA timing and its validated lowered trajectory. */
MK_API mk_result MK_CALL mk_time_path(mk_path_handle path,
    const double *max_velocity MK_IN_ARRAY(joint_count),
    const double *max_acceleration MK_IN_ARRAY(joint_count), uint32_t joint_count,
    const double *speed_caps MK_IN_ARRAY(speed_cap_count), uint32_t speed_cap_count,
    double start_speed, double end_speed, double lowering_tolerance,
    mk_time_law_handle *out_law MK_OUT MK_OWNED,
    mk_trajectory_handle *out_trajectory MK_OUT MK_OWNED);
MK_API mk_result MK_CALL mk_time_law_binding_count(mk_time_law_handle law,
    uint32_t *out_count MK_OUT);
MK_API mk_result MK_CALL mk_time_law_get_binding(mk_time_law_handle law,
    uint32_t index, mk_timing_binding *out_binding);
/** Converts Ruckig Community's offline phases to degree-3 native segments. */
MK_API mk_result MK_CALL mk_generate_state_to_state(const mk_state_to_state_request *request,
    mk_trajectory_handle *out_trajectory MK_OUT MK_OWNED, int32_t *out_ruckig_result MK_OUT);

#ifdef __cplusplus
}
#endif
#endif
