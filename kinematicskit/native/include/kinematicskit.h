#ifndef KINEMATICSKIT_H
#define KINEMATICSKIT_H

#include <stdint.h>

#if defined(__clang__)
#define KK_OUT __attribute__((annotate("hxi:out")))
#define KK_OWNED __attribute__((annotate("hxi:owned")))
#define KK_HANDLE __attribute__((annotate("hxi:handle")))
#define KK_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#define KK_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#define KK_OUT_ARRAY(count) __attribute__((annotate("hxi:out_array")))
#else
#define KK_OUT
#define KK_OWNED
#define KK_HANDLE
#define KK_HANDLE_DESTROY(symbol)
#define KK_IN_ARRAY(count)
#define KK_OUT_ARRAY(count)
#endif

#if defined(_WIN32)
#if defined(KK_STATIC)
#define KK_API
#elif defined(KK_BUILDING_LIBRARY)
#define KK_API __declspec(dllexport)
#else
#define KK_API __declspec(dllimport)
#endif
#define KK_CALL __cdecl
#else
#define KK_API __attribute__((visibility("default")))
#define KK_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * KinematicsKit native core: the compiled kinematic model of the Haxe kit
 * (`kinematicskit.KinematicModel`), its forward kinematics and Jacobians,
 * and a bounded damped least-squares QP step (ProxQP, dense).
 *
 * A model arrives once as two packed arrays (layout below, produced by
 * `kinematicskit.native.NativeKinematics`), so the boundary carries only
 * flat numbers. Transforms are seven doubles (x, y, z, qx, qy, qz, qw);
 * quaternions cross the boundary in that order (KK-D10). Lengths are in the
 * model's unit. The arithmetic matches the Haxe snapshot operation for
 * operation.
 *
 * Functions never throw across this boundary and are not thread-safe for
 * the same model. Handles are process-local, generation-checked and never
 * zero.
 *
 * Packed model, format 2.
 *   ints:  [2, bodies, joints, dofs, frames, terms,
 *           per body:  parent joint (-1 for a root),
 *           body order (bodies entries),
 *           per joint: kind (0 fixed, 1 revolute, 2 prismatic), parent body,
 *                      child body, DOF (-1 fixed), coupling source (-1 none, -2 sum),
 *           joint order (joints entries), joint value order (joints entries),
 *           per frame: body,
 *           CSR joint term starts (joints + 1), term DOFs (terms)]
 *   reals: [per body: root pose (7),
 *           per joint: parent_T_joint (7), joint_T_child (7), unit axis (3),
 *                      coupling ratio, coupling offset, Jacobian scale,
 *           per frame: body_T_frame (7), joint constants (joints), term scales (terms)]
 */

enum { KK_API_VERSION = 1, KK_MODEL_FORMAT = 2 };

typedef int32_t kk_result;
enum {
    KK_OK = 0,
    KK_ERROR_INVALID_ARGUMENT = -1,
    KK_ERROR_INVALID_HANDLE = -2,
    KK_ERROR_OUT_OF_MEMORY = -3,
    KK_ERROR_SOLVER = -4,
    KK_ERROR_INTERNAL = -6
};

typedef struct kk_model_handle { uint32_t id; } kk_model_handle
    KK_HANDLE KK_HANDLE_DESTROY(kk_model_destroy);

KK_API uint32_t KK_CALL kk_api_version(void);

/** Validates and copies a packed model (see the layout above). */
KK_API kk_result KK_CALL kk_model_create(
    const int32_t *ints KK_IN_ARRAY(int_count), uint32_t int_count,
    const double *reals KK_IN_ARRAY(real_count), uint32_t real_count,
    kk_model_handle *out_model KK_OUT KK_OWNED);
KK_API void KK_CALL kk_model_destroy(kk_model_handle model);

/**
 * Forward kinematics: world poses of every body (7 per body) for DOF values
 * `q`. `root_poses` overrides every body's root pose (7 per body, ignored
 * for non-roots) when `root_count` equals the body count; pass zero to use
 * the model's.
 */
KK_API kk_result KK_CALL kk_forward(kk_model_handle model,
    const double *q KK_IN_ARRAY(dof_count), uint32_t dof_count,
    const double *root_poses KK_IN_ARRAY(root_count), uint32_t root_count,
    double *out_poses KK_OUT_ARRAY(pose_count), uint32_t pose_count);

/**
 * Geometric Jacobian (6 rows: linear, then angular; row-major) of the world
 * point `point` (3) moving rigidly with `body`, over the DOFs listed in
 * `columns`, at DOF values `q` with the model's root poses.
 */
KK_API kk_result KK_CALL kk_point_jacobian(kk_model_handle model,
    const double *q KK_IN_ARRAY(dof_count), uint32_t dof_count,
    uint32_t body, const double *point KK_IN_ARRAY(point_count), uint32_t point_count,
    const int32_t *columns KK_IN_ARRAY(column_count), uint32_t column_count,
    double *out_jacobian KK_OUT_ARRAY(jacobian_count), uint32_t jacobian_count);

/** Why a QP step stopped (`kk_qp_solve`'s `out_status`). */
enum {
    KK_QP_SOLVED = 0,
    KK_QP_MAX_ITERATIONS = 1,
    /** The bounds admit no step (e.g. a lower bound above its upper bound). */
    KK_QP_INFEASIBLE = 2,
    KK_QP_FAILED = 3
};

typedef struct kk_qp_handle { uint32_t id; } kk_qp_handle
    KK_HANDLE KK_HANDLE_DESTROY(kk_qp_destroy);

/**
 * A QP step solver for `width` variables. Keep one per problem width and
 * reuse it: every solve warm-starts from the previous one.
 */
KK_API kk_result KK_CALL kk_qp_create(uint32_t width, kk_qp_handle *out_qp KK_OUT KK_OWNED);
KK_API void KK_CALL kk_qp_destroy(kk_qp_handle qp);

/**
 * Solves  minimize 1/2 |J*D - e|^2 + 1/2 lambda^2|D|^2  subject to  lower <= D <= upper
 * for D (`step_count` = width). `jacobian` is `row_count` x width,
 * row-major; `residual` has `row_count` entries. Infinite bounds mean
 * unbounded. `tolerance` is the solver's absolute accuracy and
 * `max_iterations` its budget. Returns KK_OK when the solver ran, with its
 * outcome in `out_status` (KK_QP_*); `out_step` then holds its last iterate,
 * projected onto the bounds so they hold exactly.
 */
KK_API kk_result KK_CALL kk_qp_solve(kk_qp_handle qp,
    const double *jacobian KK_IN_ARRAY(jacobian_count), uint32_t jacobian_count,
    const double *residual KK_IN_ARRAY(row_count), uint32_t row_count,
    const double *lower KK_IN_ARRAY(lower_count), uint32_t lower_count,
    const double *upper KK_IN_ARRAY(upper_count), uint32_t upper_count,
    double damping, double tolerance, uint32_t max_iterations,
    double *out_step KK_OUT_ARRAY(step_count), uint32_t step_count,
    int32_t *out_status KK_OUT, uint32_t *out_iterations KK_OUT);

/**
 * Sets general rows  row_lower <= C*D <= row_upper  for this QP's following
 * solves (`constraint` is `constraint_row_count` x width, row-major; infinite
 * bounds mean unbounded), e.g. collision-avoidance rows; zero rows clear them.
 * With rows set, `kk_qp_solve` honours them, and when they and the bounds
 * admit no step it relaxes every row by a slack penalized far above the task
 * (as mink relaxes its collision rows); `kk_qp_relaxed` then says which rows
 * needed it, and `out_status` is the relaxed solve's.
 */
KK_API kk_result KK_CALL kk_qp_set_rows(kk_qp_handle qp,
    const double *constraint KK_IN_ARRAY(constraint_count), uint32_t constraint_count,
    const double *row_lower KK_IN_ARRAY(constraint_row_count), uint32_t constraint_row_count,
    const double *row_upper KK_IN_ARRAY(row_upper_count), uint32_t row_upper_count);

/** After a solve with rows: 1 for each row (in order) whose relaxation was needed, else 0. */
KK_API kk_result KK_CALL kk_qp_relaxed(kk_qp_handle qp,
    int32_t *out_relaxed KK_OUT_ARRAY(relaxed_count), uint32_t relaxed_count);

#ifdef __cplusplus
}
#endif

#endif
