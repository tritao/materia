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
 * and (from K3b) a QP differential-IK step.
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
 * Packed model, format 1.
 *   ints:  [1, bodies, joints, dofs, frames,
 *           per body:  parent joint (-1 for a root),
 *           body order (bodies entries),
 *           per joint: kind (0 fixed, 1 revolute, 2 prismatic), parent body,
 *                      child body, DOF (-1 fixed), coupling source (-1 none),
 *           joint order (joints entries), joint value order (joints entries),
 *           per frame: body]
 *   reals: [per body: root pose (7),
 *           per joint: parent_T_joint (7), joint_T_child (7), unit axis (3),
 *                      coupling ratio, coupling offset, Jacobian scale,
 *           per frame: body_T_frame (7)]
 */

enum { KK_API_VERSION = 1, KK_MODEL_FORMAT = 1 };

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

#ifdef __cplusplus
}
#endif

#endif
