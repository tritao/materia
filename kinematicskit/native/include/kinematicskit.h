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
 * a bounded damped least-squares QP step (ProxQP, dense), and a collision
 * world bound to the model's bodies (coal; `kk_collision_*` below).
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
    /** A checked collision pair whose shapes coal cannot compare (e.g. a height field and a mesh). */
    KK_ERROR_UNSUPPORTED = -5,
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

/*
 * Collision world (kinematicskit/plans/COLLISION.md, C1). Shapes attach to a
 * body of a copy of the model (or to the world, body -1) with an offset
 * (seven doubles, in the body's frame), and `kk_collision_update` poses them
 * all from DOF values and root poses. Objects keep their ids (never reused)
 * for the world's lifetime.
 *
 * Every pair of objects has a status (KK_PAIR_*). Only KK_PAIR_CHECKED
 * pairs are queried; the others say why not:
 *   - static:    both objects on the world;
 *   - rigid:     their bodies never move relative to each other (the same
 *                body, or joined only through fixed joints);
 *   - adjacent:  their bodies are one movable joint apart (through fixed
 *                joints on either side);
 *   - reference: they overlapped at the configuration given to
 *                `kk_collision_allow_overlapping`;
 *   - allowed:   allowed by `kk_collision_set_pair_rule`.
 * A KK_PAIR_RULE_CHECK rule overrides all of these. A pair that would be
 * checked but whose shapes coal cannot compare is KK_PAIR_UNSUPPORTED, and
 * queries fail with KK_ERROR_UNSUPPORTED until it is allowed or removed:
 * a check is never silently skipped.
 *
 * Distances are signed (negative when penetrating), with the closest points
 * in world coordinates and the unit normal from the first object toward the
 * second (pairs are reported with the lower id first). Queries return the
 * total number of matching pairs and fill at most the given capacity,
 * closest first.
 */

/** Shape kinds for `kk_collision_add_shape`, with their parameters. */
enum {
    /** Half extents x, y, z. */
    KK_SHAPE_BOX = 0,
    /** Radius. */
    KK_SHAPE_SPHERE = 1,
    /** Radius, half length of the segment along local z. */
    KK_SHAPE_CAPSULE = 2,
    /** Radius, half length along local z. */
    KK_SHAPE_CYLINDER = 3,
    /** Unit normal x, y, z and offset d: the solid side is n·p <= d. */
    KK_SHAPE_HALFSPACE = 4
};

/** A pair's status (`kk_collision_pair_status`). */
enum {
    KK_PAIR_CHECKED = 0,
    KK_PAIR_STATIC = 1,
    KK_PAIR_RIGID = 2,
    KK_PAIR_ADJACENT = 3,
    KK_PAIR_OVERLAPS_AT_REFERENCE = 4,
    KK_PAIR_ALLOWED = 5,
    KK_PAIR_UNSUPPORTED = 6
};

/** Declared pair rules (`kk_collision_set_pair_rule`). */
enum {
    /** Clears a declared rule: the pair's status follows the defaults. */
    KK_PAIR_RULE_DEFAULT = 0,
    KK_PAIR_RULE_ALLOW = 1,
    KK_PAIR_RULE_CHECK = 2
};

typedef struct kk_collision_world_handle { uint32_t id; } kk_collision_world_handle
    KK_HANDLE KK_HANDLE_DESTROY(kk_collision_world_destroy);

/** A collision world over a copy of `model`; shapes are added afterwards. */
KK_API kk_result KK_CALL kk_collision_world_create(kk_model_handle model,
    kk_collision_world_handle *out_world KK_OUT KK_OWNED);
KK_API void KK_CALL kk_collision_world_destroy(kk_collision_world_handle world);

/** Adds a primitive (KK_SHAPE_*, `params` as listed there) on `body` (-1: the world). */
KK_API kk_result KK_CALL kk_collision_add_shape(kk_collision_world_handle world, int32_t body,
    const double *offset KK_IN_ARRAY(offset_count), uint32_t offset_count,
    int32_t kind, const double *params KK_IN_ARRAY(param_count), uint32_t param_count,
    uint32_t *out_object KK_OUT);

/**
 * Adds the convex hull of `points` (x, y, z each; at least four, not all in
 * one plane). The points should be the hull's vertices: interior points are
 * harmless but cost time, since support queries scan them all.
 */
KK_API kk_result KK_CALL kk_collision_add_convex(kk_collision_world_handle world, int32_t body,
    const double *offset KK_IN_ARRAY(offset_count), uint32_t offset_count,
    const double *points KK_IN_ARRAY(point_count), uint32_t point_count,
    uint32_t *out_object KK_OUT);

/** Adds a triangle mesh: `vertices` x, y, z each, `indices` three per triangle. */
KK_API kk_result KK_CALL kk_collision_add_mesh(kk_collision_world_handle world, int32_t body,
    const double *offset KK_IN_ARRAY(offset_count), uint32_t offset_count,
    const double *vertices KK_IN_ARRAY(vertex_count), uint32_t vertex_count,
    const int32_t *indices KK_IN_ARRAY(index_count), uint32_t index_count,
    uint32_t *out_object KK_OUT);

/**
 * Adds a height field centred on its offset: `rows` x `cols` heights
 * (row-major, at least 2 x 2) over `x_size` by `y_size`. Columns run along
 * +x from -x_size/2; rows run along -y from +y_size/2 (coal's layout).
 * Heights below `min_height` are raised to it; the solid reaches down to it.
 * coal answers no distance queries on height fields, so the world compares
 * each cell near the other object as two triangular prisms (split as coal
 * splits them for collision) and takes the closest: a penetration depth is
 * then measured within one cell. Two height fields cannot be compared.
 */
KK_API kk_result KK_CALL kk_collision_add_height_field(kk_collision_world_handle world, int32_t body,
    const double *offset KK_IN_ARRAY(offset_count), uint32_t offset_count,
    double x_size, double y_size,
    const double *heights KK_IN_ARRAY(height_count), uint32_t height_count,
    uint32_t rows, double min_height, uint32_t *out_object KK_OUT);

/** Replaces a height field's heights (same grid size), e.g. after digging. */
KK_API kk_result KK_CALL kk_collision_set_heights(kk_collision_world_handle world, uint32_t object,
    const double *heights KK_IN_ARRAY(height_count), uint32_t height_count);

/** Moves an object to `body` (-1: the world) at `offset`: re-attaches a grasped or placed part. */
KK_API kk_result KK_CALL kk_collision_attach(kk_collision_world_handle world, uint32_t object, int32_t body,
    const double *offset KK_IN_ARRAY(offset_count), uint32_t offset_count);

KK_API kk_result KK_CALL kk_collision_remove(kk_collision_world_handle world, uint32_t object);

/** Declares a rule (KK_PAIR_RULE_*) for one pair of objects. */
KK_API kk_result KK_CALL kk_collision_set_pair_rule(kk_collision_world_handle world, uint32_t a, uint32_t b,
    int32_t rule);

/** A pair's status (KK_PAIR_*). */
KK_API kk_result KK_CALL kk_collision_pair_status(kk_collision_world_handle world, uint32_t a, uint32_t b,
    int32_t *out_status KK_OUT);

/**
 * Marks the pairs that collide at the given configuration (DOF values and
 * optional root poses, as for `kk_forward`) as KK_PAIR_OVERLAPS_AT_REFERENCE,
 * replacing any earlier marks, and leaves the world posed there. Pairs
 * with a declared rule are left as declared. `out_count` is the number
 * marked.
 */
KK_API kk_result KK_CALL kk_collision_allow_overlapping(kk_collision_world_handle world,
    const double *q KK_IN_ARRAY(dof_count), uint32_t dof_count,
    const double *root_poses KK_IN_ARRAY(root_count), uint32_t root_count,
    uint32_t *out_count KK_OUT);

/** Poses every object for DOF values `q` and optional root poses (as for `kk_forward`). */
KK_API kk_result KK_CALL kk_collision_update(kk_collision_world_handle world,
    const double *q KK_IN_ARRAY(dof_count), uint32_t dof_count,
    const double *root_poses KK_IN_ARRAY(root_count), uint32_t root_count);

/**
 * Checked pairs closer than `margin` (0: touching or penetrating) at the
 * current poses, in id order: two ids per pair in `out_pairs` (its length,
 * `pair_capacity`, bounds how many are written).
 */
KK_API kk_result KK_CALL kk_collision_check(kk_collision_world_handle world, double margin,
    int32_t *out_pairs KK_OUT_ARRAY(pair_capacity), uint32_t pair_capacity,
    uint32_t *out_count KK_OUT);

/**
 * Checked pairs closer than `query_distance`, closest first: two ids per pair
 * in `out_pairs`, and ten doubles per pair in `out_results` (signed distance,
 * closest point on the first object, closest point on the second, normal).
 * The shorter of the two arrays bounds how many pairs are written.
 */
KK_API kk_result KK_CALL kk_collision_distances(kk_collision_world_handle world, double query_distance,
    int32_t *out_pairs KK_OUT_ARRAY(pair_capacity), uint32_t pair_capacity,
    double *out_results KK_OUT_ARRAY(result_capacity), uint32_t result_capacity,
    uint32_t *out_count KK_OUT);

#ifdef __cplusplus
}
#endif

#endif
