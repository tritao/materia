#ifndef COLLISIONKIT_H
#define COLLISIONKIT_H

#include <stdint.h>

#if defined(__clang__)
#define CK_OUT __attribute__((annotate("hxi:out")))
#define CK_OWNED __attribute__((annotate("hxi:owned")))
#define CK_HANDLE __attribute__((annotate("hxi:handle")))
#define CK_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#define CK_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#define CK_OUT_ARRAY(count) __attribute__((annotate("hxi:out_array")))
#else
#define CK_OUT
#define CK_OWNED
#define CK_HANDLE
#define CK_HANDLE_DESTROY(symbol)
#define CK_IN_ARRAY(count)
#define CK_OUT_ARRAY(count)
#endif

#if defined(_WIN32)
#if defined(CK_STATIC)
#define CK_API
#elif defined(CK_BUILDING_LIBRARY)
#define CK_API __declspec(dllexport)
#else
#define CK_API __declspec(dllimport)
#endif
#define CK_CALL __cdecl
#else
#define CK_API __attribute__((visibility("default")))
#define CK_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CollisionKit native core: a collision world on coal
 * (collisionkit/plans/COLLISION.md, CL-D1 and CL2).
 *
 * The world knows bodies, not models. The caller registers bodies, poses
 * them (seven doubles each: x, y, z, qx, qy, qz, qw, in world coordinates)
 * and attaches objects (shapes) to them with offsets in the body's frame.
 * Body -1 is the world itself. Bodies are numbered from 0 in the order they
 * are added and are never removed; objects keep their ids (never reused,
 * never zero) for the world's lifetime. One world holds a whole cell:
 * several robots and mechanisms, each the caller's own model, and the
 * environment.
 *
 * Every pair of objects has a status (CK_PAIR_*). Only CK_PAIR_CHECKED pairs
 * are queried; the others say why not, in this order of precedence:
 *   1. a declared rule on the object pair;
 *   2. a declared rule on the body pair (with its reason: rigid, adjacent,
 *      closure, declared, process contact, declared contact);
 *   3. static: both bodies are static (the world is);
 *   4. rigid: both objects are on one body;
 *   5. overlaps at reference: marked by `ck_allow_overlapping` over one
 *      articulation's bodies;
 *   6. checked.
 * A CK_RULE_CHECK rule overrides everything below it. A pair that would be
 * checked but whose shapes coal cannot compare is CK_PAIR_UNSUPPORTED, and
 * queries fail with CK_ERROR_UNSUPPORTED until it is allowed or removed: a
 * check is never silently skipped.
 *
 * Distances are signed (negative when penetrating), with the closest points
 * in world coordinates and the unit normal from the first object toward the
 * second (pairs are reported with the lower object id first). Pair rows are
 * four ints: object a, object b, body a, body b (-1: the world). Distance
 * rows are ten doubles: the signed distance, the closest point on a, the
 * closest point on b, the normal.
 *
 * Margins per pair class (CL-D4) come with the query, as a square table
 * over body groups: `margins[g * groups + h]` with g <= h is the margin
 * between groups g and h (entries below the diagonal are not read). Every
 * body's group must be below the table's size.
 *
 * Functions never throw across this boundary and are not thread-safe for
 * the same world. Handles are process-local, generation-checked and never
 * zero.
 */

enum { CK_API_VERSION = 1 };

typedef int32_t ck_result;
enum {
    CK_OK = 0,
    CK_ERROR_INVALID_ARGUMENT = -1,
    CK_ERROR_INVALID_HANDLE = -2,
    CK_ERROR_OUT_OF_MEMORY = -3,
    /** A checked pair whose shapes coal cannot compare (e.g. two height fields). */
    CK_ERROR_UNSUPPORTED = -5,
    CK_ERROR_INTERNAL = -6
};

/** Primitive kinds for `ck_add_shape`, with their parameters. */
enum {
    /** Half extents x, y, z. */
    CK_SHAPE_BOX = 0,
    /** Radius. */
    CK_SHAPE_SPHERE = 1,
    /** Radius, half length of the segment along local z. */
    CK_SHAPE_CAPSULE = 2,
    /** Radius, half length along local z. */
    CK_SHAPE_CYLINDER = 3,
    /** Unit normal x, y, z and offset d: the solid side is n·p <= d. */
    CK_SHAPE_HALFSPACE = 4
};

/** A pair's status (`ck_pair_status`), and the reason an allow rule records. */
enum {
    CK_PAIR_CHECKED = 0,
    CK_PAIR_STATIC = 1,
    /** Never move relative to each other: one body, or bodies joined only through fixed joints. */
    CK_PAIR_RIGID = 2,
    /** One movable joint apart. */
    CK_PAIR_ADJACENT = 3,
    CK_PAIR_OVERLAPS_AT_REFERENCE = 4,
    /** Allowed by a declared rule. */
    CK_PAIR_ALLOWED = 5,
    CK_PAIR_UNSUPPORTED = 6,
    /** Joined by a closure (a loop of the caller's model). */
    CK_PAIR_CLOSURE = 7,
    /** Process contact (a torch and its seam), declared per window by the caller. */
    CK_PAIR_PROCESS_CONTACT = 8,
    /** Meant to touch (RobotKit's contact pairs, CL-D10). */
    CK_PAIR_DECLARED_CONTACT = 9
};

/** Declared rules (`ck_set_body_rule`, `ck_set_object_rule`). */
enum {
    /** Clears a declared rule. */
    CK_RULE_DEFAULT = 0,
    CK_RULE_ALLOW = 1,
    CK_RULE_CHECK = 2
};

typedef struct ck_world_handle { uint32_t id; } ck_world_handle
    CK_HANDLE CK_HANDLE_DESTROY(ck_world_destroy);

CK_API uint32_t CK_CALL ck_api_version(void);

/** An empty world: no bodies, no objects. */
CK_API ck_result CK_CALL ck_world_create(ck_world_handle *out_world CK_OUT CK_OWNED);
CK_API void CK_CALL ck_world_destroy(ck_world_handle world);

/** Adds a body in `group` (CL-D4's pair classes), posed at the identity and not static. */
CK_API ck_result CK_CALL ck_add_body(ck_world_handle world, int32_t group, uint32_t *out_body CK_OUT);

CK_API ck_result CK_CALL ck_set_body_group(ck_world_handle world, uint32_t body, int32_t group);

/** Marks a body static (fixed in this cell) or not. Pairs between static bodies are never checked. */
CK_API ck_result CK_CALL ck_set_body_static(ck_world_handle world, uint32_t body, int32_t fixed);

/**
 * Poses bodies `first`, `first + 1`, ... from `poses` (seven doubles each);
 * quaternions are normalized. Their objects move with them.
 */
CK_API ck_result CK_CALL ck_set_body_poses(ck_world_handle world, uint32_t first,
    const double *poses CK_IN_ARRAY(pose_count), uint32_t pose_count);

/** Adds a primitive (CK_SHAPE_*, `params` as listed there) on `body` (-1: the world). */
CK_API ck_result CK_CALL ck_add_shape(ck_world_handle world, int32_t body,
    const double *offset CK_IN_ARRAY(offset_count), uint32_t offset_count,
    int32_t kind, const double *params CK_IN_ARRAY(param_count), uint32_t param_count,
    uint32_t *out_object CK_OUT);

/**
 * Adds the convex hull of `points` (x, y, z each; at least four, not all in
 * one plane). The points should be the hull's vertices: interior points are
 * harmless but cost time, since support queries scan them all (no qhull).
 */
CK_API ck_result CK_CALL ck_add_convex(ck_world_handle world, int32_t body,
    const double *offset CK_IN_ARRAY(offset_count), uint32_t offset_count,
    const double *points CK_IN_ARRAY(point_count), uint32_t point_count,
    uint32_t *out_object CK_OUT);

/**
 * Adds a triangle mesh: `vertices` x, y, z each, `indices` three per
 * triangle. A mesh is a surface: a shape wholly inside it is not in
 * contact (CL-D8).
 */
CK_API ck_result CK_CALL ck_add_mesh(ck_world_handle world, int32_t body,
    const double *offset CK_IN_ARRAY(offset_count), uint32_t offset_count,
    const double *vertices CK_IN_ARRAY(vertex_count), uint32_t vertex_count,
    const int32_t *indices CK_IN_ARRAY(index_count), uint32_t index_count,
    uint32_t *out_object CK_OUT);

/**
 * Adds a height field centred on its offset: `rows` x `cols` heights
 * (row-major, at least 2 x 2) over `x_size` by `y_size`. Columns run along
 * +x from -x_size/2; rows run along -y from +y_size/2 (coal's layout). The
 * solid reaches down to `min_height`; heights below it are refused (coal
 * would clamp them, CL-D6), so terrain sets it below the deepest dig.
 * Distances come from coal (our fork, CL-D13), which compares each cell as
 * the two triangular prisms it splits it into for collision: a penetration
 * depth is measured within one cell. A height field is checked against
 * meshes too. Two height fields cannot be compared.
 */
CK_API ck_result CK_CALL ck_add_height_field(ck_world_handle world, int32_t body,
    const double *offset CK_IN_ARRAY(offset_count), uint32_t offset_count,
    double x_size, double y_size,
    const double *heights CK_IN_ARRAY(height_count), uint32_t height_count,
    uint32_t rows, double min_height, uint32_t *out_object CK_OUT);

/** Replaces a height field's heights (same grid, none below its minimum), e.g. after digging. */
CK_API ck_result CK_CALL ck_set_heights(ck_world_handle world, uint32_t object,
    const double *heights CK_IN_ARRAY(height_count), uint32_t height_count);

/**
 * Inflates a primitive or convex object by `radius` (coal's swept-sphere
 * radius): every distance to it shrinks by exactly that much. Meshes and
 * height fields cannot be inflated.
 */
CK_API ck_result CK_CALL ck_set_inflation(ck_world_handle world, uint32_t object, double radius);

/**
 * Moves an object to `body` (-1: the world) at `offset`: a grasped part goes
 * to the tool, a placed part back. It follows its new body's rules; its
 * reference-overlap marks are dropped.
 */
CK_API ck_result CK_CALL ck_attach(ck_world_handle world, uint32_t object, int32_t body,
    const double *offset CK_IN_ARRAY(offset_count), uint32_t offset_count);

CK_API ck_result CK_CALL ck_remove(ck_world_handle world, uint32_t object);

/**
 * Declares a rule (CK_RULE_*) for every object pair between two bodies
 * (-1: the world). An allow rule records `reason`: CK_PAIR_RIGID,
 * CK_PAIR_ADJACENT, CK_PAIR_CLOSURE, CK_PAIR_ALLOWED,
 * CK_PAIR_PROCESS_CONTACT or CK_PAIR_DECLARED_CONTACT; the pair's status is
 * that reason. Other rules ignore it.
 */
CK_API ck_result CK_CALL ck_set_body_rule(ck_world_handle world, int32_t a, int32_t b, int32_t rule,
    int32_t reason);

/** Declares a rule for one pair of objects (e.g. process contact), as for bodies; it overrides body rules. */
CK_API ck_result CK_CALL ck_set_object_rule(ck_world_handle world, uint32_t a, uint32_t b, int32_t rule,
    int32_t reason);

/** A pair's status (CK_PAIR_*). */
CK_API ck_result CK_CALL ck_pair_status(ck_world_handle world, uint32_t a, uint32_t b,
    int32_t *out_status CK_OUT);

/**
 * Over the given bodies (one articulation, posed at its reference
 * configuration), marks the object pairs that collide now as
 * CK_PAIR_OVERLAPS_AT_REFERENCE, replacing earlier marks among those bodies.
 * Pairs with a body outside the set, and pairs with a declared rule, are
 * left alone: an overlap with the environment or another articulation is
 * a layout error, and stays checked. `out_count` is the number marked.
 */
CK_API ck_result CK_CALL ck_allow_overlapping(ck_world_handle world,
    const int32_t *bodies CK_IN_ARRAY(body_count), uint32_t body_count, uint32_t *out_count CK_OUT);

/**
 * Checked pairs closer than `margin` (0: touching or penetrating), in id
 * order: four ints per pair in `out_pairs` (its length bounds how many are
 * written). `out_count` is the total.
 */
CK_API ck_result CK_CALL ck_check(ck_world_handle world, double margin,
    int32_t *out_pairs CK_OUT_ARRAY(pair_capacity), uint32_t pair_capacity,
    uint32_t *out_count CK_OUT);

/**
 * Checked pairs closer than `query_distance`, closest first: four ints per
 * pair in `out_pairs` and ten doubles per pair in `out_results`. The shorter
 * of the two arrays bounds how many pairs are written; `out_count` is the
 * total.
 */
CK_API ck_result CK_CALL ck_distances(ck_world_handle world, double query_distance,
    int32_t *out_pairs CK_OUT_ARRAY(pair_capacity), uint32_t pair_capacity,
    double *out_results CK_OUT_ARRAY(result_capacity), uint32_t result_capacity,
    uint32_t *out_count CK_OUT);

/**
 * The distances of chosen object pairs (two ids each in `pairs`), whatever
 * their status, ten doubles each in `out_results` (as many as pairs): CL-D5
 * bisects one pair without recomputing the rest.
 */
CK_API ck_result CK_CALL ck_pair_distances(ck_world_handle world,
    const int32_t *pairs CK_IN_ARRAY(pair_count), uint32_t pair_count,
    double *out_results CK_OUT_ARRAY(result_capacity), uint32_t result_capacity);

/**
 * The first checked pair, in id order, closer than its margin plus each
 * body's inflation: `margins` per group pair (see above) plus
 * `inflation[body a] + inflation[body b]` (one entry per body, or none;
 * the world's is zero). This is the query main's clearance proof makes
 * (CL-D12), with the inflation carrying CL-D5's motion bound.
 * `out_found` is 1 when there is one: its four ints in `out_pair`, its
 * signed distance and required clearance in `out_result`.
 */
CK_API ck_result CK_CALL ck_violation(ck_world_handle world,
    const double *margins CK_IN_ARRAY(margin_count), uint32_t margin_count,
    const double *inflation CK_IN_ARRAY(inflation_count), uint32_t inflation_count,
    int32_t *out_pair CK_OUT_ARRAY(pair_capacity), uint32_t pair_capacity,
    double *out_result CK_OUT_ARRAY(result_capacity), uint32_t result_capacity,
    int32_t *out_found CK_OUT);

/**
 * The closest checked pair, clear or not, as `ck_violation` reports one
 * (the required clearance from `margins`, without inflation). `out_found` is
 * 0 when no pair is checked.
 */
CK_API ck_result CK_CALL ck_closest(ck_world_handle world,
    const double *margins CK_IN_ARRAY(margin_count), uint32_t margin_count,
    int32_t *out_pair CK_OUT_ARRAY(pair_capacity), uint32_t pair_capacity,
    double *out_result CK_OUT_ARRAY(result_capacity), uint32_t result_capacity,
    int32_t *out_found CK_OUT);

/**
 * Batched violations (CL-D7, CL-D12): `poses` holds sets of every body's
 * pose (seven doubles per body, bodies in order), and `inflation` one entry
 * per body per set (or none). The sets are checked in order, as
 * `ck_violation`, until one fails: `out_set` is its index (-1 when every set
 * is clear) and the violation is reported as there. The world is left posed
 * at the last set checked.
 */
CK_API ck_result CK_CALL ck_violation_batch(ck_world_handle world,
    const double *poses CK_IN_ARRAY(pose_count), uint32_t pose_count,
    const double *inflation CK_IN_ARRAY(inflation_count), uint32_t inflation_count,
    const double *margins CK_IN_ARRAY(margin_count), uint32_t margin_count,
    int32_t *out_pair CK_OUT_ARRAY(pair_capacity), uint32_t pair_capacity,
    double *out_result CK_OUT_ARRAY(result_capacity), uint32_t result_capacity,
    int32_t *out_set CK_OUT);

/**
 * As `ck_violation_batch`, but checks every set: `out_flags` (one per set)
 * is 1 where the set has a violation, 0 where it is clear. For a planner's
 * edge checks, which need every failing set of a batch (CL-D7). The world is
 * left posed at the last set.
 */
CK_API ck_result CK_CALL ck_violation_sets(ck_world_handle world,
    const double *poses CK_IN_ARRAY(pose_count), uint32_t pose_count,
    const double *inflation CK_IN_ARRAY(inflation_count), uint32_t inflation_count,
    const double *margins CK_IN_ARRAY(margin_count), uint32_t margin_count,
    int32_t *out_flags CK_OUT_ARRAY(flag_capacity), uint32_t flag_capacity);

/*
 * Convex decomposition (CL-D11): V-HACD 4 splits a triangle mesh into convex
 * pieces, then the world measures how far points sampled over every
 * triangle (no point of the mesh farther than the sample spacing from a
 * sample) lie outside the pieces' union. Inflating every piece by
 * `measured + spacing` makes the union enclose the mesh.
 */
typedef struct ck_decomposition_handle { uint32_t id; } ck_decomposition_handle
    CK_HANDLE CK_HANDLE_DESTROY(ck_decomposition_destroy);

/**
 * Decomposes a mesh (`vertices` x, y, z each, `indices` three per triangle)
 * into at most `max_pieces` pieces of at most `max_piece_vertices` (4..64)
 * vertices, voxelized at `resolution` voxels. `sample_spacing` is the
 * enclosure samples' spacing (0: 0.5 % of the mesh's bounding diagonal).
 */
CK_API ck_result CK_CALL ck_decompose(const double *vertices CK_IN_ARRAY(vertex_count), uint32_t vertex_count,
    const int32_t *indices CK_IN_ARRAY(index_count), uint32_t index_count,
    uint32_t max_pieces, uint32_t resolution, uint32_t max_piece_vertices, double sample_spacing,
    ck_decomposition_handle *out_decomposition CK_OUT CK_OWNED);
CK_API void CK_CALL ck_decomposition_destroy(ck_decomposition_handle decomposition);

/** The number of pieces; `out_values` gets the measured outside distance, the sample spacing and the inflation. */
CK_API ck_result CK_CALL ck_decomposition_info(ck_decomposition_handle decomposition, uint32_t *out_pieces CK_OUT,
    double *out_values CK_OUT_ARRAY(value_capacity), uint32_t value_capacity);

/** One piece's vertices (x, y, z each) into `out_points`; `out_count` is how many doubles it has. */
CK_API ck_result CK_CALL ck_decomposition_piece(ck_decomposition_handle decomposition, uint32_t piece,
    double *out_points CK_OUT_ARRAY(point_capacity), uint32_t point_capacity, uint32_t *out_count CK_OUT);

#ifdef __cplusplus
}
#endif

#endif
