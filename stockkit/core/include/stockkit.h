#ifndef STOCKKIT_H
#define STOCKKIT_H

#include <stdint.h>

#if defined(__clang__)
#define SK_OUT __attribute__((annotate("hxi:out")))
#define SK_OWNED __attribute__((annotate("hxi:owned")))
#define SK_HANDLE __attribute__((annotate("hxi:handle")))
#define SK_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#define SK_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#define SK_OUT_ARRAY(count) __attribute__((annotate("hxi:out_array")))
#define SK_STRUCT_SIZE __attribute__((annotate("hxi:struct_size")))
#else
#define SK_OUT
#define SK_OWNED
#define SK_HANDLE
#define SK_HANDLE_DESTROY(symbol)
#define SK_IN_ARRAY(count)
#define SK_OUT_ARRAY(count)
#define SK_STRUCT_SIZE
#endif

#if defined(_WIN32)
#if defined(SK_STATIC)
#define SK_API
#elif defined(SK_BUILDING_LIBRARY)
#define SK_API __declspec(dllexport)
#else
#define SK_API __declspec(dllimport)
#endif
#define SK_CALL __cdecl
#else
#define SK_API __attribute__((visibility("default")))
#define SK_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * StockKit core: material-removal simulation on dexel grids.
 *
 * Units are whatever the caller uses consistently (StockKit uses metres).
 * A grid is a lattice of parallel rays; each ray holds sorted, disjoint
 * material intervals whose ends keep their exact depth along the ray, the
 * outward surface normal of the material there, and the move that made them.
 * Only Z grids are implemented; the axis is a parameter throughout.
 *
 * Functions never throw across this boundary and are not thread-safe for the
 * same stock. Handles are process-local, generation-checked and never zero.
 */

enum { SK_API_VERSION = 1 };

typedef int32_t sk_result;
enum {
    SK_OK = 0,
    SK_ERROR_INVALID_ARGUMENT = -1,
    SK_ERROR_INVALID_HANDLE = -2,
    SK_ERROR_OUT_OF_MEMORY = -3,
    SK_ERROR_LIMIT = -4,
    SK_ERROR_UNSUPPORTED = -5,
    SK_ERROR_INTERNAL = -6
};

enum { SK_AXIS_X = 0, SK_AXIS_Y = 1, SK_AXIS_Z = 2 };

/** Endpoint source for material that no move has touched. */
enum { SK_SOURCE_STOCK = 0xFFFFFFFFu };

typedef struct sk_tool_handle { uint32_t id; } sk_tool_handle
    SK_HANDLE SK_HANDLE_DESTROY(sk_tool_destroy);
typedef struct sk_stock_handle { uint32_t id; } sk_stock_handle
    SK_HANDLE SK_HANDLE_DESTROY(sk_stock_destroy);

enum { SK_SEGMENT_LINE = 0, SK_SEGMENT_ARC = 1 };

/**
 * One piece of a tool half-profile, from the tip upwards: r is the distance
 * from the tool axis and z the height above the tip. Arcs are minor arcs
 * about (center_r, center_z). Segments are continuous from (0, 0), heights
 * never decrease, and the solid is closed by a flat top at the last height.
 */
typedef struct sk_profile_segment {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t kind;
    double r0, z0, r1, z1;
    double center_r, center_z;
} sk_profile_segment;

/** Tool info: widest radius and height of the profile. */
typedef struct sk_tool_info {
    uint32_t struct_size SK_STRUCT_SIZE;
    double radius;
    double height;
} sk_tool_info;

/**
 * A lattice of rays along `axis`. For a Z grid, ray (i, j) runs along +Z
 * through (origin[0] + i * spacing, origin[1] + j * spacing). Rays are
 * grouped into square tiles of `tile_size` rays per side (0 picks the
 * default).
 */
typedef struct sk_grid {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t axis;
    double origin[2];
    double spacing;
    uint32_t count[2];
    uint32_t tile_size;
} sk_grid;

typedef struct sk_box {
    uint32_t struct_size SK_STRUCT_SIZE;
    double min[3];
    double max[3];
} sk_box;

enum { SK_MOVE_LINE = 0, SK_MOVE_ARC = 1 };
enum { SK_MOVE_RAPID = 1 };

/**
 * One tool-tip motion with the tool axis along +Z. A line runs from `start`
 * to `end`. An arc turns about the vertical axis through `center` (z is the
 * start height) from `start_angle` by the signed `sweep` (radians,
 * counter-clockwise positive) at `radius`, rising by `rise` over the sweep:
 * a helix when rise is non-zero.
 */
typedef struct sk_move {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t kind;
    uint32_t flags;
    uint32_t source;
    double start[3];
    double end[3];
    double center[3];
    double radius;
    double start_angle;
    double sweep;
    double rise;
} sk_move;

/** A material interval along a ray; normals point out of the material. */
typedef struct sk_interval {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t lo_source;
    uint32_t hi_source;
    double lo;
    double hi;
    float lo_normal[3];
    float hi_normal[3];
} sk_interval;

typedef struct sk_stock_info {
    uint32_t struct_size SK_STRUCT_SIZE;
    sk_grid grid;
    uint64_t interval_count;
    double volume;
    uint64_t bytes;
    /** Totals over every cut so far: rays whose sweep was evaluated, rays
        that lost material, and tiles skipped on their material bounds. */
    uint64_t rays_tested;
    uint64_t rays_changed;
    uint64_t tiles_skipped;
} sk_stock_info;

SK_API uint32_t SK_CALL sk_api_version(void);

/** Validates and stores a tool profile. */
SK_API sk_result SK_CALL sk_tool_create(
    const sk_profile_segment *segments SK_IN_ARRAY(segment_count), uint32_t segment_count,
    sk_tool_handle *out_tool SK_OUT SK_OWNED);
SK_API void SK_CALL sk_tool_destroy(sk_tool_handle tool);
SK_API sk_result SK_CALL sk_tool_get_info(sk_tool_handle tool, sk_tool_info *out_info SK_OUT);

/**
 * Exact material swept by one move of `tool` along the ray through (u, v)
 * on `axis` (Z: u = x, v = y): `out_count` disjoint intervals.
 */
SK_API sk_result SK_CALL sk_sweep_count_ray(sk_tool_handle tool, const sk_move *move,
    uint32_t axis, double u, double v, uint32_t *out_count SK_OUT);

/**
 * The same intervals in increasing order, with endpoint normals pointing
 * into the swept solid (the normals the remaining stock gets there) and
 * sources set to the move's source. Fails with SK_ERROR_LIMIT, writing
 * nothing, when `capacity` is below the count.
 */
SK_API sk_result SK_CALL sk_sweep_read_ray(sk_tool_handle tool, const sk_move *move,
    uint32_t axis, double u, double v,
    sk_interval *out_intervals SK_OUT_ARRAY(capacity), uint32_t capacity);

/** Solid box stock. */
SK_API sk_result SK_CALL sk_stock_create_box(const sk_grid *grid, const sk_box *box,
    sk_stock_handle *out_stock SK_OUT SK_OWNED);

/**
 * Stock from a closed, consistently oriented triangle mesh (outward normals by
 * counter-clockwise winding). `positions` holds xyz triples and `indices`
 * vertex triples. Rays that cross edges or vertices are counted exactly once.
 */
SK_API sk_result SK_CALL sk_stock_create_mesh(const sk_grid *grid,
    const double *positions SK_IN_ARRAY(position_count), uint32_t position_count,
    const uint32_t *indices SK_IN_ARRAY(index_count), uint32_t index_count,
    sk_stock_handle *out_stock SK_OUT SK_OWNED);

SK_API void SK_CALL sk_stock_destroy(sk_stock_handle stock);
SK_API sk_result SK_CALL sk_stock_get_info(sk_stock_handle stock, sk_stock_info *out_info SK_OUT);

/**
 * Removes the material swept by each move of `tool`, in order, and writes
 * the volume each move removed (each ray stands for spacing^2 of area).
 * `removed_capacity` must be at least `move_count`. Every move is validated
 * before any is cut.
 */
SK_API sk_result SK_CALL sk_stock_cut(sk_stock_handle stock, sk_tool_handle tool,
    const sk_move *moves SK_IN_ARRAY(move_count), uint32_t move_count,
    double *out_removed SK_OUT_ARRAY(removed_capacity), uint32_t removed_capacity);

/**
 * Interval count of each ray in the block [i0, i0 + ni) x [j0, j0 + nj), i
 * fastest; `count_capacity` must be at least ni * nj.
 */
SK_API sk_result SK_CALL sk_stock_read_counts(sk_stock_handle stock,
    uint32_t i0, uint32_t j0, uint32_t ni, uint32_t nj,
    uint32_t *out_counts SK_OUT_ARRAY(count_capacity), uint32_t count_capacity);

/** Number of intervals in the same block. */
SK_API sk_result SK_CALL sk_stock_count_intervals(sk_stock_handle stock,
    uint32_t i0, uint32_t j0, uint32_t ni, uint32_t nj, uint32_t *out_total SK_OUT);

/**
 * The intervals of the same block, ray by ray, i fastest. Fails with
 * SK_ERROR_LIMIT, writing nothing, when `interval_capacity` is below their
 * number.
 */
SK_API sk_result SK_CALL sk_stock_read_intervals(sk_stock_handle stock,
    uint32_t i0, uint32_t j0, uint32_t ni, uint32_t nj,
    sk_interval *out_intervals SK_OUT_ARRAY(interval_capacity), uint32_t interval_capacity);

#ifdef __cplusplus
}
#endif

#endif
