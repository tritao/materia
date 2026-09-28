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
#define SK_OUT_BUFFER(size) __attribute__((annotate("hxi:out_buffer=" #size)))
#define SK_INOUT __attribute__((annotate("hxi:inout")))
#define SK_INITIAL_CAPACITY(count) __attribute__((annotate("hxi:initial_capacity=" #count)))
#else
#define SK_OUT
#define SK_OWNED
#define SK_HANDLE
#define SK_HANDLE_DESTROY(symbol)
#define SK_IN_ARRAY(count)
#define SK_OUT_ARRAY(count)
#define SK_STRUCT_SIZE
#define SK_OUT_BUFFER(size)
#define SK_INOUT
#define SK_INITIAL_CAPACITY(count)
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
 * StockKit core: material-removal simulation on tri-dexel grids.
 *
 * Units are whatever the caller uses consistently (StockKit uses metres).
 * The stock lives on a lattice with a grid of parallel rays along each of
 * up to three axes; each ray holds sorted, disjoint material intervals whose
 * ends keep their exact depth along the ray, the outward surface normal of
 * the material there, and the move that made them. Every move cuts every
 * grid.
 *
 * Variable-length results come back in one call: the caller passes storage
 * and its capacity through an in/out count, and when the count is too small
 * the function sets it to the number needed, writes no elements and fails
 * with SK_ERROR_LIMIT, so the caller can grow the storage and call again.
 *
 * Functions never throw across this boundary and are not thread-safe for the
 * same stock. Handles are process-local, generation-checked and never zero.
 */

enum { SK_API_VERSION = 2 };

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
/** Axis bits of a lattice: `1 << axis`. */
enum { SK_AXES_X = 1, SK_AXES_Y = 2, SK_AXES_Z = 4, SK_AXES_ALL = 7 };

/** Endpoint source for material that no move has touched. */
enum { SK_SOURCE_STOCK = 0xFFFFFFFFu };
/** No surface to attribute: nothing to report, or the stock is gone from the whole stretch. */
enum { SK_SOURCE_NONE = 0xFFFFFFFEu };

typedef struct sk_tool_handle { uint32_t id; } sk_tool_handle
    SK_HANDLE SK_HANDLE_DESTROY(sk_tool_destroy);
typedef struct sk_stock_handle { uint32_t id; } sk_stock_handle
    SK_HANDLE SK_HANDLE_DESTROY(sk_stock_destroy);
typedef struct sk_snapshot_handle { uint32_t id; } sk_snapshot_handle
    SK_HANDLE SK_HANDLE_DESTROY(sk_snapshot_destroy);
typedef struct sk_mesh_handle { uint32_t id; } sk_mesh_handle
    SK_HANDLE SK_HANDLE_DESTROY(sk_mesh_destroy);

enum { SK_SEGMENT_LINE = 0, SK_SEGMENT_ARC = 1 };
enum { SK_ZONE_CUTTING = 0, SK_ZONE_SHANK = 1, SK_ZONE_HOLDER = 2, SK_ZONE_COUNT = 3 };

/**
 * One piece of a tool half-profile, from the tip upwards: r is the distance
 * from the tool axis and z the height above the tip. Arcs are minor arcs
 * about (center_r, center_z). Segments are continuous from (0, 0), heights
 * never decrease, and the solid is closed by a flat top at the last height.
 * Cutting segments run from the tip; only they remove material. Shank and
 * holder segments above them are checked for contact with the stock.
 */
typedef struct sk_profile_segment {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t kind;
    uint32_t zone;
    double r0, z0, r1, z1;
    double center_r, center_z;
} sk_profile_segment;

/** Tool info: widest radius and height of the cutting zone and of the whole tool. */
typedef struct sk_tool_info {
    uint32_t struct_size SK_STRUCT_SIZE;
    double cutting_radius;
    double cutting_height;
    double radius;
    double height;
} sk_tool_info;

/**
 * What one move did: the volume it removed and, per zone, the volume of the
 * stock left after its cut that the tool's shank and holder overlapped
 * (`contact[SK_ZONE_CUTTING]` is always zero). Each ray stands for spacing^2
 * of area.
 */
typedef struct sk_move_result {
    uint32_t struct_size SK_STRUCT_SIZE;
    double removed;
    double contact[SK_ZONE_COUNT];
} sk_move_result;

/**
 * A lattice of nodes (i, j, k) at origin + (i, j, k) * spacing, for i below
 * count[0], j below count[1] and k below count[2]. Each axis in `axes` gets
 * a grid of rays through the nodes, indexed by the other two axes in
 * increasing order: Z ray (i, j) runs along +Z through (x_i, y_j), X ray (j,
 * k) along +X through (y_j, z_k), and Y ray (i, k) along +Y through (x_i,
 * z_k). The Z grid is always present. Rays are grouped into square tiles of
 * `tile_size` rays per side (0 picks the default). Keep rays off the stock's
 * faces (a cell-centred lattice with a margin), since a ray lying in a face
 * sees it edge-on.
 */
typedef struct sk_lattice {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t axes;
    double origin[3];
    double spacing;
    uint32_t count[3];
    uint32_t tile_size;
} sk_lattice;

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

/** One grid of a stock: its rays (u fastest) and tiles, intervals and volume. */
typedef struct sk_grid_info {
    uint32_t struct_size SK_STRUCT_SIZE;
    /** Zero when the lattice has no grid along this axis. */
    uint32_t present;
    uint32_t count[2];
    uint32_t tiles[2];
    uint64_t interval_count;
    /** Material length over every ray times spacing^2: the stock volume as this grid sees it. */
    double volume;
} sk_grid_info;

typedef struct sk_stock_info {
    uint32_t struct_size SK_STRUCT_SIZE;
    sk_lattice lattice;
    /** The Z grid's volume. */
    double volume;
    uint64_t bytes;
    /** Totals over every cut so far and every grid: rays whose sweep was
        evaluated, rays that lost material, and tiles skipped on their
        material bounds. */
    uint64_t rays_tested;
    uint64_t rays_changed;
    uint64_t tiles_skipped;
    /** Threads `sk_stock_cut` uses; 0 means one per hardware thread. */
    uint32_t threads;
    /** Per axis, X, Y and Z. */
    sk_grid_info grids[3];
} sk_stock_info;

/** What one `sk_stock_cut` call did in total, over its moves and every grid. */
typedef struct sk_cut_summary {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t moves;
    double removed;
    double contact[SK_ZONE_COUNT];
    uint64_t rays_tested;
    uint64_t rays_changed;
    uint64_t tiles_skipped;
} sk_cut_summary;

SK_API uint32_t SK_CALL sk_api_version(void);

/** Validates and stores a tool profile. */
SK_API sk_result SK_CALL sk_tool_create(
    const sk_profile_segment *segments SK_IN_ARRAY(segment_count), uint32_t segment_count,
    sk_tool_handle *out_tool SK_OUT SK_OWNED);
SK_API void SK_CALL sk_tool_destroy(sk_tool_handle tool);
SK_API sk_result SK_CALL sk_tool_get_info(sk_tool_handle tool, sk_tool_info *out_info SK_OUT);

/**
 * Exact material swept by one move of `tool`'s cutting zone along the ray through (u, v)
 * on `axis` (Z: along +Z through (x, y) = (u, v); X: along +X through (y, z) = (u, v);
 * Y: along +Y through (x, z) = (u, v)): disjoint intervals in increasing order, with
 * endpoint normals pointing into the swept solid (the normals the remaining stock gets
 * there) and sources set to the move's source. `interval_count` is the capacity in and
 * the number of intervals out.
 */
SK_API sk_result SK_CALL sk_sweep_ray(sk_tool_handle tool, const sk_move *move,
    uint32_t axis, double u, double v,
    sk_interval *out_intervals SK_OUT_ARRAY(interval_count) SK_INITIAL_CAPACITY(8),
    uint32_t *interval_count SK_INOUT);

/** Solid box stock. */
SK_API sk_result SK_CALL sk_stock_create_box(const sk_lattice *lattice, const sk_box *box,
    sk_stock_handle *out_stock SK_OUT SK_OWNED);

/**
 * Stock from a closed, consistently oriented triangle mesh (outward normals by
 * counter-clockwise winding). `positions` holds xyz triples and `indices`
 * vertex triples. Rays that cross edges or vertices are counted exactly once.
 */
SK_API sk_result SK_CALL sk_stock_create_mesh(const sk_lattice *lattice,
    const double *positions SK_IN_ARRAY(position_count), uint32_t position_count,
    const uint32_t *indices SK_IN_ARRAY(index_count), uint32_t index_count,
    sk_stock_handle *out_stock SK_OUT SK_OWNED);

SK_API void SK_CALL sk_stock_destroy(sk_stock_handle stock);
SK_API sk_result SK_CALL sk_stock_get_info(sk_stock_handle stock, sk_stock_info *out_info SK_OUT);

/**
 * Each tile's revision in the grid along `axis`, row by row (`tiles[0]` per
 * row). A revision changes whenever the tile's rays change, by a cut or a
 * restore, so a preview remeshes only tiles whose revision it has not seen.
 * A Z tile's revision also changes when X or Y rays change within its
 * footprint in x and y, which a contoured mesh of it reads.
 * `revision_capacity` must be at least the grid's tile count.
 */
SK_API sk_result SK_CALL sk_stock_read_revisions(sk_stock_handle stock, uint32_t axis,
    uint64_t *out_revisions SK_OUT_ARRAY(revision_capacity), uint32_t revision_capacity);

/**
 * Captures the stock as it is. Tiles are shared with the stock until either
 * changes them, so a snapshot costs a pointer per tile plus the tiles later
 * cuts replace. Use snapshots every few thousand moves to scrub a program:
 * restore the nearest earlier one and cut forward.
 */
SK_API sk_result SK_CALL sk_stock_snapshot(sk_stock_handle stock,
    sk_snapshot_handle *out_snapshot SK_OUT SK_OWNED);
SK_API void SK_CALL sk_snapshot_destroy(sk_snapshot_handle snapshot);

/**
 * Returns the stock to `snapshot`, which may come from this stock or another
 * with the same lattice. Only tiles that differ change revision.
 */
SK_API sk_result SK_CALL sk_stock_restore(sk_stock_handle stock, sk_snapshot_handle snapshot);

/**
 * Sets how many threads `sk_stock_cut` uses: 0 (the default) means one per
 * hardware thread, 1 cuts on the calling thread. Tiles are owned by one
 * thread each and every ray sees its moves in order, so the stock and the
 * removed volumes are bit-identical for any thread count.
 */
SK_API sk_result SK_CALL sk_stock_set_threads(sk_stock_handle stock, uint32_t threads);

/**
 * Removes the material swept by each move of `tool`'s cutting zone from
 * every grid, in order, and writes what each move did (`move_count`
 * results, measured on the Z grid) and what the call did in total. Every
 * move is validated before any is cut.
 *
 * Contact is measured against the stock left after the move's own cut, so a
 * shank reaching material the flutes remove later in the same move (possible
 * only while the tool climbs) is not reported.
 */
SK_API sk_result SK_CALL sk_stock_cut(sk_stock_handle stock, sk_tool_handle tool,
    const sk_move *moves SK_IN_ARRAY(move_count), uint32_t move_count,
    sk_move_result *out_results SK_OUT_ARRAY(move_count), sk_cut_summary *out_summary SK_OUT);

/**
 * The rays of the grid along `axis` in the block [i0, i0 + ni) x [j0, j0 +
 * nj), i fastest: each ray's interval count, then all their intervals ray
 * by ray. `count_capacity` must be at least ni * nj; `interval_count` is the
 * interval capacity in and the number of intervals out.
 */
SK_API sk_result SK_CALL sk_stock_read_rays(sk_stock_handle stock, uint32_t axis,
    uint32_t i0, uint32_t j0, uint32_t ni, uint32_t nj,
    uint32_t *out_counts SK_OUT_ARRAY(count_capacity), uint32_t count_capacity,
    sk_interval *out_intervals SK_OUT_ARRAY(interval_count) SK_INITIAL_CAPACITY(64),
    uint32_t *interval_count SK_INOUT);

/**
 * One ray of the stock compared with a target part on the same grid: the
 * length of stock outside the target (leftover) and of target missing from
 * the stock (gouge), with the largest stretch of each. `gouge_source` is the
 * source of the stock surface bounding the largest gouge (the move that cut
 * too deep there), or SK_SOURCE_NONE.
 */
typedef struct sk_ray_comparison {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t gouge_source;
    double leftover;
    double gouge;
    double largest_leftover;
    double largest_gouge;
} sk_ray_comparison;

/**
 * Compares the rays of the grid along `axis` in the block [i0, i0 + ni) x
 * [j0, j0 + nj), i fastest, with `target`, which must have the same lattice
 * (for example a stock cast from the finished part's mesh).
 * `comparison_capacity` must be at least ni * nj.
 */
SK_API sk_result SK_CALL sk_stock_compare(sk_stock_handle stock, sk_stock_handle target, uint32_t axis,
    uint32_t i0, uint32_t j0, uint32_t ni, uint32_t nj,
    sk_ray_comparison *out_comparisons SK_OUT_ARRAY(comparison_capacity), uint32_t comparison_capacity);

enum { SK_MESH_BOTTOMS = 1, SK_MESH_MERGE = 2, SK_MESH_CONTOUR = 4 };

typedef struct sk_mesh_info {
    uint32_t struct_size SK_STRUCT_SIZE;
    uint32_t vertex_count;
    uint32_t triangle_count;
    /** Non-zero when faces were merged; per-ray colouring then needs an unmerged mesh. */
    uint32_t merged;
    /**
     * Contoured meshes: crossing edges whose ray had no surface near the
     * edge, drawn at the edge's middle facing along it. Non-zero where the
     * lattice cuts through the stock, else a sign that the grids disagree.
     */
    uint32_t unmatched_edges;
} sk_mesh_info;

/**
 * A display mesh of the stock over the Z grid's tiles [tile_x, tile_x +
 * tiles_x) x [tile_y, tile_y + tiles_y). Quads share no vertices.
 *
 * By default each Z ray stands for a square column spacing wide: every
 * interval gets a top face at its exact depth with its stored normal and
 * source, walls stand where neighbouring columns differ, and SK_MESH_BOTTOMS
 * adds bottom faces, which close the mesh. SK_MESH_MERGE joins equal faces
 * along rows.
 *
 * SK_MESH_CONTOUR (alone; the stock needs all three grids) dual-contours the
 * lattice instead, giving a closed surface through the stock's full depth.
 * Nodes are inside or outside by their Z rays; each edge between nodes of
 * opposite sign crosses the surface where the ray along it has an interval
 * end, with that end's normal and source. Each cell gets one vertex, the
 * least-squares meeting point of its crossings' planes clamped to the cell,
 * so flat faces stay flat and edges and corners stay sharp; each crossing
 * edge becomes a quad with its crossing's normal and source. Features
 * thinner than a cell can be lost.
 *
 * A mesh also reads the rays just past its +x and +y edges: rebuild it when
 * the tiles holding them change too. Z tile revisions also change when X or
 * Y rays in their footprint do.
 */
SK_API sk_result SK_CALL sk_stock_mesh(sk_stock_handle stock, uint32_t tile_x, uint32_t tile_y,
    uint32_t tiles_x, uint32_t tiles_y, uint32_t flags, sk_mesh_handle *out_mesh SK_OUT SK_OWNED);
SK_API void SK_CALL sk_mesh_destroy(sk_mesh_handle mesh);
SK_API sk_result SK_CALL sk_mesh_get_info(sk_mesh_handle mesh, sk_mesh_info *out_info SK_OUT);

/**
 * Mesh streams as bytes: positions and normals as float xyz per vertex,
 * indices as uint32 triangles, colours as RGBA8 per vertex (after colouring;
 * opaque white before), and the source move of each triangle as uint32, for
 * picking. Pass a null output to read the size.
 */
SK_API sk_result SK_CALL sk_mesh_copy_positions(sk_mesh_handle mesh,
    uint8_t *output SK_OUT_BUFFER(byte_capacity), uint32_t *byte_capacity SK_INOUT);
SK_API sk_result SK_CALL sk_mesh_copy_normals(sk_mesh_handle mesh,
    uint8_t *output SK_OUT_BUFFER(byte_capacity), uint32_t *byte_capacity SK_INOUT);
SK_API sk_result SK_CALL sk_mesh_copy_indices(sk_mesh_handle mesh,
    uint8_t *output SK_OUT_BUFFER(byte_capacity), uint32_t *byte_capacity SK_INOUT);
SK_API sk_result SK_CALL sk_mesh_copy_colors(sk_mesh_handle mesh,
    uint8_t *output SK_OUT_BUFFER(byte_capacity), uint32_t *byte_capacity SK_INOUT);
SK_API sk_result SK_CALL sk_mesh_copy_triangle_sources(sk_mesh_handle mesh,
    uint8_t *output SK_OUT_BUFFER(byte_capacity), uint32_t *byte_capacity SK_INOUT);

/**
 * Colours each quad by its source move: `palette[source]` (RGBA as 0xRRGGBBAA)
 * for sources inside the palette, `original` for untouched stock and
 * `fallback` for any other source. Colour by operation with a palette that
 * maps each move to its operation's colour.
 */
SK_API sk_result SK_CALL sk_mesh_color_by_source(sk_mesh_handle mesh,
    const uint32_t *palette SK_IN_ARRAY(palette_count), uint32_t palette_count,
    uint32_t original, uint32_t fallback);

/**
 * Colours each quad made from a ray of the grid along `axis` by that ray:
 * `ray_colors[j * count_u + i]`, for example a deviation map from
 * `sk_stock_compare` on the same grid. Column meshes are made from Z rays
 * only. A contoured quad is made from the ray along its edge: floors and
 * ceilings mostly from Z rays, walls from X and Y rays, so colour with each
 * grid's comparison to see gouges in walls. Needs a mesh built without
 * SK_MESH_MERGE and a colour for every ray of the grid.
 */
SK_API sk_result SK_CALL sk_mesh_color_by_ray(sk_mesh_handle mesh, uint32_t axis,
    const uint32_t *ray_colors SK_IN_ARRAY(ray_count), uint32_t ray_count);

/**
 * Colours each quad of a contoured mesh by how the stock it was built from
 * deviates from `target` (a stock on the same lattice, for example cast from
 * the finished part) right at the quad's surface. Along the quad's ray, the
 * gouge is the target missing from the stock that starts at the surface on
 * its empty side, and the leftover is stock outside the target that ends
 * there on its material side. Quads whose gouge exceeds `tolerance` get
 * `gouge`, else those whose leftover does get `leftover`, else `on_target`
 * (colours as 0xRRGGBBAA). A wall cut too deep shows on the X or Y rays
 * crossing it, a floor on the Z rays, and the stock's other surfaces on the
 * same rays stay unaffected. Colour before cutting the stock further. Needs
 * a mesh built with SK_MESH_CONTOUR.
 */
SK_API sk_result SK_CALL sk_mesh_color_by_deviation(sk_mesh_handle mesh, sk_stock_handle target,
    double tolerance, uint32_t on_target, uint32_t leftover, uint32_t gouge);

#ifdef __cplusplus
}
#endif

#endif
