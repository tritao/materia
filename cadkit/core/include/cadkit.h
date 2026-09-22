#pragma once

#include <stdint.h>

/* Haxeon reads these Clang annotations when importing this header. Other
 * compilers see an empty ABI-neutral definition. */
#if defined(__clang__)
#  define CADKIT_HXI_ANNOTATE(name) __attribute__((annotate(name)))
#  define CADKIT_HXI_HANDLE __attribute__((annotate("hxi:handle")))
#  define CADKIT_HXI_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#  define CADKIT_HXI_OUT __attribute__((annotate("hxi:out")))
#  define CADKIT_HXI_OUT_ARRAY(count) __attribute__((annotate("hxi:out_array")))
#  define CADKIT_HXI_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#  define CADKIT_HXI_OUT_BUFFER(size) __attribute__((annotate("hxi:out_buffer=" #size)))
#  define CADKIT_HXI_INOUT __attribute__((annotate("hxi:inout")))
#  define CADKIT_HXI_OWNED __attribute__((annotate("hxi:owned")))
#  define CADKIT_HXI_UTF8 __attribute__((annotate("hxi:utf8")))
#  define CADKIT_HXI_RETURNS_BORROWED_UTF8 \
    __attribute__((annotate("hxi:returns_borrowed_utf8")))
#else
#  define CADKIT_HXI_HANDLE
#  define CADKIT_HXI_HANDLE_DESTROY(symbol)
#  define CADKIT_HXI_OUT
#  define CADKIT_HXI_OUT_ARRAY(count)
#  define CADKIT_HXI_IN_ARRAY(count)
#  define CADKIT_HXI_OUT_BUFFER(size)
#  define CADKIT_HXI_INOUT
#  define CADKIT_HXI_OWNED
#  define CADKIT_HXI_UTF8
#  define CADKIT_HXI_RETURNS_BORROWED_UTF8
#endif

#if defined(_WIN32) && defined(CADKIT_SHARED)
#  if defined(cadkit_core_EXPORTS)
#    define CADKIT_API __declspec(dllexport)
#  else
#    define CADKIT_API __declspec(dllimport)
#  endif
#else
#  define CADKIT_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum cad_result {
    CAD_OK = 0,
    CAD_ERROR_INVALID_HANDLE,
    CAD_ERROR_INVALID_ARGUMENT,
    CAD_ERROR_OPERATION_FAILED,
    CAD_ERROR_OUT_OF_MEMORY,
    CAD_ERROR_BUFFER_TOO_SMALL
} cad_result;

typedef struct cad_vec3 {
    double x;
    double y;
    double z;
} cad_vec3;

typedef struct cad_bounds {
    cad_vec3 min;
    cad_vec3 max;
} cad_bounds;

/* Zero is never a valid handle. Handles are generation checked by the core. */
typedef uint32_t cad_shape CADKIT_HXI_HANDLE CADKIT_HXI_HANDLE_DESTROY(cad_shape_destroy);
typedef uint32_t cad_mesh CADKIT_HXI_HANDLE CADKIT_HXI_HANDLE_DESTROY(cad_mesh_destroy);
typedef uint32_t cad_operation CADKIT_HXI_HANDLE CADKIT_HXI_HANDLE_DESTROY(cad_operation_destroy);

/* Haxeon projects counted input arrays of fixed-layout structures. Keeping
 * the handle in a one-field record preserves a bulk edge-list ABI while
 * making the list available to generated language bindings. */
typedef struct cad_shape_ref {
    cad_shape shape;
} cad_shape_ref;

typedef enum cad_shape_kind {
    CAD_SHAPE_UNKNOWN = 0,
    CAD_SHAPE_COMPOUND,
    CAD_SHAPE_COMPSOLID,
    CAD_SHAPE_SOLID,
    CAD_SHAPE_SHELL,
    CAD_SHAPE_FACE,
    CAD_SHAPE_WIRE,
    CAD_SHAPE_EDGE,
    CAD_SHAPE_VERTEX
} cad_shape_kind;

typedef enum cad_surface_kind {
    CAD_SURFACE_UNKNOWN = 0,
    CAD_SURFACE_PLANE,
    CAD_SURFACE_CYLINDER,
    CAD_SURFACE_CONE,
    CAD_SURFACE_SPHERE,
    CAD_SURFACE_TORUS,
    CAD_SURFACE_BEZIER,
    CAD_SURFACE_BSPLINE
} cad_surface_kind;

typedef enum cad_curve_kind {
    CAD_CURVE_UNKNOWN = 0,
    CAD_CURVE_LINE,
    CAD_CURVE_CIRCLE,
    CAD_CURVE_ELLIPSE,
    CAD_CURVE_HYPERBOLA,
    CAD_CURVE_PARABOLA,
    CAD_CURVE_BEZIER,
    CAD_CURVE_BSPLINE,
    CAD_CURVE_OFFSET
} cad_curve_kind;

typedef enum cad_history_relation {
    CAD_HISTORY_GENERATED = 0,
    CAD_HISTORY_MODIFIED,
    CAD_HISTORY_DELETED
} cad_history_relation;

typedef struct cad_mesh_options {
    double linear_deflection;
    double angular_deflection;
} cad_mesh_options;

/* Index range for one CAD face in the mesh index stream. face_index uses the
 * same zero-based ordering as cad_shape_subshape_at(..., CAD_SHAPE_FACE, ...).
 */
typedef struct cad_mesh_face_range {
    uint32_t face_index;
    uint32_t first_index;
    uint32_t index_count;
} cad_mesh_face_range;

/* Modeling constructors. Points are world coordinates; angles use radians.
 * Counted arrays are borrowed for the call. All shape outputs are owned.
 * Failures zero the output handle. Wires must be connected; faces planar.
 */
CADKIT_API cad_result cad_line(cad_vec3 start, cad_vec3 end,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_arc(cad_vec3 start, cad_vec3 middle, cad_vec3 end,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_circle(cad_vec3 center, cad_vec3 normal, double radius,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_spline(
    const cad_vec3* points CADKIT_HXI_IN_ARRAY(point_count), uint32_t point_count,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_polyline(
    const cad_vec3* points CADKIT_HXI_IN_ARRAY(point_count), uint32_t point_count,
    uint8_t closed, cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_wire(
    const cad_shape_ref* edges CADKIT_HXI_IN_ARRAY(edge_count), uint32_t edge_count,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
/* Closed coplanar boundaries; hole orientation is normalized automatically. */
CADKIT_API cad_result cad_planar_face(cad_shape outer,
    const cad_shape_ref* holes CADKIT_HXI_IN_ARRAY(hole_count), uint32_t hole_count,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_compound(
    const cad_shape_ref* shapes CADKIT_HXI_IN_ARRAY(shape_count), uint32_t shape_count,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
/* Rigid placement: x and z must be nonzero perpendicular directions. */
CADKIT_API cad_result cad_shape_place(cad_shape shape, cad_vec3 origin,
    cad_vec3 x_direction, cad_vec3 z_direction,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_shape_place_operation(cad_shape shape, cad_vec3 origin,
    cad_vec3 x_direction, cad_vec3 z_direction,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_shape_valid(cad_shape shape,
    uint8_t* out_valid CADKIT_HXI_OUT);
CADKIT_API cad_result cad_loft(
    const cad_shape_ref* wires CADKIT_HXI_IN_ARRAY(wire_count), uint32_t wire_count,
    uint8_t solid, uint8_t ruled,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_loft_operation(
    const cad_shape_ref* wires CADKIT_HXI_IN_ARRAY(wire_count), uint32_t wire_count,
    uint8_t solid, uint8_t ruled,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);
/* Profile is a face or wire; spine is a connected wire. */
CADKIT_API cad_result cad_sweep(cad_shape profile, cad_shape spine,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_sweep_operation(cad_shape profile, cad_shape spine,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);
/* Offset a planar wire with round joins. May return several wires. */
CADKIT_API cad_result cad_wire_offset(cad_shape profile, double distance,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_wire_offset_operation(cad_shape profile, double distance,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);
/* Remove selected faces and offset remaining skin. Negative thickness goes inward. */
CADKIT_API cad_result cad_shell(cad_shape solid,
    const cad_shape_ref* faces CADKIT_HXI_IN_ARRAY(face_count), uint32_t face_count,
    double thickness, cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);
CADKIT_API cad_result cad_shell_operation(cad_shape solid,
    const cad_shape_ref* faces CADKIT_HXI_IN_ARRAY(face_count), uint32_t face_count,
    double thickness, cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);
/* Directional projection of an edge/wire onto a target shape. */
CADKIT_API cad_result cad_project(cad_shape curve, cad_shape target, cad_vec3 direction,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_box(
    double width,
    double depth,
    double height,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_cylinder(
    double radius,
    double height,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_sphere(
    double radius,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* STEP import/export is file-based and deliberately transfers one shape
 * without application metadata or assembly/document semantics. Paths are
 * UTF-8 strings owned by the caller for the duration of the call. */
CADKIT_API cad_result cad_step_import(
    const char* path CADKIT_HXI_UTF8,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_step_export(
    cad_shape shape,
    const char* path CADKIT_HXI_UTF8);

/* Transform functions return a new shape and leave the input shape unchanged. */
CADKIT_API cad_result cad_shape_translate(
    cad_shape shape,
    cad_vec3 delta,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* Rotate around an axis through the origin. Angle is expressed in radians. */
CADKIT_API cad_result cad_shape_rotate(
    cad_shape shape,
    cad_vec3 axis,
    double angle,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_mirror(
    cad_shape shape,
    cad_vec3 plane_origin,
    cad_vec3 plane_normal,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* Creates an independent owning handle for the same OCCT topology. */
CADKIT_API cad_result cad_shape_clone(
    cad_shape shape,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* Sweep a profile by a finite translation vector. */
CADKIT_API cad_result cad_shape_extrude(
    cad_shape profile,
    cad_vec3 delta,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* Revolve a profile around an axis through axis_origin by angle radians. */
CADKIT_API cad_result cad_shape_revolve(
    cad_shape profile,
    cad_vec3 axis_origin,
    cad_vec3 axis_direction,
    double angle,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* Apply a constant fillet or chamfer to every edge of a solid. */
CADKIT_API cad_result cad_shape_fillet(
    cad_shape shape,
    double radius,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_chamfer(
    cad_shape shape,
    double distance,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* Apply a constant fillet or chamfer only to the listed edges. Every edge
 * handle must be a distinct edge belonging to shape. */
CADKIT_API cad_result cad_shape_fillet_edges(
    cad_shape shape,
    const cad_shape_ref* edges CADKIT_HXI_IN_ARRAY(edge_count),
    uint32_t edge_count,
    double radius,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_chamfer_edges(
    cad_shape shape,
    const cad_shape_ref* edges CADKIT_HXI_IN_ARRAY(edge_count),
    uint32_t edge_count,
    double distance,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* Operation variants retain OCCT's topology history alongside the result. */
CADKIT_API cad_result cad_shape_translate_operation(
    cad_shape shape,
    cad_vec3 delta,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_rotate_operation(
    cad_shape shape,
    cad_vec3 axis,
    double angle,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_mirror_operation(
    cad_shape shape,
    cad_vec3 plane_origin,
    cad_vec3 plane_normal,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_extrude_operation(
    cad_shape profile,
    cad_vec3 delta,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_revolve_operation(
    cad_shape profile,
    cad_vec3 axis_origin,
    cad_vec3 axis_direction,
    double angle,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_fillet_operation(
    cad_shape shape,
    double radius,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_chamfer_operation(
    cad_shape shape,
    double distance,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_fillet_edges_operation(
    cad_shape shape,
    const cad_shape_ref* edges CADKIT_HXI_IN_ARRAY(edge_count),
    uint32_t edge_count,
    double radius,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_chamfer_edges_operation(
    cad_shape shape,
    const cad_shape_ref* edges CADKIT_HXI_IN_ARRAY(edge_count),
    uint32_t edge_count,
    double distance,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_fuse(
    cad_shape first,
    cad_shape second,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_cut(
    cad_shape first,
    cad_shape second,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_common(
    cad_shape first,
    cad_shape second,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_fuse_operation(
    cad_shape first,
    cad_shape second,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_cut_operation(
    cad_shape first,
    cad_shape second,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_common_operation(
    cad_shape first,
    cad_shape second,
    cad_operation* out_operation CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_bounds(
    cad_shape shape,
    cad_bounds* out_bounds CADKIT_HXI_OUT);

CADKIT_API cad_result cad_shape_area(
    cad_shape shape,
    double* out_area CADKIT_HXI_OUT);

CADKIT_API cad_result cad_shape_volume(
    cad_shape shape,
    double* out_volume CADKIT_HXI_OUT);

CADKIT_API cad_result cad_shape_kind_get(
    cad_shape shape,
    cad_shape_kind* out_kind CADKIT_HXI_OUT);

CADKIT_API cad_result cad_shape_subshape_count(
    cad_shape shape,
    cad_shape_kind kind,
    uint32_t* out_count CADKIT_HXI_OUT);

/* Haxeon-friendly indexed access; cad_shape_subshapes remains the bulk C API. */
CADKIT_API cad_result cad_shape_subshape_at(
    cad_shape shape,
    cad_shape_kind kind,
    uint32_t index,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_subshapes(
    cad_shape shape,
    cad_shape_kind kind,
    cad_shape* output,
    uint32_t capacity);

/* Topological identity ignores orientation, matching OCCT IsSame(). */
CADKIT_API cad_result cad_shape_is_same(
    cad_shape first,
    cad_shape second,
    uint8_t* out_same CADKIT_HXI_OUT);

CADKIT_API cad_result cad_face_surface_kind(
    cad_shape face,
    cad_surface_kind* out_kind CADKIT_HXI_OUT);

CADKIT_API cad_result cad_face_center(
    cad_shape face,
    cad_vec3* out_center CADKIT_HXI_OUT);

CADKIT_API cad_result cad_face_area(
    cad_shape face,
    double* out_area CADKIT_HXI_OUT);

/* Returns a representative unit normal evaluated at the midpoint of the
 * face's parametric bounds. */
CADKIT_API cad_result cad_face_normal(
    cad_shape face,
    cad_vec3* out_normal CADKIT_HXI_OUT);

CADKIT_API cad_result cad_edge_curve_kind(
    cad_shape edge,
    cad_curve_kind* out_kind CADKIT_HXI_OUT);

CADKIT_API cad_result cad_edge_length(
    cad_shape edge,
    double* out_length CADKIT_HXI_OUT);

/* Returns a unit tangent at normalized edge parameter t in [0, 1]. */
CADKIT_API cad_result cad_edge_tangent_at(
    cad_shape edge,
    double parameter,
    cad_vec3* out_tangent CADKIT_HXI_OUT);

CADKIT_API cad_result cad_vertex_position(
    cad_shape vertex,
    cad_vec3* out_position CADKIT_HXI_OUT);

CADKIT_API cad_result cad_operation_result_shape(
    cad_operation operation,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_operation_history_count(
    cad_operation operation,
    cad_history_relation relation,
    uint32_t* out_count CADKIT_HXI_OUT);

CADKIT_API cad_result cad_operation_history_source_at(
    cad_operation operation,
    cad_history_relation relation,
    uint32_t index,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

/* Generated and modified relations have one target per entry. Deleted
 * relations intentionally have no target and must not be queried here. */
CADKIT_API cad_result cad_operation_history_target_at(
    cad_operation operation,
    cad_history_relation relation,
    uint32_t index,
    cad_shape* out_shape CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_shape_tessellate(
    cad_shape shape,
    const cad_mesh_options* options,
    cad_mesh* out_mesh CADKIT_HXI_OUT CADKIT_HXI_OWNED);

CADKIT_API cad_result cad_mesh_vertex_count(
    cad_mesh mesh,
    uint32_t* out_count CADKIT_HXI_OUT);

CADKIT_API cad_result cad_mesh_index_count(
    cad_mesh mesh,
    uint32_t* out_count CADKIT_HXI_OUT);

CADKIT_API cad_result cad_mesh_face_range_count(
    cad_mesh mesh,
    uint32_t* out_count CADKIT_HXI_OUT);

CADKIT_API cad_result cad_mesh_face_range_at(
    cad_mesh mesh,
    uint32_t index,
    cad_mesh_face_range* out_range CADKIT_HXI_OUT);

CADKIT_API cad_result cad_mesh_copy_face_ranges(
    cad_mesh mesh,
    cad_mesh_face_range* output,
    uint32_t capacity);

/* Copy arrays require capacity in elements, not bytes. */
CADKIT_API cad_result cad_mesh_copy_vertices(
    cad_mesh mesh,
    cad_vec3* output,
    uint32_t capacity);

CADKIT_API cad_result cad_mesh_copy_normals(
    cad_mesh mesh,
    cad_vec3* output,
    uint32_t capacity);

CADKIT_API cad_result cad_mesh_copy_indices(
    cad_mesh mesh,
    uint32_t* output,
    uint32_t capacity);

/* Haxeon-facing byte copies keep each mesh stream bulk-oriented. The byte
 * capacity is both the query result and the input capacity in bytes. */
CADKIT_API cad_result cad_mesh_copy_vertices_bytes(
    cad_mesh mesh,
    uint8_t* output CADKIT_HXI_OUT_BUFFER(byte_capacity),
    uint32_t* byte_capacity CADKIT_HXI_INOUT);

CADKIT_API cad_result cad_mesh_copy_normals_bytes(
    cad_mesh mesh,
    uint8_t* output CADKIT_HXI_OUT_BUFFER(byte_capacity),
    uint32_t* byte_capacity CADKIT_HXI_INOUT);

CADKIT_API cad_result cad_mesh_copy_indices_bytes(
    cad_mesh mesh,
    uint8_t* output CADKIT_HXI_OUT_BUFFER(byte_capacity),
    uint32_t* byte_capacity CADKIT_HXI_INOUT);

CADKIT_API void cad_shape_destroy(cad_shape shape);

CADKIT_API void cad_mesh_destroy(cad_mesh mesh);

CADKIT_API void cad_operation_destroy(cad_operation operation);

CADKIT_API const char* cad_last_error(void) CADKIT_HXI_RETURNS_BORROWED_UTF8;

#ifdef __cplusplus
}
#endif
