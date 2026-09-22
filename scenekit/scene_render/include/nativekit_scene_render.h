#ifndef NATIVEKIT_SCENE_RENDER_H
#define NATIVEKIT_SCENE_RENDER_H

/* ------------------------------------------------------------------------- */
/* Dependencies                                                              */
/* ------------------------------------------------------------------------- */

#include "nativekit_scene.h"
#include "nativekit_gpu.h"

/* ------------------------------------------------------------------------- */
/* Export visibility                                                         */
/* ------------------------------------------------------------------------- */

#if defined(_WIN32)
#if defined(NK_STATIC)
#define NKSRENDER_API
#elif defined(NKSRENDER_BUILDING_LIBRARY)
#define NKSRENDER_API __declspec(dllexport)
#else
#define NKSRENDER_API __declspec(dllimport)
#endif
#else
#define NKSRENDER_API __attribute__((visibility("default")))
#endif

/* ------------------------------------------------------------------------- */
/* C linkage                                                                 */
/* ------------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Runtime handles                                                           */
/* ------------------------------------------------------------------------- */

typedef uint32_t nkscene_render_plan NK_HANDLE NK_HANDLE_DESTROY(nkscene_render_plan_destroy);
typedef uint32_t
    nkscene_render_executor NK_HANDLE NK_HANDLE_DESTROY(nkscene_render_executor_destroy);
typedef uint32_t
    nkscene_render_spatial_index NK_HANDLE NK_HANDLE_DESTROY(nkscene_render_spatial_index_destroy);
typedef uint32_t
    nkscene_render_pick_request NK_HANDLE NK_HANDLE_DESTROY(nkscene_render_pick_request_destroy);

/* ------------------------------------------------------------------------- */
/* View configuration                                                        */
/* ------------------------------------------------------------------------- */

typedef struct nkscene_render_visibility_override {
    nkscene_occurrence_id occurrence;
    uint32_t visible NK_BOOL32;
} nkscene_render_visibility_override;

typedef struct nkscene_render_material_override {
    nkscene_occurrence_id occurrence;
    nkscene_material_id material;
} nkscene_render_material_override;

typedef struct nkscene_render_source_visibility_override {
    nkscene_entity_id source;
    uint32_t visible NK_BOOL32;
} nkscene_render_source_visibility_override;

typedef struct nkscene_render_source_material_override {
    nkscene_entity_id source;
    nkscene_material_id material;
} nkscene_render_source_material_override;

typedef struct nkscene_render_clip_plane {
    float normal[3];
    float distance;
    uint32_t enabled NK_BOOL32;
} nkscene_render_clip_plane;

typedef struct nkscene_render_camera {
    uint32_t enabled NK_BOOL32;
    nkscene_transform view_projection;
} nkscene_render_camera;

typedef struct nkscene_render_view {
    uint32_t struct_size NK_STRUCT_SIZE;
    nkscene_occurrence_id root;
    uint32_t include_invisible NK_BOOL32;
    const nkscene_render_visibility_override *
        visibility_overrides NK_BORROWED_ARRAY(visibility_override_count);
    uint32_t visibility_override_count;
    const nkscene_render_material_override *
        material_overrides NK_BORROWED_ARRAY(material_override_count);
    uint32_t material_override_count;
    nkscene_render_camera camera;
    const nkscene_render_clip_plane *clip_planes NK_BORROWED_ARRAY(clip_plane_count);
    uint32_t clip_plane_count;
    const nkscene_render_material_override *
        selection_overrides NK_BORROWED_ARRAY(selection_override_count);
    uint32_t selection_override_count;
    const nkscene_render_material_override *hover_overrides NK_BORROWED_ARRAY(hover_override_count);
    uint32_t hover_override_count;
    const nkscene_entity_id *isolated_sources NK_BORROWED_ARRAY(isolated_source_count);
    uint32_t isolated_source_count;
    const nkscene_render_source_visibility_override *
        source_visibility_overrides NK_BORROWED_ARRAY(source_visibility_override_count);
    uint32_t source_visibility_override_count;
    const nkscene_render_source_material_override *
        source_material_overrides NK_BORROWED_ARRAY(source_material_override_count);
    uint32_t source_material_override_count;
    const nkscene_occurrence_id *isolated_occurrences NK_BORROWED_ARRAY(isolated_occurrence_count);
    uint32_t isolated_occurrence_count;
    /** Optional scene camera occurrence used when camera.enabled is zero. */
    nkscene_occurrence_id camera_occurrence;
} nkscene_render_view;

/* ------------------------------------------------------------------------- */
/* Render state                                                              */
/* ------------------------------------------------------------------------- */

typedef struct nkscene_render_update {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t plan_rebuilt NK_BOOL32;
    uint32_t geometry_rebuilt NK_BOOL32;
    uint64_t patched_instances;
    uint64_t patched_visibility;
    uint64_t patched_materials;
    uint64_t rebuilt_batches;
    uint64_t updated_geometry_resources;
    uint64_t updated_material_resources;
    uint64_t invalidated_items;
    uint64_t patched_culling;
    uint64_t visible_items;
    uint64_t culled_items;
} nkscene_render_update;

/* ------------------------------------------------------------------------- */
/* Spatial queries and picking                                               */
/* ------------------------------------------------------------------------- */

typedef struct nkscene_render_pick_result {
    nkscene_occurrence_id occurrence;
    nkscene_entity_id source;
    uint32_t subelement;
    float world_position[3];
    float depth;
} nkscene_render_pick_result;

typedef struct nkscene_render_ray {
    float origin[3];
    float direction[3];
} nkscene_render_ray;

typedef struct nkscene_render_spatial_occurrence {
    nkscene_occurrence_id occurrence;
} nkscene_render_spatial_occurrence;

enum {
    NKS_RENDER_PICK_PENDING = 1,
    NKS_RENDER_PICK_READY = 2,
    NKS_RENDER_PICK_STALE = 3,
    NKS_RENDER_PICK_FAILED = 4
};

/* ------------------------------------------------------------------------- */
/* Execution state                                                           */
/* ------------------------------------------------------------------------- */

typedef struct nkscene_render_execution_stats {
    uint32_t struct_size NK_STRUCT_SIZE;
    nkgpu_result result;
    uint64_t geometry_resources_created;
    uint64_t geometry_resources_updated;
    uint64_t material_resources_created;
    uint64_t material_resources_updated;
    uint64_t instance_buffers_created;
    uint64_t instance_records_updated;
    uint64_t commands;
    uint64_t draw_calls;
} nkscene_render_execution_stats;

/* ------------------------------------------------------------------------- */
/* Render plan                                                               */
/* ------------------------------------------------------------------------- */

NKSRENDER_API nkscene_result NKS_CALL
nkscene_render_plan_compile(nkscene_snapshot snapshot, const nkscene_render_view *view,
                            nkscene_render_plan *out_plan NK_OUT NK_OWNED);
NKSRENDER_API void NKS_CALL nkscene_render_plan_destroy(nkscene_render_plan plan);
NKSRENDER_API nkscene_result NKS_CALL
nkscene_render_plan_get_item_count(nkscene_render_plan plan, uint64_t *out_count NK_OUT);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_plan_update(
    nkscene_render_plan plan, nkscene_snapshot snapshot, nkscene_change_set changes,
    const nkscene_render_view *view, nkscene_render_update *out_update NK_INOUT);
/** Refreshes a plan when no scene ChangeSet is available. */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_plan_refresh(
    nkscene_render_plan plan, nkscene_snapshot snapshot, const nkscene_render_view *view,
    nkscene_render_update *out_update NK_INOUT);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_plan_pick(
    nkscene_render_plan plan, nkscene_snapshot snapshot, uint32_t primitive,
    const float world_position[3], float depth, nkscene_render_pick_result *out_result NK_OUT);

/* ------------------------------------------------------------------------- */
/* Spatial index                                                             */
/* ------------------------------------------------------------------------- */

/** Builds a read-only spatial index for one immutable scene snapshot. */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_spatial_index_create(
    nkscene_snapshot snapshot, nkscene_render_spatial_index *out_index NK_OUT NK_OWNED);
NKSRENDER_API void NKS_CALL
nkscene_render_spatial_index_destroy(nkscene_render_spatial_index index);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_spatial_index_get_revision(
    nkscene_render_spatial_index index, uint64_t *out_revision NK_OUT);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_spatial_index_query_bounds(
    nkscene_render_spatial_index index, const nkscene_bounds *bounds, uint64_t *out_count NK_OUT);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_spatial_index_query_ray(
    nkscene_render_spatial_index index, const nkscene_render_ray *ray, uint64_t *out_count NK_OUT);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_spatial_index_get_occurrence(
    nkscene_render_spatial_index index, uint64_t result_index,
    nkscene_render_spatial_occurrence *out_result NK_OUT);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_spatial_index_pick_ray(
    nkscene_render_spatial_index index, const nkscene_render_ray *ray,
    nkscene_render_pick_result *out_result NK_OUT);
/** Performs nearest-hit picking for a batch of rays against one immutable snapshot.
 * `rays` and `out_results` must each reference `ray_count` elements.
 */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_spatial_index_pick_rays(
    nkscene_render_spatial_index index,
    const nkscene_render_ray *rays NK_IN_ARRAY(ray_count), uint64_t ray_count,
    nkscene_render_pick_result *out_results NK_OUT_ARRAY(ray_count));

/* ------------------------------------------------------------------------- */
/* GPU executor                                                              */
/* ------------------------------------------------------------------------- */

/**
 * Creates an executor bound to a GPU renderer. Pass a zero renderer for the
 * headless resource and command path. The renderer must outlive the executor.
 */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_executor_create(
    nkgpu_renderer renderer, nkscene_render_executor *out_executor NK_OUT NK_OWNED);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_executor_create_from_renderer_id(
    uint32_t renderer_id, nkscene_render_executor *out_executor NK_OUT NK_OWNED);
NKSRENDER_API void NKS_CALL nkscene_render_executor_destroy(nkscene_render_executor executor);
/** Executes a plan and reports resource, instance, command, and draw counters. */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_executor_execute(
    nkscene_render_executor executor, nkscene_render_plan plan, nkscene_snapshot snapshot,
    nkscene_render_execution_stats *out_stats NK_OUT);
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_executor_get_last_result(
    nkscene_render_executor executor, nkgpu_result *out_result NK_OUT);
/** Renders into an offscreen RGBA8 target and copies tightly packed pixels. */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_executor_capture_rgba8(
    nkscene_render_executor executor, nkscene_render_plan plan, nkscene_snapshot snapshot,
    uint32_t width, uint32_t height, float clear_red, float clear_green, float clear_blue,
    float clear_alpha, uint8_t *pixels, uint64_t pixel_size);
/** Runs the GPU ID pass and resolves one pixel to scene ownership. */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_executor_pick_pixel(
    nkscene_render_executor executor, nkscene_render_plan plan, nkscene_snapshot snapshot,
    uint32_t width, uint32_t height, uint32_t x, uint32_t y,
    nkscene_render_pick_result *out_result NK_OUT);
/** Starts an asynchronous GPU ID pass and pixel readback. */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_executor_pick_pixel_begin(
    nkscene_render_executor executor, nkscene_render_plan plan, nkscene_snapshot snapshot,
    uint32_t width, uint32_t height, uint32_t x, uint32_t y,
    nkscene_render_pick_request *out_request NK_OUT NK_OWNED);
NKSRENDER_API void NKS_CALL
nkscene_render_pick_request_destroy(nkscene_render_pick_request request);
/** Polls an asynchronous pick without blocking. A ready result is valid only for the
 * supplied current plan and snapshot; otherwise the state is stale. */
NKSRENDER_API nkscene_result NKS_CALL nkscene_render_executor_pick_pixel_poll(
    nkscene_render_executor executor, nkscene_render_pick_request request,
    nkscene_render_plan current_plan, nkscene_snapshot current_snapshot, uint32_t *out_state NK_OUT,
    nkgpu_result *out_error NK_OUT, nkscene_render_pick_result *out_result NK_OUT);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* NATIVEKIT_SCENE_RENDER_H */
