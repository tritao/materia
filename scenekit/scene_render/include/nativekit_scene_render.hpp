#pragma once

/* ------------------------------------------------------------------------- */
/* Dependencies                                                              */
/* ------------------------------------------------------------------------- */

#include "nativekit_scene_render.h"
#include "nativekit_scene.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace nkscene {

/* ------------------------------------------------------------------------- */
/* Internal declarations                                                     */
/* ------------------------------------------------------------------------- */

struct SceneView;
class RenderPlan;
class NativeKitGpuExecutor;
struct GpuExecutionStats;
namespace render_internal {
void rebuild_batches(RenderPlan &plan);
void build_items(RenderPlan &plan, const SceneSnapshot &snapshot, const SceneView &view);
std::size_t move_item_batch(RenderPlan &plan, std::size_t item_index, GeometryId geometry,
                            MaterialId material);
void capture_view_policy(RenderPlan &plan, const SceneView &view);
void update_ancestor_index(RenderPlan &plan, const SceneSnapshot &snapshot,
                           const ChangeSet &changes);
} // namespace render_internal

/* ------------------------------------------------------------------------- */
/* View configuration                                                        */
/* ------------------------------------------------------------------------- */

struct VisibilityOverride {
    NodeId node;
    bool visible = true;
};

struct MaterialOverride {
    NodeId node;
    MaterialId material;
};

struct SourceVisibilityOverride {
    EntityId source;
    bool visible = true;
};

struct SourceMaterialOverride {
    EntityId source;
    MaterialId material;
};

/** Declarative, scene-independent presentation filter for one SceneView. */
struct SceneViewFilter {
    /** Source entities retained by isolation, including their ancestors. */
    std::vector<EntityId> isolated_sources;
    /** Source-level visibility rules below explicit node overrides. */
    std::vector<SourceVisibilityOverride> source_visibility_overrides;
    /** Source-level base materials below node and interaction layers. */
    std::vector<SourceMaterialOverride> source_material_overrides;
    /** Explicit nodes retained by isolation, including their subtrees. */
    std::vector<NodeId> isolated_nodes;

    void set_isolated_source(EntityId source, bool isolated) {
        const auto found = std::find(isolated_sources.begin(), isolated_sources.end(), source);
        if (isolated && found == isolated_sources.end())
            isolated_sources.push_back(source);
        else if (!isolated && found != isolated_sources.end())
            isolated_sources.erase(found);
    }

    void set_source_visibility_override(EntityId source, bool visible) {
        const auto found =
            std::find_if(source_visibility_overrides.begin(), source_visibility_overrides.end(),
                         [source](const auto &value) { return value.source == source; });
        if (found != source_visibility_overrides.end())
            found->visible = visible;
        else
            source_visibility_overrides.push_back({source, visible});
    }

    void set_source_material_override(EntityId source, MaterialId material) {
        const auto found =
            std::find_if(source_material_overrides.begin(), source_material_overrides.end(),
                         [source](const auto &value) { return value.source == source; });
        if (found != source_material_overrides.end())
            found->material = material;
        else
            source_material_overrides.push_back({source, material});
    }

    void set_isolated_node(NodeId node, bool isolated) {
        const auto found =
            std::find(isolated_nodes.begin(), isolated_nodes.end(), node);
        if (isolated && found == isolated_nodes.end())
            isolated_nodes.push_back(node);
        else if (!isolated && found != isolated_nodes.end())
            isolated_nodes.erase(found);
    }

    void clear_isolation() noexcept {
        isolated_sources.clear();
        isolated_nodes.clear();
    }

    void clear() noexcept {
        isolated_sources.clear();
        source_visibility_overrides.clear();
        source_material_overrides.clear();
        isolated_nodes.clear();
    }
};

struct ClipPlane {
    std::array<float, 3> normal{0.0f, 0.0f, 1.0f};
    float distance = 0.0f;
    bool enabled = true;
};

struct SceneCamera {
    bool enabled = false;
    std::array<float, 16> view_projection{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                          0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
};

/** Runtime world transform supplied by a presentation without editing the scene. */
struct PoseOverride {
    NodeId node;
    LocalTransform world_transform;
};

struct SceneView {
    /** Invalid means that the view contains every node. */
    NodeId root;
    /** Include scene-hidden nodes as visible for inspection views. */
    bool include_invisible = false;
    /** Later entries replace earlier entries for the same node. */
    std::vector<VisibilityOverride> visibility_overrides;
    /** Base presentation material overrides. */
    std::vector<MaterialOverride> material_overrides;
    /** Selection material layer, below hover material overrides. */
    std::vector<MaterialOverride> selection_material_overrides;
    /** Hover material layer, above selection and base material overrides. */
    std::vector<MaterialOverride> hover_material_overrides;
    /** Declarative source and isolation presentation rules. */
    SceneViewFilter filter;
    /** Optional world-to-clip transform used for bounds culling and rendering. */
    SceneCamera camera;
    /** Optional scene camera node used when camera.enabled is false. */
    NodeId camera_node;
    /** Runtime world poses overlaid on this view. */
    std::vector<PoseOverride> pose_overrides;
    /** Conservative node-level sectioning planes. */
    std::vector<ClipPlane> clip_planes;

    void set_visibility_override(NodeId node, bool visible) {
        const auto found = std::find_if(
            visibility_overrides.begin(), visibility_overrides.end(),
            [node](const auto &value) { return value.node == node; });
        if (found != visibility_overrides.end())
            found->visible = visible;
        else
            visibility_overrides.push_back({node, visible});
    }

    void set_material_override(NodeId node, MaterialId material) {
        set_material_override_in(material_overrides, node, material);
    }

    void set_selection_material_override(NodeId node, MaterialId material) {
        set_material_override_in(selection_material_overrides, node, material);
    }

    void set_hover_material_override(NodeId node, MaterialId material) {
        set_material_override_in(hover_material_overrides, node, material);
    }

    void set_isolated_source(EntityId source, bool isolated) {
        filter.set_isolated_source(source, isolated);
    }

    void set_source_visibility_override(EntityId source, bool visible) {
        filter.set_source_visibility_override(source, visible);
    }

    void set_source_material_override(EntityId source, MaterialId material) {
        filter.set_source_material_override(source, material);
    }

    void set_isolated_node(NodeId node, bool isolated) {
        filter.set_isolated_node(node, isolated);
    }

    void clear_selection_material_overrides() noexcept { selection_material_overrides.clear(); }

    void clear_hover_material_overrides() noexcept { hover_material_overrides.clear(); }

  private:
    static void set_material_override_in(std::vector<MaterialOverride> &overrides,
                                         NodeId node, MaterialId material) {
        const auto found =
            std::find_if(overrides.begin(), overrides.end(), [node](const auto &value) {
                return value.node == node;
            });
        if (found != overrides.end())
            found->material = material;
        else
            overrides.push_back({node, material});
    }
};

/* ------------------------------------------------------------------------- */
/* Render plan data                                                          */
/* ------------------------------------------------------------------------- */

enum class RenderFlags : std::uint32_t {
    None = 0,
    Hidden = 1u << 0,
    Opaque = 1u << 1,
    Culled = 1u << 2
};

constexpr RenderFlags operator|(RenderFlags lhs, RenderFlags rhs) noexcept {
    return static_cast<RenderFlags>(static_cast<std::uint32_t>(lhs) |
                                    static_cast<std::uint32_t>(rhs));
}

constexpr RenderFlags &operator|=(RenderFlags &lhs, RenderFlags rhs) noexcept {
    lhs = lhs | rhs;
    return lhs;
}

constexpr bool has_render_flag(RenderFlags value, RenderFlags flag) noexcept {
    return (static_cast<std::uint32_t>(value) & static_cast<std::uint32_t>(flag)) != 0;
}

struct RenderItem {
    NodeId node;
    GeometryId geometry;
    MaterialId material;
    std::uint32_t pickId = 0;
    std::uint32_t transformIndex = 0;
    RenderFlags flags = RenderFlags::Opaque;
};

struct InstanceBatch {
    GeometryId geometry;
    MaterialId material;
    std::vector<NodeId> instances;
};

struct RenderUpdate {
    bool plan_rebuilt = false;
    bool geometry_rebuilt = false;
    std::size_t patched_instances = 0;
    std::size_t patched_visibility = 0;
    std::size_t patched_materials = 0;
    std::size_t rebuilt_batches = 0;
    std::size_t updated_geometry_resources = 0;
    std::size_t updated_material_resources = 0;
    std::size_t invalidated_items = 0;
    std::size_t patched_culling = 0;
    std::size_t culling_candidates = 0;
    std::size_t visible_items = 0;
    std::size_t culled_items = 0;
};

/* ------------------------------------------------------------------------- */
/* Spatial queries and picking                                               */
/* ------------------------------------------------------------------------- */

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct SubelementId {
    std::uint32_t value = 0;
    constexpr bool valid() const noexcept { return value != 0; }
};

struct PickResult {
    NodeId node;
    EntityId source;
    SubelementId subelement;
    Vec3 worldPosition;
    float depth = 0.0f;
};

class NKSRENDER_API GpuPickRequest {
  public:
    GpuPickRequest() = default;
    ~GpuPickRequest();
    GpuPickRequest(GpuPickRequest &&) noexcept;
    GpuPickRequest &operator=(GpuPickRequest &&) noexcept;
    GpuPickRequest(const GpuPickRequest &) = delete;
    GpuPickRequest &operator=(const GpuPickRequest &) = delete;

  private:
    friend class NativeKitGpuExecutor;
    struct State;
    std::unique_ptr<State> state_;
};

struct Ray {
    Vec3 origin;
    Vec3 direction;
};

class NKSRENDER_API SceneSpatialIndex {
  public:
    explicit SceneSpatialIndex(const SceneSnapshot &snapshot, const SceneView *view = nullptr);
    ~SceneSpatialIndex();
    SceneSpatialIndex(SceneSpatialIndex &&) noexcept;
    SceneSpatialIndex &operator=(SceneSpatialIndex &&) noexcept;
    SceneSpatialIndex(const SceneSpatialIndex &) = delete;
    SceneSpatialIndex &operator=(const SceneSpatialIndex &) = delete;

    std::uint64_t source_revision() const noexcept;
    /** Refit one existing node after a transform-only snapshot update. */
    bool update_node_bounds(const SceneSnapshot &snapshot, NodeId node) noexcept;
    std::span<const NodeId> query_bounds(const Bounds &) const;
    /** Returns snapshot nodes whose bounds intersect all supplied planes. */
    std::span<const NodeId> query_frustum(std::span<const std::array<float, 4>> planes) const;
    std::span<const NodeId> query_ray(const Ray &) const;
    std::size_t query_result_count() const noexcept;
    NodeId query_result(std::size_t index) const noexcept;
    PickResult pick_ray(const Ray &) const;
    std::vector<PickResult> pick_rays(std::span<const Ray>) const;

  private:
    struct State;
    std::unique_ptr<State> state_;
};

/* ------------------------------------------------------------------------- */
/* GPU execution data                                                        */
/* ------------------------------------------------------------------------- */

struct GpuCommand {
    NodeId node;
    GeometryId geometry;
    MaterialId material;
    std::uint32_t transformIndex = 0;
};

struct GpuExecutionStats {
    nkgpu_result result = NKGPU_OK;
    std::size_t geometry_resources_created = 0;
    std::size_t geometry_resources_updated = 0;
    std::size_t material_resources_created = 0;
    std::size_t material_resources_updated = 0;
    std::size_t instance_buffers_created = 0;
    std::size_t instance_records_updated = 0;
    std::size_t commands = 0;
    std::size_t draw_calls = 0;
    /** Number of complete executor reconciliations performed by this call. */
    std::size_t full_rebuilds = 0;
    /** Number of GPU batch records inspected while synchronizing batch layout. */
    std::size_t batches_inspected = 0;
    /** Number of batch instances inspected while synchronizing batch layout. */
    std::size_t batch_instances_inspected = 0;
    /** Number of command records patched by an incremental layout update. */
    std::size_t commands_patched = 0;
};

/* ------------------------------------------------------------------------- */
/* Render plan                                                               */
/* ------------------------------------------------------------------------- */

class RenderPlan {
  public:
    static constexpr std::size_t max_clip_planes = 32;

    std::uint64_t source_revision() const noexcept { return source_revision_; }
    std::uint64_t view_signature() const noexcept { return view_signature_; }
    std::span<const RenderItem> items() const noexcept { return items_; }
    std::span<const WorldTransform> transforms() const noexcept { return transforms_; }
    std::span<const InstanceBatch> batches() const noexcept { return batches_; }
    std::size_t item_index(NodeId node) const noexcept {
        const auto found = item_by_node_.find(node);
        return found == item_by_node_.end() ? invalid_item_index : found->second;
    }
    std::size_t batch_index(NodeId node) const noexcept {
        const auto item = item_index(node);
        return item == invalid_item_index || item >= item_batch_.size() ? invalid_item_index
                                                                        : item_batch_[item];
    }
    std::size_t batch_index(GeometryId geometry, MaterialId material) const noexcept {
        const auto found = batch_by_key_.find({geometry, material});
        return found == batch_by_key_.end() ? invalid_item_index : found->second;
    }
    std::span<const std::size_t> items_for_source(EntityId source) const noexcept {
        const auto found = items_by_source_.find(source);
        return found == items_by_source_.end() ? std::span<const std::size_t>{}
                                               : std::span<const std::size_t>{found->second};
    }
    std::size_t compile_count() const noexcept { return compile_count_; }
    std::size_t visible_items() const noexcept { return visible_items_; }
    std::size_t culled_items() const noexcept { return culled_items_; }
    const std::array<float, 16> &view_projection() const noexcept { return view_projection_; }
    std::span<const std::array<float, 4>> clip_planes() const noexcept {
        return {clip_planes_.data(), clip_plane_count_};
    }

  private:
    static constexpr std::size_t gpu_delta_history_limit = 64;

    struct GpuDelta {
        std::uint64_t revision = 0;
        bool full_rebuild = false;
        bool layout_changed = false;
        bool resource_delta_complete = true;
        std::vector<NodeId> transforms;
        std::vector<NodeId> layout_nodes;
        std::vector<GeometryId> geometries;
        std::vector<MaterialId> materials;
    };

    struct BatchKey {
        GeometryId geometry;
        MaterialId material;

        friend bool operator==(const BatchKey &, const BatchKey &) = default;
    };

    struct BatchKeyHash {
        std::size_t operator()(const BatchKey &key) const noexcept {
            const auto geometry = std::hash<std::uint64_t>{}(key.geometry.value);
            const auto material = std::hash<std::uint64_t>{}(key.material.value);
            return geometry ^ (material + 0x9e3779b9u + (geometry << 6) + (geometry >> 2));
        }
    };

    static constexpr std::size_t invalid_item_index = static_cast<std::size_t>(-1);
    std::uint64_t source_revision_ = 0;
    std::uint64_t view_signature_ = 0;
    std::uint64_t presentation_signature_ = 0;
    std::vector<RenderItem> items_;
    std::vector<WorldTransform> transforms_;
    std::vector<EntityId> item_sources_;
    std::vector<std::vector<NodeId>> item_ancestors_;
    std::vector<InstanceBatch> batches_;
    std::unordered_map<NodeId, std::size_t> item_by_node_;
    std::unordered_map<EntityId, std::vector<std::size_t>> items_by_source_;
    std::unordered_map<NodeId, std::vector<std::size_t>> items_by_ancestor_;
    std::unordered_map<GeometryId, std::vector<std::size_t>> items_by_geometry_;
    std::unordered_map<MaterialId, std::vector<std::size_t>> items_by_material_;
    std::unordered_map<GeometryId, std::vector<std::size_t>> batches_by_geometry_;
    std::unordered_map<MaterialId, std::vector<std::size_t>> batches_by_material_;
    std::unordered_map<BatchKey, std::size_t, BatchKeyHash> batch_by_key_;
    std::vector<std::size_t> item_batch_;
    std::vector<std::size_t> item_batch_position_;
    std::unordered_map<GeometryId, std::uint64_t> geometry_revisions_;
    std::unordered_map<MaterialId, std::uint64_t> material_revisions_;
    std::uint64_t geometry_resources_revision_ = 0;
    std::uint64_t material_resources_revision_ = 0;
    std::vector<NodeId> view_override_nodes_;
    std::vector<EntityId> view_source_policy_sources_;
    std::vector<EntityId> view_isolation_sources_;
    std::vector<NodeId> view_isolation_nodes_;
    bool view_global_policy_ = false;
    bool view_source_rules_only_ = false;
    bool view_isolation_only_ = false;
    NodeId view_root_;
    std::array<float, 16> view_projection_ = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                              0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    bool camera_enabled_ = false;
    std::array<std::array<float, 4>, max_clip_planes> clip_planes_{};
    std::uint32_t clip_plane_count_ = 0;
    std::size_t visible_items_ = 0;
    std::size_t culled_items_ = 0;
    std::size_t compile_count_ = 0;
    std::uint64_t culling_signature_ = 0;
    std::uint64_t pose_signature_ = 0;
    std::uint64_t gpu_identity_ = 0;
    std::uint64_t gpu_revision_ = 0;
    std::uint64_t gpu_delta_history_start_ = 0;
    std::vector<GpuDelta> gpu_delta_history_;
    std::shared_ptr<SceneSpatialIndex> culling_index_;
    std::unordered_set<NodeId> culling_dirty_nodes_;
    std::unordered_set<NodeId> culling_unbounded_nodes_;
    std::unordered_set<NodeId> culled_nodes_;

    friend NKSRENDER_API RenderPlan compile(const SceneSnapshot &, const SceneView &);
    friend NKSRENDER_API RenderUpdate update(RenderPlan &, const SceneSnapshot &, const ChangeSet &,
                                             const SceneView &);
    friend NKSRENDER_API RenderUpdate refresh(RenderPlan &, const SceneSnapshot &,
                                              const SceneView &);
    friend class NativeKitGpuExecutor;
    friend void render_internal::rebuild_batches(RenderPlan &plan);
    friend std::size_t render_internal::move_item_batch(RenderPlan &plan, std::size_t item_index,
                                                        GeometryId geometry, MaterialId material);
    friend void render_internal::capture_view_policy(RenderPlan &plan, const SceneView &view);
    friend void render_internal::build_items(RenderPlan &plan, const SceneSnapshot &snapshot,
                                             const SceneView &view);
    friend void render_internal::update_ancestor_index(RenderPlan &plan,
                                                       const SceneSnapshot &snapshot,
                                                       const ChangeSet &changes);
};

/* ------------------------------------------------------------------------- */
/* Render plan operations                                                    */
/* ------------------------------------------------------------------------- */

NKSRENDER_API RenderPlan compile(const SceneSnapshot &snapshot, const SceneView &view);
NKSRENDER_API RenderUpdate update(RenderPlan &plan, const SceneSnapshot &snapshot,
                                  const ChangeSet &changes, const SceneView &view);
NKSRENDER_API RenderUpdate refresh(RenderPlan &plan, const SceneSnapshot &snapshot,
                                   const SceneView &view);

NKSRENDER_API PickResult pick(const RenderPlan &, const SceneSnapshot &, std::uint32_t primitive,
                              Vec3 world_position, float depth);

/* ------------------------------------------------------------------------- */
/* Image capture                                                             */
/* ------------------------------------------------------------------------- */

/** Optional image-space processing applied by capture_rgba8 on supported GPU backends. */
struct RgbaPostProcess {
    /** Multiplicative brightness adjustment expressed in powers of two. */
    float exposure_stops = 0.0f;
    /** Additional linear brightness multiplier. */
    float gain = 1.0f;
    /** Additive Gaussian standard deviation in normalized RGB units. */
    float noise_stddev = 0.0f;
    /** Normalized RGB step; zero leaves values unquantized. */
    float quantization = 0.0f;
    /** Radial lens coefficients in normalized image coordinates. */
    float distortion_k1 = 0.0f;
    float distortion_k2 = 0.0f;
    /** Independent probability of replacing each pixel with black. */
    float dropout_probability = 0.0f;
    std::uint64_t seed = 0;
    std::uint64_t sequence = 0;

    bool enabled() const noexcept {
        return exposure_stops != 0.0f || gain != 1.0f || noise_stddev != 0.0f ||
               quantization != 0.0f || distortion_k1 != 0.0f || distortion_k2 != 0.0f ||
               dropout_probability != 0.0f;
    }
};

/* ------------------------------------------------------------------------- */
/* GPU executor                                                              */
/* ------------------------------------------------------------------------- */

/**
 * Executes the opaque triangle subset of a RenderPlan through NativeKit GPU.
 * Geometry payloads use GeometryPayload's object-local float3 vertices and
 * optional uint32 triangle indices. Constructing the executor without a
 * renderer retains the headless resource/command path.
 * A renderer must outlive the executor while GPU resources are cached.
 */
class NKSRENDER_API NativeKitGpuExecutor {
  public:
    NativeKitGpuExecutor();
    explicit NativeKitGpuExecutor(nkgpu_renderer renderer);
    ~NativeKitGpuExecutor();
    NativeKitGpuExecutor(NativeKitGpuExecutor &&) noexcept;
    NativeKitGpuExecutor &operator=(NativeKitGpuExecutor &&) noexcept;
    NativeKitGpuExecutor(const NativeKitGpuExecutor &) = delete;
    NativeKitGpuExecutor &operator=(const NativeKitGpuExecutor &) = delete;

    void set_renderer(nkgpu_renderer renderer) noexcept;
    nkgpu_renderer renderer() const noexcept;
    nkgpu_result last_result() const noexcept;

    GpuExecutionStats execute(const RenderPlan &, const SceneSnapshot &);
    /** Renders a plan into an off-screen RGBA8 image and reads it back. */
    nkgpu_result capture_rgba8(const RenderPlan &, const SceneSnapshot &, std::uint32_t width,
                               std::uint32_t height, std::array<float, 4> clear_color,
                               std::vector<std::uint8_t> &out_pixels,
                               RgbaPostProcess post_process = {},
                               nk_graphics_image *out_image = nullptr);
    /** Renders a depth-tested plan and reads normalized device depth values back. */
    nkgpu_result capture_depth(const RenderPlan &, const SceneSnapshot &, std::uint32_t width,
                               std::uint32_t height, std::vector<float> &out_depth);
    /** Renders a depth-tested plan and reads encoded node IDs back. */
    nkgpu_result capture_pick_ids(const RenderPlan &, const SceneSnapshot &, std::uint32_t width,
                                  std::uint32_t height, std::vector<std::uint32_t> &out_ids);
    /** Renders an ID-only pass and resolves one pixel to scene ownership. */
    nkgpu_result pick_pixel(const RenderPlan &, const SceneSnapshot &, std::uint32_t width,
                            std::uint32_t height, std::uint32_t x, std::uint32_t y,
                            PickResult *out_result);
    nkgpu_result begin_pick_pixel(const RenderPlan &, const SceneSnapshot &, std::uint32_t width,
                                  std::uint32_t height, std::uint32_t x, std::uint32_t y,
                                  std::shared_ptr<GpuPickRequest> &out_request);
    std::uint32_t poll_pick_pixel(GpuPickRequest &, const RenderPlan &current_plan,
                                  const SceneSnapshot &current_snapshot, PickResult *out_result,
                                  nkgpu_result *out_error);
    std::span<const GpuCommand> commands() const noexcept;

  private:
    bool synchronize(const RenderPlan &, const SceneSnapshot &, GpuExecutionStats &);

    struct State;
    std::unique_ptr<State> state_;
};

} // namespace nkscene
