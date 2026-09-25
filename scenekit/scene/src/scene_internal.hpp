#pragma once

#include "nativekit_scene.h"

#include "changeset.hpp"
#include "component_store.hpp"
#include "hierarchy.hpp"
#include "node_store.hpp"
#include "resources.hpp"
#include "transaction.hpp"

#include <cstdint>
#include <array>
#include <memory>
#include <string>
#include <span>
#include <unordered_map>
#include <vector>

namespace nkscene {

constexpr std::size_t published_node_page_capacity = 256;
constexpr std::size_t published_direct_lookup_limit = 16384;
// Bound both linked history and the total changed IDs stored by that history.
constexpr std::size_t published_resource_delta_max_entries = 4096;
constexpr std::size_t published_resource_delta_max_ids = 65536;

struct SnapshotMaterialization;

struct PublishedResourceDelta {
    std::uint64_t geometry_revision = 0;
    std::uint64_t material_revision = 0;
    std::uint64_t geometry_base_revision = 0;
    std::uint64_t material_base_revision = 0;
    // Include this delta; used to trim the chain without walking it on publication.
    std::size_t retained_entries = 1;
    std::size_t retained_ids = 0;
    // False when this publication deliberately omitted an oversized resource ID list.
    bool geometry_complete = true;
    bool material_complete = true;
    std::vector<GeometryId> geometries;
    std::vector<MaterialId> materials;
    std::shared_ptr<const PublishedResourceDelta> previous;
};

struct PublishedNodePage {
    std::array<SceneNode, published_node_page_capacity> values{};
};

struct PublishedNodeState {
    std::size_t slot_count = 0;
    std::vector<std::shared_ptr<const PublishedNodePage>> pages;
    std::shared_ptr<const std::unordered_map<NodeId, NodeHandle>> changed_handles;
    mutable std::shared_ptr<const std::unordered_map<NodeId, const SceneNode *>>
        lookup;
    mutable std::shared_ptr<const SnapshotMaterialization> materialized;
};

struct SnapshotMaterialization {
    std::vector<SceneNode> nodes;
    std::unordered_map<NodeId, std::vector<NodeId>> children_by_parent;
    std::unordered_map<EntityId, std::vector<NodeId>> nodes_by_source;
    std::unordered_map<GeometryId, std::vector<NodeId>> nodes_by_geometry;
    std::unordered_map<MaterialId, std::vector<NodeId>> nodes_by_material;
};

struct PublishedSceneState {
    RevisionCounters revisions;
    std::unordered_map<EntityId, std::string> entity_names;
    std::uint64_t geometry_store_revision = 0;
    std::uint64_t material_store_revision = 0;
    std::uint64_t image_store_revision = 0;
    std::uint64_t texture_store_revision = 0;
    std::uint64_t sampler_store_revision = 0;
    std::uint64_t geometry_resources_revision = 0;
    std::uint64_t material_resources_revision = 0;
    std::shared_ptr<const PublishedResourceDelta> resource_delta;
    std::shared_ptr<const PublishedNodeState> nodes;
    std::shared_ptr<const std::vector<GeometryResource>> geometries;
    std::shared_ptr<const std::vector<MaterialResource>> materials;
    std::shared_ptr<const std::vector<ImageResource>> images;
    std::shared_ptr<const std::vector<TextureResource>> textures;
    std::shared_ptr<const std::vector<SamplerResource>> samplers;
    std::shared_ptr<const std::vector<CameraResource>> cameras;
    std::shared_ptr<const std::vector<LightResource>> lights;
};

class NKS_API Scene {
public:
    Scene();

    NodeId reserve_node_id() noexcept { return nodes.reserve_id(); }
    GeometryId reserve_geometry_id() noexcept { return GeometryId{next_geometry_id++}; }
    MaterialId reserve_material_id() noexcept { return MaterialId{next_material_id++}; }
    ImageId reserve_image_id() noexcept { return ImageId{next_image_id++}; }
    TextureId reserve_texture_id() noexcept { return TextureId{next_texture_id++}; }
    SamplerId reserve_sampler_id() noexcept { return SamplerId{next_sampler_id++}; }
    CameraId reserve_camera_id() noexcept { return CameraId{next_camera_id++}; }
    LightId reserve_light_id() noexcept { return LightId{next_light_id++}; }
    GeometryId create_geometry();
    MaterialId create_material();
    ImageId create_image() {
        const auto id = reserve_image_id();
        images.create(id);
        return id;
    }
    TextureId create_texture() {
        const auto id = reserve_texture_id();
        textures.create(id);
        return id;
    }
    SamplerId create_sampler() {
        const auto id = reserve_sampler_id();
        samplers.create(id);
        return id;
    }
    CameraId create_camera() {
        const auto id = reserve_camera_id();
        cameras.create(id);
        return id;
    }
    LightId create_light() {
        const auto id = reserve_light_id();
        lights.create(id);
        return id;
    }
    void destroy_geometry(GeometryId id) noexcept;
    void destroy_material(MaterialId id) noexcept;
    void destroy_image(ImageId id) noexcept { images.destroy(id); }
    void destroy_texture(TextureId id) noexcept { textures.destroy(id); }
    void destroy_sampler(SamplerId id) noexcept { samplers.destroy(id); }
    void destroy_camera(CameraId id) noexcept { cameras.destroy(id); }
    void destroy_light(LightId id) noexcept { lights.destroy(id); }
    std::size_t node_count() const noexcept { return nodes.size(); }
    bool contains(NodeId id) const noexcept { return nodes.contains(id); }

    nkscene_result commit(const Transaction &transaction, ChangeSet &changes);
    SceneSnapshot snapshot() const;

    std::uint64_t revision() const noexcept { return revisions.scene; }
    const RevisionCounters &revision_counters() const noexcept { return revisions; }

    const NodeStore &node_store() const noexcept { return nodes; }
    const HierarchyIndex &hierarchy_index() const noexcept { return hierarchy; }
    const ComponentStore<LocalTransform> &transforms() const noexcept { return local_transforms; }
    const ComponentStore<WorldTransform> &world_transforms() const noexcept {
        return world_transforms_;
    }
    const ComponentStore<Visibility> &visibilities() const noexcept { return visibilities_; }
    const ComponentStore<CameraRef> &camera_refs() const noexcept { return camera_refs_; }
    const ComponentStore<LightRef> &light_refs() const noexcept { return light_refs_; }
    const ComponentStore<std::string> &names() const noexcept { return names_; }
    const GeometryStore &geometry_store() const noexcept { return geometries; }
    const MaterialStore &material_store() const noexcept { return materials; }
    GeometryStore &geometry_store() noexcept { return geometries; }
    MaterialStore &material_store() noexcept { return materials; }
    ImageStore &image_store() noexcept { return images; }
    TextureStore &texture_store() noexcept { return textures; }
    SamplerStore &sampler_store() noexcept { return samplers; }
    CameraStore &camera_store() noexcept { return cameras; }
    LightStore &light_store() noexcept { return lights; }
    const ImageStore &image_store() const noexcept { return images; }
    const TextureStore &texture_store() const noexcept { return textures; }
    const SamplerStore &sampler_store() const noexcept { return samplers; }
    const CameraStore &camera_store() const noexcept { return cameras; }
    const LightStore &light_store() const noexcept { return lights; }

    /** Publishes externally edited resources into the next immutable snapshot. */
    void publish();

private:
    struct TransactionOverlay {
        struct Entry {
            NodeId id;
            NodeHandle handle;
            NodeId parent = invalid_node;
            bool live = false;
            bool created = false;
        };

        std::unordered_map<NodeId, std::size_t> indices;
        std::vector<Entry> entries;
    };

    nkscene_result validate(const Transaction &transaction,
                            TransactionOverlay &overlay) const noexcept;
    void recompute_world_transforms(ChangeSet &changes,
                                    const TransactionOverlay &overlay);
    void refresh_geometry_bounds(ChangeSet &changes,
                                 std::unordered_map<NodeId, std::size_t> &change_indices);
    void publish_state(const ChangeSet *changes,
                       std::span<const std::uint32_t> destroyed_slots) const;
    void record_change(ChangeSet &changes, std::unordered_map<NodeId, std::size_t> &indices,
                       NodeId id, ChangeDomain domain);

    NodeStore nodes;
    ComponentStore<SourceEntity> source_entities;
    ComponentStore<Parent> parent_components;
    ComponentStore<LocalTransform> local_transforms;
    ComponentStore<WorldTransform> world_transforms_;
    ComponentStore<GeometryRef> geometry_refs;
    ComponentStore<MaterialRef> material_refs;
    ComponentStore<CameraRef> camera_refs_;
    ComponentStore<LightRef> light_refs_;
    ComponentStore<Visibility> visibilities_;
    ComponentStore<std::string> names_;
    std::unordered_map<EntityId, std::string> entity_names;
    ComponentStore<Bounds> bounds;
    HierarchyIndex hierarchy;
    GeometryStore geometries;
    MaterialStore materials;
    ImageStore images;
    TextureStore textures;
    SamplerStore samplers;
    CameraStore cameras;
    LightStore lights;
    std::uint64_t next_geometry_id = 1;
    std::uint64_t next_material_id = 1;
    std::uint64_t next_image_id = 1;
    std::uint64_t next_texture_id = 1;
    std::uint64_t next_sampler_id = 1;
    std::uint64_t next_camera_id = 1;
    std::uint64_t next_light_id = 1;
    RevisionCounters revisions;
    mutable std::shared_ptr<const PublishedSceneState> published_;
};

NKS_API std::shared_ptr<const SceneSnapshot> resolve_snapshot_handle(
    nkscene_snapshot snapshot) noexcept;
NKS_API std::shared_ptr<const ChangeSet> resolve_change_set_handle(
    nkscene_change_set changes) noexcept;

} // namespace nkscene
