#pragma once

#include "ids.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace nkscene {

class Scene;

struct SourceEntity {
    EntityId id;
};

struct Parent {
    NodeId id;
};

struct GeometryRef {
    GeometryId id;
};

struct MaterialRef {
    MaterialId id;
};

struct CameraRef {
    CameraId id;
};

struct LightRef {
    LightId id;
};

struct Visibility {
    bool visible = true;
};

struct CreateNode {
    NodeId node;
};

struct DestroyNode {
    NodeId node;
};

struct SetParent {
    NodeId node;
    NodeId parent;
};

struct SetTransform {
    NodeId node;
    LocalTransform transform;
};

struct SetGeometry {
    NodeId node;
    GeometryId geometry;
};

struct SetMaterial {
    NodeId node;
    MaterialId material;
};

struct SetCamera {
    NodeId node;
    CameraId camera;
};

struct SetLight {
    NodeId node;
    LightId light;
};

struct SetVisibility {
    NodeId node;
    bool visible;
};

struct SetSourceEntity {
    NodeId node;
    EntityId source;
};

struct SetName {
    NodeId node;
    std::string name;
};

struct SetEntityName {
    EntityId entity;
    std::string name;
};

using Mutation = std::variant<CreateNode, DestroyNode, SetParent,
                              SetTransform, SetGeometry, SetMaterial, SetCamera, SetLight,
                              SetVisibility, SetSourceEntity, SetName, SetEntityName>;

class Transaction {
public:
    explicit Transaction(std::shared_ptr<Scene> scene) : scene_(std::move(scene)) {}

    const std::shared_ptr<Scene> &scene() const noexcept { return scene_; }
    bool active() const noexcept { return active_; }
    void close() noexcept { active_ = false; }

    void add_create(NodeId id) { mutations_.emplace_back(CreateNode{id}); }
    void add_destroy(NodeId id) { mutations_.emplace_back(DestroyNode{id}); }
    void add_parent(NodeId id, NodeId parent) {
        mutations_.emplace_back(SetParent{id, parent});
    }
    void add_transform(NodeId id, const LocalTransform &transform) {
        mutations_.emplace_back(SetTransform{id, transform});
    }
    void add_transforms(std::span<const TransformUpdate> updates) {
        mutations_.reserve(mutations_.size() + updates.size());
        for (const auto &update : updates)
            add_transform(update.node, update.transform);
    }
    void add_geometry(NodeId id, GeometryId geometry) {
        mutations_.emplace_back(SetGeometry{id, geometry});
    }
    void add_material(NodeId id, MaterialId material) {
        mutations_.emplace_back(SetMaterial{id, material});
    }
    void add_camera(NodeId id, CameraId camera) {
        mutations_.emplace_back(SetCamera{id, camera});
    }
    void add_light(NodeId id, LightId light) {
        mutations_.emplace_back(SetLight{id, light});
    }
    void add_visibility(NodeId id, bool visible) {
        mutations_.emplace_back(SetVisibility{id, visible});
    }
    void add_source_entity(NodeId id, EntityId source) {
        mutations_.emplace_back(SetSourceEntity{id, source});
    }
    void add_name(NodeId id, std::string name) {
        mutations_.emplace_back(SetName{id, std::move(name)});
    }
    void add_entity_name(EntityId id, std::string name) {
        mutations_.emplace_back(SetEntityName{id, std::move(name)});
    }

    const std::vector<Mutation> &mutations() const noexcept { return mutations_; }

private:
    std::shared_ptr<Scene> scene_;
    std::vector<Mutation> mutations_;
    bool active_ = true;
};

} // namespace nkscene
