#include "render_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_set>

namespace nkscene::render_internal {

namespace {

Bounds transformed_bounds(const Bounds &local, const LocalTransform &transform) noexcept {
    Bounds result;
    for (auto &value : result.minimum)
        value = std::numeric_limits<float>::infinity();
    for (auto &value : result.maximum)
        value = -std::numeric_limits<float>::infinity();
    if (!local.valid)
        return result;
    for (const auto x : {local.minimum[0], local.maximum[0]})
        for (const auto y : {local.minimum[1], local.maximum[1]})
            for (const auto z : {local.minimum[2], local.maximum[2]}) {
                const auto &m = transform.matrix;
                const std::array<float, 3> point{
                    m[0] * x + m[4] * y + m[8] * z + m[12],
                    m[1] * x + m[5] * y + m[9] * z + m[13],
                    m[2] * x + m[6] * y + m[10] * z + m[14]};
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    result.minimum[axis] = std::min(result.minimum[axis], point[axis]);
                    result.maximum[axis] = std::max(result.maximum[axis], point[axis]);
                }
            }
    result.valid = true;
    return result;
}

bool outside_plane(const Bounds &bounds, const std::array<float, 4> &plane) noexcept {
    const auto x = plane[0] >= 0.0f ? bounds.maximum[0] : bounds.minimum[0];
    const auto y = plane[1] >= 0.0f ? bounds.maximum[1] : bounds.minimum[1];
    const auto z = plane[2] >= 0.0f ? bounds.maximum[2] : bounds.minimum[2];
    return plane[0] * x + plane[1] * y + plane[2] * z + plane[3] < 0.0f;
}

std::array<float, 3> column(const LocalTransform &transform, std::size_t index) noexcept {
    return {transform.matrix[index * 4], transform.matrix[index * 4 + 1],
            transform.matrix[index * 4 + 2]};
}

float dot(const std::array<float, 3> &lhs, const std::array<float, 3> &rhs) noexcept {
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

std::array<float, 3> cross(const std::array<float, 3> &lhs,
                           const std::array<float, 3> &rhs) noexcept {
    return {lhs[1] * rhs[2] - lhs[2] * rhs[1], lhs[2] * rhs[0] - lhs[0] * rhs[2],
            lhs[0] * rhs[1] - lhs[1] * rhs[0]};
}

std::array<float, 3> normalized(std::array<float, 3> value) noexcept {
    const auto length = std::sqrt(dot(value, value));
    if (length > 1.0e-6f)
        for (auto &component : value)
            component /= length;
    return value;
}

std::array<float, 16> multiply(const std::array<float, 16> &lhs,
                               const std::array<float, 16> &rhs) noexcept {
    std::array<float, 16> result{};
    for (std::size_t column_index = 0; column_index < 4; ++column_index)
        for (std::size_t row = 0; row < 4; ++row)
            for (std::size_t index = 0; index < 4; ++index)
                result[column_index * 4 + row] +=
                    lhs[index * 4 + row] * rhs[column_index * 4 + index];
    return result;
}

std::array<float, 16> scene_camera_projection(const CameraResource &camera) noexcept {
    const auto near_plane = camera.near_plane;
    const auto far_plane = camera.far_plane;
    const auto aspect = camera.aspect_ratio > 0.0f ? camera.aspect_ratio : 1.0f;
    std::array<float, 16> result{};
    if (camera.projection == CameraProjection::Orthographic) {
        const auto half_height = camera.orthographic_height * 0.5f;
        const auto half_width = half_height * aspect;
        result[0] = 1.0f / half_width;
        result[5] = 1.0f / half_height;
        result[10] = 2.0f / (far_plane - near_plane);
        result[14] = -(far_plane + near_plane) / (far_plane - near_plane);
        result[15] = 1.0f;
    } else {
        const auto focal = 1.0f / std::tan(camera.fov_y * 0.5f);
        result[0] = focal / aspect;
        result[5] = focal;
        result[10] = (far_plane + near_plane) / (far_plane - near_plane);
        result[11] = 1.0f;
        result[14] = -(2.0f * far_plane * near_plane) / (far_plane - near_plane);
    }
    return result;
}

std::array<float, 16> scene_camera_view(const LocalTransform &transform) noexcept {
    const auto position = column(transform, 3);
    const auto forward = normalized(column(transform, 0));
    const auto up = normalized(column(transform, 2));
    const auto right = normalized(cross(forward, up));
    return {right[0],
            up[0],
            forward[0],
            0.0f,
            right[1],
            up[1],
            forward[1],
            0.0f,
            right[2],
            up[2],
            forward[2],
            0.0f,
            -dot(right, position),
            -dot(up, position),
            -dot(forward, position),
            1.0f};
}

SceneCamera camera_from_node(const SceneNode &node,
                                   const CameraResource &resource) noexcept {
    SceneCamera result;
    result.enabled = true;
    result.view_projection = multiply(scene_camera_projection(resource),
                                      scene_camera_view(node.world_transform.transform));
    return result;
}

} // namespace

SceneCamera camera_for_snapshot(const SceneSnapshot &snapshot, const SceneView &view) noexcept {
    if (view.camera.enabled)
        return view.camera;
    if (view.camera_node.valid()) {
        const auto *node = snapshot.find(view.camera_node);
        if (node && node->camera.valid()) {
            if (const auto *resource = snapshot.find_camera(node->camera))
                return camera_from_node(*node, *resource);
        }
    }
    return {};
}

bool culled_by_camera(const Bounds &bounds, const SceneCamera &camera) noexcept {
    if (!camera.enabled || !bounds.valid)
        return false;
    const auto &m = camera.view_projection;
    const std::array<std::array<float, 4>, 6> planes = {
        {{m[0] + m[3], m[4] + m[7], m[8] + m[11], m[12] + m[15]},
         {m[3] - m[0], m[7] - m[4], m[11] - m[8], m[15] - m[12]},
         {m[1] + m[3], m[5] + m[7], m[9] + m[11], m[13] + m[15]},
         {m[3] - m[1], m[7] - m[5], m[11] - m[9], m[15] - m[13]},
         {m[2] + m[3], m[6] + m[7], m[10] + m[11], m[14] + m[15]},
         {m[3] - m[2], m[7] - m[6], m[11] - m[10], m[15] - m[14]}}};
    for (const auto &plane : planes)
        if (outside_plane(bounds, plane))
            return true;
    return false;
}

bool culled_by_clip_planes(const Bounds &bounds, std::span<const ClipPlane> planes) noexcept {
    if (!bounds.valid)
        return false;
    for (const auto &plane : planes) {
        if (!plane.enabled)
            continue;
        const auto x = plane.normal[0] >= 0.0f ? bounds.maximum[0] : bounds.minimum[0];
        const auto y = plane.normal[1] >= 0.0f ? bounds.maximum[1] : bounds.minimum[1];
        const auto z = plane.normal[2] >= 0.0f ? bounds.maximum[2] : bounds.minimum[2];
        if (plane.normal[0] * x + plane.normal[1] * y + plane.normal[2] * z + plane.distance < 0.0f)
            return true;
    }
    return false;
}

void append_culling_planes(const SceneView &view, std::vector<std::array<float, 4>> &planes) {
    if (view.camera.enabled) {
        const auto &m = view.camera.view_projection;
        planes.push_back({m[0] + m[3], m[4] + m[7], m[8] + m[11], m[12] + m[15]});
        planes.push_back({m[3] - m[0], m[7] - m[4], m[11] - m[8], m[15] - m[12]});
        planes.push_back({m[1] + m[3], m[5] + m[7], m[9] + m[11], m[13] + m[15]});
        planes.push_back({m[3] - m[1], m[7] - m[5], m[11] - m[9], m[15] - m[13]});
        planes.push_back({m[2] + m[3], m[6] + m[7], m[10] + m[11], m[14] + m[15]});
        planes.push_back({m[3] - m[2], m[7] - m[6], m[11] - m[10], m[15] - m[14]});
    }
    for (const auto &plane : view.clip_planes) {
        if (plane.enabled)
            planes.push_back({plane.normal[0], plane.normal[1], plane.normal[2], plane.distance});
    }
}

EffectiveState effective_state(const SceneSnapshot &snapshot, const SceneView &view) {
    EffectiveState result;
    const auto nodes = snapshot.nodes();
    result.in_view.reserve(nodes.size());
    result.visible.reserve(nodes.size());
    result.material.reserve(nodes.size());

    for (const auto &node : nodes) {
        result.in_view.emplace(node.node, false);
        result.material.emplace(node.node, node.material);
    }

    std::unordered_map<NodeId, bool> visibility_overrides;
    visibility_overrides.reserve(view.visibility_overrides.size());
    for (const auto &override : view.visibility_overrides)
        visibility_overrides[override.node] = override.visible;

    std::unordered_map<EntityId, bool> source_visibility_overrides;
    source_visibility_overrides.reserve(view.filter.source_visibility_overrides.size());
    for (const auto &override : view.filter.source_visibility_overrides)
        source_visibility_overrides[override.source] = override.visible;

    std::unordered_set<NodeId> isolated_keep;
    if (!view.filter.isolated_sources.empty()) {
        for (const auto source : view.filter.isolated_sources) {
            for (const auto node_id : snapshot.nodes_for_source(source)) {
                auto current = node_id;
                while (current.valid()) {
                    if (!isolated_keep.insert(current).second)
                        break;
                    const auto *node = snapshot.find(current);
                    if (!node || !node->parent.valid())
                        break;
                    current = node->parent;
                }
            }
        }
    }
    for (const auto node_id : view.filter.isolated_nodes) {
        if (!snapshot.find(node_id))
            continue;
        std::vector<NodeId> pending{node_id};
        while (!pending.empty()) {
            const auto current = pending.back();
            pending.pop_back();
            if (!isolated_keep.insert(current).second)
                continue;
            const auto parent = snapshot.find(current)->parent;
            if (parent.valid())
                pending.push_back(parent);
            for (const auto child : snapshot.children(current))
                pending.push_back(child);
        }
    }
    const bool isolation_active =
        !view.filter.isolated_sources.empty() || !view.filter.isolated_nodes.empty();

    std::unordered_map<NodeId, MaterialId> material_overrides;
    material_overrides.reserve(view.material_overrides.size() +
                               view.selection_material_overrides.size() +
                               view.hover_material_overrides.size());
    for (const auto &node : nodes) {
        const auto found = std::find_if(
            view.filter.source_material_overrides.begin(),
            view.filter.source_material_overrides.end(),
            [&node](const auto &override) { return override.source == node.source; });
        if (found != view.filter.source_material_overrides.end())
            result.material[node.node] = found->material;
    }
    const auto apply_material_layer = [&material_overrides](const auto &overrides) {
        for (const auto &override : overrides)
            material_overrides[override.node] = override.material;
    };
    apply_material_layer(view.material_overrides);
    apply_material_layer(view.selection_material_overrides);
    apply_material_layer(view.hover_material_overrides);
    for (const auto &[node, material] : material_overrides)
        if (result.material.contains(node))
            result.material[node] = material;

    std::unordered_map<NodeId, bool> visited;
    visited.reserve(nodes.size());
    const auto walk = [&](NodeId start, bool parent_visible, bool selected,
                          const auto &self) -> void {
        const auto *node = snapshot.find(start);
        if (!node || visited.contains(start))
            return;
        visited.emplace(start, true);
        const auto override_found = visibility_overrides.find(start);
        const auto source_override_found = source_visibility_overrides.find(node->source);
        bool local_visible = view.include_invisible || node->visible;
        if (source_override_found != source_visibility_overrides.end())
            local_visible = source_override_found->second;
        if (override_found != visibility_overrides.end())
            local_visible = override_found->second;
        if (isolation_active && !isolated_keep.contains(start))
            local_visible = false;
        const bool visible = parent_visible && local_visible;
        if (selected) {
            result.in_view[start] = true;
            result.visible[start] = visible;
        }
        for (const auto child : snapshot.children(start))
            self(child, visible, selected, self);
    };

    if (view.root.valid()) {
        if (snapshot.find(view.root))
            walk(view.root, true, true, walk);
    } else {
        for (const auto &node : nodes)
            if (!node.parent.valid() || !snapshot.find(node.parent))
                walk(node.node, true, true, walk);
        for (const auto &node : nodes)
            if (!visited.contains(node.node))
                walk(node.node, true, true, walk);
    }
    return result;
}

std::uint64_t presentation_signature(const SceneView &view) noexcept {
    std::uint64_t hash = 1469598103934665603ull;
    const auto add = [&hash](std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ull;
    };
    add(view.root.value);
    add(view.include_invisible ? 1 : 0);
    add(view.visibility_overrides.size());
    for (const auto &override : view.visibility_overrides) {
        add(override.node.value);
        add(override.visible ? 1 : 0);
    }
    add(view.material_overrides.size());
    for (const auto &override : view.material_overrides) {
        add(override.node.value);
        add(override.material.value);
    }
    add(view.selection_material_overrides.size());
    for (const auto &override : view.selection_material_overrides) {
        add(override.node.value);
        add(override.material.value);
    }
    add(view.hover_material_overrides.size());
    for (const auto &override : view.hover_material_overrides) {
        add(override.node.value);
        add(override.material.value);
    }
    add(view.filter.isolated_sources.size());
    for (const auto source : view.filter.isolated_sources)
        add(source.value);
    add(view.filter.source_visibility_overrides.size());
    for (const auto &override : view.filter.source_visibility_overrides) {
        add(override.source.value);
        add(override.visible ? 1 : 0);
    }
    add(view.filter.source_material_overrides.size());
    for (const auto &override : view.filter.source_material_overrides) {
        add(override.source.value);
        add(override.material.value);
    }
    add(view.filter.isolated_nodes.size());
    for (const auto node : view.filter.isolated_nodes)
        add(node.value);
    return hash;
}

std::uint64_t pose_signature(const SceneView &view) noexcept {
    std::uint64_t hash = 1469598103934665603ull;
    const auto add = [&hash](std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ull;
    };
    for (const auto &pose : view.pose_overrides) {
        add(pose.node.value);
        for (const auto value : pose.world_transform.matrix)
            add(std::hash<float>{}(value));
    }
    return hash;
}

std::uint64_t view_signature(const SceneView &view) noexcept {
    std::uint64_t hash = presentation_signature(view);
    const auto add = [&hash](std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ull;
    };
    add(view.camera.enabled ? 1 : 0);
    add(pose_signature(view));
    add(view.camera_node.value);
    for (const auto value : view.camera.view_projection)
        add(std::hash<float>{}(value));
    add(view.studio_lighting.enabled ? 1 : 0);
    if (view.studio_lighting.enabled) {
        for (const auto &light : view.studio_lighting.directions)
            for (const auto value : light)
                add(std::hash<float>{}(value));
        for (const auto &color : view.studio_lighting.colors)
            for (const auto value : color)
                add(std::hash<float>{}(value));
        for (const auto value : view.studio_lighting.ambient_sky)
            add(std::hash<float>{}(value));
        for (const auto value : view.studio_lighting.ambient_ground)
            add(std::hash<float>{}(value));
    }
    add(view.clip_planes.size());
    for (const auto &plane : view.clip_planes) {
        for (const auto value : plane.normal)
            add(std::hash<float>{}(value));
        add(std::hash<float>{}(plane.distance));
        add(plane.enabled ? 1 : 0);
    }
    return hash;
}

std::uint64_t culling_signature(const SceneView &view) noexcept {
    std::uint64_t hash = 1469598103934665603ull;
    const auto add = [&hash](std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ull;
    };
    add(view.camera.enabled ? 1 : 0);
    add(view.camera_node.value);
    for (const auto value : view.camera.view_projection)
        add(std::hash<float>{}(value));
    add(view.clip_planes.size());
    for (const auto &plane : view.clip_planes) {
        for (const auto value : plane.normal)
            add(std::hash<float>{}(value));
        add(std::hash<float>{}(plane.distance));
        add(plane.enabled ? 1 : 0);
    }
    return hash;
}

void build_items(RenderPlan &plan, const SceneSnapshot &snapshot, const SceneView &view) {
    const auto state = effective_state(snapshot, view);
    const auto camera = camera_for_snapshot(snapshot, view);
    plan.items_.clear();
    plan.transforms_.clear();
    plan.item_sources_.clear();
    plan.item_ancestors_.clear();
    plan.item_by_node_.clear();
    plan.items_by_source_.clear();
    plan.items_by_ancestor_.clear();
    plan.items_by_geometry_.clear();
    plan.items_by_material_.clear();
    plan.visible_items_ = 0;
    plan.culled_items_ = 0;
    plan.view_projection_ = camera.view_projection;
    plan.items_.reserve(snapshot.nodes().size());
    plan.transforms_.reserve(snapshot.nodes().size());
    plan.item_sources_.reserve(snapshot.nodes().size());
    plan.item_ancestors_.reserve(snapshot.nodes().size());
    for (const auto &node : snapshot.nodes()) {
        if (!node.geometry.valid() || !node.material.valid() ||
            !snapshot.find_geometry(node.geometry) ||
            !snapshot.find_material(node.material) ||
            !state.in_view.at(node.node))
            continue;
        const auto transform_index = static_cast<std::uint32_t>(plan.transforms_.size());
        const auto pose = std::find_if(view.pose_overrides.begin(), view.pose_overrides.end(),
                                       [&node](const auto &value) {
                                           return value.node == node.node;
                                       });
        const auto transform = pose == view.pose_overrides.end()
                                   ? node.world_transform
                                   : WorldTransform{pose->world_transform,
                                                    node.world_transform.revision};
        plan.transforms_.push_back(transform);
        RenderItem item;
        item.node = node.node;
        item.geometry = node.geometry;
        item.material = state.material.at(node.node);
        item.pickId = static_cast<std::uint32_t>(plan.items_.size() + 1);
        item.transformIndex = transform_index;
        item.flags = RenderFlags::Opaque;
        if (!state.visible.at(node.node))
            item.flags |= RenderFlags::Hidden;
        const auto bounds = pose == view.pose_overrides.end()
                                ? node.bounds
                                : transformed_bounds(snapshot.find_geometry(node.geometry)->bounds,
                                                     pose->world_transform);
        if (culled_by_camera(bounds, camera) || culled_by_clip_planes(bounds, view.clip_planes)) {
            item.flags |= RenderFlags::Culled;
            ++plan.culled_items_;
        } else if (state.visible.at(node.node)) {
            ++plan.visible_items_;
        }
        const auto item_index = plan.items_.size();
        plan.items_.push_back(item);
        plan.item_sources_.push_back(node.source);
        plan.item_ancestors_.emplace_back();
        plan.item_by_node_.emplace(item.node, item_index);
        plan.items_by_source_[node.source].push_back(item_index);
        plan.items_by_geometry_[item.geometry].push_back(item_index);
        plan.items_by_material_[item.material].push_back(item_index);

        auto ancestor = item.node;
        while (ancestor.valid()) {
            plan.item_ancestors_[item_index].push_back(ancestor);
            plan.items_by_ancestor_[ancestor].push_back(item_index);
            const auto *value = snapshot.find(ancestor);
            if (!value)
                break;
            ancestor = value->parent;
        }
    }
}

void update_ancestor_index(RenderPlan &plan, const SceneSnapshot &snapshot,
                           const ChangeSet &changes) {
    std::unordered_set<std::size_t> affected_items;
    const auto add_item = [&plan, &affected_items](NodeId node) {
        const auto found = plan.item_by_node_.find(node);
        if (found != plan.item_by_node_.end())
            affected_items.insert(found->second);
    };
    for (const auto &change : changes.changes)
        add_item(change.node);
    for (const auto node : changes.world_transform_nodes)
        add_item(node);
    for (const auto node : changes.effective_state_nodes)
        add_item(node);

    for (const auto item_index : affected_items) {
        for (const auto ancestor : plan.item_ancestors_[item_index]) {
            const auto found = plan.items_by_ancestor_.find(ancestor);
            if (found == plan.items_by_ancestor_.end())
                continue;
            auto &items = found->second;
            items.erase(std::remove(items.begin(), items.end(), item_index), items.end());
            if (items.empty())
                plan.items_by_ancestor_.erase(found);
        }
        plan.item_ancestors_[item_index].clear();
        auto ancestor = plan.items_[item_index].node;
        while (ancestor.valid()) {
            plan.item_ancestors_[item_index].push_back(ancestor);
            plan.items_by_ancestor_[ancestor].push_back(item_index);
            const auto *value = snapshot.find(ancestor);
            if (!value)
                break;
            ancestor = value->parent;
        }
    }
}

} // namespace nkscene::render_internal
