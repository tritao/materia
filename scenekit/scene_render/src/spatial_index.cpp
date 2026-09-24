#include "nativekit_scene_render.hpp"
#include "render_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nkscene {

namespace {

constexpr std::uint32_t invalid_slot = std::numeric_limits<std::uint32_t>::max();
constexpr std::size_t leaf_capacity = 8;
// Periodically rebuild after refits so long drags cannot leave a poorly partitioned BVH.
constexpr std::uint32_t max_incremental_refits = 128;

struct Entry {
    NodeId node;
    Bounds bounds;
};

struct BvhNode {
    Bounds bounds;
    std::uint32_t left = invalid_slot;
    std::uint32_t right = invalid_slot;
    std::uint32_t parent = invalid_slot;
    std::uint32_t first = 0;
    std::uint32_t count = 0;

    bool leaf() const noexcept { return left == invalid_slot; }
};

Bounds merge_bounds(const Bounds &lhs, const Bounds &rhs) noexcept {
    if (!lhs.valid)
        return rhs;
    if (!rhs.valid)
        return lhs;
    Bounds result;
    result.valid = true;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        result.minimum[axis] = std::min(lhs.minimum[axis], rhs.minimum[axis]);
        result.maximum[axis] = std::max(lhs.maximum[axis], rhs.maximum[axis]);
    }
    return result;
}

float centroid(const Bounds &bounds, std::size_t axis) noexcept {
    return (bounds.minimum[axis] + bounds.maximum[axis]) * 0.5f;
}

std::size_t split_axis(const Bounds &bounds) noexcept {
    std::size_t axis = 0;
    auto extent = bounds.maximum[0] - bounds.minimum[0];
    for (std::size_t candidate = 1; candidate < 3; ++candidate) {
        const auto candidate_extent = bounds.maximum[candidate] - bounds.minimum[candidate];
        if (candidate_extent > extent) {
            axis = candidate;
            extent = candidate_extent;
        }
    }
    return axis;
}

bool overlaps(const Bounds &lhs, const Bounds &rhs) noexcept {
    if (!lhs.valid || !rhs.valid)
        return false;
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (lhs.maximum[axis] < rhs.minimum[axis] || rhs.maximum[axis] < lhs.minimum[axis])
            return false;
    return true;
}

bool outside_planes(const Bounds &bounds, std::span<const std::array<float, 4>> planes) noexcept {
    if (!bounds.valid)
        return false;
    for (const auto &plane : planes) {
        const auto x = plane[0] >= 0.0f ? bounds.maximum[0] : bounds.minimum[0];
        const auto y = plane[1] >= 0.0f ? bounds.maximum[1] : bounds.minimum[1];
        const auto z = plane[2] >= 0.0f ? bounds.maximum[2] : bounds.minimum[2];
        if (plane[0] * x + plane[1] * y + plane[2] * z + plane[3] < 0.0f)
            return true;
    }
    return false;
}

bool normalize_ray(const Ray &ray, Ray &normalized) noexcept {
    const auto length =
        std::sqrt(ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y +
                  ray.direction.z * ray.direction.z);
    if (!(length > 1.0e-8f))
        return false;
    normalized.origin = ray.origin;
    normalized.direction = {ray.direction.x / length, ray.direction.y / length,
                            ray.direction.z / length};
    return true;
}

bool ray_hits_bounds(const Ray &ray, const Bounds &bounds) noexcept {
    if (!bounds.valid)
        return false;
    float near_distance = 0.0f;
    float far_distance = std::numeric_limits<float>::infinity();
    const std::array<float, 3> origin{ray.origin.x, ray.origin.y, ray.origin.z};
    const std::array<float, 3> direction{ray.direction.x, ray.direction.y, ray.direction.z};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (std::abs(direction[axis]) < 1.0e-8f) {
            if (origin[axis] < bounds.minimum[axis] || origin[axis] > bounds.maximum[axis])
                return false;
            continue;
        }
        auto first = (bounds.minimum[axis] - origin[axis]) / direction[axis];
        auto second = (bounds.maximum[axis] - origin[axis]) / direction[axis];
        if (first > second)
            std::swap(first, second);
        near_distance = std::max(near_distance, first);
        far_distance = std::min(far_distance, second);
        if (near_distance > far_distance)
            return false;
    }
    return far_distance >= 0.0f;
}

Vec3 transform_point(const LocalTransform &transform, const GeometryVertex &vertex) noexcept {
    const auto &point = vertex.position;
    return {transform.matrix[0] * point[0] + transform.matrix[4] * point[1] +
                transform.matrix[8] * point[2] + transform.matrix[12],
            transform.matrix[1] * point[0] + transform.matrix[5] * point[1] +
                transform.matrix[9] * point[2] + transform.matrix[13],
            transform.matrix[2] * point[0] + transform.matrix[6] * point[1] +
                transform.matrix[10] * point[2] + transform.matrix[14]};
}

Bounds transformed_bounds(const Bounds &local, const LocalTransform &transform) noexcept {
    Bounds bounds;
    if (!local.valid)
        return bounds;
    for (auto &value : bounds.minimum)
        value = std::numeric_limits<float>::infinity();
    for (auto &value : bounds.maximum)
        value = -std::numeric_limits<float>::infinity();
    for (const auto x : {local.minimum[0], local.maximum[0]})
        for (const auto y : {local.minimum[1], local.maximum[1]})
            for (const auto z : {local.minimum[2], local.maximum[2]}) {
                GeometryVertex vertex;
                vertex.position = {x, y, z};
                const auto point = transform_point(transform, vertex);
                const std::array<float, 3> values{point.x, point.y, point.z};
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    bounds.minimum[axis] = std::min(bounds.minimum[axis], values[axis]);
                    bounds.maximum[axis] = std::max(bounds.maximum[axis], values[axis]);
                }
            }
    bounds.valid = true;
    return bounds;
}

Vec3 subtract(Vec3 lhs, Vec3 rhs) noexcept {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 cross(Vec3 lhs, Vec3 rhs) noexcept {
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

float dot(Vec3 lhs, Vec3 rhs) noexcept {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 scale_add(Vec3 origin, Vec3 direction, float distance) noexcept {
    return {origin.x + direction.x * distance, origin.y + direction.y * distance,
            origin.z + direction.z * distance};
}

bool ray_hits_triangle(const Ray &ray, Vec3 first, Vec3 second, Vec3 third,
                       float &distance) noexcept {
    constexpr float epsilon = 1.0e-7f;
    const auto edge_one = subtract(second, first);
    const auto edge_two = subtract(third, first);
    const auto perpendicular = cross(ray.direction, edge_two);
    const auto determinant = dot(edge_one, perpendicular);
    if (std::abs(determinant) < epsilon)
        return false;
    const auto inverse = 1.0f / determinant;
    const auto offset = subtract(ray.origin, first);
    const auto barycentric_u = dot(offset, perpendicular) * inverse;
    if (barycentric_u < 0.0f || barycentric_u > 1.0f)
        return false;
    const auto direction = cross(offset, edge_one);
    const auto barycentric_v = dot(ray.direction, direction) * inverse;
    if (barycentric_v < 0.0f || barycentric_u + barycentric_v > 1.0f)
        return false;
    const auto candidate = dot(edge_two, direction) * inverse;
    if (candidate < 0.0f)
        return false;
    distance = candidate;
    return true;
}

std::uint32_t vertex_index(const GeometryPayload &payload, std::size_t index) noexcept {
    return payload.indexed() ? payload.indices[index] : static_cast<std::uint32_t>(index);
}

void intersect_triangles(const Ray &ray, const SceneNode &node,
                         const GeometryResource &geometry, const LocalTransform &transform,
                         float &best_distance, PickResult &result,
                         std::span<const ClipPlane> clip_planes = {}) noexcept {
    if (geometry.payload->primitive_type != PrimitiveType::Triangles)
        return;
    const auto primitive_count = geometry.payload->element_count() / 3;
    for (std::size_t primitive = 0; primitive < primitive_count; ++primitive) {
        const auto first_index =
            static_cast<std::size_t>(vertex_index(*geometry.payload, primitive * 3));
        const auto second_index =
            static_cast<std::size_t>(vertex_index(*geometry.payload, primitive * 3 + 1));
        const auto third_index =
            static_cast<std::size_t>(vertex_index(*geometry.payload, primitive * 3 + 2));
        if (first_index >= geometry.payload->vertices.size() ||
            second_index >= geometry.payload->vertices.size() ||
            third_index >= geometry.payload->vertices.size())
            continue;
        float distance = 0.0f;
        if (!ray_hits_triangle(ray,
                               transform_point(transform, geometry.payload->vertices[first_index]),
                               transform_point(transform, geometry.payload->vertices[second_index]),
                               transform_point(transform, geometry.payload->vertices[third_index]),
                               distance) || distance >= best_distance)
            continue;
        const auto point = scale_add(ray.origin, ray.direction, distance);
        std::size_t enabled_planes = 0;
        bool clipped = false;
        for (const auto &plane : clip_planes) {
            if (!plane.enabled)
                continue;
            if (++enabled_planes > RenderPlan::max_clip_planes)
                break;
            if (plane.normal[0] * point.x + plane.normal[1] * point.y +
                    plane.normal[2] * point.z + plane.distance < 0.0f) {
                clipped = true;
                break;
            }
        }
        if (clipped)
            continue;
        best_distance = distance;
        result.node = node.node;
        result.source = node.source;
        result.subelement = {geometry.subelements->id_for_primitive(primitive)};
        result.worldPosition = point;
        result.depth = distance;
    }
}

template <class Visitor>
void visit_bounds(const std::vector<BvhNode> &nodes, const std::vector<Entry> &entries,
                  std::uint32_t node_index, const Bounds &query, Visitor &&visitor) {
    if (node_index == invalid_slot || !overlaps(nodes[node_index].bounds, query))
        return;
    const auto &node = nodes[node_index];
    if (node.leaf()) {
        for (std::uint32_t index = 0; index < node.count; ++index)
            if (overlaps(entries[node.first + index].bounds, query))
                visitor(entries[node.first + index]);
        return;
    }
    visit_bounds(nodes, entries, node.left, query, visitor);
    visit_bounds(nodes, entries, node.right, query, visitor);
}

template <class Visitor>
void visit_ray(const std::vector<BvhNode> &nodes, const std::vector<Entry> &entries,
               std::uint32_t node_index, const Ray &ray, Visitor &&visitor) {
    if (node_index == invalid_slot || !ray_hits_bounds(ray, nodes[node_index].bounds))
        return;
    const auto &node = nodes[node_index];
    if (node.leaf()) {
        for (std::uint32_t index = 0; index < node.count; ++index)
            if (ray_hits_bounds(ray, entries[node.first + index].bounds))
                visitor(entries[node.first + index]);
        return;
    }
    visit_ray(nodes, entries, node.left, ray, visitor);
    visit_ray(nodes, entries, node.right, ray, visitor);
}

template <class Visitor>
void visit_frustum(const std::vector<BvhNode> &nodes, const std::vector<Entry> &entries,
                   std::uint32_t node_index, std::span<const std::array<float, 4>> planes,
                   Visitor &&visitor) {
    if (node_index == invalid_slot || outside_planes(nodes[node_index].bounds, planes))
        return;
    const auto &node = nodes[node_index];
    if (node.leaf()) {
        for (std::uint32_t index = 0; index < node.count; ++index)
            if (!outside_planes(entries[node.first + index].bounds, planes))
                visitor(entries[node.first + index]);
        return;
    }
    visit_frustum(nodes, entries, node.left, planes, visitor);
    visit_frustum(nodes, entries, node.right, planes, visitor);
}

bool same_visibility_policy(const SceneView &left, const SceneView &right) noexcept {
    if (left.root != right.root || left.include_invisible != right.include_invisible ||
        left.visibility_overrides.size() != right.visibility_overrides.size() ||
        left.filter.isolated_sources.size() != right.filter.isolated_sources.size() ||
        left.filter.isolated_nodes.size() != right.filter.isolated_nodes.size() ||
        left.filter.source_visibility_overrides.size() !=
            right.filter.source_visibility_overrides.size())
        return false;
    for (std::size_t index = 0; index < left.visibility_overrides.size(); ++index) {
        const auto &a = left.visibility_overrides[index];
        const auto &b = right.visibility_overrides[index];
        if (a.node != b.node || a.visible != b.visible)
            return false;
    }
    for (std::size_t index = 0; index < left.filter.isolated_sources.size(); ++index)
        if (left.filter.isolated_sources[index] != right.filter.isolated_sources[index])
            return false;
    for (std::size_t index = 0; index < left.filter.isolated_nodes.size(); ++index)
        if (left.filter.isolated_nodes[index] != right.filter.isolated_nodes[index])
            return false;
    for (std::size_t index = 0; index < left.filter.source_visibility_overrides.size(); ++index) {
        const auto &a = left.filter.source_visibility_overrides[index];
        const auto &b = right.filter.source_visibility_overrides[index];
        if (a.source != b.source || a.visible != b.visible)
            return false;
    }
    return true;
}

} // namespace

struct SceneSpatialIndex::State {
    struct VisibilityCache {
        SceneView view;
        render_internal::EffectiveState effective;
    };

    struct PoseCache {
        std::vector<PoseOverride> source;
        std::unordered_map<NodeId, LocalTransform> transforms;
        std::vector<Entry> bounds;
    };

    explicit State(const SceneSnapshot &value, const SceneView *view)
        : snapshot(value), revision(value.revision()) {
        if (view)
            presentation = *view;
        if (view)
            for (const auto &pose : view->pose_overrides)
                pose_transforms.insert_or_assign(pose.node, pose.world_transform);
    }

    SceneSnapshot snapshot;
    std::uint64_t revision = 0;
    std::uint32_t incremental_refits = 0;
    std::vector<Entry> entries;
    std::vector<BvhNode> nodes;
    std::vector<std::uint32_t> entry_leaves;
    std::unordered_map<NodeId, std::size_t> entry_indices;
    std::unordered_map<NodeId, LocalTransform> pose_transforms;
    std::optional<SceneView> presentation;
    mutable std::optional<PoseCache> pose_cache;
    mutable std::optional<VisibilityCache> visibility_cache;
    mutable std::vector<NodeId> results;
};

SceneSpatialIndex::SceneSpatialIndex(const SceneSnapshot &snapshot, const SceneView *view)
    : state_(std::make_unique<State>(snapshot, view)) {
    state_->entries.reserve(snapshot.nodes().size());
    for (const auto &node : snapshot.nodes()) {
        const auto pose = state_->pose_transforms.find(node.node);
        if (pose == state_->pose_transforms.end()) {
            if (node.bounds.valid)
                state_->entries.push_back({node.node, node.bounds});
            continue;
        }
        const auto *geometry = snapshot.find_geometry(node.geometry);
        if (!geometry || !geometry->bounds.valid)
            continue;
        state_->entries.push_back({node.node, transformed_bounds(geometry->bounds, pose->second)});
    }
    const auto build_node = [&](auto &&self, std::size_t first, std::size_t last,
                                std::uint32_t parent) -> std::uint32_t {
        const auto node_index = static_cast<std::uint32_t>(state_->nodes.size());
        state_->nodes.emplace_back();
        state_->nodes[node_index].parent = parent;
        Bounds node_bounds;
        for (std::size_t index = first; index < last; ++index)
            node_bounds = merge_bounds(node_bounds, state_->entries[index].bounds);
        state_->nodes[node_index].bounds = node_bounds;
        const auto count = last - first;
        if (count <= leaf_capacity) {
            state_->nodes[node_index].first = static_cast<std::uint32_t>(first);
            state_->nodes[node_index].count = static_cast<std::uint32_t>(count);
            for (std::size_t index = first; index < last; ++index)
                state_->entry_leaves[index] = node_index;
            return node_index;
        }

        const auto axis = split_axis(node_bounds);
        const auto middle = first + count / 2;
        std::nth_element(state_->entries.begin() + static_cast<std::ptrdiff_t>(first),
                         state_->entries.begin() + static_cast<std::ptrdiff_t>(middle),
                         state_->entries.begin() + static_cast<std::ptrdiff_t>(last),
                         [axis](const Entry &lhs, const Entry &rhs) {
                             return centroid(lhs.bounds, axis) < centroid(rhs.bounds, axis);
                         });
        state_->nodes[node_index].left = self(self, first, middle, node_index);
        state_->nodes[node_index].right = self(self, middle, last, node_index);
        return node_index;
    };
    if (!state_->entries.empty()) {
        state_->entry_leaves.resize(state_->entries.size(), invalid_slot);
        build_node(build_node, 0, state_->entries.size(), invalid_slot);
        state_->entry_indices.reserve(state_->entries.size());
        for (std::size_t index = 0; index < state_->entries.size(); ++index)
            state_->entry_indices.emplace(state_->entries[index].node, index);
    }
}

SceneSpatialIndex::~SceneSpatialIndex() = default;
SceneSpatialIndex::SceneSpatialIndex(SceneSpatialIndex &&) noexcept = default;
SceneSpatialIndex &SceneSpatialIndex::operator=(SceneSpatialIndex &&) noexcept = default;

std::uint64_t SceneSpatialIndex::source_revision() const noexcept {
    return state_ ? state_->revision : 0;
}

bool SceneSpatialIndex::update_node_bounds(const SceneSnapshot &snapshot, NodeId node) noexcept {
    return update_node_bounds(snapshot, std::span<const NodeId>(&node, 1));
}

bool SceneSpatialIndex::update_node_bounds(const SceneSnapshot &snapshot,
                                           std::span<const NodeId> changed_nodes) noexcept {
    if (!state_ || !state_->pose_transforms.empty() ||
        (!changed_nodes.empty() && state_->incremental_refits >= max_incremental_refits))
        return false;
    const auto &before = state_->snapshot.revisions();
    const auto &after = snapshot.revisions();
    if (after.transform < before.transform || after.transform - before.transform > 1)
        return false;
    if (after.transform != before.transform && changed_nodes.empty())
        return false;
    if (after.bounds != before.bounds && changed_nodes.empty())
        return false;
    if (before.hierarchy != after.hierarchy || before.geometry != after.geometry)
        return false;
    for (const auto node : changed_nodes) {
        if (!node.valid() || state_->entry_indices.find(node) == state_->entry_indices.end())
            return false;
        const auto *updated = snapshot.find_node(node);
        if (!updated || !updated->bounds.valid)
            return false;
    }
    for (const auto node : changed_nodes) {
        const auto entry = state_->entry_indices.find(node);
        const auto *updated = snapshot.find_node(node);
        state_->entries[entry->second].bounds = updated->bounds;
        auto node_index = state_->entry_leaves[entry->second];
        while (node_index != invalid_slot) {
            auto &branch = state_->nodes[node_index];
            Bounds bounds;
            if (branch.leaf()) {
                for (std::uint32_t index = 0; index < branch.count; ++index)
                    bounds = merge_bounds(bounds,
                                          state_->entries[branch.first + index].bounds);
            } else {
                bounds = merge_bounds(state_->nodes[branch.left].bounds,
                                      state_->nodes[branch.right].bounds);
            }
            branch.bounds = bounds;
            node_index = branch.parent;
        }
    }
    state_->snapshot = snapshot;
    state_->revision = snapshot.revision();
    state_->pose_cache.reset();
    state_->visibility_cache.reset();
    if (!changed_nodes.empty())
        ++state_->incremental_refits;
    state_->results.clear();
    return true;
}

std::span<const NodeId> SceneSpatialIndex::query_bounds(const Bounds &bounds) const {
    if (!state_)
        return {};
    state_->results.clear();
    if (!bounds.valid || state_->nodes.empty())
        return {};
    visit_bounds(state_->nodes, state_->entries, 0, bounds,
                 [this](const Entry &entry) { state_->results.push_back(entry.node); });
    std::sort(state_->results.begin(), state_->results.end(),
              [](NodeId lhs, NodeId rhs) { return lhs.value < rhs.value; });
    return state_->results;
}

std::span<const NodeId>
SceneSpatialIndex::query_frustum(std::span<const std::array<float, 4>> planes) const {
    if (!state_)
        return {};
    state_->results.clear();
    if (state_->nodes.empty())
        return {};
    visit_frustum(state_->nodes, state_->entries, 0, planes,
                  [this](const Entry &entry) { state_->results.push_back(entry.node); });
    std::sort(state_->results.begin(), state_->results.end(),
              [](NodeId lhs, NodeId rhs) { return lhs.value < rhs.value; });
    return state_->results;
}

std::span<const NodeId> SceneSpatialIndex::query_ray(const Ray &ray) const {
    if (!state_)
        return {};
    state_->results.clear();
    if (state_->nodes.empty())
        return {};
    Ray normalized;
    if (!normalize_ray(ray, normalized))
        return {};
    visit_ray(state_->nodes, state_->entries, 0, normalized,
              [this](const Entry &entry) { state_->results.push_back(entry.node); });
    std::sort(state_->results.begin(), state_->results.end(),
              [](NodeId lhs, NodeId rhs) { return lhs.value < rhs.value; });
    return state_->results;
}

std::size_t SceneSpatialIndex::query_result_count() const noexcept {
    return state_ ? state_->results.size() : 0;
}

NodeId SceneSpatialIndex::query_result(std::size_t index) const noexcept {
    if (!state_ || index >= state_->results.size())
        return {};
    return state_->results[index];
}

PickResult SceneSpatialIndex::pick_ray(const Ray &ray) const {
    if (state_ && state_->presentation)
        return pick_ray(ray, *state_->presentation);
    PickResult result;
    Ray normalized;
    if (!normalize_ray(ray, normalized))
        return result;
    const auto candidates = query_ray(ray);
    auto best_distance = std::numeric_limits<float>::infinity();
    for (const auto node_id : candidates) {
        const auto *node = state_->snapshot.find(node_id);
        if (!node || !node->visible || !node->geometry.valid())
            continue;
        const auto *geometry = state_->snapshot.find_geometry(node->geometry);
        if (!geometry)
            continue;
        const auto pose = state_->pose_transforms.find(node_id);
        const auto &transform = pose == state_->pose_transforms.end()
                                    ? node->world_transform.transform
                                    : pose->second;
        intersect_triangles(normalized, *node, *geometry, transform, best_distance, result);
    }
    return result;
}

PickResult SceneSpatialIndex::pick_ray(const Ray &ray, const SceneView &view) const {
    PickResult result;
    Ray normalized;
    if (!state_ || !normalize_ray(ray, normalized))
        return result;

    if (!state_->visibility_cache ||
        !same_visibility_policy(state_->visibility_cache->view, view)) {
        State::VisibilityCache next{view, render_internal::effective_state(state_->snapshot, view)};
        state_->visibility_cache = std::move(next);
    }
    const auto &visibility = state_->visibility_cache->effective;
    const auto camera = render_internal::camera_for_snapshot(state_->snapshot, view);
    const auto matches_poses = [&] {
        if (!state_->pose_cache || state_->pose_cache->source.size() != view.pose_overrides.size())
            return false;
        for (std::size_t index = 0; index < view.pose_overrides.size(); ++index) {
            const auto &cached = state_->pose_cache->source[index];
            const auto &current = view.pose_overrides[index];
            if (cached.node != current.node ||
                cached.world_transform.matrix != current.world_transform.matrix)
                return false;
        }
        return true;
    };
    if (!matches_poses()) {
        State::PoseCache next;
        next.source = view.pose_overrides;
        next.transforms.reserve(view.pose_overrides.size());
        next.bounds.reserve(view.pose_overrides.size());
        for (const auto &pose : view.pose_overrides)
            next.transforms.insert_or_assign(pose.node, pose.world_transform);
        for (const auto &[node_id, transform] : next.transforms) {
            const auto *node = state_->snapshot.find(node_id);
            if (!node || !node->geometry.valid())
                continue;
            const auto *geometry = state_->snapshot.find_geometry(node->geometry);
            if (geometry && geometry->bounds.valid)
                next.bounds.push_back({node_id, transformed_bounds(geometry->bounds, transform)});
        }
        state_->pose_cache = std::move(next);
    }
    const auto &poses = state_->pose_cache->transforms;

    auto best_distance = std::numeric_limits<float>::infinity();
    const auto visit = [&](NodeId node_id, const LocalTransform *pose, const Bounds &bounds) {
        const auto visible = visibility.visible.find(node_id);
        const auto in_view = visibility.in_view.find(node_id);
        if (visible == visibility.visible.end() || !visible->second ||
            in_view == visibility.in_view.end() || !in_view->second ||
            render_internal::culled_by_camera(bounds, camera) ||
            render_internal::culled_by_clip_planes(bounds, view.clip_planes))
            return;
        const auto *node = state_->snapshot.find(node_id);
        if (!node || !node->geometry.valid() || !node->material.valid())
            return;
        const auto *geometry = state_->snapshot.find_geometry(node->geometry);
        if (!geometry || !state_->snapshot.find_material(node->material))
            return;
        const auto &transform = pose ? *pose : node->world_transform.transform;
        intersect_triangles(normalized, *node, *geometry, transform, best_distance, result,
                            view.clip_planes);
    };

    // Authored bounds stay in the retained BVH. Posed nodes use their presentation bounds
    // instead, so a moved object cannot be missed because its authored bounds are elsewhere.
    if (!state_->nodes.empty())
        visit_ray(state_->nodes, state_->entries, 0, normalized, [&](const Entry &entry) {
            if (!poses.contains(entry.node))
                visit(entry.node, nullptr, entry.bounds);
        });
    for (const auto &entry : state_->pose_cache->bounds) {
        if (!ray_hits_bounds(normalized, entry.bounds))
            continue;
        const auto pose = poses.find(entry.node);
        if (pose != poses.end())
            visit(entry.node, &pose->second, entry.bounds);
    }
    return result;
}

std::vector<PickResult> SceneSpatialIndex::pick_rays(std::span<const Ray> rays) const {
    std::vector<PickResult> results;
    results.reserve(rays.size());
    for (const auto &ray : rays)
        results.push_back(pick_ray(ray));
    return results;
}

} // namespace nkscene
