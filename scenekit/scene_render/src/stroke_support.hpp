#pragma once

#include "nativekit_scene.hpp"
#include <algorithm>
#include <array>
#include <functional>
#include <unordered_map>
#include <vector>

namespace nkscene::render_internal {

using StrokePoint = std::array<float, 3>;
struct StrokeSupportTriangle { StrokePoint third{}; bool reversed = false; };
struct StrokeSupport {
    std::array<StrokeSupportTriangle,2> triangles{};
    unsigned count = 0;
    auto begin() const { return triangles.begin(); }
    auto end() const { return triangles.begin()+count; }
};

// Match the render mesh itself, without a world-space welding tolerance. A
// support record preserves an incident triangle's third vertex and winding.
// Unmatched analytic strokes retain their original centerline depth.
inline std::vector<StrokeSupport> stroke_supports(const GeometryPayload &mesh) {
    if (mesh.stroke_segments.empty()) return {};
    using Key = std::array<StrokePoint, 2>;
    struct Hash {
        std::size_t operator()(const Key &key) const {
            std::size_t result = 0;
            for (const auto &point : key) for (float value : point)
                result ^= std::hash<float>{}(value) + 0x9e3779b9 + (result << 6) + (result >> 2);
            return result;
        }
    };
    struct Incident { std::array<StrokeSupportTriangle,2> triangles{}; unsigned count = 0; };
    const auto key_for = [](StrokePoint a, StrokePoint b) {
        if (b < a) std::swap(a, b);
        return Key{a, b};
    };
    std::unordered_map<Key, Incident, Hash> edges;
    edges.reserve(mesh.stroke_segments.size());
    for (const auto &stroke : mesh.stroke_segments)
        edges.try_emplace(key_for(stroke.start, stroke.end));
    const auto vertex = [&](std::size_t index) {
        return mesh.vertices[mesh.indices.empty() ? index : mesh.indices[index]].position;
    };
    for (std::size_t index = 0; index + 2 < mesh.element_count(); index += 3) {
        const StrokePoint p[3] = {vertex(index), vertex(index+1), vertex(index+2)};
        for (unsigned edge = 0; edge < 3; ++edge) {
            auto found = edges.find(key_for(p[edge], p[(edge+1)%3]));
            if (found == edges.end()) continue;
            auto &incident = found->second;
            if (incident.count < 2) incident.triangles[incident.count] = {p[(edge+2)%3], p[(edge+1)%3] < p[edge]};
            ++incident.count;
        }
    }
    std::vector<StrokeSupport> result;
    result.reserve(mesh.stroke_segments.size());
    for (const auto &stroke : mesh.stroke_segments) {
        StrokeSupport support;
        const auto &incident = edges.at(key_for(stroke.start, stroke.end));
        // Nonmanifold edges have no unambiguous pair of incident faces.
        if (incident.count <= 2) {
            support.triangles = incident.triangles;
            support.count = incident.count;
            for (unsigned index=0; index<support.count; ++index)
                support.triangles[index].reversed ^= stroke.end < stroke.start;
        }
        result.push_back(support);
    }
    return result;
}

} // namespace nkscene::render_internal
