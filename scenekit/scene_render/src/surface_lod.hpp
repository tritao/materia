#pragma once

#include "nativekit_scene.hpp"
#include <span>
#include <vector>

namespace nkscene::render_internal {

// Display-only simplification. Original vertices, attributes, topology resources
// and picking data are never replaced. Error includes the weighted normal metric.
inline constexpr double surface_lod_pixel_error = 0.05;
inline constexpr std::size_t surface_lod_min_triangles = 8192;

struct SurfaceLod {
    std::vector<std::uint32_t> indices;
    float error = 0; // Simplification error in object-local units.
};

std::vector<SurfaceLod> build_surface_lods(const float *positions, const float *normals,
    std::size_t vertex_count, std::size_t vertex_stride, std::span<const std::uint32_t> indices);

// Upper bound on projected displacement per local unit throughout an expanded
// local AABB. Matches the surface shader's affine model transform, including
// nonuniform/reflected instances. Eye-plane crossings return infinity.
double surface_lod_pixel_scale(const Bounds &bounds, const std::array<float,16> &projection,
    const std::array<float,16> &model, unsigned width, unsigned height, double error_radius);

} // namespace nkscene::render_internal
