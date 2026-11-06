#include "surface_lod.hpp"
#include "../third_party/meshoptimizer/meshoptimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace nkscene::render_internal {

std::vector<SurfaceLod> build_surface_lods(const float *positions, const float *normals,
    std::size_t vertex_count, std::size_t stride, std::span<const std::uint32_t> indices) {
    std::vector<SurfaceLod> levels;
    const auto count = indices.empty() ? vertex_count : indices.size();
    if (!positions || !normals || stride < 12 || stride % sizeof(float) ||
        count < surface_lod_min_triangles * 3 || count % 3 ||
        vertex_count > std::numeric_limits<std::uint32_t>::max()) return levels;
    for (std::size_t i = 0; i < vertex_count; ++i) {
        const auto *p = reinterpret_cast<const float *>(reinterpret_cast<const char *>(positions) + i*stride);
        const auto *n = reinterpret_cast<const float *>(reinterpret_cast<const char *>(normals) + i*stride);
        for (unsigned axis = 0; axis < 3; ++axis)
            if (!std::isfinite(p[axis]) || !std::isfinite(n[axis])) return levels;
    }
    for (const auto index : indices) if (index >= vertex_count) return levels;
    const auto scale = meshopt_simplifyScale(positions, vertex_count, stride);
    if (!(scale > 0) || !std::isfinite(scale)) return levels;
    std::vector<std::uint32_t> sequential;
    if (indices.empty()) {
        sequential.resize(vertex_count);
        std::iota(sequential.begin(), sequential.end(), 0u);
        indices = sequential;
    }
    // CAD triangle soups repeat vertices. Join exact position/normal matches in
    // the display-only index input, retaining original vertex representatives
    // and every normal discontinuity. Untextured, constant-colour eligibility
    // is checked by the executor before invoking this helper.
    const meshopt_Stream streams[] = {{positions,12,stride}, {normals,12,stride}};
    std::vector<std::uint32_t> welded(indices.size());
    meshopt_generateShadowIndexBufferMulti(welded.data(), indices.data(), indices.size(),
        vertex_count, streams, 2);
    indices = welded;
    constexpr float weights[] = {0.5f, 0.5f, 0.5f};
    const auto target = std::max<std::size_t>(384, count / 64 / 3 * 3);
    auto previous_count = count;
    for (const float tolerance : {0.0001f, 0.001f, 0.01f}) {
        SurfaceLod level;
        level.indices.resize(count); // The simplifier requires the full input capacity.
        float error = 0;
        // Flat-shaded CAD has a normal discontinuity at almost every triangle.
        // Permit attribute-aware collapses, bounded by the weighted normal and
        // position metric, while retaining open boundaries and source vertices.
        const auto reduced = meshopt_simplifyWithAttributes(level.indices.data(), indices.data(), count,
            positions, vertex_count, stride, normals, stride, weights, 3, nullptr, target,
            tolerance, meshopt_SimplifyLockBorder | meshopt_SimplifyPermissive, &error);
        if (reduced < 3 || reduced >= previous_count * 0.8 || !std::isfinite(error)) continue;
        level.indices.resize(reduced);
        level.error = error * scale;
        previous_count = reduced;
        levels.push_back(std::move(level));
    }
    return levels;
}

double surface_lod_pixel_scale(const Bounds &bounds, const std::array<float,16> &projection,
    const std::array<float,16> &model, unsigned width, unsigned height, double error_radius) {
    const auto infinity = std::numeric_limits<double>::infinity();
    if (!bounds.valid || !width || !height || !(error_radius >= 0)) return infinity;
    std::array<double,16> clip{};
    // The shader computes world.xyz from model, then projects vec4(world.xyz, 1).
    for (unsigned column = 0; column < 4; ++column)
        for (unsigned row = 0; row < 4; ++row) {
            clip[column*4+row] = column == 3 ? projection[12+row] : 0;
            for (unsigned k = 0; k < 3; ++k)
                clip[column*4+row] += double(projection[k*4+row]) * model[column*4+k];
        }
    double min_w = infinity, max_x = 0, max_y = 0;
    for (unsigned corner = 0; corner < 8; ++corner) {
        double value[4] = {clip[12], clip[13], clip[14], clip[15]};
        for (unsigned axis = 0; axis < 3; ++axis) {
            const auto coordinate = (corner & (1u << axis)) ? bounds.maximum[axis] : bounds.minimum[axis];
            for (unsigned row = 0; row < 4; ++row) value[row] += clip[axis*4+row] * coordinate;
        }
        if (!std::isfinite(value[0]) || !std::isfinite(value[1]) || !std::isfinite(value[3])) return infinity;
        min_w = std::min(min_w, value[3]);
        max_x = std::max(max_x, std::abs(value[0]));
        max_y = std::max(max_y, std::abs(value[1]));
    }
    const auto row_norm = [&](unsigned row) { return std::hypot(clip[row], clip[4+row], clip[8+row]); };
    const auto nx = row_norm(0), ny = row_norm(1), nw = row_norm(3);
    min_w -= error_radius * nw;
    max_x += error_radius * nx;
    max_y += error_radius * ny;
    if (!(min_w > 0) || !std::isfinite(min_w)) return infinity;
    const auto dx = width * 0.5 * (nx/min_w + max_x*nw/(min_w*min_w));
    const auto dy = height * 0.5 * (ny/min_w + max_y*nw/(min_w*min_w));
    return std::hypot(dx,dy);
}

} // namespace nkscene::render_internal
