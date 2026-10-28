// C ABI for ring stock and rake tools. Handle tables and helpers are shared with api.cpp.
#include "api_internal.hpp"

#include <algorithm>
#include <cmath>

using namespace stockkit;
using namespace stockkit::api;

namespace {

/** Rake poses must be finite rigid transforms: orthonormal rotation, no reflection. */
bool valid_poses(const sk_rake_pose *poses, uint32_t count) {
    for (uint32_t k = 0; k < count; ++k) {
        const sk_rake_pose &p = poses[k];
        if (p.struct_size != sizeof(p) || !finite(p.rotation, 9) || !finite(p.translation, 3)) return false;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) {
                double dot = 0;
                for (int c = 0; c < 3; ++c) dot += p.rotation[3 * a + c] * p.rotation[3 * b + c];
                if (std::abs(dot - (a == b ? 1.0 : 0.0)) > 1e-9) return false;
            }
        double det = p.rotation[0] * (p.rotation[4] * p.rotation[8] - p.rotation[5] * p.rotation[7])
            - p.rotation[1] * (p.rotation[3] * p.rotation[8] - p.rotation[5] * p.rotation[6])
            + p.rotation[2] * (p.rotation[3] * p.rotation[7] - p.rotation[4] * p.rotation[6]);
        if (det < 0) return false;
    }
    return true;
}

} // namespace

SK_API sk_result SK_CALL sk_ring_stock_create(const sk_ring_grid *grid, sk_ring_stock_handle *out_stock) {
    return guarded([&]() -> sk_result {
        if (!out_stock) return SK_ERROR_INVALID_ARGUMENT;
        out_stock->id = 0;
        if (!grid || !RingStock::valid(*grid)) return SK_ERROR_INVALID_ARGUMENT;
        auto stock = std::make_unique<RingStock>(*grid);
        uint32_t id = ring_stocks().add(std::move(stock));
        if (!id) return SK_ERROR_LIMIT;
        out_stock->id = id;
        return SK_OK;
    });
}

SK_API void SK_CALL sk_ring_stock_destroy(sk_ring_stock_handle stock) {
    guarded([&]() -> sk_result {
        ring_stocks().remove(stock.id);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_set_threads(sk_ring_stock_handle stock, uint32_t threads) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        if (threads > 128) return SK_ERROR_INVALID_ARGUMENT;
        s->pool.resize(threads == 0 ? std::min(128u, Pool::hardware()) : threads);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_get_info(sk_ring_stock_handle stock, sk_ring_info *out_info) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        if (!out_info) return SK_ERROR_INVALID_ARGUMENT;
        *out_info = s->info();
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_read_circle(sk_ring_stock_handle stock, uint32_t r, uint32_t z,
    sk_ring_interval *out, uint32_t *capacity) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        if (!capacity || r >= s->grid.radial_count || z >= s->grid.axial_count) return SK_ERROR_INVALID_ARGUMENT;
        const auto &c = s->circle(r, z);
        auto room = *capacity;
        *capacity = uint32_t(c.size());
        if (room < c.size() || (!out && !c.empty())) return SK_ERROR_LIMIT;
        if (!c.empty()) std::memcpy(out, c.data(), c.size() * sizeof(c[0]));
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_cut_region(sk_ring_stock_handle stock, const sk_ring_cut *cuts, uint32_t count) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        if (!cuts && count) return SK_ERROR_INVALID_ARGUMENT;
        for (uint32_t k = 0; k < count; ++k)
            if (!s->valid_cut(cuts[k])) return SK_ERROR_INVALID_ARGUMENT;
        s->cut(cuts, count);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_snapshot(sk_ring_stock_handle stock, sk_ring_snapshot_handle *out_snapshot) {
    return guarded([&]() -> sk_result {
        if (!out_snapshot) return SK_ERROR_INVALID_ARGUMENT;
        out_snapshot->id = 0;
        auto *s = ring_stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        auto shot = std::make_unique<RingSnapshot>();
        shot->grid = s->grid;
        shot->layers = s->layers;
        auto id = ring_snapshots().add(std::move(shot));
        if (!id) return SK_ERROR_LIMIT;
        out_snapshot->id = id;
        return SK_OK;
    });
}

SK_API void SK_CALL sk_ring_snapshot_destroy(sk_ring_snapshot_handle snapshot) {
    guarded([&]() -> sk_result {
        ring_snapshots().remove(snapshot.id);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_restore(sk_ring_stock_handle stock, sk_ring_snapshot_handle snapshot) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        auto *shot = ring_snapshots().get(snapshot.id);
        if (!s || !shot) return SK_ERROR_INVALID_HANDLE;
        if (!s->same_grid(shot->grid)) return SK_ERROR_INVALID_ARGUMENT;
        auto layers = shot->layers;
        s->layers.swap(layers);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_compare(sk_ring_stock_handle stock, sk_ring_stock_handle target,
    sk_ring_comparison *out, uint32_t count) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        auto *t = ring_stocks().get(target.id);
        if (!s || !t) return SK_ERROR_INVALID_HANDLE;
        if (!s->same_grid(t->grid)) return SK_ERROR_INVALID_ARGUMENT;
        if (count != s->grid.radial_count * s->grid.axial_count || !out) return SK_ERROR_INVALID_ARGUMENT;
        ring_compare(*s, *t, out);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_contour(sk_ring_stock_handle stock, double max_angle, sk_mesh_handle *out_mesh) {
    return guarded([&]() -> sk_result {
        if (!out_mesh) return SK_ERROR_INVALID_ARGUMENT;
        out_mesh->id = 0;
        auto *s = ring_stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        if (!std::isfinite(max_angle) || max_angle < 1e-4 || max_angle > 1.5707963267948966)
            return SK_ERROR_INVALID_ARGUMENT;
        auto mesh = std::make_unique<MeshEntry>();
        s->contour(max_angle, mesh->mesh);
        auto id = meshes().add(std::move(mesh));
        if (!id) return SK_ERROR_LIMIT;
        out_mesh->id = id;
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_cut_slab(sk_ring_stock_handle stock, double angle,
    double lo, double hi, uint32_t gash, uint32_t pass) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        if (!std::isfinite(angle) || !std::isfinite(lo) || !std::isfinite(hi) || lo > hi ||
            gash >= SK_SOURCE_NONE || pass >= SK_SOURCE_NONE) return SK_ERROR_INVALID_ARGUMENT;
        constexpr double tau = 6.28318530717958647692;
        angle = std::fmod(angle, tau);
        std::vector<sk_ring_cut> cuts;
        for (uint32_t r = 0; r < s->grid.radial_count; ++r) {
            double radius = s->grid.radius_min + (r + 0.5) * (s->grid.radius_max - s->grid.radius_min) / s->grid.radial_count;
            if (lo >= radius || hi <= -radius || lo == hi) continue;
            double a = std::acos(std::clamp(hi / radius, -1.0, 1.0));
            double b = std::acos(std::clamp(lo / radius, -1.0, 1.0));
            cuts.push_back({sizeof(sk_ring_cut), r, 0, s->grid.axial_count, angle + a, b - a, gash, pass});
            cuts.push_back({sizeof(sk_ring_cut), r, 0, s->grid.axial_count, angle - b, b - a, gash, pass});
        }
        s->cut(cuts.data(), uint32_t(cuts.size()));
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_rake_tool_create(const double *vertices, uint32_t coordinate_count,
    const sk_rake_region *regions, uint32_t region_count, uint32_t gashes, sk_rake_tool_handle *out_tool) {
    return guarded([&]() -> sk_result {
        if (!out_tool) return SK_ERROR_INVALID_ARGUMENT;
        out_tool->id = 0;
        if (coordinate_count % 3) return SK_ERROR_INVALID_ARGUMENT;
        auto tool = std::make_unique<RakeTool>();
        if (!build_rake_tool(vertices, coordinate_count / 3, regions, region_count, gashes, *tool))
            return SK_ERROR_INVALID_ARGUMENT;
        auto id = rake_tools().add(std::move(tool));
        if (!id) return SK_ERROR_LIMIT;
        out_tool->id = id;
        return SK_OK;
    });
}

SK_API void SK_CALL sk_rake_tool_destroy(sk_rake_tool_handle tool) {
    guarded([&]() -> sk_result {
        rake_tools().remove(tool.id);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_sweep_rake(sk_ring_stock_handle stock, sk_rake_tool_handle tool,
    uint32_t gash, uint32_t pass, const sk_rake_pose *poses, uint32_t pose_count) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        auto *t = rake_tools().get(tool.id);
        if (!s || !t) return SK_ERROR_INVALID_HANDLE;
        if (gash >= t->gashes.size() || pass >= SK_SOURCE_NONE || !poses || pose_count < 2 ||
            !valid_poses(poses, pose_count)) return SK_ERROR_INVALID_ARGUMENT;
        sweep_rake(*s, *t, gash, pass, poses, pose_count);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_ring_stock_sweep_rake_pair(sk_ring_stock_handle stock, sk_ring_stock_handle second,
    sk_rake_tool_handle tool, uint32_t gash, uint32_t pass, const sk_rake_pose *poses, uint32_t pose_count) {
    return guarded([&]() -> sk_result {
        auto *s = ring_stocks().get(stock.id);
        auto *other = ring_stocks().get(second.id);
        auto *t = rake_tools().get(tool.id);
        if (!s || !other || !t) return SK_ERROR_INVALID_HANDLE;
        if (s == other) return SK_ERROR_INVALID_ARGUMENT;
        if (gash >= t->gashes.size() || pass >= SK_SOURCE_NONE || !poses || pose_count < 2 ||
            !valid_poses(poses, pose_count)) return SK_ERROR_INVALID_ARGUMENT;
        auto before_first = s->layers, before_second = other->layers;
        try {
            sweep_rake_pair(*s, *other, *t, gash, pass, poses, pose_count);
        } catch (...) {
            s->layers = std::move(before_first);
            other->layers = std::move(before_second);
            throw;
        }
        return SK_OK;
    });
}
