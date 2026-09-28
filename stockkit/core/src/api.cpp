#include "stockkit.h"

#include "mesh.hpp"
#include "preview.hpp"
#include "profile.hpp"
#include "stock.hpp"
#include "sweep.hpp"
#include "tool.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <vector>

namespace {

using namespace stockkit;

// Handle layout: kind in the top 4 bits, then a 12-bit generation, then a
// 16-bit slot. Slot 0 is never used, so a zero handle is always invalid. A
// slot retires when its generation is exhausted instead of wrapping around.
constexpr uint32_t kKindTool = 1, kKindStock = 2, kKindSnapshot = 3, kKindMesh = 4;
constexpr uint32_t kGenerationMask = 0xFFF;

template <typename T, uint32_t Kind>
class Table {
public:
    uint32_t add(std::unique_ptr<T> value) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t slot = 0;
        if (!free_.empty()) {
            slot = free_.back();
            free_.pop_back();
        } else {
            if (entries_.size() >= 0xFFFF) return 0;
            entries_.push_back({});
            slot = static_cast<uint32_t>(entries_.size());
        }
        Entry &entry = entries_[slot - 1];
        entry.value = std::move(value);
        return (Kind << 28) | (entry.generation << 16) | slot;
    }

    T *get(uint32_t handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        Entry *entry = find(handle);
        return entry ? entry->value.get() : nullptr;
    }

    void remove(uint32_t handle) {
        std::unique_ptr<T> doomed;
        std::lock_guard<std::mutex> lock(mutex_);
        Entry *entry = find(handle);
        if (!entry) return;
        doomed = std::move(entry->value);
        if (entry->generation < kGenerationMask) {
            ++entry->generation;
            free_.push_back(handle & 0xFFFF);
        }
    }

private:
    struct Entry {
        uint32_t generation = 1;
        std::unique_ptr<T> value;
    };

    Entry *find(uint32_t handle) {
        uint32_t slot = handle & 0xFFFF;
        if ((handle >> 28) != Kind || slot == 0 || slot > entries_.size()) return nullptr;
        Entry &entry = entries_[slot - 1];
        if (entry.generation != ((handle >> 16) & kGenerationMask) || !entry.value) return nullptr;
        return &entry;
    }

    std::mutex mutex_;
    std::vector<Entry> entries_;
    std::vector<uint32_t> free_;
};

Table<Tool, kKindTool> &tools() {
    static Table<Tool, kKindTool> table;
    return table;
}

Table<Stock, kKindStock> &stocks() {
    static Table<Stock, kKindStock> table;
    return table;
}

Table<Stock::Snapshot, kKindSnapshot> &snapshots() {
    static Table<Stock::Snapshot, kKindSnapshot> table;
    return table;
}

/** A preview mesh and the grid size its ray indices refer to. */
struct MeshEntry {
    PreviewMesh mesh;
    uint64_t rays = 0;
};

Table<MeshEntry, kKindMesh> &meshes() {
    static Table<MeshEntry, kKindMesh> table;
    return table;
}

template <typename T>
sk_result copy_bytes(const std::vector<T> &values, uint8_t *output, uint32_t *byte_capacity) {
    if (!byte_capacity) return SK_ERROR_INVALID_ARGUMENT;
    if (values.size() > 0xFFFFFFFFu / sizeof(T)) return SK_ERROR_LIMIT;
    uint32_t required = uint32_t(values.size() * sizeof(T));
    uint32_t capacity = *byte_capacity;
    *byte_capacity = required;
    if (capacity < required || (required != 0 && !output)) return SK_ERROR_LIMIT;
    if (required != 0) std::memcpy(output, values.data(), required);
    return SK_OK;
}

template <typename F>
sk_result guarded(F &&body) {
    try {
        return body();
    } catch (const std::bad_alloc &) {
        return SK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return SK_ERROR_INTERNAL;
    }
}

bool finite(const double *values, int n) {
    for (int k = 0; k < n; ++k)
        if (!std::isfinite(values[k])) return false;
    return true;
}

sk_result to_grid(const sk_grid *in, Grid &out) {
    if (!in || in->struct_size < sizeof(sk_grid)) return SK_ERROR_INVALID_ARGUMENT;
    if (in->axis > SK_AXIS_Z) return SK_ERROR_INVALID_ARGUMENT;
    if (in->axis != SK_AXIS_Z) return SK_ERROR_UNSUPPORTED;
    if (!finite(in->origin, 2) || !std::isfinite(in->spacing) || !(in->spacing > 0))
        return SK_ERROR_INVALID_ARGUMENT;
    if (in->count[0] == 0 || in->count[1] == 0) return SK_ERROR_INVALID_ARGUMENT;
    if (uint64_t(in->count[0]) * in->count[1] > (uint64_t(1) << 32)) return SK_ERROR_LIMIT;
    if (in->tile_size > 1024) return SK_ERROR_INVALID_ARGUMENT;
    out.axis = in->axis;
    out.origin[0] = in->origin[0];
    out.origin[1] = in->origin[1];
    out.spacing = in->spacing;
    out.count[0] = in->count[0];
    out.count[1] = in->count[1];
    out.tile = in->tile_size == 0 ? 16 : in->tile_size;
    return SK_OK;
}

sk_result to_motion(const sk_move *in, Motion &out) {
    if (!in || in->struct_size < sizeof(sk_move)) return SK_ERROR_INVALID_ARGUMENT;
    if (in->kind == SK_MOVE_LINE) {
        if (!finite(in->start, 3) || !finite(in->end, 3)) return SK_ERROR_INVALID_ARGUMENT;
        out = Motion{};
        for (int k = 0; k < 3; ++k) {
            out.p0[k] = in->start[k];
            out.p1[k] = in->end[k];
        }
        return SK_OK;
    }
    if (in->kind == SK_MOVE_ARC) {
        double values[] = {in->center[0], in->center[1], in->center[2], in->radius, in->start_angle,
            in->sweep, in->rise};
        if (!finite(values, 7) || in->radius < 0) return SK_ERROR_INVALID_ARGUMENT;
        out = Motion{};
        out.arc = true;
        out.cx = in->center[0];
        out.cy = in->center[1];
        out.z0 = in->center[2];
        out.radius = in->radius;
        out.start_angle = in->start_angle;
        out.sweep = in->sweep;
        out.rise = in->rise;
        return SK_OK;
    }
    return SK_ERROR_INVALID_ARGUMENT;
}

void to_interval(const Interval &in, sk_interval &out) {
    out.struct_size = sizeof(sk_interval);
    out.lo = in.lo;
    out.hi = in.hi;
    for (int k = 0; k < 3; ++k) {
        out.lo_normal[k] = in.lo_normal[k];
        out.hi_normal[k] = in.hi_normal[k];
    }
    out.lo_source = in.lo_source;
    out.hi_source = in.hi_source;
}

} // namespace

extern "C" {

SK_API uint32_t SK_CALL sk_api_version(void) { return SK_API_VERSION; }

SK_API sk_result SK_CALL sk_tool_create(const sk_profile_segment *segments, uint32_t segment_count,
    sk_tool_handle *out_tool) {
    return guarded([&]() -> sk_result {
        if (!out_tool) return SK_ERROR_INVALID_ARGUMENT;
        out_tool->id = 0;
        if (!segments || segment_count == 0) return SK_ERROR_INVALID_ARGUMENT;
        std::vector<Segment> list;
        std::vector<uint32_t> zones;
        for (uint32_t k = 0; k < segment_count; ++k) {
            const sk_profile_segment &in = segments[k];
            if (in.struct_size < sizeof(sk_profile_segment) || in.kind > SK_SEGMENT_ARC || in.zone >= SK_ZONE_COUNT)
                return SK_ERROR_INVALID_ARGUMENT;
            list.push_back({in.kind == SK_SEGMENT_ARC, in.r0, in.z0, in.r1, in.z1, in.center_r, in.center_z});
            zones.push_back(in.zone);
        }
        auto profile = std::make_unique<Tool>();
        std::string error;
        if (!Tool::build(list, zones, *profile, error)) return SK_ERROR_INVALID_ARGUMENT;
        uint32_t id = tools().add(std::move(profile));
        if (id == 0) return SK_ERROR_LIMIT;
        out_tool->id = id;
        return SK_OK;
    });
}

SK_API void SK_CALL sk_tool_destroy(sk_tool_handle tool) {
    guarded([&]() -> sk_result {
        tools().remove(tool.id);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_tool_get_info(sk_tool_handle tool, sk_tool_info *out_info) {
    return guarded([&]() -> sk_result {
        if (!out_info) return SK_ERROR_INVALID_ARGUMENT;
        Tool *t = tools().get(tool.id);
        if (!t) return SK_ERROR_INVALID_HANDLE;
        out_info->struct_size = sizeof(sk_tool_info);
        out_info->cutting_radius = t->cutting.radius();
        out_info->cutting_height = t->cutting.height();
        out_info->radius = t->cutting.radius();
        out_info->height = t->cutting.height();
        for (const Band &band : t->bands) {
            out_info->radius = std::max(out_info->radius, band.profile.radius());
            out_info->height = std::max(out_info->height, band.profile.height());
        }
        return SK_OK;
    });
}

namespace {

sk_result sweep_ray(sk_tool_handle tool, const sk_move *move, uint32_t axis, double u, double v,
    std::vector<Span> &spans) {
    Tool *t = tools().get(tool.id);
    if (!t) return SK_ERROR_INVALID_HANDLE;
    Motion motion;
    sk_result result = to_motion(move, motion);
    if (result != SK_OK) return result;
    if (axis > SK_AXIS_Z || !std::isfinite(u) || !std::isfinite(v)) return SK_ERROR_INVALID_ARGUMENT;
    if (axis != SK_AXIS_Z) return SK_ERROR_UNSUPPORTED;
    SweptVolume(t->cutting, motion).intersect_z(u, v, spans);
    return SK_OK;
}

} // namespace

SK_API sk_result SK_CALL sk_sweep_count_ray(sk_tool_handle tool, const sk_move *move, uint32_t axis, double u,
    double v, uint32_t *out_count) {
    return guarded([&]() -> sk_result {
        if (!out_count) return SK_ERROR_INVALID_ARGUMENT;
        *out_count = 0;
        std::vector<Span> spans;
        sk_result result = sweep_ray(tool, move, axis, u, v, spans);
        if (result == SK_OK) *out_count = static_cast<uint32_t>(spans.size());
        return result;
    });
}

SK_API sk_result SK_CALL sk_sweep_read_ray(sk_tool_handle tool, const sk_move *move, uint32_t axis, double u,
    double v, sk_interval *out_intervals, uint32_t capacity) {
    return guarded([&]() -> sk_result {
        if (capacity > 0 && !out_intervals) return SK_ERROR_INVALID_ARGUMENT;
        std::vector<Span> spans;
        sk_result result = sweep_ray(tool, move, axis, u, v, spans);
        if (result != SK_OK) return result;
        if (spans.size() > capacity) return SK_ERROR_LIMIT;
        for (size_t k = 0; k < spans.size(); ++k) {
            Interval interval;
            interval.lo = spans[k].lo;
            interval.hi = spans[k].hi;
            for (int c = 0; c < 3; ++c) {
                interval.lo_normal[c] = spans[k].lo_normal[c];
                interval.hi_normal[c] = spans[k].hi_normal[c];
            }
            interval.lo_source = interval.hi_source = move->source;
            to_interval(interval, out_intervals[k]);
        }
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_create_box(const sk_grid *grid, const sk_box *box, sk_stock_handle *out_stock) {
    return guarded([&]() -> sk_result {
        if (!out_stock) return SK_ERROR_INVALID_ARGUMENT;
        out_stock->id = 0;
        Grid g;
        sk_result result = to_grid(grid, g);
        if (result != SK_OK) return result;
        if (!box || box->struct_size < sizeof(sk_box) || !finite(box->min, 3) || !finite(box->max, 3))
            return SK_ERROR_INVALID_ARGUMENT;
        for (int k = 0; k < 3; ++k)
            if (!(box->max[k] > box->min[k])) return SK_ERROR_INVALID_ARGUMENT;
        auto stock = std::make_unique<Stock>(g);
        std::vector<Interval> ray(1);
        ray[0].lo = box->min[2];
        ray[0].hi = box->max[2];
        const float down[3] = {0, 0, -1}, up[3] = {0, 0, 1};
        std::copy_n(down, 3, ray[0].lo_normal);
        std::copy_n(up, 3, ray[0].hi_normal);
        ray[0].lo_source = ray[0].hi_source = kSourceStock;
        const std::vector<Interval> empty;
        // Rays on the box's sides are inside it: the box is closed.
        for (uint32_t j = 0; j < g.count[1]; ++j)
            for (uint32_t i = 0; i < g.count[0]; ++i) {
                double x = stock->ray_u(i), y = stock->ray_v(j);
                bool inside = x >= box->min[0] && x <= box->max[0] && y >= box->min[1] && y <= box->max[1];
                if (inside) stock->write(i, j, ray);
            }
        stock->pack();
        uint32_t id = stocks().add(std::move(stock));
        if (id == 0) return SK_ERROR_LIMIT;
        out_stock->id = id;
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_create_mesh(const sk_grid *grid, const double *positions,
    uint32_t position_count, const uint32_t *indices, uint32_t index_count, sk_stock_handle *out_stock) {
    return guarded([&]() -> sk_result {
        if (!out_stock) return SK_ERROR_INVALID_ARGUMENT;
        out_stock->id = 0;
        Grid g;
        sk_result result = to_grid(grid, g);
        if (result != SK_OK) return result;
        if ((position_count > 0 && !positions) || (index_count > 0 && !indices)) return SK_ERROR_INVALID_ARGUMENT;
        auto stock = std::make_unique<Stock>(g);
        std::string error;
        if (!cast_mesh_z(*stock, positions, position_count, indices, index_count, error))
            return SK_ERROR_INVALID_ARGUMENT;
        stock->pack();
        uint32_t id = stocks().add(std::move(stock));
        if (id == 0) return SK_ERROR_LIMIT;
        out_stock->id = id;
        return SK_OK;
    });
}

SK_API void SK_CALL sk_stock_destroy(sk_stock_handle stock) {
    guarded([&]() -> sk_result {
        stocks().remove(stock.id);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_get_info(sk_stock_handle stock, sk_stock_info *out_info) {
    return guarded([&]() -> sk_result {
        if (!out_info) return SK_ERROR_INVALID_ARGUMENT;
        Stock *s = stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        const Grid &g = s->grid();
        out_info->struct_size = sizeof(sk_stock_info);
        out_info->grid.struct_size = sizeof(sk_grid);
        out_info->grid.axis = g.axis;
        out_info->grid.origin[0] = g.origin[0];
        out_info->grid.origin[1] = g.origin[1];
        out_info->grid.spacing = g.spacing;
        out_info->grid.count[0] = g.count[0];
        out_info->grid.count[1] = g.count[1];
        out_info->grid.tile_size = g.tile;
        out_info->interval_count = s->interval_count();
        out_info->volume = s->volume();
        out_info->bytes = s->bytes();
        const CutStats &stats = s->stats();
        out_info->rays_tested = stats.rays_tested;
        out_info->rays_changed = stats.rays_changed;
        out_info->tiles_skipped = stats.tiles_skipped;
        out_info->threads = s->threads();
        out_info->tiles[0] = s->tiles_across();
        out_info->tiles[1] = s->tiles_down();
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_read_revisions(sk_stock_handle stock, uint64_t *out_revisions,
    uint32_t revision_capacity) {
    return guarded([&]() -> sk_result {
        Stock *s = stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        const std::vector<uint64_t> &revisions = s->revisions();
        if (revision_capacity < revisions.size() || (revision_capacity > 0 && !out_revisions))
            return SK_ERROR_INVALID_ARGUMENT;
        std::copy(revisions.begin(), revisions.end(), out_revisions);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_snapshot(sk_stock_handle stock, sk_snapshot_handle *out_snapshot) {
    return guarded([&]() -> sk_result {
        if (!out_snapshot) return SK_ERROR_INVALID_ARGUMENT;
        out_snapshot->id = 0;
        Stock *s = stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        uint32_t id = snapshots().add(s->snapshot());
        if (id == 0) return SK_ERROR_LIMIT;
        out_snapshot->id = id;
        return SK_OK;
    });
}

SK_API void SK_CALL sk_snapshot_destroy(sk_snapshot_handle snapshot) {
    guarded([&]() -> sk_result {
        snapshots().remove(snapshot.id);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_restore(sk_stock_handle stock, sk_snapshot_handle snapshot) {
    return guarded([&]() -> sk_result {
        Stock *s = stocks().get(stock.id);
        Stock::Snapshot *shot = snapshots().get(snapshot.id);
        if (!s || !shot) return SK_ERROR_INVALID_HANDLE;
        return s->restore(*shot) ? SK_OK : SK_ERROR_INVALID_ARGUMENT;
    });
}

SK_API sk_result SK_CALL sk_stock_set_threads(sk_stock_handle stock, uint32_t threads) {
    return guarded([&]() -> sk_result {
        Stock *s = stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        if (threads > 1024) return SK_ERROR_INVALID_ARGUMENT;
        s->set_threads(threads);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_cut(sk_stock_handle stock, sk_tool_handle tool, const sk_move *moves,
    uint32_t move_count, sk_move_result *out_results, uint32_t result_capacity) {
    return guarded([&]() -> sk_result {
        if ((move_count > 0 && (!moves || !out_results)) || result_capacity < move_count)
            return SK_ERROR_INVALID_ARGUMENT;
        Stock *s = stocks().get(stock.id);
        Tool *t = tools().get(tool.id);
        if (!s || !t) return SK_ERROR_INVALID_HANDLE;
        // Validate everything first so a bad move leaves the stock untouched.
        std::vector<Motion> motions(move_count);
        for (uint32_t k = 0; k < move_count; ++k) {
            sk_result result = to_motion(&moves[k], motions[k]);
            if (result != SK_OK) return result;
        }
        std::vector<MoveSweep> sweeps;
        sweeps.reserve(move_count);
        for (uint32_t k = 0; k < move_count; ++k) sweeps.emplace_back(*t, motions[k], moves[k].source);
        std::vector<MoveResult> results(move_count);
        s->cut(sweeps, results.data());
        for (uint32_t k = 0; k < move_count; ++k) {
            out_results[k] = sk_move_result{};
            out_results[k].struct_size = sizeof(sk_move_result);
            out_results[k].removed = results[k].removed;
            for (uint32_t z = 0; z < SK_ZONE_COUNT; ++z) out_results[k].contact[z] = results[k].contact[z];
        }
        return SK_OK;
    });
}

namespace {

sk_result check_block(Stock *s, uint32_t i0, uint32_t j0, uint32_t ni, uint32_t nj) {
    if (!s) return SK_ERROR_INVALID_HANDLE;
    const Grid &g = s->grid();
    if (uint64_t(i0) + ni > g.count[0] || uint64_t(j0) + nj > g.count[1]) return SK_ERROR_INVALID_ARGUMENT;
    return SK_OK;
}

} // namespace

SK_API sk_result SK_CALL sk_stock_read_counts(sk_stock_handle stock, uint32_t i0, uint32_t j0, uint32_t ni,
    uint32_t nj, uint32_t *out_counts, uint32_t count_capacity) {
    return guarded([&]() -> sk_result {
        Stock *s = stocks().get(stock.id);
        sk_result result = check_block(s, i0, j0, ni, nj);
        if (result != SK_OK) return result;
        if (count_capacity < uint64_t(ni) * nj || (count_capacity > 0 && !out_counts)) return SK_ERROR_INVALID_ARGUMENT;
        for (uint32_t j = 0; j < nj; ++j)
            for (uint32_t i = 0; i < ni; ++i) out_counts[size_t(j) * ni + i] = s->count(i0 + i, j0 + j);
        return SK_OK;
    });
}

namespace {

uint64_t block_total(Stock *s, uint32_t i0, uint32_t j0, uint32_t ni, uint32_t nj) {
    uint64_t total = 0;
    for (uint32_t j = 0; j < nj; ++j)
        for (uint32_t i = 0; i < ni; ++i) total += s->count(i0 + i, j0 + j);
    return total;
}

} // namespace

SK_API sk_result SK_CALL sk_stock_count_intervals(sk_stock_handle stock, uint32_t i0, uint32_t j0, uint32_t ni,
    uint32_t nj, uint32_t *out_total) {
    return guarded([&]() -> sk_result {
        if (!out_total) return SK_ERROR_INVALID_ARGUMENT;
        *out_total = 0;
        Stock *s = stocks().get(stock.id);
        sk_result result = check_block(s, i0, j0, ni, nj);
        if (result != SK_OK) return result;
        uint64_t total = block_total(s, i0, j0, ni, nj);
        if (total > 0xFFFFFFFFu) return SK_ERROR_LIMIT;
        *out_total = static_cast<uint32_t>(total);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_read_intervals(sk_stock_handle stock, uint32_t i0, uint32_t j0, uint32_t ni,
    uint32_t nj, sk_interval *out_intervals, uint32_t interval_capacity) {
    return guarded([&]() -> sk_result {
        Stock *s = stocks().get(stock.id);
        sk_result result = check_block(s, i0, j0, ni, nj);
        if (result != SK_OK) return result;
        uint64_t total = block_total(s, i0, j0, ni, nj);
        if (total > interval_capacity) return SK_ERROR_LIMIT;
        if (total > 0 && !out_intervals) return SK_ERROR_INVALID_ARGUMENT;
        std::vector<Interval> ray;
        size_t at = 0;
        for (uint32_t j = 0; j < nj; ++j)
            for (uint32_t i = 0; i < ni; ++i) {
                s->read(i0 + i, j0 + j, ray);
                for (const Interval &interval : ray) to_interval(interval, out_intervals[at++]);
            }
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_compare(sk_stock_handle stock, sk_stock_handle target, uint32_t i0, uint32_t j0,
    uint32_t ni, uint32_t nj, sk_ray_comparison *out_comparisons, uint32_t comparison_capacity) {
    return guarded([&]() -> sk_result {
        Stock *s = stocks().get(stock.id);
        Stock *t = stocks().get(target.id);
        if (!t) return SK_ERROR_INVALID_HANDLE;
        sk_result result = check_block(s, i0, j0, ni, nj);
        if (result != SK_OK) return result;
        const Grid &a = s->grid(), &b = t->grid();
        if (a.axis != b.axis || a.origin[0] != b.origin[0] || a.origin[1] != b.origin[1] || a.spacing != b.spacing ||
            a.count[0] != b.count[0] || a.count[1] != b.count[1])
            return SK_ERROR_INVALID_ARGUMENT;
        if (comparison_capacity < uint64_t(ni) * nj || (comparison_capacity > 0 && !out_comparisons))
            return SK_ERROR_INVALID_ARGUMENT;
        std::vector<Interval> x, y;
        for (uint32_t j = 0; j < nj; ++j)
            for (uint32_t i = 0; i < ni; ++i) {
                s->read(i0 + i, j0 + j, x);
                t->read(i0 + i, j0 + j, y);
                RayComparison c = compare_ray(x, y);
                sk_ray_comparison &out = out_comparisons[size_t(j) * ni + i];
                out = sk_ray_comparison{};
                out.struct_size = sizeof(sk_ray_comparison);
                out.gouge_source = c.gouge_source;
                out.leftover = c.leftover;
                out.gouge = c.gouge;
                out.largest_leftover = c.largest_leftover;
                out.largest_gouge = c.largest_gouge;
            }
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_stock_mesh(sk_stock_handle stock, uint32_t tile_x, uint32_t tile_y, uint32_t tiles_x,
    uint32_t tiles_y, uint32_t flags, sk_mesh_handle *out_mesh) {
    return guarded([&]() -> sk_result {
        if (!out_mesh) return SK_ERROR_INVALID_ARGUMENT;
        out_mesh->id = 0;
        Stock *s = stocks().get(stock.id);
        if (!s) return SK_ERROR_INVALID_HANDLE;
        if (flags & ~uint32_t(SK_MESH_BOTTOMS | SK_MESH_MERGE)) return SK_ERROR_INVALID_ARGUMENT;
        if (uint64_t(tile_x) + tiles_x > s->tiles_across() || uint64_t(tile_y) + tiles_y > s->tiles_down())
            return SK_ERROR_INVALID_ARGUMENT;
        auto entry = std::make_unique<MeshEntry>();
        PreviewOptions options;
        options.bottoms = flags & SK_MESH_BOTTOMS;
        options.merge = flags & SK_MESH_MERGE;
        build_preview(*s, tile_x, tile_y, tiles_x, tiles_y, options, entry->mesh);
        entry->mesh.colors.assign(entry->mesh.vertex_count(), 0xFFFFFFFFu);
        entry->rays = uint64_t(s->grid().count[0]) * s->grid().count[1];
        uint32_t id = meshes().add(std::move(entry));
        if (id == 0) return SK_ERROR_LIMIT;
        out_mesh->id = id;
        return SK_OK;
    });
}

SK_API void SK_CALL sk_mesh_destroy(sk_mesh_handle mesh) {
    guarded([&]() -> sk_result {
        meshes().remove(mesh.id);
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_mesh_get_info(sk_mesh_handle mesh, sk_mesh_info *out_info) {
    return guarded([&]() -> sk_result {
        if (!out_info) return SK_ERROR_INVALID_ARGUMENT;
        MeshEntry *m = meshes().get(mesh.id);
        if (!m) return SK_ERROR_INVALID_HANDLE;
        *out_info = sk_mesh_info{};
        out_info->struct_size = sizeof(sk_mesh_info);
        out_info->vertex_count = m->mesh.vertex_count();
        out_info->triangle_count = m->mesh.triangle_count();
        out_info->merged = m->mesh.merged ? 1 : 0;
        return SK_OK;
    });
}

#define SK_MESH_COPY(name, field)                                                                  \
    SK_API sk_result SK_CALL name(sk_mesh_handle mesh, uint8_t *output, uint32_t *byte_capacity) { \
        return guarded([&]() -> sk_result {                                                       \
            MeshEntry *m = meshes().get(mesh.id);                                                  \
            if (!m) return SK_ERROR_INVALID_HANDLE;                                                \
            return copy_bytes(m->mesh.field, output, byte_capacity);                               \
        });                                                                                        \
    }

SK_MESH_COPY(sk_mesh_copy_positions, positions)
SK_MESH_COPY(sk_mesh_copy_normals, normals)
SK_MESH_COPY(sk_mesh_copy_indices, indices)
SK_MESH_COPY(sk_mesh_copy_colors, colors)
SK_MESH_COPY(sk_mesh_copy_triangle_sources, triangle_sources)
#undef SK_MESH_COPY

namespace {

/** 0xRRGGBBAA to the bytes R, G, B, A in memory order. */
uint32_t rgba_bytes(uint32_t rgba) {
    uint8_t bytes[4] = {uint8_t(rgba >> 24), uint8_t(rgba >> 16), uint8_t(rgba >> 8), uint8_t(rgba)};
    uint32_t packed;
    std::memcpy(&packed, bytes, 4);
    return packed;
}

} // namespace

SK_API sk_result SK_CALL sk_mesh_color_by_source(sk_mesh_handle mesh, const uint32_t *palette, uint32_t palette_count,
    uint32_t original, uint32_t fallback) {
    return guarded([&]() -> sk_result {
        MeshEntry *m = meshes().get(mesh.id);
        if (!m) return SK_ERROR_INVALID_HANDLE;
        if (palette_count > 0 && !palette) return SK_ERROR_INVALID_ARGUMENT;
        PreviewMesh &p = m->mesh;
        for (size_t k = 0; k < p.vertex_sources.size(); ++k) {
            uint32_t source = p.vertex_sources[k];
            uint32_t color = source == SK_SOURCE_STOCK ? original : source < palette_count ? palette[source] : fallback;
            p.colors[k] = rgba_bytes(color);
        }
        return SK_OK;
    });
}

SK_API sk_result SK_CALL sk_mesh_color_by_ray(sk_mesh_handle mesh, const uint32_t *ray_colors, uint32_t ray_count) {
    return guarded([&]() -> sk_result {
        MeshEntry *m = meshes().get(mesh.id);
        if (!m) return SK_ERROR_INVALID_HANDLE;
        if (m->mesh.merged) return SK_ERROR_UNSUPPORTED;
        if (ray_count < m->rays || (ray_count > 0 && !ray_colors)) return SK_ERROR_INVALID_ARGUMENT;
        PreviewMesh &p = m->mesh;
        for (size_t k = 0; k < p.vertex_rays.size(); ++k) p.colors[k] = rgba_bytes(ray_colors[p.vertex_rays[k]]);
        return SK_OK;
    });
}

} // extern "C"
