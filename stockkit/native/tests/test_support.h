// Shared helpers for the StockKit native checks, which go through the C ABI only.
// Units are millimetres; the core is unit-agnostic.
#pragma once

#include "stockkit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace sk_test {

constexpr double kPi = 3.14159265358979323846;

inline int failures = 0;
inline int checks = 0;

inline void check(bool ok, const std::string &what) {
    ++checks;
    if (!ok) {
        ++failures;
        std::printf("FAIL: %s\n", what.c_str());
    }
}

inline void near(double actual, double expected, double tolerance, const std::string &what) {
    check(std::fabs(actual - expected) <= tolerance,
        what + ": got " + std::to_string(actual) + ", expected " + std::to_string(expected));
}

inline sk_profile_segment line(double r0, double z0, double r1, double z1) {
    sk_profile_segment s{};
    s.struct_size = sizeof s;
    s.kind = SK_SEGMENT_LINE;
    s.r0 = r0, s.z0 = z0, s.r1 = r1, s.z1 = z1;
    return s;
}

inline sk_profile_segment arc(double cr, double cz, double r0, double z0, double r1, double z1) {
    sk_profile_segment s = line(r0, z0, r1, z1);
    s.kind = SK_SEGMENT_ARC;
    s.center_r = cr, s.center_z = cz;
    return s;
}

inline sk_tool_handle tool(const std::vector<sk_profile_segment> &segments) {
    sk_tool_handle handle{};
    sk_result result = sk_tool_create(segments.data(), uint32_t(segments.size()), &handle);
    check(result == SK_OK, "tool create");
    return handle;
}

inline sk_tool_handle flat(double d, double length) { return tool({line(0, 0, d / 2, 0), line(d / 2, 0, d / 2, length)}); }

inline sk_tool_handle ball(double d, double length) {
    double r = d / 2;
    return tool({arc(0, r, 0, 0, r, r), line(r, r, r, length)});
}

inline sk_tool_handle bull(double d, double corner, double length) {
    double r = d / 2, f = r - corner;
    return tool({line(0, 0, f, 0), arc(f, corner, f, 0, r, corner), line(r, corner, r, length)});
}

inline sk_move line_move(double x0, double y0, double z0, double x1, double y1, double z1, uint32_t source = 0) {
    sk_move m{};
    m.struct_size = sizeof m;
    m.kind = SK_MOVE_LINE;
    m.source = source;
    m.start[0] = x0, m.start[1] = y0, m.start[2] = z0;
    m.end[0] = x1, m.end[1] = y1, m.end[2] = z1;
    return m;
}

inline sk_move arc_move(double cx, double cy, double z, double radius, double start, double sweep, double rise,
    uint32_t source = 0) {
    sk_move m{};
    m.struct_size = sizeof m;
    m.kind = SK_MOVE_ARC;
    m.source = source;
    m.center[0] = cx, m.center[1] = cy, m.center[2] = z;
    m.radius = radius;
    m.start_angle = start;
    m.sweep = sweep;
    m.rise = rise;
    return m;
}

/** One sweep query: a capacity too small reports what it needs, then the call succeeds. */
inline std::vector<sk_interval> sweep_axis(sk_tool_handle t, const sk_move &m, uint32_t axis, double u, double v) {
    uint32_t count = 0;
    sk_result first = sk_sweep_ray(t, &m, axis, u, v, nullptr, &count);
    check(first == SK_OK || first == SK_ERROR_LIMIT, "sweep size query");
    std::vector<sk_interval> out(count);
    check(sk_sweep_ray(t, &m, axis, u, v, out.data(), &count) == SK_OK && count == out.size(), "sweep read");
    return out;
}

inline std::vector<sk_interval> sweep(sk_tool_handle t, const sk_move &m, double x, double y) {
    return sweep_axis(t, m, SK_AXIS_Z, x, y);
}

/** A Z-only lattice: rays through (x0 + i * spacing, y0 + j * spacing). */
inline sk_lattice grid(double x0, double y0, double spacing, uint32_t ni, uint32_t nj, uint32_t tile = 8) {
    sk_lattice g{};
    g.struct_size = sizeof g;
    g.axes = SK_AXES_Z;
    g.origin[0] = x0, g.origin[1] = y0;
    g.spacing = spacing;
    g.count[0] = ni, g.count[1] = nj, g.count[2] = 1;
    g.tile_size = tile;
    return g;
}

/** A lattice with all three grids, nodes from (x0, y0, z0). */
inline sk_lattice lattice(double x0, double y0, double z0, double spacing, uint32_t ni, uint32_t nj, uint32_t nk,
    uint32_t tile = 8) {
    sk_lattice g = grid(x0, y0, spacing, ni, nj, tile);
    g.axes = SK_AXES_ALL;
    g.origin[2] = z0;
    g.count[2] = nk;
    return g;
}

inline sk_stock_handle box_stock(const sk_lattice &g, double x0, double y0, double z0, double x1, double y1, double z1) {
    sk_box b{};
    b.struct_size = sizeof b;
    b.min[0] = x0, b.min[1] = y0, b.min[2] = z0;
    b.max[0] = x1, b.max[1] = y1, b.max[2] = z1;
    sk_stock_handle s{};
    check(sk_stock_create_box(&g, &b, &s) == SK_OK, "box stock");
    return s;
}

inline std::vector<sk_move_result> cut_results(sk_stock_handle s, sk_tool_handle t, const std::vector<sk_move> &moves,
    const std::string &what) {
    std::vector<sk_move_result> results(moves.size());
    sk_cut_summary summary{};
    check(sk_stock_cut(s, t, moves.data(), uint32_t(moves.size()), results.data(), &summary) == SK_OK, what);
    double removed = 0;
    for (const sk_move_result &r : results) removed += r.removed;
    check(summary.moves == moves.size() && summary.removed == removed, what + ": summary adds up the moves");
    return results;
}

inline std::vector<double> cut(sk_stock_handle s, sk_tool_handle t, const std::vector<sk_move> &moves,
    const std::string &what) {
    std::vector<double> removed;
    for (const sk_move_result &r : cut_results(s, t, moves, what)) removed.push_back(r.removed);
    return removed;
}

inline sk_stock_info info_of(sk_stock_handle s) {
    sk_stock_info info{};
    info.struct_size = sizeof info;
    check(sk_stock_get_info(s, &info) == SK_OK, "stock info");
    return info;
}

struct Rays {
    std::vector<uint32_t> counts;
    std::vector<sk_interval> intervals;
    std::vector<uint32_t> offsets;
    uint32_t ni = 0;
    const sk_interval *at(uint32_t i, uint32_t j, uint32_t &n) const {
        size_t r = size_t(j) * ni + i;
        n = counts[r];
        return intervals.data() + offsets[r];
    }
};

inline Rays read_all(sk_stock_handle s, uint32_t axis = SK_AXIS_Z) {
    sk_stock_info info{};
    info.struct_size = sizeof info;
    sk_stock_get_info(s, &info);
    Rays rays;
    const sk_grid_info &grid = info.grids[axis];
    rays.ni = grid.count[0];
    uint32_t ni = grid.count[0], nj = grid.count[1];
    rays.counts.resize(size_t(ni) * nj);
    // Too small a capacity reports the total and writes no intervals.
    uint32_t total = 0;
    check(sk_stock_read_rays(s, axis, 0, 0, ni, nj, rays.counts.data(), uint32_t(rays.counts.size()), nullptr,
              &total) == (grid.interval_count == 0 ? SK_OK : SK_ERROR_LIMIT),
        "short read refused");
    check(total == grid.interval_count, "interval total matches info");
    rays.intervals.resize(total);
    check(sk_stock_read_rays(s, axis, 0, 0, ni, nj, rays.counts.data(), uint32_t(rays.counts.size()),
              rays.intervals.data(), &total) == SK_OK && total == rays.intervals.size(),
        "read rays");
    uint32_t offset = 0;
    for (uint32_t c : rays.counts) {
        rays.offsets.push_back(offset);
        offset += c;
    }
    return rays;
}

/** Lowest tip-plus-envelope over a finely sampled move: an upper bound on the true floor. */
inline double sampled_floor(const std::function<void(double, double &, double &, double &)> &pose,
    const std::function<double(double)> &lower, double reach, double x, double y, int samples) {
    double best = INFINITY;
    for (int k = 0; k <= samples; ++k) {
        double px, py, pz;
        pose(double(k) / samples, px, py, pz);
        double d = std::hypot(x - px, y - py);
        if (d <= reach) best = std::min(best, pz + lower(d));
    }
    return best;
}

/** The revision counter of every tile in the Z grid. */
inline std::vector<uint64_t> revisions_of(sk_stock_handle s) {
    sk_stock_info info = info_of(s);
    std::vector<uint64_t> revisions(size_t(info.grids[SK_AXIS_Z].tiles[0]) * info.grids[SK_AXIS_Z].tiles[1]);
    check(sk_stock_read_revisions(s, SK_AXIS_Z, revisions.data(), uint32_t(revisions.size())) == SK_OK, "read revisions");
    return revisions;
}

/** Prints the tally and returns the process exit code. */
inline int finish() {
    std::printf("%d of %d checks passed\n", checks - failures, checks);
    return failures == 0 ? 0 : 1;
}

} // namespace sk_test
