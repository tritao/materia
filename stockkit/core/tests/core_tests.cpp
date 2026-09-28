// Closed-form checks for StockKit core, through the C ABI only.
// Units here are millimetres; the core is unit-agnostic.

#include "stockkit.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
int failures = 0;
int checks = 0;

void check(bool ok, const std::string &what) {
    ++checks;
    if (!ok) {
        ++failures;
        std::printf("FAIL: %s\n", what.c_str());
    }
}

void near(double actual, double expected, double tolerance, const std::string &what) {
    check(std::fabs(actual - expected) <= tolerance,
        what + ": got " + std::to_string(actual) + ", expected " + std::to_string(expected));
}

sk_profile_segment line(double r0, double z0, double r1, double z1) {
    sk_profile_segment s{};
    s.struct_size = sizeof s;
    s.kind = SK_SEGMENT_LINE;
    s.r0 = r0, s.z0 = z0, s.r1 = r1, s.z1 = z1;
    return s;
}

sk_profile_segment arc(double cr, double cz, double r0, double z0, double r1, double z1) {
    sk_profile_segment s = line(r0, z0, r1, z1);
    s.kind = SK_SEGMENT_ARC;
    s.center_r = cr, s.center_z = cz;
    return s;
}

sk_tool_handle tool(const std::vector<sk_profile_segment> &segments) {
    sk_tool_handle handle{};
    sk_result result = sk_tool_create(segments.data(), uint32_t(segments.size()), &handle);
    check(result == SK_OK, "tool create");
    return handle;
}

sk_tool_handle flat(double d, double length) { return tool({line(0, 0, d / 2, 0), line(d / 2, 0, d / 2, length)}); }

sk_tool_handle ball(double d, double length) {
    double r = d / 2;
    return tool({arc(0, r, 0, 0, r, r), line(r, r, r, length)});
}

sk_tool_handle bull(double d, double corner, double length) {
    double r = d / 2, f = r - corner;
    return tool({line(0, 0, f, 0), arc(f, corner, f, 0, r, corner), line(r, corner, r, length)});
}

sk_move line_move(double x0, double y0, double z0, double x1, double y1, double z1, uint32_t source = 0) {
    sk_move m{};
    m.struct_size = sizeof m;
    m.kind = SK_MOVE_LINE;
    m.source = source;
    m.start[0] = x0, m.start[1] = y0, m.start[2] = z0;
    m.end[0] = x1, m.end[1] = y1, m.end[2] = z1;
    return m;
}

sk_move arc_move(double cx, double cy, double z, double radius, double start, double sweep, double rise,
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
std::vector<sk_interval> sweep_axis(sk_tool_handle t, const sk_move &m, uint32_t axis, double u, double v) {
    uint32_t count = 0;
    sk_result first = sk_sweep_ray(t, &m, axis, u, v, nullptr, &count);
    check(first == SK_OK || first == SK_ERROR_LIMIT, "sweep size query");
    std::vector<sk_interval> out(count);
    check(sk_sweep_ray(t, &m, axis, u, v, out.data(), &count) == SK_OK && count == out.size(), "sweep read");
    return out;
}

std::vector<sk_interval> sweep(sk_tool_handle t, const sk_move &m, double x, double y) {
    return sweep_axis(t, m, SK_AXIS_Z, x, y);
}

/** A Z-only lattice: rays through (x0 + i * spacing, y0 + j * spacing). */
sk_lattice grid(double x0, double y0, double spacing, uint32_t ni, uint32_t nj, uint32_t tile = 8) {
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
sk_lattice lattice(double x0, double y0, double z0, double spacing, uint32_t ni, uint32_t nj, uint32_t nk,
    uint32_t tile = 8) {
    sk_lattice g = grid(x0, y0, spacing, ni, nj, tile);
    g.axes = SK_AXES_ALL;
    g.origin[2] = z0;
    g.count[2] = nk;
    return g;
}

sk_stock_handle box_stock(const sk_lattice &g, double x0, double y0, double z0, double x1, double y1, double z1) {
    sk_box b{};
    b.struct_size = sizeof b;
    b.min[0] = x0, b.min[1] = y0, b.min[2] = z0;
    b.max[0] = x1, b.max[1] = y1, b.max[2] = z1;
    sk_stock_handle s{};
    check(sk_stock_create_box(&g, &b, &s) == SK_OK, "box stock");
    return s;
}

std::vector<sk_move_result> cut_results(sk_stock_handle s, sk_tool_handle t, const std::vector<sk_move> &moves,
    const std::string &what) {
    std::vector<sk_move_result> results(moves.size());
    sk_cut_summary summary{};
    check(sk_stock_cut(s, t, moves.data(), uint32_t(moves.size()), results.data(), &summary) == SK_OK, what);
    double removed = 0;
    for (const sk_move_result &r : results) removed += r.removed;
    check(summary.moves == moves.size() && summary.removed == removed, what + ": summary adds up the moves");
    return results;
}

std::vector<double> cut(sk_stock_handle s, sk_tool_handle t, const std::vector<sk_move> &moves,
    const std::string &what) {
    std::vector<double> removed;
    for (const sk_move_result &r : cut_results(s, t, moves, what)) removed.push_back(r.removed);
    return removed;
}

sk_stock_info info_of(sk_stock_handle s) {
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

Rays read_all(sk_stock_handle s, uint32_t axis = SK_AXIS_Z) {
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
double sampled_floor(const std::function<void(double, double &, double &, double &)> &pose,
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

void flat_slot() {
    sk_tool_handle t = flat(6, 20);
    sk_lattice g = grid(0, 0, 0.5, 101, 61);
    sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
    double before = info_of(s).volume;
    auto removed = cut(s, t, {line_move(10, 15, 15, 40, 15, 15, 7)}, "flat slot cut");
    Rays rays = read_all(s);
    int wrong = 0;
    double expected_removed = 0;
    for (uint32_t j = 0; j < 61; ++j)
        for (uint32_t i = 0; i < 101; ++i) {
            double x = 0.5 * i, y = 0.5 * j;
            double along = std::max(10.0, std::min(40.0, x));
            bool cut = std::hypot(x - along, y - 15) <= 3;
            uint32_t n;
            const sk_interval *v = rays.at(i, j, n);
            double top = cut ? 15 : 20;
            if (cut) expected_removed += 5 * 0.25;
            if (n != 1 || v[0].lo != 0 || v[0].hi != top) ++wrong;
            else if (cut && (v[0].hi_source != 7 || v[0].hi_normal[2] != 1)) ++wrong;
            else if (!cut && v[0].hi_source != SK_SOURCE_STOCK) ++wrong;
        }
    check(wrong == 0, "flat slot rays (" + std::to_string(wrong) + " wrong)");
    near(removed[0], expected_removed, 1e-9, "flat slot removed volume");
    near(before - info_of(s).volume, expected_removed, 1e-9, "stock volume drops by the removed volume");
    check(info_of(s).tiles_skipped == 0, "flat slot tiles all tested");
    removed = cut(s, t, {line_move(10, 15, 20.5, 40, 15, 20.5)}, "move above");
    check(removed[0] == 0 && info_of(s).tiles_skipped > 0, "move above the stock skips tiles");
    sk_stock_destroy(s);
    sk_tool_destroy(t);
}

void ball_slot() {
    sk_tool_handle t = ball(6, 20);
    sk_move m = line_move(10, 15, 12, 40, 15, 12);
    for (double e : {0.0, 0.7, 1.9, 2.5, 2.999}) {
        auto spans = sweep(t, m, 25, 15 + e);
        check(spans.size() == 1, "ball slot one span");
        if (spans.empty()) continue;
        near(spans[0].lo, 12 + 3 - std::sqrt(9 - e * e), 1e-12, "ball slot floor at e=" + std::to_string(e));
        near(spans[0].hi, 32, 0, "ball slot top");
        // Surface normal of the floor points at the ball centre.
        double ny = -e / 3, nz = std::sqrt(9 - e * e) / 3;
        near(spans[0].lo_normal[1], ny, 1e-6, "ball slot normal y");
        near(spans[0].lo_normal[2], nz, 1e-6, "ball slot normal z");
    }
    check(sweep(t, m, 25, 18.001).empty(), "ball slot misses past its radius");
    // End cap: beyond the line end the floor is the sphere at the end point.
    auto cap = sweep(t, m, 41.5, 16);
    near(cap[0].lo, 12 + 3 - std::sqrt(9 - 1.5 * 1.5 - 1), 1e-12, "ball slot end cap");
    sk_tool_destroy(t);
}

void bull_arc() {
    sk_tool_handle t = bull(10, 2, 20);
    const double cx = 25, cy = 15, rho = 8, z = 10;
    sk_move m = arc_move(cx, cy, z, rho, 0, kPi, 0);
    auto lower = [](double d) { return d <= 3 ? 0.0 : 2 - std::sqrt(std::max(0.0, 4 - (d - 3) * (d - 3))); };
    int wrong = 0, tested = 0;
    for (double x = 10; x <= 40; x += 0.37)
        for (double y = 5; y <= 30; y += 0.41) {
            double D = std::hypot(x - cx, y - cy), angle = std::atan2(y - cy, x - cx);
            double d;
            if (angle >= 0 && angle <= kPi) d = std::fabs(D - rho);
            else d = std::min(std::hypot(x - cx - rho, y - cy), std::hypot(x - cx + rho, y - cy));
            auto spans = sweep(t, m, x, y);
            if (d > 5) {
                if (!spans.empty()) ++wrong;
                continue;
            }
            ++tested;
            if (spans.size() != 1 || std::fabs(spans[0].lo - (z + lower(d))) > 1e-12 || spans[0].hi != z + 20) ++wrong;
        }
    check(wrong == 0 && tested > 100, "bull-nose arc floors (" + std::to_string(wrong) + " wrong of " +
        std::to_string(tested) + ")");
    sk_tool_destroy(t);
}

void plunge() {
    sk_tool_handle t = flat(6, 20);
    sk_lattice g = grid(0, 0, 0.5, 101, 61);
    sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
    cut(s, t, {line_move(25, 15, 25, 25, 15, 5)}, "plunge cut");
    Rays rays = read_all(s);
    uint32_t n;
    const sk_interval *v = rays.at(50, 30, n);
    check(n == 1 && v[0].lo == 0 && v[0].hi == 5, "plunge leaves [0, 5] under the tool");
    v = rays.at(56, 30, n);
    check(n == 1 && v[0].hi == 5, "plunge cuts at the tool's edge");
    v = rays.at(57, 30, n);
    check(n == 1 && v[0].hi == 20, "plunge leaves stock outside the tool");
    sk_stock_destroy(s);
    sk_tool_destroy(t);
}

void ball_ramp() {
    const double R = 3;
    sk_tool_handle t = ball(2 * R, 20);
    // Tip from (10, 15, 15) to (40, 25, 5): a ramp.
    const double p0[3] = {10, 15, 15}, p1[3] = {40, 25, 5};
    sk_move m = line_move(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
    double L = std::hypot(p1[0] - p0[0], p1[1] - p0[1]);
    double k = (p1[2] - p0[2]) / L;
    int wrong = 0, tested = 0;
    for (double x = 8; x <= 43; x += 0.61)
        for (double y = 11; y <= 29; y += 0.53) {
            // Capsule floor in closed form: the ball centre runs 3 above the tip.
            double dx = (p1[0] - p0[0]) / L, dy = (p1[1] - p0[1]) / L;
            double u0 = (x - p0[0]) * dx + (y - p0[1]) * dy;
            double e = std::fabs((x - p0[0]) * dy - (y - p0[1]) * dx);
            double best = INFINITY;
            auto floor_at = [&](double u) {
                u = std::max(0.0, std::min(L, u));
                double w = u - u0, d2 = e * e + w * w;
                if (d2 > R * R) return double(INFINITY);
                return p0[2] + k * u + R - std::sqrt(R * R - d2);
            };
            if (e < R) {
                double w = -k * std::sqrt(R * R - e * e) / std::sqrt(1 + k * k);
                best = std::min({floor_at(u0 + w), floor_at(0), floor_at(L)});
            }
            // Rays beyond the ball's reach can still meet the shank; the closed form covers the ball only.
            if (!std::isfinite(best)) continue;
            auto spans = sweep(t, m, x, y);
            ++tested;
            if (spans.empty() || std::fabs(spans[0].lo - best) > 1e-9) ++wrong;
        }
    check(wrong == 0 && tested > 100, "ball ramp capsule floor (" + std::to_string(wrong) + " wrong of " +
        std::to_string(tested) + ")");
    sk_tool_destroy(t);
}

void bull_ramp_and_helix() {
    sk_tool_handle t = bull(10, 2, 20);
    auto lower = [](double d) { return d <= 3 ? 0.0 : 2 - std::sqrt(std::max(0.0, 4 - (d - 3) * (d - 3))); };
    const double p0[3] = {10, 15, 15}, p1[3] = {40, 20, 9};
    sk_move ramp = line_move(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
    auto ramp_pose = [&](double t, double &x, double &y, double &z) {
        x = p0[0] + t * (p1[0] - p0[0]);
        y = p0[1] + t * (p1[1] - p0[1]);
        z = p0[2] + t * (p1[2] - p0[2]);
    };
    const double cx = 25, cy = 15, rho = 3, z0 = 12, sweep_angle = 4 * kPi, rise = -4;
    sk_move helix = arc_move(cx, cy, z0, rho, 0.3, sweep_angle, rise);
    auto helix_pose = [&](double t, double &x, double &y, double &z) {
        double a = 0.3 + t * sweep_angle;
        x = cx + rho * std::cos(a);
        y = cy + rho * std::sin(a);
        z = z0 + t * rise;
    };
    struct Case {
        const char *name;
        sk_move move;
        std::function<void(double, double &, double &, double &)> pose;
        double x0, x1, y0, y1;
    };
    for (const Case &c : {Case{"bull ramp", ramp, ramp_pose, 6, 45, 9, 26},
             Case{"bull helix", helix, helix_pose, 16, 34, 6, 24}}) {
        int wrong = 0, tested = 0;
        double worst = 0;
        for (double x = c.x0; x <= c.x1; x += 0.73)
            for (double y = c.y0; y <= c.y1; y += 0.67) {
                double sampled = sampled_floor(c.pose, lower, 5, x, y, 200000);
                auto spans = sweep(t, c.move, x, y);
                if (!std::isfinite(sampled)) continue;
                ++tested;
                // The sampled floor is an upper bound and within its step of the truth.
                if (spans.empty() || spans[0].lo > sampled + 1e-12 || spans[0].lo < sampled - 1e-4) ++wrong;
                else worst = std::max(worst, sampled - spans[0].lo);
            }
        check(wrong == 0 && tested > 50, std::string(c.name) + " floor vs sampling (" + std::to_string(wrong) +
            " wrong of " + std::to_string(tested) + ", worst gap " + std::to_string(worst) + ")");
    }
    sk_tool_destroy(t);
}

void necked_tool() {
    // Flutes of radius 3 up to 5, a neck of radius 2 up to 15, a shank of radius 3 up to 25.
    sk_tool_handle t = tool({line(0, 0, 3, 0), line(3, 0, 3, 5), line(3, 5, 2, 5), line(2, 5, 2, 15),
        line(2, 15, 3, 15), line(3, 15, 3, 25)});
    sk_move m = line_move(10, 15, 10, 40, 15, 10);
    auto spans = sweep(t, m, 25, 17.5);
    check(spans.size() == 2, "necked tool gives two spans beside the neck");
    if (spans.size() == 2) {
        near(spans[0].lo, 10, 0, "neck lower span bottom");
        near(spans[0].hi, 15, 0, "neck lower span top");
        near(spans[1].lo, 25, 0, "neck upper span bottom");
        near(spans[1].hi, 35, 0, "neck upper span top");
        near(spans[0].hi_normal[2], -1, 0, "shoulder under the neck faces down into the tool");
    }
    auto inside = sweep(t, m, 25, 16.5);
    check(inside.size() == 1 && inside[0].lo == 10 && inside[0].hi == 35, "necked tool is solid inside the neck");
    // A ramp past the neck: the upper span's floor rises with the tool.
    sk_move ramp = line_move(10, 15, 10, 40, 15, 16);
    auto ramped = sweep(t, ramp, 25, 17.5);
    check(ramped.size() == 2, "ramped necked tool keeps the gap");
    sk_tool_destroy(t);
}

void box_mesh() {
    auto mesh_stock = [](const sk_lattice &g, const std::vector<double> &p, const std::vector<uint32_t> &idx,
                          sk_result &result) {
        sk_stock_handle s{};
        result = sk_stock_create_mesh(&g, p.data(), uint32_t(p.size()), idx.data(), uint32_t(idx.size()), &s);
        return s;
    };
    auto box = [](double x0, double y0, double z0, double x1, double y1, double z1, std::vector<double> &p,
                   std::vector<uint32_t> &idx) {
        p = {x0, y0, z0, x1, y0, z0, x1, y1, z0, x0, y1, z0, x0, y0, z1, x1, y0, z1, x1, y1, z1, x0, y1, z1};
        // Outward, counter-clockwise seen from outside.
        idx = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3,
            4, 7};
    };
    sk_lattice g = grid(0, 0, 0.5, 101, 61);
    std::vector<double> p;
    std::vector<uint32_t> idx;
    box(0.25, 0.25, 0, 49.75, 29.75, 20, p, idx);
    sk_result result;
    sk_stock_handle s = mesh_stock(g, p, idx, result);
    check(result == SK_OK, "box mesh stock");
    sk_stock_handle b = box_stock(g, 0.25, 0.25, 0, 49.75, 29.75, 20);
    sk_stock_info a{}, c{};
    a.struct_size = c.struct_size = sizeof a;
    sk_stock_get_info(s, &a);
    sk_stock_get_info(b, &c);
    check(a.grids[SK_AXIS_Z].interval_count == c.grids[SK_AXIS_Z].interval_count && a.volume == c.volume, "box mesh matches box stock");
    Rays rays = read_all(s);
    uint32_t n;
    const sk_interval *v = rays.at(10, 10, n);
    check(n == 1 && v[0].lo_normal[2] == -1 && v[0].hi_normal[2] == 1 && v[0].lo_source == SK_SOURCE_STOCK,
        "box mesh normals and source");
    sk_stock_destroy(s);
    sk_stock_destroy(b);

    // Edges and vertices exactly on rays: every ray is claimed once, so parity closes.
    box(0, 0, 0, 50, 30, 20, p, idx);
    s = mesh_stock(g, p, idx, result);
    check(result == SK_OK, "box mesh with edges on rays");
    sk_stock_get_info(s, &a);
    // Half-open in x and y: rays on the low sides are in, on the high sides out.
    near(a.volume, 100 * 60 * 0.25 * 20, 1e-6, "box mesh on rays volume");
    sk_stock_destroy(s);

    // Octahedron |x| + |y| + |z| <= 6 about a ray, vertices and edges on rays.
    std::vector<double> o = {31, 15, 10, 19, 15, 10, 25, 21, 10, 25, 9, 10, 25, 15, 16, 25, 15, 4};
    // Vertices: 0 +x, 1 -x, 2 +y, 3 -y, 4 +z, 5 -z.
    std::vector<uint32_t> oi = {0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5};
    s = mesh_stock(g, o, oi, result);
    check(result == SK_OK, "octahedron mesh");
    rays = read_all(s);
    int wrong = 0;
    for (uint32_t j = 0; j < 61; ++j)
        for (uint32_t i = 0; i < 101; ++i) {
            double x = 0.5 * i - 25, y = 0.5 * j - 15;
            double half = 6 - std::fabs(x) - std::fabs(y);
            const sk_interval *r = rays.at(i, j, n);
            if (half <= 0) {
                if (n != 0) ++wrong;
            } else if (n != 1 || std::fabs(r[0].lo - (10 - half)) > 1e-12 || std::fabs(r[0].hi - (10 + half)) > 1e-12) {
                ++wrong;
            }
        }
    check(wrong == 0, "octahedron rays (" + std::to_string(wrong) + " wrong)");
    sk_stock_destroy(s);

    // An open mesh is refused (a missing vertical face is invisible to Z rays; drop a top one).
    idx.erase(idx.begin() + 6, idx.begin() + 9);
    s = mesh_stock(g, p, idx, result);
    check(result == SK_ERROR_INVALID_ARGUMENT, "open mesh refused");
}

void provenance_and_rapids() {
    sk_tool_handle t = flat(6, 20);
    sk_lattice g = grid(0, 0, 0.5, 101, 61);
    sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
    std::vector<sk_move> moves = {line_move(10, 15, 15, 40, 15, 15, 1), line_move(10, 15, 30, 40, 15, 30, 2),
        line_move(25, 5, 18, 25, 25, 18, 3)};
    moves[1].flags = moves[2].flags = SK_MOVE_RAPID;
    auto removed = cut(s, t, moves, "three moves");
    check(removed[0] > 0 && removed[1] == 0 && removed[2] > 0, "per-move removal shows the rapid through stock");
    std::vector<sk_move_result> short_output(2);
    check(sk_stock_cut(s, t, moves.data(), 3, short_output.data(), nullptr) == SK_ERROR_INVALID_ARGUMENT,
        "cut output shorter than the moves is refused");
    Rays rays = read_all(s);
    uint32_t n;
    const sk_interval *v = rays.at(50, 16, n); // (25, 8): only the rapid crossed it
    check(n == 1 && v[0].hi == 18 && v[0].hi_source == 3, "rapid's cut is attributed to it");
    v = rays.at(50, 30, n); // (25, 15): slot floor below the rapid
    check(n == 1 && v[0].hi == 15 && v[0].hi_source == 1, "slot floor keeps its move");
    sk_stock_destroy(s);
    sk_tool_destroy(t);
}

void handles() {
    sk_tool_handle t = flat(6, 20);
    sk_tool_info info{};
    check(sk_tool_get_info(t, &info) == SK_OK && info.radius == 3 && info.height == 20, "tool info");
    sk_tool_destroy(t);
    check(sk_tool_get_info(t, &info) == SK_ERROR_INVALID_HANDLE, "destroyed tool is invalid");
    sk_tool_handle zero{0};
    check(sk_tool_get_info(zero, &info) == SK_ERROR_INVALID_HANDLE, "zero handle is invalid");
    sk_tool_handle again = flat(6, 20);
    check(again.id != t.id, "reused slot gets a new generation");
    sk_stock_info stock_info{};
    sk_stock_handle wrong_kind{again.id};
    check(sk_stock_get_info(wrong_kind, &stock_info) == SK_ERROR_INVALID_HANDLE, "tool handle is not a stock");
    sk_tool_destroy(again);
    sk_profile_segment gap[] = {line(0, 0, 3, 0), line(3, 1, 3, 20)};
    sk_tool_handle bad{};
    check(sk_tool_create(gap, 2, &bad) == SK_ERROR_INVALID_ARGUMENT && bad.id == 0, "discontinuous profile refused");
    sk_lattice x = grid(0, 0, 1, 10, 10);
    x.axes = SK_AXES_X | SK_AXES_Y;
    sk_box b{};
    b.struct_size = sizeof b;
    b.max[0] = b.max[1] = b.max[2] = 1;
    sk_stock_handle s{};
    check(sk_stock_create_box(&x, &b, &s) == SK_ERROR_INVALID_ARGUMENT, "a lattice needs its Z grid");
    x.axes = 8 | SK_AXES_Z;
    check(sk_stock_create_box(&x, &b, &s) == SK_ERROR_INVALID_ARGUMENT, "unknown axes are refused");
}

sk_profile_segment zoned(sk_profile_segment segment, uint32_t zone) {
    segment.zone = zone;
    return segment;
}

/** Flutes of radius 3 up to 5, a shank of radius 3 up to 25, a holder of radius 10 up to 45. */
sk_tool_handle held_tool() {
    return tool({line(0, 0, 3, 0), line(3, 0, 3, 5), zoned(line(3, 5, 3, 25), SK_ZONE_SHANK),
        zoned(line(3, 25, 10, 25), SK_ZONE_HOLDER), zoned(line(10, 25, 10, 45), SK_ZONE_HOLDER)});
}

void shank_and_holder_contact() {
    sk_tool_handle t = held_tool();
    sk_tool_info info{};
    check(sk_tool_get_info(t, &info) == SK_OK && info.cutting_radius == 3 && info.cutting_height == 5 &&
            info.radius == 10 && info.height == 45,
        "tool info separates flutes from the whole tool");
    sk_lattice g = grid(0, 0, 0.5, 101, 61);
    // Sum of `height` over rays within `reach` of the segment (10, 15)-(40, 15): an exact ray-sampled volume.
    auto footprint = [&](double reach, double height) {
        double total = 0;
        for (uint32_t j = 0; j < 61; ++j)
            for (uint32_t i = 0; i < 101; ++i) {
                double x = 0.5 * i, y = 0.5 * j, along = std::max(10.0, std::min(40.0, x));
                if (std::hypot(x - along, y - 15) <= reach) total += height * 0.25;
            }
        return total;
    };

    // Flutes 5 long in a slot 8 deep: the shank rubs the top 3.
    sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
    auto slot = cut_results(s, t, {line_move(10, 15, 12, 40, 15, 12)}, "deep slot");
    near(slot[0].removed, footprint(3, 5), 1e-9, "flutes remove their own length");
    near(slot[0].contact[SK_ZONE_SHANK], footprint(3, 3), 1e-9, "shank contact is the stock above the flutes");
    check(slot[0].contact[SK_ZONE_HOLDER] == 0 && slot[0].contact[SK_ZONE_CUTTING] == 0, "holder clear of the stock");
    sk_stock_destroy(s);

    // Tip 8 below the stock: the shank passes through 17 of stock, the holder dips 3 into it.
    s = box_stock(g, 0, 0, 0, 50, 30, 20);
    auto deep = cut_results(s, t, {line_move(10, 15, -8, 40, 15, -8)}, "holder crash");
    check(deep[0].removed == 0, "flutes below the stock remove nothing");
    near(deep[0].contact[SK_ZONE_SHANK], footprint(3, 17), 1e-9, "shank contact through the stock");
    near(deep[0].contact[SK_ZONE_HOLDER], footprint(10, 3), 1e-9, "holder contact where it dips into the stock");
    sk_stock_destroy(s);

    // A plunge: the shank follows the flutes into the hole they cut.
    s = box_stock(g, 0, 0, 0, 50, 30, 20);
    auto plunge = cut_results(s, t, {line_move(25, 15, 25, 25, 15, 16)}, "plunge with shank");
    check(plunge[0].removed > 0 && plunge[0].contact[SK_ZONE_SHANK] == 0, "shank in the flutes' own hole is clear");
    sk_stock_destroy(s);

    // Same results on any thread count.
    std::vector<sk_move> moves;
    for (int k = 0; k < 40; ++k) moves.push_back(line_move(5 + k, 5, 14 - 0.6 * k, 6 + k, 25, 14 - 0.6 * k, k));
    std::vector<sk_move_result> reference;
    for (uint32_t threads : {1u, 4u, 0u}) {
        s = box_stock(g, 0, 0, 0, 50, 30, 20);
        sk_stock_set_threads(s, threads);
        auto results = cut_results(s, t, moves, "threaded contact");
        if (threads == 1) reference = results;
        else
            check(std::memcmp(results.data(), reference.data(), results.size() * sizeof(sk_move_result)) == 0,
                "contact is bit-identical with " + std::to_string(threads) + " threads");
        sk_stock_destroy(s);
    }
    check(reference.back().contact[SK_ZONE_SHANK] > 0 && reference.back().contact[SK_ZONE_HOLDER] > 0,
        "deepening passes reach shank and holder");
    sk_tool_destroy(t);

    sk_profile_segment gap[] = {line(0, 0, 3, 0), zoned(line(3, 0, 3, 5), SK_ZONE_SHANK), line(3, 5, 3, 10)};
    sk_tool_handle bad{};
    check(sk_tool_create(gap, 3, &bad) == SK_ERROR_INVALID_ARGUMENT, "flutes above a shank are refused");
    sk_profile_segment blunt[] = {zoned(line(0, 0, 3, 0), SK_ZONE_SHANK), zoned(line(3, 0, 3, 5), SK_ZONE_SHANK)};
    check(sk_tool_create(blunt, 2, &bad) == SK_ERROR_INVALID_ARGUMENT, "a tool without flutes is refused");
}

void target_comparison() {
    sk_tool_handle t = flat(6, 20);
    sk_lattice g = grid(0, 0, 0.5, 101, 61);
    // The target is a block 15 high; the stock is 20. One pass clears to 15,
    // then a second slot dips to 14.9.
    sk_stock_handle target = box_stock(g, 0, 0, 0, 50, 30, 15);
    sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
    cut(s, t, {line_move(10, 15, 15, 40, 15, 15, 4), line_move(25, 5, 14.9, 25, 25, 14.9, 9)}, "cut for comparison");
    std::vector<sk_ray_comparison> rays(101 * 61);
    check(sk_stock_compare(s, target, SK_AXIS_Z, 0, 0, 101, 61, rays.data(), uint32_t(rays.size())) == SK_OK, "compare");
    const sk_ray_comparison &cleared = rays[30 * 101 + 30];  // (15, 15): the first pass only
    check(cleared.leftover == 0 && cleared.gouge == 0 && cleared.gouge_source == SK_SOURCE_NONE, "cleared to the target");
    const sk_ray_comparison &uncut = rays[2 * 101 + 2];     // (1, 1)
    check(uncut.leftover == 5 && uncut.largest_leftover == 5 && uncut.gouge == 0, "uncut stock is leftover");
    const sk_ray_comparison &dipped = rays[20 * 101 + 50];  // (25, 10): the second slot
    near(dipped.gouge, 0.1, 1e-12, "the dip is a gouge");
    check(dipped.gouge_source == 9 && dipped.leftover == 0, "the gouge names the move that dipped");
    // Grids must match.
    sk_lattice other = grid(0, 0, 0.25, 201, 121);
    sk_stock_handle fine = box_stock(other, 0, 0, 0, 50, 30, 15);
    check(sk_stock_compare(s, fine, SK_AXIS_Z, 0, 0, 1, 1, rays.data(), 1) == SK_ERROR_INVALID_ARGUMENT, "grids must match");
    sk_stock_destroy(fine);
    sk_stock_destroy(s);
    sk_stock_destroy(target);
    sk_tool_destroy(t);
}

std::vector<uint64_t> revisions_of(sk_stock_handle s) {
    sk_stock_info info = info_of(s);
    std::vector<uint64_t> revisions(size_t(info.grids[SK_AXIS_Z].tiles[0]) * info.grids[SK_AXIS_Z].tiles[1]);
    check(sk_stock_read_revisions(s, SK_AXIS_Z, revisions.data(), uint32_t(revisions.size())) == SK_OK, "read revisions");
    return revisions;
}

bool same_rays(sk_stock_handle a, sk_stock_handle b) {
    Rays x = read_all(a), y = read_all(b);
    return x.counts == y.counts && x.intervals.size() == y.intervals.size() &&
        std::memcmp(x.intervals.data(), y.intervals.data(), x.intervals.size() * sizeof(sk_interval)) == 0;
}

void snapshots() {
    sk_tool_handle t = flat(6, 20);
    sk_lattice g = grid(0, 0, 0.5, 101, 61);
    sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
    sk_stock_handle copy = box_stock(g, 0, 0, 0, 50, 30, 20);
    cut(s, t, {line_move(10, 15, 15, 40, 15, 15, 0)}, "before snapshot");
    cut(copy, t, {line_move(10, 15, 15, 40, 15, 15, 0)}, "reference copy");
    sk_snapshot_handle shot{};
    check(sk_stock_snapshot(s, &shot) == SK_OK, "snapshot");
    auto before = revisions_of(s);
    cut(s, t, {line_move(45, 2, 10, 45, 28, 10, 1)}, "after snapshot");
    auto after = revisions_of(s);
    size_t changed = 0;
    for (size_t k = 0; k < before.size(); ++k) changed += after[k] != before[k];
    check(changed > 0 && changed < before.size(), "a cut changes only the tiles it touches");
    check(!same_rays(s, copy), "the later cut changed the stock");
    check(sk_stock_restore(s, shot) == SK_OK && same_rays(s, copy), "restore returns the stock to the snapshot");
    auto restored = revisions_of(s);
    size_t moved = 0;
    for (size_t k = 0; k < before.size(); ++k) {
        moved += restored[k] != after[k];
        check(restored[k] >= after[k], "revisions never go back");
    }
    check(moved == changed, "restore changes the revisions of exactly the tiles that differ");
    // Cutting after a restore leaves the snapshot intact.
    cut(s, t, {line_move(2, 2, 5, 48, 28, 5, 2)}, "diverge again");
    sk_stock_handle other = box_stock(g, 0, 0, 0, 50, 30, 20);
    check(sk_stock_restore(other, shot) == SK_OK && same_rays(other, copy), "the snapshot survives later cuts");
    sk_lattice wrong = grid(0, 0, 0.25, 201, 121);
    sk_stock_handle mismatched = box_stock(wrong, 0, 0, 0, 50, 30, 20);
    check(sk_stock_restore(mismatched, shot) == SK_ERROR_INVALID_ARGUMENT, "snapshots need the same grid");
    sk_snapshot_destroy(shot);
    check(sk_stock_restore(s, shot) == SK_ERROR_INVALID_HANDLE, "destroyed snapshot is invalid");
    for (sk_stock_handle h : {s, copy, other, mismatched}) sk_stock_destroy(h);
    sk_tool_destroy(t);
}

struct MeshData {
    std::vector<float> positions;
    std::vector<uint32_t> indices, sources, colors;
};

template <typename T>
std::vector<T> mesh_stream(sk_mesh_handle m, sk_result (*copy)(sk_mesh_handle, uint8_t *, uint32_t *)) {
    uint32_t size = 0;
    copy(m, nullptr, &size);
    std::vector<T> values(size / sizeof(T));
    check(copy(m, reinterpret_cast<uint8_t *>(values.data()), &size) == SK_OK, "mesh stream");
    return values;
}

MeshData mesh_of(sk_stock_handle s, uint32_t tx, uint32_t ty, uint32_t nx, uint32_t ny, uint32_t flags,
    sk_mesh_handle *keep = nullptr) {
    sk_mesh_handle m{};
    check(sk_stock_mesh(s, tx, ty, nx, ny, flags, &m) == SK_OK, "mesh");
    MeshData d;
    d.positions = mesh_stream<float>(m, sk_mesh_copy_positions);
    d.indices = mesh_stream<uint32_t>(m, sk_mesh_copy_indices);
    d.sources = mesh_stream<uint32_t>(m, sk_mesh_copy_triangle_sources);
    d.colors = mesh_stream<uint32_t>(m, sk_mesh_copy_colors);
    if (keep) *keep = m;
    else sk_mesh_destroy(m);
    return d;
}

/** Signed volume by the divergence theorem, in double. */
double enclosed(const MeshData &d) {
    double total = 0;
    for (size_t k = 0; k < d.indices.size(); k += 3) {
        const float *a = &d.positions[3 * d.indices[k]], *b = &d.positions[3 * d.indices[k + 1]];
        const float *c = &d.positions[3 * d.indices[k + 2]];
        total += (double(a[0]) * (double(b[1]) * c[2] - double(b[2]) * c[1]) -
                     double(a[1]) * (double(b[0]) * c[2] - double(b[2]) * c[0]) +
                     double(a[2]) * (double(b[0]) * c[1] - double(b[1]) * c[0])) /
            6;
    }
    return total;
}

void preview_mesh() {
    sk_tool_handle t = ball(6, 20);
    sk_lattice g = grid(0.1, 0.2, 0.5, 90, 50, 8);
    sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
    cut(s, t, {line_move(10, 15, 15, 40, 15, 15, 0), arc_move(25, 15, 12, 6, 0, 3, 0, 1),
        line_move(5, 5, 21, 30, 25, 14, 2)}, "cut for preview");
    sk_stock_info info = info_of(s);
    MeshData closed = mesh_of(s, 0, 0, info.grids[SK_AXIS_Z].tiles[0], info.grids[SK_AXIS_Z].tiles[1], SK_MESH_BOTTOMS);
    near(enclosed(closed), info.volume, 1e-5 * info.volume, "closed preview mesh encloses the stock's volume");
    MeshData merged = mesh_of(s, 0, 0, info.grids[SK_AXIS_Z].tiles[0], info.grids[SK_AXIS_Z].tiles[1], SK_MESH_BOTTOMS | SK_MESH_MERGE);
    near(enclosed(merged), info.volume, 1e-5 * info.volume, "merged preview mesh encloses the same volume");
    check(merged.indices.size() < closed.indices.size() / 2, "merging joins the flat faces");
    // Chunks: the walls between them are emitted once, so the pieces add up to the whole.
    double pieces = 0;
    size_t triangles = 0;
    for (uint32_t ty = 0; ty < info.grids[SK_AXIS_Z].tiles[1]; ty += 3)
        for (uint32_t tx = 0; tx < info.grids[SK_AXIS_Z].tiles[0]; tx += 4) {
            MeshData piece = mesh_of(s, tx, ty, std::min(4u, info.grids[SK_AXIS_Z].tiles[0] - tx), std::min(3u, info.grids[SK_AXIS_Z].tiles[1] - ty),
                SK_MESH_BOTTOMS);
            pieces += enclosed(piece);
            triangles += piece.indices.size();
        }
    near(pieces, info.volume, 1e-5 * info.volume, "chunked meshes enclose the stock's volume");
    check(triangles == closed.indices.size(), "chunks hold the same triangles as one mesh");
    // Triangle sources name the moves; colours follow them.
    bool named = false;
    for (uint32_t source : closed.sources) named = named || source == 1;
    check(named, "the arc's surfaces carry its source");
    sk_mesh_handle m{};
    mesh_of(s, 0, 0, info.grids[SK_AXIS_Z].tiles[0], info.grids[SK_AXIS_Z].tiles[1], 0, &m);
    uint32_t palette[3] = {0xFF0000FFu, 0x00FF00FFu, 0x0000FFFFu};
    check(sk_mesh_color_by_source(m, palette, 3, 0x808080FFu, 0) == SK_OK, "colour by source");
    auto colors = mesh_stream<uint32_t>(m, sk_mesh_copy_colors);
    auto sources = mesh_stream<uint32_t>(m, sk_mesh_copy_triangle_sources);
    auto indices = mesh_stream<uint32_t>(m, sk_mesh_copy_indices);
    bool matches = true;
    for (size_t k = 0; k < sources.size(); ++k) {
        const uint8_t *rgba = reinterpret_cast<const uint8_t *>(&colors[indices[3 * k]]);
        uint32_t expected = sources[k] == SK_SOURCE_STOCK ? 0x808080FFu : palette[sources[k]];
        uint32_t actual = uint32_t(rgba[0]) << 24 | uint32_t(rgba[1]) << 16 | uint32_t(rgba[2]) << 8 | rgba[3];
        matches = matches && actual == expected;
    }
    check(matches, "each triangle is coloured by its source");
    std::vector<uint32_t> rays(size_t(90) * 50, 0x123456FFu);
    check(sk_mesh_color_by_ray(m, rays.data(), uint32_t(rays.size())) == SK_OK, "colour by ray");
    sk_mesh_destroy(m);
    mesh_of(s, 0, 0, 1, 1, SK_MESH_MERGE, &m);
    check(sk_mesh_color_by_ray(m, rays.data(), uint32_t(rays.size())) == SK_ERROR_UNSUPPORTED,
        "merged meshes cannot be coloured per ray");
    sk_mesh_destroy(m);
    check(sk_stock_mesh(s, 0, 0, info.grids[SK_AXIS_Z].tiles[0] + 1, 1, 0, &m) == SK_ERROR_INVALID_ARGUMENT, "tile range is checked");
    sk_stock_destroy(s);
    sk_tool_destroy(t);
}

/** A zig-zag of short lines, arcs, ramps and a helix over several levels, cut at each thread count. */
void thread_determinism() {
    sk_tool_handle t = bull(6, 1, 20);
    std::vector<sk_move> moves;
    uint32_t source = 0;
    for (int level = 1; level <= 3; ++level) {
        double z = 20 - 1.5 * level;
        moves.push_back(arc_move(25, 15, z + 1.5, 2, 0, -4 * kPi, -1.5, source++));
        for (double y = 4; y <= 26; y += 2.5) {
            for (double x = 4; x < 46; x += 1.7) moves.push_back(line_move(x, y, z, x + 1.7, y, z, source++));
            moves.push_back(arc_move(46, y + 1.25, z, 1.25, -kPi / 2, kPi, 0, source++));
        }
        moves.push_back(line_move(5, 5, z + 1, 45, 25, z - 0.5, source++));
    }
    // A Z grid, then all three grids of a cell-centred lattice.
    for (const sk_lattice &g : {grid(0, 0, 0.23, 218, 131, 8), lattice(0.115, 0.115, 0.115, 0.23, 218, 131, 87, 8)}) {
        const uint32_t axes = g.axes;
        std::vector<sk_interval> reference[3];
        std::vector<double> reference_removed;
        for (uint32_t threads : {1u, 2u, 3u, 8u, 0u}) {
            sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
            check(sk_stock_set_threads(s, threads) == SK_OK, "set threads");
            auto removed = cut(s, t, moves, "threaded cut");
            for (uint32_t axis = 0; axis < 3; ++axis) {
                if (!((axes >> axis) & 1)) continue;
                Rays rays = read_all(s, axis);
                if (threads == 1) {
                    reference[axis] = rays.intervals;
                    continue;
                }
                bool same = rays.intervals.size() == reference[axis].size() &&
                    std::memcmp(rays.intervals.data(), reference[axis].data(),
                        reference[axis].size() * sizeof(sk_interval)) == 0;
                check(same, "grid " + std::to_string(axis) + " with " + std::to_string(threads) +
                        " threads is bit-identical to one thread");
            }
            if (threads == 1) reference_removed = removed;
            else
                check(std::memcmp(removed.data(), reference_removed.data(), removed.size() * sizeof(double)) == 0,
                    "removed volumes with " + std::to_string(threads) + " threads are bit-identical");
            sk_stock_destroy(s);
        }
        for (uint32_t axis = 0; axis < 3; ++axis)
            if ((axes >> axis) & 1) check(reference[axis].size() > 1000, "determinism program cuts many rays");
    }
    sk_tool_destroy(t);
}

/** Box stock, a slot and mesh casting on all three grids of a cell-centred lattice. */
void tri_dexel() {
    // Nodes at cell centres of [0, 50] x [0, 30] x [0, 20], so no ray lies in a face.
    sk_lattice g = lattice(0.25, 0.25, 0.25, 0.5, 100, 60, 40);
    sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
    sk_stock_info info = info_of(s);
    const uint32_t counts[3][2] = {{60, 40}, {100, 40}, {100, 60}};
    for (uint32_t axis = 0; axis < 3; ++axis) {
        const sk_grid_info &grid = info.grids[axis];
        check(grid.present && grid.count[0] == counts[axis][0] && grid.count[1] == counts[axis][1],
            "grid " + std::to_string(axis) + " rays");
        near(grid.volume, 30000, 1e-9, "grid " + std::to_string(axis) + " sees the box's volume");
    }
    Rays along_x = read_all(s, SK_AXIS_X);
    uint32_t n;
    const sk_interval *r = along_x.at(3, 5, n);
    check(n == 1 && r[0].lo == 0 && r[0].hi == 50 && r[0].lo_normal[0] == -1 && r[0].hi_normal[0] == 1,
        "X rays cross the box with its end faces' normals");
    sk_snapshot_handle shot{};
    check(sk_stock_snapshot(s, &shot) == SK_OK, "tri-dexel snapshot");

    // A flat slot (radius 3) along X at y = 15 with its tip at z = 15, from x = 10 to 40.
    sk_tool_handle t = flat(6, 20);
    cut(s, t, {line_move(10, 15, 15, 40, 15, 15, 7)}, "tri-dexel slot");
    along_x = read_all(s, SK_AXIS_X);
    Rays along_y = read_all(s, SK_AXIS_Y);
    int wrong = 0, tested = 0;
    for (uint32_t k = 0; k < 40; ++k) {
        double z = 0.25 + 0.5 * k;
        for (uint32_t j = 0; j < 60; ++j) {
            double dy = 0.25 + 0.5 * j - 15;
            r = along_x.at(j, k, n);
            bool ok;
            if (z > 15 && std::fabs(dy) < 3) {
                ++tested;
                // The slot's rounded ends, as seen from the side.
                double w = std::sqrt(9 - dy * dy);
                ok = n == 2 && r[0].lo == 0 && std::fabs(r[0].hi - (10 - w)) < 1e-12 &&
                    std::fabs(r[1].lo - (40 + w)) < 1e-12 && r[1].hi == 50 && r[0].hi_source == 7 &&
                    r[1].lo_source == 7 && r[0].hi_normal[0] > 0 && r[1].lo_normal[0] < 0;
            } else {
                ok = n == 1 && r[0].lo == 0 && r[0].hi == 50 && r[0].hi_source == SK_SOURCE_STOCK;
            }
            if (!ok) ++wrong;
        }
        for (uint32_t i = 0; i < 100; ++i) {
            double x = 0.25 + 0.5 * i;
            double end = std::fabs(x - std::max(10.0, std::min(40.0, x)));
            r = along_y.at(i, k, n);
            bool ok;
            if (z > 15 && end < 3) {
                ++tested;
                double half = std::sqrt(9 - end * end);
                ok = n == 2 && r[0].lo == 0 && std::fabs(r[0].hi - (15 - half)) < 1e-12 &&
                    std::fabs(r[1].lo - (15 + half)) < 1e-12 && r[1].hi == 30 && r[0].hi_source == 7 &&
                    r[0].hi_normal[1] > 0 && r[1].lo_normal[1] < 0 && r[0].hi_normal[2] == 0;
            } else {
                ok = n == 1 && r[0].lo == 0 && r[0].hi == 30;
            }
            if (!ok) ++wrong;
        }
    }
    check(wrong == 0 && tested > 300, "slot seen along X and Y (" + std::to_string(wrong) + " wrong of " +
            std::to_string(tested) + ")");
    // Each grid measures the slot's volume to within its rays' resolution.
    sk_stock_info after = info_of(s);
    const double slot = (30 * 6 + kPi * 9) * 5;
    for (uint32_t axis = 0; axis < 3; ++axis)
        near(30000 - after.grids[axis].volume, slot, 0.03 * slot, "grid " + std::to_string(axis) + " slot volume");
    check(sk_stock_restore(s, shot) == SK_OK, "tri-dexel restore");
    for (uint32_t axis = 0; axis < 3; ++axis)
        check(info_of(s).grids[axis].volume == 30000, "restore returns grid " + std::to_string(axis));
    sk_snapshot_destroy(shot);
    sk_stock_destroy(s);

    // The octahedron |x - 25| + |y - 15| + |z - 10| <= 6 cast along each axis; Y rays cast a reflected copy.
    std::vector<double> o = {31, 15, 10, 19, 15, 10, 25, 21, 10, 25, 9, 10, 25, 15, 16, 25, 15, 4};
    std::vector<uint32_t> oi = {0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5};
    sk_stock_handle m{};
    check(sk_stock_create_mesh(&g, o.data(), uint32_t(o.size()), oi.data(), uint32_t(oi.size()), &m) == SK_OK,
        "octahedron on all grids");
    const double centre[3] = {25, 15, 10};
    wrong = 0;
    tested = 0;
    for (uint32_t axis = 0; axis < 2; ++axis) {
        Rays rays = read_all(m, axis);
        const uint32_t ua = axis == 0 ? 1 : 0, va = 2;
        for (uint32_t b = 0; b < 40; ++b)
            for (uint32_t a = 0; a < counts[axis][0]; ++a) {
                double du = 0.25 + 0.5 * a - centre[ua], dv = 0.25 + 0.5 * b - centre[va];
                double half = 6 - std::fabs(du) - std::fabs(dv);
                r = rays.at(a, b, n);
                if (half <= 0) {
                    if (n != 0) ++wrong;
                    continue;
                }
                ++tested;
                // Outward normals (±1, ±1, ±1) / sqrt(3), signed by the octant.
                const float third = float(1 / std::sqrt(3.0));
                bool ok = n == 1 && std::fabs(r[0].lo - (centre[axis] - half)) < 1e-12 &&
                    std::fabs(r[0].hi - (centre[axis] + half)) < 1e-12 &&
                    std::fabs(r[0].lo_normal[axis] + third) < 1e-6 && std::fabs(r[0].hi_normal[axis] - third) < 1e-6 &&
                    std::fabs(r[0].lo_normal[ua] - (du > 0 ? third : -third)) < 1e-6 &&
                    std::fabs(r[0].lo_normal[va] - (dv > 0 ? third : -third)) < 1e-6;
                if (!ok) ++wrong;
            }
    }
    check(wrong == 0 && tested > 200, "octahedron along X and Y (" + std::to_string(wrong) + " wrong of " +
            std::to_string(tested) + ")");
    sk_stock_destroy(m);

    // Stock far along +x: an X tile's material bounds are x values, which must not be compared with z.
    sk_lattice far = lattice(100.25, 0.25, 0.25, 0.5, 100, 60, 40);
    s = box_stock(far, 100, 0, 0, 150, 30, 20);
    cut(s, t, {line_move(110, 15, 15, 140, 15, 15, 1)}, "slot in stock far along x");
    info = info_of(s);
    for (uint32_t axis = 0; axis < 3; ++axis)
        near(30000 - info.grids[axis].volume, slot, 0.03 * slot, "grid " + std::to_string(axis) + " cuts stock far along x");
    sk_stock_destroy(s);
    sk_tool_destroy(t);
}

} // namespace

/** Checks a horizontal ray's intervals and their ends' normals, when given. */
void expect_spans(sk_tool_handle t, const sk_move &m, uint32_t axis, double u, double v,
    const std::vector<std::pair<double, double>> &expected, const std::string &what,
    const std::vector<float> &lo_normal = {}, const std::vector<float> &hi_normal = {}) {
    auto spans = sweep_axis(t, m, axis, u, v);
    check(spans.size() == expected.size(),
        what + ": " + std::to_string(spans.size()) + " spans, expected " + std::to_string(expected.size()));
    if (spans.size() != expected.size()) return;
    for (size_t k = 0; k < spans.size(); ++k) {
        near(spans[k].lo, expected[k].first, 1e-12, what + " lo");
        near(spans[k].hi, expected[k].second, 1e-12, what + " hi");
    }
    for (int c = 0; c < 3 && !lo_normal.empty(); ++c) near(spans[0].lo_normal[c], lo_normal[c], 1e-6, what + " lo normal");
    for (int c = 0; c < 3 && !hi_normal.empty(); ++c)
        near(spans.back().hi_normal[c], hi_normal[c], 1e-6, what + " hi normal");
}

void horizontal_level() {
    sk_tool_handle f = flat(6, 20);
    const double s8 = std::sqrt(8.0);
    // A slot along X at y = 20, tip at z = 5.
    sk_move along_x = line_move(10, 20, 5, 40, 20, 5);
    expect_spans(f, along_x, SK_AXIS_Y, 25, 8, {{17, 23}}, "Y ray across an X slot", {0, 1, 0}, {0, -1, 0});
    expect_spans(f, along_x, SK_AXIS_X, 21, 8, {{10 - s8, 40 + s8}}, "X ray along an X slot",
        {float(s8 / 3), -1.0f / 3, 0}, {float(-s8 / 3), -1.0f / 3, 0});
    expect_spans(f, along_x, SK_AXIS_X, 20, 4.9, {}, "X ray below the tip");
    expect_spans(f, along_x, SK_AXIS_X, 20, 25.1, {}, "X ray above the tool");
    expect_spans(f, along_x, SK_AXIS_X, 23.5, 8, {}, "X ray beside the slot");
    // The same slot along Y.
    sk_move along_y = line_move(20, 10, 5, 20, 40, 5);
    expect_spans(f, along_y, SK_AXIS_Y, 21, 8, {{10 - s8, 40 + s8}}, "Y ray along a Y slot",
        {-1.0f / 3, float(s8 / 3), 0});
    expect_spans(f, along_y, SK_AXIS_X, 25, 8, {{17, 23}}, "X ray across a Y slot", {1, 0, 0}, {-1, 0, 0});
    // A diagonal slot: the capsule's sides.
    sk_move diagonal = line_move(10, 10, 5, 30, 30, 5);
    expect_spans(f, diagonal, SK_AXIS_X, 20, 8, {{20 - 3 * std::sqrt(2.0), 20 + 3 * std::sqrt(2.0)}},
        "X ray across a diagonal slot");

    // A ball mill below its centre: the section at height 1 has radius sqrt(5).
    sk_tool_handle b = ball(6, 20);
    const double s5 = std::sqrt(5.0);
    expect_spans(b, along_x, SK_AXIS_X, 20, 6, {{10 - s5, 40 + s5}}, "ball section below its centre",
        {float(s5 / 3), 0, 2.0f / 3}, {float(-s5 / 3), 0, 2.0f / 3});

    // Arcs of radius 10 about (50, 50) with the flat mill (radius 3).
    sk_move circle = arc_move(50, 50, 5, 10, 0, 2 * kPi, 0);
    expect_spans(f, circle, SK_AXIS_X, 50, 8, {{37, 43}, {57, 63}}, "X ray through a circle's centre",
        {1, 0, 0}, {-1, 0, 0});
    expect_spans(f, circle, SK_AXIS_Y, 50, 8, {{37, 43}, {57, 63}}, "Y ray through a circle's centre",
        {0, 1, 0}, {0, -1, 0});
    const double s168 = std::sqrt(168.0), s48 = std::sqrt(48.0);
    for (double sweep : {kPi, -kPi}) {
        sk_move upper = arc_move(50, 50, 5, 10, sweep > 0 ? 0 : kPi, sweep, 0);
        std::string dir = sweep > 0 ? " (counter-clockwise)" : " (clockwise)";
        expect_spans(f, upper, SK_AXIS_X, 51, 8, {{50 - s168, 50 - s48}, {50 + s48, 50 + s168}},
            "X ray through an upper half circle" + dir);
        expect_spans(f, upper, SK_AXIS_X, 49, 8, {{40 - s8, 40 + s8}, {60 - s8, 60 + s8}},
            "X ray under an upper half circle meets its end discs" + dir);
        expect_spans(f, upper, SK_AXIS_Y, 50, 8, {{57, 63}}, "Y ray through an upper half circle" + dir);
    }
    // The left half circle, from (50, 60) to (50, 40), seen by Y rays.
    sk_move left = arc_move(50, 50, 5, 10, kPi / 2, kPi, 0);
    expect_spans(f, left, SK_AXIS_Y, 49, 8, {{50 - s168, 50 - s48}, {50 + s48, 50 + s168}},
        "Y ray through a left half circle");
    expect_spans(f, left, SK_AXIS_Y, 51, 8, {{40 - s8, 40 + s8}, {60 - s8, 60 + s8}},
        "Y ray beside a left half circle meets its end discs");
    // A quarter arc from (60, 50) to (50, 60): the sector's edge bounds the ray.
    sk_move quarter = arc_move(50, 50, 5, 10, 0, kPi / 2, 0);
    expect_spans(f, quarter, SK_AXIS_X, 55, 8, {{50 + std::sqrt(24.0), 62}}, "X ray through a quarter arc");
    // An arc tighter than the tool sweeps a full disc.
    sk_move tight = arc_move(50, 50, 5, 2, 0, 2 * kPi, 0);
    expect_spans(f, tight, SK_AXIS_X, 50, 8, {{45, 55}}, "X ray through a tight circle");

    // Necked tool: flutes of radius 3 to height 5, a neck of radius 2 to 15.
    sk_tool_handle n = tool({line(0, 0, 3, 0), line(3, 0, 3, 5), line(3, 5, 2, 5), line(2, 5, 2, 15),
        line(2, 15, 3, 15), line(3, 15, 3, 25)});
    sk_move neck = line_move(10, 15, 10, 40, 15, 10);
    const double w = std::sqrt(9 - 6.25);
    expect_spans(n, neck, SK_AXIS_X, 17.5, 12, {{10 - w, 40 + w}}, "X ray beside the flutes");
    expect_spans(n, neck, SK_AXIS_X, 17.5, 18, {}, "X ray beside the neck misses it");
    expect_spans(n, neck, SK_AXIS_Y, 25, 18, {{13, 17}}, "Y ray through the neck", {0, 1, 0}, {0, -1, 0});
    sk_tool_destroy(f);
    sk_tool_destroy(b);
    sk_tool_destroy(n);
}

void horizontal_plunge_ramp_helix() {
    // A 90-degree V-bit, radius 5 from height 5 up.
    sk_tool_handle v = tool({line(0, 0, 5, 5), line(5, 5, 5, 20)});
    sk_move plunge = line_move(20, 20, 10, 20, 20, 4);
    const float h = float(std::sqrt(0.5));
    expect_spans(v, plunge, SK_AXIS_Y, 20, 7, {{17, 23}}, "V-bit plunge sweeps its widest section", {0, h, h},
        {0, -h, h});
    expect_spans(v, plunge, SK_AXIS_X, 21, 7, {{20 - std::sqrt(8.0), 20 + std::sqrt(8.0)}}, "V-bit plunge off-axis");
    expect_spans(v, plunge, SK_AXIS_X, 20, 3.9, {}, "below a plunge");
    sk_move rising = line_move(20, 20, 4, 20, 20, 10);
    expect_spans(v, rising, SK_AXIS_Y, 20, 7, {{17, 23}}, "rising plunge");

    // A flat mill (radius 3) ramping down along X from (0, 0, 0) to (10, 0, -2).
    sk_tool_handle f = flat(6, 20);
    sk_move ramp = line_move(0, 0, 0, 10, 0, -2);
    expect_spans(f, ramp, SK_AXIS_X, 0, -1, {{2, 13}}, "X ray along a ramp", {1, 0, 0}, {-1, 0, 0});
    expect_spans(f, ramp, SK_AXIS_Y, 5, -1, {{-3, 3}}, "Y ray across a ramp", {0, 1, 0}, {0, -1, 0});
    expect_spans(f, ramp, SK_AXIS_X, 0, -2.5, {}, "X ray under a ramp");

    // A flat mill (radius 2) on a descending turn of radius 5 about the origin.
    sk_tool_handle g = flat(4, 10);
    sk_move helix = arc_move(0, 0, 0, 5, 0, 2 * kPi, -2);
    expect_spans(g, helix, SK_AXIS_X, 0, -1, {{-7, -3}, {3, 7}}, "X ray through the lower half of a helix");
    expect_spans(g, helix, SK_AXIS_X, 0, -1.5, {{3, 7}}, "X ray through the last quarter of a helix");
    expect_spans(g, helix, SK_AXIS_Y, 0, -1, {{-7, -3}}, "Y ray through the lower half of a helix");
    sk_tool_destroy(v);
    sk_tool_destroy(f);
    sk_tool_destroy(g);
}

/**
 * Horizontal rays against the union of densely sampled poses, each cut by
 * the ray exactly: the sampled union must lie inside the swept intervals,
 * and the swept intervals within a sampling step of it.
 */
void horizontal_sampled() {
    struct Shape {
        const char *name;
        sk_tool_handle handle;
        std::function<double(double)> radius; // section radius at height h, negative outside
    };
    std::vector<Shape> shapes = {
        {"flat", flat(6, 20), [](double h) { return h >= 0 && h <= 20 ? 3.0 : -1.0; }},
        {"ball", ball(6, 20),
            [](double h) { return h < 0 || h > 20 ? -1.0 : h < 3 ? std::sqrt(9 - (3 - h) * (3 - h)) : 3.0; }},
        {"bull", bull(10, 2, 20),
            [](double h) { return h < 0 || h > 20 ? -1.0 : h < 2 ? 3 + std::sqrt(4 - (2 - h) * (2 - h)) : 5.0; }},
        {"V-bit", tool({line(0, 0, 5, 5), line(5, 5, 5, 20)}),
            [](double h) { return h < 0 || h > 20 ? -1.0 : std::min(h, 5.0); }},
        {"necked", tool({line(0, 0, 3, 0), line(3, 0, 3, 5), line(3, 5, 2, 5), line(2, 5, 2, 15),
                           line(2, 15, 3, 15), line(3, 15, 3, 25)}),
            [](double h) { return h < 0 || h > 25 ? -1.0 : h <= 5 || h >= 15 ? 3.0 : 2.0; }},
    };
    struct Case {
        const char *name;
        sk_move move;
    };
    const std::vector<Case> cases = {
        {"level line", line_move(10, 12, 10, 38, 25, 10)},
        {"plunge", line_move(24, 18, 16, 24, 18, 6)},
        {"ramp", line_move(10, 15, 15, 40, 20, 9)},
        {"climb", line_move(12, 24, 6, 36, 12, 14)},
        {"level arc", arc_move(25, 18, 10, 8, 0.4, 3.9, 0)},
        {"helix", arc_move(25, 18, 14, 4, 0.3, 4 * kPi, -5)},
        {"rising helix", arc_move(25, 18, 8, 7, 2.0, -5.0, 3)},
    };
    const int samples = 100000;
    const double tolerance = 2e-3;
    for (const Shape &shape : shapes)
        for (const Case &c : cases) {
            int wrong = 0, tested = 0;
            unsigned seed = 12345;
            auto next = [&]() {
                seed = seed * 1664525u + 1013904223u;
                return (seed >> 8) / double(1u << 24);
            };
            for (int r = 0; r < 60; ++r) {
                uint32_t axis = r % 2 == 0 ? SK_AXIS_X : SK_AXIS_Y;
                double u = axis == SK_AXIS_X ? 8 + 22 * next() : 6 + 36 * next();
                double v = 4 + 20 * next();
                auto spans = sweep_axis(shape.handle, c.move, axis, u, v);
                // The sampled union, merging stretches closer than the tolerance.
                std::vector<std::pair<double, double>> sampled;
                for (int k = 0; k <= samples; ++k) {
                    double t = double(k) / samples, x, y, z;
                    if (c.move.kind == SK_MOVE_LINE) {
                        x = c.move.start[0] + t * (c.move.end[0] - c.move.start[0]);
                        y = c.move.start[1] + t * (c.move.end[1] - c.move.start[1]);
                        z = c.move.start[2] + t * (c.move.end[2] - c.move.start[2]);
                    } else {
                        double a = c.move.start_angle + t * c.move.sweep;
                        x = c.move.center[0] + c.move.radius * std::cos(a);
                        y = c.move.center[1] + c.move.radius * std::sin(a);
                        z = c.move.center[2] + t * c.move.rise;
                    }
                    double along = axis == SK_AXIS_X ? x : y, off = u - (axis == SK_AXIS_X ? y : x);
                    double rr = shape.radius(v - z);
                    if (rr < std::fabs(off)) continue;
                    double w = std::sqrt(rr * rr - off * off);
                    sampled.push_back({along - w, along + w});
                }
                std::sort(sampled.begin(), sampled.end());
                std::vector<std::pair<double, double>> merged;
                for (auto &p : sampled) {
                    if (!merged.empty() && p.first <= merged.back().second + 2 * tolerance)
                        merged.back().second = std::max(merged.back().second, p.second);
                    else merged.push_back(p);
                }
                if (merged.empty() && spans.empty()) continue;
                ++tested;
                bool ok = true;
                // Sampled poses are inside the swept set.
                for (auto &p : sampled) {
                    bool inside = false;
                    for (auto &s : spans) inside |= s.lo <= p.first + 1e-9 && s.hi >= p.second - 1e-9;
                    if (!inside && p.second - p.first > 1e-9) ok = false;
                }
                // The swept set is within the tolerance of them.
                for (auto &s : spans) {
                    bool close = false;
                    for (auto &p : merged) close |= s.lo >= p.first - tolerance && s.hi <= p.second + tolerance;
                    if (!close) ok = false;
                }
                if (!ok) {
                    ++wrong;
                    if (wrong <= 3) {
                        std::printf("  %s %s axis %u (%.6f, %.6f):", shape.name, c.name, axis, u, v);
                        for (auto &s : spans) std::printf(" [%.6f, %.6f]", s.lo, s.hi);
                        std::printf(" sampled");
                        for (auto &p : merged) std::printf(" [%.6f, %.6f]", p.first, p.second);
                        std::printf("\n");
                    }
                }
            }
            check(wrong == 0 && tested > 10, std::string(shape.name) + " " + c.name + " horizontal rays vs sampling (" +
                std::to_string(wrong) + " wrong of " + std::to_string(tested) + ")");
        }
    for (const Shape &shape : shapes) sk_tool_destroy(shape.handle);
}

int main() {
    flat_slot();
    ball_slot();
    bull_arc();
    plunge();
    ball_ramp();
    bull_ramp_and_helix();
    necked_tool();
    horizontal_level();
    horizontal_plunge_ramp_helix();
    horizontal_sampled();
    box_mesh();
    provenance_and_rapids();
    handles();
    thread_determinism();
    tri_dexel();
    shank_and_holder_contact();
    target_comparison();
    snapshots();
    preview_mesh();
    std::printf("%d of %d checks passed\n", checks - failures, checks);
    return failures == 0 ? 0 : 1;
}
