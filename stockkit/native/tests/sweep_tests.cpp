// Closed-form sweep checks: each cutter shape along lines, arcs, ramps and helices, vertical and horizontal.
// Units here are millimetres; the core is unit-agnostic.

#include "test_support.h"

using namespace sk_test;

namespace {

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

/** A flat disc of diameter `d` and height `h`: swept along a line it lays a bead of that width and height. */
std::vector<double> add(sk_stock_handle s, sk_tool_handle t, const std::vector<sk_move> &moves, const std::string &what) {
    std::vector<double> added(moves.size());
    sk_add_summary summary{};
    check(sk_stock_add(s, t, moves.data(), uint32_t(moves.size()), added.data(), &summary) == SK_OK, what);
    double total = 0;
    for (double volume : added) total += volume;
    check(summary.moves == moves.size() && summary.added == total, what + ": summary adds up the moves");
    return added;
}

/** Material is built up from empty stock: volumes, unions, provenance, normals, every grid and any thread count. */
void adding() {
    const double spacing = 0.1;
    // 14 x 8 x 4 mm of lattice, with nodes off the faces of what is added (a bead 0.5 wide holds five rays).
    sk_lattice g = lattice(-1.043, -1.037, -0.553, spacing, 140, 80, 40);
    sk_stock_handle empty{};
    check(sk_stock_create_empty(&g, &empty) == SK_OK, "empty stock");
    sk_stock_info before = info_of(empty);
    check(before.volume == 0 && before.grids[0].interval_count == 0 && before.grids[2].interval_count == 0, "empty stock holds nothing");
    sk_stock_handle none{};
    sk_lattice bad = g;
    bad.spacing = 0;
    check(sk_stock_create_empty(&bad, &none) == SK_ERROR_INVALID_ARGUMENT, "empty stock needs a lattice");
    check(sk_stock_create_empty(&g, nullptr) == SK_ERROR_INVALID_ARGUMENT, "empty stock needs somewhere to put the handle");

    // A bead 0.5 wide and 0.2 high: a disc of diameter 0.5, 0.2 tall, tip down, along x. Volume: width x height x
    // length plus the two end caps (a disc's own volume).
    sk_tool_handle bead = flat(0.5, 0.2);
    const double w = 0.5, h = 0.2, length = 8.0;
    const double disc = kPi * (w / 2) * (w / 2) * h;
    std::vector<double> first = add(empty, bead, {line_move(0, 0, 0, length, 0, 0, 7)}, "one bead");
    const double expected = w * h * length + disc;
    near(first[0], expected, 0.06 * expected, "a bead adds its swept volume");
    sk_stock_info one = info_of(empty);
    near(one.volume, first[0], 1e-12, "the stock holds what was added");
    for (int axis = 0; axis < 3; ++axis)
        near(one.grids[axis].volume, one.volume, 0.08 * one.volume, "every grid sees the bead's volume");

    // The same move again adds nothing; a move overlapping it adds only what is new.
    std::vector<double> again = add(empty, bead, {line_move(0, 0, 0, length, 0, 0, 8)}, "the same bead again");
    near(again[0], 0, 1e-12, "a bead inside material adds nothing");
    sk_stock_info same = info_of(empty);
    near(same.volume, one.volume, 1e-12, "and the stock does not change");
    std::vector<double> longer = add(empty, bead, {line_move(length / 2, 0, 0, length + 4, 0, 0, 9)}, "an overlapping bead");
    near(longer[0], w * h * 4, 0.1 * (w * h * 4), "an overlapping bead adds only its new stretch");
    near(info_of(empty).volume, w * h * (length + 4) + disc, 0.06 * (w * h * (length + 4) + disc), "the union is one longer bead");

    // Provenance and normals: the top of the bead faces up and records the move that laid it; its start is
    // the first move's, the stretch added later the later move's.
    Rays rays = read_all(empty);
    uint32_t n = 0;
    // Ray over the middle of the first stretch: x = 2, y = 0.
    uint32_t i = uint32_t(std::lround((2.0 - g.origin[0]) / spacing)), j = uint32_t(std::lround((0.0 - g.origin[1]) / spacing));
    const sk_interval *middle = rays.at(i, j, n);
    check(n == 1, "one interval over the bead");
    near(middle[0].lo, 0, 1e-9, "the bead's underside is at the tip");
    near(middle[0].hi, h, 1e-9, "its top is a layer height up");
    check(middle[0].hi_normal[2] > 0.99f && middle[0].lo_normal[2] < -0.99f, "the top faces up and the underside down");
    check(middle[0].lo_source == 7 && middle[0].hi_source == 7, "and both record the move that made them");
    uint32_t end_i = uint32_t(std::lround((10.0 - g.origin[0]) / spacing));
    const sk_interval *tail = rays.at(end_i, j, n);
    check(n == 1 && tail[0].hi_source == 9 && tail[0].lo_source == 9, "the stretch added later records the later move");

    // A second layer on top of the first stands on it: one interval of twice the height, the top the new move's.
    std::vector<double> top = add(empty, bead, {line_move(0, 0, h, length, 0, h, 11)}, "a second layer");
    check(top[0] > 0.5 * w * h * length, "the second layer adds a bead's worth");
    Rays stacked = read_all(empty);
    const sk_interval *column = stacked.at(i, j, n);
    check(n == 1, "a bead on a bead is one interval");
    near(column[0].hi, 2 * h, 1e-9, "twice a layer high");
    check(column[0].lo_source == 7 && column[0].hi_source == 11, "bottom from the first layer, top from the second");

    // The thread count changes nothing.
    sk_stock_handle serial{}, parallel{};
    check(sk_stock_create_empty(&g, &serial) == SK_OK && sk_stock_create_empty(&g, &parallel) == SK_OK, "two empty stocks");
    sk_stock_set_threads(serial, 1);
    sk_stock_set_threads(parallel, 4);
    std::vector<sk_move> program;
    for (int row = 0; row < 12; ++row) {
        double y = -2.0 + 0.4 * row;
        program.push_back(line_move(row % 2 ? 6 : -4, y, 0, row % 2 ? -4 : 6, y, 0, uint32_t(100 + row)));
    }
    program.push_back(arc_move(1, 1, 0.2, 1.5, 0, 4.0, 0.3, 200));
    std::vector<double> a = add(serial, bead, program, "serial add"), b = add(parallel, bead, program, "parallel add");
    bool identical = a.size() == b.size();
    for (size_t k = 0; identical && k < a.size(); ++k) identical = a[k] == b[k];
    check(identical && info_of(serial).volume == info_of(parallel).volume, "adding is bit-identical for any thread count");

    // Adding is the inverse of cutting: add a bead, cut the same sweep out, nothing is left.
    sk_stock_handle round{};
    check(sk_stock_create_empty(&g, &round) == SK_OK, "round trip stock");
    add(round, bead, {line_move(0, 0, 0, 5, 0, 0, 1)}, "round trip add");
    check(info_of(round).volume > 0, "the round trip stock holds a bead");
    cut(round, bead, {line_move(0, 0, 0, 5, 0, 0, 2)}, "round trip cut");
    near(info_of(round).volume, 0, 1e-9, "cutting the same sweep leaves nothing");

    // A bad move leaves the stock as it was.
    sk_move invalid = line_move(0, 0, 0, 1, 0, 0);
    invalid.struct_size = 1;
    double volume = 0;
    sk_add_summary summary{};
    double stock_before = info_of(empty).volume;
    check(sk_stock_add(empty, bead, &invalid, 1, &volume, &summary) == SK_ERROR_INVALID_ARGUMENT, "a malformed move is refused");
    check(info_of(empty).volume == stock_before, "and adds nothing");
    sk_stock_handle stale{};
    check(sk_stock_add(stale, bead, &invalid, 1, &volume, &summary) == SK_ERROR_INVALID_HANDLE, "a stale stock is refused");

    sk_tool_destroy(bead);
    for (sk_stock_handle s : {empty, serial, parallel, round}) sk_stock_destroy(s);
}

} // namespace

int main() {
    adding();
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
    return finish();
}
