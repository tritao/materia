// Milestone-1 benchmark: about 50k moves on 200 x 200 mm stock with 0.25 mm rays.
// Usage: stockkit_core_bench [spacing_mm]

#include "stockkit.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

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

sk_move line_move(double x0, double y0, double z0, double x1, double y1, double z1, uint32_t source) {
    sk_move m{};
    m.struct_size = sizeof m;
    m.kind = SK_MOVE_LINE;
    m.source = source;
    m.start[0] = x0, m.start[1] = y0, m.start[2] = z0;
    m.end[0] = x1, m.end[1] = y1, m.end[2] = z1;
    return m;
}

sk_move arc_move(double cx, double cy, double z, double radius, double start, double sweep, uint32_t source) {
    sk_move m{};
    m.struct_size = sizeof m;
    m.kind = SK_MOVE_ARC;
    m.source = source;
    m.center[0] = cx, m.center[1] = cy, m.center[2] = z;
    m.radius = radius;
    m.start_angle = start;
    m.sweep = sweep;
    return m;
}

/** Zig-zag rows 3 mm apart, split into 1.5 mm lines and joined by half circles, over six levels. */
std::vector<sk_move> pocket() {
    std::vector<sk_move> moves;
    const double step = 3, segment = 1.5, lo = 3, hi = 197;
    uint32_t source = 0;
    for (int level = 1; level <= 6; ++level) {
        double z = 30 - level;
        bool forward = true;
        for (double y = lo; y <= hi - step + 1e-9; y += step) {
            int pieces = int(std::ceil((hi - lo) / segment));
            for (int k = 0; k < pieces; ++k) {
                double a = lo + (hi - lo) * k / pieces, b = lo + (hi - lo) * (k + 1) / pieces;
                if (!forward) {
                    a = hi - (a - lo);
                    b = hi - (b - lo);
                }
                moves.push_back(line_move(a, y, z, b, y, z, source++));
            }
            double x = forward ? hi : lo;
            // Half circle up to the next row, turning away from the stock's middle.
            moves.push_back(arc_move(x, y + step / 2, z, step / 2, -kPi / 2, forward ? kPi : -kPi, source++));
            forward = !forward;
        }
    }
    return moves;
}

} // namespace

int main(int argc, char **argv) {
    double spacing = argc > 1 ? std::atof(argv[1]) : 0.25;
    sk_profile_segment segments[] = {line(0, 0, 3, 0), line(3, 0, 3, 20)};
    sk_tool_handle flat{};
    if (sk_tool_create(segments, 2, &flat) != SK_OK) return 1;
    sk_profile_segment ball_segments[] = {arc(0, 3, 0, 0, 3, 3), line(3, 3, 3, 20)};
    sk_tool_handle ball{};
    if (sk_tool_create(ball_segments, 2, &ball) != SK_OK) return 1;

    sk_grid grid{};
    grid.struct_size = sizeof grid;
    grid.axis = SK_AXIS_Z;
    grid.spacing = spacing;
    grid.count[0] = grid.count[1] = uint32_t(std::lround(200 / spacing)) + 1;
    sk_box box{};
    box.struct_size = sizeof box;
    box.max[0] = box.max[1] = 200;
    box.max[2] = 30;

    std::vector<sk_move> moves = pocket();
    for (sk_tool_handle tool : {flat, ball}) {
        sk_stock_handle stock{};
        auto made = std::chrono::steady_clock::now();
        if (sk_stock_create_box(&grid, &box, &stock) != SK_OK) return 1;
        auto start = std::chrono::steady_clock::now();
        std::vector<double> removed(moves.size());
        if (sk_stock_cut(stock, tool, moves.data(), uint32_t(moves.size()), removed.data(), uint32_t(removed.size())) !=
            SK_OK)
            return 1;
        double total = 0;
        for (double r : removed) total += r;
        auto end = std::chrono::steady_clock::now();
        sk_stock_info info{};
        info.struct_size = sizeof info;
        sk_stock_get_info(stock, &info);
        std::printf("%s: %zu moves, %u x %u rays at %.3g mm: create %.3f s, cut %.3f s; "
                    "%.3g rays tested, %.3g changed, removed %.0f mm^3, %llu intervals, %.1f MB\n",
            tool.id == flat.id ? "flat 6 mm" : "ball 6 mm", moves.size(), grid.count[0], grid.count[1], spacing,
            std::chrono::duration<double>(start - made).count(), std::chrono::duration<double>(end - start).count(),
            double(info.rays_tested), double(info.rays_changed), total,
            (unsigned long long)info.interval_count, info.bytes / 1e6);
        sk_stock_destroy(stock);
    }
    return 0;
}
