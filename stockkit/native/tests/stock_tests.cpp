// Stock state checks: provenance, handles, snapshots, determinism, tri-dexel agreement, and tool contact.
// Units here are millimetres; the core is unit-agnostic.

#include "test_support.h"

using namespace sk_test;

namespace {

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

int main() {
    provenance_and_rapids();
    handles();
    thread_determinism();
    tri_dexel();
    shank_and_holder_contact();
    target_comparison();
    snapshots();
    return finish();
}
