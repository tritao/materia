// Preview and contour mesh checks: closed, consistent surfaces from the stock.
// Units here are millimetres; the core is unit-agnostic.

#include "test_support.h"

using namespace sk_test;

namespace {

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

struct MeshData {
    std::vector<float> positions, normals;
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
    check(sk_mesh_color_by_ray(m, SK_AXIS_Z, rays.data(), uint32_t(rays.size())) == SK_OK, "colour by ray");
    sk_mesh_destroy(m);
    mesh_of(s, 0, 0, 1, 1, SK_MESH_MERGE, &m);
    check(sk_mesh_color_by_ray(m, SK_AXIS_Z, rays.data(), uint32_t(rays.size())) == SK_ERROR_UNSUPPORTED,
        "merged meshes cannot be coloured per ray");
    sk_mesh_destroy(m);
    check(sk_stock_mesh(s, 0, 0, info.grids[SK_AXIS_Z].tiles[0] + 1, 1, 0, &m) == SK_ERROR_INVALID_ARGUMENT, "tile range is checked");
    sk_stock_destroy(s);
    sk_tool_destroy(t);
}

/** A contoured mesh of tiles and the edges it drew without a matching ray surface. */
MeshData contour_of(sk_stock_handle s, uint32_t tx, uint32_t ty, uint32_t nx, uint32_t ny, uint32_t *unmatched) {
    sk_mesh_handle m{};
    MeshData d = mesh_of(s, tx, ty, nx, ny, SK_MESH_CONTOUR, &m);
    sk_mesh_info info{};
    info.struct_size = sizeof info;
    check(sk_mesh_get_info(m, &info) == SK_OK, "contour mesh info");
    *unmatched = info.unmatched_edges;
    d.normals = mesh_stream<float>(m, sk_mesh_copy_normals);
    sk_mesh_destroy(m);
    return d;
}

/**
 * Closed and consistently oriented: after welding equal positions, every
 * directed edge of a non-degenerate triangle is matched by as many edges
 * the other way.
 */
bool closed_surface(const std::vector<const MeshData *> &parts) {
    std::map<std::array<float, 3>, uint32_t> welded;
    std::map<std::pair<uint32_t, uint32_t>, int> edges;
    for (const MeshData *d : parts)
        for (size_t k = 0; k < d->indices.size(); k += 3) {
            uint32_t id[3];
            for (int c = 0; c < 3; ++c) {
                const float *p = &d->positions[3 * d->indices[k + c]];
                id[c] = welded.emplace(std::array<float, 3>{p[0], p[1], p[2]}, uint32_t(welded.size())).first->second;
            }
            if (id[0] == id[1] || id[1] == id[2] || id[2] == id[0]) continue;
            for (int c = 0; c < 3; ++c) {
                uint32_t a = id[c], b = id[(c + 1) % 3];
                if (a < b) ++edges[{a, b}];
                else --edges[{b, a}];
            }
        }
    for (const auto &e : edges)
        if (e.second != 0) return false;
    return true;
}

void contour_mesh() {
    // A box between the nodes of its lattice: exact faces, edges and corners.
    {
        sk_lattice g = lattice(0.25, 0.25, 0.25, 0.5, 100, 60, 40);
        const double lo[3] = {1.1, 2.2, 0.7}, hi[3] = {48.3, 27.9, 18.35};
        sk_stock_handle s = box_stock(g, lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
        sk_stock_info info = info_of(s);
        uint32_t unmatched = 0;
        MeshData d = contour_of(s, 0, 0, info.grids[SK_AXIS_Z].tiles[0], info.grids[SK_AXIS_Z].tiles[1], &unmatched);
        check(unmatched == 0, "box: every crossing is on a ray surface");
        check(closed_surface({&d}), "box: the contoured mesh is closed");
        near(enclosed(d), (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]), 1e-3, "box: contoured volume");
        double off = 0;
        bool axial = true;
        for (size_t v = 0; v < d.positions.size() / 3; ++v) {
            double gap = 1e9;
            for (int c = 0; c < 3; ++c) {
                double p = d.positions[3 * v + c];
                off = std::max(off, std::max(lo[c] - p, p - hi[c]));
                gap = std::min({gap, std::fabs(p - lo[c]), std::fabs(p - hi[c])});
            }
            off = std::max(off, gap);
            int ones = 0;
            for (int c = 0; c < 3; ++c) ones += std::fabs(d.normals[3 * v + c]) == 1;
            axial = axial && ones == 1;
        }
        check(off < 1e-5, "box: every vertex lies on the box (" + std::to_string(off) + " off)");
        check(axial, "box: normals are the faces' normals");
        // Chunks of 3 x 2 tiles own disjoint edges and join exactly.
        std::vector<MeshData> pieces;
        size_t triangles = 0;
        for (uint32_t ty = 0; ty < info.grids[SK_AXIS_Z].tiles[1]; ty += 2)
            for (uint32_t tx = 0; tx < info.grids[SK_AXIS_Z].tiles[0]; tx += 3) {
                pieces.push_back(contour_of(s, tx, ty, std::min(3u, info.grids[SK_AXIS_Z].tiles[0] - tx),
                    std::min(2u, info.grids[SK_AXIS_Z].tiles[1] - ty), &unmatched));
                triangles += pieces.back().indices.size();
            }
        std::vector<const MeshData *> parts;
        for (const MeshData &p : pieces) parts.push_back(&p);
        check(triangles == d.indices.size(), "box: chunks hold the same triangles as one mesh");
        check(closed_surface(parts), "box: chunks join into a closed surface");
        // Stock past the lattice is closed off half a cell outside the last nodes.
        sk_stock_destroy(s);
        s = box_stock(g, -5, -5, -5, 60, 40, 30);
        d = contour_of(s, 0, 0, info.grids[SK_AXIS_Z].tiles[0], info.grids[SK_AXIS_Z].tiles[1], &unmatched);
        check(unmatched > 0 && closed_surface({&d}), "box past the lattice: closed at the lattice's edge");
        near(enclosed(d), 50.0 * 30 * 20, 1e-3, "box past the lattice: the lattice's cells");
        sk_stock_destroy(s);
    }
    // Slots: the flat mill's walls and rounded ends, and the ball mill's floor.
    {
        sk_lattice g = lattice(0.25, 0.25, 0.25, 0.5, 100, 60, 40);
        const double pi = 3.14159265358979323846;
        struct Case { const char *name; sk_tool_handle tool; double removed, tolerance; };
        Case cases[] = {
            {"flat slot", flat(6, 20), (30 * 6 + pi * 9) * 5, 1e-3},
            {"ball slot", ball(6, 20), 30 * (pi * 9 / 2 + 12) + (2 * pi * 27 / 3 + pi * 9 * 2), 2e-3},
        };
        for (Case &c : cases) {
            sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
            cut(s, c.tool, {line_move(10, 15, 15, 40, 15, 15, 7)}, c.name);
            sk_stock_info info = info_of(s);
            uint32_t unmatched = 0;
            MeshData d = contour_of(s, 0, 0, info.grids[SK_AXIS_Z].tiles[0], info.grids[SK_AXIS_Z].tiles[1], &unmatched);
            std::string name = c.name;
            check(unmatched == 0, name + ": every crossing is on a ray surface");
            check(closed_surface({&d}), name + ": closed");
            double volume = enclosed(d);
            near(volume, 30000 - c.removed, c.tolerance * c.removed,
                name + ": contoured volume (" + std::to_string(30000 - volume) + " removed)");
            bool cut_faces = false, walls = false;
            for (size_t k = 0; k < d.sources.size(); ++k) {
                if (d.sources[k] != 7) continue;
                cut_faces = true;
                const float *n = &d.normals[3 * d.indices[3 * k]];
                walls = walls || (std::fabs(n[2]) < 1e-6 && std::fabs(n[1]) > 0.99);
            }
            check(cut_faces && walls, name + ": its walls carry the move's source and wall normals");
            sk_stock_destroy(s);
            sk_tool_destroy(c.tool);
        }
    }
    // A finishing pass shaving 0.1 mm off a wall changes no Z ray, yet renews
    // the Z tiles it passes over and moves the contoured wall.
    {
        sk_lattice g = lattice(0.25, 0.25, 0.25, 0.5, 100, 60, 40);
        sk_stock_handle s = box_stock(g, 0, 0, 0, 50, 30, 20);
        sk_tool_handle t = flat(6, 30);
        sk_stock_info info = info_of(s);
        uint32_t unmatched = 0;
        const uint32_t tiles_x = info.grids[SK_AXIS_Z].tiles[0], tiles_y = info.grids[SK_AXIS_Z].tiles[1];
        MeshData before = contour_of(s, 0, 0, tiles_x, tiles_y, &unmatched);
        std::vector<uint64_t> z_before = revisions_of(s);
        sk_snapshot_handle shot{};
        check(sk_stock_snapshot(s, &shot) == SK_OK, "shave snapshot");
        cut(s, t, {line_move(-2.9, -5, -1, -2.9, 35, -1, 3)}, "shave the -x wall");
        info = info_of(s);
        near(info.grids[SK_AXIS_Z].volume, 30000, 1e-9, "shave: the Z grid sees nothing");
        near(info.grids[SK_AXIS_X].volume, 30000 - 0.1 * 30 * 20, 1e-6, "shave: the X grid sees it");
        std::vector<uint64_t> z_after = revisions_of(s);
        bool renewed = z_after[0] != z_before[0] && z_after[(tiles_y - 1) * tiles_x] != z_before[(tiles_y - 1) * tiles_x];
        bool spared = z_after[tiles_x - 1] == z_before[tiles_x - 1] && z_after[tiles_y * tiles_x - 1] == z_before[tiles_y * tiles_x - 1];
        check(renewed && spared, "shave: renews the revisions of the Z tiles along the wall only");
        MeshData after = contour_of(s, 0, 0, tiles_x, tiles_y, &unmatched);
        near(enclosed(before) - enclosed(after), 0.1 * 30 * 20, 1e-3, "shave: the contoured wall moves");
        z_before = z_after;
        check(sk_stock_restore(s, shot) == SK_OK, "shave restore");
        z_after = revisions_of(s);
        check(z_after[0] != z_before[0] && z_after[(tiles_y - 1) * tiles_x] != z_before[(tiles_y - 1) * tiles_x],
            "shave: restoring X and Y tiles renews the Z tiles over them");
        sk_snapshot_destroy(shot);
        sk_stock_destroy(s);
        sk_tool_destroy(t);
    }
    // A through-slot 0.4 mm too wide and 0.2 mm too shallow: its walls are
    // gouged by 0.2 mm between Z rays, which only Y rays see, and its floor
    // keeps 0.2 mm of leftover. Colouring by deviation marks each surface by
    // what its own ray finds there.
    {
        sk_lattice g = lattice(0.25, 0.25, 0.25, 0.5, 100, 60, 40);
        sk_stock_handle target = box_stock(g, 0, 0, 0, 50, 30, 20), s = box_stock(g, 0, 0, 0, 50, 30, 20);
        sk_tool_handle right = flat(6, 20), wide = flat(6.4, 20);
        cut(target, right, {line_move(-10, 15, 14.8, 60, 15, 14.8, 0)}, "target slot");
        cut(s, wide, {line_move(-10, 1, 30, 60, 1, 30, 0), line_move(-10, 15, 15, 60, 15, 15, 1)}, "wide slot");
        sk_stock_info info = info_of(s);
        double deepest[3] = {0, 0, 0}, thickest[3] = {0, 0, 0};
        bool named = true;
        for (uint32_t axis = 0; axis < 3; ++axis) {
            const sk_grid_info &grid = info.grids[axis];
            uint32_t n = grid.count[0] * grid.count[1];
            std::vector<sk_ray_comparison> rays(n);
            check(sk_stock_compare(s, target, axis, 0, 0, grid.count[0], grid.count[1], rays.data(), n) == SK_OK,
                "slot comparison");
            for (const sk_ray_comparison &r : rays) {
                deepest[axis] = std::max(deepest[axis], r.largest_gouge);
                thickest[axis] = std::max(thickest[axis], r.largest_leftover);
                if (r.largest_gouge > 0.1) named = named && r.gouge_source == 1;
            }
        }
        check(deepest[SK_AXIS_Z] == 0 && deepest[SK_AXIS_X] == 0, "wall gouge: no Z or X ray sees it");
        near(deepest[SK_AXIS_Y], 0.2, 1e-9, "wall gouge: Y rays see 0.2 mm");
        near(thickest[SK_AXIS_Z], 0.2, 1e-9, "floor leftover: Z rays see 0.2 mm");
        check(named, "wall gouge: the wide slot's move made it");
        sk_mesh_handle m{};
        const uint32_t tiles_x = info.grids[SK_AXIS_Z].tiles[0], tiles_y = info.grids[SK_AXIS_Z].tiles[1];
        check(sk_stock_mesh(s, 0, 0, tiles_x, tiles_y, SK_MESH_CONTOUR, &m) == SK_OK, "slot mesh");
        const uint32_t green = 0x00FF00FFu, yellow = 0xFFFF00FFu, red = 0xFF0000FFu;
        check(sk_mesh_color_by_deviation(m, target, 0.1, green, yellow, red) == SK_OK, "colour by deviation");
        auto mesh_colors = mesh_stream<uint32_t>(m, sk_mesh_copy_colors);
        auto normals = mesh_stream<float>(m, sk_mesh_copy_normals);
        auto positions = mesh_stream<float>(m, sk_mesh_copy_positions);
        int walls = 0, floors = 0, faces = 0, wrong = 0;
        for (size_t v = 0; v < mesh_colors.size(); ++v) {
            const uint8_t *rgba = reinterpret_cast<const uint8_t *>(&mesh_colors[v]);
            uint32_t color = uint32_t(rgba[0]) << 24 | uint32_t(rgba[1]) << 16 | uint32_t(rgba[2]) << 8 | rgba[3];
            const float *p = &positions[3 * v], *n = &normals[3 * v];
            bool inner = p[0] > 1 && p[0] < 49;
            if (inner && std::fabs(n[1]) > 0.99 && p[2] > 15.5 && std::fabs(std::fabs(p[1] - 15) - 3.2) < 0.01) {
                ++walls; // the slot's walls
                wrong += color != red;
            } else if (inner && n[2] > 0.99 && std::fabs(p[2] - 15) < 1e-4 && std::fabs(p[1] - 15) < 2.5) {
                ++floors; // the slot's floor, away from its walls
                wrong += color != yellow;
            } else if (inner && std::fabs(n[1]) > 0.99 && (p[1] < 0.01 || p[1] > 29.99)) {
                ++faces; // the box's sides, on the same Y rays as the walls
                wrong += color != green;
            }
        }
        check(walls > 0 && floors > 0 && faces > 0 && wrong == 0, "deviation colours: walls gouged, floor leftover, "
            "sides on target (" + std::to_string(wrong) + " of " + std::to_string(walls + floors + faces) + " wrong)");
        // Colouring by ray takes each grid's colours for the quads made from its rays.
        std::vector<uint32_t> ys(size_t(info.grids[SK_AXIS_Y].count[0]) * info.grids[SK_AXIS_Y].count[1], red);
        check(sk_mesh_color_by_ray(m, SK_AXIS_Y, ys.data(), uint32_t(ys.size())) == SK_OK, "colour by Y ray");
        check(sk_mesh_color_by_ray(m, 3, ys.data(), uint32_t(ys.size())) == SK_ERROR_INVALID_ARGUMENT,
            "colour by ray checks the axis");
        check(sk_mesh_color_by_ray(m, SK_AXIS_Y, ys.data(), uint32_t(ys.size() - 1)) == SK_ERROR_INVALID_ARGUMENT,
            "colour by ray needs every ray of the grid");
        mesh_colors = mesh_stream<uint32_t>(m, sk_mesh_copy_colors);
        bool only_y = true;
        for (size_t v = 0; v < mesh_colors.size(); ++v) {
            const uint8_t *rgba = reinterpret_cast<const uint8_t *>(&mesh_colors[v]);
            bool is_red = rgba[0] == 0xFF && rgba[1] == 0 && rgba[2] == 0;
            if (std::fabs(normals[3 * v + 2]) > 0.99 && is_red) only_y = false; // Z quads keep their colour
        }
        check(only_y, "colour by Y ray leaves Z quads alone");
        sk_mesh_destroy(m);
        // A 1.2 mm groove in the top: along the Y rays through it the gouge
        // ends where the stock resumes across the groove, not where the part does.
        sk_tool_handle narrow = flat(1.2, 20);
        cut(s, narrow, {line_move(-10, 5, 19.7, 60, 5, 19.7, 2)}, "groove");
        check(sk_stock_mesh(s, 0, 0, tiles_x, tiles_y, SK_MESH_CONTOUR, &m) == SK_OK, "groove mesh");
        check(sk_mesh_color_by_deviation(m, target, 2, green, yellow, red) == SK_OK, "colour the groove");
        mesh_colors = mesh_stream<uint32_t>(m, sk_mesh_copy_colors);
        normals = mesh_stream<float>(m, sk_mesh_copy_normals);
        positions = mesh_stream<float>(m, sk_mesh_copy_positions);
        int groove = 0, gouged = 0;
        for (size_t v = 0; v < mesh_colors.size(); ++v) {
            const float *p = &positions[3 * v];
            if (std::fabs(normals[3 * v + 1]) < 0.99 || p[2] < 19.7 || std::fabs(std::fabs(p[1] - 5) - 0.6) > 0.01) continue;
            ++groove;
            const uint8_t *rgba = reinterpret_cast<const uint8_t *>(&mesh_colors[v]);
            gouged += rgba[1] == 0;
        }
        check(groove > 0 && gouged == 0, "groove walls gouge only the groove's width (" + std::to_string(gouged) +
            " of " + std::to_string(groove) + " marked)");
        sk_mesh_destroy(m);
        sk_tool_destroy(narrow);
        check(sk_stock_mesh(s, 0, 0, 1, 1, 0, &m) == SK_OK &&
                sk_mesh_color_by_deviation(m, target, 0.1, green, yellow, red) == SK_ERROR_UNSUPPORTED,
            "colour by deviation needs a contoured mesh");
        sk_mesh_destroy(m);
        sk_stock_destroy(s);
        sk_stock_destroy(target);
        sk_tool_destroy(right);
        sk_tool_destroy(wide);
    }
    // Contouring needs all three grids and takes no other flags.
    {
        sk_stock_handle s = box_stock(grid(0, 0, 0.5, 20, 20), 0, 0, 0, 5, 5, 5);
        sk_mesh_handle m{};
        check(sk_stock_mesh(s, 0, 0, 1, 1, SK_MESH_CONTOUR, &m) == SK_ERROR_UNSUPPORTED, "contour needs X and Y grids");
        sk_stock_destroy(s);
        s = box_stock(lattice(0, 0, 0, 0.5, 20, 20, 20), 0, 0, 0, 5, 5, 5);
        check(sk_stock_mesh(s, 0, 0, 1, 1, SK_MESH_CONTOUR | SK_MESH_BOTTOMS, &m) == SK_ERROR_INVALID_ARGUMENT,
            "contour takes no column flags");
        sk_stock_destroy(s);
    }
}

} // namespace

int main() {
    box_mesh();
    preview_mesh();
    contour_mesh();
    return finish();
}
