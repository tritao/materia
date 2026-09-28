#include "preview.hpp"

#include <algorithm>

namespace stockkit {

namespace {

constexpr uint32_t kNoRay = 0xFFFFFFFFu;

struct Builder {
    PreviewMesh &mesh;

    /** Adds a quad a, b, c, d (counter-clockwise seen from its front) with one normal. */
    void quad(const float a[3], const float b[3], const float c[3], const float d[3], const float normal[3],
        uint32_t source, uint32_t ray) {
        uint32_t base = mesh.vertex_count();
        for (const float *p : {a, b, c, d}) {
            mesh.positions.insert(mesh.positions.end(), p, p + 3);
            mesh.normals.insert(mesh.normals.end(), normal, normal + 3);
            mesh.vertex_sources.push_back(source);
            mesh.vertex_rays.push_back(ray);
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        mesh.triangle_sources.insert(mesh.triangle_sources.end(), {source, source});
    }

    void horizontal(double x0, double x1, double y0, double y1, double z, const float normal[3], uint32_t source,
        uint32_t ray, bool up) {
        float a[3] = {float(x0), float(y0), float(z)}, b[3] = {float(x1), float(y0), float(z)};
        float c[3] = {float(x1), float(y1), float(z)}, d[3] = {float(x0), float(y1), float(z)};
        if (up) quad(a, b, c, d, normal, source, ray);
        else quad(a, d, c, b, normal, source, ray);
    }

    /** A wall in the plane x = at (facing +x or -x) between y0 and y1. */
    void wall_x(double at, double y0, double y1, double lo, double hi, bool positive, uint32_t source, uint32_t ray) {
        float a[3] = {float(at), float(y0), float(lo)}, b[3] = {float(at), float(y1), float(lo)};
        float c[3] = {float(at), float(y1), float(hi)}, d[3] = {float(at), float(y0), float(hi)};
        float normal[3] = {positive ? 1.f : -1.f, 0, 0};
        if (positive) quad(a, b, c, d, normal, source, ray);
        else quad(a, d, c, b, normal, source, ray);
    }

    /** A wall in the plane y = at (facing +y or -y) between x0 and x1. */
    void wall_y(double at, double x0, double x1, double lo, double hi, bool positive, uint32_t source, uint32_t ray) {
        float a[3] = {float(x1), float(at), float(lo)}, b[3] = {float(x0), float(at), float(lo)};
        float c[3] = {float(x0), float(at), float(hi)}, d[3] = {float(x1), float(at), float(hi)};
        float normal[3] = {0, positive ? 1.f : -1.f, 0};
        if (positive) quad(a, b, c, d, normal, source, ray);
        else quad(a, d, c, b, normal, source, ray);
    }
};

/**
 * The move whose cut made a wall: the neighbour's surface just below or just
 * above the stretch of material it lacks, else whichever of the material's
 * own surfaces ends the stretch, else the original stock.
 */
uint32_t wall_source(const Interval *neighbour_below, const Interval *neighbour_above, const Interval &own,
    double lo, double hi) {
    if (neighbour_below) return neighbour_below->hi_source;
    if (neighbour_above) return neighbour_above->lo_source;
    if (own.hi == hi) return own.hi_source;
    if (own.lo == lo) return own.lo_source;
    return kSourceStock;
}

/** Calls `stretch(lo, hi, source)` for the parts of column `a` that column `b` lacks. */
template <typename F>
void exposed(const std::vector<Interval> &a, const std::vector<Interval> &b, F &&stretch) {
    difference(a, b, [&](double lo, double hi, const Interval *below, const Interval *above) {
        // Find the column-a interval holding the stretch for its own sources.
        const Interval *own = &a.front();
        for (const Interval &v : a)
            if (v.lo <= lo && v.hi >= hi) own = &v;
        stretch(lo, hi, wall_source(below, above, *own, lo, hi));
    });
}

bool same_face(double z, const float *normal, uint32_t source, double z2, const float *normal2, uint32_t source2) {
    return z == z2 && source == source2 && normal[0] == normal2[0] && normal[1] == normal2[1] &&
        normal[2] == normal2[2];
}

/** A top or bottom face being extended along a row. */
struct Open {
    double x0, x1, z;
    float normal[3];
    uint32_t source;
    uint32_t ray;
    bool up;
};

} // namespace

void build_preview(const Stock &stock, uint32_t tile_x, uint32_t tile_y, uint32_t tiles_x, uint32_t tiles_y,
    const PreviewOptions &options, PreviewMesh &out) {
    out = PreviewMesh{};
    out.merged = options.merge;
    Builder builder{out};
    const Grid &grid = stock.grid();
    const double s = grid.spacing, half = s / 2;
    const uint32_t nx = grid.count[0], ny = grid.count[1];
    const uint32_t i0 = std::min(nx, tile_x * grid.tile), i1 = std::min(nx, (tile_x + tiles_x) * grid.tile);
    const uint32_t j0 = std::min(ny, tile_y * grid.tile), j1 = std::min(ny, (tile_y + tiles_y) * grid.tile);
    std::vector<Interval> column, right, up, below, left;
    std::vector<Open> open, next;
    auto emit = [&](const Open &face, double y0, double y1) {
        builder.horizontal(face.x0, face.x1, y0, y1, face.z, face.normal, face.source,
            options.merge ? kNoRay : face.ray, face.up);
    };
    for (uint32_t j = j0; j < j1; ++j) {
        const double y = stock.ray_v(j), y0 = y - half, y1 = y + half;
        open.clear();
        for (uint32_t i = i0; i < i1; ++i) {
            const double x = stock.ray_u(i), x0 = x - half, x1 = x + half;
            const uint32_t ray = j * nx + i;
            stock.read(i, j, column);
            // Top and bottom faces, extended from the previous column where they match.
            next.clear();
            for (const Interval &v : column)
                for (int side = 0; side < (options.bottoms ? 2 : 1); ++side) {
                    bool top = side == 0;
                    Open face{x0, x1, top ? v.hi : v.lo, {}, top ? v.hi_source : v.lo_source, ray, top};
                    std::copy_n(top ? v.hi_normal : v.lo_normal, 3, face.normal);
                    bool extended = false;
                    if (options.merge)
                        for (Open &candidate : open)
                            if (candidate.up == top && candidate.x1 == x0 &&
                                same_face(candidate.z, candidate.normal, candidate.source, face.z, face.normal,
                                    face.source)) {
                                candidate.x1 = x1;
                                next.push_back(candidate);
                                candidate.x1 = -1; // taken
                                extended = true;
                                break;
                            }
                    if (!extended) next.push_back(face);
                }
            for (const Open &face : open)
                if (face.x1 != -1) emit(face, y0, y1);
            open.swap(next);
            if (!options.merge) {
                for (const Open &face : open) emit(face, y0, y1);
                open.clear();
            }

            // Walls against the +x neighbour (both ways), and the grid's -x edge.
            if (i + 1 < nx) stock.read(i + 1, j, right);
            else right.clear();
            exposed(column, right, [&](double lo, double hi, uint32_t source) {
                builder.wall_x(x1, y0, y1, lo, hi, true, source, ray);
            });
            if (i + 1 < nx)
                exposed(right, column, [&](double lo, double hi, uint32_t source) {
                    builder.wall_x(x1, y0, y1, lo, hi, false, source, ray + 1);
                });
            if (i == 0)
                for (const Interval &v : column) builder.wall_x(x0, y0, y1, v.lo, v.hi, false, kSourceStock, ray);
            // Walls against the +y neighbour, and the grid's -y edge.
            if (j + 1 < ny) stock.read(i, j + 1, up);
            else up.clear();
            exposed(column, up, [&](double lo, double hi, uint32_t source) {
                builder.wall_y(y1, x0, x1, lo, hi, true, source, ray);
            });
            if (j + 1 < ny)
                exposed(up, column, [&](double lo, double hi, uint32_t source) {
                    builder.wall_y(y1, x0, x1, lo, hi, false, source, ray + nx);
                });
            if (j == 0)
                for (const Interval &v : column) builder.wall_y(y0, x0, x1, v.lo, v.hi, false, kSourceStock, ray);
        }
        for (const Open &face : open) emit(face, y0, y1);
    }
    if (options.merge) out.vertex_rays.clear();
}

} // namespace stockkit
