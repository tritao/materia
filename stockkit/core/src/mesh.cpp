#include "mesh.hpp"

#include <algorithm>
#include <cmath>

namespace stockkit {

namespace {

// Exact 2D orientation (Shewchuk's expansion arithmetic, with a fast filter).

void two_sum(double a, double b, double &x, double &y) {
    x = a + b;
    double bv = x - a, av = x - bv;
    y = (a - av) + (b - bv);
}

void two_product(double a, double b, double &x, double &y) {
    x = a * b;
    y = std::fma(a, b, -x);
}

/** Adds `b` to the nonoverlapping expansion `e` (grow-expansion). */
void grow(std::vector<double> &e, double b) {
    double q = b;
    std::vector<double> out;
    out.reserve(e.size() + 1);
    for (double term : e) {
        double sum, err;
        two_sum(q, term, sum, err);
        if (err != 0) out.push_back(err);
        q = sum;
    }
    if (q != 0) out.push_back(q);
    e.swap(out);
}

/** Sign of (bx - ax)(cy - ay) - (by - ay)(cx - ax), exactly. */
int orient(double ax, double ay, double bx, double by, double cx, double cy) {
    double l = (bx - ax) * (cy - ay), r = (by - ay) * (cx - ax);
    double det = l - r;
    double bound = (3.3306690738754716e-16 + 1e-30) * (std::fabs(l) + std::fabs(r));
    if (det > bound) return 1;
    if (-det > bound) return -1;
    // Each difference is an exact two-term expansion; multiply out.
    double b[2], c[2], d[2], f[2];
    two_sum(bx, -ax, b[0], b[1]);
    two_sum(cy, -ay, c[0], c[1]);
    two_sum(by, -ay, d[0], d[1]);
    two_sum(cx, -ax, f[0], f[1]);
    std::vector<double> e;
    for (double u : b)
        for (double v : c) {
            double x, y;
            two_product(u, v, x, y);
            grow(e, x);
            grow(e, y);
        }
    for (double u : d)
        for (double v : f) {
            double x, y;
            two_product(u, v, x, y);
            grow(e, -x);
            grow(e, -y);
        }
    // Nonoverlapping and increasing in magnitude: the last term decides.
    if (e.empty()) return 0;
    return e.back() > 0 ? 1 : -1;
}

/** Edge ownership: of two opposite directed edges, exactly one owns their points. */
bool owns(double dx, double dy) {
    return dy < 0 || (dy == 0 && dx > 0);
}

struct Hit {
    double z;
    int winding; // +1 entering material going up, -1 leaving
    float normal[3];
};

} // namespace

bool cast_mesh(DexelGrid &stock, const double *world_positions, uint32_t position_count,
    const uint32_t *world_indices, uint32_t index_count, std::string &error) {
    if (position_count % 3 != 0 || index_count % 3 != 0) {
        error = "mesh positions and indices must come in triples";
        return false;
    }
    const uint32_t vertices = position_count / 3;
    for (uint32_t k = 0; k < index_count; ++k)
        if (world_indices[k] >= vertices) {
            error = "mesh index out of range";
            return false;
        }
    for (uint32_t k = 0; k < position_count; ++k)
        if (!std::isfinite(world_positions[k])) {
            error = "mesh positions must be finite";
            return false;
        }
    const Grid &grid = stock.grid();
    // Cast in (u, v, w) with the ray along w. For Y rays (x, z, y) is a
    // reflection, so triangles are reversed to keep their outward side;
    // normals come back to world axes by the same permutation.
    const uint32_t axes[3] = {grid.u_axis(), grid.v_axis(), grid.axis};
    const bool reflected = grid.axis == 1;
    std::vector<double> permuted(position_count);
    for (uint32_t k = 0; k < position_count; k += 3)
        for (int c = 0; c < 3; ++c) permuted[k + c] = world_positions[k + axes[c]];
    std::vector<uint32_t> reordered(world_indices, world_indices + index_count);
    if (reflected)
        for (uint32_t t = 0; t + 2 < index_count; t += 3) std::swap(reordered[t + 1], reordered[t + 2]);
    const double *positions = permuted.data();
    const uint32_t *indices = reordered.data();
    const double s = grid.spacing;
    std::vector<std::vector<Hit>> hits(static_cast<size_t>(grid.count[0]) * grid.count[1]);
    for (uint32_t t = 0; t < index_count; t += 3) {
        const double *p[3] = {positions + 3 * size_t(indices[t]), positions + 3 * size_t(indices[t + 1]),
            positions + 3 * size_t(indices[t + 2])};
        int facing = orient(p[0][0], p[0][1], p[1][0], p[1][1], p[2][0], p[2][1]);
        if (facing == 0) continue; // vertical: seen edge-on by every ray
        // Walk the projection counter-clockwise; down-facing triangles are reversed.
        if (facing < 0) std::swap(p[1], p[2]);
        double ux = p[1][0] - p[0][0], uy = p[1][1] - p[0][1], uz = p[1][2] - p[0][2];
        double vx = p[2][0] - p[0][0], vy = p[2][1] - p[0][1], vz = p[2][2] - p[0][2];
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        double length = std::sqrt(nx * nx + ny * ny + nz * nz);
        // After reordering the normal points up; the mesh's own normal is flipped for down-facing faces.
        float local[3] = {float(facing * nx / length), float(facing * ny / length), float(facing * nz / length)};
        float normal[3];
        for (int c = 0; c < 3; ++c) normal[axes[c]] = local[c];
        double xmin = std::min({p[0][0], p[1][0], p[2][0]}), xmax = std::max({p[0][0], p[1][0], p[2][0]});
        double ymin = std::min({p[0][1], p[1][1], p[2][1]}), ymax = std::max({p[0][1], p[1][1], p[2][1]});
        int64_t i0 = std::max<int64_t>(0, int64_t(std::ceil((xmin - grid.origin[0]) / s)) - 1);
        int64_t i1 = std::min<int64_t>(int64_t(grid.count[0]) - 1, int64_t(std::floor((xmax - grid.origin[0]) / s)) + 1);
        int64_t j0 = std::max<int64_t>(0, int64_t(std::ceil((ymin - grid.origin[1]) / s)) - 1);
        int64_t j1 = std::min<int64_t>(int64_t(grid.count[1]) - 1, int64_t(std::floor((ymax - grid.origin[1]) / s)) + 1);
        for (int64_t j = j0; j <= j1; ++j)
            for (int64_t i = i0; i <= i1; ++i) {
                double x = stock.ray_u(uint32_t(i)), y = stock.ray_v(uint32_t(j));
                bool inside = true;
                for (int e = 0; e < 3 && inside; ++e) {
                    const double *a = p[e], *b = p[(e + 1) % 3];
                    int side = orient(a[0], a[1], b[0], b[1], x, y);
                    inside = side > 0 || (side == 0 && owns(b[0] - a[0], b[1] - a[1]));
                }
                if (!inside) continue;
                // Height on the triangle's plane.
                // nz can round to zero only for slivers the exact test barely
                // accepts; any height on the sliver is then as good as another.
                double z = nz > 0 ? p[0][2] - (nx * (x - p[0][0]) + ny * (y - p[0][1])) / nz
                                  : (p[0][2] + p[1][2] + p[2][2]) / 3;
                Hit hit{z, facing > 0 ? -1 : 1, {normal[0], normal[1], normal[2]}};
                hits[size_t(j) * grid.count[0] + size_t(i)].push_back(hit);
            }
    }
    std::vector<Interval> intervals;
    for (uint32_t j = 0; j < grid.count[1]; ++j)
        for (uint32_t i = 0; i < grid.count[0]; ++i) {
            std::vector<Hit> &ray = hits[size_t(j) * grid.count[0] + i];
            if (ray.empty()) continue;
            // Entries before exits at equal height, so touching solids merge.
            std::sort(ray.begin(), ray.end(), [](const Hit &a, const Hit &b) {
                return a.z < b.z || (a.z == b.z && a.winding > b.winding);
            });
            intervals.clear();
            int winding = 0;
            Interval open{};
            for (const Hit &hit : ray) {
                int before = winding;
                winding += hit.winding;
                if (before <= 0 && winding > 0) {
                    open.lo = hit.z;
                    std::copy_n(hit.normal, 3, open.lo_normal);
                    open.lo_source = kSourceStock;
                } else if (before > 0 && winding <= 0) {
                    open.hi = hit.z;
                    std::copy_n(hit.normal, 3, open.hi_normal);
                    open.hi_source = kSourceStock;
                    if (open.hi > open.lo) intervals.push_back(open);
                }
            }
            if (winding != 0) {
                error = "mesh is not closed along the ray at (" + std::to_string(stock.ray_u(i)) + ", " +
                    std::to_string(stock.ray_v(j)) + ")";
                return false;
            }
            stock.write(i, j, intervals);
        }
    return true;
}

} // namespace stockkit
