#include "decompose.hpp"

#define ENABLE_VHACD_IMPLEMENTATION 1
#include <VHACD.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>

namespace ck {

namespace {

using Vec = std::array<double, 3>;

Vec sub(const Vec &a, const Vec &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
Vec add(const Vec &a, const Vec &b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec scale(const Vec &a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double dot(const Vec &a, const Vec &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vec cross(const Vec &a, const Vec &b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double norm(const Vec &a) { return std::sqrt(dot(a, a)); }

/** The closest point to `p` on triangle (a, b, c) (Ericson, Real-Time Collision Detection, 5.1.5). */
Vec closest_on_triangle(const Vec &p, const Vec &a, const Vec &b, const Vec &c) {
    const Vec ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const Vec bp = sub(p, b);
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return add(a, scale(ab, d1 / (d1 - d3)));
    const Vec cp = sub(p, c);
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return add(a, scale(ac, d2 / (d2 - d6)));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return add(b, scale(sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    const double denom = 1.0 / (va + vb + vc);
    return add(a, add(scale(ab, vb * denom), scale(ac, vc * denom)));
}

/** A convex piece: its triangles, outward planes and bounds. */
struct Piece {
    std::vector<Vec> points;
    std::vector<std::array<uint32_t, 3>> triangles;
    std::vector<Vec> normals;
    std::vector<double> offsets;
    Vec low{}, high{};
    double tolerance = 0.0;

    /** Distance from `p` to the piece: 0 inside, else to its surface. */
    double distance(const Vec &p) const {
        bool inside = true;
        for (size_t i = 0; i < normals.size() && inside; ++i)
            if (dot(normals[i], p) - offsets[i] > tolerance) inside = false;
        if (inside) return 0.0;
        double best = std::numeric_limits<double>::infinity();
        for (const auto &t : triangles) {
            const Vec q = closest_on_triangle(p, points[t[0]], points[t[1]], points[t[2]]);
            best = std::min(best, norm(sub(p, q)));
        }
        return best;
    }

    /** A lower bound on `distance` from the piece's bounds. */
    double bound(const Vec &p) const {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) {
            const double gap = std::max({low[k] - p[k], p[k] - high[k], 0.0});
            sum += gap * gap;
        }
        return std::sqrt(sum);
    }
};

Piece make_piece(const std::vector<double> &points, const std::vector<uint32_t> &triangles) {
    Piece piece;
    Vec centre{0, 0, 0};
    const size_t count = points.size() / 3;
    piece.low = {INFINITY, INFINITY, INFINITY};
    piece.high = {-INFINITY, -INFINITY, -INFINITY};
    for (size_t i = 0; i < count; ++i) {
        const Vec p{points[3 * i], points[3 * i + 1], points[3 * i + 2]};
        piece.points.push_back(p);
        centre = add(centre, scale(p, 1.0 / double(count)));
        for (int k = 0; k < 3; ++k) {
            piece.low[k] = std::min(piece.low[k], p[k]);
            piece.high[k] = std::max(piece.high[k], p[k]);
        }
    }
    const double size = norm(sub(piece.high, piece.low));
    piece.tolerance = 1e-12 * std::max(size, 1.0);
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        const std::array<uint32_t, 3> t{triangles[i], triangles[i + 1], triangles[i + 2]};
        piece.triangles.push_back(t);
        Vec n = cross(sub(piece.points[t[1]], piece.points[t[0]]), sub(piece.points[t[2]], piece.points[t[0]]));
        const double length = norm(n);
        if (!(length > 1e-14 * std::max(size * size, 1e-300))) continue;
        n = scale(n, 1.0 / length);
        double offset = dot(n, piece.points[t[0]]);
        // Outward: away from the centre.
        if (dot(n, centre) > offset) {
            n = scale(n, -1.0);
            offset = -offset;
        }
        piece.normals.push_back(n);
        piece.offsets.push_back(offset);
    }
    return piece;
}

} // namespace

double outside_distance(const double *vertices, uint32_t vertex_count, const int32_t *indices, uint32_t index_count,
                        const std::vector<std::vector<double>> &piece_points,
                        const std::vector<std::vector<uint32_t>> &piece_triangles, double spacing) {
    std::vector<Piece> pieces;
    for (size_t i = 0; i < piece_points.size(); ++i) pieces.push_back(make_piece(piece_points[i], piece_triangles[i]));
    auto vertex = [&](int32_t index) {
        return Vec{vertices[3 * index], vertices[3 * index + 1], vertices[3 * index + 2]};
    };
    (void)vertex_count;
    double worst = 0.0;
    for (uint32_t t = 0; t + 2 < index_count; t += 3) {
        const Vec a = vertex(indices[t]), b = vertex(indices[t + 1]), c = vertex(indices[t + 2]);
        const double longest = std::max({norm(sub(b, a)), norm(sub(c, a)), norm(sub(c, b))});
        const int n = std::max(1, int(std::ceil(longest / spacing)));
        for (int i = 0; i <= n; ++i)
            for (int j = 0; i + j <= n; ++j) {
                const Vec p = add(a, add(scale(sub(b, a), double(i) / n), scale(sub(c, a), double(j) / n)));
                double nearest = std::numeric_limits<double>::infinity();
                for (const Piece &piece : pieces) {
                    if (piece.bound(p) >= nearest) continue;
                    nearest = std::min(nearest, piece.distance(p));
                    if (nearest == 0.0) break;
                }
                worst = std::max(worst, nearest);
            }
    }
    return worst;
}

bool decompose(const double *vertices, uint32_t vertex_count, const int32_t *indices, uint32_t index_count,
               const DecomposeOptions &options, Decomposition &out) {
    std::vector<uint32_t> triangles(indices, indices + index_count);
    VHACD::IVHACD::Parameters parameters;
    parameters.m_maxConvexHulls = options.max_pieces;
    parameters.m_resolution = options.resolution;
    parameters.m_maxNumVerticesPerCH = options.max_piece_vertices;
    // One thread: the browser host has none to spare, and the caller decides when to decompose.
    parameters.m_asyncACD = false;
    std::unique_ptr<VHACD::IVHACD, void (*)(VHACD::IVHACD *)> vhacd(VHACD::CreateVHACD(),
                                                                   [](VHACD::IVHACD *v) { v->Release(); });
    if (!vhacd->Compute(vertices, vertex_count / 3, triangles.data(), index_count / 3, parameters)) return false;
    const uint32_t count = vhacd->GetNConvexHulls();
    if (count == 0) return false;
    std::vector<std::vector<uint32_t>> piece_triangles;
    out.pieces.clear();
    for (uint32_t i = 0; i < count; ++i) {
        VHACD::IVHACD::ConvexHull hull;
        if (!vhacd->GetConvexHull(i, hull) || hull.m_points.size() < 4) continue;
        std::vector<double> points;
        for (const auto &p : hull.m_points) {
            points.push_back(p.mX);
            points.push_back(p.mY);
            points.push_back(p.mZ);
        }
        std::vector<uint32_t> tris;
        for (const auto &t : hull.m_triangles) {
            tris.push_back(t.mI0);
            tris.push_back(t.mI1);
            tris.push_back(t.mI2);
        }
        out.pieces.push_back(std::move(points));
        piece_triangles.push_back(std::move(tris));
    }
    if (out.pieces.empty()) return false;
    double spacing = options.sample_spacing;
    if (!(spacing > 0.0)) {
        Vec low{INFINITY, INFINITY, INFINITY}, high{-INFINITY, -INFINITY, -INFINITY};
        for (uint32_t i = 0; i + 2 < vertex_count; i += 3)
            for (int k = 0; k < 3; ++k) {
                low[k] = std::min(low[k], vertices[i + k]);
                high[k] = std::max(high[k], vertices[i + k]);
            }
        spacing = 0.005 * norm(sub(high, low));
    }
    out.spacing = spacing;
    out.measured = outside_distance(vertices, vertex_count, indices, index_count, out.pieces, piece_triangles, spacing);
    out.inflation = out.measured + spacing;
    return true;
}

} // namespace ck
