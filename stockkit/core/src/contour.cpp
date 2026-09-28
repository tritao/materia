#include "contour.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>

namespace stockkit {

namespace {

/** A surface crossing on a lattice edge. */
struct Crossing {
    double point[3];
    double normal[3];
    uint32_t source;
    bool matched;
};

/**
 * Eigen decomposition of a symmetric 3x3 matrix by cyclic Jacobi rotations:
 * `a` becomes diagonal (the eigenvalues) and the columns of `v` the
 * eigenvectors.
 */
void jacobi(double a[3][3], double v[3][3]) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) v[r][c] = r == c ? 1 : 0;
    for (int sweep = 0; sweep < 12; ++sweep) {
        double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
        if (off == 0) return;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (a[p][q] == 0) continue;
                double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                double t = (theta >= 0 ? 1 : -1) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
                double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; ++k) {
                    double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
    }
}

/** Eigenvalues below this fraction of the largest leave their direction to the mass point. */
constexpr double kFeatureThreshold = 0.05;

class Contour {
public:
    Contour(const Stock &stock, PreviewMesh &mesh, ContourStats &stats)
        : lattice_(stock.lattice()), mesh_(mesh), stats_(stats) {
        for (uint32_t axis = 0; axis < 3; ++axis) {
            grids_[axis] = stock.grid(axis);
            count_[axis] = int32_t(lattice_.count[axis]);
        }
        s_ = lattice_.spacing;
    }

    void build(int32_t i0, int32_t i1, int32_t j0, int32_t j1) {
        // Owned edges start at nodes [ea, eb) x [fa, fb); their cells reach one node further each way.
        const int32_t ea = i0 == 0 ? -1 : i0 + 1, eb = std::min(i1 + 1, count_[0]);
        const int32_t fa = j0 == 0 ? -1 : j0 + 1, fb = std::min(j1 + 1, count_[1]);
        if (ea >= eb || fa >= fb) return;
        window_i_ = ea - 1;
        window_j_ = fa - 1;
        window_w_ = eb - ea + 2;
        window_h_ = fb - fa + 2;
        columns_.assign(size_t(window_w_) * window_h_, {});
        loaded_.assign(columns_.size(), 0);
        std::vector<int32_t> xor_bounds;
        for (int32_t j = fa; j < fb; ++j)
            for (int32_t i = ea; i < eb; ++i) {
                const std::vector<int32_t> &here = column(i, j);
                for (size_t r = 0; r < here.size(); r += 2) {
                    edge(2, i, j, here[r] - 1, false);
                    edge(2, i, j, here[r + 1] - 1, true);
                }
                for (uint32_t axis = 0; axis < 2; ++axis) {
                    const std::vector<int32_t> &next = axis == 0 ? column(i + 1, j) : column(i, j + 1);
                    symmetric_difference(here, next, xor_bounds);
                    for (size_t r = 0; r < xor_bounds.size(); r += 2)
                        for (int32_t k = xor_bounds[r]; k < xor_bounds[r + 1]; ++k)
                            edge(axis, i, j, k, inside(here, k));
                }
            }
    }

private:
    double node(uint32_t axis, int32_t n) const { return lattice_.origin[axis] + s_ * n; }

    /** The smallest node index k in [0, nz] with z_k >= t. */
    int32_t first_node_at(double t) const {
        const int32_t n = count_[2];
        double guess = std::ceil((t - lattice_.origin[2]) / s_);
        int32_t k = int32_t(std::max(0.0, std::min(guess, double(n))));
        while (k > 0 && node(2, k - 1) >= t) --k;
        while (k < n && node(2, k) < t) ++k;
        return k;
    }

    /** The nodes inside material along column (i, j), as sorted half-open ranges [ka, kb). */
    const std::vector<int32_t> &column(int32_t i, int32_t j) {
        static const std::vector<int32_t> empty;
        if (i < 0 || j < 0 || i >= count_[0] || j >= count_[1]) return empty;
        const size_t slot = size_t(j - window_j_) * window_w_ + size_t(i - window_i_);
        std::vector<int32_t> &ranges = columns_[slot];
        if (loaded_[slot]) return ranges;
        loaded_[slot] = 1;
        // A node is inside an interval when lo <= z < hi.
        const RayView ray = grids_[2]->view(uint32_t(i), uint32_t(j));
        for (uint32_t m = 0; m < ray.count; ++m) {
            int32_t ka = first_node_at(ray.lo[m]), kb = first_node_at(ray.hi[m]);
            if (ka >= kb) continue;
            if (!ranges.empty() && ranges.back() == ka) ranges.back() = kb;
            else ranges.insert(ranges.end(), {ka, kb});
        }
        return ranges;
    }

    static bool inside(const std::vector<int32_t> &ranges, int32_t k) {
        for (size_t r = 0; r < ranges.size(); r += 2) {
            if (k < ranges[r]) return false;
            if (k < ranges[r + 1]) return true;
        }
        return false;
    }

    bool inside(int32_t i, int32_t j, int32_t k) { return inside(column(i, j), k); }

    /** The node ranges inside exactly one of `a` and `b`. */
    static void symmetric_difference(const std::vector<int32_t> &a, const std::vector<int32_t> &b,
        std::vector<int32_t> &out) {
        out.clear();
        size_t p = 0, q = 0;
        while (p < a.size() || q < b.size()) {
            int32_t next;
            if (q == b.size() || (p < a.size() && a[p] < b[q])) next = a[p++];
            else if (p == a.size() || b[q] < a[p]) next = b[q++];
            else {
                // Both toggle here: their difference does not.
                ++p;
                ++q;
                continue;
            }
            if (!out.empty() && out.back() == next) out.pop_back();
            else out.push_back(next);
        }
    }

    /**
     * The crossing on the edge along `axis` from node (i, j, k) to the next
     * node, where the first node is inside when `first_inside`.
     */
    Crossing crossing(uint32_t axis, int32_t i, int32_t j, int32_t k, bool first_inside) const {
        const int32_t at[3] = {i, j, k};
        Crossing out;
        for (uint32_t c = 0; c < 3; ++c) out.point[c] = node(c, at[c]);
        const double t0 = node(axis, at[axis]), t1 = node(axis, at[axis] + 1);
        const DexelGrid &grid = *grids_[axis];
        const int32_t u = at[grid.grid().u_axis()], v = at[grid.grid().v_axis()];
        const double reach = s_ / 2;
        int64_t best = -1;
        double best_distance = reach;
        const double *ends = nullptr;
        RayView ray;
        if (u >= 0 && v >= 0 && uint32_t(u) < grid.grid().count[0] && uint32_t(v) < grid.grid().count[1]) {
            ray = grid.view(uint32_t(u), uint32_t(v));
            // Leaving material at an interval's hi, entering it at a lo.
            ends = first_inside ? ray.hi : ray.lo;
            const double *first = std::lower_bound(ends, ends + ray.count, t0 - reach);
            for (const double *e = first; e < ends + ray.count && *e <= t1 + reach; ++e) {
                double d = *e <= t0 ? t0 - *e : *e > t1 ? *e - t1 : 0;
                if (d < best_distance || (best < 0 && d <= best_distance)) {
                    best_distance = d;
                    best = e - ends;
                    if (d == 0) break;
                }
            }
        }
        if (best < 0) {
            out.point[axis] = (t0 + t1) / 2;
            out.normal[0] = out.normal[1] = out.normal[2] = 0;
            out.normal[axis] = first_inside ? 1 : -1;
            out.source = kSourceStock;
            out.matched = false;
            return out;
        }
        out.point[axis] = std::min(t1, std::max(t0, ends[best]));
        const float *normal = first_inside ? &ray.hi_normal[3 * best] : &ray.lo_normal[3 * best];
        double length = std::sqrt(double(normal[0]) * normal[0] + double(normal[1]) * normal[1] +
            double(normal[2]) * normal[2]);
        for (uint32_t c = 0; c < 3; ++c)
            out.normal[c] = length > 0 ? normal[c] / length : c == axis ? (first_inside ? 1 : -1) : 0;
        out.source = first_inside ? ray.hi_source[best] : ray.lo_source[best];
        out.matched = true;
        return out;
    }

    /** The vertex of cell (i, j, k), whose corners are nodes i..i+1, j..j+1, k..k+1. */
    const std::array<double, 3> &vertex(int32_t i, int32_t j, int32_t k) {
        const uint64_t key = (uint64_t(i + 1) * uint64_t(count_[1] + 1) + uint64_t(j + 1)) *
            uint64_t(count_[2] + 1) + uint64_t(k + 1);
        auto found = vertices_.find(key);
        if (found != vertices_.end()) return found->second;
        ++stats_.cells;
        bool corner[2][2][2];
        for (int dx = 0; dx < 2; ++dx)
            for (int dy = 0; dy < 2; ++dy) {
                const std::vector<int32_t> &ranges = column(i + dx, j + dy);
                for (int dz = 0; dz < 2; ++dz) corner[dx][dy][dz] = inside(ranges, k + dz);
            }
        crossings_.clear();
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b) {
                if (corner[0][a][b] != corner[1][a][b]) crossings_.push_back(crossing(0, i, j + a, k + b, corner[0][a][b]));
                if (corner[a][0][b] != corner[a][1][b]) crossings_.push_back(crossing(1, i + a, j, k + b, corner[a][0][b]));
                if (corner[a][b][0] != corner[a][b][1]) crossings_.push_back(crossing(2, i + a, j + b, k, corner[a][b][0]));
            }
        const double low[3] = {node(0, i), node(1, j), node(2, k)};
        const double high[3] = {node(0, i + 1), node(1, j + 1), node(2, k + 1)};
        std::array<double, 3> result;
        if (crossings_.empty()) {
            for (int c = 0; c < 3; ++c) result[c] = (low[c] + high[c]) / 2;
            return vertices_.emplace(key, result).first->second;
        }
        // Minimise the squared distances to the crossings' planes about their mass point.
        double mass[3] = {0, 0, 0};
        for (const Crossing &x : crossings_)
            for (int c = 0; c < 3; ++c) mass[c] += x.point[c];
        for (int c = 0; c < 3; ++c) mass[c] /= double(crossings_.size());
        double a[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, rhs[3] = {0, 0, 0};
        for (const Crossing &x : crossings_) {
            const double *n = x.normal;
            double d = n[0] * (x.point[0] - mass[0]) + n[1] * (x.point[1] - mass[1]) + n[2] * (x.point[2] - mass[2]);
            for (int r = 0; r < 3; ++r) {
                rhs[r] += n[r] * d;
                for (int c = 0; c < 3; ++c) a[r][c] += n[r] * n[c];
            }
        }
        double v[3][3];
        jacobi(a, v);
        const double largest = std::max({a[0][0], a[1][1], a[2][2]});
        for (int c = 0; c < 3; ++c) result[c] = mass[c];
        for (int e = 0; e < 3; ++e) {
            if (!(a[e][e] > kFeatureThreshold * largest)) continue;
            double along = (v[0][e] * rhs[0] + v[1][e] * rhs[1] + v[2][e] * rhs[2]) / a[e][e];
            for (int c = 0; c < 3; ++c) result[c] += along * v[c][e];
        }
        for (int c = 0; c < 3; ++c) result[c] = std::min(high[c], std::max(low[c], result[c]));
        return vertices_.emplace(key, result).first->second;
    }

    /** Emits the quad of the edge along `axis` from node (i, j, k), if its nodes differ. */
    void edge(uint32_t axis, int32_t i, int32_t j, int32_t k, bool first_inside) {
        const Crossing x = crossing(axis, i, j, k, first_inside);
        if (!x.matched) ++stats_.unmatched;
        // The four cells around the edge, counter-clockwise seen from +axis.
        const uint32_t b = (axis + 1) % 3, c = (axis + 2) % 3;
        static const int kAround[4][2] = {{-1, -1}, {0, -1}, {0, 0}, {-1, 0}};
        std::array<double, 3> corners[4];
        for (int q = 0; q < 4; ++q) {
            int32_t cell[3] = {i, j, k};
            cell[b] += kAround[q][0];
            cell[c] += kAround[q][1];
            corners[q] = vertex(cell[0], cell[1], cell[2]);
        }
        // Face +axis when the material ends on this edge, -axis when it starts.
        if (!first_inside) std::swap(corners[1], corners[3]);
        const int32_t inner[3] = {first_inside ? i : i + (axis == 0), first_inside ? j : j + (axis == 1), 0};
        const uint32_t ray = uint32_t(inner[1]) * lattice_.count[0] + uint32_t(inner[0]);
        // Split along the diagonal whose triangles best follow the crossing's normal.
        auto facing = [&](int p, int q, int r) {
            double e1[3], e2[3];
            for (int m = 0; m < 3; ++m) {
                e1[m] = corners[q][m] - corners[p][m];
                e2[m] = corners[r][m] - corners[p][m];
            }
            double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
            double length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            return length > 0 ? (n[0] * x.normal[0] + n[1] * x.normal[1] + n[2] * x.normal[2]) / length : 0.0;
        };
        const bool other = std::min(facing(0, 1, 3), facing(1, 2, 3)) > std::min(facing(0, 1, 2), facing(0, 2, 3));
        const uint32_t base = mesh_.vertex_count();
        const float normal[3] = {float(x.normal[0]), float(x.normal[1]), float(x.normal[2])};
        for (const auto &p : corners) {
            mesh_.positions.insert(mesh_.positions.end(), {float(p[0]), float(p[1]), float(p[2])});
            mesh_.normals.insert(mesh_.normals.end(), normal, normal + 3);
            mesh_.vertex_sources.push_back(x.source);
            mesh_.vertex_rays.push_back(ray);
        }
        if (other) mesh_.indices.insert(mesh_.indices.end(), {base, base + 1, base + 3, base + 1, base + 2, base + 3});
        else mesh_.indices.insert(mesh_.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        mesh_.triangle_sources.insert(mesh_.triangle_sources.end(), {x.source, x.source});
    }

    const Lattice &lattice_;
    PreviewMesh &mesh_;
    ContourStats &stats_;
    const DexelGrid *grids_[3];
    int32_t count_[3];
    double s_;
    int32_t window_i_ = 0, window_j_ = 0, window_w_ = 0, window_h_ = 0;
    std::vector<std::vector<int32_t>> columns_;
    std::vector<uint8_t> loaded_;
    std::unordered_map<uint64_t, std::array<double, 3>> vertices_;
    std::vector<Crossing> crossings_;
};

} // namespace

void build_contour(const Stock &stock, uint32_t tile_x, uint32_t tile_y, uint32_t tiles_x, uint32_t tiles_y,
    PreviewMesh &out, ContourStats *stats) {
    out = PreviewMesh{};
    ContourStats local;
    const Lattice &lattice = stock.lattice();
    const uint32_t tile = lattice.tile;
    const uint32_t i0 = std::min(lattice.count[0], tile_x * tile), i1 = std::min(lattice.count[0], (tile_x + tiles_x) * tile);
    const uint32_t j0 = std::min(lattice.count[1], tile_y * tile), j1 = std::min(lattice.count[1], (tile_y + tiles_y) * tile);
    if (i0 < i1 && j0 < j1) {
        Contour contour(stock, out, local);
        contour.build(int32_t(i0), int32_t(i1), int32_t(j0), int32_t(j1));
    }
    if (stats) *stats = local;
}

} // namespace stockkit
