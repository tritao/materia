// Convex decomposition through the C ABI (CL-D11): V-HACD pieces of a non-convex L-shaped
// solid, and the measured inflation after which the pieces' union encloses the mesh, checked
// independently at random surface points through the world's distances.
#include "collisionkit.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const char *what) {
    if (!condition) {
        std::printf("FAILED: %s\n", what);
        ++failures;
    }
}

const double kIdentity[7] = {0, 0, 0, 0, 0, 0, 1};

/** A closed prism over polygon `outline` (x, y pairs, counter-clockwise, star-shaped from its first vertex), z in [0, height]. */
void prism(const std::vector<double> &outline, double height, std::vector<double> &vertices,
           std::vector<int32_t> &indices) {
    const int n = int(outline.size() / 2);
    for (int level = 0; level < 2; ++level)
        for (int i = 0; i < n; ++i) {
            vertices.push_back(outline[2 * i]);
            vertices.push_back(outline[2 * i + 1]);
            vertices.push_back(level * height);
        }
    for (int i = 1; i + 1 < n; ++i) {
        indices.insert(indices.end(), {0, i + 1, i});              // bottom, facing -z
        indices.insert(indices.end(), {n, n + i, n + i + 1});      // top, facing +z
    }
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        indices.insert(indices.end(), {i, j, n + j, i, n + j, n + i});
    }
}

struct Result {
    std::vector<std::vector<double>> pieces;
    double measured = 0, spacing = 0, inflation = 0;
};

Result decompose(const std::vector<double> &vertices, const std::vector<int32_t> &indices, uint32_t max_pieces) {
    Result out;
    ck_decomposition_handle handle{0};
    const ck_result status = ck_decompose(vertices.data(), uint32_t(vertices.size()), indices.data(),
                                          uint32_t(indices.size()), max_pieces, 20000, 64, 0.0, &handle);
    expect(status == CK_OK, "decomposed");
    if (status != CK_OK) return out;
    uint32_t count = 0;
    double values[3];
    expect(ck_decomposition_info(handle, &count, values, 3) == CK_OK, "info");
    out.measured = values[0];
    out.spacing = values[1];
    out.inflation = values[2];
    for (uint32_t i = 0; i < count; ++i) {
        std::vector<double> points(192);
        uint32_t size = 0;
        expect(ck_decomposition_piece(handle, i, points.data(), uint32_t(points.size()), &size) == CK_OK, "piece");
        points.resize(size);
        out.pieces.push_back(points);
    }
    ck_decomposition_destroy(handle);
    return out;
}

/** The largest distance from random points on the mesh's surface to the union of the inflated pieces. */
double worst_outside(const std::vector<double> &vertices, const std::vector<int32_t> &indices, const Result &result,
                     double inflation) {
    ck_world_handle world{0};
    expect(ck_world_create(&world) == CK_OK, "world");
    uint32_t body = 0;
    ck_add_body(world, 0, &body);
    std::vector<int32_t> pieces;
    for (const auto &points : result.pieces) {
        uint32_t id = 0;
        expect(ck_add_convex(world, -1, kIdentity, 7, points.data(), uint32_t(points.size()), &id) == CK_OK,
               "piece added");
        expect(ck_set_inflation(world, id, inflation) == CK_OK, "piece inflated");
        pieces.push_back(int32_t(id));
    }
    const double radius = 1e-6;
    uint32_t probe = 0;
    expect(ck_add_shape(world, 0, kIdentity, 7, CK_SHAPE_SPHERE, &radius, 1, &probe) == CK_OK, "probe");
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    double worst = -INFINITY;
    for (size_t t = 0; t + 2 < indices.size(); t += 3)
        for (int sample = 0; sample < 200; ++sample) {
            double u = unit(rng), v = unit(rng);
            if (u + v > 1) {
                u = 1 - u;
                v = 1 - v;
            }
            double pose[7] = {0, 0, 0, 0, 0, 0, 1};
            for (int k = 0; k < 3; ++k) {
                const double a = vertices[3 * indices[t] + k], b = vertices[3 * indices[t + 1] + k],
                             c = vertices[3 * indices[t + 2] + k];
                pose[k] = a + u * (b - a) + v * (c - a);
            }
            ck_set_body_poses(world, 0, pose, 7);
            std::vector<int32_t> pairs;
            for (int32_t piece : pieces) pairs.insert(pairs.end(), {int32_t(probe), piece});
            std::vector<double> rows(10 * pieces.size());
            expect(ck_pair_distances(world, pairs.data(), uint32_t(pairs.size()), rows.data(), uint32_t(rows.size())) ==
                       CK_OK,
                   "probe distances");
            double nearest = INFINITY;
            for (size_t i = 0; i < pieces.size(); ++i) nearest = std::min(nearest, rows[10 * i] + radius);
            worst = std::max(worst, nearest);
        }
    ck_world_destroy(world);
    return worst;
}

} // namespace

int main() {
    // A non-convex L: two arms of 1 x 0.25 around a corner, 0.2 thick.
    std::vector<double> vertices;
    std::vector<int32_t> indices;
    prism({0, 0, 1, 0, 1, 0.25, 0.25, 0.25, 0.25, 1, 0, 1}, 0.2, vertices, indices);
    const Result l = decompose(vertices, indices, 8);
    expect(l.pieces.size() >= 2 && l.pieces.size() <= 8, "the L needs more than one piece");
    for (const auto &piece : l.pieces) expect(piece.size() >= 12 && piece.size() <= 192, "pieces of 4..64 vertices");
    expect(l.spacing > 0 && std::abs(l.inflation - (l.measured + l.spacing)) < 1e-15,
           "inflation is the measured outside distance plus the spacing");
    expect(l.inflation < 0.05, "the inflation is small against the part");
    std::printf("L: %zu pieces, measured %.6f, spacing %.6f, inflation %.6f\n", l.pieces.size(), l.measured,
                l.spacing, l.inflation);
    const double enclosed = worst_outside(vertices, indices, l, l.inflation);
    expect(enclosed <= 1e-9, "after inflation the pieces enclose the mesh");
    std::printf("L: worst surface point outside the inflated pieces %.3g\n", enclosed);
    // Without the inflation the measured excess is real: some samples lie outside (or the pieces were exact).
    const double bare = worst_outside(vertices, indices, l, 0.0);
    expect(bare <= l.measured + l.spacing + 1e-9, "the measurement bounds the bare excess");

    // A box decomposes into one piece that needs almost no inflation.
    std::vector<double> box_vertices;
    std::vector<int32_t> box_indices;
    prism({0, 0, 0.5, 0, 0.5, 0.3, 0, 0.3}, 0.2, box_vertices, box_indices);
    const Result box = decompose(box_vertices, box_indices, 8);
    expect(box.pieces.size() == 1, "a box is one piece");
    expect(box.measured < 0.01, "a box's piece nearly matches it");

    // Arguments.
    ck_decomposition_handle handle{0};
    expect(ck_decompose(vertices.data(), 9, indices.data(), uint32_t(indices.size()), 8, 20000, 64, 0, &handle) ==
               CK_ERROR_INVALID_ARGUMENT,
           "indices past the vertices are refused");
    expect(ck_decompose(vertices.data(), uint32_t(vertices.size()), indices.data(), uint32_t(indices.size()), 8, 20000,
                        65, 0, &handle) == CK_ERROR_INVALID_ARGUMENT,
           "pieces have at most 64 vertices");

    if (failures) {
        std::printf("%d decomposition checks failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("decomposition: all checks passed\n");
    return EXIT_SUCCESS;
}
