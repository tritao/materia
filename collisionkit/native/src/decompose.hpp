#pragma once

#include <cstdint>
#include <vector>

namespace ck {

/** Options for `decompose` (`collisionkit.h`). */
struct DecomposeOptions {
    uint32_t max_pieces = 16;
    uint32_t resolution = 100000;
    uint32_t max_piece_vertices = 64;
    /** Spacing of the enclosure samples on the mesh (0: 0.5 % of the mesh's bounding diagonal). */
    double sample_spacing = 0.0;
};

/** Convex pieces and the inflation that makes their union enclose the mesh (CL-D11). */
struct Decomposition {
    /** x, y, z per vertex, per piece. */
    std::vector<std::vector<double>> pieces;
    /** The farthest an enclosure sample lies outside every piece. */
    double measured = 0.0;
    /** The sample spacing used, which bounds how far any mesh point is from a sample. */
    double spacing = 0.0;
    /** measured + spacing: inflating every piece by it encloses the mesh. */
    double inflation = 0.0;
};

/** False when V-HACD returns no pieces. */
bool decompose(const double *vertices, uint32_t vertex_count, const int32_t *indices, uint32_t index_count,
               const DecomposeOptions &options, Decomposition &out);

/**
 * How far points sampled over every triangle of the mesh (no two neighbours
 * more than `spacing` apart) lie outside the union of convex `pieces`, each
 * given by its vertices and triangles: the largest distance from a sample to
 * the nearest piece (0 inside one).
 */
double outside_distance(const double *vertices, uint32_t vertex_count, const int32_t *indices, uint32_t index_count,
                        const std::vector<std::vector<double>> &piece_points,
                        const std::vector<std::vector<uint32_t>> &piece_triangles, double spacing);

} // namespace ck
