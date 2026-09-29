#pragma once

#include "stock.hpp"

#include <string>

namespace stockkit {

/**
 * Fills a dexel grid from a closed, outward-oriented triangle mesh. Each ray
 * is tested against each triangle's projection with exact orientation
 * predicates and a tie rule under which, of two triangles sharing an edge or
 * vertex, exactly one claims a ray through it; material is where the
 * winding number along the ray is positive. Grids along X and Y cast the
 * mesh with its coordinates permuted so that the ray runs along the third.
 */
bool cast_mesh(DexelGrid &grid, const double *positions, uint32_t position_count,
    const uint32_t *indices, uint32_t index_count, std::string &error);

} // namespace stockkit
