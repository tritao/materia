#pragma once

#include "preview.hpp"
#include "stock.hpp"

#include <cstdint>

namespace stockkit {

struct ContourStats {
    /** Cells that got a vertex. */
    uint64_t cells = 0;
    /**
     * Sign-changing edges whose ray had no surface of the right kind near
     * the edge, so the crossing fell back to the edge's middle with the
     * axis as normal: where the lattice cuts through the stock, or where the
     * grids disagree about a node.
     */
    uint64_t unmatched = 0;
};

/**
 * A closed display mesh of a tri-dexel stock by dual contouring. Lattice
 * nodes are inside or outside by the Z rays through them (nodes past the
 * lattice are outside). Each edge between nodes of opposite sign lies on a
 * ray of the grid along it, whose nearest interval end of the right kind
 * gives the crossing point, normal and source exactly. Each cell with a
 * crossing gets one vertex, the least-squares meeting point of its
 * crossings' planes (dropping directions the normals barely constrain, and
 * clamped to the cell), which keeps flat faces flat and edges and corners
 * sharp. Each crossing edge becomes a quad joining the vertices of the four
 * cells around it, with the crossing's normal and source.
 *
 * Meshes the Z tiles [tile_x, tile_x + tiles_x) x [tile_y, tile_y + tiles_y).
 * Chunks meshed separately join exactly: the chunk whose rays start at i0
 * owns the edges whose first node lies in (i0, i1] (the first chunk from
 * node -1), and likewise along y, so a mesh reads rays [i0, i1 + 1] and
 * must be rebuilt when the tiles holding them change. Features thinner than
 * a cell can be lost, and a vertex can be shared by surfaces that only meet
 * inside its cell (the mesh is closed but not always manifold there).
 *
 * The stock must have all three grids. Quads never share vertices.
 */
void build_contour(const Stock &stock, uint32_t tile_x, uint32_t tile_y, uint32_t tiles_x, uint32_t tiles_y,
    PreviewMesh &out, ContourStats *stats = nullptr);

} // namespace stockkit
