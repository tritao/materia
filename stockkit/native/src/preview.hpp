#pragma once

#include "stock.hpp"

#include <cstdint>
#include <vector>

namespace stockkit {

/**
 * A display mesh of part of a Z-grid stock. Each ray stands for a square
 * column of material spacing wide, centred on the ray: every interval gets a
 * top face (and optionally a bottom face) at its exact depth with its stored
 * normal and source, and walls stand where neighbouring columns differ. The
 * mesh is closed when bottoms are included, and its volume is the stock's.
 *
 * Quads never share vertices, so each vertex carries its quad's source and
 * ray and can be coloured per quad.
 */
struct PreviewMesh {
    std::vector<float> positions; // xyz per vertex
    std::vector<float> normals;   // xyz per vertex
    std::vector<uint32_t> indices;
    std::vector<uint32_t> triangle_sources;
    std::vector<uint32_t> vertex_sources;
    /**
     * The ray each vertex's quad was made from: its grid's axis and its index
     * (j * count_u + i) in that grid. Column meshes use Z rays only. Unset
     * when tops were merged.
     */
    std::vector<uint32_t> vertex_rays;
    std::vector<uint8_t> vertex_axes;
    /**
     * Contoured meshes: each quad's surface coordinate along its ray, and
     * whether the stock's material lies below it (the ray leaves material
     * there) or above.
     */
    std::vector<double> quad_depths;
    std::vector<uint8_t> quad_exits;
    std::vector<uint32_t> colors; // RGBA8 per vertex, once coloured
    bool merged = false;

    uint32_t vertex_count() const { return uint32_t(positions.size() / 3); }
    uint32_t triangle_count() const { return uint32_t(indices.size() / 3); }
};

struct PreviewOptions {
    bool bottoms = false;
    /** Merge neighbouring top and bottom faces along rows when depth, normal and source agree. */
    bool merge = true;
};

/**
 * Meshes the tiles [tile_x, tile_x + tiles_x) x [tile_y, tile_y + tiles_y).
 * A wall between two columns belongs to the mesh holding the column with the
 * lower index, so a mesh also reads the columns just past its +x and +y
 * edges, and must be rebuilt when those tiles change.
 */
void build_preview(const DexelGrid &stock, uint32_t tile_x, uint32_t tile_y, uint32_t tiles_x, uint32_t tiles_y,
    const PreviewOptions &options, PreviewMesh &out);

} // namespace stockkit
