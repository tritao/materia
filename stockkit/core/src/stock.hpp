#pragma once

#include "sweep.hpp"

#include <cstdint>
#include <limits>
#include <vector>

namespace stockkit {

constexpr uint32_t kSourceStock = 0xFFFFFFFFu;

struct Grid {
    uint32_t axis = 2;
    double origin[2] = {0, 0};
    double spacing = 0;
    uint32_t count[2] = {0, 0};
    uint32_t tile = 16;
};

/** One material interval with its end data, used when reading or rewriting a ray. */
struct Interval {
    double lo, hi;
    float lo_normal[3], hi_normal[3];
    uint32_t lo_source, hi_source;
};

struct CutStats {
    uint64_t rays_changed = 0;
    uint64_t rays_tested = 0;
    uint64_t tiles_skipped = 0;
};

/**
 * Tiled dexel grid. Each tile stores its intervals field by field (depths,
 * normals and sources in separate arrays); each ray owns a slot range in its
 * tile and is moved to the end of the tile's arrays when it outgrows it. A
 * tile keeps the highest material top and lowest material bottom of its rays,
 * so a move whose sweep lies wholly above or below skips the tile.
 */
class Stock {
public:
    explicit Stock(const Grid &grid);

    const Grid &grid() const { return grid_; }
    double ray_u(uint32_t i) const { return grid_.origin[0] + grid_.spacing * i; }
    double ray_v(uint32_t j) const { return grid_.origin[1] + grid_.spacing * j; }

    void read(uint32_t i, uint32_t j, std::vector<Interval> &out) const;
    uint32_t count(uint32_t i, uint32_t j) const;
    void write(uint32_t i, uint32_t j, const std::vector<Interval> &intervals);
    /** Packs every tile to exactly the slots its rays use, after bulk writes. */
    void pack();

    /** Removes the swept volume and returns how much; endpoints it creates get `source`. */
    double cut(const SweptVolume &sweep, uint32_t source);
    const CutStats &stats() const { return stats_; }

    uint64_t interval_count() const;
    double volume() const;
    uint64_t bytes() const;

private:
    struct Tile {
        uint32_t i0 = 0, j0 = 0, ni = 0, nj = 0;
        std::vector<uint32_t> first, count, capacity;
        std::vector<double> lo, hi;
        std::vector<float> lo_normal, hi_normal; // xyz per interval
        std::vector<uint32_t> lo_source, hi_source;
        uint32_t used = 0;
        uint64_t live = 0;
        double top = -std::numeric_limits<double>::infinity();
        double bottom = std::numeric_limits<double>::infinity();

        void grow(uint32_t slots);
        void compact();
        void refresh_bounds();
    };

    Tile &tile_of(uint32_t i, uint32_t j, uint32_t &local);
    const Tile &tile_of(uint32_t i, uint32_t j, uint32_t &local) const;
    void write_local(Tile &tile, uint32_t local, const Interval *intervals, uint32_t n);

    Grid grid_;
    uint32_t tiles_i_ = 0, tiles_j_ = 0;
    std::vector<Tile> tiles_;
    CutStats stats_;
    std::vector<Span> spans_;
    std::vector<Interval> scratch_;
};

} // namespace stockkit
