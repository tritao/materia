#pragma once

#include "pool.hpp"
#include "sweep.hpp"
#include "tool.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
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

/** Everything one move sweeps: the cutting solid removes material, the bands only touch it. */
struct MoveSweep {
    MoveSweep(const Tool &tool, const Motion &motion, uint32_t source);

    SweptVolume cutting;
    std::vector<std::pair<uint32_t, SweptVolume>> bands; // zone, sweep
    uint32_t source;
    Bounds reach; // union of all the bounds
};

/** What one move did: material removed, and stock its shank and holder bands overlapped afterwards. */
struct MoveResult {
    double removed = 0;
    double contact[kZoneCount] = {0, 0, 0};
};

/** One ray of a stock compared with a target; see sk_ray_comparison. */
struct RayComparison {
    double leftover = 0, gouge = 0, largest_leftover = 0, largest_gouge = 0;
    uint32_t gouge_source;
};

/**
 * The parts of `a` outside `b`, calling `stretch(lo, hi, below, above)` for
 * each, where `below` and `above` are the `b` intervals touching it from
 * below and above (null when the stretch ends at an `a` endpoint instead).
 */
template <typename F>
inline void difference(const std::vector<Interval> &a, const std::vector<Interval> &b, F &&stretch) {
    size_t k = 0;
    for (const Interval &piece : a) {
        double lo = piece.lo;
        const Interval *below = nullptr;
        while (k < b.size() && b[k].hi <= lo) ++k;
        size_t m = k;
        while (lo < piece.hi) {
            if (m < b.size() && b[m].lo <= lo) {
                // Inside b: skip past it.
                below = &b[m];
                lo = std::max(lo, b[m].hi);
                ++m;
                continue;
            }
            double hi = piece.hi;
            const Interval *above = nullptr;
            if (m < b.size() && b[m].lo < piece.hi) {
                hi = b[m].lo;
                above = &b[m];
            }
            if (hi > lo) stretch(lo, hi, below && below->hi == lo ? below : nullptr, above);
            lo = hi;
        }
    }
}

/** Compares two sorted interval lists: `stock` against `target`. */
RayComparison compare_ray(const std::vector<Interval> &stock, const std::vector<Interval> &target);

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
    /** Replaces a ray's intervals; call `pack` after a series of writes. */
    void write(uint32_t i, uint32_t j, const std::vector<Interval> &intervals);
    /** Packs every tile to exactly the slots its rays use and refreshes its material bounds. */
    void pack();

    /**
     * Removes each move's cutting sweep in order, giving the endpoints it
     * creates the move's source, and measures how much of the stock left
     * after each cut its shank and holder bands overlap.
     *
     * Tiles are split among threads by a fixed ownership pattern; each thread
     * walks every move in order over the tiles it owns, so a ray only ever
     * sees its moves in program order. Results are summed per tile, then over
     * tiles in tile order, so they are bit-identical for any thread count.
     *
     * Contact is measured after the move's own cut. A band that reaches stock
     * which the flutes remove later in the same move (only possible while the
     * tool climbs) is not seen.
     */
    void cut(const std::vector<MoveSweep> &moves, MoveResult *results);
    const CutStats &stats() const { return stats_; }

    /** Threads used by `cut`; 0 means one per hardware thread. */
    void set_threads(uint32_t threads);
    uint32_t threads() const { return threads_; }

    /**
     * Tiles, row by row, and their revisions: a tile's revision changes
     * whenever its rays change (a cut or a restore), so a preview remeshes
     * only tiles whose revision it has not seen.
     */
    uint32_t tiles_across() const { return tiles_i_; }
    uint32_t tiles_down() const { return tiles_j_; }
    const std::vector<uint64_t> &revisions() const { return revisions_; }

    /** An immutable copy of the stock that shares tiles until either side changes them. */
    struct Snapshot;
    std::unique_ptr<Snapshot> snapshot() const;
    /** Returns the stock to `snapshot`, which must come from a stock with the same grid. */
    bool restore(const Snapshot &snapshot);

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

    /** Per-thread scratch and results for one `cut` call. */
    struct Worker {
        std::vector<Span> spans;
        std::vector<Interval> scratch;
        CutStats stats;
        struct Record { uint32_t move, tile; MoveResult result; };
        std::vector<Record> records;
    };
    struct RayRange { int64_t i0, i1, j0, j1; };

    bool ray_range(const Bounds &bounds, RayRange &out) const;
    /** Cuts one move from the tiles `owner` owns among `owners`. */
    void cut_owned(const MoveSweep &move, uint32_t index, uint32_t owner, uint32_t owners, Worker &worker);
    bool cut_tile(uint32_t index, const SweptVolume &sweep, const RayRange &range, uint32_t source, Worker &worker,
        double &removed_volume);
    double touch_tile(const Tile &tile, const SweptVolume &sweep, const RayRange &range, Worker &worker) const;
    /** The tile for writing: a private copy if a snapshot still shares it; bumps its revision. */
    Tile &mutable_tile(uint32_t index);
    uint32_t tile_index(uint32_t i, uint32_t j, uint32_t &local) const;
    void write_local(Tile &tile, uint32_t local, const Interval *intervals, uint32_t n);

    Grid grid_;
    uint32_t tiles_i_ = 0, tiles_j_ = 0;
    std::vector<std::shared_ptr<Tile>> tiles_;
    std::vector<uint64_t> revisions_;

public:
    struct Snapshot {
        Grid grid;
        std::vector<std::shared_ptr<Tile>> tiles;
    };

private:
    CutStats stats_;
    uint32_t threads_ = 0;
    Pool pool_;
    std::vector<Worker> workers_;
};

} // namespace stockkit
