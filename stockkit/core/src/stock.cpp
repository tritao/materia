#include "stock.hpp"

#include <algorithm>
#include <cmath>

namespace stockkit {

Stock::Stock(const Grid &grid) : grid_(grid) {
    if (grid_.tile == 0) grid_.tile = 16;
    tiles_i_ = (grid_.count[0] + grid_.tile - 1) / grid_.tile;
    tiles_j_ = (grid_.count[1] + grid_.tile - 1) / grid_.tile;
    tiles_.resize(static_cast<size_t>(tiles_i_) * tiles_j_);
    for (uint32_t tj = 0; tj < tiles_j_; ++tj)
        for (uint32_t ti = 0; ti < tiles_i_; ++ti) {
            Tile &tile = tiles_[static_cast<size_t>(tj) * tiles_i_ + ti];
            tile.i0 = ti * grid_.tile;
            tile.j0 = tj * grid_.tile;
            tile.ni = std::min(grid_.tile, grid_.count[0] - tile.i0);
            tile.nj = std::min(grid_.tile, grid_.count[1] - tile.j0);
            size_t rays = static_cast<size_t>(tile.ni) * tile.nj;
            tile.first.assign(rays, 0);
            tile.count.assign(rays, 0);
            tile.capacity.assign(rays, 0);
        }
}

Stock::Tile &Stock::tile_of(uint32_t i, uint32_t j, uint32_t &local) {
    uint32_t ti = i / grid_.tile, tj = j / grid_.tile;
    Tile &tile = tiles_[static_cast<size_t>(tj) * tiles_i_ + ti];
    local = (j - tile.j0) * tile.ni + (i - tile.i0);
    return tile;
}

const Stock::Tile &Stock::tile_of(uint32_t i, uint32_t j, uint32_t &local) const {
    return const_cast<Stock *>(this)->tile_of(i, j, local);
}

void Stock::Tile::grow(uint32_t slots) {
    size_t size = static_cast<size_t>(used) + slots;
    lo.resize(size);
    hi.resize(size);
    lo_normal.resize(size * 3);
    hi_normal.resize(size * 3);
    lo_source.resize(size);
    hi_source.resize(size);
}

/** Packs every ray's intervals to the front, dropping abandoned slots. */
void Stock::Tile::compact() {
    Tile packed;
    packed.first.resize(first.size());
    packed.capacity.resize(first.size());
    uint64_t total = 0;
    for (uint32_t c : count) total += c;
    packed.used = 0;
    packed.grow(static_cast<uint32_t>(total));
    for (size_t r = 0; r < first.size(); ++r) {
        uint32_t from = first[r], n = count[r], to = packed.used;
        std::copy_n(lo.begin() + from, n, packed.lo.begin() + to);
        std::copy_n(hi.begin() + from, n, packed.hi.begin() + to);
        std::copy_n(lo_normal.begin() + 3 * size_t(from), 3 * n, packed.lo_normal.begin() + 3 * size_t(to));
        std::copy_n(hi_normal.begin() + 3 * size_t(from), 3 * n, packed.hi_normal.begin() + 3 * size_t(to));
        std::copy_n(lo_source.begin() + from, n, packed.lo_source.begin() + to);
        std::copy_n(hi_source.begin() + from, n, packed.hi_source.begin() + to);
        packed.first[r] = to;
        packed.capacity[r] = n;
        packed.used += n;
    }
    first.swap(packed.first);
    capacity.swap(packed.capacity);
    lo.swap(packed.lo);
    hi.swap(packed.hi);
    lo_normal.swap(packed.lo_normal);
    hi_normal.swap(packed.hi_normal);
    lo_source.swap(packed.lo_source);
    hi_source.swap(packed.hi_source);
    used = packed.used;
}

void Stock::Tile::refresh_bounds() {
    top = -std::numeric_limits<double>::infinity();
    bottom = std::numeric_limits<double>::infinity();
    for (size_t r = 0; r < count.size(); ++r) {
        if (count[r] == 0) continue;
        bottom = std::min(bottom, lo[first[r]]);
        top = std::max(top, hi[first[r] + count[r] - 1]);
    }
}

uint32_t Stock::count(uint32_t i, uint32_t j) const {
    uint32_t local;
    return tile_of(i, j, local).count[local];
}

void Stock::read(uint32_t i, uint32_t j, std::vector<Interval> &out) const {
    uint32_t local;
    const Tile &tile = tile_of(i, j, local);
    out.resize(tile.count[local]);
    for (uint32_t k = 0; k < tile.count[local]; ++k) {
        uint32_t at = tile.first[local] + k;
        Interval &v = out[k];
        v.lo = tile.lo[at];
        v.hi = tile.hi[at];
        std::copy_n(&tile.lo_normal[3 * size_t(at)], 3, v.lo_normal);
        std::copy_n(&tile.hi_normal[3 * size_t(at)], 3, v.hi_normal);
        v.lo_source = tile.lo_source[at];
        v.hi_source = tile.hi_source[at];
    }
}

void Stock::write_local(Tile &tile, uint32_t local, const Interval *intervals, uint32_t n) {
    tile.live -= tile.count[local];
    tile.live += n;
    if (n > tile.capacity[local]) {
        // Abandon the old slots; compact once half of the tile is abandoned.
        if (tile.used - tile.live > tile.live + 64) tile.compact();
        tile.first[local] = tile.used;
        tile.capacity[local] = n + 1;
        tile.grow(n + 1);
        tile.used += n + 1;
    }
    uint32_t at = tile.first[local];
    for (uint32_t k = 0; k < n; ++k, ++at) {
        tile.lo[at] = intervals[k].lo;
        tile.hi[at] = intervals[k].hi;
        std::copy_n(intervals[k].lo_normal, 3, &tile.lo_normal[3 * size_t(at)]);
        std::copy_n(intervals[k].hi_normal, 3, &tile.hi_normal[3 * size_t(at)]);
        tile.lo_source[at] = intervals[k].lo_source;
        tile.hi_source[at] = intervals[k].hi_source;
    }
    tile.count[local] = n;
}

void Stock::write(uint32_t i, uint32_t j, const std::vector<Interval> &intervals) {
    uint32_t local;
    Tile &tile = tile_of(i, j, local);
    write_local(tile, local, intervals.data(), static_cast<uint32_t>(intervals.size()));
}

void Stock::set_threads(uint32_t threads) { threads_ = threads; }

void Stock::cut(const std::vector<SweptVolume> &sweeps, const uint32_t *sources, double *removed) {
    const uint32_t tiles = tiles_i_ * tiles_j_;
    uint32_t owners = threads_ == 0 ? Pool::hardware() : threads_;
    owners = std::max<uint32_t>(1, std::min<uint32_t>({owners, tiles, uint32_t(sweeps.size())}));
    // Few moves are not worth waking threads for.
    if (sweeps.size() < 4) owners = 1;
    pool_.resize(std::max(pool_.size(), owners));
    owners = std::min(owners, pool_.size());
    workers_.resize(owners);
    for (Worker &worker : workers_) {
        worker.stats = CutStats{};
        worker.removals.clear();
    }
    pool_.run(owners, [&](unsigned owner) {
        Worker &worker = workers_[owner];
        for (size_t k = 0; k < sweeps.size(); ++k)
            cut_owned(sweeps[k], uint32_t(k), sources[k], owner, owners, worker);
    });
    // Per-tile volumes, summed per move in tile order whatever the thread count.
    std::vector<Worker::Removal> all;
    for (Worker &worker : workers_) {
        all.insert(all.end(), worker.removals.begin(), worker.removals.end());
        stats_.rays_tested += worker.stats.rays_tested;
        stats_.rays_changed += worker.stats.rays_changed;
        stats_.tiles_skipped += worker.stats.tiles_skipped;
    }
    std::sort(all.begin(), all.end(), [](const Worker::Removal &a, const Worker::Removal &b) {
        return a.move < b.move || (a.move == b.move && a.tile < b.tile);
    });
    std::fill(removed, removed + sweeps.size(), 0.0);
    for (const Worker::Removal &r : all) removed[r.move] += r.volume;
}

void Stock::cut_owned(const SweptVolume &sweep, uint32_t move, uint32_t source, uint32_t owner, uint32_t owners,
    Worker &worker) {
    CutStats &stats = worker.stats;
    std::vector<Span> &spans = worker.spans;
    std::vector<Interval> &scratch = worker.scratch;
    const Bounds &b = sweep.bounds();
    if (grid_.count[0] == 0 || grid_.count[1] == 0) return;
    const double s = grid_.spacing;
    // Rays strictly inside the xy bounds; the bounds are closed but a ray on
    // them only grazes the tool.
    auto first_index = [&](double lo, double origin, uint32_t n) -> int64_t {
        double k = std::ceil((lo - origin) / s);
        return static_cast<int64_t>(std::max(0.0, std::min(k, double(n))));
    };
    auto last_index = [&](double hi, double origin, uint32_t n) -> int64_t {
        double k = std::floor((hi - origin) / s);
        return static_cast<int64_t>(std::max(-1.0, std::min(k, double(n) - 1)));
    };
    int64_t i0 = first_index(b.min[0], grid_.origin[0], grid_.count[0]);
    int64_t i1 = last_index(b.max[0], grid_.origin[0], grid_.count[0]);
    int64_t j0 = first_index(b.min[1], grid_.origin[1], grid_.count[1]);
    int64_t j1 = last_index(b.max[1], grid_.origin[1], grid_.count[1]);
    if (i0 > i1 || j0 > j1) return;
    const double area = s * s;
    const double sweep_low = sweep.lowest();
    const double sweep_high = b.max[2];
    for (uint32_t tj = uint32_t(j0) / grid_.tile; tj <= uint32_t(j1) / grid_.tile; ++tj)
        for (uint32_t ti = uint32_t(i0) / grid_.tile; ti <= uint32_t(i1) / grid_.tile; ++ti) {
            // Diagonal ownership: moves along x or y alternate between owners.
            if ((ti + tj) % owners != owner) continue;
            const uint32_t index = tj * tiles_i_ + ti;
            Tile &tile = tiles_[index];
            if (!(tile.top > sweep_low) || !(tile.bottom < sweep_high)) {
                ++stats.tiles_skipped;
                continue;
            }
            bool changed = false;
            double tile_removed = 0;
            uint32_t ia = std::max<uint32_t>(uint32_t(i0), tile.i0);
            uint32_t ib = std::min<uint32_t>(uint32_t(i1), tile.i0 + tile.ni - 1);
            uint32_t ja = std::max<uint32_t>(uint32_t(j0), tile.j0);
            uint32_t jb = std::min<uint32_t>(uint32_t(j1), tile.j0 + tile.nj - 1);
            for (uint32_t j = ja; j <= jb; ++j)
                for (uint32_t i = ia; i <= ib; ++i) {
                    uint32_t local = (j - tile.j0) * tile.ni + (i - tile.i0);
                    uint32_t n = tile.count[local];
                    if (n == 0) continue;
                    uint32_t first = tile.first[local];
                    if (!(tile.hi[first + n - 1] > sweep_low) || !(tile.lo[first] < sweep_high)) continue;
                    ++stats.rays_tested;
                    if (!(tile.hi[first + n - 1] > sweep.floor_bound(ray_u(i), ray_v(j)))) continue;
                    sweep.intersect_z(ray_u(i), ray_v(j), spans);
                    if (spans.empty()) continue;
                    // Subtract every span from the ray.
                    scratch.clear();
                    double removed = 0;
                    for (uint32_t k = 0; k < n; ++k) {
                        uint32_t at = first + k;
                        Interval piece;
                        piece.lo = tile.lo[at];
                        piece.hi = tile.hi[at];
                        std::copy_n(&tile.lo_normal[3 * size_t(at)], 3, piece.lo_normal);
                        std::copy_n(&tile.hi_normal[3 * size_t(at)], 3, piece.hi_normal);
                        piece.lo_source = tile.lo_source[at];
                        piece.hi_source = tile.hi_source[at];
                        for (const Span &span : spans) {
                            if (span.hi <= piece.lo) continue;
                            if (span.lo >= piece.hi) break;
                            if (span.lo > piece.lo) {
                                Interval below = piece;
                                below.hi = span.lo;
                                std::copy_n(span.lo_normal, 3, below.hi_normal);
                                below.hi_source = source;
                                scratch.push_back(below);
                            }
                            removed += std::min(span.hi, piece.hi) - std::max(span.lo, piece.lo);
                            if (span.hi >= piece.hi) {
                                piece.lo = piece.hi; // consumed
                                break;
                            }
                            piece.lo = span.hi;
                            std::copy_n(span.hi_normal, 3, piece.lo_normal);
                            piece.lo_source = source;
                        }
                        if (piece.hi > piece.lo) scratch.push_back(piece);
                    }
                    if (removed <= 0) continue;
                    write_local(tile, local, scratch.data(), static_cast<uint32_t>(scratch.size()));
                    tile_removed += removed * area;
                    ++stats.rays_changed;
                    changed = true;
                }
            if (changed) {
                tile.refresh_bounds();
                worker.removals.push_back({move, index, tile_removed});
            }
        }
}


void Stock::pack() {
    for (Tile &tile : tiles_) {
        tile.compact();
        tile.refresh_bounds();
    }
}

uint64_t Stock::interval_count() const {
    uint64_t total = 0;
    for (const Tile &tile : tiles_) total += tile.live;
    return total;
}

double Stock::volume() const {
    double total = 0;
    for (const Tile &tile : tiles_)
        for (size_t r = 0; r < tile.count.size(); ++r)
            for (uint32_t k = 0; k < tile.count[r]; ++k)
                total += tile.hi[tile.first[r] + k] - tile.lo[tile.first[r] + k];
    return total * grid_.spacing * grid_.spacing;
}

uint64_t Stock::bytes() const {
    uint64_t total = sizeof(Stock) + tiles_.capacity() * sizeof(Tile);
    for (const Tile &tile : tiles_) {
        total += (tile.first.capacity() + tile.count.capacity() + tile.capacity.capacity()) * 4;
        total += (tile.lo.capacity() + tile.hi.capacity()) * 8;
        total += (tile.lo_normal.capacity() + tile.hi_normal.capacity()) * 4;
        total += (tile.lo_source.capacity() + tile.hi_source.capacity()) * 4;
    }
    return total;
}

} // namespace stockkit
