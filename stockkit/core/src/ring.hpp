#pragma once
#include "stockkit.h"
#include "pool.hpp"
#include "preview.hpp"
#include <memory>
#include <vector>

namespace stockkit {
using RingCircle = std::vector<sk_ring_interval>;
struct RingLayer { std::vector<RingCircle> circles; };
struct RingSnapshot {
    sk_ring_grid grid;
    std::vector<std::shared_ptr<RingLayer>> layers;
};
class RingStock {
public:
    sk_ring_grid grid;
    std::vector<std::shared_ptr<RingLayer>> layers;
    Pool pool;
    explicit RingStock(const sk_ring_grid &g);
    static bool valid(const sk_ring_grid &g);
    bool same_grid(const sk_ring_grid &g) const;
    bool valid_cut(const sk_ring_cut &c) const;
    void cut(const sk_ring_cut *cuts, uint32_t count);
    const RingCircle &circle(uint32_t r, uint32_t z) const { return layers[r]->circles[z]; }
    sk_ring_info info() const;
    void contour(double max_angle, PreviewMesh &out) const;
};
void ring_compare(const RingStock &a, const RingStock &b, sk_ring_comparison *out);
} // namespace stockkit
