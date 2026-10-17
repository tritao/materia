#include "ring.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace stockkit {
namespace {
constexpr double pi = 3.14159265358979323846, tau = 2 * pi;
constexpr uint32_t original = SK_SOURCE_STOCK;

// Ordered subtraction preserves old endpoints and attributes only newly exposed ones.
RingCircle subtract(const RingCircle &in, double lo, double hi, uint32_t gash, uint32_t pass) {
    RingCircle out;
    out.reserve(in.size() + 1);
    for (const auto &i : in) {
        if (hi <= i.lo || lo >= i.hi) { out.push_back(i); continue; }
        if (i.lo < lo) out.push_back({i.lo, lo, i.lo_gash, i.lo_pass, gash, pass});
        if (hi < i.hi) out.push_back({hi, i.hi, gash, pass, i.hi_gash, i.hi_pass});
    }
    return out;
}
RingCircle difference(const RingCircle &a, const RingCircle &b) {
    RingCircle out;
    size_t j = 0;
    for (const auto &i : a) {
        double cursor = i.lo;
        uint32_t gash = i.lo_gash, pass = i.lo_pass;
        while (j < b.size() && b[j].hi <= cursor) ++j;
        size_t k = j;
        while (k < b.size() && b[k].lo < i.hi) {
            const auto &cut = b[k];
            if (cursor < cut.lo) out.push_back({cursor, cut.lo, gash, pass, cut.lo_gash, cut.lo_pass});
            if (cut.hi > cursor) { cursor = cut.hi; gash = cut.hi_gash; pass = cut.hi_pass; }
            if (cursor >= i.hi) break;
            ++k;
        }
        if (cursor < i.hi) out.push_back({cursor, i.hi, gash, pass, i.hi_gash, i.hi_pass});
    }
    return out;
}
double length(const RingCircle &a) {
    double sum = 0;
    for (const auto &i : a) sum += i.hi - i.lo;
    return sum;
}
}

bool RingStock::valid(const sk_ring_grid &g) {
    return g.struct_size == sizeof(g) && g.radial_count > 0 && g.axial_count > 0 &&
        uint64_t(g.radial_count) * g.axial_count <= 10000000 &&
        std::isfinite(g.radius_min) && std::isfinite(g.radius_max) &&
        std::isfinite(g.z_min) && std::isfinite(g.z_max) && g.radius_min > 0 &&
        g.radius_max > g.radius_min && g.z_max > g.z_min &&
        std::isfinite(g.radius_max - g.radius_min) && std::isfinite(g.z_max - g.z_min);
}
RingStock::RingStock(const sk_ring_grid &g) : grid(g) {
    layers.reserve(g.radial_count);
    for (uint32_t r = 0; r < g.radial_count; ++r) {
        auto layer = std::make_shared<RingLayer>();
        layer->circles.resize(g.axial_count);
        for (auto &c : layer->circles) c.push_back({0, tau, original, original, original, original});
        layers.push_back(std::move(layer));
    }
}
bool RingStock::same_grid(const sk_ring_grid &g) const {
    return grid.radial_count == g.radial_count && grid.axial_count == g.axial_count &&
        grid.radius_min == g.radius_min && grid.radius_max == g.radius_max &&
        grid.z_min == g.z_min && grid.z_max == g.z_max;
}
bool RingStock::valid_cut(const sk_ring_cut &c) const {
    return c.struct_size == sizeof(c) && c.radial_index < grid.radial_count &&
        c.axial_first < grid.axial_count && c.axial_count > 0 &&
        c.axial_count <= grid.axial_count - c.axial_first &&
        std::isfinite(c.start) && std::isfinite(c.sweep) && c.sweep >= 0 && c.sweep <= tau &&
        c.gash < SK_SOURCE_NONE && c.pass < SK_SOURCE_NONE;
}
void RingStock::cut(const sk_ring_cut *cuts, uint32_t count) {
    std::vector<std::vector<uint32_t>> bins(grid.radial_count);
    for (uint32_t k = 0; k < count; ++k) if (cuts[k].sweep > 0) bins[cuts[k].radial_index].push_back(k);
    auto next = layers;
    const unsigned jobs = std::min<unsigned>(pool.size(), grid.radial_count);
    // A job owns entire radial layers, so interval ordering and provenance do not depend on scheduling.
    pool.run(jobs, [&](unsigned job) {
        for (uint32_t r = job; r < grid.radial_count; r += jobs) {
            if (bins[r].empty()) continue;
            auto layer = std::make_shared<RingLayer>(*layers[r]);
            for (uint32_t k : bins[r]) {
                const auto &c = cuts[k];
                double start = std::fmod(c.start, tau);
                if (start < 0) start += tau;
                if (start == 0) start = 0; // canonical positive zero
                for (uint32_t z = c.axial_first; z < c.axial_first + c.axial_count; ++z) {
                    auto &material = layer->circles[z];
                    if (c.sweep == tau) material.clear();
                    else if (start + c.sweep <= tau)
                        material = subtract(material, start, start + c.sweep, c.gash, c.pass);
                    else {
                        material = subtract(material, start, tau, c.gash, c.pass);
                        material = subtract(material, 0, start + c.sweep - tau, c.gash, c.pass);
                    }
                }
            }
            // Store only live intervals. This keeps the reference-part budget independent of vector growth.
            for (auto &c : layer->circles) RingCircle(c).swap(c);
            next[r] = std::move(layer);
        }
    });
    layers.swap(next); // commit only after all jobs succeed
}
sk_ring_info RingStock::info() const {
    sk_ring_info out{};
    out.struct_size = sizeof(out);
    out.circle_count = grid.radial_count * grid.axial_count;
    out.bytes = sizeof(*this) + layers.capacity() * sizeof(layers[0]);
    for (const auto &layer : layers) {
        out.bytes += sizeof(RingLayer) + 2 * sizeof(void*) + layer->circles.capacity() * sizeof(RingCircle);
        for (const auto &c : layer->circles) {
            out.interval_count += c.size();
            out.bytes += c.capacity() * sizeof(sk_ring_interval);
        }
    }
    return out;
}

// Compare in a fixed order so even reductions can be reproduced across thread counts.
void ring_compare(const RingStock &a, const RingStock &b, sk_ring_comparison *out) {
    for (uint32_t z = 0; z < a.grid.axial_count; ++z) for (uint32_t r = 0; r < a.grid.radial_count; ++r) {
        out[z * a.grid.radial_count + r] = {length(difference(a.circle(r,z), b.circle(r,z))),
            length(difference(b.circle(r,z), a.circle(r,z)))};
    }
}

void RingStock::contour(double max_angle, PreviewMesh &out) const {
    using V = std::array<double, 3>;
    const double dr = (grid.radius_max-grid.radius_min)/grid.radial_count;
    const double dz = (grid.z_max-grid.z_min)/grid.axial_count;
    auto point = [](double r, double angle, double z) -> V { return {r*std::cos(angle), r*std::sin(angle), z}; };
    auto quad = [&](V a, V b, V c, V d, uint32_t source) {
        if (out.positions.size()/3 > std::numeric_limits<uint32_t>::max()-4) throw std::bad_alloc();
        V u{b[0]-a[0],b[1]-a[1],b[2]-a[2]}, v{c[0]-a[0],c[1]-a[1],c[2]-a[2]};
        V n{u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]};
        double norm = std::sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
        if (norm == 0) return;
        uint32_t first = out.vertex_count();
        for (const auto &p : {a,b,c,d}) {
            for (int k=0;k<3;++k) { out.positions.push_back(float(p[k])); out.normals.push_back(float(n[k]/norm)); }
            out.vertex_sources.push_back(source);
            out.colors.push_back(0xFFFFFFFFu);
        }
        for (uint32_t i : {0u,1u,2u,0u,2u,3u}) out.indices.push_back(first+i);
        out.triangle_sources.push_back(source); out.triangle_sources.push_back(source);
    };
    const RingCircle empty;
    for (uint32_t r=0;r<grid.radial_count;++r) for(uint32_t z=0;z<grid.axial_count;++z) {
        const auto &mat=circle(r,z);
        if(mat.empty()) continue;
        double ri=grid.radius_min+r*dr, ro=ri+dr, zl=grid.z_min+z*dz, zh=zl+dz;
        for(int face=0;face<4;++face) {
            const RingCircle *neighbor=&empty;
            if(face==0 && r>0) neighbor=&circle(r-1,z);
            if(face==1 && r+1<grid.radial_count) neighbor=&circle(r+1,z);
            if(face==2 && z>0) neighbor=&circle(r,z-1);
            if(face==3 && z+1<grid.axial_count) neighbor=&circle(r,z+1);
            for(const auto &i:difference(mat,*neighbor)) {
                uint32_t steps=uint32_t(std::ceil((i.hi-i.lo)/max_angle));
                for(uint32_t k=0;k<steps;++k) {
                    double a=i.lo+(i.hi-i.lo)*k/steps,b=i.lo+(i.hi-i.lo)*(k+1)/steps;
                    if(face==0) quad(point(ri,b,zl),point(ri,a,zl),point(ri,a,zh),point(ri,b,zh),i.lo_pass);
                    if(face==1) quad(point(ro,a,zl),point(ro,b,zl),point(ro,b,zh),point(ro,a,zh),i.lo_pass);
                    if(face==2) quad(point(ri,a,zl),point(ri,b,zl),point(ro,b,zl),point(ro,a,zl),i.lo_pass);
                    if(face==3) quad(point(ri,b,zh),point(ri,a,zh),point(ro,a,zh),point(ro,b,zh),i.lo_pass);
                }
            }
        }
        bool seam=mat.front().lo==0 && mat.back().hi==tau;
        for(const auto &i:mat) {
            if(!(seam && i.lo==0)) quad(point(ri,i.lo,zl),point(ro,i.lo,zl),point(ro,i.lo,zh),point(ri,i.lo,zh),i.lo_pass);
            if(!(seam && i.hi==tau)) quad(point(ro,i.hi,zl),point(ri,i.hi,zl),point(ri,i.hi,zh),point(ro,i.hi,zh),i.hi_pass);
        }
    }
}
} // namespace stockkit
