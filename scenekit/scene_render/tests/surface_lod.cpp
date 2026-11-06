#include "surface_lod.hpp"
#include <cassert>
#include <cmath>
#include <limits>

using namespace nkscene;
using namespace nkscene::render_internal;

int main() {
    // A flat CAD face with many redundant triangles must simplify without
    // changing source storage, its outer boundary, or its normal field.
    struct Vertex { float p[3], n[3]; };
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    constexpr unsigned side = 96;
    for (unsigned y=0; y<=side; ++y) for (unsigned x=0; x<=side; ++x)
        vertices.push_back({{float(x)/side, float(y)/side, 0}, {0,0,1}});
    for (unsigned y=0; y<side; ++y) for (unsigned x=0; x<side; ++x) {
        const auto a=y*(side+1)+x, b=a+1, c=a+side+1, d=c+1;
        indices.insert(indices.end(), {a,b,d,a,d,c});
    }
    const auto original = indices;
    const auto levels = build_surface_lods(vertices[0].p, vertices[0].n,
        vertices.size(), sizeof(Vertex), indices);
    assert(!levels.empty() && levels.back().indices.size() < indices.size()/2);
    assert(indices == original);
    std::vector<Vertex> soup;
    for (auto index : indices) soup.push_back(vertices[index]);
    const auto soup_levels = build_surface_lods(soup[0].p,soup[0].n,soup.size(),sizeof(Vertex),{});
    assert(!soup_levels.empty() && soup_levels.back().indices.size() < indices.size()/2);
    for (const auto &level : levels) {
        assert(std::isfinite(level.error) && level.error >= 0);
        for (auto index : level.indices) assert(index < vertices.size());
    }
    indices[0] = unsigned(vertices.size());
    assert(build_surface_lods(vertices[0].p, vertices[0].n,
        vertices.size(), sizeof(Vertex), indices).empty());
    assert(build_surface_lods(nullptr, nullptr, 0, sizeof(Vertex), {}).empty());

    const std::array<float,16> identity{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    Bounds bounds{{-1,-1,-1}, {1,1,1}, true};
    const auto scale = surface_lod_pixel_scale(bounds, identity, identity, 800,600,0);
    assert(std::abs(scale-500) < 1e-6);
    auto model = identity;
    model[0] = -2; model[5] = 3; // Reflection and unequal instance scales.
    assert(surface_lod_pixel_scale(bounds, identity, model,800,600,0) >= 900);
    auto perspective = identity;
    perspective[11] = -1; perspective[15] = 0;
    assert(std::isinf(surface_lod_pixel_scale(bounds,perspective,identity,800,600,0)));
    model = identity; model[14] = -10;
    const auto far = surface_lod_pixel_scale(bounds,perspective,model,800,600,0.01);
    model[14] = -5;
    assert(surface_lod_pixel_scale(bounds,perspective,model,800,600,0.01) > far);
    model[0] = -2; model[5] = 0.7f; model[8] = 0.2f;
    const auto bound = surface_lod_pixel_scale(bounds,perspective,model,800,600,0.01);
    const auto screen = [&](double x, double y, double z) {
        const auto w = -(model[2]*x+model[6]*y+model[10]*z+model[14]);
        return std::array<double,2>{400*(model[0]*x+model[4]*y+model[8]*z+model[12])/w,
                                   300*(model[1]*x+model[5]*y+model[9]*z+model[13])/w};
    };
    for (unsigned sample=0; sample<1000; ++sample) {
        const auto x=std::sin(sample*1.1), y=std::cos(sample*1.7), z=std::sin(sample*0.3);
        const auto a=screen(x,y,z), b=screen(x+0.001,y-0.002,z+0.003);
        assert(std::hypot(b[0]-a[0],b[1]-a[1]) <= bound*std::sqrt(0.000014));
    }
    bounds.valid = false;
    assert(std::isinf(surface_lod_pixel_scale(bounds,identity,identity,800,600,0)));
}
