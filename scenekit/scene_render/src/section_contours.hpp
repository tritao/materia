#pragma once

#include "nativekit_scene.hpp"
#include <cmath>
#include <limits>
#include <map>

namespace nkscene::render_internal {

using SectionPlaneLocal = std::array<double, 4>;
using SectionPoint = std::array<float, 3>;

// Plane covectors pull back by M^T, including nonuniform scales and reflections.
inline SectionPlaneLocal section_plane_local(const std::array<float,4> &plane,
                                             const std::array<float,16> &model) {
    SectionPlaneLocal local{};
    for (unsigned col=0; col<4; ++col)
        for (unsigned row=0; row<4; ++row)
            local[col] += double(model[col*4+row])*plane[row];
    const double norm=std::hypot(local[0],local[1],local[2]);
    if (norm>0) for (auto &value:local) value/=norm;
    return local;
}

/** Exact mesh-plane segments, with deterministic shared-edge interpolation.
 * Coplanar triangles contribute no diagonals. An edge on the plane contributes
 * once only when its adjacent noncoplanar triangles cross to opposite sides.
 */
inline std::vector<GeometryPayload::StrokeSegment>
section_contour_segments(const GeometryPayload &mesh,const SectionPlaneLocal &plane) {
    using SegmentKey=std::array<SectionPoint,2>;
    // Bits 0/1 are incident positive/negative triangles; bit 2 is a proper cut.
    std::map<SegmentKey,unsigned> segments;
    double scale=std::abs(plane[3]);
    for (const auto &vertex:mesh.vertices)
        for (unsigned axis=0;axis<3;++axis)
            scale=std::max(scale,std::abs(double(vertex.position[axis])));
    const double epsilon=scale*64*std::numeric_limits<double>::epsilon();
    auto distance=[&](SectionPoint p) {
        double d=plane[3];for(unsigned axis=0;axis<3;++axis)d+=plane[axis]*p[axis];
        return std::abs(d)<=epsilon?0.0:d;
    };
    auto insert=[&](SectionPoint a,SectionPoint b,unsigned flag) {
        if(a==b)return;
        if(b<a)std::swap(a,b);
        segments[{a,b}]|=flag;
    };
    auto vertex=[&](std::size_t i) {return mesh.vertices[mesh.indices.empty()?i:mesh.indices[i]].position;};
    for(std::size_t i=0;i+2<mesh.element_count();i+=3) {
        SectionPoint p[3]={vertex(i),vertex(i+1),vertex(i+2)};
        double d[3]={distance(p[0]),distance(p[1]),distance(p[2])};
        int zeros=0,positive=0,negative=0;
        for(double v:d){zeros+=v==0;positive+=v>0;negative+=v<0;}
        if(zeros==3)continue;
        if(zeros==2) {
            SectionPoint edge[2];int n=0;for(int j=0;j<3;++j)if(d[j]==0)edge[n++]=p[j];
            insert(edge[0],edge[1],positive?1:2);continue;
        }
        if(!positive||!negative)continue; // Tangent vertex/edge has no cap area.
        SectionPoint cut[3];int count=0;
        for(int j=0;j<3;++j) {
            if(d[j]==0)cut[count++]=p[j];
            int k=(j+1)%3;
            if((d[j]>0&&d[k]<0)||(d[j]<0&&d[k]>0)) {
                auto a=p[j],b=p[k];if(b<a)std::swap(a,b);
                double da=distance(a),db=distance(b),t=da/(da-db);
                SectionPoint point{};for(unsigned axis=0;axis<3;++axis)
                    point[axis]=float(double(a[axis])+t*(double(b[axis])-a[axis]));
                cut[count++]=point;
            }
        }
        if(count==2)insert(cut[0],cut[1],4);
    }
    std::vector<GeometryPayload::StrokeSegment> result;
    result.reserve(segments.size());
    for(const auto &[edge,sides]:segments)
        if((sides&4)||(sides&3)==3)result.push_back({edge[0],edge[1],0});
    return result;
}

struct SectionContour {
    std::weak_ptr<const GeometryPayload> source;
    SectionPlaneLocal plane{};
    std::uint64_t revision=0;
    std::vector<GeometryPayload::StrokeSegment> segments;

    bool update(const std::shared_ptr<const GeometryPayload> &mesh,const SectionPlaneLocal &local,std::uint64_t mesh_revision=0) {
        if(source.lock()==mesh && plane==local && revision==mesh_revision)return false;
        segments=section_contour_segments(*mesh,local);
        source=mesh;plane=local;revision=mesh_revision;return true;
    }
};

} // namespace nkscene::render_internal
