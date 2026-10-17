#include "rake.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace stockkit {
namespace {
constexpr double tau=6.28318530717958647692;
struct P2 { double x,y; };
double cross(P2 a,P2 b,P2 c) {return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
RakePoint minus(RakePoint a,RakePoint b) {return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
RakePoint cross3(RakePoint a,RakePoint b) {return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
double norm(RakePoint a) {return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);}
std::vector<P2> hull(std::vector<P2> p) {
    std::sort(p.begin(),p.end(),[](P2 a,P2 b){return a.x<b.x || (a.x==b.x&&a.y<b.y);});
    p.erase(std::unique(p.begin(),p.end(),[](P2 a,P2 b){return a.x==b.x&&a.y==b.y;}),p.end());
    if(p.size()<3)return {};
    std::vector<P2> h;
    for(auto a:p) {while(h.size()>1 && cross(h[h.size()-2],h.back(),a)<=0)h.pop_back();h.push_back(a);}
    auto bottom=h.size();
    for(size_t k=p.size()-1;k-->0;) {auto a=p[k];while(h.size()>bottom && cross(h[h.size()-2],h.back(),a)<=0)h.pop_back();h.push_back(a);}
    h.pop_back();return h;
}
using Arc=std::pair<double,double>;
std::vector<Arc> circle_inside(const std::vector<P2>&p,double radius) {
    std::vector<Arc> spans{{0,tau}};
    for(size_t k=0;k<p.size()&&!spans.empty();++k) {
        P2 a=p[k],b=p[(k+1)%p.size()];double nx=a.y-b.y,ny=b.x-a.x;
        double scale=std::hypot(nx,ny),d=(nx*a.x+ny*a.y)/(scale*radius);
        if(d>=1)return {};
        if(d<=-1)continue;
        double centre=std::atan2(ny,nx),half=std::acos(d),lo=centre-half,hi=centre+half;
        while(lo<0){lo+=tau;hi+=tau;}
        while(lo>=tau){lo-=tau;hi-=tau;}
        std::vector<Arc> allowed;
        if(hi<=tau)allowed.push_back({lo,hi});
        else {allowed.push_back({0,hi-tau});allowed.push_back({lo,tau});}
        std::vector<Arc> next;
        for(auto s:spans)for(auto t:allowed) {
            double l=std::max(s.first,t.first),h=std::min(s.second,t.second);
            if(l<h)next.push_back({l,h});
        }
        spans.swap(next);
    }
    return spans;
}
RakePoint transform(RakePoint p,const sk_rake_pose &pose) {
    RakePoint out{};
    for(int i=0;i<3;++i)out[i]=pose.translation[i]+pose.rotation[3*i]*p[0]+pose.rotation[3*i+1]*p[1]+pose.rotation[3*i+2]*p[2];
    return out;
}
}

bool build_rake_tool(const double *vertices,uint32_t vertex_count,const sk_rake_region *regions,
    uint32_t region_count,uint32_t gashes,RakeTool &out) {
    if(!vertices||!regions||!region_count||gashes<2||gashes>256||vertex_count<3)return false;
    out.gashes.resize(gashes);
    for(uint32_t region=0;region<region_count;++region) {
        const auto &r=regions[region];
        if(r.struct_size!=sizeof(r)||r.gash>=gashes||r.count<3||r.first>vertex_count||r.count>vertex_count-r.first)return false;
        std::vector<RakePoint> points;
        for(uint32_t k=0;k<r.count;++k) {
            auto i=3ull*(r.first+k);RakePoint p{vertices[i],vertices[i+1],vertices[i+2]};
            for(double v:p)if(!std::isfinite(v))return false;
            if(points.empty()||norm(minus(p,points.back()))>1e-12)points.push_back(p);
        }
        if(points.size()>2&&norm(minus(points.front(),points.back()))<1e-12)points.pop_back();
        if(points.size()<3)return false;
        RakePoint n{};
        for(size_t k=1;k+1<points.size();++k) {
            auto candidate=cross3(minus(points[k],points[0]),minus(points[k+1],points[0]));
            if(norm(candidate)>norm(n))n=candidate;
        }
        double magnitude=norm(n);if(!std::isfinite(magnitude)||magnitude<=1e-14)return false;
        for(auto p:points) {
            auto d=minus(p,points[0]);
            if(std::abs(d[0]*n[0]+d[1]*n[1]+d[2]*n[2])/magnitude>1e-8)return false;
        }
        int drop=0;for(int k=1;k<3;++k)if(std::abs(n[k])>std::abs(n[drop]))drop=k;
        int u=(drop+1)%3,v=(drop+2)%3;
        std::vector<P2> p;for(auto q:points)p.push_back({q[u],q[v]});
        double area=0;for(size_t k=0;k<p.size();++k)area+=p[k].x*p[(k+1)%p.size()].y-p[k].y*p[(k+1)%p.size()].x;
        if(!std::isfinite(area)||area==0)return false;
        std::vector<uint32_t> remaining;for(uint32_t k=0;k<p.size();++k)remaining.push_back(k);
        if(area<0)std::reverse(remaining.begin(),remaining.end());
        auto &mesh=out.gashes[r.gash];uint32_t base=uint32_t(mesh.vertices.size());
        mesh.vertices.insert(mesh.vertices.end(),points.begin(),points.end());
        while(remaining.size()>2) {
            bool ear=false;
            for(size_t k=0;k<remaining.size();++k) {
                uint32_t a=remaining[(k+remaining.size()-1)%remaining.size()],b=remaining[k],c=remaining[(k+1)%remaining.size()];
                if(cross(p[a],p[b],p[c])<=1e-16)continue;
                bool inside=false;
                for(auto q:remaining)if(q!=a&&q!=b&&q!=c&&cross(p[a],p[b],p[q])>=-1e-16&&cross(p[b],p[c],p[q])>=-1e-16&&cross(p[c],p[a],p[q])>=-1e-16){inside=true;break;}
                if(inside)continue;
                mesh.triangles.push_back({base+a,base+b,base+c});
                remaining.erase(remaining.begin()+k);ear=true;break;
            }
            if(!ear)return false;
        }
    }
    return std::all_of(out.gashes.begin(),out.gashes.end(),[](const RakeMesh &m){return !m.triangles.empty();});
}

void sweep_rake(RingStock &stock,const RakeTool &tool,uint32_t gash,uint32_t pass,
    const sk_rake_pose *poses,uint32_t pose_count) {
    const auto &mesh=tool.gashes[gash];const auto &g=stock.grid;
    std::vector<std::vector<Arc>> removed(size_t(g.radial_count)*g.axial_count);
    const double dr=(g.radius_max-g.radius_min)/g.radial_count,dz=(g.z_max-g.z_min)/g.axial_count;
    std::vector<RakePoint> previous,current(mesh.vertices.size());
    for(size_t k=0;k<mesh.vertices.size();++k)current[k]=transform(mesh.vertices[k],poses[0]);
    for(uint32_t pose=1;pose<pose_count;++pose) {
        previous.swap(current);current.resize(mesh.vertices.size());
        for(size_t k=0;k<mesh.vertices.size();++k)current[k]=transform(mesh.vertices[k],poses[pose]);
        double allMinZ=std::numeric_limits<double>::infinity(),allMaxZ=-allMinZ;
        for(size_t k=0;k<current.size();++k) {allMinZ=std::min({allMinZ,previous[k][2],current[k][2]});allMaxZ=std::max({allMaxZ,previous[k][2],current[k][2]});}
        if(allMinZ>g.z_max||allMaxZ<g.z_min)continue;
        for(auto tri:mesh.triangles) {
            std::array<RakePoint,6> p{previous[tri[0]],previous[tri[1]],previous[tri[2]],current[tri[0]],current[tri[1]],current[tri[2]]};
            auto planeNormal=cross3(minus(p[1],p[0]),minus(p[2],p[0]));
            double thickness=0;
            for(size_t k=3;k<6;++k) {auto d=minus(p[k],p[0]);thickness=std::max(thickness,std::abs(d[0]*planeNormal[0]+d[1]*planeNormal[1]+d[2]*planeNormal[2]));}
            if(thickness==0)continue; // a stationary or in-plane moving face sweeps no volume
            double minz=p[0][2],maxz=minz,minx=p[0][0],maxx=minx,miny=p[0][1],maxy=miny;
            for(auto v:p){minz=std::min(minz,v[2]);maxz=std::max(maxz,v[2]);minx=std::min(minx,v[0]);maxx=std::max(maxx,v[0]);miny=std::min(miny,v[1]);maxy=std::max(maxy,v[1]);}
            if(minx>g.radius_max||maxx<-g.radius_max||miny>g.radius_max||maxy<-g.radius_max||minz>g.z_max||maxz<g.z_min)continue;
            double closeX=std::clamp(0.0,minx,maxx),closeY=std::clamp(0.0,miny,maxy);
            double farX=std::max(std::abs(minx),std::abs(maxx)),farY=std::max(std::abs(miny),std::abs(maxy));
            if(std::hypot(closeX,closeY)>g.radius_max||std::hypot(farX,farY)<g.radius_min)continue;
            int first=int(std::clamp(std::ceil((minz-g.z_min)/dz-0.5),0.0,double(g.axial_count))),last=int(std::clamp(std::floor((maxz-g.z_min)/dz-0.5),-1.0,double(g.axial_count)-1));
            for(int z=first;z<=last;++z) {
                double height=g.z_min+(z+0.5)*dz;std::vector<P2> intersections;
                for(size_t a=0;a<6;++a) {
                    if(p[a][2]==height)intersections.push_back({p[a][0],p[a][1]});
                    for(size_t b=a+1;b<6;++b)if((p[a][2]<height&&p[b][2]>height)||(p[b][2]<height&&p[a][2]>height)) {
                        double t=(height-p[a][2])/(p[b][2]-p[a][2]);
                        intersections.push_back({p[a][0]+t*(p[b][0]-p[a][0]),p[a][1]+t*(p[b][1]-p[a][1])});
                    }
                }
                auto polygon=hull(std::move(intersections));if(polygon.size()<3)continue;
                double rmax=0,rmin=std::numeric_limits<double>::infinity();bool contains=true;
                for(size_t k=0;k<polygon.size();++k) {
                    P2 a=polygon[k],b=polygon[(k+1)%polygon.size()];rmax=std::max(rmax,std::hypot(a.x,a.y));
                    double dx=b.x-a.x,dy=b.y-a.y,t=std::clamp(-(a.x*dx+a.y*dy)/(dx*dx+dy*dy),0.0,1.0);
                    rmin=std::min(rmin,std::hypot(a.x+t*dx,a.y+t*dy));if(cross(a,b,{0,0})<0)contains=false;
                }
                if(contains)rmin=0;
                int r0=int(std::clamp(std::ceil((rmin-g.radius_min)/dr-0.5),0.0,double(g.radial_count))),r1=int(std::clamp(std::floor((rmax-g.radius_min)/dr-0.5),-1.0,double(g.radial_count)-1));
                for(int r=r0;r<=r1;++r) {
                    auto arcs=circle_inside(polygon,g.radius_min+(r+0.5)*dr);
                    auto &bin=removed[size_t(z)*g.radial_count+r];bin.insert(bin.end(),arcs.begin(),arcs.end());
                }
            }
        }
    }
    std::vector<sk_ring_cut> cuts;
    for(uint32_t z=0;z<g.axial_count;++z)for(uint32_t r=0;r<g.radial_count;++r) {
        auto &arcs=removed[size_t(z)*g.radial_count+r];if(arcs.empty())continue;
        std::sort(arcs.begin(),arcs.end());double lo=arcs[0].first,hi=arcs[0].second;
        auto emit=[&](){cuts.push_back({sizeof(sk_ring_cut),r,z,1,lo,hi-lo,gash,pass});};
        for(size_t k=1;k<arcs.size();++k)if(arcs[k].first<=hi+32*std::numeric_limits<double>::epsilon()*std::max(1.0,std::abs(hi)))hi=std::max(hi,arcs[k].second);else {emit();lo=arcs[k].first;hi=arcs[k].second;}
        emit();
    }
    stock.cut(cuts.data(),uint32_t(cuts.size()));
}
}
