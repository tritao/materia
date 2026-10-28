#include "stockkit.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double pi=3.14159265358979323846,tau=2*pi;
int checks=0;
void check(bool value,const char *what) { ++checks; if(!value) throw std::runtime_error(what); }
void near(double a,double b,double tol,const char *what) { check(std::isfinite(a)&&std::abs(a-b)<=tol,what); }
sk_ring_grid grid(double ri=1,double ro=3,uint32_t nr=8,uint32_t nz=4) {
    return {sizeof(sk_ring_grid),nr,nz,ri,ro,-1,1};
}
sk_ring_stock_handle create(const sk_ring_grid &g) {
    sk_ring_stock_handle h{}; check(sk_ring_stock_create(&g,&h)==SK_OK,"create"); return h;
}
std::vector<sk_ring_interval> read(sk_ring_stock_handle s,uint32_t r,uint32_t z) {
    uint32_t n=0; auto result=sk_ring_stock_read_circle(s,r,z,nullptr,&n);
    check(result==SK_OK||result==SK_ERROR_LIMIT,"circle sizing");
    std::vector<sk_ring_interval> out(n);
    check(sk_ring_stock_read_circle(s,r,z,out.data(),&n)==SK_OK,"circle read");return out;
}
sk_ring_cut cut(uint32_t r,double start,double sweep,uint32_t gash=7,uint32_t pass=2,uint32_t z=0,uint32_t nz=4) {
    return {sizeof(sk_ring_cut),r,z,nz,start,sweep,gash,pass};
}
void same(const std::vector<sk_ring_interval>&a,const std::vector<sk_ring_interval>&b) {
    check(a.size()==b.size(),"same interval count");
    check(a.empty()||std::memcmp(a.data(),b.data(),a.size()*sizeof(a[0]))==0,"bit-identical endpoints and provenance");
}
void slab() {
    auto g=grid();auto s=create(g);
    check(sk_ring_stock_cut_slab(s,0,0.3,1.2,9,4)==SK_OK,"analytic slab");
    for(uint32_t r=0;r<g.radial_count;++r) for(uint32_t z=0;z<g.axial_count;++z) {
        double radius=g.radius_min+(r+0.5)*(g.radius_max-g.radius_min)/g.radial_count;
        double a=std::acos(std::min(1.0,1.2/radius)),b=std::acos(0.3/radius);
        auto c=read(s,r,z);
        if(a==0) {
            check(c.size()==1,"slab crosses angular seam");near(c[0].lo,b,1e-14,"slab lower boundary");
            near(c[0].hi,tau-b,1e-14,"slab upper boundary");
        } else {
            check(c.size()==3,"slab has two removed arcs");
            near(c[0].hi,a,1e-14,"slab first plane positive");near(c[1].lo,b,1e-14,"slab second plane positive");
            near(c[1].hi,tau-b,1e-14,"slab second plane negative");near(c[2].lo,tau-a,1e-14,"slab first plane negative");
        }
        check(c[c.size()==1?0:1].lo_gash==9,"slab boundary gash");
        check(c[c.size()==1?0:1].lo_pass==4,"slab boundary pass");
    }
    sk_ring_stock_destroy(s);
}
void operations() {
    auto g=grid();auto s=create(g);
    auto wrapping=cut(0,tau-0.2,0.5);
    check(sk_ring_stock_cut_region(s,&wrapping,1)==SK_OK,"wrapped cut");
    auto c=read(s,0,0);check(c.size()==1,"seam removed");near(c[0].lo,0.3,1e-14,"wrap low");
    near(c[0].hi,tau-0.2,1e-14,"wrap high");check(c[0].lo_pass==2&&c[0].hi_gash==7,"wrap provenance");
    auto before=c;
    wrapping.gash=10;wrapping.pass=8;
    check(sk_ring_stock_cut_region(s,&wrapping,1)==SK_OK,"recut");same(before,read(s,0,0));
    auto cuts=std::vector<sk_ring_cut>{cut(0,1,1,11,3),cut(0,1.5,1,12,5),cut(0,0.8,0,20,20)};
    check(sk_ring_stock_cut_region(s,cuts.data(),uint32_t(cuts.size()))==SK_OK,"overlap and zero sweep");
    c=read(s,0,0);check(c.size()==2,"overlap merges void");
    check(c[0].hi_gash==11&&c[1].lo_gash==12,"overlap preserves old and new boundaries");
    sk_ring_snapshot_handle shot{};check(sk_ring_stock_snapshot(s,&shot)==SK_OK,"snapshot");
    auto saved=c;auto full=cut(0,-99,tau);
    check(sk_ring_stock_cut_region(s,&full,1)==SK_OK,"full circle cut");check(read(s,0,0).empty(),"no phantom seam material");
    check(sk_ring_stock_restore(s,shot)==SK_OK,"restore");same(saved,read(s,0,0));
    auto other=create(g);check(sk_ring_stock_restore(other,shot)==SK_OK,"snapshot to another same-grid stock");
    sk_ring_stock_destroy(s);check(sk_ring_stock_restore(other,shot)==SK_OK,"snapshot survives owner destruction");
    same(saved,read(other,0,0));
    auto bad=grid(1,4);auto mismatch=create(bad);
    check(sk_ring_stock_restore(mismatch,shot)==SK_ERROR_INVALID_ARGUMENT,"reject mismatched grid");
    auto valid=cut(1,0,1),invalid=cut(1,0,-1);
    sk_ring_cut batch[]={valid,invalid}; auto original=read(other,1,0);
    check(sk_ring_stock_cut_region(other,batch,2)==SK_ERROR_INVALID_ARGUMENT,"batch validation atomic");same(original,read(other,1,0));
    invalid=valid;invalid.struct_size=0;
    check(sk_ring_stock_cut_region(other,&invalid,1)==SK_ERROR_INVALID_ARGUMENT,"reject wrong schema");
    invalid=valid;invalid.axial_count=UINT32_MAX;
    check(sk_ring_stock_cut_region(other,&invalid,1)==SK_ERROR_INVALID_ARGUMENT,"reject axial overflow");
    invalid=valid;invalid.start=NAN;
    check(sk_ring_stock_cut_region(other,&invalid,1)==SK_ERROR_INVALID_ARGUMENT,"reject NaN");
    sk_ring_snapshot_destroy(shot);check(sk_ring_stock_restore(other,shot)==SK_ERROR_INVALID_HANDLE,"stale snapshot");
    sk_ring_stock_destroy(other);sk_ring_stock_destroy(mismatch);
    sk_ring_info info{};check(sk_ring_stock_get_info(other,&info)==SK_ERROR_INVALID_HANDLE,"stale stock");
    auto fresh=create(g);check(fresh.id!=other.id,"generation advances");sk_ring_stock_destroy(fresh);
    auto badGrid=g;badGrid.struct_size=0;sk_ring_stock_handle out{123};
    check(sk_ring_stock_create(&badGrid,&out)==SK_ERROR_INVALID_ARGUMENT&&out.id==0,"create clears failed handle");
}
void deterministic() {
    auto g=grid(2,6,32,12);std::vector<sk_ring_cut> cuts;
    for(uint32_t k=0;k<700;++k) cuts.push_back(cut(k%32,(double(k%71)-35)*0.13,0.013+(k%9)*0.009,
        k%12,k/12,k%6,6));
    auto a=create(g);
    check(sk_ring_stock_cut_region(a,cuts.data(),uint32_t(cuts.size()))==SK_OK,"serial cuts");
    for(uint32_t threads:{2u,4u}) {
        auto b=create(g);check(sk_ring_stock_set_threads(b,threads)==SK_OK,"set worker count");
        check(sk_ring_stock_cut_region(b,cuts.data(),uint32_t(cuts.size()))==SK_OK,"parallel cuts");
        for(uint32_t r=0;r<g.radial_count;++r)for(uint32_t z=0;z<g.axial_count;++z)same(read(a,r,z),read(b,r,z));
        std::vector<sk_ring_comparison> comparison(g.radial_count*g.axial_count);
        check(sk_ring_stock_compare(a,b,comparison.data(),uint32_t(comparison.size()))==SK_OK,"compare equal stocks");
        for(const auto &v:comparison)check(v.leftover==0&&v.gouge==0,"equal stock has no deviations");
        // Resizing a pool after a completed parallel round must not execute a stale job.
        for(uint32_t resized:{1u,4u,2u}) {
            check(sk_ring_stock_set_threads(b,resized)==SK_OK,"resize used pool");
            check(sk_ring_stock_cut_region(b,cuts.data(),uint32_t(cuts.size()))==SK_OK,"cut after pool resize");
            for(uint32_t r=0;r<g.radial_count;++r)for(uint32_t z=0;z<g.axial_count;++z)same(read(a,r,z),read(b,r,z));
        }
        sk_ring_stock_destroy(b);
    }
    auto target=create(g);std::vector<sk_ring_comparison> comparison(g.radial_count*g.axial_count);
    check(sk_ring_stock_compare(a,target,comparison.data(),uint32_t(comparison.size()))==SK_OK,"compare blank");
    for(uint32_t z=0;z<g.axial_count;++z)for(uint32_t r=0;r<g.radial_count;++r) {
        double retained=0;for(const auto &i:read(a,r,z))retained+=i.hi-i.lo;
        near(comparison[z*g.radial_count+r].gouge,tau-retained,2e-14,"gouge angular measure");
        check(comparison[z*g.radial_count+r].leftover==0,"cut stock has no leftover vs blank");
    }
    sk_ring_stock_destroy(a);sk_ring_stock_destroy(target);
}
void contour() {
    auto g=grid(2,3,2,2);auto s=create(g);
    // A full annulus has no surfaces on the artificial angular seam or between equal cells.
    sk_mesh_handle mesh{};check(sk_ring_stock_contour(s,pi/16,&mesh)==SK_OK,"annular contour");
    sk_mesh_info info{};check(sk_mesh_get_info(mesh,&info)==SK_OK,"contour info");
    check(info.triangle_count==512,"only outer, inner, bottom and top faces");
    uint32_t cap=0;check(sk_mesh_copy_positions(mesh,nullptr,&cap)==SK_ERROR_LIMIT,"positions sizing");
    std::vector<uint8_t> bytes(cap);check(sk_mesh_copy_positions(mesh,bytes.data(),&cap)==SK_OK,"positions");
    for(size_t i=0;i<bytes.size();i+=4) { float value;std::memcpy(&value,&bytes[i],4);check(std::isfinite(value),"finite mesh"); }
    uint32_t indexBytes=0;
    check(sk_mesh_copy_indices(mesh,nullptr,&indexBytes)==SK_ERROR_LIMIT,"index sizing");
    std::vector<uint8_t> indices(indexBytes);
    check(sk_mesh_copy_indices(mesh,indices.data(),&indexBytes)==SK_OK,"mesh indices");
    auto vertex=[&](uint32_t index,int axis) { float value;std::memcpy(&value,&bytes[12*index+4*axis],4);return double(value); };
    double signedVolume=0;
    for(size_t k=0;k<indices.size();k+=12) {
        uint32_t idx[3];std::memcpy(idx,&indices[k],12);
        double a[3],b[3],c[3];
        for(int axis=0;axis<3;++axis) {a[axis]=vertex(idx[0],axis);b[axis]=vertex(idx[1],axis);c[axis]=vertex(idx[2],axis);}
        signedVolume+=(a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6;
    }
    near(signedVolume,32*std::sin(tau/32)/2*(9-4)*2,1e-5,"outward winding and chordal annulus volume");
    sk_mesh_destroy(mesh);
    auto c=cut(0,0.4,0.7,3,5,0,1);check(sk_ring_stock_cut_region(s,&c,1)==SK_OK,"nonuniform contour cut");
    check(sk_ring_stock_contour(s,pi/16,&mesh)==SK_OK,"nonuniform contour");
    check(sk_mesh_get_info(mesh,&info)==SK_OK&&info.triangle_count>0,"cut mesh exists");
    sk_mesh_destroy(mesh);sk_ring_stock_destroy(s);
}
void budget() {
    // Reference flexspline: m=.3, z=200, tooth-root wall=.6, face=8.
    auto g=grid(29.025,30.3,255,80);g.z_min=0;g.z_max=8;auto s=create(g);
    std::vector<sk_ring_cut> cuts;
    for(uint32_t r=0;r<g.radial_count;++r) {
        double radius=g.radius_min+(r+0.5)*(g.radius_max-g.radius_min)/g.radial_count;
        if(radius<=29.625)continue; // continuous wall below the roots
        for(uint32_t tooth=0;tooth<200;++tooth)cuts.push_back(cut(r,tau*tooth/200,pi/200,tooth%12,1,0,80));
    }
    check(sk_ring_stock_set_threads(s,4)==SK_OK,"reference workers");
    check(sk_ring_stock_cut_region(s,cuts.data(),uint32_t(cuts.size()))==SK_OK,"reference interval occupancy");
    sk_ring_info info{};check(sk_ring_stock_get_info(s,&info)==SK_OK,"reference memory");
    check(info.bytes<100ull*1024*1024,"reference stock below 100 MiB");
    check(info.interval_count>2000000,"reference benchmark includes 200-tooth material intervals");
    double worst=0;sk_ring_snapshot_handle shot{};
    for(int repeat=0;repeat<30;++repeat) {
        auto start=std::chrono::steady_clock::now();
        check(sk_ring_stock_snapshot(s,&shot)==SK_OK,"reference snapshot");
        worst=std::max(worst,std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count());
        sk_ring_snapshot_destroy(shot);
    }
    check(worst<0.5,"full-part snapshot below 0.5 s");
    check(sk_ring_stock_snapshot(s,&shot)==SK_OK,"retained reference snapshot");
    auto saved=read(s,254,0);auto c=cut(254,0,tau,0,3,0,1);
    check(sk_ring_stock_cut_region(s,&c,1)==SK_OK,"copy-on-write reference cut");
    check(sk_ring_stock_restore(s,shot)==SK_OK,"reference restore");same(saved,read(s,254,0));
    std::printf("reference: circles=%u intervals=%llu bytes=%llu snapshot_worst_s=%.9f\n",info.circle_count,
        (unsigned long long)info.interval_count,(unsigned long long)info.bytes,worst);
    sk_ring_snapshot_destroy(shot);sk_ring_stock_destroy(s);
}
}
int main() {
    try { slab();operations();deterministic();contour();budget(); }
    catch(const std::exception &e) { std::printf("FAIL after %d checks: %s\n",checks,e.what());return 1; }
    std::printf("Ring core: %d checks passed\n",checks);return 0;
}
