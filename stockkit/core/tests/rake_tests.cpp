#include "stockkit.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>
namespace {
constexpr double tau=6.28318530717958647692;
int checks=0;
void check(bool b,const char *message){++checks;if(!b)throw std::runtime_error(message);}
void near(double a,double b,double t,const char *message){check(std::isfinite(a)&&std::abs(a-b)<=t,message);}
sk_ring_stock_handle stock(double ri=1,double ro=3) {
    sk_ring_grid g{sizeof(sk_ring_grid),8,2,ri,ro,-0.5,0.5};sk_ring_stock_handle out{};
    check(sk_ring_stock_create(&g,&out)==0,"stock create");return out;
}
sk_rake_pose pose(double z) {
    sk_rake_pose out{};out.struct_size=sizeof(out);out.rotation[0]=out.rotation[4]=out.rotation[8]=1;out.translation[2]=z;return out;
}
sk_rake_tool_handle tool(const std::vector<double>&vertices) {
    sk_rake_region regions[]={{sizeof(sk_rake_region),0,uint32_t(vertices.size()/3),0},{sizeof(sk_rake_region),0,uint32_t(vertices.size()/3),1}};
    sk_rake_tool_handle out{};check(sk_rake_tool_create(vertices.data(),uint32_t(vertices.size()),regions,2,2,&out)==0,"rake tool create");return out;
}
std::vector<sk_ring_interval> read(sk_ring_stock_handle s,uint32_t r,uint32_t z) {
    uint32_t count=0;auto status=sk_ring_stock_read_circle(s,r,z,nullptr,&count);
    check(status==0||status==SK_ERROR_LIMIT,"circle count");std::vector<sk_ring_interval> out(count);
    check(sk_ring_stock_read_circle(s,r,z,out.data(),&count)==0,"read circle");return out;
}
void analytic() {
    auto s=stock(),truth=stock();auto t=tool({0.3,-4,0,1.2,-4,0,1.2,4,0,0.3,4,0});
    sk_rake_pose poses[]={pose(-1),pose(1)};
    check(sk_ring_stock_sweep_rake(s,t,0,7,poses,2)==0,"translated rake sweep");
    check(sk_ring_stock_cut_slab(truth,0,0.3,1.2,0,7)==0,"slab oracle");
    for(uint32_t r=0;r<8;++r)for(uint32_t z=0;z<2;++z) {
        auto a=read(s,r,z),b=read(truth,r,z);check(a.size()==b.size(),"slab topology");
        for(size_t k=0;k<a.size();++k) {
            near(a[k].lo,b[k].lo,1e-13,"slab lower endpoint");near(a[k].hi,b[k].hi,1e-13,"slab upper endpoint");
            check(a[k].lo_pass==b[k].lo_pass&&a[k].hi_pass==b[k].hi_pass,"slab provenance");
        }
    }
    auto invalid=poses[0];invalid.rotation[0]=2;
    sk_rake_pose bad[]={poses[0],invalid};
    check(sk_ring_stock_sweep_rake(s,t,0,7,bad,2)==SK_ERROR_INVALID_ARGUMENT,"non-rigid pose rejected");
    sk_rake_tool_destroy(t);check(sk_ring_stock_sweep_rake(s,t,0,7,poses,2)==SK_ERROR_INVALID_HANDLE,"stale tool rejected");
    sk_ring_stock_destroy(s);sk_ring_stock_destroy(truth);
}
void concave() {
    auto s=stock(0.5,1.5);auto t=tool({-2,-2,0,2,-2,0,2,0,0,0,0,0,0,2,0,-2,2,0});
    sk_rake_pose still[]={pose(0),pose(0)};
    check(sk_ring_stock_sweep_rake(s,t,0,2,still,2)==0,"static face");
    near(read(s,0,0)[0].hi,tau,0,"static face removes no material");
    sk_rake_pose poses[]={pose(-1),pose(1)};
    check(sk_ring_stock_sweep_rake(s,t,0,2,poses,2)==0,"concave polygon sweep");
    for(uint32_t r=0;r<8;++r)for(uint32_t z=0;z<2;++z) {
        auto a=read(s,r,z);if(a.size()!=1){for(auto i:a)std::printf("notch r=%u z=%u %.17g %.17g\n",r,z,i.lo,i.hi);}check(a.size()==1,"concave region does not fill its notch");
        near(a[0].lo,0,1e-13,"notch start");near(a[0].hi,tau/4,1e-13,"notch end");
    }
    sk_rake_tool_destroy(t);sk_ring_stock_destroy(s);
}
void rotational_convergence() {
    auto toolHandle=tool({1,0,-1,3,0,-1,3,0,1,1,0,1});
    double errors[2]={};
    for(int resolution=0;resolution<2;++resolution) {
        uint32_t steps=resolution==0?4:64;
        sk_ring_grid g{sizeof(sk_ring_grid),4,1,2.98,3.02,-0.5,0.5};sk_ring_stock_handle s{};
        check(sk_ring_stock_create(&g,&s)==0,"rotating stock");
        std::vector<sk_rake_pose> poses;
        for(uint32_t k=0;k<=steps;++k) {
            double angle=tau/4*k/steps;auto p=pose(0);
            p.rotation[0]=p.rotation[4]=std::cos(angle);p.rotation[1]=-std::sin(angle);p.rotation[3]=std::sin(angle);
            poses.push_back(p);
        }
        check(sk_ring_stock_sweep_rake(s,toolHandle,0,1,poses.data(),uint32_t(poses.size()))==0,"rotating face sweep");
        for(uint32_t r=0;r<4;++r) {
            double retained=0;for(auto arc:read(s,r,0))retained+=arc.hi-arc.lo;
            double radius=g.radius_min+(r+0.5)*(g.radius_max-g.radius_min)/g.radial_count;
            double expected=radius<3?tau/4:0;
            errors[resolution]+=std::abs(tau-retained-expected)*radius;
        }
        sk_ring_stock_destroy(s);
    }
    check(errors[0]>0.01,"coarse curved path has chord error");
    check(errors[1]<1e-10,"refined sweep matches analytic rotating sector");
    sk_rake_tool_destroy(toolHandle);
}

}
int main() {try{analytic();concave();rotational_convergence();}catch(const std::exception&e){std::printf("FAIL after %d checks: %s\n",checks,e.what());return 1;}
std::printf("Rake sweep: %d checks passed\n",checks);return 0;}
