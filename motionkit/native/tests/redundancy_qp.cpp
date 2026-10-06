#ifdef NDEBUG
#undef NDEBUG
#endif
#include "motionkit.h"
#include <cassert>
#include <cmath>
#include <vector>

int main() {
    constexpr unsigned n=51;
    std::vector<mk_refinement_sample> s(n);
    std::vector<mk_refinement_solution> q(n),repeat(n);
    mk_refinement_limits limits{};limits.struct_size=sizeof(limits);limits.coordinate_count=1;
    limits.max_velocity[0]=.02;limits.max_acceleration[0]=.0001;
    limits.route_weight=1;limits.seed_weight=1;limits.curvature_weight=.1;
    for(unsigned i=0;i<n;++i) {
        auto &k=s[i];k.struct_size=sizeof(k);k.s=2.6*i/(n-1);k.feed=.01146;
        k.route[0]=k.s+.02*std::sin(20*k.s);k.seed[0]=k.s;
        k.lower[0]=-.2;k.upper[0]=2.8;
    }
    std::vector<mk_refinement_bound> bounds(2);
    for(auto &pin:bounds){pin.struct_size=sizeof(pin);pin.coefficients[0]=1;}
    bounds[0].sample=0;bounds[0].lower=bounds[0].upper=0;
    bounds[1].sample=n-1;bounds[1].lower=bounds[1].upper=2.6;
    assert(mk_refine_redundancy(&limits,s.data(),n,bounds.data(),2,q.data())==MK_OK);
    assert(mk_refine_redundancy(&limits,s.data(),n,bounds.data(),2,repeat.data())==MK_OK);
    for(unsigned i=0;i<n;++i) {
        assert(q[i].value[0]==repeat[i].value[0]);assert(q[i].first[0]==repeat[i].first[0]);
        assert(std::abs(q[i].first[0])*.01146<=.02+1e-8);
        assert(std::abs(q[i].second[0])*.01146*.01146<=.0001+1e-10);
        assert(std::abs(q[i].value[0]-s[i].s)<.004);
    }
    for(unsigned i=0;i+1<n;++i)for(unsigned k=0;k<=100;++k) {
        const double h=s[i+1].s-s[i].s,t=k/100.,delta=q[i+1].value[0]-q[i].value[0];
        const double b=q[i].first[0],c=3*delta/(h*h)-(2*b+q[i+1].first[0])/h;
        const double d=-2*delta/(h*h*h)+(b+q[i+1].first[0])/(h*h),x=t*h;
        assert(std::abs(b+x*(2*c+3*x*d))*.01146<=.02+1e-8);
        assert(std::abs(2*c+6*x*d)*.01146*.01146<=.0001+1e-10);
        if(i+2<n && k==100)assert(std::abs(2*c+6*x*d-q[i+1].second[0])<1e-6);
    }
    // A pinned displacement cannot be supplied at an impossibly low drive speed.
    limits.max_velocity[0]=.005;
    assert(mk_refine_redundancy(&limits,s.data(),n,bounds.data(),2,q.data())==MK_ERROR_GENERATION);
    limits.max_velocity[0]=.02;
    mk_refinement_bound arm{};arm.struct_size=sizeof(arm);arm.sample=n/2;arm.coefficients[0]=1;
    arm.lower=1.29;arm.upper=1.31;
    bounds.push_back(arm);
    assert(mk_refine_redundancy(&limits,s.data(),n,bounds.data(),3,q.data())==MK_OK);
    assert(q[n/2].value[0]>=1.29-1e-7 && q[n/2].value[0]<=1.31+1e-7);
    arm.lower=3;arm.upper=4;
    bounds[2]=arm;
    assert(mk_refine_redundancy(&limits,s.data(),n,bounds.data(),3,q.data())==MK_ERROR_GENERATION);
    s[1].s=s[0].s;
    assert(mk_refine_redundancy(&limits,s.data(),n,bounds.data(),2,q.data())==MK_ERROR_INVALID_ARGUMENT);
}
