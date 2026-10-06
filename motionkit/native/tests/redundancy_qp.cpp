#ifdef NDEBUG
#undef NDEBUG
#endif
#include "motionkit.h"
#include "redundancy_qp_fixtures.h"
#include <cassert>
#include <cmath>
#include <vector>
#include <iterator>

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
    // Actual roll-only weldment input, with nonuniform stopped-section knots.
    // The original 500-iteration budget rejected this feasible problem.
    std::vector<mk_refinement_sample> captured(std::size(weldment_samples));
    std::vector<mk_refinement_bound> capturedBounds(std::size(weldment_bounds));
    for(size_t i=0;i<captured.size();++i){auto &k=captured[i];k={};k.struct_size=sizeof(k);
        k.s=weldment_samples[i][0];k.feed=weldment_samples[i][1];
        k.route[0]=weldment_samples[i][2];k.seed[0]=weldment_samples[i][3];
        k.gradient[0]=weldment_samples[i][4];k.lower[0]=-1e6;k.upper[0]=1e6;
    }
    for(size_t i=0;i<capturedBounds.size();++i){auto &b=capturedBounds[i];b={};b.struct_size=sizeof(b);
        b.sample=static_cast<uint32_t>(weldment_bounds[i][0]);b.derivative=static_cast<uint32_t>(weldment_bounds[i][1]);
        b.coefficients[0]=weldment_bounds[i][2];b.lower=weldment_bounds[i][3];b.upper=weldment_bounds[i][4];
    }
    limits.max_velocity[0]=1e6;limits.max_acceleration[0]=8e5;
    limits.route_weight=1;limits.seed_weight=10;limits.curvature_weight=.01;
    std::vector<mk_refinement_solution> checked(captured.size());
    assert(mk_refine_redundancy(&limits,captured.data(),static_cast<uint32_t>(captured.size()),
        capturedBounds.data(),static_cast<uint32_t>(capturedBounds.size()),checked.data())==MK_OK);
    for(const auto &b:capturedBounds){const auto &v=checked[b.sample];
        const double value=b.coefficients[0]*(b.derivative==0?v.value[0]:b.derivative==1?v.first[0]:v.second[0]);
        assert(value>=b.lower-1e-7 && value<=b.upper+1e-7);
    }

}
