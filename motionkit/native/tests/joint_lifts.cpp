#include "motionkit.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>
int main() {
    const double tau=2*acos(-1.0);
    mk_joint_lift_request r={};r.struct_size=sizeof(r);r.joint_count=3;
    r.periodic[1]=r.periodic[2]=1;r.lower[0]=-.5;r.upper[0]=.5;
    r.lower[1]=r.lower[2]=-2*tau;r.upper[1]=r.upper[2]=2*tau;
    double q[]={.2,0,0};uint32_t n=0;
    assert(mk_joint_lift_count(&r,q,3,&n)==MK_OK && n==25);
    std::vector<mk_joint_lift> lifts(n);uint32_t written=0;
    assert(mk_enumerate_joint_lifts(&r,q,3,lifts.data(),n,&written)==MK_OK && written==n);
    for(unsigned i=0;i<n;++i) {
        assert(lifts[i].wraps[0]==0 && lifts[i].joints[0]==q[0]);
        assert(lifts[i].wraps[1]==int(i/5)-2 && lifts[i].wraps[2]==int(i%5)-2);
        for(unsigned j=0;j<3;++j) {
            assert(lifts[i].joints[j]>=r.lower[j]-1e-9 && lifts[i].joints[j]<=r.upper[j]+1e-9);
            assert(std::abs(lifts[i].joints[j]-q[j]-tau*lifts[i].wraps[j])<1e-12);
        }
    }
    assert(mk_enumerate_joint_lifts(&r,q,3,lifts.data(),n-1,&written)==MK_ERROR_INVALID_ARGUMENT);
    q[0]=.6;assert(mk_joint_lift_count(&r,q,3,&n)==MK_OK && n==0);
    assert(mk_enumerate_joint_lifts(&r,q,3,nullptr,0,&written)==MK_OK && written==0);
    q[0]=.2;q[1]=.3;assert(mk_joint_lift_count(&r,q,3,&n)==MK_OK && n==20);
    r.lower[1]=r.upper[1]=.3;assert(mk_joint_lift_count(&r,q,3,&n)==MK_OK && n==5);
    r.lower[1]=r.upper[1]=.4;assert(mk_joint_lift_count(&r,q,3,&n)==MK_OK && n==0);
    r.lower[1]=-100*tau;r.upper[1]=100*tau;r.lower[2]=r.upper[2]=0;
    assert(mk_joint_lift_count(&r,q,3,&n)==MK_OK && n==200); // no artificial 64-turn truncation
    r.lower[1]=r.lower[2]=-1e6*tau;r.upper[1]=r.upper[2]=1e6*tau;
    assert(mk_joint_lift_count(&r,q,3,&n)==MK_ERROR_INVALID_ARGUMENT);
    r.upper[1]=std::numeric_limits<double>::infinity();assert(mk_joint_lift_count(&r,q,3,&n)==MK_ERROR_INVALID_ARGUMENT);
}
