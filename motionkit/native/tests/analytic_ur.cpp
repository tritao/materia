#include "motionkit.h"
#include <cassert>
#include <cmath>
#include <random>
#include <iostream>
int main() {
    std::mt19937 random(712019); std::uniform_real_distribution<double> angle(-3.0,3.0);
    const double dimensions[][6]={{-.24365,-.21325,.1519,.11235,.08535,.0819},{-.425,-.39225,.089159,.10915,.09465,.0823},{-.612,-.5723,.1273,.163941,.1157,.0922},{-.8989,-.7149,.1807,.17415,.11985,.11655}};
    unsigned checked=0;
    for(auto &d:dimensions) {
        mk_ur_parameters p={}; p.struct_size=sizeof(p); p.a2=d[0];p.a3=d[1];p.d1=d[2];p.d4=d[3];p.d5=d[4];p.d6=d[5];
        for(unsigned j=0;j<6;++j) {p.sign_corrections[j]=j%2?-1:1;p.offsets[j]=.17*j;}
        for(unsigned sample=0;sample<400;++sample) {
            double q[6]; for(double &v:q) v=angle(random);
            if(sample<2) q[4]=(p.offsets[4]+sample*acos(-1.0))*p.sign_corrections[4];
            mk_opw_pose pose={}; assert(mk_analytic_ur_forward(&p,q,6,&pose)==MK_OK);
            mk_analytic_solution out[8]; uint32_t count=0;
            assert(mk_analytic_ur_inverse(&p,&pose,q[5],out,8,&count)==MK_OK); assert(count>0 && count<=8);
            bool found=false; unsigned seen=0;
            for(unsigned i=0;i<count;++i) {
                assert(out[i].branch<8); assert(!(seen&(1u<<out[i].branch)));seen|=1u<<out[i].branch;
                mk_opw_pose actual={};assert(mk_analytic_ur_forward(&p,out[i].joints,6,&actual)==MK_OK);
                double error=0;for(unsigned j=0;j<3;++j) error+=fabs(actual.position[j]-pose.position[j]);assert(error<1e-7);
                double dot=0;for(unsigned j=0;j<4;++j) dot+=actual.quaternion[j]*pose.quaternion[j];assert(fabs(fabs(dot)-1)<1e-8);
                bool same=true;for(unsigned j=0;j<6;++j) same &= fabs(remainder(q[j]-out[i].joints[j],2*acos(-1.0)))<1e-6;
                found|=same;
                if(sample<2 && same) assert(out[i].singular&1);
            }
            if(!found) { std::cerr<<"missing sample "<<sample<<" a2 "<<p.a2<<" q ";for(double v:q)std::cerr<<v<<" ";std::cerr<<" branches "<<count<<"\n"; } assert(found);++checked;
        }
        mk_opw_pose far={};far.struct_size=sizeof(far);far.position[0]=100;far.quaternion[3]=1;
        mk_analytic_solution out[8];uint32_t count=99;
        assert(mk_analytic_ur_inverse(&p,&far,0,out,8,&count)==MK_OK && count==0);
        p.sign_corrections[0]=0;assert(mk_analytic_ur_inverse(&p,&far,0,out,8,&count)==MK_ERROR_INVALID_ARGUMENT);
    }
    std::cout<<"UR analytic round trips: "<<checked<<"\n";
}
