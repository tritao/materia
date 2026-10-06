#include "../../native/src/offset_wrist_polynomial.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <limits>
int main(int argc,char **argv) {
    std::array<long double,9> r={2.L/15,-2.L/3,11.L/15,14.L/15,1.L/3,2.L/15,-1.L/3,2.L/3,2.L/3};
    std::array<long double,3> p={.6L,.2L,.7L};
    std::array<long double,8> d={.17L,-.09L,.08L,.4L,.6L,.5L,.12L,.035L};
    if(argc==2 && std::string(argv[1])=="--stdin") {
        for(auto *values:{r.data(),p.data(),d.data()}) {
            const int count=values==r.data() ? 9 : values==p.data() ? 3 : 8;
            for(int i=0;i<count;++i) if(!(std::cin>>values[i])) return 2;
        }
    }
    if(argc==2 && std::string(argv[1])=="--invalid") {
        int rejected=0;
        auto check=[&](auto rr,auto pp,auto dd) {
            try { motionkit_offset::constraints(rr,pp,dd); }
            catch(const std::invalid_argument &) { ++rejected; return; }
            throw std::runtime_error("Invalid input accepted");
        };
        auto rr=r;rr[0]=std::numeric_limits<long double>::quiet_NaN();check(rr,p,d);
        auto pp=p;pp[1]=std::numeric_limits<long double>::infinity();check(r,pp,d);
        auto dd=d;dd[7]=std::numeric_limits<long double>::quiet_NaN();check(r,p,dd);
        for(int index:{4,5,6}) {dd=d;dd[index]=-1;check(r,p,dd);}
        rr=r;rr[0]+=0.01L;check(rr,p,d);
        rr=r;for(int i=0;i<3;++i) rr[i]=-rr[i];check(rr,p,d);
        std::cout<<rejected<<" invalid inputs rejected\n";return 0;
    }
    std::cout<<std::setprecision(24);
    for(int base=0;base<2;++base)for(int wrist=0;wrist<2;++wrist){
        auto result=motionkit_offset::constraints(r,p,d,base,wrist);
        for(const auto *poly:{&result.lateral,&result.length})
            for(int i=0;i<9;++i)for(int j=0;j<9;++j)
                std::cout<<base<<' '<<wrist<<' '<<(poly==&result.length)<<' '<<i<<' '<<j<<' '<<poly->coefficients[i][j]<<'\n';
    }
}
