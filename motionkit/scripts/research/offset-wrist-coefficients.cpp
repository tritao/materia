#include "../../native/src/offset_wrist_polynomial.h"
#include <iostream>
#include <iomanip>
int main() {
    const std::array<long double,9> r={2.L/15,-2.L/3,11.L/15,14.L/15,1.L/3,2.L/15,-1.L/3,2.L/3,2.L/3};
    const std::array<long double,3> p={.6L,.2L,.7L};
    const std::array<long double,8> d={.17L,-.09L,.08L,.4L,.6L,.5L,.12L,.035L};
    std::cout<<std::setprecision(24);
    for(int base=0;base<2;++base)for(int wrist=0;wrist<2;++wrist){
        auto result=motionkit_offset::constraints(r,p,d,base,wrist);
        for(const auto *poly:{&result.lateral,&result.length})
            for(int i=0;i<9;++i)for(int j=0;j<9;++j)
                std::cout<<base<<' '<<wrist<<' '<<(poly==&result.length)<<' '<<i<<' '<<j<<' '<<poly->coefficients[i][j]<<'\n';
    }
}
