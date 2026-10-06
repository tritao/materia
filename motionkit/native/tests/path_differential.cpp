#ifdef NDEBUG
#undef NDEBUG
#endif
#include "motionkit.h"
#include <cassert>
#include <cmath>
int main(){
    mk_differential_request r{};r.struct_size=sizeof(r);r.joint_count=7;
    r.rank_tolerance=1e-10;r.linear_tolerance=r.angular_tolerance=1e-9;
    r.known[0]=1;r.known_first[0]=.3;r.known_second[0]=-.2;
    double j[42]{},dj[42]{},first[7],second[7];mk_differential_report report;
    // Moving upstream coordinate plus six independent task coordinates.
    for(unsigned row=0;row<6;++row){j[row*7]=.1*(row+1);dj[row*7]=.03;
        j[row*7+row+1]=1;dj[row*7+row+1]=.02*(row+1);
        double desired_first=.07*(row+1),desired_second=-.04*(row+1);
        r.task_velocity[row]=j[row*7]*r.known_first[0]+desired_first;
        r.task_acceleration[row]=j[row*7]*r.known_second[0]+desired_second+dj[row*7]*r.known_first[0]+dj[row*7+row+1]*desired_first;}
    assert(mk_path_differential(&r,j,dj,42,7,first,second,&report)==MK_OK);
    assert(report.rank==6 && !report.singular && first[0]==.3 && second[0]==-.2);
    for(unsigned k=1;k<7;++k){assert(std::abs(first[k]-.07*k)<1e-12);assert(std::abs(second[k]+.04*k)<1e-12);}
    j[5*7+6]=0;
    assert(mk_path_differential(&r,j,dj,42,7,first,second,&report)==MK_ERROR_GENERATION);
    assert(report.singular && report.rank==5);
    j[5*7+6]=1;r.known[0]=2;
    assert(mk_path_differential(&r,j,dj,42,7,first,second,&report)==MK_ERROR_INVALID_ARGUMENT);
    r.known[0]=1;j[0]=NAN;
    assert(mk_path_differential(&r,j,dj,42,7,first,second,&report)==MK_ERROR_INVALID_ARGUMENT);
    j[0]=.1;
    for(unsigned k=0;k<7;++k){r.known[k]=1;r.known_first[k]=k?.07*k:.3;r.known_second[k]=k?-.04*k:-.2;}
    // Rebuild a fully prescribed, compatible differential task.
    for(unsigned row=0;row<6;++row){r.task_velocity[row]=r.task_acceleration[row]=0;
        for(unsigned k=0;k<7;++k){r.task_velocity[row]+=j[row*7+k]*r.known_first[k];r.task_acceleration[row]+=j[row*7+k]*r.known_second[k]+dj[row*7+k]*r.known_first[k];}}
    assert(mk_path_differential(&r,j,dj,42,7,first,second,&report)==MK_OK && report.rank==0);
    r.task_velocity[0]+=.1;
    assert(mk_path_differential(&r,j,dj,42,7,first,second,&report)==MK_ERROR_GENERATION && !report.singular);

    mk_differential_request cart{};cart.struct_size=sizeof(cart);cart.joint_count=3;
    cart.rank_tolerance=1e-10;cart.linear_tolerance=cart.angular_tolerance=1e-9;
    double cart_j[18]{},cart_dj[18]{},cart_first[3],cart_second[3];
    for(unsigned k=0;k<3;++k){cart_j[k*3+k]=1;cart.task_velocity[k]=.2*(k+1);cart.task_acceleration[k]=-.1*(k+1);}
    assert(mk_path_differential(&cart,cart_j,cart_dj,18,3,cart_first,cart_second,&report)==MK_OK && report.rank==3);
    for(unsigned k=0;k<3;++k){assert(std::abs(cart_first[k]-cart.task_velocity[k])<1e-12);assert(std::abs(cart_second[k]-cart.task_acceleration[k])<1e-12);}
    cart.task_velocity[5]=.1;
    assert(mk_path_differential(&cart,cart_j,cart_dj,18,3,cart_first,cart_second,&report)==MK_ERROR_GENERATION && !report.singular);

}
