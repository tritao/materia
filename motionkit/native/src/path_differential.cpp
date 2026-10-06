#include "motionkit.h"
#include <Eigen/QR>
#include <array>
#include <cmath>
#include <new>
extern "C" mk_result MK_CALL mk_path_differential(const mk_differential_request *r,
    const double *j,const double *dj,uint32_t size,uint32_t n,
    double *first,double *second,mk_differential_report *report) {
    if(!r || r->struct_size!=sizeof(*r) || !j || !dj || !first || !second || !report ||
        n==0 || n>MK_MAX_JOINTS || n!=r->joint_count || size!=6*n ||
        !std::isfinite(r->rank_tolerance) || r->rank_tolerance<=0 || r->rank_tolerance>=1 ||
        !std::isfinite(r->linear_tolerance) || r->linear_tolerance<=0 ||
        !std::isfinite(r->angular_tolerance) || r->angular_tolerance<=0)return MK_ERROR_INVALID_ARGUMENT;
    *report={};report->struct_size=sizeof(*report);
    std::array<unsigned,MK_MAX_JOINTS> unknown{};unsigned count=0;
    for(unsigned k=0;k<n;++k){
        if(r->known[k]>1 || !std::isfinite(r->known_first[k]) || !std::isfinite(r->known_second[k]))return MK_ERROR_INVALID_ARGUMENT;
        if(!r->known[k])unknown[count++]=k;
    }
    if(count>6)return MK_ERROR_UNSUPPORTED;
    for(unsigned k=0;k<size;++k)if(!std::isfinite(j[k]) || !std::isfinite(dj[k]))return MK_ERROR_INVALID_ARGUMENT;
    for(unsigned k=0;k<6;++k)if(!std::isfinite(r->task_velocity[k]) || !std::isfinite(r->task_acceleration[k]))return MK_ERROR_INVALID_ARGUMENT;
    try {
        Eigen::MatrixXd matrix(6,count);Eigen::VectorXd velocity(6),acceleration(6);
        for(unsigned row=0;row<6;++row){velocity[row]=r->task_velocity[row];acceleration[row]=r->task_acceleration[row];
            for(unsigned k=0;k<n;++k)if(r->known[k]){velocity[row]-=j[row*n+k]*r->known_first[k];acceleration[row]-=j[row*n+k]*r->known_second[k];}
            for(unsigned k=0;k<count;++k)matrix(row,k)=j[row*n+unknown[k]];
        }
        for(unsigned k=0;k<n;++k){first[k]=r->known[k]?r->known_first[k]:0;second[k]=r->known[k]?r->known_second[k]:0;}
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr;qr.setThreshold(r->rank_tolerance);if(count)qr.compute(matrix);
        report->rank=count?qr.rank():0;report->singular=report->rank<count;
        if(report->singular)return MK_ERROR_GENERATION;
        if(count){Eigen::VectorXd solved=qr.solve(velocity);for(unsigned k=0;k<count;++k)first[unknown[k]]=solved[k];}
        for(unsigned row=0;row<6;++row)for(unsigned k=0;k<n;++k)acceleration[row]-=dj[row*n+k]*first[k];
        if(count){Eigen::VectorXd solved=qr.solve(acceleration);for(unsigned k=0;k<count;++k)second[unknown[k]]=solved[k];}
        bool accurate=true;
        for(unsigned row=0;row<6;++row){double v=-r->task_velocity[row],a=-r->task_acceleration[row];
            for(unsigned k=0;k<n;++k){v+=j[row*n+k]*first[k];a+=j[row*n+k]*second[k]+dj[row*n+k]*first[k];}
            report->velocity_error=std::max(report->velocity_error,std::abs(v));
            report->acceleration_error=std::max(report->acceleration_error,std::abs(a));
            double tolerance=row<3?r->linear_tolerance:r->angular_tolerance;
            if(!std::isfinite(v) || !std::isfinite(a) || std::abs(v)>tolerance || std::abs(a)>tolerance)accurate=false;
        }
        for(unsigned k=0;k<n;++k)if(!std::isfinite(first[k]) || !std::isfinite(second[k]))accurate=false;
        return accurate?MK_OK:MK_ERROR_GENERATION;
    }catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
}
