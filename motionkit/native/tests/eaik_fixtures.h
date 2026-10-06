#pragma once
#include "motionkit.h"
#include <Eigen/Geometry>
#include <array>

namespace eaik_fixtures {
using H=Eigen::Matrix<double,3,6>;
using P=Eigen::Matrix<double,3,7>;
inline mk_eaik_model spherical(const std::array<double,7> &d,
    const std::array<double,6> &offsets={},const std::array<int,6> &signs={1,1,1,1,1,1}) {
    H h;h<<0,0,0,0,0,0, 0,1,1,0,1,0, 1,0,0,1,0,1;
    P p;p<<0,d[0],0,d[1],0,0,0, 0,d[2],0,0,0,0,0, 0,d[3],d[4],d[5],0,0,d[6];
    mk_eaik_model out{};out.struct_size=sizeof(out);
    Eigen::Matrix3d r=Eigen::Matrix3d::Identity();
    Eigen::Map<P> converted_p(out.displacements);converted_p.col(0)=p.col(0);
    Eigen::Map<H> converted_h(out.axes);
    for(unsigned j=0;j<6;++j){
        converted_h.col(j)=r*h.col(j)*signs[j];
        r=r*Eigen::AngleAxisd(-offsets[j],h.col(j)).toRotationMatrix();
        converted_p.col(j+1)=r*p.col(j+1);
    }
    Eigen::Map<Eigen::Matrix3d> terminal(out.terminal_rotation);terminal=r;
    return out;
}
inline mk_eaik_model parallel(const std::array<double,6> &d) {
    H h;h<<0,0,0,0,0,0, 0,-1,-1,-1,0,-1, 1,0,0,0,-1,0;
    P p;p<<0,0,d[0],d[1],0,0,0, 0,0,0,0,-d[3],0,-d[5], 0,d[2],0,0,0,-d[4],0;
    mk_eaik_model out{};out.struct_size=sizeof(out);
    Eigen::Map<H> mapped_h(out.axes);mapped_h=h;Eigen::Map<P> mapped_p(out.displacements);mapped_p=p;
    Eigen::Map<Eigen::Matrix3d> mapped_r(out.terminal_rotation);
    mapped_r=Eigen::AngleAxisd(std::acos(-1.0)/2,Eigen::Vector3d::UnitX()).toRotationMatrix();
    return out;
}
inline mk_eaik_configuration labels(bool spherical,double shoulder=0,double elbow=0) {
    mk_eaik_configuration out{};out.struct_size=sizeof(out);out.spherical=spherical;
    for(auto &sign:out.signs)sign=1;
    out.base_rotation[0]=out.base_rotation[4]=out.base_rotation[8]=1;
    out.shoulder_offset=shoulder;out.elbow_offset=elbow;return out;
}
inline Eigen::Matrix4d forward(const mk_eaik_model &model,const double *q) {
    const Eigen::Map<const H> h(model.axes);const Eigen::Map<const P> p(model.displacements);
    Eigen::Matrix4d t=Eigen::Matrix4d::Identity();Eigen::Matrix3d r=Eigen::Matrix3d::Identity();Eigen::Vector3d x=p.col(0);
    for(unsigned j=0;j<6;++j){r=r*Eigen::AngleAxisd(q[j],h.col(j)).toRotationMatrix();x+=r*p.col(j+1);}
    t.block<3,3>(0,0)=r*Eigen::Map<const Eigen::Matrix3d>(model.terminal_rotation);t.block<3,1>(0,3)=x;return t;
}
inline mk_analytic_pose pose(const Eigen::Matrix4d &t) {
    mk_analytic_pose out{};out.struct_size=sizeof(out);Eigen::Quaterniond r(t.block<3,3>(0,0));
    for(unsigned j=0;j<3;++j)out.position[j]=t(j,3);
    out.quaternion[0]=r.x();out.quaternion[1]=r.y();out.quaternion[2]=r.z();out.quaternion[3]=r.w();return out;
}
inline std::array<mk_eaik_model,4> published() {
    const double pi=std::acos(-1.0);
    // Preserve the four historical numerical fixtures, including the assumed
    // R2000/TX40 joint conventions; these are not manufacturer certification.
    return {spherical({.100,-.135,0,.615,.705,.755,.085},{0,0,-pi/2,0,0,0}),
        spherical({.025,-.035,0,.400,.315,.365,.080},{0,-pi/2,0,0,0,0},{-1,1,1,-1,1,-1}),
        spherical({.720,-.225,0,.600,1.075,1.280,.235},{0,0,-pi/2,0,0,0}),
        spherical({0,0,.035,.320,.225,.225,.065},{0,0,-pi/2,0,0,0})};
}
}
