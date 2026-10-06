/*********************************************************************
 *
 * Provides forward and inverse kinematics for Univeral robot designs
 * Author: Kelsey Hawkins (kphawkins@gatech.edu)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Georgia Institute of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// Adapted from ROS-Industrial universal_robot/ur_kinematics, noetic-devel.
// Runtime geometry and standard DH frames replace upstream robot-size macros.
// Original inverse by Kelsey Hawkins; license retained above.
#include "motionkit.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace {
constexpr double ZERO_THRESH = 1e-12;
const double PI = std::acos(-1.0);
int SIGN(double x) { return (x > 0) - (x < 0); }
int inverse(const mk_ur_parameters &p, const Eigen::Isometry3d &t, double *q_sols,
            unsigned *branches, double q6_des) {
 const double a2=p.a2, a3=p.a3, d1=p.d1, d4=p.d4, d5=p.d5, d6=p.d6;
 const double T00=t(0,0), T01=t(0,1), T02=t(0,2), T03=t(0,3);
 const double T10=t(1,0), T11=t(1,1), T12=t(1,2), T13=t(1,3);
 const double T20=t(2,0), T21=t(2,1), T22=t(2,2), T23=t(2,3);

    int num_sols = 0;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
      double A = d6*T12 - T13;
      double B = d6*T02 - T03;
      double R = A*A + B*B;
      if(fabs(A) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
          div = SIGN(d4)*SIGN(A);
        else
          div = d4/A;
        double arccos = acos(div);
        q1[0] = arccos;
        q1[1] = 2.0*PI - arccos;
      }
      else if(d4*d4 > R) {
        return num_sols;
      }
      else {
        double arccos = acos(d4 / sqrt(R)) ;
        double arctan = atan2(-B, A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if(fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if(fabs(neg) < ZERO_THRESH)
          neg = 0.0;
        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*PI + pos;
        if(neg >= 0.0)
          q1[1] = neg; 
        else
          q1[1] = 2.0*PI + neg;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
      for(int i=0;i<2;i++) {
        double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
        double div;
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = acos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0*PI - arccos;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
      for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
          double c1 = cos(q1[i]), s1 = sin(q1[i]);
          double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(fabs(s5) < ZERO_THRESH)
            q6 = q6_des;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                       SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < ZERO_THRESH)
              q6 = 0.0;
            if(q6 < 0.0)
              q6 += 2.0*PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double q2[2], q3[2], q4[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = cos(q6), s6 = sin(q6);
          double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + 
                        T03*c1 + T13*s1;
          double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

          double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
          if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // TODO NO SOLUTION
            continue;
          }
          double arccos = acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0*PI - arccos;
          double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
          double s3 = sin(arccos);
          double A = (a2 + a3*c3), B = a3*s3;
          q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
          q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
          double c23_0 = cos(q2[0]+q3[0]);
          double s23_0 = sin(q2[0]+q3[0]);
          double c23_1 = cos(q2[1]+q3[1]);
          double s23_1 = sin(q2[1]+q3[1]);
          q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
          ////////////////////////////////////////////////////////////////////////////////
          for(int k=0;k<2;k++) {
            if(fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < ZERO_THRESH)
              q4[k] = 0.0;
            else if(q4[k] < 0.0) q4[k] += 2.0*PI;
            branches[num_sols] = i*4 + j*2 + k;
            q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k]; 
            q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k]; 
            q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6; 
            num_sols++;
          }

        }
      }
    }
    return num_sols;
  
}
bool valid(const mk_ur_parameters *p) {
    if (!p || p->struct_size != sizeof(*p)) return false;
    const double dimensions[] = {p->a2,p->a3,p->d1,p->d4,p->d5,p->d6};
    for (double d : dimensions) if (!std::isfinite(d)) return false;
    if (std::abs(p->a2) < 1e-10 || std::abs(p->a3) < 1e-10 || std::abs(p->d6) < 1e-10) return false;
    for (unsigned i=0;i<6;++i)
        if (!std::isfinite(p->offsets[i]) || (p->sign_corrections[i]!=1 && p->sign_corrections[i]!=-1)) return false;
    return true;
}
Eigen::Isometry3d forward(const mk_ur_parameters &p, const double *q) {
    const double a[] = {0,p.a2,p.a3,0,0,0};
    const double d[] = {p.d1,0,0,p.d4,p.d5,p.d6};
    const double alpha[] = {PI/2,0,0,PI/2,-PI/2,0};
    Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
    for (unsigned i=0;i<6;++i) {
        const double angle=p.sign_corrections[i]*q[i]-p.offsets[i];
        const double c=cos(angle),s=sin(angle),ca=cos(alpha[i]),sa=sin(alpha[i]);
        Eigen::Isometry3d step=Eigen::Isometry3d::Identity();
        step.matrix() << c,-s*ca,s*sa,a[i]*c, s,c*ca,-c*sa,a[i]*s, 0,sa,ca,d[i], 0,0,0,1;
        t=t*step;
    }
    return t;
}
}
extern "C" mk_result MK_CALL mk_analytic_ur_forward(const mk_ur_parameters *p,
    const double *q,uint32_t n,mk_opw_pose *out) {
    if (!valid(p) || !q || n!=6 || !out) return MK_ERROR_INVALID_ARGUMENT;
    for (unsigned i=0;i<6;++i) if (!std::isfinite(q[i])) return MK_ERROR_INVALID_ARGUMENT;
    const auto t=forward(*p,q); const Eigen::Quaterniond r(t.linear());
    *out={}; out->struct_size=sizeof(*out);
    for(unsigned i=0;i<3;++i) out->position[i]=t.translation()[i];
    out->quaternion[0]=r.x(); out->quaternion[1]=r.y(); out->quaternion[2]=r.z(); out->quaternion[3]=r.w();
    return MK_OK;
}
extern "C" mk_result MK_CALL mk_analytic_ur_inverse(const mk_ur_parameters *p,
    const mk_opw_pose *target,double seed,mk_analytic_solution *out,uint32_t capacity,uint32_t *count) {
    if (!valid(p) || !target || target->struct_size!=sizeof(*target) || !out || capacity<8 || !count || !std::isfinite(seed))
        return MK_ERROR_INVALID_ARGUMENT;
    *count=0;
    for(double v:target->position) if(!std::isfinite(v)) return MK_ERROR_INVALID_ARGUMENT;
    for(double v:target->quaternion) if(!std::isfinite(v)) return MK_ERROR_INVALID_ARGUMENT;
    Eigen::Quaterniond r(target->quaternion[3],target->quaternion[0],target->quaternion[1],target->quaternion[2]);
    if(std::abs(r.norm()-1)>1e-8) return MK_ERROR_INVALID_ARGUMENT;
    Eigen::Isometry3d t=Eigen::Isometry3d::Identity(); t.linear()=r.toRotationMatrix();
    for(unsigned i=0;i<3;++i) t.translation()[i]=target->position[i];
    double raw[48]; unsigned ids[8];
    const int n=inverse(*p,t,raw,ids,p->sign_corrections[5]*seed-p->offsets[5]);
    for(int i=0;i<n;++i) {
        mk_analytic_solution solution={}; solution.struct_size=sizeof(solution); solution.branch=ids[i];
        bool finite=true;
        for(unsigned j=0;j<6;++j) { solution.joints[j]=(raw[6*i+j]+p->offsets[j])*p->sign_corrections[j]; finite &= std::isfinite(solution.joints[j]); }
        if(!finite) continue;
        const auto check=forward(*p,solution.joints);
        if((check.matrix()-t.matrix()).norm()>1e-6) continue;
        solution.singular=(std::abs(sin(raw[6*i+4]))<1e-7?1u:0u) | (std::abs(sin(raw[6*i+2]))<1e-7?2u:0u);
        out[(*count)++]=solution;
    }
    return MK_OK;
}
