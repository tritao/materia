#include "motionkit.h"
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <vector>
#include <set>
#include <tuple>
using T=Eigen::Isometry3d;
T pose(const double *p,const double *q) {
    T t=T::Identity();t.translation()=Eigen::Vector3d(p[0],p[1],p[2]);
    t.linear()=Eigen::Quaterniond(q[3],q[0],q[1],q[2]).toRotationMatrix();return t;
}
void store(const T &t,double *p,double *r) {
    Eigen::Quaterniond q(t.linear());for(unsigned i=0;i<3;++i)p[i]=t.translation()[i];
    r[0]=q.x();r[1]=q.y();r[2]=q.z();r[3]=q.w();
}
int main() {
    mk_ur_parameters p={};p.struct_size=sizeof(p);p.a2=-.425;p.a3=-.39225;p.d1=.089159;p.d4=.10915;p.d5=.09465;p.d6=.0823;
    for(auto &sign:p.sign_corrections)sign=1;
    mk_serial_cell_model model={};model.struct_size=sizeof(model);model.joint_count=8;model.external_count=2;model.arm_joint_count=6;
    for(unsigned i=0;i<6;++i)model.arm_joint_indices[i]=i+1;
    model.external_joint_indices[0]=0;model.external_joint_indices[1]=7;
    model.external_scopes[1]=1;model.external_kinds[1]=1;
    model.external_axes[0]=.8;model.external_axes[1]=.6;model.external_axes[5]=1;
    model.external_origins[3]=.2;model.external_origins[4]=-.2;model.external_origins[5]=.1;
    T base=T::Identity();base.translation()=Eigen::Vector3d(.1,.2,.4);
    base.linear()=(Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitY())).toRotationMatrix();
    T work=T::Identity();work.translation()=Eigen::Vector3d(.2,-.2,.1);work.linear()=Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T tool=T::Identity();tool.translation()=Eigen::Vector3d(.05,.01,.1);tool.linear()=Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitY()).toRotationMatrix();
    store(base,model.base_position,model.base_quaternion);store(work,model.work_position,model.work_quaternion);store(tool,model.tool_position,model.tool_quaternion);
    mk_external_lattice external={};external.struct_size=sizeof(external);external.joint_count=8;external.axis_count=2;
    external.joint_indices[0]=0;external.joint_indices[1]=7;external.point_counts[0]=3;external.point_counts[1]=2;
    external.lower[0]=-.03;external.upper[0]=.03;external.lower[1]=-.1;external.upper[1]=.1;
    mk_orientation_lattice orientation={};orientation.struct_size=sizeof(orientation);orientation.mode=1;orientation.roll_count=4;
    mk_joint_lift_request limits={};limits.struct_size=sizeof(limits);limits.joint_count=8;
    for(unsigned i=0;i<8;++i){limits.lower[i]=-3.2;limits.upper[i]=3.2;limits.periodic[i]=i>0;}
    double seed[]={0,.2,-1.1,.8,-.5,.7,.3,0};
    mk_opw_parameters opw={};opw.struct_size=sizeof(opw);opw.a1=.1;opw.a2=-.135;opw.c1=.615;opw.c2=.705;opw.c3=.755;opw.c4=.085;
    for(auto &sign:opw.sign_corrections)sign=1;
    for(unsigned shared=0;shared<2;++shared) {
    model.external_scopes[0]=shared ? 2 : 0;
    for(unsigned family=0;family<2;++family) {
    const auto forward=[&](const double *q,mk_opw_pose *out){return family==0 ? mk_analytic_ur_forward(&p,q,6,out) : mk_opw_forward(&opw,q,6,out);};
    mk_opw_pose arm;assert(forward(seed+1,&arm)==MK_OK);
    const T goal=work.inverse()*base*pose(arm.position,arm.quaternion)*tool;
    mk_opw_pose target={};target.struct_size=sizeof(target);store(goal,target.position,target.quaternion);
    const auto candidate_count=[&](uint32_t *out){return family==0 ? mk_ur_candidate_count(&p,&model,&external,&orientation,&limits,&target,seed,8,out) : mk_opw_candidate_count(&opw,&model,&external,&orientation,&limits,&target,seed,8,out);};
    const auto sample=[&](mk_lattice_candidate *out,uint32_t capacity,uint32_t *written){return family==0 ? mk_sample_ur_candidates(&p,&model,&external,&orientation,&limits,&target,seed,8,out,capacity,written) : mk_sample_opw_candidates(&opw,&model,&external,&orientation,&limits,&target,seed,8,out,capacity,written);};
    for(unsigned mode=0;mode<3;++mode) {
        orientation.mode=mode;orientation.tilt_rings=1;orientation.azimuth_count=4;orientation.half_angle=.1;
        uint32_t count=0,written=0;assert(candidate_count(&count)==MK_OK && count>0);
        std::vector<mk_lattice_candidate> candidates(count),repeat(count);
        assert(sample(candidates.data(),count,&written)==MK_OK && written==count);
        assert(sample(repeat.data(),count,&written)==MK_OK);
        std::set<std::tuple<unsigned,unsigned,unsigned,unsigned,unsigned>> covered;
        for(unsigned i=0;i<count;++i) {
            const auto &c=candidates[i],&r=repeat[i];assert(c.branch<8);
            assert(c.external_coordinates[0]<3 && c.external_coordinates[1]<2);
            assert(c.joints[0]==external.lower[0]+.03*c.external_coordinates[0]);
            assert(c.joints[7]==(c.external_coordinates[1]?.1:-.1));assert(c.wraps[0]==0 && c.wraps[7]==0);
            for(unsigned j=0;j<8;++j){assert(c.joints[j]>=limits.lower[j]-1e-9 && c.joints[j]<=limits.upper[j]+1e-9);assert(c.joints[j]==r.joints[j] && c.wraps[j]==r.wraps[j]);}
            assert(c.branch==r.branch && c.roll_index==r.roll_index && c.tilt_index==r.tilt_index && c.azimuth_index==r.azimuth_index);
            mk_opw_pose fk;assert(forward(c.joints+1,&fk)==MK_OK);
            T moved_base=base;moved_base.translation()+=Eigen::Vector3d(.8,.6,0)*c.joints[0];
            T turn=T::Identity();turn.linear()=Eigen::AngleAxisd(c.joints[7],Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const Eigen::Vector3d origin(.2,-.2,.1);turn.translation()=origin-turn.linear()*origin;
            T moved_work=turn*work;
            if(shared)moved_work.translation()+=Eigen::Vector3d(.8,.6,0)*c.joints[0];
            const T actual=moved_work.inverse()*moved_base*pose(fk.position,fk.quaternion)*tool;
            assert((actual.translation()-goal.translation()).norm()<1e-7);
            const double dot=actual.linear().col(2).dot(goal.linear().col(2));
            const double tilt=acos(std::max(-1.0,std::min(1.0,dot)));
            assert(tilt<(mode==2?.100001:1e-7));
            if(mode==0)assert((actual.linear()-goal.linear()).norm()<1e-7);
            covered.emplace(c.external_coordinates[0],c.external_coordinates[1],c.roll_index,c.tilt_index,c.azimuth_index);
        }
        assert(covered.size()==6*(mode==0?1u:mode==1?4u:20u));
        assert(sample(candidates.data(),count-1,&written)==MK_ERROR_INVALID_ARGUMENT);
    }
    external.upper[0]=10;uint32_t count=0;
    assert(candidate_count(&count)==MK_ERROR_INVALID_ARGUMENT);
    external.upper[0]=.03;
    target.position[0]=100;
    assert(candidate_count(&count)==MK_OK && count==0);
    uint32_t written=99;assert(sample(nullptr,0,&written)==MK_OK && written==0);
    const auto original=model.arm_joint_indices[0];model.arm_joint_indices[0]=model.arm_joint_indices[1];
    assert(candidate_count(&count)==MK_ERROR_INVALID_ARGUMENT);model.arm_joint_indices[0]=original;
    }
    }
    for(unsigned n=3;n<=5;++n) {
        mk_analytic_cartesian_model cart={};cart.struct_size=sizeof(cart);cart.joint_count=n;
        cart.translation_axes[0]=cart.translation_axes[4]=cart.translation_axes[8]=1;
        cart.rotary_axes[2]=1;cart.rotary_axes[4]=1;cart.home_quaternion[3]=1;
        cart.home_position[0]=.1;cart.home_position[2]=.3;
        mk_serial_cell_model model={};model.struct_size=sizeof(model);model.joint_count=n;model.arm_joint_count=n;
        model.base_quaternion[3]=model.work_quaternion[3]=model.tool_quaternion[3]=1;
        for(unsigned i=0;i<n;++i)model.arm_joint_indices[i]=i;
        mk_external_lattice external={};external.struct_size=sizeof(external);external.joint_count=n;
        mk_joint_lift_request limits={};limits.struct_size=sizeof(limits);limits.joint_count=n;
        for(unsigned i=0;i<n;++i){limits.lower[i]=i<3 ? -2 : -3.2;limits.upper[i]=i<3 ? 2 : 3.2;limits.periodic[i]=i>=3;}
        double seed[]={.1,.2,.3,.7,.4};mk_opw_pose target;
        assert(mk_analytic_cartesian_forward(&cart,seed,n,&target)==MK_OK);
        mk_orientation_lattice orientation={};orientation.struct_size=sizeof(orientation);orientation.roll_count=4;
        orientation.tilt_rings=1;orientation.azimuth_count=4;orientation.half_angle=.1;
        for(unsigned mode=0;mode<3;++mode) {
            orientation.mode=mode;uint32_t count=0,written=0;
            assert(mk_cartesian_candidate_count(&cart,&model,&external,&orientation,&limits,&target,seed,n,&count)==MK_OK && count>0);
            if(mode==0)assert(count==1);
            if(mode==1)assert(count==(n==3?1u:n==4?4u:2u));
            std::vector<mk_lattice_candidate> candidates(count);
            assert(mk_sample_cartesian_candidates(&cart,&model,&external,&orientation,&limits,&target,seed,n,candidates.data(),count,&written)==MK_OK && written==count);
            bool original=false;
            const auto wanted=pose(target.position,target.quaternion);
            for(const auto &candidate:candidates) {
                bool same=true;for(unsigned i=0;i<n;++i)if(std::abs(candidate.joints[i]-seed[i])>1e-7)same=false;
                original|=same;
                mk_opw_pose fk;assert(mk_analytic_cartesian_forward(&cart,candidate.joints,n,&fk)==MK_OK);
                auto actual=pose(fk.position,fk.quaternion);
                assert((actual.translation()-wanted.translation()).norm()<1e-7);
                double angle=acos(std::max(-1.0,std::min(1.0,actual.linear().col(2).dot(wanted.linear().col(2)))));
                assert(angle<(mode==2?.100001:1e-7));
                if(mode==0)assert((actual.linear()-wanted.linear()).norm()<1e-7);
                for(unsigned i=0;i<n;++i)assert(candidate.joints[i]>=limits.lower[i]-1e-9 && candidate.joints[i]<=limits.upper[i]+1e-9);
            }
            assert(original);
        }
        model.tool_position[0]=.1;uint32_t invalid_count=0;
        assert(mk_cartesian_candidate_count(&cart,&model,&external,&orientation,&limits,&target,seed,n,&invalid_count)==MK_ERROR_INVALID_ARGUMENT);
        model.tool_position[0]=0;limits.periodic[0]=1;
        assert(mk_cartesian_candidate_count(&cart,&model,&external,&orientation,&limits,&target,seed,n,&invalid_count)==MK_ERROR_INVALID_ARGUMENT);
    }

    for(unsigned conventions=0;conventions<2;++conventions)for(unsigned pole=0;pole<2;++pole) {
        mk_opw_parameters opw={};opw.struct_size=sizeof(opw);opw.a1=.1;opw.a2=-.135;
        opw.c1=.615;opw.c2=.705;opw.c3=.755;opw.c4=.085;
        for(unsigned i=0;i<6;++i){opw.sign_corrections[i]=conventions && i%2 ? -1 : 1;opw.offsets[i]=conventions ? .1*i : 0;}
        mk_serial_cell_model model={};model.struct_size=sizeof(model);model.joint_count=6;model.arm_joint_count=6;
        model.base_quaternion[3]=model.work_quaternion[3]=model.tool_quaternion[3]=1;
        for(unsigned i=0;i<6;++i)model.arm_joint_indices[i]=i;
        mk_external_lattice external={};external.struct_size=sizeof(external);external.joint_count=6;
        mk_orientation_lattice orientation={};orientation.struct_size=sizeof(orientation);
        mk_joint_lift_request limits={};limits.struct_size=sizeof(limits);limits.joint_count=6;
        for(unsigned i=0;i<6;++i){limits.lower[i]=-7;limits.upper[i]=7;limits.periodic[i]=1;}
        double canonical[]={.2,-.3,.4,.5,pole*acos(-1.0),.7},seed[6];
        for(unsigned i=0;i<6;++i)seed[i]=(canonical[i]+opw.offsets[i])*opw.sign_corrections[i];
        mk_opw_pose target;assert(mk_opw_forward(&opw,seed,6,&target)==MK_OK);
        uint32_t count=0,written=0;assert(mk_opw_candidate_count(&opw,&model,&external,&orientation,&limits,&target,seed,6,&count)==MK_OK);
        std::vector<mk_lattice_candidate> candidates(count);
        assert(mk_sample_opw_candidates(&opw,&model,&external,&orientation,&limits,&target,seed,6,candidates.data(),count,&written)==MK_OK);
        bool original=false;
        for(const auto &c:candidates) {
            bool same=true;for(unsigned i=0;i<6;++i)if(std::abs(c.joints[i]-seed[i])>1e-6)same=false;
            if(same){original=true;assert(c.singular);}
            mk_opw_pose actual;assert(mk_opw_forward(&opw,c.joints,6,&actual)==MK_OK);
            assert((pose(actual.position,actual.quaternion).matrix()-pose(target.position,target.quaternion).matrix()).norm()<1e-7);
        }
        assert(original);
    }

}
