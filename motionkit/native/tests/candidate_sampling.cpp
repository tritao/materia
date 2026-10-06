#include "eaik_fixtures.h"
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <vector>
#include <set>
#include <tuple>
#include <iostream>
using T=Eigen::Isometry3d;
T pose(const double *p,const double *q) {
    T t=T::Identity();t.translation()=Eigen::Vector3d(p[0],p[1],p[2]);
    t.linear()=Eigen::Quaterniond(q[3],q[0],q[1],q[2]).toRotationMatrix();return t;
}
void store(const T &t,double *p,double *r) {
    Eigen::Quaterniond q(t.linear());for(unsigned i=0;i<3;++i)p[i]=t.translation()[i];
    r[0]=q.x();r[1]=q.y();r[2]=q.z();r[3]=q.w();
}
void check_compact(const std::vector<mk_lattice_candidate> &expected,unsigned n,unsigned external,
    const std::vector<double> &joints,const std::vector<int32_t> &wraps,const std::vector<uint32_t> &cells) {
    for(unsigned i=0;i<expected.size();++i){const auto &c=expected[i];
        for(unsigned j=0;j<n;++j){assert(joints[i*n+j]==c.joints[j]);assert(wraps[i*n+j]==c.wraps[j]);}
        auto row=cells.data()+i*(external+5);
        for(unsigned j=0;j<external;++j)assert(row[j]==c.external_coordinates[j]);
        assert(row[external]==c.roll_index && row[external+1]==c.tilt_index && row[external+2]==c.azimuth_index);
        assert(row[external+3]==c.branch && row[external+4]==c.singular);
    }
}
int main() {
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
    for(unsigned shared=0;shared<2;++shared) {
    model.external_scopes[0]=shared ? 2 : 0;
    for(unsigned family=0;family<2;++family) {
    const auto geometry=family==0 ? eaik_fixtures::parallel({-.425,-.39225,.089159,.10915,.09465,.0823}) : eaik_fixtures::spherical({.1,-.135,0,.615,.705,.755,.085});
    const auto labels=eaik_fixtures::labels(family!=0,family ? .1 : 0,family ? std::atan2(-.135,.755) : 0);
    mk_eaik_handle solver{};assert(mk_eaik_create(&geometry,&solver)==MK_OK);
    const auto forward=[&](const double *q,mk_analytic_pose *out){out->struct_size=sizeof(*out);return mk_eaik_forward(solver,q,6,out);};
    mk_analytic_pose arm{};assert(forward(seed+1,&arm)==MK_OK);
    const T goal=work.inverse()*base*pose(arm.position,arm.quaternion)*tool;
    mk_analytic_pose target={};target.struct_size=sizeof(target);store(goal,target.position,target.quaternion);
    const auto candidate_count=[&](uint32_t *out){return mk_eaik_candidate_count(solver,&labels,&model,&external,&orientation,&limits,&target,seed,8,out);};
    const auto sample=[&](mk_lattice_candidate *out,uint32_t capacity,uint32_t *written){return mk_sample_eaik_candidates(solver,&labels,&model,&external,&orientation,&limits,&target,seed,8,out,capacity,written);};
    for(unsigned mode=0;mode<3;++mode) {
        orientation.mode=mode;orientation.tilt_rings=1;orientation.azimuth_count=4;orientation.half_angle=.1;
        uint32_t count=0,written=0;assert(candidate_count(&count)==MK_OK && count>0);
        std::vector<mk_lattice_candidate> candidates(count),repeat(count);
        assert(sample(candidates.data(),count,&written)==MK_OK && written==count);
        assert(sample(repeat.data(),count,&written)==MK_OK);
        std::vector<double> compact_joints(count*8);std::vector<int32_t> compact_wraps(count*8);
        std::vector<uint32_t> compact_cells(count*7);
        const auto compact_sample=[&](unsigned joint_values,unsigned cell_values){return
            mk_sample_eaik_candidates_compact(solver,&labels,&model,&external,&orientation,&limits,&target,seed,8,
                compact_joints.data(),compact_wraps.data(),joint_values,compact_cells.data(),cell_values,&written);};
        assert(compact_sample(compact_joints.size(),compact_cells.size())==MK_OK && written==count);
        check_compact(candidates,8,2,compact_joints,compact_wraps,compact_cells);
        assert(compact_sample(compact_joints.size()-1,compact_cells.size())==MK_ERROR_INVALID_ARGUMENT);
        assert(compact_sample(compact_joints.size(),compact_cells.size()-1)==MK_ERROR_INVALID_ARGUMENT);
        std::set<std::tuple<unsigned,unsigned,unsigned,unsigned,unsigned>> covered;
        for(unsigned i=0;i<count;++i) {
            const auto &c=candidates[i],&r=repeat[i];assert(c.branch<8);
            assert(c.external_coordinates[0]<3 && c.external_coordinates[1]<2);
            assert(c.joints[0]==external.lower[0]+.03*c.external_coordinates[0]);
            assert(c.joints[7]==(c.external_coordinates[1]?.1:-.1));assert(c.wraps[0]==0 && c.wraps[7]==0);
            for(unsigned j=0;j<8;++j){assert(c.joints[j]>=limits.lower[j]-1e-9 && c.joints[j]<=limits.upper[j]+1e-9);assert(c.joints[j]==r.joints[j] && c.wraps[j]==r.wraps[j]);}
            assert(c.branch==r.branch && c.roll_index==r.roll_index && c.tilt_index==r.tilt_index && c.azimuth_index==r.azimuth_index);
            mk_analytic_pose fk;assert(forward(c.joints+1,&fk)==MK_OK);
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
    mk_eaik_destroy(solver);
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
        double seed[]={.1,.2,.3,.7,.4};mk_analytic_pose target;
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
            std::vector<double> compact_joints(count*n);std::vector<int32_t> compact_wraps(count*n);
            std::vector<uint32_t> compact_cells(count*5);
            assert(mk_sample_cartesian_candidates_compact(&cart,&model,&external,&orientation,&limits,&target,seed,n,
                compact_joints.data(),compact_wraps.data(),compact_joints.size(),compact_cells.data(),compact_cells.size(),&written)==MK_OK && written==count);
            check_compact(candidates,n,0,compact_joints,compact_wraps,compact_cells);
            bool original=false;
            const auto wanted=pose(target.position,target.quaternion);
            for(const auto &candidate:candidates) {
                bool same=true;for(unsigned i=0;i<n;++i)if(std::abs(candidate.joints[i]-seed[i])>1e-7)same=false;
                original|=same;
                mk_analytic_pose fk;assert(mk_analytic_cartesian_forward(&cart,candidate.joints,n,&fk)==MK_OK);
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
        std::array<double,6> offsets{};std::array<int,6> signs{};
        for(unsigned i=0;i<6;++i){signs[i]=conventions && i%2 ? -1 : 1;offsets[i]=conventions ? .1*i : 0;}
        auto geometry=eaik_fixtures::spherical({.1,-.135,0,.615,.705,.755,.085},offsets,signs);
        auto labels=eaik_fixtures::labels(true,.1,std::atan2(-.135,.755));
        for(unsigned i=0;i<6;++i){labels.reference[i]=offsets[i]*signs[i];labels.signs[i]=signs[i];}
        mk_eaik_handle solver{};assert(mk_eaik_create(&geometry,&solver)==MK_OK);
        mk_serial_cell_model model={};model.struct_size=sizeof(model);model.joint_count=6;model.arm_joint_count=6;
        model.base_quaternion[3]=model.work_quaternion[3]=model.tool_quaternion[3]=1;
        for(unsigned i=0;i<6;++i)model.arm_joint_indices[i]=i;
        mk_external_lattice external={};external.struct_size=sizeof(external);external.joint_count=6;
        mk_orientation_lattice orientation={};orientation.struct_size=sizeof(orientation);
        mk_joint_lift_request limits={};limits.struct_size=sizeof(limits);limits.joint_count=6;
        for(unsigned i=0;i<6;++i){limits.lower[i]=-7;limits.upper[i]=7;limits.periodic[i]=1;}
        double canonical[]={.2,-.3,.4,.5,pole*acos(-1.0),.7},seed[6];
        for(unsigned i=0;i<6;++i)seed[i]=(canonical[i]+offsets[i])*signs[i];
        mk_analytic_pose target{};target.struct_size=sizeof(target);assert(mk_eaik_forward(solver,seed,6,&target)==MK_OK);
        uint32_t count=0,written=0;assert(mk_eaik_candidate_count(solver,&labels,&model,&external,&orientation,&limits,&target,seed,6,&count)==MK_OK);
        std::vector<mk_lattice_candidate> candidates(count);
        assert(mk_sample_eaik_candidates(solver,&labels,&model,&external,&orientation,&limits,&target,seed,6,candidates.data(),count,&written)==MK_OK);
        bool original=false;
        for(const auto &c:candidates) {
            bool same=true;for(unsigned i=0;i<6;++i)if(std::abs(c.joints[i]-seed[i])>1e-6)same=false;
            if(same){original=true;assert(c.singular);}
            mk_analytic_pose actual{};actual.struct_size=sizeof(actual);assert(mk_eaik_forward(solver,c.joints,6,&actual)==MK_OK);
            assert((pose(actual.position,actual.quaternion).matrix()-pose(target.position,target.quaternion).matrix()).norm()<1e-7);
        }
        if(!original){
            mk_analytic_solution debug[32];uint32_t debugCount=0;
            assert(mk_eaik_inverse_labelled(solver,&labels,&target,seed,6,debug,32,&debugCount)==MK_OK);
            for(unsigned i=0;i<debugCount;++i){std::cerr<<"branch="<<debug[i].branch<<" singular="<<debug[i].singular<<" q=";for(double x:debug[i].joints)std::cerr<<x<<",";std::cerr<<"\n";}
            std::cerr<<"missing pole="<<pole<<" conventions="<<conventions<<" count="<<count<<" seed=";for(double x:seed)std::cerr<<x<<",";std::cerr<<"\n";}
        assert(original);mk_eaik_destroy(solver);
    }

}
