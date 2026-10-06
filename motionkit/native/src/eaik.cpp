#include "motionkit.h"
#include "EAIK.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace {
std::mutex solver_mutex;
uint32_t next_id=1;
std::unordered_map<uint32_t,std::shared_ptr<const EAIK::Robot>> solvers;
std::shared_ptr<const EAIK::Robot> find(mk_eaik_handle handle) {
    std::lock_guard lock(solver_mutex);
    const auto it=solvers.find(handle.id);
    return it==solvers.end() ? nullptr : it->second;
}
bool valid_pose(const mk_analytic_pose *p) {
    if(!p || p->struct_size<sizeof(*p)) return false;
    for(double x:p->position) if(!std::isfinite(x)) return false;
    for(double x:p->quaternion) if(!std::isfinite(x)) return false;
    return std::abs(Eigen::Map<const Eigen::Vector4d>(p->quaternion).norm()-1)<1e-8;
}
Eigen::Matrix4d pose(const mk_analytic_pose &p) {
    Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
    t.block<3,1>(0,3)=Eigen::Map<const Eigen::Vector3d>(p.position);
    t.block<3,3>(0,0)=Eigen::Quaterniond(p.quaternion[3],p.quaternion[0],p.quaternion[1],p.quaternion[2]).toRotationMatrix();
    return t;
}
}
extern "C" {
mk_result MK_CALL mk_eaik_create(const mk_eaik_model *m,mk_eaik_handle *out) {
    if(!out) return MK_ERROR_INVALID_ARGUMENT;
    out->id=0;
    if(!m || m->struct_size<sizeof(*m)) return MK_ERROR_INVALID_ARGUMENT;
    const Eigen::Map<const Eigen::Matrix<double,3,6>> h(m->axes);
    const Eigen::Map<const Eigen::Matrix<double,3,7>> p(m->displacements);
    const Eigen::Map<const Eigen::Matrix3d> r(m->terminal_rotation);
    if(!h.allFinite() || !p.allFinite() || !r.allFinite() ||
       (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()>1e-8 || std::abs(r.determinant()-1)>1e-8)
        return MK_ERROR_INVALID_ARGUMENT;
    for(int j=0;j<6;++j) if(std::abs(h.col(j).norm()-1)>1e-8) return MK_ERROR_INVALID_ARGUMENT;
    try {
        auto robot=std::make_shared<EAIK::Robot>(Eigen::MatrixXd(h),Eigen::MatrixXd(p),Eigen::Matrix3d(r));
        if(!robot->has_known_decomposition()) return MK_ERROR_UNSUPPORTED;
        std::lock_guard lock(solver_mutex);
        if(next_id==0) return MK_ERROR_GENERATION;
        const auto id=next_id++;
        solvers.emplace(id,std::move(robot));out->id=id;
        return MK_OK;
    } catch(const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
      catch(...) { return MK_ERROR_GENERATION; }
}
void MK_CALL mk_eaik_destroy(mk_eaik_handle handle) {
    std::lock_guard lock(solver_mutex);solvers.erase(handle.id);
}
mk_result MK_CALL mk_eaik_forward(mk_eaik_handle handle,const double *q,uint32_t n,mk_analytic_pose *out) {
    if(!q || n!=6 || !out || out->struct_size<sizeof(*out)) return MK_ERROR_INVALID_ARGUMENT;
    for(unsigned j=0;j<n;++j) if(!std::isfinite(q[j])) return MK_ERROR_INVALID_ARGUMENT;
    const auto robot=find(handle);if(!robot) return MK_ERROR_INVALID_HANDLE;
    try {
        const auto t=robot->fwdkin(std::vector<double>(q,q+6));
        const Eigen::Quaterniond r(t.block<3,3>(0,0));
        for(unsigned j=0;j<3;++j)out->position[j]=t(j,3);
        out->quaternion[0]=r.x();out->quaternion[1]=r.y();out->quaternion[2]=r.z();out->quaternion[3]=r.w();
        return MK_OK;
    } catch(const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
      catch(...) { return MK_ERROR_GENERATION; }
}
mk_result MK_CALL mk_eaik_inverse(mk_eaik_handle handle,const mk_analytic_pose *target,
    mk_analytic_solution *out,uint32_t capacity,uint32_t *count) {
    if(!count) return MK_ERROR_INVALID_ARGUMENT;
    *count=0;
    if(!valid_pose(target) || (!out && capacity)) return MK_ERROR_INVALID_ARGUMENT;
    const auto robot=find(handle);if(!robot)return MK_ERROR_INVALID_HANDLE;
    try {
        const auto goal=pose(*target);
        const auto solutions=robot->calculate_IK(goal);
        std::vector<std::array<double,6>> exact;
        for(size_t i=0;i<solutions.Q.size();++i) {
            if(solutions.is_LS_vec[i] || solutions.Q[i].size()!=6)continue;
            std::array<double,6> q{};bool finite=true;
            for(unsigned j=0;j<6;++j){q[j]=std::remainder(solutions.Q[i][j],2*std::acos(-1.0));finite=finite && std::isfinite(q[j]);}
            if(!finite) continue;
            const auto fk=robot->fwdkin(std::vector<double>(q.begin(),q.end()));
            if((fk.block<3,1>(0,3)-goal.block<3,1>(0,3)).norm()>1e-9 ||
               Eigen::AngleAxisd(fk.block<3,3>(0,0).transpose()*goal.block<3,3>(0,0)).angle()>1e-9)continue;
            bool duplicate=false;
            for(const auto &prior:exact){double error=0;for(unsigned j=0;j<6;++j)error=std::max(error,std::abs(std::remainder(q[j]-prior[j],2*std::acos(-1.0))));if(error<1e-10){duplicate=true;break;}}
            if(!duplicate)exact.push_back(q);
        }
        // Stable ordering is for transport only; physical labels must come from geometry.
        std::sort(exact.begin(),exact.end());
        if(exact.size()>capacity) return MK_ERROR_INVALID_ARGUMENT;
        for(size_t i=0;i<exact.size();++i){out[i]={};out[i].struct_size=sizeof(out[i]);out[i].branch=static_cast<uint32_t>(i);std::copy(exact[i].begin(),exact[i].end(),out[i].joints);}
        *count=static_cast<uint32_t>(exact.size());return MK_OK;
    } catch(const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
      catch(...) { return MK_ERROR_GENERATION; }
}
mk_result MK_CALL mk_eaik_inverse_labelled(mk_eaik_handle handle,const mk_eaik_configuration *c,
    const mk_analytic_pose *target,const double *seed,uint32_t n,mk_analytic_solution *out,uint32_t capacity,uint32_t *count) {
    if(!count)return MK_ERROR_INVALID_ARGUMENT;
    *count=0;
    if(!seed || n!=6)return MK_ERROR_INVALID_ARGUMENT;
    for(unsigned j=0;j<6;++j)if(!std::isfinite(seed[j]))return MK_ERROR_INVALID_ARGUMENT;
    if(!c || c->struct_size<sizeof(*c) || c->spherical>1) return MK_ERROR_INVALID_ARGUMENT;
    for(double x:c->reference)if(!std::isfinite(x))return MK_ERROR_INVALID_ARGUMENT;
    for(double x:c->signs)if(x!=1 && x!=-1)return MK_ERROR_INVALID_ARGUMENT;
    try {
    const auto robot=find(handle);if(!robot)return MK_ERROR_INVALID_HANDLE;
    if(!valid_pose(target) || (!out && capacity))return MK_ERROR_INVALID_ARGUMENT;
    const auto goal=pose(*target);
    const auto raw=robot->calculate_IK(goal);
    std::vector<mk_analytic_solution> candidates;
    for(size_t i=0;i<raw.Q.size();++i){
        if(raw.Q[i].size()!=6)continue;
        mk_analytic_solution answer{};answer.struct_size=sizeof(answer);
        bool finite=true;for(unsigned j=0;j<6;++j){answer.joints[j]=std::remainder(raw.Q[i][j],2*std::acos(-1.0));finite=finite && std::isfinite(answer.joints[j]);}
        if(finite)candidates.push_back(answer);
    }
    const Eigen::Matrix3d base=Eigen::Map<const Eigen::Matrix3d>(c->base_rotation);
    const Eigen::Vector3d base_p=Eigen::Map<const Eigen::Vector3d>(c->base_position);
    if(!base.allFinite() || !base_p.allFinite() || (base.transpose()*base-Eigen::Matrix3d::Identity()).norm()>1e-8 || std::abs(base.determinant()-1)>1e-8 || !std::isfinite(c->shoulder_offset) || !std::isfinite(c->elbow_offset))return MK_ERROR_INVALID_ARGUMENT;
    const auto h=robot->get_original_H(),p=robot->get_original_P();
    std::vector<mk_analytic_solution> exact;
    for(auto answer:candidates) {
        double t[6];
        for(unsigned j=0;j<6;++j)t[j]=(answer.joints[j]-c->reference[j])*c->signs[j];
        Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
        Eigen::Vector3d origin=p.col(0),wrist=origin;
        std::array<Eigen::Vector3d,6> axes,origins;
        for(unsigned j=0;j<6;++j){
            axes[j]=rotation*h.col(j);origins[j]=origin;
            if(j==(c->spherical ? 4u : 5u))wrist=origin;
            rotation=rotation*Eigen::AngleAxisd(answer.joints[j],h.col(j)).toRotationMatrix();
            origin+=rotation*p.col(j+1);
        }
        if(c->spherical)wrist=origins[3]+axes[3]*(origins[4]-origins[3]).dot(axes[3]);
        wrist=base.transpose()*(wrist-base_p);
        const bool front=c->spherical ? wrist.x()*std::cos(t[0])+wrist.y()*std::sin(t[0])-c->shoulder_offset>=0 :
            std::sin(t[0]-std::atan2(wrist.x(),-wrist.y()))>=0;
        const bool up=std::sin(t[2]+c->elbow_offset)>=0,flip=std::sin(t[4])<0;
        answer.branch=(front ? 0u : 2u)+(up ? 0u : 1u)+(flip ? 4u : 0u);
        if(c->spherical && axes[3].cross(axes[5]).norm()<1e-6) {
            std::vector<double> seeded(answer.joints,answer.joints+6);seeded[3]=seed[3];
            seeded[4]=c->reference[4]+(std::cos(t[4])>=0 ? 0.0 : std::acos(-1.0))*c->signs[4];seeded[5]=0;
            const auto zero=robot->fwdkin(seeded);
            const Eigen::Matrix3d delta=zero.block<3,3>(0,0).transpose()*pose(*target).block<3,3>(0,0);
            const Eigen::AngleAxisd spin(delta);
            const auto end=robot->fwdkin(std::vector<double>(6,0.0));
            const Eigen::Vector3d axis=end.block<3,3>(0,0).transpose()*h.col(5);
            seeded[5]=spin.angle()*spin.axis().dot(axis);
            const auto checked=robot->fwdkin(seeded),goal=pose(*target);
            if((checked.block<3,1>(0,3)-goal.block<3,1>(0,3)).norm()<1e-9 &&
                Eigen::AngleAxisd(checked.block<3,3>(0,0).transpose()*goal.block<3,3>(0,0)).angle()<1e-9)
                std::copy(seeded.begin(),seeded.end(),answer.joints);
        }
        answer.singular=(axes[3].cross(axes[5]).norm()<1e-7 ? 1u : 0u) |
            (std::abs(std::sin(t[2]+c->elbow_offset))<1e-7 ? 2u : 0u);
        const auto fk=robot->fwdkin(std::vector<double>(answer.joints,answer.joints+6));
        if((fk.block<3,1>(0,3)-goal.block<3,1>(0,3)).norm()>1e-9 ||
            Eigen::AngleAxisd(fk.block<3,3>(0,0).transpose()*goal.block<3,3>(0,0)).angle()>1e-9)continue;
        bool duplicate=false;
        for(const auto &prior:exact){if(prior.branch!=answer.branch)continue;double delta=0;for(unsigned j=0;j<6;++j)delta=std::max(delta,std::abs(std::remainder(answer.joints[j]-prior.joints[j],2*std::acos(-1.0))));if(delta<1e-10){duplicate=true;break;}}
        if(!duplicate)exact.push_back(answer);
    }
    std::stable_sort(exact.begin(),exact.end(),[](const auto &a,const auto &b){
        if(a.branch!=b.branch)return a.branch<b.branch;
        return std::lexicographical_compare(a.joints,a.joints+6,b.joints,b.joints+6);
    });
    if(exact.size()>capacity)return MK_ERROR_INVALID_ARGUMENT;
    std::copy(exact.begin(),exact.end(),out);*count=static_cast<uint32_t>(exact.size());
    return MK_OK;
    } catch(const std::bad_alloc &) { *count=0;return MK_ERROR_OUT_OF_MEMORY; }
      catch(...) { *count=0;return MK_ERROR_GENERATION; }
}

}
