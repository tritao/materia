#include "motionkit.h"
#include "joint_lift_ranges.h"
#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <cmath>
#include <limits>
#include <new>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <cstring>
namespace {
using T=Eigen::Isometry3d;
using V=Eigen::Vector3d;
V vector(const double *p) {return V(p[0],p[1],p[2]);}
bool valid_pose(const double *p,const double *q) {
    for(unsigned i=0;i<3;++i)if(!std::isfinite(p[i]))return false;
    for(unsigned i=0;i<4;++i)if(!std::isfinite(q[i]))return false;
    return std::abs(Eigen::Quaterniond(q[3],q[0],q[1],q[2]).norm()-1)<1e-8;
}
T pose(const double *p,const double *q) {
    T t=T::Identity();t.translation()=vector(p);t.linear()=Eigen::Quaterniond(q[3],q[0],q[1],q[2]).toRotationMatrix();return t;
}
mk_analytic_pose record(const T &t) {
    mk_analytic_pose p={};p.struct_size=sizeof(p);Eigen::Quaterniond q(t.linear());
    for(unsigned i=0;i<3;++i)p.position[i]=t.translation()[i];
    p.quaternion[0]=q.x();p.quaternion[1]=q.y();p.quaternion[2]=q.z();p.quaternion[3]=q.w();return p;
}
bool valid_model(const mk_serial_cell_model *m,const mk_external_lattice *e,uint32_t n,uint32_t arm_count) {
    if(!m || m->struct_size!=sizeof(*m) || m->joint_count!=n || n>MK_MAX_JOINTS || n<arm_count || m->arm_joint_count!=arm_count ||
        m->external_count!=n-arm_count || !e || e->axis_count!=m->external_count || e->joint_count!=n ||
        !valid_pose(m->base_position,m->base_quaternion) || !valid_pose(m->work_position,m->work_quaternion) ||
        !valid_pose(m->tool_position,m->tool_quaternion))return false;
    std::array<bool,MK_MAX_JOINTS> seen={};
    for(unsigned i=0;i<arm_count;++i) {
        auto j=m->arm_joint_indices[i];if(j>=n || seen[j])return false;seen[j]=true;
    }
    for(unsigned i=0;i<m->external_count;++i) {
        auto j=m->external_joint_indices[i];
        if(j>=n || seen[j] || j!=e->joint_indices[i] || m->external_scopes[i]>2 || m->external_kinds[i]>1)return false;
        seen[j]=true;
        for(unsigned k=0;k<3;++k)if(!std::isfinite(m->external_axes[3*i+k]) || !std::isfinite(m->external_origins[3*i+k]))return false;
        if(std::abs(vector(m->external_axes+3*i).norm()-1)>1e-8)return false;
    }
    return true;
}
T frame(const mk_serial_cell_model &m,const double *q,unsigned scope) {
    T t=T::Identity();
    for(unsigned i=0;i<m.external_count;++i)if(m.external_scopes[i]==scope || m.external_scopes[i]==2) {
        const V axis=vector(m.external_axes+3*i),origin=vector(m.external_origins+3*i);
        T step=T::Identity();const double value=q[m.external_joint_indices[i]];
        if(m.external_kinds[i]==0)step.translation()=axis*value;
        else {step.linear()=Eigen::AngleAxisd(value,axis).toRotationMatrix();step.translation()=origin-step.linear()*origin;}
        t=t*step;
    }
    return t*(scope==0 ? pose(m.base_position,m.base_quaternion) : pose(m.work_position,m.work_quaternion));
}
struct CandidateBatch {
    uint32_t joint_count=0,external_count=0,count=0;
    std::vector<double> joints;
    std::vector<int32_t> wraps;
    std::vector<uint32_t> coordinates;
};
std::mutex batch_mutex;
uint32_t next_batch_id=1;
std::unordered_map<uint32_t,std::shared_ptr<const CandidateBatch>> batches;
struct CompactOutput {double *joints;int32_t *wraps;uint32_t *coordinates;CandidateBatch *batch=nullptr;};
mk_result run(const mk_analytic_cartesian_model *cartesian,const mk_serial_cell_model *m,const mk_external_lattice *e,
    const mk_orientation_lattice *o,const mk_joint_lift_request *limits,const mk_analytic_pose *target,
    const double *seed,uint32_t n,mk_lattice_candidate *out,uint32_t &count,CompactOutput *compact=nullptr,
    mk_eaik_handle eaik={},const mk_eaik_configuration *labels=nullptr) {
    const uint32_t arm_count=cartesian ? cartesian->joint_count : 6;
    if(arm_count<3 || arm_count>6)return MK_ERROR_INVALID_ARGUMENT;
    if(!seed || !limits || limits->struct_size!=sizeof(*limits) || limits->joint_count!=n || !valid_model(m,e,n,arm_count))
        return MK_ERROR_INVALID_ARGUMENT;
    if(cartesian) {
        // Cartesian's native descriptor already includes the complete TCP.
        if(vector(m->tool_position).norm()>1e-12 ||
            std::abs(m->tool_quaternion[0])+std::abs(m->tool_quaternion[1])+std::abs(m->tool_quaternion[2])>1e-12)
            return MK_ERROR_INVALID_ARGUMENT;
        for(unsigned i=0;i<3;++i)if(limits->periodic[m->arm_joint_indices[i]])return MK_ERROR_INVALID_ARGUMENT;
    }
    uint32_t nc,no,unused;
    auto status=mk_external_lattice_count(e,&nc);if(status!=MK_OK)return status;
    status=mk_orientation_lattice_count(o,&no);if(status!=MK_OK)return status;
    status=mk_joint_lift_count(limits,seed,n,&unused);if(status!=MK_OK)return status;
    double arm_seed[6]={};for(unsigned i=0;i<arm_count;++i)arm_seed[i]=seed[m->arm_joint_indices[i]];
    mk_analytic_pose validation{};validation.struct_size=sizeof(validation);
    status=eaik.id ? mk_eaik_forward(eaik,arm_seed,6,&validation) :
        mk_analytic_cartesian_forward(cartesian,arm_seed,arm_count,&validation);if(status!=MK_OK)return status;
    for(unsigned i=0;i<m->external_count;++i) {
        const auto joint=m->external_joint_indices[i];
        if(e->lower[i]<limits->lower[joint]-1e-9 || e->upper[i]>limits->upper[joint]+1e-9)return MK_ERROR_INVALID_ARGUMENT;
    }
    std::vector<mk_external_cell> cells(nc);std::vector<mk_orientation_sample> orientations(no);
    status=mk_sample_external_cells(e,seed,n,cells.data(),nc,&unused);if(status!=MK_OK)return status;
    status=mk_sample_orientations(o,target,orientations.data(),no,&unused);if(status!=MK_OK)return status;
    auto held_limits=*limits;for(unsigned i=0;i<m->external_count;++i)held_limits.periodic[m->external_joint_indices[i]]=0;
    const T tool_inverse=pose(m->tool_position,m->tool_quaternion).inverse();
    uint64_t total=0;
    for(const auto &cell:cells) {
        struct Seen { std::array<double,MK_MAX_JOINTS> q; uint32_t branch; };
        std::vector<Seen> seen;
        const T base_inverse=frame(*m,cell.joints,0).inverse(),work=frame(*m,cell.joints,1);
        for(const auto &orientation:orientations) {
            const auto goal=record(base_inverse*work*pose(orientation.position,orientation.quaternion)*tool_inverse);
            mk_analytic_solution branches[32];uint32_t nb;
            if(eaik.id) status=mk_eaik_inverse_labelled(eaik,labels,&goal,arm_seed,6,branches,32,&nb);
            else if(cartesian) status=mk_analytic_cartesian_inverse(cartesian,&goal,o->mode==0 ? 0 : 1,
                arm_seed[3]+(o->mode==0 ? 0 : 6.28318530717958647692*orientation.roll_index/o->roll_count),branches,8,&nb);
            if(status!=MK_OK)return status;
            for(unsigned b=0;b<nb;++b) {
                std::array<double,MK_MAX_JOINTS> q={};
                for(unsigned i=0;i<n;++i)q[i]=cell.joints[i];
                for(unsigned i=0;i<arm_count;++i)q[m->arm_joint_indices[i]]=branches[b].joints[i];
                if(cartesian) {
                    bool duplicate=false;
                    for(const auto &prior:seen)if(prior.branch==branches[b].branch) {
                        bool same=true;
                        for(unsigned i=0;i<n;++i) {
                            double delta=q[i]-prior.q[i];
                            if(held_limits.periodic[i])delta=std::remainder(delta,6.28318530717958647692);
                            if(std::abs(delta)>1e-9)same=false;
                        }
                        if(same){duplicate=true;break;}
                    }
                    if(duplicate)continue;
                    seen.push_back({q,branches[b].branch});
                }
                uint32_t nl;
                status=mk_joint_lift_count(&held_limits,q.data(),n,&nl);if(status!=MK_OK)return status;
                if(total+nl>std::numeric_limits<uint32_t>::max())return MK_ERROR_LIMIT;
                if(compact && compact->batch &&
                    (total+nl)*(uint64_t(n)*12+(uint64_t(m->external_count)+5)*4)>268435456)return MK_ERROR_LIMIT;
                if(compact && compact->batch && nl){
                    // Enumerate into compact storage directly, using the same
                    // lift ranges and lexicographic mixed-radix ordering.
                    std::array<motionkit_lifts::Range,MK_MAX_JOINTS> ranges;
                    uint32_t verified;
                    if(!motionkit_lifts::ranges(&held_limits,q.data(),n,ranges,verified) || verified!=nl)return MK_ERROR_GENERATION;
                    auto &batch=*compact->batch;
                    for(uint32_t index=0;index<nl;++index){
                        uint64_t code=index;
                        const auto offset=batch.joints.size();batch.joints.resize(offset+n);batch.wraps.resize(offset+n);
                        for(uint32_t j=n;j>0;--j){const auto i=j-1;
                            const auto width=uint64_t(int64_t(ranges[i].last)-ranges[i].first)+1;
                            const auto wrap=int32_t(int64_t(ranges[i].first)+int64_t(code%width));code/=width;
                            batch.joints[offset+i]=q[i]+motionkit_lifts::tau*wrap;batch.wraps[offset+i]=wrap;
                        }
                        for(unsigned i=0;i<m->external_count;++i)batch.coordinates.push_back(cell.coordinates[i]);
                        batch.coordinates.insert(batch.coordinates.end(),{orientation.roll_index,orientation.tilt_index,
                            orientation.azimuth_index,branches[b].branch,branches[b].singular});
                    }
                    total+=nl;
                }else if((out || compact) && nl) {
                    std::vector<mk_joint_lift> lifts(nl);
                    status=mk_enumerate_joint_lifts(&held_limits,q.data(),n,lifts.data(),nl,&unused);if(status!=MK_OK)return status;
                    for(const auto &lift:lifts) {
                        if(compact){

                            const auto joint_offset=size_t(total)*n,cell_offset=size_t(total)*(m->external_count+5);
                            for(unsigned i=0;i<n;++i){compact->joints[joint_offset+i]=lift.joints[i];compact->wraps[joint_offset+i]=lift.wraps[i];}
                            auto cells=compact->coordinates+cell_offset;
                            for(unsigned i=0;i<m->external_count;++i)cells[i]=cell.coordinates[i];
                            cells[m->external_count]=orientation.roll_index;cells[m->external_count+1]=orientation.tilt_index;
                            cells[m->external_count+2]=orientation.azimuth_index;cells[m->external_count+3]=branches[b].branch;
                            cells[m->external_count+4]=branches[b].singular;
                        }else{
                            auto &candidate=out[total];candidate={};candidate.struct_size=sizeof(candidate);
                            for(unsigned i=0;i<n;++i){candidate.joints[i]=lift.joints[i];candidate.wraps[i]=lift.wraps[i];}
                            for(unsigned i=0;i<m->external_count;++i)candidate.external_coordinates[i]=cell.coordinates[i];
                            candidate.roll_index=orientation.roll_index;candidate.tilt_index=orientation.tilt_index;
                            candidate.azimuth_index=orientation.azimuth_index;candidate.branch=branches[b].branch;candidate.singular=branches[b].singular;
                        }
                        ++total;
                    }
                } else total+=nl;
            }
        }
    }
    count=uint32_t(total);return MK_OK;
}
}
extern "C" mk_result MK_CALL mk_cartesian_candidate_count(const mk_analytic_cartesian_model *p,const mk_serial_cell_model *m,
    const mk_external_lattice *e,const mk_orientation_lattice *o,const mk_joint_lift_request *limits,
    const mk_analytic_pose *target,const double *seed,uint32_t n,uint32_t *out_count) {
    if(!out_count)return MK_ERROR_INVALID_ARGUMENT;
    try {uint32_t count;auto status=run(p,m,e,o,limits,target,seed,n,nullptr,count);if(status==MK_OK)*out_count=count;return status;}
    catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
}
extern "C" mk_result MK_CALL mk_sample_cartesian_candidates(const mk_analytic_cartesian_model *p,const mk_serial_cell_model *m,
    const mk_external_lattice *e,const mk_orientation_lattice *o,const mk_joint_lift_request *limits,
    const mk_analytic_pose *target,const double *seed,uint32_t n,mk_lattice_candidate *out,uint32_t capacity,uint32_t *out_count) {
    if(!out_count)return MK_ERROR_INVALID_ARGUMENT;
    try {
        uint32_t count;auto status=run(p,m,e,o,limits,target,seed,n,nullptr,count);if(status!=MK_OK)return status;
        if(capacity<count || (count && !out))return MK_ERROR_INVALID_ARGUMENT;
        if(count)status=run(p,m,e,o,limits,target,seed,n,out,count);
        if(status==MK_OK)*out_count=count;return status;
    }catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
}

namespace {
mk_result sample_compact(const mk_analytic_cartesian_model *cart,
    const mk_serial_cell_model *model,const mk_external_lattice *external,const mk_orientation_lattice *orientation,
    const mk_joint_lift_request *limits,const mk_analytic_pose *target,const double *seed,uint32_t n,
    double *joints,int32_t *wraps,uint32_t joint_value_count,uint32_t *coordinates,uint32_t coordinate_count,uint32_t *out_count) {
    if(!out_count)return MK_ERROR_INVALID_ARGUMENT;
    try {
        uint32_t count;auto status=run(cart,model,external,orientation,limits,target,seed,n,nullptr,count);
        if(status!=MK_OK)return status;
        if(uint64_t(count)*n!=joint_value_count || uint64_t(count)*(uint64_t(model->external_count)+5)!=coordinate_count ||
            (count && (!joints || !wraps || !coordinates)))return MK_ERROR_INVALID_ARGUMENT;
        CompactOutput output{joints,wraps,coordinates};
        if(count)status=run(cart,model,external,orientation,limits,target,seed,n,nullptr,count,&output);
        if(status==MK_OK)*out_count=count;return status;
    }catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
}
}
extern "C" mk_result MK_CALL mk_sample_cartesian_candidates_compact(const mk_analytic_cartesian_model *parameters,
    const mk_serial_cell_model *model,const mk_external_lattice *external,const mk_orientation_lattice *orientation,
    const mk_joint_lift_request *limits,const mk_analytic_pose *target,const double *seed,uint32_t n,
    double *joints,int32_t *wraps,uint32_t joint_value_count,uint32_t *coordinates,uint32_t coordinate_count,uint32_t *out_count) {
    return sample_compact(parameters,
        model,external,orientation,limits,target,seed,n,joints,wraps,joint_value_count,coordinates,coordinate_count,out_count);
}

extern "C" mk_result MK_CALL mk_eaik_candidate_count(mk_eaik_handle solver,const mk_eaik_configuration *labels,
    const mk_serial_cell_model *model,const mk_external_lattice *external,const mk_orientation_lattice *orientation,
    const mk_joint_lift_request *limits,const mk_analytic_pose *target,const double *seed,uint32_t n,uint32_t *out_count) {
    if(!out_count || !solver.id)return MK_ERROR_INVALID_ARGUMENT;
    *out_count=0;
    try {uint32_t count=0;const auto status=run(nullptr,model,external,orientation,limits,target,seed,n,nullptr,count,nullptr,solver,labels);if(status==MK_OK)*out_count=count;return status;}
    catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
    catch(...){return MK_ERROR_GENERATION;}
}
extern "C" mk_result MK_CALL mk_sample_eaik_candidates_compact(mk_eaik_handle solver,const mk_eaik_configuration *labels,
    const mk_serial_cell_model *model,const mk_external_lattice *external,const mk_orientation_lattice *orientation,
    const mk_joint_lift_request *limits,const mk_analytic_pose *target,const double *seed,uint32_t n,
    double *joints,int32_t *wraps,uint32_t joint_value_count,uint32_t *coordinates,uint32_t coordinate_count,uint32_t *out_count) {
    if(!out_count || !solver.id)return MK_ERROR_INVALID_ARGUMENT;
    *out_count=0;
    try {
        uint32_t count=0;auto status=run(nullptr,model,external,orientation,limits,target,seed,n,nullptr,count,nullptr,solver,labels);
        if(status!=MK_OK)return status;
        if(uint64_t(count)*n!=joint_value_count || uint64_t(count)*(uint64_t(model->external_count)+5)!=coordinate_count ||
            (count && (!joints || !wraps || !coordinates)))return MK_ERROR_INVALID_ARGUMENT;
        CompactOutput output{joints,wraps,coordinates};
        if(count)status=run(nullptr,model,external,orientation,limits,target,seed,n,nullptr,count,&output,solver,labels);
        if(status==MK_OK)*out_count=count;return status;
    }catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
    catch(...){return MK_ERROR_GENERATION;}
}

extern "C" mk_result MK_CALL mk_sample_eaik_candidates(mk_eaik_handle solver,const mk_eaik_configuration *labels,const mk_serial_cell_model *m,
    const mk_external_lattice *e,const mk_orientation_lattice *o,const mk_joint_lift_request *limits,
    const mk_analytic_pose *target,const double *seed,uint32_t n,mk_lattice_candidate *out,uint32_t capacity,uint32_t *out_count) {
    if(!out_count)return MK_ERROR_INVALID_ARGUMENT;
    try {
        uint32_t count;auto status=run(nullptr,m,e,o,limits,target,seed,n,nullptr,count,nullptr,solver,labels);if(status!=MK_OK)return status;
        if(capacity<count || (count && !out))return MK_ERROR_INVALID_ARGUMENT;
        if(count)status=run(nullptr,m,e,o,limits,target,seed,n,out,count,nullptr,solver,labels);
        if(status==MK_OK)*out_count=count;return status;
    }catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
}

namespace {
mk_result create_batch(const mk_analytic_cartesian_model *cartesian,mk_eaik_handle eaik,
    const mk_eaik_configuration *labels,const mk_serial_cell_model *model,const mk_external_lattice *external,
    const mk_orientation_lattice *orientation,const mk_joint_lift_request *limits,const mk_analytic_pose *target,
    const double *seed,uint32_t n,mk_candidate_batch_handle *out,uint32_t *count) {
    if(!out || !count)return MK_ERROR_INVALID_ARGUMENT;
    out->id=0;*count=0;
    if(!cartesian && !eaik.id)return MK_ERROR_INVALID_ARGUMENT;
    try {
        auto batch=std::make_shared<CandidateBatch>();
        CompactOutput output{nullptr,nullptr,nullptr,batch.get()};
        uint32_t generated=0;
        const auto status=run(cartesian,model,external,orientation,limits,target,seed,n,nullptr,generated,&output,eaik,labels);
        if(status!=MK_OK)return status;
        batch->joint_count=n;batch->external_count=model->external_count;batch->count=generated;
        std::lock_guard lock(batch_mutex);
        if(next_batch_id==0)return MK_ERROR_LIMIT;
        const auto id=next_batch_id++;
        batches.emplace(id,std::move(batch));out->id=id;*count=generated;
        return MK_OK;
    }catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
     catch(...){return MK_ERROR_GENERATION;}
}
}
extern "C" mk_result MK_CALL mk_create_eaik_candidate_batch(mk_eaik_handle solver,
    const mk_eaik_configuration *labels,const mk_serial_cell_model *model,const mk_external_lattice *external,
    const mk_orientation_lattice *orientation,const mk_joint_lift_request *limits,const mk_analytic_pose *target,
    const double *seed,uint32_t n,mk_candidate_batch_handle *out,uint32_t *count) {
    return create_batch(nullptr,solver,labels,model,external,orientation,limits,target,seed,n,out,count);
}
extern "C" mk_result MK_CALL mk_create_cartesian_candidate_batch(const mk_analytic_cartesian_model *parameters,
    const mk_serial_cell_model *model,const mk_external_lattice *external,const mk_orientation_lattice *orientation,
    const mk_joint_lift_request *limits,const mk_analytic_pose *target,const double *seed,uint32_t n,
    mk_candidate_batch_handle *out,uint32_t *count) {
    return create_batch(parameters,{},nullptr,model,external,orientation,limits,target,seed,n,out,count);
}
extern "C" void MK_CALL mk_candidate_batch_destroy(mk_candidate_batch_handle handle) {
    std::lock_guard lock(batch_mutex);batches.erase(handle.id);
}
extern "C" mk_result MK_CALL mk_read_candidate_batch(mk_candidate_batch_handle handle,
    double *joints,int32_t *wraps,uint32_t joint_count,uint32_t *coordinates,uint32_t coordinate_count,uint32_t *out_count) {
    if(!out_count)return MK_ERROR_INVALID_ARGUMENT;
    *out_count=0;
    std::shared_ptr<const CandidateBatch> batch;
    {std::lock_guard lock(batch_mutex);const auto found=batches.find(handle.id);
        if(found==batches.end())return MK_ERROR_INVALID_HANDLE;
        batch=found->second;
    }
    if(batch->joints.size()!=joint_count || batch->coordinates.size()!=coordinate_count ||
        (batch->count && (!joints || !wraps || !coordinates)))return MK_ERROR_INVALID_ARGUMENT;
    if(batch->count){
        std::memcpy(joints,batch->joints.data(),batch->joints.size()*sizeof(double));
        std::memcpy(wraps,batch->wraps.data(),batch->wraps.size()*sizeof(int32_t));
        std::memcpy(coordinates,batch->coordinates.data(),batch->coordinates.size()*sizeof(uint32_t));
    }
    *out_count=batch->count;return MK_OK;
}
