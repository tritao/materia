#include "coarse_ladder.h"
#include <new>
namespace {
struct CandidateSlice {
    const mk_lattice_candidate *data;unsigned count;
    unsigned size()const{return count;}
    bool empty()const{return count==0;}
    const mk_lattice_candidate &operator[](unsigned i)const{return data[i];}
    const mk_lattice_candidate *begin()const{return data;}
    const mk_lattice_candidate *end()const{return count?data+count:data;}
};
struct Slices {
    const mk_configuration_sample *samples;unsigned count;
    const mk_lattice_candidate *candidates;
    unsigned size()const{return count;}
    bool empty()const{return count==0;}
    CandidateSlice operator[](unsigned i)const{return {candidates?candidates+samples[i].first_candidate:nullptr,samples[i].candidate_count};}
};
}
extern "C" mk_result MK_CALL mk_search_ladder(const mk_ladder_request *request,
    const mk_configuration_sample *samples,uint32_t sample_count,
    const mk_lattice_candidate *candidates,const double *state_costs,uint32_t candidate_count,
    uint32_t *out_indices,mk_ladder_result *out_result) {
    if(!request || request->struct_size!=sizeof(*request) || !samples || !sample_count ||
        (candidate_count && (!candidates || !state_costs)) || !out_indices || !out_result)
        return MK_ERROR_INVALID_ARGUMENT;
    *out_result={};out_result->struct_size=sizeof(*out_result);out_result->failed_sample=UINT32_MAX;
    for(unsigned i=0;i<sample_count;++i){out_indices[i]=UINT32_MAX;
        const auto &s=samples[i];
        if(s.struct_size!=sizeof(s) || !std::isfinite(s.distance) || s.distance<0 ||
            (i && s.distance<=samples[i-1].distance) || s.first_candidate>candidate_count ||
            s.candidate_count>candidate_count-s.first_candidate)return MK_ERROR_INVALID_ARGUMENT;
    }
    if(request->joint_count==0 || request->joint_count>MK_MAX_JOINTS || request->external_count>request->joint_count ||
        request->roll_count==0 || request->tilt_count==0 || request->azimuth_count==0)
        return MK_ERROR_INVALID_ARGUMENT;
    for(unsigned i=0;i<candidate_count;++i){const auto &c=candidates[i];
        if(c.struct_size!=sizeof(c) || c.roll_index>=request->roll_count || c.tilt_index>=request->tilt_count ||
            c.azimuth_index>=request->azimuth_count || std::isnan(state_costs[i]) || state_costs[i]<0)
            return MK_ERROR_INVALID_ARGUMENT;
        for(unsigned j=0;j<request->joint_count;++j)if(!std::isfinite(c.joints[j]))return MK_ERROR_INVALID_ARGUMENT;
    }
    try {
        motionkit::LadderSettings settings;settings.joints=request->joint_count;settings.externals=request->external_count;
        settings.rolls=request->roll_count;settings.tilts=request->tilt_count;settings.azimuths=request->azimuth_count;
        settings.roll_weight=request->roll_weight;
        for(unsigned j=0;j<MK_MAX_JOINTS;++j){settings.jump[j]=request->max_jump[j];settings.weight[j]=request->weights[j];
            settings.velocity[j]=request->velocity[j];settings.start[j]=request->start_joints[j];}
        std::vector<std::vector<double>> costs(sample_count);
        for(unsigned i=0;i<sample_count;++i)if(samples[i].candidate_count)
            costs[i].assign(state_costs+samples[i].first_candidate,state_costs+samples[i].first_candidate+samples[i].candidate_count);
        auto slices=Slices{samples,sample_count,candidates};
        auto result=request->coarse_sample_stride ? motionkit::coarse_ladder(slices,settings,
            motionkit::CoarseLadderSettings{request->coarse_sample_stride,request->coarse_lattice_stride,
                request->corridor_radius,request->corridor_widenings},costs) : motionkit::structured_ladder(slices,settings,costs);
        out_result->failed_sample=result.failed;out_result->cost=result.cost;out_result->tested_edges=result.tested_edges;
        if(result.failed!=UINT32_MAX){out_result->failed_distance=samples[result.failed].distance;
            out_result->failure_kind=result.no_candidates?1:2;return MK_ERROR_GENERATION;}
        for(unsigned i=0;i<sample_count;++i)out_indices[i]=samples[i].first_candidate+result.route[i];
        return MK_OK;
    }catch(const std::invalid_argument &){return MK_ERROR_INVALID_ARGUMENT;}
    catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
}
