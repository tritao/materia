#include "coarse_ladder.h"
#include <new>
#include <memory>
#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <console_bridge/console.h>
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
using State=descartes_light::State<double>;
class SmallSampler final:public descartes_light::WaypointSamplerD {
    const CandidateSlice slice;const double *costs;const motionkit::LadderSettings &settings;bool first;
public:
    SmallSampler(CandidateSlice slice,const double *costs,const motionkit::LadderSettings &settings,bool first)
        :slice(slice),costs(costs),settings(settings),first(first){}
    std::vector<descartes_light::StateSample<double>> sample()const override{
        std::vector<descartes_light::StateSample<double>> result;
        for(unsigned i=0;i<slice.size();++i)if(std::isfinite(costs[i])){
            Eigen::VectorXd index(1);index[0]=i;double cost=costs[i];
            if(first)for(unsigned j=0;j<settings.joints;++j)
                cost+=settings.weight[j]*std::abs(slice[i].joints[j]-settings.start[j])/settings.velocity[j];
            result.emplace_back(std::make_shared<State>(index),cost);
        }return result;
    }
};
class SmallEdge final:public descartes_light::EdgeEvaluatorD {
    CandidateSlice prior,current;const motionkit::LadderSettings &settings;uint64_t &tested;
public:
    SmallEdge(CandidateSlice prior,CandidateSlice current,const motionkit::LadderSettings &settings,uint64_t &tested)
        :prior(prior),current(current),settings(settings),tested(tested){}
    std::pair<bool,double> evaluate(const State &a,const State &b)const override{
        ++tested;const auto &from=prior[static_cast<unsigned>(a.values[0])],&to=current[static_cast<unsigned>(b.values[0])];
        auto distance=[](unsigned a,unsigned b,unsigned period){unsigned d=a>b?a-b:b-a;return period>1?std::min(d,period-d):d;};
        unsigned roll=distance(from.roll_index,to.roll_index,settings.rolls);
        if(from.branch==to.branch){
            for(unsigned j=0;j<settings.externals;++j)if(distance(from.external_coordinates[j],to.external_coordinates[j],0)>1)return {false,0};
            if(roll>1 || distance(from.tilt_index,to.tilt_index,0)>1 || distance(from.azimuth_index,to.azimuth_index,settings.azimuths)>1)return {false,0};
        }
        double cost=settings.roll_weight*roll;
        for(unsigned j=0;j<settings.joints;++j){double d=std::abs(from.joints[j]-to.joints[j]);
            if(d>settings.jump[j]+1e-12)return {false,0};cost+=settings.weight[j]*d/settings.velocity[j];}
        return {true,cost};
    }
};
// A bounded all-pairs graph uses Descartes; large ladders retain indexed DP.
motionkit::LadderResult small_ladder(const Slices &layers,const motionkit::LadderSettings &settings,
        const std::vector<std::vector<double>> &costs) {
    // Validate with the same shared contract, including singleton handling.
    // The structured reference also supplies authoritative failure diagnostics.
    auto reference=motionkit::structured_ladder(layers,settings,costs);
    if(reference.failed!=UINT32_MAX || layers.size()==1)return reference;
    uint64_t tested=0;
    std::string log;console_bridge::ScopedDiagnosticSink sink(log);
    std::vector<descartes_light::WaypointSamplerD::ConstPtr> samplers;
    std::vector<descartes_light::EdgeEvaluatorD::ConstPtr> edges;
    for(unsigned i=0;i<layers.size();++i){samplers.push_back(std::make_shared<SmallSampler>(layers[i],costs[i].data(),settings,i==0));
        if(i)edges.push_back(std::make_shared<SmallEdge>(layers[i-1],layers[i],settings,tested));}
    try {
    descartes_light::LadderGraphSolverD solver(1);
    if(!solver.build(samplers,edges,{}))return reference;
    auto selected=solver.search();if(selected.trajectory.size()!=layers.size())return reference;
    motionkit::LadderResult result;result.backend=3;result.cost=selected.cost;result.tested_edges=tested+reference.tested_edges;
    for(const auto &state:selected.trajectory)result.route.push_back(static_cast<unsigned>(state->values[0]));
    return result;
    }catch(const std::bad_alloc &){throw;}catch(const std::exception &){return reference;}
}

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
        uint64_t pairs=0;bool small=sample_count>1;
        for(unsigned i=0;i<sample_count;++i){small=small && samples[i].candidate_count<=32;
            if(i)pairs+=uint64_t(samples[i-1].candidate_count)*samples[i].candidate_count;}
        small=small && pairs<=1000000;
        auto result=request->coarse_sample_stride ? motionkit::coarse_ladder(slices,settings,
            motionkit::CoarseLadderSettings{request->coarse_sample_stride,request->coarse_lattice_stride,
                request->corridor_radius,request->corridor_widenings},costs) : small ? small_ladder(slices,settings,costs) : motionkit::structured_ladder(slices,settings,costs);
        out_result->failed_sample=result.failed;out_result->cost=result.cost;out_result->tested_edges=result.tested_edges;out_result->backend=result.backend;
        if(result.failed!=UINT32_MAX){out_result->failed_distance=samples[result.failed].distance;
            out_result->failure_kind=result.no_candidates?1:2;return MK_ERROR_GENERATION;}
        for(unsigned i=0;i<sample_count;++i)out_indices[i]=samples[i].first_candidate+result.route[i];
        return MK_OK;
    }catch(const std::invalid_argument &){return MK_ERROR_INVALID_ARGUMENT;}
    catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
}
