#include "coarse_ladder.h"
#include <new>
#include <memory>
#include <set>
#include <tuple>
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
    const mk_lattice_candidate &candidate(unsigned i)const{return candidates[i];}
    CandidateSlice operator[](unsigned i)const{return {candidates?candidates+samples[i].first_candidate:nullptr,samples[i].candidate_count};}
};
struct CompactCandidate {
    const double *joints;const uint32_t *external_coordinates;
    unsigned roll_index,tilt_index,azimuth_index,branch;
};
struct CompactSlice {
    const double *joints;const uint32_t *coordinates;
    unsigned count,joint_count,external_count;
    unsigned size()const{return count;}
    bool empty()const{return count==0;}
    CompactCandidate operator[](unsigned i)const {
        auto c=coordinates+i*(external_count+4);
        return {joints+i*joint_count,c,c[external_count],c[external_count+1],c[external_count+2],c[external_count+3]};
    }
    struct Iterator {
        const CompactSlice *slice;unsigned index;
        CompactCandidate operator*()const{return (*slice)[index];}
        Iterator &operator++(){++index;return *this;}
        bool operator!=(const Iterator &other)const{return index!=other.index;}
    };
    Iterator begin()const{return {this,0};}
    Iterator end()const{return {this,count};}
};
struct CompactSlices {
    const mk_configuration_sample *samples;unsigned count;
    const double *joints;const uint32_t *coordinates;unsigned joint_count,external_count;
    unsigned size()const{return count;}
    bool empty()const{return count==0;}
    CompactCandidate candidate(unsigned i)const{return CompactSlice{joints,coordinates,0,joint_count,external_count}[i];}
    CompactSlice operator[](unsigned i)const {
        auto first=samples[i].first_candidate;
        return {joints?joints+size_t(first)*joint_count:nullptr,
            coordinates?coordinates+size_t(first)*(external_count+4):nullptr,
            samples[i].candidate_count,joint_count,external_count};
    }
};
using State=descartes_light::State<double>;
template<class Slice>
class SmallSampler final:public descartes_light::WaypointSamplerD {
    const Slice slice;const double *costs;const motionkit::LadderSettings &settings;bool first;
public:
    SmallSampler(Slice slice,const double *costs,const motionkit::LadderSettings &settings,bool first)
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
template<class Slice>
class SmallEdge final:public descartes_light::EdgeEvaluatorD {
    Slice prior,current;const motionkit::LadderSettings &settings;uint64_t &tested;
    unsigned layer;const std::function<bool(unsigned,unsigned,unsigned)> &allowed;
public:
    SmallEdge(Slice prior,Slice current,const motionkit::LadderSettings &settings,uint64_t &tested,
        unsigned layer,const std::function<bool(unsigned,unsigned,unsigned)> &allowed)
        :prior(prior),current(current),settings(settings),tested(tested),layer(layer),allowed(allowed){}
    std::pair<bool,double> evaluate(const State &a,const State &b)const override{
        if(allowed && !allowed(layer,static_cast<unsigned>(a.values[0]),static_cast<unsigned>(b.values[0])))return {false,0};
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
template<class Layers>
motionkit::LadderResult small_ladder(const Layers &layers,const motionkit::LadderSettings &settings,
        const std::vector<std::vector<double>> &costs,const std::function<bool(unsigned,unsigned,unsigned)> &allowed) {
    // Validate with the same shared contract, including singleton handling.
    // The structured reference also supplies authoritative failure diagnostics.
    auto reference=motionkit::structured_ladder(layers,settings,costs,allowed);
    if(reference.failed!=UINT32_MAX || layers.size()==1)return reference;
    uint64_t tested=0;
    std::string log;console_bridge::ScopedDiagnosticSink sink(log);
    std::vector<descartes_light::WaypointSamplerD::ConstPtr> samplers;
    std::vector<descartes_light::EdgeEvaluatorD::ConstPtr> edges;
    for(unsigned i=0;i<layers.size();++i){samplers.push_back(std::make_shared<SmallSampler<decltype(layers[0])>>(layers[i],costs[i].data(),settings,i==0));
        if(i)edges.push_back(std::make_shared<SmallEdge<decltype(layers[0])>>(layers[i-1],layers[i],settings,tested,i,allowed));}
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
template<class Layers>
static mk_result search_ladder(const Layers &slices,const mk_ladder_request *request,
    const mk_configuration_sample *samples,uint32_t sample_count,
    const double *state_costs,uint32_t candidate_count,
    const mk_ladder_edge *blocked_edges,uint32_t blocked_edge_count,
    uint32_t *out_indices,mk_ladder_result *out_result) {
    if(!request || request->struct_size!=sizeof(*request) || !samples || !sample_count ||
        (candidate_count && !state_costs) || !out_indices || !out_result)
        return MK_ERROR_INVALID_ARGUMENT;
    if(blocked_edge_count && !blocked_edges)return MK_ERROR_INVALID_ARGUMENT;
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
    for(unsigned i=0;i<candidate_count;++i){auto c=slices.candidate(i);auto cost=state_costs[i];
        if(c.roll_index>=request->roll_count || c.tilt_index>=request->tilt_count ||
            c.azimuth_index>=request->azimuth_count || std::isnan(cost) || cost<0)return MK_ERROR_INVALID_ARGUMENT;
        for(unsigned j=0;j<request->joint_count;++j)if(!std::isfinite(c.joints[j]))return MK_ERROR_INVALID_ARGUMENT;
    }
    for(unsigned i=0;i<blocked_edge_count;++i){const auto &e=blocked_edges[i];
        if(e.struct_size!=sizeof(e) || !e.sample || e.sample>=sample_count ||
            e.from_candidate>=samples[e.sample-1].candidate_count || e.to_candidate>=samples[e.sample].candidate_count)
            return MK_ERROR_INVALID_ARGUMENT;
    }
    try {
        std::set<std::tuple<unsigned,unsigned,unsigned>> excluded;
        for(unsigned i=0;i<blocked_edge_count;++i){const auto &e=blocked_edges[i];excluded.emplace(e.sample,e.from_candidate,e.to_candidate);}
        std::function<bool(unsigned,unsigned,unsigned)> allowed;
        if(!excluded.empty())allowed=[&](unsigned layer,unsigned from,unsigned to){return !excluded.count({layer,from,to});};
        motionkit::LadderSettings settings;settings.joints=request->joint_count;settings.externals=request->external_count;
        settings.rolls=request->roll_count;settings.tilts=request->tilt_count;settings.azimuths=request->azimuth_count;
        settings.roll_weight=request->roll_weight;
        for(unsigned j=0;j<MK_MAX_JOINTS;++j){settings.jump[j]=request->max_jump[j];settings.weight[j]=request->weights[j];
            settings.velocity[j]=request->velocity[j];settings.start[j]=request->start_joints[j];}
        std::vector<std::vector<double>> costs(sample_count);
        for(unsigned i=0;i<sample_count;++i)if(samples[i].candidate_count)
            costs[i].assign(state_costs+samples[i].first_candidate,state_costs+samples[i].first_candidate+samples[i].candidate_count);
        uint64_t pairs=0;bool small=sample_count>1;
        for(unsigned i=0;i<sample_count;++i){small=small && samples[i].candidate_count<=32;
            if(i)pairs+=uint64_t(samples[i-1].candidate_count)*samples[i].candidate_count;}
        small=small && pairs<=1000000;
        auto result=request->coarse_sample_stride ? motionkit::coarse_ladder(slices,settings,
            motionkit::CoarseLadderSettings{request->coarse_sample_stride,request->coarse_lattice_stride,
                request->corridor_radius,request->corridor_widenings},costs,allowed) : small ? small_ladder(slices,settings,costs,allowed) : motionkit::structured_ladder(slices,settings,costs,allowed);
        out_result->failed_sample=result.failed;out_result->cost=result.cost;out_result->tested_edges=result.tested_edges;out_result->backend=result.backend;
        if(result.failed!=UINT32_MAX){out_result->failed_distance=samples[result.failed].distance;
            out_result->failure_kind=result.no_candidates?1:2;return MK_ERROR_GENERATION;}
        for(unsigned i=0;i<sample_count;++i)out_indices[i]=samples[i].first_candidate+result.route[i];
        return MK_OK;
    }catch(const std::invalid_argument &){return MK_ERROR_INVALID_ARGUMENT;}
    catch(const std::bad_alloc &){return MK_ERROR_OUT_OF_MEMORY;}
}

extern "C" mk_result MK_CALL mk_search_ladder_filtered(const mk_ladder_request *request,
    const mk_configuration_sample *samples,uint32_t sample_count,
    const mk_lattice_candidate *candidates,const double *state_costs,uint32_t candidate_count,
    const mk_ladder_edge *blocked_edges,uint32_t blocked_edge_count,
    uint32_t *out_indices,mk_ladder_result *out_result) {
    if(candidate_count && !candidates)return MK_ERROR_INVALID_ARGUMENT;
    for(unsigned i=0;i<candidate_count;++i)if(candidates[i].struct_size!=sizeof(candidates[i]))return MK_ERROR_INVALID_ARGUMENT;
    return search_ladder(Slices{samples,sample_count,candidates},request,samples,sample_count,
        state_costs,candidate_count,blocked_edges,blocked_edge_count,out_indices,out_result);
}
extern "C" mk_result MK_CALL mk_search_ladder_compact_filtered(const mk_ladder_request *request,
    const mk_configuration_sample *samples,uint32_t sample_count,
    const double *joints,uint32_t joint_value_count,const uint32_t *coordinates,uint32_t coordinate_count,
    const double *state_costs,uint32_t candidate_count,
    const mk_ladder_edge *blocked_edges,uint32_t blocked_edge_count,
    uint32_t *out_indices,mk_ladder_result *out_result) {
    if(!request || request->struct_size!=sizeof(*request) ||
        uint64_t(candidate_count)*request->joint_count!=joint_value_count ||
        uint64_t(candidate_count)*(uint64_t(request->external_count)+4)!=coordinate_count ||
        (candidate_count && (!joints || !coordinates)))return MK_ERROR_INVALID_ARGUMENT;
    return search_ladder(CompactSlices{samples,sample_count,joints,coordinates,request->joint_count,request->external_count},
        request,samples,sample_count,state_costs,candidate_count,blocked_edges,blocked_edge_count,out_indices,out_result);
}

extern "C" mk_result MK_CALL mk_search_ladder(const mk_ladder_request *request,
    const mk_configuration_sample *samples,uint32_t sample_count,
    const mk_lattice_candidate *candidates,const double *state_costs,uint32_t candidate_count,
    uint32_t *out_indices,mk_ladder_result *out_result) {
    return mk_search_ladder_filtered(request,samples,sample_count,candidates,state_costs,candidate_count,
        nullptr,0,out_indices,out_result);
}
