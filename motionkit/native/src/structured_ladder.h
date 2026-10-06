#ifndef MOTIONKIT_STRUCTURED_LADDER_H
#define MOTIONKIT_STRUCTURED_LADDER_H
#include "motionkit.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace motionkit {
struct LadderSettings {
    unsigned joints=0, externals=0, rolls=1, tilts=1, azimuths=1;
    std::array<double,MK_MAX_JOINTS> jump{}, weight{}, velocity{}, start{};
    double roll_weight=0;
};
struct LadderResult {
    unsigned backend=1;
    std::vector<unsigned> route;
    double cost=std::numeric_limits<double>::infinity();
    unsigned failed=UINT32_MAX;
    bool no_candidates=false;
    uint64_t tested_edges=0;
};
using LadderLayer=std::vector<mk_lattice_candidate>;
struct LatticeKey {
    unsigned branch, dimensions=0;
    std::array<unsigned,MK_MAX_JOINTS+3> cell{};
    bool operator==(const LatticeKey &b) const {return branch==b.branch && dimensions==b.dimensions && std::equal(cell.begin(),cell.begin()+dimensions,b.cell.begin());}
};
struct LatticeHash {
    size_t operator()(const LatticeKey &a) const {
        size_t h=a.branch;for(unsigned i=0;i<a.dimensions;++i)h=(h^a.cell[i])*1099511628211ull;return h;
    }
};
inline LatticeKey lattice_key(const mk_lattice_candidate &c,unsigned external) {
    LatticeKey key{};key.branch=c.branch;key.dimensions=external+3;
    for(unsigned i=0;i<external;++i)key.cell[i]=c.external_coordinates[i];
    key.cell[external]=c.roll_index;key.cell[external+1]=c.tilt_index;key.cell[external+2]=c.azimuth_index;
    return key;
}
struct JointKey {
    std::array<int64_t,3> cell{};
    bool operator==(const JointKey &b)const{return cell==b.cell;}
};
struct JointHash {
    size_t operator()(const JointKey &a)const{
        size_t h=0;for(auto v:a.cell)h=(h^static_cast<uint64_t>(v))*1099511628211ull;return h;
    }
};
inline void validate_ladder_settings(const LadderSettings &s) {
    if(s.joints==0 || s.joints>MK_MAX_JOINTS || s.externals>MK_MAX_JOINTS ||
        s.rolls==0 || s.tilts==0 || s.azimuths==0 ||
        !std::isfinite(s.roll_weight) || s.roll_weight<0)
        throw std::invalid_argument("Invalid ladder dimensions or roll cost");
    for(unsigned j=0;j<s.joints;++j)
        if(!std::isfinite(s.jump[j]) || s.jump[j]<=0 || !std::isfinite(s.velocity[j]) || s.velocity[j]<=0 ||
            !std::isfinite(s.weight[j]) || s.weight[j]<0 || !std::isfinite(s.start[j]))
            throw std::invalid_argument("Ladder requires finite joints, positive jumps/speeds and nonnegative weights");
}
inline void validate_ladder_candidate(const mk_lattice_candidate &c,const LadderSettings &s) {
    if(c.roll_index>=s.rolls || c.tilt_index>=s.tilts || c.azimuth_index>=s.azimuths)
        throw std::invalid_argument("Ladder orientation coordinate out of range");
    for(unsigned j=0;j<s.joints;++j)
        if(!std::isfinite(c.joints[j]) || std::abs(c.joints[j])>s.jump[j]*0x1p60)
            throw std::invalid_argument("Ladder joint cannot be represented in the spatial hash");
}
/** Exact dynamic programming over the structured graph, retaining only two
 * cost layers. A layer provider must retain current and previous layers while
 * they are referenced (a two-slot cache is sufficient). State costs are supplied by the caller (margin/posture/roll).
 * Coordinate keys deliberately omit wrap numbers: FK representatives can cross
 * a wrap seam between samples; physical joint jumps decide connectivity. */
template<class Layers>
inline LadderResult structured_ladder(const Layers &layers,
        const LadderSettings &s,const std::vector<std::vector<double>> &state_cost={}) {
    validate_ladder_settings(s);
    if(!state_cost.empty() && state_cost.size()!=layers.size())
        throw std::invalid_argument("Ladder state-cost layer count mismatch");
    LadderResult out;
    if(layers.empty())return out;
    std::vector<std::vector<unsigned>> predecessor(layers.size());
    std::vector<double> previous;
    auto edge=[&](const double *a,const double *b){
        ++out.tested_edges;double cost=0;
        for(unsigned j=0;j<s.joints;++j){double d=std::abs(a[j]-b[j]);
            if(d>s.jump[j]+1e-12)return std::numeric_limits<double>::infinity();
            cost+=s.weight[j]*d/s.velocity[j];}
        return cost;
    };
    for(unsigned layer=0;layer<layers.size();++layer){
        const auto &current=layers[layer];
        if(!state_cost.empty() && state_cost[layer].size()!=current.size())
            throw std::invalid_argument("Ladder state-cost candidate count mismatch");
        for(unsigned i=0;i<current.size();++i){const auto &c=current[i];
            validate_ladder_candidate(c,s);
            if(!state_cost.empty() && (std::isnan(state_cost[layer][i]) || state_cost[layer][i]<0))
                throw std::invalid_argument("Ladder state costs must be nonnegative");
        }
        if(current.empty()){out.failed=layer;out.no_candidates=true;return out;}
        std::vector<double> costs(current.size(),std::numeric_limits<double>::infinity());
        predecessor[layer].assign(current.size(),UINT32_MAX);
        if(layer==0){
            for(unsigned i=0;i<current.size();++i){
                // Free-start travel is a cost, not a task-path jump constraint.
                costs[i]=0;for(unsigned j=0;j<s.joints;++j)
                    costs[i]+=s.weight[j]*std::abs(current[i].joints[j]-s.start[j])/s.velocity[j];
            }
        }else{
            const auto &prior=layers[layer-1];
            std::unordered_map<LatticeKey,std::vector<unsigned>,LatticeHash> lattice;
            std::unordered_map<JointKey,std::vector<unsigned>,JointHash> joints;
            // Hash the three most discriminating joint coordinates. The other
            // coordinates are checked exactly; 27 buckets contain every nearby
            // cross-branch state regardless of the number of joints.
            std::array<unsigned,3> dimensions{};
            std::vector<std::pair<double,unsigned>> spread;
            for(unsigned j=0;j<s.joints;++j){double lo=prior[0].joints[j],hi=lo;
                for(const auto &c:prior){lo=std::min(lo,c.joints[j]);hi=std::max(hi,c.joints[j]);}
                spread.push_back({(hi-lo)/s.jump[j],j});}
            std::sort(spread.rbegin(),spread.rend());
            unsigned nd=std::min(3u,s.joints);for(unsigned d=0;d<nd;++d)dimensions[d]=spread[d].second;
            auto joint_key=[&](const mk_lattice_candidate &c){JointKey k;
                for(unsigned d=0;d<nd;++d)k.cell[d]=static_cast<int64_t>(std::floor(c.joints[dimensions[d]]/(s.jump[dimensions[d]]+1e-12)));return k;};
            for(unsigned i=0;i<prior.size();++i)if(std::isfinite(previous[i])){
                lattice[lattice_key(prior[i],s.externals)].push_back(i);joints[joint_key(prior[i])].push_back(i);
            }
            struct Bounds {std::array<double,MK_MAX_JOINTS> lo,hi;};
            auto bounds=[&](const auto &cells){std::unordered_map<unsigned,Bounds> result;
                for(const auto &c:cells){auto found=result.find(c.branch);
                    if(found==result.end()){Bounds b;for(unsigned j=0;j<s.joints;++j)b.lo[j]=b.hi[j]=c.joints[j];result.emplace(c.branch,b);}
                    else for(unsigned j=0;j<s.joints;++j){found->second.lo[j]=std::min(found->second.lo[j],c.joints[j]);found->second.hi[j]=std::max(found->second.hi[j],c.joints[j]);}}
                return result;};
            auto prior_bounds=bounds(prior),current_bounds=bounds(current);
            std::unordered_map<unsigned,bool> can_flip;
            for(const auto &c:current_bounds){bool possible=false;
                for(const auto &p:prior_bounds)if(c.first!=p.first){bool close=true;
                    for(unsigned j=0;j<s.joints;++j)if(c.second.lo[j]-p.second.hi[j]>s.jump[j]+1e-12 || p.second.lo[j]-c.second.hi[j]>s.jump[j]+1e-12)close=false;
                    possible=possible || close;}can_flip[c.first]=possible;}
            for(unsigned i=0;i<current.size();++i){
                const auto &c=current[i];
                auto consider=[&](unsigned p){double cost=previous[p]+edge(prior[p].joints,c.joints);
                    unsigned rd=prior[p].roll_index>c.roll_index ? prior[p].roll_index-c.roll_index : c.roll_index-prior[p].roll_index;
                    if(s.rolls>1)rd=std::min(rd,s.rolls-rd);
                    cost+=s.roll_weight*rd;
                    if(cost<costs[i] || (cost==costs[i] && p<predecessor[layer][i])){costs[i]=cost;predecessor[layer][i]=p;}};
                auto key=lattice_key(c,s.externals);
                auto visit=[&](auto &&self,unsigned dimension)->void{
                    if(dimension==s.externals+3){auto found=lattice.find(key);if(found!=lattice.end())for(auto p:found->second)consider(p);return;}
                    if(dimension>=s.externals){unsigned count=dimension==s.externals?s.rolls:dimension==s.externals+1?s.tilts:s.azimuths;
                        if(count==1){self(self,dimension+1);return;}}
                    unsigned original=key.cell[dimension];
                    unsigned period=dimension==s.externals?s.rolls:dimension==s.externals+2?s.azimuths:0;
                    std::array<unsigned,3> values{};unsigned n=0;
                    for(int delta=-1;delta<=1;++delta){int64_t v=static_cast<int64_t>(original)+delta;
                        if(period>1)v=(v+period)%period;else if(v<0 || v>UINT32_MAX)continue;
                        bool duplicate=false;for(unsigned k=0;k<n;++k)if(values[k]==v)duplicate=true;
                        if(!duplicate)values[n++]=static_cast<unsigned>(v);}
                    for(unsigned k=0;k<n;++k){key.cell[dimension]=values[k];self(self,dimension+1);}key.cell[dimension]=original;
                };visit(visit,0);
                auto hash=joint_key(c);
                auto flips=[&](auto &&self,unsigned d)->void{
                    if(d==nd){auto found=joints.find(hash);if(found!=joints.end())for(auto p:found->second)if(prior[p].branch!=c.branch)consider(p);return;}
                    auto original=hash.cell[d];for(int delta=-1;delta<=1;++delta){hash.cell[d]=original+delta;self(self,d+1);}hash.cell[d]=original;
                };if(can_flip[c.branch])flips(flips,0);
            }
        }
        bool reachable=false;
        for(unsigned i=0;i<costs.size();++i){if(!state_cost.empty())costs[i]+=state_cost[layer][i];reachable=reachable || std::isfinite(costs[i]);}
        if(!reachable){out.failed=layer;return out;}previous=std::move(costs);
    }
    unsigned best=static_cast<unsigned>(std::min_element(previous.begin(),previous.end())-previous.begin());out.cost=previous[best];out.route.resize(layers.size());
    for(unsigned layer=layers.size();layer-->0;){out.route[layer]=best;if(layer)best=predecessor[layer][best];}
    return out;
}
}
#endif
