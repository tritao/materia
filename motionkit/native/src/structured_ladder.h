#ifndef MOTIONKIT_STRUCTURED_LADDER_H
#define MOTIONKIT_STRUCTURED_LADDER_H
#include "motionkit.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace motionkit {
struct LadderSettings {
    unsigned joints=0, externals=0, rolls=1, tilts=1, azimuths=1;
    std::array<double,MK_MAX_JOINTS> jump{}, weight{}, velocity{}, start{};
    double roll_weight=0;
};
struct LadderResult {
    std::vector<unsigned> route;
    double cost=std::numeric_limits<double>::infinity();
    unsigned failed=UINT32_MAX;
    bool no_candidates=false;
    uint64_t tested_edges=0;
};
using LadderLayer=std::vector<mk_lattice_candidate>;
struct LatticeKey {
    unsigned branch;
    std::array<unsigned,MK_MAX_JOINTS+3> cell{};
    bool operator==(const LatticeKey &b) const {return branch==b.branch && cell==b.cell;}
};
struct LatticeHash {
    size_t operator()(const LatticeKey &a) const {
        size_t h=a.branch;for(auto v:a.cell)h=(h^v)*1099511628211ull;return h;
    }
};
inline LatticeKey lattice_key(const mk_lattice_candidate &c,unsigned external) {
    LatticeKey key{};key.branch=c.branch;
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
/** Exact dynamic programming over the structured graph, retaining only two
 * cost layers. State costs are supplied by the caller (margin/posture/roll).
 * Coordinate keys deliberately omit wrap numbers: FK representatives can cross
 * a wrap seam between samples; physical joint jumps decide connectivity. */
inline LadderResult structured_ladder(const std::vector<LadderLayer> &layers,
        const LadderSettings &s,const std::vector<std::vector<double>> &state_cost={}) {
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
                for(unsigned d=0;d<nd;++d)k.cell[d]=static_cast<int64_t>(std::floor(c.joints[dimensions[d]]/s.jump[dimensions[d]]));return k;};
            for(unsigned i=0;i<prior.size();++i)if(std::isfinite(previous[i])){
                lattice[lattice_key(prior[i],s.externals)].push_back(i);joints[joint_key(prior[i])].push_back(i);
            }
            for(unsigned i=0;i<current.size();++i){
                const auto &c=current[i];
                auto consider=[&](unsigned p){double cost=previous[p]+edge(prior[p].joints,c.joints);
                    unsigned rd=std::abs(static_cast<int>(prior[p].roll_index)-static_cast<int>(c.roll_index));
                    if(s.rolls>1)rd=std::min(rd,s.rolls-rd);
                    cost+=s.roll_weight*rd;
                    if(cost<costs[i] || (cost==costs[i] && p<predecessor[layer][i])){costs[i]=cost;predecessor[layer][i]=p;}};
                auto key=lattice_key(c,s.externals);
                auto visit=[&](auto &&self,unsigned dimension)->void{
                    if(dimension==s.externals+3){auto found=lattice.find(key);if(found!=lattice.end())for(auto p:found->second)consider(p);return;}
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
                };flips(flips,0);
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
