#ifndef MOTIONKIT_COARSE_LADDER_H
#define MOTIONKIT_COARSE_LADDER_H
#include "structured_ladder.h"
namespace motionkit {
struct CoarseLadderSettings {
    unsigned sample_stride=10,lattice_stride=4,corridor_radius=2,widenings=2;
};
template<class Layers> struct CorridorLayers {
    const Layers &source;
    const LadderSettings &settings;
    const std::vector<unsigned> &anchors;
    const std::vector<mk_lattice_candidate> &centres;
    unsigned radius;
    mutable std::array<LadderLayer,2> cache;
    mutable std::array<std::vector<unsigned>,2> mapping;
    mutable std::vector<std::vector<unsigned>> all_mapping = std::vector<std::vector<unsigned>>(source.size());
    mutable std::array<unsigned,2> cached{{UINT32_MAX,UINT32_MAX}};
    unsigned size()const{return source.size();}
    bool empty()const{return source.empty();}
    const LadderLayer &operator[](unsigned layer)const {
        unsigned slot=layer%2;if(cached[slot]==layer)return cache[slot];
        auto upper=std::upper_bound(anchors.begin(),anchors.end(),layer);
        unsigned lo=upper==anchors.begin()?0:static_cast<unsigned>(upper-anchors.begin()-1);
        unsigned hi=std::min(lo+1,static_cast<unsigned>(anchors.size()-1));
        double t=lo==hi?0:double(layer-anchors[lo])/(anchors[hi]-anchors[lo]);
        auto &cells=cache[slot];auto &indices=mapping[slot];cells.clear();indices.clear();
        const auto &all=source[layer];
        auto close=[&](unsigned value,unsigned a,unsigned b,unsigned period){
            double delta=double(b)-a;
            if(period>1){if(delta>period*.5)delta-=period;if(delta<-double(period)*.5)delta+=period;}
            double d=std::abs(double(value)-(a+t*delta));
            if(period>1){d=std::fmod(d,period);d=std::min(d,period-d);}
            return d<=radius+1e-12;
        };
        for(unsigned i=0;i<all.size();++i){const auto &c=all[i],&a=centres[lo],&b=centres[hi];bool keep=true;
            for(unsigned j=0;j<settings.externals;++j)keep=keep && close(c.external_coordinates[j],a.external_coordinates[j],b.external_coordinates[j],0);
            keep=keep && close(c.roll_index,a.roll_index,b.roll_index,settings.rolls) &&
                close(c.tilt_index,a.tilt_index,b.tilt_index,0) && close(c.azimuth_index,a.azimuth_index,b.azimuth_index,settings.azimuths);
            if(keep){cells.push_back(c);indices.push_back(i);}
        }
        all_mapping[layer]=indices;cached[slot]=layer;return cells;
    }
};
/** Coarse corridor optimization is approximate. Fine edges always use original
 * jump limits. Disconnected corridors widen, then revert to the complete graph.
 * All branches and legal wrap representatives inside the corridor remain. */
template<class Layers> LadderResult coarse_ladder(const Layers &source,const LadderSettings &settings,
        const CoarseLadderSettings &options={},const std::vector<std::vector<double>> &state_cost={}) {
    if(options.sample_stride==0 || options.lattice_stride==0 || options.corridor_radius==0 || options.widenings>16)
        throw std::invalid_argument("Invalid coarse ladder resolution");
    if(source.empty())return structured_ladder(source,settings,state_cost);
    if(!state_cost.empty() && state_cost.size()!=source.size())throw std::invalid_argument("Coarse state-cost layer mismatch");
    unsigned stride=options.lattice_stride;
    std::vector<unsigned> anchors;
    for(unsigned i=0;i<source.size();){anchors.push_back(i);if(source.size()-1-i<=options.sample_stride)break;i+=options.sample_stride;}
    if(anchors.back()!=source.size()-1)anchors.push_back(source.size()-1);
    std::vector<LadderLayer> coarse;
    std::vector<std::vector<unsigned>> mapping;
    std::vector<std::vector<double>> costs;
    for(auto layer:anchors){const auto &all=source[layer];LadderLayer cells;std::vector<unsigned> indices;std::vector<double> prices;
        if(!state_cost.empty() && state_cost[layer].size()!=all.size())throw std::invalid_argument("Coarse state-cost candidate mismatch");
        for(unsigned i=0;i<all.size();++i){auto c=all[i];bool keep=all.size()==1;
            if(!keep){keep=c.roll_index%stride==0 && c.tilt_index%stride==0 && c.azimuth_index%stride==0;
                for(unsigned j=0;j<settings.externals;++j)keep=keep && c.external_coordinates[j]%stride==0;}
            if(!keep)continue;
            for(unsigned j=0;j<settings.externals;++j)c.external_coordinates[j]/=stride;
            c.roll_index/=stride;c.tilt_index/=stride;c.azimuth_index/=stride;
            cells.push_back(c);indices.push_back(i);if(!state_cost.empty())prices.push_back(state_cost[layer][i]);
        }
        coarse.push_back(std::move(cells));mapping.push_back(std::move(indices));if(!state_cost.empty())costs.push_back(std::move(prices));
    }
    auto coarse_settings=settings;
    coarse_settings.rolls=(settings.rolls-1)/stride+1;coarse_settings.tilts=(settings.tilts-1)/stride+1;coarse_settings.azimuths=(settings.azimuths-1)/stride+1;
    coarse_settings.roll_weight*=stride;
    for(unsigned j=0;j<settings.joints;++j)coarse_settings.jump[j]*=options.sample_stride;
    auto route=structured_ladder(coarse,coarse_settings,costs);
    if(route.failed!=UINT32_MAX)return structured_ladder(source,settings,state_cost);
    std::vector<mk_lattice_candidate> centres;
    for(unsigned i=0;i<anchors.size();++i)centres.push_back(source[anchors[i]][mapping[i][route.route[i]]]);
    uint64_t tested=route.tested_edges;
    unsigned radius=options.corridor_radius;
    for(unsigned attempt=0;attempt<=options.widenings;++attempt){
        CorridorLayers<Layers> corridor{source,settings,anchors,centres,radius};
        std::vector<std::vector<double>> fine_costs;
        if(!state_cost.empty())for(unsigned i=0;i<source.size();++i){corridor[i];std::vector<double> prices;
            if(state_cost[i].size()!=source[i].size())throw std::invalid_argument("Fine state-cost candidate mismatch");
            for(auto index:corridor.mapping[i%2])prices.push_back(state_cost[i][index]);fine_costs.push_back(std::move(prices));}
        auto fine=structured_ladder(corridor,settings,fine_costs);tested+=fine.tested_edges;
        if(fine.failed==UINT32_MAX){for(unsigned i=0;i<fine.route.size();++i)fine.route[i]=corridor.all_mapping[i][fine.route[i]];
            fine.tested_edges=tested;fine.backend=2;return fine;}
        if(radius>UINT32_MAX/2)break;radius*=2;
    }
    auto complete=structured_ladder(source,settings,state_cost);complete.tested_edges+=tested;return complete;
}
}
#endif
