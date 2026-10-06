#ifdef NDEBUG
#undef NDEBUG
#endif
#include "../src/coarse_ladder.h"
#include <cassert>
#include <random>
using namespace motionkit;
struct CachedLayers {
    const std::vector<LadderLayer> &source;
    mutable std::array<LadderLayer,2> cache;
    mutable std::array<unsigned,2> index{{UINT32_MAX,UINT32_MAX}};
    size_t size()const{return source.size();}
    bool empty()const{return source.empty();}
    const LadderLayer &operator[](unsigned layer)const{
        unsigned slot=layer%2;if(index[slot]!=layer){cache[slot]=source[layer];index[slot]=layer;}return cache[slot];
    }
};
// No fixed-size joint/wrap arrays or owning candidate records in this source.
struct CompactCandidate {
    const double *joints;
    const unsigned *external_coordinates;
    unsigned roll_index,tilt_index,azimuth_index,branch;
};
struct CompactLayer {
    std::vector<double> joints;
    std::vector<unsigned> cells;
    unsigned joint_count,external_count;
    unsigned size()const{return cells.size()/(external_count+4);}
    bool empty()const{return size()==0;}
    CompactCandidate operator[](unsigned i)const {
        auto c=cells.data()+i*(external_count+4);
        return {joints.data()+i*joint_count,c,c[external_count],c[external_count+1],c[external_count+2],c[external_count+3]};
    }
    struct Iterator {
        const CompactLayer *layer;unsigned index;
        CompactCandidate operator*()const{return (*layer)[index];}
        Iterator &operator++(){++index;return *this;}
        bool operator!=(const Iterator &other)const{return index!=other.index;}
    };
    Iterator begin()const{return {this,0};}
    Iterator end()const{return {this,size()};}
};
std::vector<CompactLayer> compact_layers(const std::vector<LadderLayer> &source,const LadderSettings &settings) {
    std::vector<CompactLayer> result;
    for(const auto &layer:source){CompactLayer compact{{},{},settings.joints,settings.externals};
        for(const auto &c:layer){
            for(unsigned j=0;j<settings.joints;++j)compact.joints.push_back(c.joints[j]);
            for(unsigned j=0;j<settings.externals;++j)compact.cells.push_back(c.external_coordinates[j]);
            for(auto value:{c.roll_index,c.tilt_index,c.azimuth_index,c.branch})compact.cells.push_back(value);
        }result.push_back(std::move(compact));
    }return result;
}
int main(){
    LadderSettings s;s.joints=4;s.externals=1;s.rolls=4;s.roll_weight=.07;
    for(unsigned j=0;j<4;++j){s.jump[j]=.5;s.weight[j]=1+j;s.velocity[j]=1;}
    // Exclude a transition, not its endpoints: the blocked destination is
    // still reachable from a different predecessor in the optimal route.
    LadderSettings edge_settings;edge_settings.joints=1;
    edge_settings.jump[0]=1;edge_settings.weight[0]=edge_settings.velocity[0]=1;
    std::vector<LadderLayer> edge_layers(3);
    for(double value:{0.0,0.2}){mk_lattice_candidate c{};c.joints[0]=value;edge_layers[0].push_back(c);}
    for(unsigned layer=1;layer<3;++layer){mk_lattice_candidate c{};c.joints[0]=0.0;edge_layers[layer].push_back(c);}
    auto exclude=[](unsigned layer,unsigned from,unsigned to){return !(layer==1 && from==0 && to==0);};
    auto edge_route=structured_ladder(edge_layers,edge_settings,{},exclude);
    assert(edge_route.failed==UINT32_MAX && edge_route.route==std::vector<unsigned>({1,0,0}));
    assert(std::abs(edge_route.cost-.4)<1e-12);
    CachedLayers edge_stream{edge_layers};
    auto streamed_edge_route=structured_ladder(edge_stream,edge_settings,{},exclude);
    assert(streamed_edge_route.route==edge_route.route && streamed_edge_route.cost==edge_route.cost);
    for(unsigned stride:{1u,2u}){
        auto coarse_edge_route=coarse_ladder(edge_layers,edge_settings,CoarseLadderSettings{stride,1,2,1},{},exclude);
        assert(coarse_edge_route.failed==UINT32_MAX && coarse_edge_route.route==edge_route.route);
        assert(coarse_edge_route.cost==edge_route.cost);
    }
    auto remapped_layers=edge_layers;auto remapped_settings=edge_settings;remapped_settings.rolls=4;
    for(auto &layer:remapped_layers){mk_lattice_candidate decoy{};decoy.roll_index=3;decoy.joints[0]=3;
        layer.insert(layer.begin(),decoy);}
    auto remapped_exclude=[](unsigned layer,unsigned from,unsigned to){return !(layer==1 && from==1 && to==1);};
    auto remapped_route=coarse_ladder(remapped_layers,remapped_settings,CoarseLadderSettings{2,2,1,0},{},remapped_exclude);
    assert(remapped_route.failed==UINT32_MAX && remapped_route.backend==2);
    assert(remapped_route.route==std::vector<unsigned>({2,1,1}) && std::abs(remapped_route.cost-.4)<1e-12);
    auto impossible_edge=structured_ladder(edge_layers,edge_settings,{},
        [](unsigned layer,unsigned,unsigned){return layer!=1;});
    assert(impossible_edge.failed==1 && !impossible_edge.no_candidates);
    // Exercise the large-external fallback with an actual transition.
    auto wide=edge_settings;wide.externals=6;
    auto wide_route=structured_ladder(edge_layers,wide,{},exclude);
    assert(wide_route.route==edge_route.route && wide_route.cost==edge_route.cost);
    std::mt19937 random(177);std::uniform_real_distribution<double> q(-1,1);
    for(unsigned trial=0;trial<100;++trial){
        std::vector<LadderLayer> layers(4);std::vector<std::vector<double>> states(4);
        for(unsigned l=0;l<4;++l)for(unsigned i=0;i<16;++i){
            mk_lattice_candidate c{};c.branch=i%2;c.external_coordinates[0]=(i/2)%2;c.roll_index=i/4;
            for(unsigned j=0;j<4;++j)c.joints[j]=q(random)*(trial<50?.2:1.0);layers[l].push_back(c);states[l].push_back(.01*i);
        }
        auto result=structured_ladder(layers,s,states);
        auto full_key=structured_ladder_with_key<MK_MAX_JOINTS+3>(layers,s,states);
        assert(full_key.route==result.route && full_key.cost==result.cost && full_key.failed==result.failed);
        assert(full_key.tested_edges==result.tested_edges);
        auto compact=compact_layers(layers,s);
        auto compact_result=structured_ladder(compact,s,states);
        assert(compact_result.route==result.route && compact_result.cost==result.cost);
        assert(compact_result.failed==result.failed && compact_result.tested_edges==result.tested_edges);
        auto compact_coarse=coarse_ladder(compact,s,CoarseLadderSettings{1,1,10,0},states);
        assert(compact_coarse.route==result.route && compact_coarse.cost==result.cost && compact_coarse.failed==result.failed);
        auto coarse_exact=coarse_ladder(layers,s,CoarseLadderSettings{1,1,10,0},states);
        assert(coarse_exact.failed==result.failed && coarse_exact.cost==result.cost && coarse_exact.route==result.route);
        CachedLayers streamed{layers};auto streamed_result=structured_ladder(streamed,s,states);
        assert(streamed_result.failed==result.failed && streamed_result.route==result.route);
        assert(streamed_result.cost==result.cost && streamed_result.tested_edges==result.tested_edges);

        std::vector<double> old;
        unsigned failed=UINT32_MAX;
        for(unsigned l=0;l<4;++l){std::vector<double> next(16,INFINITY);
            for(unsigned i=0;i<16;++i){auto &c=layers[l][i];
                if(l==0){next[i]=0;for(unsigned j=0;j<4;++j)next[i]+=s.weight[j]*std::abs(c.joints[j]);}
                else for(unsigned p=0;p<16;++p){auto &a=layers[l-1][p];bool legal=true;
                    unsigned rd=std::abs(int(a.roll_index)-int(c.roll_index));rd=std::min(rd,s.rolls-rd);
                    if(a.branch==c.branch && (rd>1 || std::abs(int(a.external_coordinates[0])-int(c.external_coordinates[0]))>1))legal=false;
                    double cost=old[p]+s.roll_weight*rd;for(unsigned j=0;j<4;++j){double d=std::abs(a.joints[j]-c.joints[j]);if(d>s.jump[j]+1e-12)legal=false;cost+=s.weight[j]*d;}
                    if(legal)next[i]=std::min(next[i],cost);
                }
                next[i]+=states[l][i];
            }
            old=next;if(!std::isfinite(*std::min_element(old.begin(),old.end()))){failed=l;break;}
        }
        mk_ladder_request request{};request.struct_size=sizeof(request);request.joint_count=s.joints;request.external_count=s.externals;
        request.roll_count=s.rolls;request.tilt_count=request.azimuth_count=1;request.roll_weight=s.roll_weight;
        for(unsigned j=0;j<s.joints;++j){request.max_jump[j]=s.jump[j];request.velocity[j]=s.velocity[j];request.weights[j]=s.weight[j];}
        std::vector<mk_lattice_candidate> flat;std::vector<double> prices;mk_configuration_sample samples[4]{};
        for(unsigned l=0;l<4;++l){samples[l].struct_size=sizeof(samples[l]);samples[l].distance=.1*l;samples[l].first_candidate=flat.size();samples[l].candidate_count=layers[l].size();
            for(unsigned i=0;i<layers[l].size();++i){auto c=layers[l][i];c.struct_size=sizeof(c);flat.push_back(c);prices.push_back(states[l][i]);}}
        unsigned indices[4];mk_ladder_result report;
        auto status=mk_search_ladder(&request,samples,4,flat.data(),prices.data(),flat.size(),indices,&report);
        assert(status==(failed==UINT32_MAX?MK_OK:MK_ERROR_GENERATION));assert(report.failed_sample==failed);
        std::vector<double> compact_joints;std::vector<uint32_t> compact_cells;
        for(const auto &layer:compact){compact_joints.insert(compact_joints.end(),layer.joints.begin(),layer.joints.end());
            compact_cells.insert(compact_cells.end(),layer.cells.begin(),layer.cells.end());}
        unsigned compact_indices[4];mk_ladder_result compact_report;
        auto compact_status=mk_search_ladder_compact_filtered(&request,samples,4,
            compact_joints.data(),compact_joints.size(),compact_cells.data(),compact_cells.size(),
            prices.data(),flat.size(),nullptr,0,compact_indices,&compact_report);
        assert(compact_status==status && compact_report.failed_sample==report.failed_sample);
        assert(compact_report.cost==report.cost && compact_report.backend==report.backend);
        if(status==MK_OK)assert(std::equal(indices,indices+4,compact_indices));
        assert(mk_search_ladder_compact_filtered(&request,samples,4,
            compact_joints.data(),compact_joints.size()-1,compact_cells.data(),compact_cells.size(),
            prices.data(),flat.size(),nullptr,0,compact_indices,&compact_report)==MK_ERROR_INVALID_ARGUMENT);

        if(status==MK_OK){assert(report.backend==3);assert(std::abs(report.cost-result.cost)<1e-10);}
        if(status==MK_OK){
            mk_ladder_edge blocked{sizeof(mk_ladder_edge),1,indices[0]-samples[0].first_candidate,indices[1]-samples[1].first_candidate};
            auto allowed=[&](unsigned layer,unsigned from,unsigned to){return !(layer==blocked.sample && from==blocked.from_candidate && to==blocked.to_candidate);};
            auto expected=structured_ladder(layers,s,states,allowed);
            auto compact_filtered=structured_ladder(compact,s,states,allowed);
            assert(compact_filtered.route==expected.route && compact_filtered.cost==expected.cost && compact_filtered.failed==expected.failed);
            auto compact_filtered_coarse=coarse_ladder(compact,s,CoarseLadderSettings{2,1,10,0},states,allowed);
            assert(compact_filtered_coarse.route==expected.route && compact_filtered_coarse.cost==expected.cost && compact_filtered_coarse.failed==expected.failed);
            for(unsigned coarse_stride:{0u,2u}){
                request.coarse_sample_stride=coarse_stride;request.coarse_lattice_stride=1;request.corridor_radius=10;request.corridor_widenings=0;
                auto filtered=mk_search_ladder_filtered(&request,samples,4,flat.data(),prices.data(),flat.size(),&blocked,1,indices,&report);
                assert(filtered==(expected.failed==UINT32_MAX?MK_OK:MK_ERROR_GENERATION));
                assert(report.failed_sample==expected.failed);
                auto compact_filtered_status=mk_search_ladder_compact_filtered(&request,samples,4,
                    compact_joints.data(),compact_joints.size(),compact_cells.data(),compact_cells.size(),
                    prices.data(),flat.size(),&blocked,1,compact_indices,&compact_report);
                assert(compact_filtered_status==filtered && compact_report.failed_sample==report.failed_sample);
                assert(compact_report.cost==report.cost && compact_report.backend==report.backend);
                if(filtered==MK_OK)assert(std::equal(indices,indices+4,compact_indices));
                if(filtered==MK_OK){assert(report.backend==(coarse_stride?2u:3u));assert(std::abs(report.cost-expected.cost)<1e-10);
                    assert(indices[0]-samples[0].first_candidate!=blocked.from_candidate || indices[1]-samples[1].first_candidate!=blocked.to_candidate);}
            }
            blocked.sample=0;
            assert(mk_search_ladder_filtered(&request,samples,4,flat.data(),prices.data(),flat.size(),&blocked,1,indices,&report)==MK_ERROR_INVALID_ARGUMENT);
        }
        assert(result.failed==failed);
        if(failed==UINT32_MAX)assert(std::abs(result.cost-*std::min_element(old.begin(),old.end()))<1e-10);
    }
    // The cheapest first branch dies; the globally reachable branch wins.
    std::vector<LadderLayer> layers(3);for(unsigned l=0;l<3;++l){mk_lattice_candidate c{};c.branch=1;c.joints[0]=.3+.1*l;layers[l].push_back(c);}
    mk_lattice_candidate dead{};dead.branch=0;layers[0].insert(layers[0].begin(),dead);
    s.jump[0]=.15;auto route=structured_ladder(layers,s);assert(route.failed==UINT32_MAX && route.route[0]==1);
    layers[1].clear();route=structured_ladder(layers,s);assert(route.failed==1 && route.no_candidates);
    layers[1].push_back(dead);layers[1][0].joints[0]=2;route=structured_ladder(layers,s);assert(route.failed==1 && !route.no_candidates);
    // Adjacent periodic roll cells connect across the seam even when inverse
    // representatives change wrap labels, provided physical joints stay close.
    layers.assign(2,{});mk_lattice_candidate a{},b{};
    a.roll_index=3;a.joints[0]=.3;a.wraps[0]=0;
    b.roll_index=0;b.joints[0]=.4;b.wraps[0]=1;
    layers[0].push_back(a);layers[1].push_back(b);
    route=structured_ladder(layers,s);assert(route.failed==UINT32_MAX);
    assert(std::abs(route.cost-(.4+s.roll_weight))<1e-12);
    auto invalid=[&](const LadderSettings &settings,const std::vector<std::vector<double>> &costs={}){
        bool rejected=false;try{structured_ladder(layers,settings,costs);}catch(const std::invalid_argument &){rejected=true;}assert(rejected);
    };
    auto bad=s;bad.jump[0]=0;invalid(bad);
    bad=s;bad.velocity[0]=NAN;invalid(bad);
    bad=s;bad.externals=MK_MAX_JOINTS+1;invalid(bad);
    invalid(s,{{0}});invalid(s,{{0},{NAN}});
    layers[1][0].roll_index=s.rolls;invalid(s);
    layers[1][0].roll_index=0;layers[1][0].joints[0]=INFINITY;invalid(s);

    mk_ladder_request request{};request.struct_size=sizeof(request);request.joint_count=1;
    request.roll_count=request.tilt_count=request.azimuth_count=1;
    request.max_jump[0]=.2;request.velocity[0]=request.weights[0]=1;
    mk_configuration_sample samples[2]{};
    for(unsigned i=0;i<2;++i){samples[i].struct_size=sizeof(samples[i]);samples[i].distance=.1*i;samples[i].first_candidate=i;samples[i].candidate_count=1;}
    mk_lattice_candidate cells[2]{};
    for(auto &c:cells)c.struct_size=sizeof(c);
    cells[1].joints[0]=.1;double costs[]={0,.03};unsigned selected[2];mk_ladder_result report;
    assert(mk_search_ladder(&request,samples,2,cells,costs,2,selected,&report)==MK_OK);
    assert(selected[0]==0 && selected[1]==1 && std::abs(report.cost-.13)<1e-12 && report.failed_sample==UINT32_MAX && report.backend==3);
    request.coarse_sample_stride=2;request.coarse_lattice_stride=2;request.corridor_radius=1;
    assert(mk_search_ladder(&request,samples,2,cells,costs,2,selected,&report)==MK_OK);
    assert(selected[0]==0 && selected[1]==1 && std::abs(report.cost-.13)<1e-12 && report.backend==2);
    request.coarse_lattice_stride=0;
    assert(mk_search_ladder(&request,samples,2,cells,costs,2,selected,&report)==MK_ERROR_INVALID_ARGUMENT);
    request.coarse_lattice_stride=2;
    cells[1].joints[0]=1;
    assert(mk_search_ladder(&request,samples,2,cells,costs,2,selected,&report)==MK_ERROR_GENERATION);
    assert(report.failed_sample==1 && report.failure_kind==2 && report.failed_distance==.1 && selected[0]==UINT32_MAX);
    samples[1].candidate_count=0;
    assert(mk_search_ladder(&request,samples,2,cells,costs,2,selected,&report)==MK_ERROR_GENERATION);
    assert(report.failed_sample==1 && report.failure_kind==1);
    samples[1].candidate_count=2;
    assert(mk_search_ladder(&request,samples,2,cells,costs,2,selected,&report)==MK_ERROR_INVALID_ARGUMENT);
    samples[1].candidate_count=1;request.max_jump[0]=0;
    assert(mk_search_ladder(&request,samples,2,cells,costs,2,selected,&report)==MK_ERROR_INVALID_ARGUMENT);

    LadderSettings corridor_settings;corridor_settings.joints=1;corridor_settings.externals=1;
    corridor_settings.jump[0]=.15;corridor_settings.velocity[0]=corridor_settings.weight[0]=1;
    std::vector<LadderLayer> bend(5);
    unsigned coordinates[]={0,1,2,1,0};
    for(unsigned l=0;l<5;++l){mk_lattice_candidate c{};c.joints[0]=.1*l;c.external_coordinates[0]=coordinates[l];bend[l].push_back(c);}
    auto exact_bend=structured_ladder(bend,corridor_settings);
    auto widened=coarse_ladder(bend,corridor_settings,CoarseLadderSettings{4,2,1,1});
    assert(widened.failed==UINT32_MAX && widened.route==exact_bend.route && widened.cost==exact_bend.cost && widened.backend==2);
    assert(widened.tested_edges>exact_bend.tested_edges);
    auto fallback=coarse_ladder(bend,corridor_settings,CoarseLadderSettings{4,2,1,0});
    assert(fallback.failed==UINT32_MAX && fallback.route==exact_bend.route && fallback.cost==exact_bend.cost);

    auto invalid_coarse=[&](){bool rejected=false;try{coarse_ladder(bend,corridor_settings,CoarseLadderSettings{4,2,1,0});}
        catch(const std::invalid_argument &){rejected=true;}assert(rejected);};
    corridor_settings.externals=MK_MAX_JOINTS+1;invalid_coarse();corridor_settings.externals=1;
    // This state is omitted by the narrow corridor, but still must be rejected.
    auto invalid_cell=bend[2][0];invalid_cell.external_coordinates[0]=100;invalid_cell.joints[0]=NAN;bend[2].push_back(invalid_cell);invalid_coarse();

}
