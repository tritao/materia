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
int main(){
    LadderSettings s;s.joints=4;s.externals=1;s.rolls=4;s.roll_weight=.07;
    for(unsigned j=0;j<4;++j){s.jump[j]=.5;s.weight[j]=1+j;s.velocity[j]=1;}
    std::mt19937 random(177);std::uniform_real_distribution<double> q(-1,1);
    for(unsigned trial=0;trial<100;++trial){
        std::vector<LadderLayer> layers(4);std::vector<std::vector<double>> states(4);
        for(unsigned l=0;l<4;++l)for(unsigned i=0;i<16;++i){
            mk_lattice_candidate c{};c.branch=i%2;c.external_coordinates[0]=(i/2)%2;c.roll_index=i/4;
            for(unsigned j=0;j<4;++j)c.joints[j]=q(random);layers[l].push_back(c);states[l].push_back(.01*i);
        }
        auto result=structured_ladder(layers,s,states);
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
    assert(selected[0]==0 && selected[1]==1 && std::abs(report.cost-.13)<1e-12 && report.failed_sample==UINT32_MAX);
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
    assert(widened.failed==UINT32_MAX && widened.route==exact_bend.route && widened.cost==exact_bend.cost);
    assert(widened.tested_edges>exact_bend.tested_edges);
    auto fallback=coarse_ladder(bend,corridor_settings,CoarseLadderSettings{4,2,1,0});
    assert(fallback.failed==UINT32_MAX && fallback.route==exact_bend.route && fallback.cost==exact_bend.cost);

}
