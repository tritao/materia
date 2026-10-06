#ifdef NDEBUG
#undef NDEBUG
#endif
#include "../src/structured_ladder.h"
#include <cassert>
#include <random>
using namespace motionkit;
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
}
