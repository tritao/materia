#include "../src/structured_ladder.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
using namespace motionkit;
int main(int argc,char **argv){
    const unsigned samples=argc>1?std::strtoul(argv[1],nullptr,10):1301;
    if(samples<2 || samples>1301)return 2;
    LadderSettings settings;settings.joints=7;settings.externals=1;settings.rolls=24;
    for(unsigned j=0;j<7;++j){settings.jump[j]=.15;settings.velocity[j]=1;settings.weight[j]=1;}
    std::vector<LadderLayer> layers(samples);
    for(unsigned layer=0;layer<samples;++layer){auto &cells=layers[layer];cells.reserve(7700);
        for(unsigned branch=0;branch<8;++branch)for(unsigned roll=0;roll<24;++roll)for(unsigned track=0;track<40;++track){
            mk_lattice_candidate c{};c.struct_size=sizeof(c);c.branch=branch;c.roll_index=roll;c.external_coordinates[0]=track;
            c.joints[0]=2.6*layer/(samples-1)+.01*track;
            c.joints[1]=.5*(branch&1)+.003*track;
            c.joints[2]=.5*((branch>>1)&1)-.002*track;
            c.joints[3]=.5*((branch>>2)&1);
            c.joints[4]=6.283185307179586*roll/24;
            c.joints[5]=-.003*track;c.joints[6]=.002*layer/(samples-1);
            cells.push_back(c);
        }
        for(unsigned wrap=0;wrap<20;++wrap){auto c=cells[wrap];c.joints[4]+=6.283185307179586;c.wraps[4]=1;cells.push_back(c);}
    }
    const auto start=std::chrono::steady_clock::now();auto result=structured_ladder(layers,settings);
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::printf("synthetic track-shaped ladder: samples=%u candidates=7700 seconds=%.6f tested_edges=%llu cost=%.9f failed=%u\n",samples,seconds,(unsigned long long)result.tested_edges,result.cost,result.failed);
    return result.route.size()==samples ? 0:1;
}
