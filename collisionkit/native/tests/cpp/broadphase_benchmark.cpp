#include "broadphase_scene.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>

namespace {
volatile size_t observed = 0;
double run(size_t n, size_t threshold, bool boolean=false, double motion=0.0) {
    ck_test::Cell cell(n,threshold);
    const double table[4]={0.005,0.0005,0.0005,0.0005};
    ck::Margins margins{2,table};
    ck::PairResult result;
    bool found=false;
    std::vector<double> inflation(cell.world.body_count(),motion);
    auto query = [&] {
        return boolean ? cell.world.any_violation(margins,inflation.data(),found) :
            cell.world.violation(margins,inflation.data(),found,result);
    };
    // Setup/topology is excluded: a posed cell is queried repeatedly, as in
    // the planner. Every timed query includes six-link FK and pose updates.
    for(int i=0;i<50;++i) {cell.move(i*0.013);query();}
    constexpr int iterations=3000;
    auto start=std::chrono::steady_clock::now();
    for(int i=0;i<iterations;++i) {
        cell.move(i*0.013);
        if(!query()) std::abort();
        observed+=found ? 1 : 0;
    }
    return std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count()/iterations;
}
}
int main() {
    std::printf("N boxes, checked pairs, loop us/query, tree us/query (median of 5, FK + violation)\n");
    for(size_t n : {10u,100u,1000u}) {
        std::array<double,5> loop,tree;
        for(size_t i=0;i<loop.size();++i) {
            // Alternate order to reduce systematic thermal/load effects.
            if(i%2==0) {loop[i]=run(n,std::numeric_limits<size_t>::max());tree[i]=run(n,0);}
            else {tree[i]=run(n,0);loop[i]=run(n,std::numeric_limits<size_t>::max());}
        }
        std::sort(loop.begin(),loop.end());std::sort(tree.begin(),tree.end());
        std::printf("%zu, %zu, %.3f, %.3f\n",n,6*n+10,loop[2],tree[2]);
    }
    std::printf("N boxes, body motion bound, ordered us/query, boolean us/query, speedup (median of 5)\n");
    for(size_t n : {100u,1000u}) for(double motion : {0.0,0.03,0.3}) {
        std::array<double,5> ordered,boolean;
        for(size_t i=0;i<ordered.size();++i) {
            ordered[i]=run(n,0,false,motion);boolean[i]=run(n,0,true,motion);
        }
        std::sort(ordered.begin(),ordered.end());std::sort(boolean.begin(),boolean.end());
        std::printf("%zu, %.2f, %.3f, %.3f, %.2fx\n",n,motion,ordered[2],boolean[2],ordered[2]/boolean[2]);
    }
}
