#include "motionkit.h"
#include <cassert>
#include <vector>

int main() {
    mk_ladder_request request{};
    request.struct_size=sizeof(request);request.joint_count=1;
    request.roll_count=request.tilt_count=request.azimuth_count=1;
    request.max_jump[0]=2;request.velocity[0]=request.weights[0]=1;
    // The locally cheapest first choice dies. Select the complete route.
    mk_configuration_sample samples[]={{sizeof(mk_configuration_sample),0,0,2},
        {sizeof(mk_configuration_sample),.5,2,2},{sizeof(mk_configuration_sample),1,4,1}};
    double joints[]={0,10,1,9,10};double costs[5]{};uint32_t coordinates[20]{};
    uint32_t selected[3];mk_ladder_result result{};
    assert(mk_search_ladder_compact_filtered(&request,samples,3,joints,5,coordinates,20,
        costs,5,nullptr,0,selected,&result)==MK_OK);
    assert(selected[0]==1 && selected[1]==3 && selected[2]==4 && result.failed_sample==UINT32_MAX);
    request.max_jump[0]=.5;
    assert(mk_search_ladder_compact_filtered(&request,samples,3,joints,5,coordinates,20,
        costs,5,nullptr,0,selected,&result)==MK_ERROR_GENERATION);
    assert(result.failed_sample==1 && result.failed_distance==.5 && result.failure_kind==2);
    request.max_jump[0]=2;samples[2].candidate_count=0;
    assert(mk_search_ladder_compact_filtered(&request,samples,3,joints,5,coordinates,20,
        costs,5,nullptr,0,selected,&result)==MK_ERROR_GENERATION);
    assert(result.failed_sample==2 && result.failed_distance==1 && result.failure_kind==1);
    // State costs express a posture preference while retaining normal edge costs.
    mk_configuration_sample posture[]={{sizeof(mk_configuration_sample),0,0,2},
        {sizeof(mk_configuration_sample),1,2,2}};
    double posture_joints[]={0,10,0,10};double posture_costs[]={20,0,20,0};
    request.max_jump[0]=20;
    assert(mk_search_ladder_compact_filtered(&request,posture,2,posture_joints,4,coordinates,16,
        posture_costs,4,nullptr,0,selected,&result)==MK_OK);
    assert(selected[0]==1 && selected[1]==3);
    // Exercise a longer graph without relying on a machine-dependent time limit.
    std::vector<mk_configuration_sample> many_samples;
    std::vector<double> many_joints,many_costs(8000);
    std::vector<uint32_t> many_coordinates(32000),many_selected(1000);
    for(unsigned i=0;i<1000;++i){
        many_samples.push_back({sizeof(mk_configuration_sample),i*.001,i*8,8});
        for(unsigned j=0;j<8;++j)many_joints.push_back(i*.001+j*.01);
    }
    assert(mk_search_ladder_compact_filtered(&request,many_samples.data(),1000,many_joints.data(),8000,
        many_coordinates.data(),32000,many_costs.data(),8000,nullptr,0,many_selected.data(),&result)==MK_OK);
    for(unsigned i=0;i<1000;++i)assert(many_selected[i]>=i*8 && many_selected[i]<(i+1)*8);
}
