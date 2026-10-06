#include "motionkit.h"
#include <cassert>
#include <limits>
#include <vector>
int main() {
    mk_external_lattice l={};l.struct_size=sizeof(l);l.joint_count=8;l.axis_count=2;
    l.joint_indices[0]=0;l.joint_indices[1]=7;l.point_counts[0]=3;l.point_counts[1]=2;
    l.lower[0]=-1;l.upper[0]=1;l.lower[1]=-.5;l.upper[1]=.5;
    double seed[8]={.4,.1,.2,.3,.4,.5,.6,.7};uint32_t count=0,written=0;
    assert(mk_external_lattice_count(&l,&count)==MK_OK && count==6);
    std::vector<mk_external_cell> cells(count),repeat(count);
    assert(mk_sample_external_cells(&l,seed,8,cells.data(),count,&written)==MK_OK && written==count);
    assert(mk_sample_external_cells(&l,seed,8,repeat.data(),count,&written)==MK_OK);
    for (unsigned i=0;i<count;++i) {
        assert(cells[i].coordinates[0]==i/2 && cells[i].coordinates[1]==i%2);
        assert(cells[i].joints[0]==double(i/2)-1 && cells[i].joints[7]==(i%2?.5:-.5));
        for (unsigned j=1;j<7;++j) assert(cells[i].joints[j]==seed[j]);
        for (unsigned j=0;j<8;++j) assert(cells[i].joints[j]==repeat[i].joints[j]);
    }
    assert(mk_sample_external_cells(&l,seed,8,cells.data(),count-1,&written)==MK_ERROR_INVALID_ARGUMENT);
    l.point_counts[0]=1;l.lower[0]=l.upper[0]=.23;
    assert(mk_external_lattice_count(&l,&count)==MK_OK && count==2);
    assert(mk_sample_external_cells(&l,seed,8,cells.data(),count,&written)==MK_OK);
    assert(cells[0].joints[0]==.23 && cells[1].joints[0]==.23);
    l.axis_count=0;assert(mk_external_lattice_count(&l,&count)==MK_OK && count==1);
    assert(mk_sample_external_cells(&l,seed,8,cells.data(),count,&written)==MK_OK);
    for(unsigned j=0;j<8;++j)assert(cells[0].joints[j]==seed[j]);
    l.axis_count=2;l.joint_indices[1]=0;assert(mk_external_lattice_count(&l,&count)==MK_ERROR_INVALID_ARGUMENT);
    l.joint_indices[1]=7;l.point_counts[0]=l.point_counts[1]=100000;l.lower[0]=-1;l.upper[0]=1;
    assert(mk_external_lattice_count(&l,&count)==MK_ERROR_INVALID_ARGUMENT);
    l.point_counts[0]=3;l.point_counts[1]=2;l.lower[0]=-std::numeric_limits<double>::max();l.upper[0]=std::numeric_limits<double>::max();
    assert(mk_sample_external_cells(&l,seed,8,cells.data(),6,&written)==MK_OK && cells[2].joints[0]==0);
    seed[2]=std::numeric_limits<double>::quiet_NaN();assert(mk_sample_external_cells(&l,seed,8,cells.data(),6,&written)==MK_ERROR_INVALID_ARGUMENT);
}
