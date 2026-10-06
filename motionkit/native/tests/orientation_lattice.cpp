#include "motionkit.h"
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>
#include <cstring>
int main() {
    for (unsigned mode=0;mode<3;++mode) for (unsigned placement=0;placement<20;++placement) {
        mk_orientation_lattice lattice={};lattice.struct_size=sizeof(lattice);lattice.mode=mode;
        lattice.roll_count=12;lattice.tilt_rings=3;lattice.azimuth_count=8;lattice.half_angle=.4;
        uint32_t count=0;assert(mk_orientation_lattice_count(&lattice,&count)==MK_OK);
        assert(count==(mode==0?1u:mode==1?12u:300u));
        mk_opw_pose target={};target.struct_size=sizeof(target);
        const Eigen::Quaterniond rotation(Eigen::AngleAxisd(.23*placement,Eigen::Vector3d(1,2,3).normalized()));
        target.position[0]=.1*placement;target.position[1]=-.2;target.position[2]=.6;
        target.quaternion[0]=rotation.x();target.quaternion[1]=rotation.y();target.quaternion[2]=rotation.z();target.quaternion[3]=rotation.w();
        std::vector<mk_orientation_sample> samples(count),repeat(count);uint32_t written=0;
        assert(mk_sample_orientations(&lattice,&target,samples.data(),count,&written)==MK_OK && written==count);
        assert(mk_sample_orientations(&lattice,&target,repeat.data(),count,&written)==MK_OK);
        // Padding bytes are not part of the ABI contract; compare fields explicitly.
        const auto axis=rotation*Eigen::Vector3d::UnitZ();
        for(unsigned i=0;i<count;++i) {
            const auto &s=samples[i],&r=repeat[i];
            assert(s.roll_index==r.roll_index && s.tilt_index==r.tilt_index && s.azimuth_index==r.azimuth_index);
            for(unsigned j=0;j<3;++j)assert(s.position[j]==target.position[j] && s.position[j]==r.position[j]);
            for(unsigned j=0;j<4;++j)assert(s.quaternion[j]==r.quaternion[j]);
            const Eigen::Quaterniond q(s.quaternion[3],s.quaternion[0],s.quaternion[1],s.quaternion[2]);
            assert(std::abs(q.norm()-1)<1e-12);
            const double tilt=acos(std::max(-1.0,std::min(1.0,axis.dot(q*Eigen::Vector3d::UnitZ()))));
            assert(std::abs(tilt-(mode==2?.4*s.tilt_index/3:0))<1e-7);
            assert(s.roll_index<(mode?12u:1u));
        }
        assert(mk_sample_orientations(&lattice,&target,samples.data(),count-1,&written)==MK_ERROR_INVALID_ARGUMENT);
    }
    mk_orientation_lattice bad={};bad.struct_size=sizeof(bad);bad.mode=2;bad.roll_count=12;
    uint32_t n=0;assert(mk_orientation_lattice_count(&bad,&n)==MK_OK && n==12); // zero cone reduces to roll
    bad.half_angle=.3;assert(mk_orientation_lattice_count(&bad,&n)==MK_ERROR_INVALID_ARGUMENT);
    bad.tilt_rings=std::numeric_limits<uint32_t>::max();bad.azimuth_count=bad.tilt_rings;
    assert(mk_orientation_lattice_count(&bad,&n)==MK_ERROR_INVALID_ARGUMENT);
    bad.half_angle=std::numeric_limits<double>::quiet_NaN();assert(mk_orientation_lattice_count(&bad,&n)==MK_ERROR_INVALID_ARGUMENT);
}
