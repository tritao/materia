#include "motionkit.h"
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace {
constexpr double pi = 3.14159265358979323846;
bool count(const mk_orientation_lattice *lattice, uint32_t &result) {
    if (!lattice || lattice->struct_size != sizeof(*lattice) || lattice->mode > 2 ||
        !std::isfinite(lattice->half_angle) || lattice->half_angle < 0 || lattice->half_angle > pi)
        return false;
    if (lattice->mode == 0) { result = 1; return true; }
    if (!lattice->roll_count) return false;
    uint64_t cells = 1;
    if (lattice->mode == 2 && lattice->half_angle > 0) {
        if (!lattice->tilt_rings || !lattice->azimuth_count) return false;
        cells += uint64_t(lattice->tilt_rings) * lattice->azimuth_count;
    }
    // Check before multiplication: the three input dimensions can overflow uint64.
    if (cells > std::numeric_limits<uint32_t>::max() / lattice->roll_count) return false;
    result = uint32_t(cells * lattice->roll_count);
    return true;
}
}
extern "C" mk_result MK_CALL mk_orientation_lattice_count(const mk_orientation_lattice *lattice, uint32_t *out) {
    uint32_t n;
    if (!out || !count(lattice,n)) return MK_ERROR_INVALID_ARGUMENT;
    *out = n;
    return MK_OK;
}
extern "C" mk_result MK_CALL mk_sample_orientations(const mk_orientation_lattice *lattice,
    const mk_opw_pose *target, mk_orientation_sample *out, uint32_t capacity, uint32_t *out_count) {
    uint32_t n;
    if (!out_count || !count(lattice,n) || !target || target->struct_size != sizeof(*target) || !out || capacity < n)
        return MK_ERROR_INVALID_ARGUMENT;
    for (double value : target->position) if (!std::isfinite(value)) return MK_ERROR_INVALID_ARGUMENT;
    for (double value : target->quaternion) if (!std::isfinite(value)) return MK_ERROR_INVALID_ARGUMENT;
    Eigen::Quaterniond rotation(target->quaternion[3],target->quaternion[0],target->quaternion[1],target->quaternion[2]);
    if (std::abs(rotation.norm()-1) > 1e-8) return MK_ERROR_INVALID_ARGUMENT;
    const uint32_t rings = lattice->mode == 2 && lattice->half_angle > 0 ? lattice->tilt_rings : 0;
    const uint32_t rolls = lattice->mode == 0 ? 1 : lattice->roll_count;
    uint32_t index = 0;
    for (uint32_t ring = 0; ring <= rings; ++ring) {
        const uint32_t azimuths = ring ? lattice->azimuth_count : 1;
        const double tilt = ring ? lattice->half_angle * (double(ring)/rings) : 0;
        for (uint32_t azimuth = 0; azimuth < azimuths; ++azimuth) {
            const double angle = 2*pi*azimuth/azimuths;
            const Eigen::Quaterniond head = rotation * Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(tilt,Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-angle,Eigen::Vector3d::UnitZ());
            for (uint32_t roll = 0; roll < rolls; ++roll) {
                const Eigen::Quaterniond q = head * Eigen::AngleAxisd(2*pi*roll/rolls,Eigen::Vector3d::UnitZ());
                auto &sample = out[index++]; sample = {}; sample.struct_size = sizeof(sample);
                for (unsigned i=0;i<3;++i) sample.position[i] = target->position[i];
                sample.quaternion[0]=q.x();sample.quaternion[1]=q.y();sample.quaternion[2]=q.z();sample.quaternion[3]=q.w();
                sample.roll_index=roll;sample.tilt_index=ring;sample.azimuth_index=azimuth;
            }
        }
    }
    *out_count = index;
    return MK_OK;
}
