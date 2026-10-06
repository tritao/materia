#include "motionkit.h"
#include <array>
#include <cmath>
#include <limits>

namespace {
bool count(const mk_external_lattice *lattice,uint32_t &result) {
    if (!lattice || lattice->struct_size!=sizeof(*lattice) || !lattice->joint_count ||
        lattice->joint_count>MK_MAX_JOINTS || lattice->axis_count>lattice->joint_count) return false;
    std::array<bool,MK_MAX_JOINTS> seen={};uint64_t product=1;
    for (unsigned i=0;i<lattice->axis_count;++i) {
        const auto joint=lattice->joint_indices[i],points=lattice->point_counts[i];
        const double lower=lattice->lower[i],upper=lattice->upper[i];
        if (joint>=lattice->joint_count || seen[joint] || !points || !std::isfinite(lower) ||
            !std::isfinite(upper) || lower>upper || (points==1 ? lower!=upper : lower==upper)) return false;
        seen[joint]=true;
        if (product>std::numeric_limits<uint32_t>::max()/points) return false;
        product*=points;
    }
    result=uint32_t(product);return true;
}
}
extern "C" mk_result MK_CALL mk_external_lattice_count(const mk_external_lattice *lattice,uint32_t *out) {
    uint32_t n;if (!out || !count(lattice,n)) return MK_ERROR_INVALID_ARGUMENT;
    *out=n;return MK_OK;
}
extern "C" mk_result MK_CALL mk_sample_external_cells(const mk_external_lattice *lattice,
    const double *seed,uint32_t n,mk_external_cell *out,uint32_t capacity,uint32_t *out_count) {
    uint32_t cells;
    if (!out_count || !count(lattice,cells) || !seed || n!=lattice->joint_count || !out || capacity<cells)
        return MK_ERROR_INVALID_ARGUMENT;
    for (unsigned i=0;i<n;++i) if (!std::isfinite(seed[i])) return MK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index=0;index<cells;++index) {
        auto &cell=out[index];cell={};cell.struct_size=sizeof(cell);
        for (unsigned i=0;i<n;++i) cell.joints[i]=seed[i];
        uint32_t code=index;
        for (uint32_t j=lattice->axis_count;j>0;--j) {
            const unsigned i=j-1;const auto points=lattice->point_counts[i],coordinate=code%points;code/=points;
            cell.coordinates[i]=coordinate;
            // Weighted endpoints avoid overflowing upper-lower for finite wide ranges.
            const double fraction=points==1 ? 0 : double(coordinate)/(points-1);
            cell.joints[lattice->joint_indices[i]]=coordinate==0 ? lattice->lower[i] :
                coordinate==points-1 ? lattice->upper[i] : (1-fraction)*lattice->lower[i]+fraction*lattice->upper[i];
        }
    }
    *out_count=cells;return MK_OK;
}
