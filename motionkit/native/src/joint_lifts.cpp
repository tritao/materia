#include "motionkit.h"
#include <array>
#include <cmath>
#include <limits>

#include "joint_lift_ranges.h"
using namespace motionkit_lifts;
extern "C" mk_result MK_CALL mk_joint_lift_count(const mk_joint_lift_request *r,
    const double *q,uint32_t n,uint32_t *out) {
    std::array<Range,MK_MAX_JOINTS> range;uint32_t count;
    if (!out || !ranges(r,q,n,range,count)) return MK_ERROR_INVALID_ARGUMENT;
    *out=count;return MK_OK;
}
extern "C" mk_result MK_CALL mk_enumerate_joint_lifts(const mk_joint_lift_request *r,
    const double *q,uint32_t n,mk_joint_lift *out,uint32_t capacity,uint32_t *out_count) {
    std::array<Range,MK_MAX_JOINTS> range;uint32_t count;
    if (!out_count || !ranges(r,q,n,range,count) || capacity<count || (count && !out)) return MK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index=0;index<count;++index) {
        auto &lift=out[index];lift={};lift.struct_size=sizeof(lift);
        uint64_t code=index;
        for (uint32_t j=n;j>0;--j) {
            const unsigned i=j-1;
            const uint64_t width=uint64_t(int64_t(range[i].last)-range[i].first)+1;
            const int32_t wrap=int32_t(int64_t(range[i].first)+int64_t(code%width));code/=width;
            lift.wraps[i]=wrap;lift.joints[i]=q[i]+tau*wrap;
        }
    }
    *out_count=count;return MK_OK;
}
