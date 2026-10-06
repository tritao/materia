#include "motionkit.h"
#include <array>
#include <cmath>
#include <limits>

namespace {
constexpr double tau = 6.28318530717958647692;
constexpr double tolerance = 1e-9;
struct Range { int32_t first, last; };
bool ranges(const mk_joint_lift_request *r, const double *q, uint32_t n,
    std::array<Range,MK_MAX_JOINTS> &out, uint32_t &count) {
    if (!r || r->struct_size != sizeof(*r) || !q || !n || n > MK_MAX_JOINTS || n != r->joint_count) return false;
    bool empty = false;
    for (unsigned i=0;i<n;++i) {
        if (!std::isfinite(q[i]) || !std::isfinite(r->lower[i]) || !std::isfinite(r->upper[i]) ||
            r->lower[i] > r->upper[i] || r->periodic[i] > 1) return false;
        if (!r->periodic[i]) {
            out[i]={0,0};
            empty |= q[i] < r->lower[i]-tolerance || q[i] > r->upper[i]+tolerance;
        } else {
            const double first=std::ceil((r->lower[i]-q[i]-tolerance)/tau);
            const double last=std::floor((r->upper[i]-q[i]+tolerance)/tau);
            if (first > last) { empty=true; out[i]={0,0}; continue; }
            if (first < std::numeric_limits<int32_t>::min() || last > std::numeric_limits<int32_t>::max()) return false;
            out[i]={int32_t(first),int32_t(last)};
        }
    }
    if (empty) { count=0; return true; }
    uint64_t product=1;
    for (unsigned i=0;i<n;++i) {
        const uint64_t width=uint64_t(int64_t(out[i].last)-out[i].first)+1;
        if (product > std::numeric_limits<uint32_t>::max()/width) return false;
        product *= width;
    }
    count=uint32_t(product); return true;
}
}
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
