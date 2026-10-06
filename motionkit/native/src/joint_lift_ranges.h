#ifndef MOTIONKIT_JOINT_LIFT_RANGES_H
#define MOTIONKIT_JOINT_LIFT_RANGES_H
#include "motionkit.h"
#include <array>
#include <cmath>
#include <limits>
namespace motionkit_lifts {
constexpr double tau = 6.28318530717958647692;
constexpr double tolerance = 1e-9;
struct Range { int32_t first, last; };
inline bool ranges(const mk_joint_lift_request *r, const double *q, uint32_t n,
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
#endif
