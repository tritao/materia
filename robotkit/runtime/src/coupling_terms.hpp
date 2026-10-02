#pragma once

#include "robotkit_runtime.h"

#include <cmath>
#include <cstdint>

namespace robotkit::internal {

/**
 * Helpers for a blueprint's couplings read as terms. A coupling is one term of its follower's
 * value, so a follower with several couplings is the sum of them (a CoreXY motor follows both
 * axes): follower = sum of (ratio * leader + offset). Each (leader, follower) pair appears once
 * and the terms never form a cycle.
 */

/** True when coupling `index` is the first term of its follower. */
inline bool first_term_of_follower(const rk_robot_joint_coupling *couplings, uint32_t index) {
    for (uint32_t j = 0; j < index; ++j)
        if (couplings[j].follower == couplings[index].follower) return false;
    return true;
}

/**
 * What `values` give `follower`: the sum of its couplings' ratio times leader, plus offsets when
 * `include_offset`.
 */
inline double coupled_follower_value(const rk_robot_joint_coupling *couplings, uint32_t count,
        rk_joint_id follower, const double *values, bool include_offset) {
    double sum = 0.0;
    for (uint32_t i = 0; i < count; ++i)
        if (couplings[i].follower == follower)
            sum += couplings[i].ratio * values[couplings[i].leader] + (include_offset ? couplings[i].offset : 0.0);
    return sum;
}

/**
 * The distinct followers of the couplings, every follower after the followers among its leaders,
 * written to `order` (room for RK_MAX_JOINTS); a follower in `skip` (null for none) counts as
 * settled and is left out. Returns false when the couplings form a cycle or name a joint at or
 * past `joint_count`.
 */
inline bool order_followers(const rk_robot_joint_coupling *couplings, uint32_t count,
        uint32_t joint_count, const bool *skip, uint32_t *order, uint32_t &ordered) {
    bool pending[RK_MAX_JOINTS]{};
    uint32_t waiting = 0;
    ordered = 0;
    for (uint32_t i = 0; i < count; ++i) {
        if (couplings[i].follower >= joint_count || couplings[i].leader >= joint_count) return false;
        if (!pending[couplings[i].follower]) {
            pending[couplings[i].follower] = true;
            ++waiting;
        }
    }
    while (waiting > 0) {
        bool progress = false;
        for (uint32_t joint = 0; joint < joint_count; ++joint) {
            if (!pending[joint]) continue;
            bool ready = true;
            for (uint32_t i = 0; i < count && ready; ++i)
                if (couplings[i].follower == joint && pending[couplings[i].leader]) ready = false;
            if (!ready) continue;
            pending[joint] = false;
            --waiting;
            progress = true;
            if (!skip || !skip[joint]) order[ordered++] = joint;
        }
        if (!progress) return false;
    }
    return true;
}

} // namespace robotkit::internal
