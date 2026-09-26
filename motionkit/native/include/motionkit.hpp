#ifndef MOTIONKIT_HPP
#define MOTIONKIT_HPP

#include "motionkit.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace motionkit {

/** Shared polynomial evaluator for the planner, validator and RobotKit runtime. */
inline void evaluate_segment(const mk_segment &segment, double seconds,
                             mk_trajectory_state &state) noexcept {
    state.joint_count = segment.joint_count;
    for (uint32_t joint = 0; joint < segment.joint_count; ++joint) {
        const auto *c = segment.coefficients[joint].value;
        double position = 0.0;
        double velocity = 0.0;
        double acceleration = 0.0;
        double jerk = 0.0;
        for (int degree = static_cast<int>(segment.degree); degree >= 0; --degree) {
            jerk = jerk * seconds + 3.0 * acceleration;
            acceleration = acceleration * seconds + 2.0 * velocity;
            velocity = velocity * seconds + position;
            position = position * seconds + c[degree];
        }
        state.position[joint] = position;
        state.velocity[joint] = velocity;
        state.acceleration[joint] = acceleration;
        state.jerk[joint] = jerk;
    }
}

class Trajectory {
public:
    explicit Trajectory(uint32_t joint_count) : joint_count_(joint_count) {}

    uint32_t joint_count() const noexcept { return joint_count_; }
    uint32_t segment_count() const noexcept { return static_cast<uint32_t>(segments_.size()); }
    uint64_t revision() const noexcept { return revision_; }
    const mk_segment &segment(uint32_t index) const { return segments_.at(index); }
    int64_t duration_ns() const noexcept {
        if (segments_.empty()) return 0;
        return segments_.back().t0_ns + segments_.back().duration_ns - segments_.front().t0_ns;
    }

    bool append(const mk_segment &segment) {
        if (segment.struct_size < sizeof(mk_segment) ||
            segment.joint_count != joint_count_ || segment.degree > MK_MAX_DEGREE ||
            segment.duration_ns <= 0 ||
            segment.t0_ns > std::numeric_limits<int64_t>::max() - segment.duration_ns)
            return false;
        if (!segments_.empty() && segment.t0_ns !=
                segments_.back().t0_ns + segments_.back().duration_ns)
            return false;
        const int64_t first = segments_.empty() ? segment.t0_ns : segments_.front().t0_ns;
        const int64_t end = segment.t0_ns + segment.duration_ns;
        if (static_cast<uint64_t>(end) - static_cast<uint64_t>(first) >
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
            return false;
        for (uint32_t joint = 0; joint < joint_count_; ++joint)
            for (uint32_t degree = 0; degree <= segment.degree; ++degree)
                if (!std::isfinite(segment.coefficients[joint].value[degree])) return false;
        segments_.push_back(segment);
        ++revision_;
        return true;
    }

    bool evaluate(int64_t time_ns, mk_trajectory_state &state) const noexcept {
        if (segments_.empty()) return false;
        // Exact knots select the segment on the right. The final endpoint
        // selects the final segment evaluated at its duration.
        uint32_t low = 0;
        uint32_t high = static_cast<uint32_t>(segments_.size());
        while (low + 1 < high) {
            const uint32_t middle = low + (high - low) / 2;
            if (segments_[middle].t0_ns <= time_ns) low = middle;
            else high = middle;
        }
        const auto &segment = segments_[low];
        const int64_t clamped = time_ns <= segment.t0_ns ? 0 :
            time_ns >= segment.t0_ns + segment.duration_ns ? segment.duration_ns :
            time_ns - segment.t0_ns;
        evaluate_segment(segment, static_cast<double>(clamped) / 1'000'000'000.0, state);
        return true;
    }

private:
    uint32_t joint_count_;
    std::vector<mk_segment> segments_;
    uint64_t revision_ = 0;
};

mk_result validate(const Trajectory &trajectory, const mk_limits &limits,
                   mk_validation_report &report);
mk_result generate(const mk_state_to_state_request &request, Trajectory &trajectory,
                   int32_t &ruckig_result);

} // namespace motionkit

#endif
