#ifndef MOTIONKIT_HPP
#define MOTIONKIT_HPP

#include "motionkit.h"

#include <array>
#include <algorithm>
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

/** One derivative source for runtime STOP and host-side hold lead. */
inline bool estimate_path_derivatives(const std::vector<mk_segment> &segments,
                                      int64_t time_ns, uint64_t window_ns,
                                      mk_path_derivative_estimate &out) noexcept {
    if (segments.empty()) return false;
    out = {};
    out.struct_size = sizeof(out);
    out.joint_count = segments.front().joint_count;
    const auto last_end = segments.back().t0_ns + segments.back().duration_ns;
    auto selected = [&](int64_t time) -> std::size_t {
        for (std::size_t index = 0; index < segments.size(); ++index)
            if (time < segments[index].t0_ns + segments[index].duration_ns)
                return index;
        return segments.size() - 1;
    };
    const auto current = selected(time_ns);
    const auto &segment = segments[current];
    if (segment.degree != 1) {
        const auto elapsed = std::clamp(time_ns - segment.t0_ns,
            int64_t{0}, segment.duration_ns);
        mk_trajectory_state state{};
        evaluate_segment(segment, static_cast<double>(elapsed) * 1e-9, state);
        std::copy_n(state.velocity, out.joint_count, out.velocity);
        std::copy_n(state.acceleration, out.joint_count, out.acceleration);
        std::copy_n(state.acceleration, out.joint_count, out.recent_acceleration);
        out.has_forward_acceleration = 1;
        out.has_recent_acceleration = 1;
        return true;
    }
    if (time_ns >= last_end) return true;
    for (uint32_t joint = 0; joint < out.joint_count; ++joint)
        out.velocity[joint] = segment.coefficients[joint].value[1];
    if (window_ns == 0) return true;
    const auto current_midpoint = 0.5 * (static_cast<double>(segment.t0_ns) +
        static_cast<double>(segment.t0_ns + segment.duration_ns));
    const auto ahead_time = static_cast<int64_t>(std::min(
        static_cast<uint64_t>(last_end - 1),
        static_cast<uint64_t>(time_ns) + window_ns));
    const auto ahead_index = selected(ahead_time);
    if (ahead_index != current) {
        const auto &ahead = segments[ahead_index];
        const double ahead_midpoint = 0.5 * (static_cast<double>(ahead.t0_ns) +
            static_cast<double>(ahead.t0_ns + ahead.duration_ns));
        const double spacing = (ahead_midpoint - current_midpoint) * 1e-9;
        if (spacing > 0.0) {
            mk_trajectory_state ahead_state{};
            const auto ahead_elapsed = std::clamp(ahead_time - ahead.t0_ns,
                int64_t{0}, ahead.duration_ns);
            evaluate_segment(ahead, static_cast<double>(ahead_elapsed) * 1e-9, ahead_state);
            for (uint32_t joint = 0; joint < out.joint_count; ++joint)
                out.acceleration[joint] =
                    (ahead_state.velocity[joint] - out.velocity[joint]) / spacing;
            out.has_forward_acceleration = 1;
        }
    }
    const auto behind_time = time_ns >= static_cast<int64_t>(window_ns)
        ? time_ns - static_cast<int64_t>(window_ns) : int64_t{0};
    const auto behind_index = selected(behind_time);
    if (behind_index != current) {
        const auto &behind = segments[behind_index];
        const double behind_midpoint = 0.5 * (static_cast<double>(behind.t0_ns) +
            static_cast<double>(behind.t0_ns + behind.duration_ns));
        const double spacing = (current_midpoint - behind_midpoint) * 1e-9;
        if (spacing > 0.0) {
            mk_trajectory_state behind_state{};
            const auto behind_elapsed = std::clamp(behind_time - behind.t0_ns,
                int64_t{0}, behind.duration_ns);
            evaluate_segment(behind, static_cast<double>(behind_elapsed) * 1e-9, behind_state);
            for (uint32_t joint = 0; joint < out.joint_count; ++joint)
                out.recent_acceleration[joint] =
                    (out.velocity[joint] - behind_state.velocity[joint]) / spacing;
            out.has_recent_acceleration = 1;
        }
    }
    return true;
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
