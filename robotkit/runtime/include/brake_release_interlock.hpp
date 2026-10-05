#pragma once

#include <algorithm>
#include <cstdint>
#include <cmath>

namespace robotkit::internal {

/** Sensor acquisition is quantized to owner cycles, including a zero-rate contact. */
inline bool brake_feedback_cadence_valid(double rate, uint64_t period_ns, uint64_t max_age_ns) {
    if (!std::isfinite(rate) || rate < 0.0 || !period_ns || !max_age_ns) return false;
    const double cycles = rate == 0.0 ? 1.0 :
        std::max(1.0, std::ceil(1.0e9 / rate / static_cast<double>(period_ns)));
    return cycles * static_cast<double>(period_ns) <= static_cast<double>(max_age_ns);
}

struct BrakeReleaseFeedback {
    uint64_t sequence = 0;
    uint64_t received_ns = 0;
    bool released = false;
};

enum class BrakeReleaseFault { none, invalid_policy, clock_reversed, release_timeout, contact_lost, engagement_timeout };
enum class BrakePowerMode { enabled, controlled_stop, power_off };

/** Controller handshake only. Physical contact delay and friction belong to SimKit.
 * A normal stop keeps release commanded. A powered emergency stop drops the coil
 * but retains servo holding until fresh feedback confirms engagement. Power loss
 * cannot retain holding. Reset cannot reuse feedback from a previous request. */
class BrakeReleaseInterlock {
public:
    BrakeReleaseInterlock(uint64_t release_timeout_ns, uint64_t feedback_max_age_ns)
        : timeout_ns_(release_timeout_ns), max_age_ns_(feedback_max_age_ns) {
        if (!timeout_ns_ || !max_age_ns_ || max_age_ns_ > timeout_ns_)
            fault_ = BrakeReleaseFault::invalid_policy;
    }

    void reset() {
        energized_ = ready_ = stopping_ = holding_ = false;
        have_tick_ = false;
        baseline_sequence_ = accepted_sequence_ = 0;
        fault_ = (!timeout_ns_ || !max_age_ns_ || max_age_ns_ > timeout_ns_)
            ? BrakeReleaseFault::invalid_policy : BrakeReleaseFault::none;
    }

    void update(BrakePowerMode mode, uint64_t now_ns, BrakeReleaseFeedback feedback) {
        // Removing power always removes the holding request, even when the
        // clock or feedback is faulty. There is no powered servo to retain it.
        if (mode == BrakePowerMode::power_off) {
            energized_ = ready_ = stopping_ = holding_ = false;
            baseline_sequence_ = feedback.sequence;
            have_tick_ = true;
            last_tick_ns_ = now_ns;
            return;
        }
        if (have_tick_ && now_ns < last_tick_ns_) {
            fail(BrakeReleaseFault::clock_reversed);
            return;
        }
        have_tick_ = true;
        last_tick_ns_ = now_ns;
        if (mode == BrakePowerMode::controlled_stop) {
            if (!stopping_) {
                stopping_ = true;
                // A never-released brake needs no powered handoff. Otherwise
                // the sample acquired before dropping the coil cannot confirm it.
                holding_ = energized_;
                requested_ns_ = now_ns;
                baseline_sequence_ = feedback.sequence;
            }
            energized_ = ready_ = false;
            if (holding_) {
                const bool fresh = feedback.sequence > baseline_sequence_ &&
                    feedback.received_ns >= requested_ns_ && feedback.received_ns <= now_ns &&
                    now_ns - feedback.received_ns <= max_age_ns_;
                if (fresh && !feedback.released) holding_ = false;
                else if (now_ns - requested_ns_ >= timeout_ns_)
                    fault_ = BrakeReleaseFault::engagement_timeout;
            }
            return;
        }
        if (fault_ != BrakeReleaseFault::none) return;
        stopping_ = holding_ = false;
        if (!energized_) {
            energized_ = true;
            ready_ = false;
            requested_ns_ = now_ns;
            baseline_sequence_ = feedback.sequence;
            return;
        }
        const bool fresh = feedback.sequence != 0 && feedback.sequence > baseline_sequence_ &&
            feedback.sequence >= accepted_sequence_ && feedback.received_ns >= requested_ns_ &&
            feedback.received_ns <= now_ns && now_ns - feedback.received_ns <= max_age_ns_;
        if (ready_) {
            if (!fresh || !feedback.released) fail(BrakeReleaseFault::contact_lost);
            else accepted_sequence_ = feedback.sequence;
            return;
        }
        if (now_ns - requested_ns_ >= timeout_ns_) {
            fail(BrakeReleaseFault::release_timeout);
        } else if (fresh && feedback.released) {
            ready_ = true;
            accepted_sequence_ = feedback.sequence;
        }
    }

    bool release_coil() const { return energized_; }
    bool motion_ready() const { return ready_; }
    bool stop_requested() const { return stopping_; }
    bool holding_required() const { return holding_; }
    BrakeReleaseFault fault() const { return fault_; }

private:
    void fail(BrakeReleaseFault fault) {
        fault_ = fault;
        energized_ = ready_ = false;
    }

    uint64_t timeout_ns_, max_age_ns_;
    uint64_t requested_ns_ = 0, last_tick_ns_ = 0;
    uint64_t baseline_sequence_ = 0, accepted_sequence_ = 0;
    bool energized_ = false, ready_ = false, have_tick_ = false;
    bool stopping_ = false, holding_ = false;
    BrakeReleaseFault fault_ = BrakeReleaseFault::none;
};

} // namespace robotkit::internal
