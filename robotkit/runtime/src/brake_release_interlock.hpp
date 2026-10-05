#pragma once

#include <cstdint>

namespace robotkit::internal {

struct BrakeReleaseFeedback {
    uint64_t sequence = 0;
    uint64_t received_ns = 0;
    bool released = false;
};

enum class BrakeReleaseFault { none, invalid_policy, clock_reversed, release_timeout, contact_lost };

/** Controller handshake only. Physical contact delay and friction belong to SimKit.
 * A stop that leaves the servo enabled keeps release commanded; a fault or emergency
 * stop disables it. Reset cannot reuse feedback observed before a new release request. */
class BrakeReleaseInterlock {
public:
    BrakeReleaseInterlock(uint64_t release_timeout_ns, uint64_t feedback_max_age_ns)
        : timeout_ns_(release_timeout_ns), max_age_ns_(feedback_max_age_ns) {
        if (!timeout_ns_ || !max_age_ns_ || max_age_ns_ > timeout_ns_)
            fault_ = BrakeReleaseFault::invalid_policy;
    }

    void reset() {
        energized_ = ready_ = false;
        have_tick_ = false;
        baseline_sequence_ = accepted_sequence_ = 0;
        fault_ = (!timeout_ns_ || !max_age_ns_ || max_age_ns_ > timeout_ns_)
            ? BrakeReleaseFault::invalid_policy : BrakeReleaseFault::none;
    }

    void update(bool servo_enabled, uint64_t now_ns, BrakeReleaseFeedback feedback) {
        if (fault_ != BrakeReleaseFault::none) return;
        if (have_tick_ && now_ns < last_tick_ns_) {
            fail(BrakeReleaseFault::clock_reversed);
            return;
        }
        have_tick_ = true;
        last_tick_ns_ = now_ns;
        if (!servo_enabled) {
            energized_ = ready_ = false;
            baseline_sequence_ = feedback.sequence;
            return;
        }
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
    BrakeReleaseFault fault_ = BrakeReleaseFault::none;
};

} // namespace robotkit::internal
