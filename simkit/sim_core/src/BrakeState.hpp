#pragma once

#include <cmath>
#include <optional>

namespace nksim {

/** Assumed spring-applied brake parameters, in joint effort units and seconds.
 * This device supplies passive friction, independently of the motor actuator. */
struct BrakeParameters {
    double holding_effort;
    double release_delay;
    double application_delay;
    bool valid() const {
        return std::isfinite(holding_effort) && holding_effort > 0.0 &&
            std::isfinite(release_delay) && release_delay >= 0.0 &&
            std::isfinite(application_delay) && application_delay >= 0.0;
    }
};

/** Physical coil/contact state. The simulation advances it at physics substeps;
 * the controller supplies only the release coil and observes released feedback. */
class BrakeState {
public:
    static std::optional<BrakeState> create(BrakeParameters parameters) {
        if (!parameters.valid()) return std::nullopt;
        return BrakeState(parameters);
    }

    void set_release_coil(bool energized) {
        if (energized == energized_) return;
        energized_ = energized;
        transition_time_ = 0.0;
        finish_transition_if_due();
    }

    bool advance(double seconds) {
        if (!std::isfinite(seconds) || seconds < 0.0)
            return false;
        if (released_ == energized_) return true;
        transition_time_ += seconds;
        finish_transition_if_due();
        return true;
    }

    bool released() const { return released_; }
    /** Reset the physical device to its unpowered, spring-applied state. */
    void reset() {
        energized_ = false;
        released_ = false;
        transition_time_ = 0.0;
    }
    bool release_coil() const { return energized_; }
    double friction_effort() const { return released_ ? 0.0 : parameters_.holding_effort; }

private:
    explicit BrakeState(BrakeParameters parameters) : parameters_(parameters) {}
    void finish_transition_if_due() {
        const double delay = energized_ ? parameters_.release_delay : parameters_.application_delay;
        if (released_ != energized_ && transition_time_ >= delay) released_ = energized_;
    }

    BrakeParameters parameters_;
    bool energized_ = false;
    bool released_ = false;
    double transition_time_ = 0.0;
};

} // namespace nksim
