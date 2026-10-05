#pragma once

#include <cmath>
#include <stdexcept>

namespace nksim {

/** Assumed spring-applied brake parameters, in joint effort units and seconds.
 * This device supplies passive friction, independently of the motor actuator. */
struct BrakeParameters {
    double holding_effort;
    double release_delay;
    double application_delay;
};

/** Physical coil/contact state. The simulation advances it at physics substeps;
 * the controller supplies only the release coil and observes released feedback. */
class BrakeState {
public:
    explicit BrakeState(BrakeParameters parameters) : parameters_(parameters) {
        if (!std::isfinite(parameters.holding_effort) || parameters.holding_effort <= 0.0 ||
            !std::isfinite(parameters.release_delay) || parameters.release_delay < 0.0 ||
            !std::isfinite(parameters.application_delay) || parameters.application_delay < 0.0)
            throw std::invalid_argument("Brake effort must be positive and delays finite and nonnegative");
    }

    void set_release_coil(bool energized) {
        if (energized == energized_) return;
        energized_ = energized;
        transition_time_ = 0.0;
        finish_transition_if_due();
    }

    void advance(double seconds) {
        if (!std::isfinite(seconds) || seconds < 0.0)
            throw std::invalid_argument("Brake substep must be finite and nonnegative");
        if (released_ == energized_) return;
        transition_time_ += seconds;
        finish_transition_if_due();
    }

    bool released() const { return released_; }
    bool release_coil() const { return energized_; }
    double friction_effort() const { return released_ ? 0.0 : parameters_.holding_effort; }

private:
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
