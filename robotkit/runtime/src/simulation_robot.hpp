#ifndef ROBOTKIT_SIMULATION_ROBOT_HPP
#define ROBOTKIT_SIMULATION_ROBOT_HPP

#include "robotkit_runtime.hpp"
#include "nativekit_sim.h"

#include <memory>
#include <vector>

namespace robotkit {

class Simulation;

/** Internal mapping between one RobotRuntime and the shared Simulation. */
class SimulationRobot final : public RobotEndpoint {
public:
    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override;
    void discard_pending() noexcept override { pending_targets_.clear(); }
    void reset() noexcept { pending_targets_.clear(); stopped_ = false; previous_time_ = -1.0; }

private:
    friend class Simulation;
    explicit SimulationRobot(Simulation &simulation) : simulation_(simulation) {}
    std::vector<nksim_joint_target> take_pending_targets();

    Simulation &simulation_;
    std::vector<nkscene_occurrence_id> occurrences_;
    std::vector<nksim_body> bodies_;
    std::vector<nksim_joint> joints_;
    std::vector<nksim_joint_target> pending_targets_;
    bool stopped_ = false;
    nksim_body base_body_ = 0;
    double previous_time_ = -1.0;
    double previous_velocity_[3]{};
};

} // namespace robotkit
#endif
