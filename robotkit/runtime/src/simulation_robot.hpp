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
    void reset() noexcept { pending_targets_.clear(); stopped_ = false; reset_sensors(); }
    void reset_sensors() noexcept {
        for (auto &sensor : sensors_) {
            sensor.sample = {};
            sensor.previous_time = -1.0;
            sensor.next_due = 0.0;
            sensor.random = sensor.config.noise_seed ? sensor.config.noise_seed : 1;
        }
    }

private:
    friend class Simulation;
    explicit SimulationRobot(Simulation &simulation) : simulation_(simulation) {}
    std::vector<nksim_joint_target> take_pending_targets();

    Simulation &simulation_;
    std::vector<nkscene_node_id> nodes_;
    std::vector<nksim_body> bodies_;
    std::vector<nksim_joint> joints_;
    std::vector<nksim_joint_target> pending_targets_;
    bool stopped_ = false;
    nksim_body base_body_ = 0;
    struct SensorState {
        rk_sensor_config config{};
        rk_sensor_sample sample{};
        double previous_time = -1.0;
        double previous_velocity[3]{};
        double next_due = 0.0;
        uint32_t random = 1;
    };
    std::vector<SensorState> sensors_;
};

} // namespace robotkit
#endif
