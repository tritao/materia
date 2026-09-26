#ifndef ROBOTKIT_SIMULATION_ROBOT_HPP
#define ROBOTKIT_SIMULATION_ROBOT_HPP

#include "robotkit_runtime.hpp"
#include "nativekit_sim.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace robotkit {

class Simulation;

/** Internal mapping between one RobotRuntime and the shared Simulation. */
class SimulationRobot final : public RobotEndpoint {
public:
    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override;
    void discard_pending() noexcept override {
        pending_targets_.clear();
        staged_valid_ = false;
    }
    void reset() noexcept {
        pending_targets_.clear();
        staged_valid_ = false;
        std::fill(commanded_.begin(), commanded_.end(), JointCommand{});
        stopped_ = false;
        reset_sensors();
    }
    void reset_sensors() noexcept {
        for (auto &sensor : sensors_) {
            sensor.sample = {};
            sensor.previous_time = -1.0;
            sensor.next_due = 0.0;
            sensor.random = sensor.config.noise_seed ? sensor.config.noise_seed : 1;
        }
    }

    /**
     * Velocity target the physics backend holds for one robot joint after the
     * latest completed apply/take cycle, in rad/s or m/s: the runtime's
     * rate-clamped velocity target, zero after any stop or reset, and zero when
     * the joint is idle or held by a position or effort target.
     */
    double applied_velocity(uint32_t joint) const noexcept {
        return joint < commanded_.size() &&
                commanded_[joint].mode == NKSIM_JOINT_TARGET_VELOCITY
            ? commanded_[joint].target : 0.0;
    }

private:
    friend class Simulation;
    explicit SimulationRobot(Simulation &simulation) : simulation_(simulation) {}
    /** Hands this tick's targets to the backend and commits what they command. */
    std::vector<nksim_joint_target> take_pending_targets();
    /** Queues a zero-velocity hold for one actuated joint. */
    void queue_velocity_hold(std::size_t joint);
    /** Mutable copy of the committed joint commands for the apply in progress. */
    struct JointCommand {
        uint32_t mode = 0; /**< 0 idle, else the NKSIM_JOINT_TARGET_* mode. */
        double target = 0.0;
    };
    std::vector<JointCommand> &staged_commands();

    Simulation &simulation_;
    std::vector<nkscene_node_id> nodes_;
    std::vector<nksim_body> bodies_;
    std::vector<nksim_joint> joints_;
    std::vector<uint8_t> actuated_joints_;
    std::vector<nksim_joint_target> pending_targets_;
    // Last target mode/value the backend holds per robot joint. Commands
    // applied during a tick are staged and only committed when the tick hands
    // them to the backend, so discarded commands never count as applied.
    std::vector<JointCommand> commanded_;
    std::vector<JointCommand> staged_;
    bool staged_valid_ = false;
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
