#ifndef ROBOTKIT_SIMULATION_ROBOT_HPP
#define ROBOTKIT_SIMULATION_ROBOT_HPP

#include "robotkit_runtime.hpp"
#include "robotkit_simkit.h"
#include "nativekit_sim.h"

#include <algorithm>
#include <memory>
#include <string>
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
        if (staged_valid_)
            stopped_ = staged_stopped_;
        staged_valid_ = false;
    }
    void reset() noexcept {
        std::fill(slip_.begin(), slip_.end(), 0.0);
        pending_targets_.clear();
        staged_valid_ = false;
        staged_stopped_ = false;
        std::fill(commanded_.begin(), commanded_.end(), JointCommand{});
        stopped_ = false;
        for (auto &drive : pneumatic_drives_) drive.to_a = drive.normally_to_a;
        reset_sensors();
        queue_rest_holds();
    }
    /** Holds `joints` (flags by robot joint) at the designed pose until commanded, and after resets. */
    void hold_at_rest(std::vector<uint8_t> joints) {
        held_at_rest_ = std::move(joints);
        queue_rest_holds();
    }
    void reset_sensors() noexcept {
        for (auto &sensor : sensors_) {
            sensor.sample = {};
            sensor.previous_time = -1.0;
            sensor.next_due = 0.0;
            sensor.random = sensor.config.noise_seed ? sensor.config.noise_seed : 1;
            sensor.active = false;
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
    /** Resolves valve coils and queues simulation-owned cylinder effort for this physics tick. */
    rk_result apply_pneumatic_valves(const RobotRuntime &runtime,
                                     std::vector<nksim_joint_target> &targets);
    /** Resolves spindle setpoint channels and queues their velocity targets. */
    rk_result apply_velocity_drives(const RobotRuntime &runtime,
                                    std::vector<nksim_joint_target> &targets) const;
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
    /**
     * Distance, per joint, the simulated joint is behind its commanded position, as a stepper that
     * lost steps is: added to every position or servo target. Zero is none; reset clears it.
     */
    std::vector<double> slip_;
    /** Servo gains per joint: a joint with stiffness runs as a servo on its position targets. */
    std::vector<rk_robot_joint_servo> servo_;
    std::vector<double> reflected_inertia_;
    /** Joints that only move through couplings to a servo joint: they get no targets of their own. */
    std::vector<uint8_t> passive_;
    /** Seconds between commands, to estimate a servo's velocity target from successive positions. */
    double step_ = 0.01;
    std::vector<nksim_joint_target> pending_targets_;
    std::vector<uint8_t> held_at_rest_;
    void queue_rest_holds() noexcept;
    // Last target mode/value the backend holds per robot joint. Commands
    // applied during a tick are staged and only committed when the tick hands
    // them to the backend, so discarded commands never count as applied.
    std::vector<JointCommand> commanded_;
    std::vector<JointCommand> staged_;
    bool staged_valid_ = false;
    bool staged_stopped_ = false;
    bool stopped_ = false;
    nksim_body base_body_ = 0;
    struct SensorState {
        rk_sensor_config config{};
        rk_sensor_sample sample{};
        /** The latest acquisition's values, held between acquisitions of a slower sensor. */
        double values[RK_SENSOR_VALUE_POOL]{};
        double previous_time = -1.0;
        double previous_velocity[3]{};
        double next_due = 0.0;
        uint32_t random = 1;
        uint32_t joint = UINT32_MAX;
        double window_lower = 0.0;
        double window_upper = 0.0;
        double hysteresis = 0.0;
        bool active = false;
    };
    std::vector<SensorState> sensors_;
    struct PneumaticDriveState {
        uint32_t joint = 0;
        std::string channel_a;
        std::string channel_b;
        bool has_channel_b = false;
        bool normally_to_a = true;
        bool to_a = true;
        double extension_force = 0.0;
        double retraction_force = 0.0;
        double extend_sign = 1.0;
    };
    std::vector<PneumaticDriveState> pneumatic_drives_;
    struct VelocityDriveState {
        uint32_t joint = 0;
        std::string speed_channel;
        std::string direction_channel;
        double radians_per_speed_unit = 0.0;
        double max_effort = 0.0;
        double max_rate = 0.0;
    };
    std::vector<VelocityDriveState> velocity_drives_;
};

} // namespace robotkit
#endif
