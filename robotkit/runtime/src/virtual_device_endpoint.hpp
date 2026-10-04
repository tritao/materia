#pragma once

#include "rkd6_endpoint.hpp"
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace robotkit {

/** Deterministic in-process RKD6 link and board configuration. */
struct VirtualDeviceConfig6 {
    /** Optional board-defined peripheral profile; the host does not interpret its parameters. */
    std::uint32_t peripheral_kind = 0;
    std::uint32_t external_sensor_mask = 0;
    std::vector<double> peripheral_parameters;
    std::uint8_t profile = 1; // 1 full, 2 minimal
    std::uint64_t device_tick_hz = 1'000'000;
    std::uint32_t step_tick_hz = 40'000;
    std::uint64_t offset_ticks = 50'000;
    std::int32_t drift_ppm = 0;
    unsigned baud = 921'600;
    std::uint64_t latency_ns = 100'000;
    std::uint64_t jitter_ns = 0;
    double frame_drop_rate = 0;
    double corruption_rate = 0;
    std::uint64_t seed = 1;
    /** The virtual board's unique id, which a deployment must name; "Virtual-Device-1" by default. */
    std::array<std::uint8_t, 16> controller{'V', 'i', 'r', 't', 'u', 'a', 'l', '-',
        'D', 'e', 'v', 'i', 'c', 'e', '-', '1'};
    std::vector<double> steps_per_unit;
    std::vector<DeviceActuator6> actuators;
    double target_error = 1e-5;
    std::uint64_t clock_bound_ns = 500'000;
    std::uint64_t link_loss_timeout_ns = 500'000'000;
};

struct VirtualStepRecord6 {
    std::uint64_t ticks;
    std::uint32_t actuator;
    bool forward;
    bool operator==(const VirtualStepRecord6 &) const = default;
};

struct VirtualEventRecord6 {
    std::uint64_t plan_id;
    std::uint64_t scheduled_path_ticks;
    std::uint64_t applied_path_ticks;
    std::uint64_t device_ticks;
    std::uint32_t channel;
    std::uint8_t kind;
    std::uint8_t digital;
};

class RK_API VirtualDeviceEndpoint final : public RobotEndpoint {
public:
    static std::shared_ptr<VirtualDeviceEndpoint> create(
        const rk_robot_runtime_blueprint &blueprint, VirtualDeviceConfig6 config);
    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(std::uint64_t timestamp_ns, rk_robot_state &state) override;
    bool ready_for_plans() const noexcept;
    bool reports_safety_state() const noexcept override { return true; }
    double observed_position_precision() const noexcept override {
        return inner_->observed_position_precision();
    }
    rk_safety_state initial_safety_state() const noexcept override { return RK_SAFETY_READY; }
    bool executes_trajectory_queue() const noexcept override { return true; }
    int32_t diagnostic_code() const noexcept override;
    const char *fault_reason() const noexcept { return inner_->fault_reason(); }
    rk_result submit_device_plan(const PlanRequest &, std::uint64_t base_time_ns,
        std::uint64_t owner_now_ns, std::uint64_t committed_through_ns,
        const rk_robot_runtime_blueprint &) override;
    /** Recreate the board and UART queues when its simulation clock resets. */
    bool reset();
    bool stop_device();
    bool set_peripheral_input(std::uint32_t input, double value);
    bool sensor_sample(std::uint32_t slot, rk_sensor_sample &sample, std::span<double> values) const;
    void cut_link(bool cut);
    bool miss_next_steps(std::uint32_t actuator, std::uint32_t count);
    std::array<std::uint8_t, 16> controller() const noexcept { return controller_; }
    std::vector<double> actuator_positions() const;
    std::vector<double> joint_positions() const;
    std::vector<float> channel_values() const;
    std::vector<VirtualStepRecord6> step_log() const;
    std::vector<VirtualEventRecord6> event_log() const;

private:
    class Link;
    VirtualDeviceEndpoint(std::shared_ptr<Rkd6Endpoint> inner, Link *link,
        std::vector<DeviceActuator6> actuators, std::uint32_t joint_count,
        std::array<std::uint8_t, 16> controller, const rk_robot_runtime_blueprint &blueprint,
        VirtualDeviceConfig6 config)
        : inner_(std::move(inner)), link_(link), actuators_(std::move(actuators)),
          joint_count_(joint_count), controller_(controller),
          blueprint_(std::make_shared<rk_robot_runtime_blueprint>(blueprint)), config_(std::move(config)) {}
    std::shared_ptr<Rkd6Endpoint> inner_;
    Link *link_;
    std::vector<DeviceActuator6> actuators_;
    std::uint32_t joint_count_;
    std::array<std::uint8_t, 16> controller_;
    std::shared_ptr<const rk_robot_runtime_blueprint> blueprint_;
    VirtualDeviceConfig6 config_;
};

} // namespace robotkit
