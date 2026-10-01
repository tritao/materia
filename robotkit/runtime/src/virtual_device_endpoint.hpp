#pragma once

#include "rkd6_endpoint.hpp"
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace robotkit {

/** Deterministic in-process RKD6 link and board configuration. */
struct VirtualDeviceConfig6 {
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
    std::array<std::uint8_t, 16> fingerprint{};
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
    bool reports_safety_state() const noexcept override { return true; }
    rk_safety_state initial_safety_state() const noexcept override { return RK_SAFETY_READY; }
    bool executes_trajectory_queue() const noexcept override { return true; }
    int32_t diagnostic_code() const noexcept override;
    rk_result submit_device_plan(const PlanRequest &, std::uint64_t base_time_ns,
        std::uint64_t owner_now_ns, std::uint64_t committed_through_ns,
        const rk_robot_runtime_blueprint &) override;
    void cut_link(bool cut);
    bool miss_next_steps(std::uint32_t actuator, std::uint32_t count);
    std::array<std::uint8_t, 16> fingerprint() const noexcept { return fingerprint_; }
    std::vector<double> actuator_positions() const;
    std::vector<double> joint_positions() const;
    std::vector<float> channel_values() const;
    std::vector<VirtualStepRecord6> step_log() const;
    std::vector<VirtualEventRecord6> event_log() const;

private:
    class Link;
    VirtualDeviceEndpoint(std::shared_ptr<Rkd6Endpoint> inner, Link *link,
        std::vector<DeviceActuator6> actuators, std::uint32_t joint_count,
        std::array<std::uint8_t, 16> fingerprint)
        : inner_(std::move(inner)), link_(link), actuators_(std::move(actuators)),
          joint_count_(joint_count), fingerprint_(fingerprint) {}
    std::shared_ptr<Rkd6Endpoint> inner_;
    Link *link_;
    std::vector<DeviceActuator6> actuators_;
    std::uint32_t joint_count_;
    std::array<std::uint8_t, 16> fingerprint_;
};

} // namespace robotkit
