#pragma once

#include "robotkit_device_host.hpp"
#include "robotkit_runtime.hpp"

#include <array>
#include <cstdint>
#include <memory>

namespace robotkit {

/** Runtime adapter for the POSIX device link. */
class RK_API DeviceSerialEndpoint final : public RobotEndpoint {
public:
    static std::shared_ptr<DeviceSerialEndpoint> open(const char *path, unsigned baud,
        std::array<std::uint8_t, 16> fingerprint, std::uint8_t joint_count,
        double max_target_error, std::uint8_t *session_status = nullptr);
    /** Takes an already negotiated link after verifying its initial safe STATE. */
    static std::shared_ptr<DeviceSerialEndpoint> attach(std::unique_ptr<device::HostLink> link,
        std::uint8_t joint_count, double max_target_error);

    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(std::uint64_t timestamp_ns, rk_robot_state &state) override;
    bool reports_safety_state() const noexcept override { return true; }
    rk_safety_state initial_safety_state() const noexcept override { return RK_SAFETY_EMERGENCY_STOP; }
    bool supports_trajectory_queue() const noexcept override { return false; }

private:
    DeviceSerialEndpoint(std::unique_ptr<device::HostLink> link, std::uint8_t joint_count,
        double max_target_error, device::HostState initial_state);
    std::unique_ptr<device::HostLink> link_;
    std::uint8_t joint_count_;
    double max_target_error_;
    std::uint64_t last_command_sequence_ = 0;
    device::HostState initial_state_{};
    bool has_initial_state_ = true;
};

} // namespace robotkit
