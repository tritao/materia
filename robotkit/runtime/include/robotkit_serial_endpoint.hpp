#ifndef ROBOTKIT_SERIAL_ENDPOINT_HPP
#define ROBOTKIT_SERIAL_ENDPOINT_HPP

#include "robotkit_runtime.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace robotkit {

/**
 * Small framed serial endpoint for the first physical RobotKit backend.
 *
 * Version 4 session, command, and state frames use bounded variable-length
 * payloads, CRC-32, and compiled joint/sensor slot indices. A device
 * implementation can be tested with a serial loopback before it is attached
 * to a motor controller. RobotRuntime owns command arbitration and limits;
 * the device confirms its latched safety state and accepted command sequence.
 */
class RK_API SerialRobotEndpoint final : public RobotEndpoint {
public:
    /** Opens a non-blocking POSIX serial device and establishes a safe session. */
    static std::shared_ptr<SerialRobotEndpoint> open(const char *path,
                                                     uint32_t baud = 115200);
    /**
     * Uses an already-negotiated descriptor. Callers must provide the active
     * nonzero host session ID and the device's confirmed safety state.
     */
    SerialRobotEndpoint(int descriptor, bool take_ownership, uint64_t session_id,
                        rk_safety_state initial_safety);
    ~SerialRobotEndpoint() override;

    SerialRobotEndpoint(const SerialRobotEndpoint &) = delete;
    SerialRobotEndpoint &operator=(const SerialRobotEndpoint &) = delete;

    rk_result apply(const rk_robot_command &command) override;
    /** Samples the next fresh state frame, waiting up to the link timeout. */
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override;
    bool reports_safety_state() const noexcept override { return true; }
    rk_safety_state initial_safety_state() const noexcept override { return initial_safety_; }

    /**
     * Replaces a disconnected POSIX descriptor without replacing the endpoint
     * object held by RobotRuntime. The source timestamp watermark is reset so
     * a restarted device may begin a new clock epoch.
     */
    rk_result reconnect(int descriptor, bool take_ownership = true) noexcept;
    bool connected() const noexcept { return descriptor_ >= 0; }

private:
    SerialRobotEndpoint(int descriptor, bool take_ownership, uint64_t session_id,
                        rk_safety_state initial_safety, bool session_ready);
    rk_result begin_session();
    rk_result wait_for_session_state(uint64_t session_id);

    int descriptor_ = -1;
    bool owns_descriptor_ = true;
    std::vector<std::uint8_t> input_;
    uint64_t session_id_ = 0;
    uint64_t last_command_sequence_ = 0;
    std::uint64_t last_source_timestamp_ns_ = 0;
    bool has_source_timestamp_ = false;
    rk_safety_state initial_safety_ = RK_SAFETY_READY;
    bool session_ready_ = false;
    rk_robot_state pending_state_{};
    uint64_t pending_state_sequence_ = 0;
    bool has_pending_state_ = false;
};

} // namespace robotkit

#endif
