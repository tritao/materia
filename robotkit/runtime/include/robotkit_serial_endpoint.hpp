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
 * The wire format is deliberately fixed-width and documented by the packet
 * values below: version 2 command packets travel host-to-device and state
 * packets travel device-to-host. Each target includes its compiled joint
 * index, so sparse and mixed-mode batches retain their meaning. A device
 * implementation can therefore be tested with a
 * serial loopback before it is attached to a particular motor controller.
 * RobotRuntime still owns validation, sequencing, limits, and safety state.
 */
class RK_API SerialRobotEndpoint final : public RobotEndpoint {
public:
    /** Opens a non-blocking POSIX serial device at the requested baud rate. */
    static std::shared_ptr<SerialRobotEndpoint> open(const char *path,
                                                     uint32_t baud = 115200);
    /** Takes ownership of an already-open non-blocking descriptor. */
    explicit SerialRobotEndpoint(int descriptor, bool take_ownership = true);
    ~SerialRobotEndpoint() override;

    SerialRobotEndpoint(const SerialRobotEndpoint &) = delete;
    SerialRobotEndpoint &operator=(const SerialRobotEndpoint &) = delete;

    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override;

    /**
     * Replaces a disconnected POSIX descriptor without replacing the endpoint
     * object held by RobotRuntime. The source timestamp watermark is reset so
     * a restarted device may begin a new clock epoch.
     */
    rk_result reconnect(int descriptor, bool take_ownership = true) noexcept;
    bool connected() const noexcept { return descriptor_ >= 0; }

private:
    int descriptor_ = -1;
    bool owns_descriptor_ = true;
    std::vector<std::uint8_t> input_;
    std::uint64_t last_source_timestamp_ns_ = 0;
    bool has_source_timestamp_ = false;
};

} // namespace robotkit

#endif
