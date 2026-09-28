#pragma once
#include "robotkit_runtime.h"
#include <array>
#include <cstdint>
#include <memory>

namespace robotkit {
class Rkd6Endpoint;
/** POSIX serial transport for the RKD6 scheduled-device path. */
class RK_API DeviceSerialEndpoint {
public:
    static std::shared_ptr<Rkd6Endpoint> open(const char *path, unsigned baud,
        const rk_robot_runtime_blueprint &blueprint,
        std::array<std::uint8_t, 16> fingerprint, double target_error,
        std::uint32_t step_tick_hz = 40'000,
        std::uint64_t link_loss_timeout_ns = 500'000'000,
        std::uint64_t clock_bound_ns = 30'000'000,
        std::uint64_t link_latency_ns = 100'000,
        rk_result *error = nullptr);
};
} // namespace robotkit
