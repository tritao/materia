#pragma once

#include "clock_estimator6.hpp"
#include "device_wire6.hpp"
#include "robotkit_runtime.h"
#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace robotkit {

/** One RKD6 channel driven by a SimpleTransmission. Channel order is stable. */
struct DeviceActuator6 {
    std::uint8_t joint = 0;
    double ratio = 1.0;
    double offset = 0.0;
    double steps_per_unit = 1'000.0;
    double max_rate = 0.0; // zero means the actuator has no authored rate limit
    std::uint16_t direction_setup_ticks = 0;
    double dual_drive_skew_bound = 0.0;
    std::string id;
};

std::array<std::uint8_t, 16> fingerprint_device_layout6(
    std::array<std::uint8_t, 16> base, std::span<const DeviceActuator6> layout);

struct DeviceSegment6 {
    device_wire6::Segment6Header header{};
    std::vector<device_wire6::Segment6Coefficients> coefficients;
};

struct CompiledDevicePlan6 {
    bool ok = false;
    std::string error;
    double worst_position_error = 0;
    std::vector<DeviceSegment6> segments;
};

CompiledDevicePlan6 compile_device_segments6(
    std::span<const rk_trajectory_segment> segments, std::uint64_t plan_id,
    bool ends_at_rest, std::uint64_t host_plan_start_ns,
    const ClockEstimator6 &clock, const rk_robot_runtime_blueprint &blueprint,
    std::uint64_t device_tick_hz, std::uint64_t step_tick_hz,
    std::uint8_t max_degree, double target_error,
    std::span<const DeviceActuator6> layout = {});

} // namespace robotkit
