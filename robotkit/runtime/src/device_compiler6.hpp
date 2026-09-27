#pragma once

#include "clock_estimator6.hpp"
#include "device_wire6.hpp"
#include "robotkit_runtime.h"
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace robotkit {

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
    std::uint8_t max_degree, double target_error);

} // namespace robotkit
