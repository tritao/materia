#pragma once

#include "clock_estimator6.hpp"
#include "device_wire6.hpp"
#include "robotkit_runtime.hpp"
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
    // The least whole step-tick periods between two steps, chosen by the host's device binding.
    std::uint32_t min_step_ticks = 1;
    std::uint16_t direction_setup_ticks = 0;
    double dual_drive_skew_bound = 0.0;
    std::string id;
    // Original shaft mapping; 255 retains the legacy planner mapping.
    std::uint8_t feedback_joint = 255;
    double feedback_ratio = 1.0;
    double feedback_offset = 0.0;
    std::uint8_t measured_joint() const { return feedback_joint == 255 ? joint : feedback_joint; }
    double measured_ratio() const { return feedback_joint == 255 ? ratio : feedback_ratio; }
    double measured_offset() const { return feedback_joint == 255 ? offset : feedback_offset; }
};

struct DeviceInput6 {
    std::uint8_t actuator = 0;
    bool active_high = false;
    std::string switch_id;
};

struct DeviceSegment6 {
    std::uint64_t host_time_from_start_ns = 0;
    std::uint64_t host_duration_ns = 0;
    device_wire6::Segment6Header header{};
    std::vector<device_wire6::Segment6Coefficients> coefficients;
};

struct CompiledDevicePlan6 {
    bool ok = false;
    std::string error;
    double worst_position_error = 0;
    std::vector<DeviceSegment6> segments;
};

/**
  Compiles host segments starting at `host_plan_start_ns` into device segments.
  Clock mapping schedules the start; durations round conservatively without
  compressing physical motion. A nonzero `anchor_ticks` starts the plan on
  that tick so a continuation meets the queued path exactly.
  Homing alone may use the blueprint's declared overtravel past soft limits;
  derivative and actuator step-rate limits still apply.
**/
CompiledDevicePlan6 compile_device_segments6(
    std::span<const robotkit::TrajectorySegment> segments, std::uint64_t plan_id,
    bool ends_at_rest, std::uint64_t host_plan_start_ns,
    const ClockEstimator6 &clock, const rk_robot_runtime_blueprint &blueprint,
    std::uint64_t device_tick_hz, std::uint64_t step_tick_hz,
    std::uint8_t max_degree, double target_error,
    std::span<const DeviceActuator6> layout = {},
    std::uint64_t anchor_ticks = 0, bool homing = false);

} // namespace robotkit
