#include "device_compiler6.hpp"
#include "motionkit.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>

namespace robotkit {
namespace {

float evaluate_f32(const device_wire6::Segment6Coefficients &c,
                   std::uint8_t degree, float tau) {
    const float coefficients[] = {c.c0, c.c1, c.c2, c.c3, c.c4, c.c5};
    float q = coefficients[degree];
    for (int k = degree - 1; k >= 0; --k) q = q * tau + coefficients[k];
    return q;
}

double evaluate_f64(const rk_trajectory_coefficients &c, std::uint8_t degree, double tau) {
    double q = c.value[degree];
    for (int k = degree - 1; k >= 0; --k) q = q * tau + c.value[k];
    return q;
}

CompiledDevicePlan6 failure(const char *message) {
    CompiledDevicePlan6 result;
    result.error = message;
    return result;
}

} // namespace

std::array<std::uint8_t, 16> fingerprint_device_layout6(
    std::array<std::uint8_t, 16> base, std::span<const DeviceActuator6> layout,
    std::span<const rk_channel_declaration> channels) {
    if (layout.empty() && channels.empty()) return base;
    std::uint64_t hash = 14695981039346656037ULL;
    auto mix = [&](std::uint64_t value) {
        for (int byte = 0; byte < 8; ++byte) {
            hash ^= static_cast<std::uint8_t>(value >> (8 * byte));
            hash *= 1099511628211ULL;
        }
    };
    for (auto byte : base) mix(byte);
    mix(layout.size());
    for (const auto &a : layout) {
        for (unsigned char c : a.id) mix(c);
        mix(0);
        mix(a.joint); mix(std::bit_cast<std::uint64_t>(a.ratio));
        mix(std::bit_cast<std::uint64_t>(a.offset));
        mix(std::bit_cast<std::uint64_t>(a.steps_per_unit));
        mix(std::bit_cast<std::uint64_t>(a.max_rate));
        mix(a.direction_setup_ticks);
        mix(std::bit_cast<std::uint64_t>(a.dual_drive_skew_bound));
    }
    mix(channels.size());
    for (const auto &channel : channels) {
        for (unsigned char c : channel.id) mix(c);
        mix(channel.kind);
        mix(channel.safe_value.kind);
        mix(channel.safe_value.digital);
        mix(std::bit_cast<std::uint64_t>(channel.safe_value.analog));
        mix(std::bit_cast<std::uint64_t>(channel.safe_value.argument));
        for (unsigned char c : channel.safe_value.command) mix(c);
    }
    for (int i = 0; i < 16; ++i) {
        hash ^= hash >> 32; hash *= 1099511628211ULL;
        base[i] = static_cast<std::uint8_t>(hash >> ((i % 8) * 8));
    }
    return base;
}

CompiledDevicePlan6 compile_device_segments6(
    std::span<const rk_trajectory_segment> segments, std::uint64_t plan_id,
    bool ends_at_rest, std::uint64_t host_plan_start_ns,
    const ClockEstimator6 &clock, const rk_robot_runtime_blueprint &blueprint,
    std::uint64_t device_tick_hz, std::uint64_t step_tick_hz,
    std::uint8_t max_degree, double target_error,
    std::span<const DeviceActuator6> layout) {
    if (!clock.may_commit()) return failure("clock_sync_lost");
    if (segments.empty() || plan_id == 0 || blueprint.joint_count == 0 ||
        blueprint.joint_count > device_wire6::MAX_ACTUATORS ||
        device_tick_hz == 0 || step_tick_hz == 0 || max_degree > 5 ||
        !std::isfinite(target_error) || target_error < 0)
        return failure("invalid device compiler input");
    const auto actuator_count = layout.empty() ? blueprint.joint_count : layout.size();
    if (actuator_count == 0 || actuator_count > device_wire6::MAX_ACTUATORS)
        return failure("invalid actuator layout");
    for (const auto &actuator : layout)
        if (actuator.joint >= blueprint.joint_count || !std::isfinite(actuator.ratio) ||
            actuator.ratio == 0 || !std::isfinite(actuator.offset) ||
            !std::isfinite(actuator.steps_per_unit) || actuator.steps_per_unit <= 0 ||
            !std::isfinite(actuator.max_rate) || actuator.max_rate < 0)
            return failure("invalid actuator layout");
    const auto resolution_ns = (1'000'000'000ULL + step_tick_hz - 1) / step_tick_hz;
    const auto base_ticks = clock.map_host_ns(host_plan_start_ns);
    CompiledDevicePlan6 result;
    result.segments.reserve(segments.size());
    mk_trajectory_handle validation{};
    if (mk_trajectory_create(blueprint.joint_count, &validation) != MK_OK)
        return failure("cannot create converted trajectory");
    auto reject = [&](const char *message) {
        mk_trajectory_destroy(validation);
        return failure(message);
    };
    std::uint64_t expected_ns = 0;
    std::int64_t converted_time_ns = 0;
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const auto &source = segments[i];
        if (source.joint_count != blueprint.joint_count || source.degree > max_degree ||
            source.degree > 5 || source.duration_ns == 0 ||
            source.time_from_start_ns != expected_ns ||
            source.time_from_start_ns > UINT64_MAX - source.duration_ns ||
            host_plan_start_ns > UINT64_MAX - source.time_from_start_ns - source.duration_ns)
            return reject("invalid or unsupported segment");
        expected_ns += source.duration_ns;
        const auto host_start = host_plan_start_ns + source.time_from_start_ns;
        const auto start_ticks = clock.map_host_ns(host_start);
        const auto end_ticks = clock.map_host_ns(host_plan_start_ns + expected_ns);
        if (end_ticks <= start_ticks || start_ticks < base_ticks)
            return reject("device tick mapping collapsed segment");
        const auto duration_ticks = end_ticks - start_ticks;
        const auto device_duration_seconds = static_cast<double>(duration_ticks) /
            static_cast<double>(device_tick_hz);
        const auto host_duration_seconds = static_cast<double>(source.duration_ns) / 1e9;
        const auto scale = host_duration_seconds / device_duration_seconds;
        DeviceSegment6 wire;
        wire.header = {0, plan_id, start_ticks, duration_ticks,
                       static_cast<std::uint8_t>(source.degree),
                       static_cast<std::uint8_t>(actuator_count),
                       static_cast<std::uint8_t>(ends_at_rest && i + 1 == segments.size()), 0};
        wire.coefficients.reserve(actuator_count);
        mk_segment converted{};
        converted.struct_size = sizeof(converted);
        converted.t0_ns = converted_time_ns;
        converted.duration_ns = static_cast<std::int64_t>(std::llround(
            static_cast<double>(duration_ticks) * 1e9 / device_tick_hz));
        if (converted.duration_ns <= 0 || converted_time_ns > INT64_MAX - converted.duration_ns)
            return reject("converted time overflow");
        converted_time_ns += converted.duration_ns;
        converted.degree = source.degree;
        converted.joint_count = source.joint_count;
        for (std::uint32_t joint = 0; joint < source.joint_count; ++joint) {
            double factor = 1.0;
            for (std::uint32_t k = 0; k <= source.degree; ++k) {
                converted.coefficients[joint].value[k] =
                    static_cast<float>(source.coefficients[joint].value[k] * factor);
                factor *= scale;
            }
        }
        // Joint -> actuator transmission conversion lives here. Plan C's
        // JointCoupling v5 will validate/derive follower joints immediately
        // before this loop when that model contract lands on main.
        for (std::size_t actuator = 0; actuator < actuator_count; ++actuator) {
            const auto joint = layout.empty() ? actuator : layout[actuator].joint;
            const auto ratio = layout.empty() ? 1.0 : layout[actuator].ratio;
            const auto offset = layout.empty() ? 0.0 : layout[actuator].offset;
            device_wire6::Segment6Coefficients c{};
            c.actuator = static_cast<std::uint8_t>(actuator);
            float *fields[] = {&c.c0, &c.c1, &c.c2, &c.c3, &c.c4, &c.c5};
            double factor = 1.0;
            for (std::uint32_t k = 0; k <= source.degree; ++k) {
                const auto value = ratio * (source.coefficients[joint].value[k] -
                    (k == 0 ? offset : 0.0)) * factor;
                if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
                    return reject("f32 coefficient overflow");
                *fields[k] = static_cast<float>(value);
                factor *= scale;
            }
            wire.coefficients.push_back(c);
        }
        if (mk_trajectory_append_segment(validation, &converted) != MK_OK)
            return reject("converted segment invalid");
        const auto samples = (source.duration_ns + resolution_ns - 1) / resolution_ns;
        for (std::uint64_t sample = 0; sample <= samples; ++sample) {
            const auto host_elapsed_ns = std::min<std::uint64_t>(source.duration_ns, sample * resolution_ns);
            const auto device_elapsed_seconds = static_cast<double>(host_elapsed_ns) / 1e9 / scale;
            const auto host_tau = static_cast<double>(host_elapsed_ns) / 1e9;
            for (std::size_t actuator = 0; actuator < actuator_count; ++actuator) {
                const auto joint = layout.empty() ? actuator : layout[actuator].joint;
                const auto ratio = layout.empty() ? 1.0 : layout[actuator].ratio;
                const auto offset = layout.empty() ? 0.0 : layout[actuator].offset;
                const auto actual = static_cast<double>(evaluate_f32(wire.coefficients[actuator],
                    wire.header.degree, static_cast<float>(device_elapsed_seconds)));
                const auto exact = evaluate_f64(source.coefficients[joint], wire.header.degree, host_tau);
                const auto error = std::abs(actual / ratio + offset - exact);
                result.worst_position_error = std::max(result.worst_position_error, error);
                if (error > target_error) return reject("converted trajectory exceeds target_error");
                if (!layout.empty()) {
                    double velocity = 0.0;
                    double power = 1.0;
                    for (std::uint32_t k = 1; k <= source.degree; ++k) {
                        velocity += k * source.coefficients[joint].value[k] * power;
                        power *= host_tau;
                    }
                    const auto actuator_rate = std::abs(velocity * ratio);
                    if ((layout[actuator].max_rate > 0 &&
                         actuator_rate > layout[actuator].max_rate + 1e-6) ||
                        actuator_rate * layout[actuator].steps_per_unit > step_tick_hz + 1e-6)
                        return reject("actuator step-rate limit exceeded");
                }
            }
        }
        result.segments.push_back(std::move(wire));
    }
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = blueprint.joint_count;
    limits.executor_time_resolution_ns = resolution_ns;
    limits.max_continuity_jump[0] = 1e-6;
    for (std::uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        const auto &source = blueprint.joints[joint];
        limits.position_claimed[joint] = 1;
        limits.position_lower[joint] = source.lower_limit;
        limits.position_upper[joint] = source.upper_limit;
        limits.max_velocity[joint] = source.max_velocity;
        limits.max_acceleration[joint] = source.max_acceleration;
    }
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    const auto validated = mk_validate(validation, &limits, &report);
    mk_trajectory_destroy(validation);
    if (validated != MK_OK) return failure("mk_validate rejected converted trajectory");
    for (const auto &check : report.checks)
        if (check.status == MK_CHECK_FAILED)
            return failure("converted trajectory violates deployment limits");
    result.ok = true;
    return result;
}

} // namespace robotkit
