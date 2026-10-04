#include "device_compiler6.hpp"
#include "coupling_terms.hpp"
#include "trajectory_core.h"
#include <algorithm>
#include <cstdio>
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

double evaluate_f64(const robotkit::SegmentCoefficients &c, std::uint8_t degree, double tau) {
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

CompiledDevicePlan6 compile_device_segments6(
    std::span<const robotkit::TrajectorySegment> segments, std::uint64_t plan_id,
    bool ends_at_rest, std::uint64_t host_plan_start_ns,
    const ClockEstimator6 &clock, const rk_robot_runtime_blueprint &blueprint,
    std::uint64_t device_tick_hz, std::uint64_t step_tick_hz,
    std::uint8_t max_degree, double target_error,
    std::span<const DeviceActuator6> layout, std::uint64_t anchor_ticks) {
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
    const auto original_segments = segments;
    std::vector<robotkit::TrajectorySegment> lowered;
    std::vector<std::size_t> original_index;
    std::vector<std::uint64_t> original_offset;
    if (max_degree == 1) {
        const auto period = blueprint.owner_period_ns ? blueprint.owner_period_ns : 10'000'000ULL;
        std::uint64_t expected = 0;
        for (std::size_t i = 0; i < original_segments.size(); ++i) {
            const auto &source = original_segments[i];
            if (source.joint_count != blueprint.joint_count || source.degree > 5 ||
                source.duration_ns == 0 || source.time_from_start_ns != expected ||
                source.duration_ns > UINT64_MAX - expected)
                return failure("invalid source segment");
            for (std::uint64_t offset = 0; offset < source.duration_ns;) {
                if (lowered.size() >= 100'000) return failure("minimal profile segment limit");
                // Linear polynomials need no chord approximation. Longer pieces give
                // the shallow board queue time to absorb a host scheduling stall;
                // keep boundaries so uncommitted motion can still be replaced.
                const auto piece_period = source.degree <= 1 ? std::max<std::uint64_t>(period, 100'000'000) : period;
                const auto duration = std::min<std::uint64_t>(piece_period, source.duration_ns - offset);
                auto piece = source;
                piece.time_from_start_ns = expected + offset;
                piece.duration_ns = duration;
                piece.degree = 1;
                for (std::uint32_t joint = 0; joint < source.joint_count; ++joint) {
                    const auto t0 = static_cast<double>(offset) / 1e9;
                    const auto t1 = static_cast<double>(offset + duration) / 1e9;
                    const auto q0 = evaluate_f64(source.coefficients[joint], source.degree, t0);
                    const auto q1 = evaluate_f64(source.coefficients[joint], source.degree, t1);
                    piece.coefficients[joint].value[0] = q0;
                    piece.coefficients[joint].value[1] = (q1 - q0) / (static_cast<double>(duration) / 1e9);
                    for (int k = 2; k < 6; ++k) piece.coefficients[joint].value[k] = 0;
                }
                lowered.push_back(piece);
                original_index.push_back(i);
                original_offset.push_back(offset);
                offset += duration;
            }
            expected += source.duration_ns;
        }
        segments = lowered;
    }
    const auto resolution_ns = (1'000'000'000ULL + step_tick_hz - 1) / step_tick_hz;
    // A continuation starts on the device tick where the queued path ends. Mapping its
    // start afresh would move it whenever a time sync refined the clock in between.
    const auto mapped_start_ticks = clock.map_host_ns(host_plan_start_ns);
    const std::int64_t tick_shift = anchor_ticks == 0 ? 0 :
        static_cast<std::int64_t>(anchor_ticks) - static_cast<std::int64_t>(mapped_start_ticks);
    auto device_ticks = [&](std::uint64_t host_ns) {
        return static_cast<std::uint64_t>(static_cast<std::int64_t>(clock.map_host_ns(host_ns)) + tick_shift);
    };
    const auto base_ticks = device_ticks(host_plan_start_ns);
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
        auto source = segments[i];
        if (source.joint_count != blueprint.joint_count || source.degree > max_degree ||
            source.degree > 5 || source.duration_ns == 0 ||
            source.time_from_start_ns != expected_ns ||
            source.time_from_start_ns > UINT64_MAX - source.duration_ns ||
            host_plan_start_ns > UINT64_MAX - source.time_from_start_ns - source.duration_ns)
            return reject("invalid or unsupported segment");
        // Resolve joint-to-joint couplings before converting joints to actuator
        // coordinates. Validate the authored follower first, then use the
        // exact relation for the trajectory sent to the device.
        if (blueprint.coupling_count > RK_MAX_JOINT_COUPLINGS)
            return reject("invalid joint coupling layout");
        // A follower is the sum of its couplings' terms, taken once its leaders are final.
        std::uint32_t followers[RK_MAX_JOINTS], follower_count = 0;
        for (std::uint32_t relation = 0; relation < blueprint.coupling_count; ++relation) {
            const auto &coupling = blueprint.couplings[relation];
            if (coupling.leader >= source.joint_count ||
                coupling.follower >= source.joint_count ||
                !std::isfinite(coupling.ratio) || !std::isfinite(coupling.offset))
                return reject("invalid joint coupling layout");
        }
        if (!robotkit::internal::order_followers(blueprint.couplings, blueprint.coupling_count,
                source.joint_count, nullptr, followers, follower_count))
            return reject("invalid joint coupling layout");
        for (std::uint32_t index = 0; index < follower_count; ++index) {
            const auto follower = followers[index];
            for (std::uint32_t degree = 0; degree <= source.degree; ++degree) {
                double expected = 0.0;
                for (std::uint32_t relation = 0; relation < blueprint.coupling_count; ++relation) {
                    const auto &coupling = blueprint.couplings[relation];
                    if (coupling.follower == follower)
                        expected += coupling.ratio * source.coefficients[coupling.leader].value[degree] +
                            (degree == 0 ? coupling.offset : 0.0);
                }
                if (!std::isfinite(expected) ||
                    !std::isfinite(source.coefficients[follower].value[degree]) ||
                    std::abs(source.coefficients[follower].value[degree] - expected) > 1e-6)
                    return reject("coupled follower trajectory mismatch");
                source.coefficients[follower].value[degree] = expected;
            }
        }
        expected_ns += source.duration_ns;
        const auto host_start = host_plan_start_ns + source.time_from_start_ns;
        const auto start_ticks = device_ticks(host_start);
        const auto end_ticks = device_ticks(host_plan_start_ns + expected_ns);
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
        // Joint-to-actuator transmissions remain distinct from the resolved
        // joint-to-joint couplings above.
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
                const auto &reference = max_degree == 1 ? original_segments[original_index[i]] : source;
                const auto reference_tau = host_tau +
                    (max_degree == 1 ? static_cast<double>(original_offset[i]) / 1e9 : 0.0);
                const auto exact = evaluate_f64(reference.coefficients[joint], reference.degree, reference_tau);
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
    // Both neighbouring float segments approximate the same authored seam within
    // target_error, so their positions may differ by twice that declared error.
    limits.max_continuity_jump[0] = 2.0 * target_error;
    for (std::uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        const auto &source = blueprint.joints[joint];
        limits.position_claimed[joint] = 1;
        limits.position_lower[joint] = source.lower_limit;
        limits.position_upper[joint] = source.upper_limit;
        limits.max_velocity[joint] = source.max_velocity;
        limits.max_acceleration[joint] = source.max_acceleration;
        limits.derivative_claimed[joint] = ((source.limit_flags & RK_LIMIT_VELOCITY) ? 1u : 0u) |
            ((source.limit_flags & RK_LIMIT_ACCELERATION) ? 2u : 0u);
    }
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    const auto validated = mk_validate(validation, &limits, &report);
    mk_trajectory_destroy(validation);
    if (validated != MK_OK) return failure("mk_validate rejected converted trajectory");
    for (const auto &check : report.checks)
        if (check.status == MK_CHECK_FAILED) {
            char detail[256];
            std::snprintf(detail, sizeof(detail),
                "converted trajectory violates deployment limits: joint=%u derivative=%u value=%.17g limit=%.17g tolerance=%.17g",
                check.joint, check.derivative_order, check.value, check.limit, check.tolerance);
            return failure(detail);
        }
    result.ok = true;
    return result;
}

} // namespace robotkit
