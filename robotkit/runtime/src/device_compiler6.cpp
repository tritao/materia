#include "device_compiler6.hpp"
#include "coupling_terms.hpp"
#include "trajectory_core.h"
#include "trajectory_merge.hpp"
#include <algorithm>
#include <cstdio>
#include <bit>
#include <cmath>
#include <limits>

namespace robotkit {
namespace {

double evaluate_f64(const robotkit::SegmentCoefficients &c, std::uint8_t degree, double tau) {
    double q = c.value[degree];
    for (int k = degree - 1; k >= 0; --k) q = q * tau + c.value[k];
    return q;
}

CompiledDevicePlan6 failure(DeviceCompileFailure6 reason, const char *message) {
    CompiledDevicePlan6 result;
    result.error = message;
    result.failure = reason;
    return result;
}

/** A tenth of a motor step; derivative deviations use one percent of that motor's allowed rate. */
constexpr double MERGE_POSITION_STEPS = 0.1;
constexpr double MERGE_SPEED_FRACTION = 0.01;
/** 2^53: the largest integer step base for which a double still represents every whole step. */
constexpr double MAX_BASE_STEPS = 9007199254740992.0;

} // namespace

namespace {

CompiledDevicePlan6 compile_once(
    std::span<const robotkit::TrajectorySegment> segments, std::uint64_t plan_id,
    bool ends_at_rest, std::uint64_t host_plan_start_ns,
    const ClockEstimator6 &clock, const rk_robot_runtime_blueprint &blueprint,
    std::uint64_t device_tick_hz, std::uint64_t step_tick_hz,
    std::uint8_t max_degree, double target_error,
    std::span<const DeviceActuator6> layout, std::uint64_t anchor_ticks, bool homing,
    const double *start_tolerance, bool merge, const DeviceLink6 &link) {
    if (!clock.may_commit()) return failure(DeviceCompileFailure6::Clock, "clock_sync_lost");
    if (segments.empty() || plan_id == 0 || blueprint.joint_count == 0 ||
        blueprint.joint_count > device_wire6::MAX_ACTUATORS ||
        device_tick_hz == 0 || step_tick_hz == 0 || max_degree > 5 ||
        !std::isfinite(target_error) || target_error < 0)
        return failure(DeviceCompileFailure6::InvalidInput, "invalid device compiler input");
    const auto actuator_count = layout.empty() ? blueprint.joint_count : layout.size();
    if (actuator_count == 0 || actuator_count > device_wire6::MAX_ACTUATORS)
        return failure(DeviceCompileFailure6::InvalidInput, "invalid actuator layout");
    for (const auto &actuator : layout)
        if (actuator.joint >= blueprint.joint_count || !std::isfinite(actuator.ratio) ||
            actuator.ratio == 0 || !std::isfinite(actuator.offset) ||
            !std::isfinite(actuator.steps_per_unit) || actuator.steps_per_unit <= 0 ||
            actuator.min_step_ticks == 0)
            return failure(DeviceCompileFailure6::InvalidInput, "invalid actuator layout");
    // Validate original geometry and follower evidence before optional merging can smooth a bad input away.
    if (segments.size() > UINT32_MAX || blueprint.coupling_count > RK_MAX_JOINT_COUPLINGS)
        return failure(DeviceCompileFailure6::InvalidInput, "invalid source layout");
    for (std::uint32_t c = 0; c < blueprint.coupling_count; ++c) {
        const auto &coupling = blueprint.couplings[c];
        if (coupling.leader >= blueprint.joint_count || coupling.follower >= blueprint.joint_count ||
            !std::isfinite(coupling.ratio) || !std::isfinite(coupling.offset))
            return failure(DeviceCompileFailure6::Coupling, "invalid joint coupling layout");
    }
    std::uint64_t expected_source = 0;
    for (const auto &source : segments) {
        if (source.joint_count != blueprint.joint_count || source.degree > 5 || source.duration_ns == 0 ||
            source.time_from_start_ns != expected_source || source.duration_ns > INT64_MAX - expected_source ||
            host_plan_start_ns > UINT64_MAX - expected_source - source.duration_ns)
            return failure(DeviceCompileFailure6::InvalidInput, "invalid source segment");
        expected_source += source.duration_ns;
        for (std::uint32_t j = 0; j < source.joint_count; ++j)
            for (std::uint32_t k = 0; k <= source.degree; ++k)
                if (!std::isfinite(source.coefficients[j].value[k]))
                    return failure(DeviceCompileFailure6::InvalidInput, "nonfinite source coefficient");
        for (std::uint32_t j = 0; j < source.joint_count; ++j) {
            bool follower = false;
            for (std::uint32_t c = 0; c < blueprint.coupling_count; ++c)
                follower |= blueprint.couplings[c].follower == j;
            if (!follower) continue;
            for (std::uint32_t k = 0; k <= source.degree; ++k) {
                double value = 0;
                for (std::uint32_t c = 0; c < blueprint.coupling_count; ++c) {
                    const auto &coupling = blueprint.couplings[c];
                    if (coupling.follower == j) value += coupling.ratio * source.coefficients[coupling.leader].value[k] +
                        (k == 0 ? coupling.offset : 0);
                }
                if (!std::isfinite(value) || std::abs(value - source.coefficients[j].value[k]) > 1e-6)
                    return failure(DeviceCompileFailure6::Coupling, "coupled follower trajectory mismatch");
            }
        }
    }
    const auto original_segments = segments;
    std::vector<robotkit::TrajectorySegment> lowered;
    if (max_degree == 1) {
        const auto period = blueprint.owner_period_ns ? blueprint.owner_period_ns : 10'000'000ULL;
        std::uint64_t expected = 0;
        for (std::size_t i = 0; i < original_segments.size(); ++i) {
            const auto &source = original_segments[i];
            if (source.joint_count != blueprint.joint_count || source.degree > 5 ||
                source.duration_ns == 0 || source.time_from_start_ns != expected ||
                source.duration_ns > UINT64_MAX - expected)
                return failure(DeviceCompileFailure6::InvalidInput, "invalid source segment");
            for (std::uint64_t offset = 0; offset < source.duration_ns;) {
                if (lowered.size() >= 100'000) return failure(DeviceCompileFailure6::Limits, "minimal profile segment limit");
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
                offset += duration;
            }
            expected += source.duration_ns;
        }
        segments = lowered;
    }
    std::vector<robotkit::TrajectorySegment> merged;
    std::vector<bool> is_merged;
    const auto window = device_merge_window6(link);
    if (merge && target_error > 0 && window.below_ns && max_degree == 5 && !layout.empty() && segments.size() > 1) {
        const auto count = blueprint.joint_count;
        std::array<double, MK_MAX_JOINTS> pt, vt, at, vl, al, lo, hi;
        pt.fill(std::numeric_limits<double>::infinity()); vt = pt; at = pt;
        for (std::uint32_t j = 0; j < count; ++j) {
            const auto &joint = blueprint.joints[j];
            vl[j] = (joint.limit_flags & RK_LIMIT_VELOCITY) ? joint.max_velocity : std::numeric_limits<double>::infinity();
            al[j] = (joint.limit_flags & RK_LIMIT_ACCELERATION) ? joint.max_acceleration : std::numeric_limits<double>::infinity();
            lo[j] = joint.lower_limit - (homing ? blueprint.joint_overtravel[j] : 0.0);
            hi[j] = joint.upper_limit + (homing ? blueprint.joint_overtravel[j] : 0.0);
        }
        for (const auto &actuator : layout) {
            const auto j = actuator.joint;
            const double gain = std::abs(actuator.ratio) * actuator.steps_per_unit;
            const double rate = static_cast<double>(step_tick_hz) / actuator.min_step_ticks / gain;
            pt[j] = std::min(pt[j], MERGE_POSITION_STEPS / gain);
            vt[j] = std::min(vt[j], MERGE_SPEED_FRACTION * rate);
            vl[j] = std::min(vl[j], rate);
            at[j] = std::min(at[j], MERGE_SPEED_FRACTION * al[j]);
        }
        // Undriven joints still retain the plan's conversion budget and deployment derivatives.
        for (std::uint32_t j = 0; j < count; ++j) {
            if (!std::isfinite(pt[j])) pt[j] = target_error;
            if (!std::isfinite(vt[j])) vt[j] = MERGE_SPEED_FRACTION * vl[j];
            if (!std::isfinite(at[j])) at[j] = MERGE_SPEED_FRACTION * al[j];
        }
        const motionkit::PolynomialSource view{segments.data(), static_cast<uint32_t>(segments.size()), count,
            [](const void *context, uint32_t index) {
                const auto &s = static_cast<const TrajectorySegment *>(context)[index];
                return motionkit::PolynomialSegmentView{static_cast<int64_t>(s.time_from_start_ns),
                    static_cast<int64_t>(s.duration_ns), s.degree, s.joint_count, s.coefficients, sizeof(SegmentCoefficients)};
            }};
        struct Output { std::vector<TrajectorySegment> &segments; std::vector<bool> &changed; } output{merged, is_merged};
        const auto status = motionkit::merge_polynomials(view, {pt.data(), vt.data(), at.data(), vl.data(), al.data(),
            lo.data(), hi.data(), static_cast<double>(window.up_to_ns) * 1e-9,
            static_cast<int64_t>(window.below_ns), 0}, {&output,
            [](void *context, const mk_segment &piece, uint32_t first, uint32_t last) -> mk_result {
                auto &out = *static_cast<Output *>(context);
                TrajectorySegment s{};
                s.time_from_start_ns = piece.t0_ns; s.duration_ns = piece.duration_ns;
                s.degree = piece.degree; s.joint_count = piece.joint_count;
                for (uint32_t j = 0; j < s.joint_count; ++j)
                    std::copy_n(piece.coefficients[j].value, s.degree + 1, s.coefficients[j].value);
                out.segments.push_back(s); out.changed.push_back(last > first);
                return MK_OK;
            }});
        if (status != MK_OK) return failure(status == MK_ERROR_OUT_OF_MEMORY ? DeviceCompileFailure6::Resource :
            DeviceCompileFailure6::InvalidInput, "polynomial merge failed");
        segments = merged;
    }
    if (is_merged.empty()) is_merged.assign(segments.size(), false);
    auto converted_failure = [&](DeviceCompileFailure6 reason, const char *message, bool affected) {
        auto result = failure(reason, message);
        if (affected && (reason == DeviceCompileFailure6::Limits || reason == DeviceCompileFailure6::StepRate ||
            reason == DeviceCompileFailure6::Precision || reason == DeviceCompileFailure6::Coupling)) {
            result.failure = DeviceCompileFailure6::MergeRefused;
            result.merge_cause = reason;
        }
        return result;
    };
    // The segments in force, by start time, for finding the original that covers an instant.
    std::vector<std::uint64_t> original_starts;
    original_starts.reserve(original_segments.size());
    for (const auto &segment : original_segments) original_starts.push_back(segment.time_from_start_ns);
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
        return failure(DeviceCompileFailure6::Resource, "cannot create converted trajectory");
    bool active_merged = false;
    std::vector<std::int64_t> validation_starts;
    auto reject = [&](DeviceCompileFailure6 reason, const char *message) {
        mk_trajectory_destroy(validation);
        return converted_failure(reason, message, active_merged);
    };
    std::uint64_t expected_ns = 0;
    std::int64_t converted_time_ns = 0;
    double seam_allowance = target_error;
    for (std::size_t i = 0; i < segments.size(); ++i) {
        active_merged = is_merged[i];
        auto source = segments[i];
        if (source.joint_count != blueprint.joint_count || source.degree > max_degree ||
            source.degree > 5 || source.duration_ns == 0 ||
            source.time_from_start_ns != expected_ns ||
            source.time_from_start_ns > UINT64_MAX - source.duration_ns ||
            host_plan_start_ns > UINT64_MAX - source.time_from_start_ns - source.duration_ns)
            return reject(DeviceCompileFailure6::InvalidInput, "invalid or unsupported segment");
        // Resolve joint-to-joint couplings before converting joints to actuator
        // coordinates. Validate the authored follower first, then use the
        // exact relation for the trajectory sent to the device.
        if (blueprint.coupling_count > RK_MAX_JOINT_COUPLINGS)
            return reject(DeviceCompileFailure6::Coupling, "invalid joint coupling layout");
        // A follower is the sum of its couplings' terms, taken once its leaders are final.
        std::uint32_t followers[RK_MAX_JOINTS], follower_count = 0;
        for (std::uint32_t relation = 0; relation < blueprint.coupling_count; ++relation) {
            const auto &coupling = blueprint.couplings[relation];
            if (coupling.leader >= source.joint_count ||
                coupling.follower >= source.joint_count ||
                !std::isfinite(coupling.ratio) || !std::isfinite(coupling.offset))
                return reject(DeviceCompileFailure6::Coupling, "invalid joint coupling layout");
        }
        if (!robotkit::internal::order_followers(blueprint.couplings, blueprint.coupling_count,
                source.joint_count, nullptr, followers, follower_count))
            return reject(DeviceCompileFailure6::Coupling, "invalid joint coupling layout");
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
                // Compare a merged coefficient in normalized segment time: multiplying by T^degree removes the
                // inverse-duration amplification of quintic coefficients. Keep the check on every power.
                if (!std::isfinite(expected) ||
                    !std::isfinite(source.coefficients[follower].value[degree]) ||
                    std::abs(source.coefficients[follower].value[degree] - expected) *
                        (is_merged[i] ? std::pow(static_cast<double>(source.duration_ns) * 1e-9, degree) : 1.0) > 1e-6)
                    return reject(DeviceCompileFailure6::Coupling, "coupled follower trajectory mismatch");
                source.coefficients[follower].value[degree] = expected;
            }
        }
        expected_ns += source.duration_ns;
        const auto host_start = host_plan_start_ns + source.time_from_start_ns;
        // Keep segments contiguous and never shorten their physical duration when
        // rounding the clock mapping. Compression can push a plan authored at
        // its velocity/acceleration ceiling above the deployment limits.
        const auto start_ticks = result.segments.empty() ? base_ticks :
            result.segments.back().header.t0_ticks + result.segments.back().header.duration_ticks;
        const auto mapped_start = device_ticks(host_start);
        const auto mapped_end = device_ticks(host_plan_start_ns + expected_ns);
        if (mapped_end <= mapped_start) return reject(DeviceCompileFailure6::Clock, "device tick mapping collapsed segment");
        const auto mapped_duration = mapped_end - mapped_start;
        const long double minimum_ticks = std::ceil(
            static_cast<long double>(source.duration_ns) * device_tick_hz / 1'000'000'000.0L);
        if (minimum_ticks > UINT64_MAX) return reject(DeviceCompileFailure6::Overflow, "device duration overflow");
        const auto safe_duration = std::max(mapped_duration, static_cast<std::uint64_t>(minimum_ticks));
        if (safe_duration > UINT64_MAX - start_ticks) return reject(DeviceCompileFailure6::Overflow, "device time overflow");
        const auto end_ticks = start_ticks + safe_duration;
        if (end_ticks <= start_ticks || start_ticks < base_ticks)
            return reject(DeviceCompileFailure6::Clock, "device tick mapping collapsed segment");
        const auto duration_ticks = end_ticks - start_ticks;
        const auto device_duration_seconds = static_cast<double>(duration_ticks) /
            static_cast<double>(device_tick_hz);
        const auto host_duration_seconds = static_cast<double>(source.duration_ns) / 1e9;
        const auto scale = host_duration_seconds / device_duration_seconds;
        DeviceSegment6 wire;
        wire.host_time_from_start_ns = source.time_from_start_ns;
        wire.host_duration_ns = source.duration_ns;
        wire.header = {0, plan_id, start_ticks, duration_ticks,
                       static_cast<std::uint8_t>(source.degree),
                       static_cast<std::uint8_t>(actuator_count),
                       static_cast<std::uint8_t>(ends_at_rest && i + 1 == segments.size()), 0};
        wire.coefficients.reserve(actuator_count);
        mk_segment converted{};
        converted.struct_size = sizeof(converted);
        converted.t0_ns = converted_time_ns;
        validation_starts.push_back(converted_time_ns);
        converted.duration_ns = static_cast<std::int64_t>(std::llround(
            static_cast<double>(duration_ticks) * 1e9 / device_tick_hz));
        if (converted.duration_ns <= 0 || converted_time_ns > INT64_MAX - converted.duration_ns)
            return reject(DeviceCompileFailure6::Overflow, "converted time overflow");
        converted_time_ns += converted.duration_ns;
        converted.degree = source.degree;
        converted.joint_count = source.joint_count;
        for (std::uint32_t joint = 0; joint < source.joint_count; ++joint) {
            double factor = 1.0;
            for (std::uint32_t k = 0; k <= source.degree; ++k) {
                // The position goes unrounded: the wire carries it as an integer step base and a small float.
                converted.coefficients[joint].value[k] = k == 0 ? source.coefficients[joint].value[k] * factor :
                    static_cast<double>(static_cast<float>(source.coefficients[joint].value[k] * factor));
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
                auto value = ratio * (source.coefficients[joint].value[k] -
                    (k == 0 ? offset : 0.0)) * factor;
                if (k == 0 && !layout.empty()) {
                    // The position's whole steps go as an integer, so that the float that carries the rest is small
                    // however far the actuator has turned (an extruder past a few thousand radians).
                    const auto steps = std::round(value * layout[actuator].steps_per_unit);
                    if (!std::isfinite(steps) || std::abs(steps) > MAX_BASE_STEPS) return reject(DeviceCompileFailure6::Overflow, "step base overflow");
                    c.base_steps = static_cast<std::int64_t>(steps);
                    value -= steps / layout[actuator].steps_per_unit;
                }
                if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
                    return reject(DeviceCompileFailure6::Overflow, "f32 coefficient overflow");
                *fields[k] = static_cast<float>(value);
                factor *= scale;
            }
            wire.coefficients.push_back(c);
        }
        if (mk_trajectory_append_segment(validation, &converted) != MK_OK)
            return reject(DeviceCompileFailure6::InvalidInput, "converted segment invalid");
        const auto samples = (source.duration_ns + resolution_ns - 1) / resolution_ns;
        for (std::uint64_t sample = 0; sample <= samples; ++sample) {
            const auto host_elapsed_ns = std::min<std::uint64_t>(source.duration_ns, sample * resolution_ns);
            const auto device_elapsed_seconds = static_cast<double>(host_elapsed_ns) / 1e9 / scale;
            const auto host_tau = static_cast<double>(host_elapsed_ns) / 1e9;
            // The original covering this instant (the one before a join, so that the end of a segment is its own).
            const auto absolute_ns = source.time_from_start_ns + host_elapsed_ns;
            const auto covering = std::upper_bound(original_starts.begin(), original_starts.end(), absolute_ns);
            const std::size_t original = covering == original_starts.begin() ? 0 :
                static_cast<std::size_t>(covering - original_starts.begin()) - 1;
            const auto &reference = original_segments[original];
            const auto reference_tau = std::min(static_cast<double>(absolute_ns - reference.time_from_start_ns) / 1e9,
                static_cast<double>(reference.duration_ns) / 1e9);
            for (std::size_t actuator = 0; actuator < actuator_count; ++actuator) {
                const auto joint = layout.empty() ? actuator : layout[actuator].joint;
                const auto ratio = layout.empty() ? 1.0 : layout[actuator].ratio;
                const auto offset = layout.empty() ? 0.0 : layout[actuator].offset;
                const auto device_tau = static_cast<float>(device_elapsed_seconds);
                const auto actual = wire_position(wire.coefficients[actuator], wire.header.degree, device_tau,
                    layout.empty() ? 1.0 : layout[actuator].steps_per_unit);
                const auto exact = evaluate_f64(reference.coefficients[joint], reference.degree, reference_tau);
                const auto error = std::abs(actual / ratio + offset - exact);
                result.worst_position_error = std::max(result.worst_position_error, error);
                // The conversion may be off by the declared error; a merged segment keeps to a tenth of a step of the
                // originals on top of that. The wire's integer step base keeps the float that carries the rest small
                // however far the actuator has turned, so no allowance for the float's range is needed.
                const auto one_step = layout.empty() ? 0.0 : 1.0 / (std::abs(ratio) * layout[actuator].steps_per_unit);
                const auto allowed = target_error + (is_merged[i] ? MERGE_POSITION_STEPS * one_step : 0.0);
                seam_allowance = std::max(seam_allowance, allowed);
                if (error > allowed) return reject(DeviceCompileFailure6::Precision, "converted trajectory exceeds target_error");
                if (!layout.empty()) {
                    double velocity = 0.0;
                    double power = 1.0;
                    for (std::uint32_t k = 1; k <= source.degree; ++k) {
                        velocity += k * source.coefficients[joint].value[k] * power;
                        power *= host_tau;
                    }
                    const auto actuator_rate = std::abs(velocity * ratio);
                    // The device steps at most once per `min_step_ticks`; the layout's steps per unit are the wire's f32 value.
                    const auto steps = layout[actuator].steps_per_unit;
                    if (actuator_rate * steps * layout[actuator].min_step_ticks > step_tick_hz + 1e-6)
                        return reject(DeviceCompileFailure6::StepRate, "actuator step-rate limit exceeded");
                }
            }
        }
        result.segments.push_back(std::move(wire));
    }
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = blueprint.joint_count;
    limits.executor_time_resolution_ns = resolution_ns;
    // Both neighbouring float segments approximate the same authored seam within what each was allowed,
    // so their positions may differ by twice that.
    limits.max_continuity_jump[0] = 2.0 * seam_allowance;
    for (std::uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        const auto &source = blueprint.joints[joint];
        limits.position_claimed[joint] = 1;
        const bool has_overtravel = blueprint.struct_size >=
            offsetof(rk_robot_runtime_blueprint, joint_overtravel) + sizeof(blueprint.joint_overtravel);
        const auto travel = homing && has_overtravel ? blueprint.joint_overtravel[joint] : 0.0;
        limits.position_lower[joint] = source.lower_limit - travel;
        limits.position_upper[joint] = source.upper_limit + travel;
        if (start_tolerance != nullptr) limits.position_start_tolerance[joint] = start_tolerance[joint];
        // What a drive can miss its target by is one step of the finest motor on the joint, not the planner's start mismatch: a
        // start outside a claimed bound is admitted by that, so a homed axis whose coordinate sits one step off its limit
        // starts, and one further does not.
        double resolution = std::numeric_limits<double>::infinity();
        for (const auto &actuator : layout)
            if (actuator.joint == joint)
                resolution = std::min(resolution, 1.0 / (std::abs(actuator.ratio) * actuator.steps_per_unit));
        if (std::isfinite(resolution)) limits.position_start_tolerance[joint] = resolution;
        limits.max_velocity[joint] = source.max_velocity;
        limits.max_acceleration[joint] = source.max_acceleration;
        limits.derivative_claimed[joint] = ((source.limit_flags & RK_LIMIT_VELOCITY) ? 1u : 0u) |
            ((source.limit_flags & RK_LIMIT_ACCELERATION) ? 2u : 0u);
    }
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    const auto validated = mk_validate(validation, &limits, &report);
    mk_trajectory_destroy(validation);
    if (validated != MK_OK) return failure(DeviceCompileFailure6::InvalidInput, "mk_validate rejected converted trajectory");
    for (const auto &check : report.checks)
        if (check.status == MK_CHECK_FAILED) {
            char detail[256];
            std::snprintf(detail, sizeof(detail),
                "converted trajectory violates deployment limits: joint=%u derivative=%u value=%.17g limit=%.17g tolerance=%.17g",
                check.joint, check.derivative_order, check.value, check.limit, check.tolerance);
            const auto time_ns = static_cast<std::int64_t>(std::llround(check.time_seconds * 1e9));
            const auto at = std::upper_bound(validation_starts.begin(), validation_starts.end(), time_ns);
            const auto index = at == validation_starts.begin() ? 0 : static_cast<std::size_t>(at - validation_starts.begin() - 1);
            const bool affected = is_merged[index] || (index > 0 && time_ns == validation_starts[index] && is_merged[index - 1]);
            return converted_failure(DeviceCompileFailure6::Limits, detail, affected);
        }
    result.ok = true;
    return result;
}

} // namespace

CompiledDevicePlan6 compile_device_segments6(
    std::span<const robotkit::TrajectorySegment> segments, std::uint64_t plan_id,
    bool ends_at_rest, std::uint64_t host_plan_start_ns,
    const ClockEstimator6 &clock, const rk_robot_runtime_blueprint &blueprint,
    std::uint64_t device_tick_hz, std::uint64_t step_tick_hz,
    std::uint8_t max_degree, double target_error,
    std::span<const DeviceActuator6> layout, std::uint64_t anchor_ticks, bool homing,
    const double *start_tolerance, const DeviceLink6 &link) {
    // Merged segments are the first try; a plan they would take past a deployment limit goes as it came.
    auto merged = compile_once(segments, plan_id, ends_at_rest, host_plan_start_ns, clock, blueprint, device_tick_hz,
        step_tick_hz, max_degree, target_error, layout, anchor_ticks, homing, start_tolerance, true, link);
    if (merged.ok || merged.failure != DeviceCompileFailure6::MergeRefused) return merged;
    auto unmerged = compile_once(segments, plan_id, ends_at_rest, host_plan_start_ns, clock, blueprint, device_tick_hz,
        step_tick_hz, max_degree, target_error, layout, anchor_ticks, homing, start_tolerance, false, link);
    unmerged.retried_without_merge = true;
    unmerged.merge_cause = merged.merge_cause;
    return unmerged;
}

DeviceMergeWindow6 device_merge_window6(const DeviceLink6 &link) {
    if (!link.segment_frame_bytes || !link.baud || link.queue_capacity < 2 ||
        !std::isfinite(link.segment_rate) || link.segment_rate <= 0) return {};
    const long double service = std::max(10.0L * link.segment_frame_bytes * 1e9L / link.baud,
        1e9L / link.segment_rate);
    // Refill Q rows before the Q-1 rows behind the active row drain; reserve the stated refill delay.
    const auto below = std::ceil((link.queue_capacity * service + link.refill_delay_ns) / (link.queue_capacity - 1));
    if (below < 1 || below > INT64_MAX / 2) return {};
    // Two adjacent rows each just below the sustainable period must fit in one replacement.
    return {static_cast<std::uint64_t>(below), 2 * static_cast<std::uint64_t>(below)};
}

double wire_position(const device_wire6::Segment6Coefficients &c, std::uint8_t degree, float tau,
                     double steps_per_unit) {
    const float coefficients[] = {c.c0, c.c1, c.c2, c.c3, c.c4, c.c5};
    float q = coefficients[degree];
    for (int k = degree - 1; k >= 0; --k) q = q * tau + coefficients[k];
    return static_cast<double>(c.base_steps) / steps_per_unit + static_cast<double>(q);
}

} // namespace robotkit
