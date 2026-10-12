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

struct Motion { double position, velocity, acceleration; };

/** Position, velocity and acceleration of one joint's polynomial `tau` seconds into its segment. */
Motion motion_at(const robotkit::SegmentCoefficients &c, std::uint32_t degree, double tau) {
    double p = 0.0, v = 0.0, a = 0.0;
    for (int k = static_cast<int>(degree); k >= 0; --k) p = p * tau + c.value[k];
    for (int k = static_cast<int>(degree) - 1; k >= 0; --k) v = v * tau + (k + 1.0) * c.value[k + 1];
    for (int k = static_cast<int>(degree) - 2; k >= 0; --k) a = a * tau + (k + 2.0) * (k + 1.0) * c.value[k + 2];
    return {p, v, a};
}

/**
  The segments `first` to `last` as one quintic through the first one's starting state and the last one's final state
  (position, velocity, acceleration of every joint). The caller checks it against the originals.
**/
robotkit::TrajectorySegment through_run(std::span<const robotkit::TrajectorySegment> source, std::size_t first,
                                        std::size_t last) {
    robotkit::TrajectorySegment merged{};
    merged.time_from_start_ns = source[first].time_from_start_ns;
    merged.duration_ns = source[last].time_from_start_ns + source[last].duration_ns - merged.time_from_start_ns;
    merged.degree = 5;
    merged.joint_count = source[first].joint_count;
    const double T = static_cast<double>(merged.duration_ns) * 1e-9;
    const double end = static_cast<double>(source[last].duration_ns) * 1e-9;
    for (std::uint32_t joint = 0; joint < merged.joint_count; ++joint) {
        const auto from = motion_at(source[first].coefficients[joint], source[first].degree, 0.0);
        const auto to = motion_at(source[last].coefficients[joint], source[last].degree, end);
        const double V0 = from.velocity * T, A0 = from.acceleration * T * T;
        const double V1 = to.velocity * T, A1 = to.acceleration * T * T;
        const double D = to.position - from.position - V0 - 0.5 * A0, E = V1 - V0 - A0, F = A1 - A0;
        const double normalized[6] = {from.position, V0, 0.5 * A0, 10.0 * D - 4.0 * E + 0.5 * F,
            -15.0 * D + 7.0 * E - F, 6.0 * D - 3.0 * E + 0.5 * F};
        double scale = 1.0;
        for (int k = 0; k < 6; ++k) { merged.coefficients[joint].value[k] = normalized[k] / scale; scale *= T; }
    }
    return merged;
}

/** The accuracy a merged run of segments must keep on every motor: a tenth of a step of position, a percent of the step rate in speed. */
constexpr double MERGE_POSITION_STEPS = 0.1;
/** What a position may be off by in the float the wire carries it in, in steps, where that is more than the declared error. */
constexpr double CONVERSION_STEPS = 0.1;
constexpr double MERGE_SPEED_FRACTION = 0.01;
/**
  Only segments shorter than this are merged, and a merged segment is no longer than the next: a link refills a
  segment in a fraction of a millisecond, so longer segments are no problem and are sent as planned.
**/
constexpr std::uint64_t MERGE_BELOW_NS = 2'000'000ULL;
constexpr std::uint64_t MERGE_UP_TO_NS = 4'000'000ULL;

/**
  Whether the quintic `merged` follows the originals it replaces: at the originals' joins and across the run, every
  motor's position stays within a tenth of a step of the original and its speed within a percent of the step rate.
**/
bool follows(const robotkit::TrajectorySegment &merged, std::span<const robotkit::TrajectorySegment> source,
             std::size_t first, std::size_t last, std::span<const robotkit::DeviceActuator6> layout,
             std::uint64_t step_tick_hz, const rk_robot_runtime_blueprint &blueprint) {
    // A run that ends at a joint's travel limit (a printer's Z on its endstop side) may not round past it between
    // the originals' joins: the device's deployment check holds every joint to its limits.
    const double merged_seconds = static_cast<double>(merged.duration_ns) * 1e-9;
    for (int step = 0; step <= 64; ++step)
        for (std::uint32_t joint = 0; joint < merged.joint_count; ++joint) {
            const auto at = motion_at(merged.coefficients[joint], merged.degree, merged_seconds * step / 64.0);
            if (at.position < blueprint.joints[joint].lower_limit || at.position > blueprint.joints[joint].upper_limit)
                return false;
        }
    std::vector<std::uint64_t> times;
    const std::size_t runs = last - first + 1;
    // The joins, thinned to at most 24, and 16 even steps across the run.
    const std::size_t stride = std::max<std::size_t>(1, runs / 24);
    for (std::size_t i = first; i <= last; i += stride) times.push_back(source[i].time_from_start_ns);
    for (int step = 1; step <= 16; ++step)
        times.push_back(merged.time_from_start_ns + merged.duration_ns * step / 16);
    for (const auto absolute : times) {
        // The original covering this instant.
        std::size_t index = first;
        while (index < last && absolute >= source[index + 1].time_from_start_ns) ++index;
        const auto &original = source[index];
        const double tau = std::min<double>(static_cast<double>(absolute - original.time_from_start_ns) * 1e-9,
            static_cast<double>(original.duration_ns) * 1e-9);
        const double mine = static_cast<double>(absolute - merged.time_from_start_ns) * 1e-9;
        for (const auto &actuator : layout) {
            const auto &o = original.coefficients[actuator.joint];
            const auto &m = merged.coefficients[actuator.joint];
            const auto want = motion_at(o, original.degree, tau), have = motion_at(m, merged.degree, mine);
            const double gain = std::abs(actuator.ratio) * actuator.steps_per_unit;
            if (std::abs(have.position - want.position) * gain > MERGE_POSITION_STEPS ||
                std::abs(have.velocity - want.velocity) * gain > MERGE_SPEED_FRACTION * static_cast<double>(step_tick_hz))
                return false;
        }
    }
    return true;
}

} // namespace

namespace {

CompiledDevicePlan6 compile_once(
    std::span<const robotkit::TrajectorySegment> segments, std::uint64_t plan_id,
    bool ends_at_rest, std::uint64_t host_plan_start_ns,
    const ClockEstimator6 &clock, const rk_robot_runtime_blueprint &blueprint,
    std::uint64_t device_tick_hz, std::uint64_t step_tick_hz,
    std::uint8_t max_degree, double target_error,
    std::span<const DeviceActuator6> layout, std::uint64_t anchor_ticks, bool homing,
    const double *start_tolerance, bool merge) {
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
            actuator.min_step_ticks == 0)
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
    // A sliced curve plans to hundreds of short segments a second, more than a serial link can refill. Runs of them
    // become one quintic when it follows them to a tenth of a step (see `follows`): the longest run, no longer than
    // MERGE_UP_TO_NS, that does, found by doubling and then bisecting.
    std::vector<robotkit::TrajectorySegment> merged;
    std::vector<bool> is_merged;
    if (merge && max_degree == 5 && !layout.empty() && segments.size() > 1) {
        const std::uint64_t longest = MERGE_UP_TO_NS;
        for (const auto &segment : segments)
            if (segment.joint_count != blueprint.joint_count || segment.degree > 5) { merge = false; break; }
        if (merge) {
            merged.reserve(segments.size() / 4 + 1);
            for (std::size_t first = 0; first < segments.size();) {
                // The last segment (counted from `first`) that still fits an owner period.
                std::size_t limit = first;
                // Only a segment too short for the link starts a run.
                while (segments[first].duration_ns < MERGE_BELOW_NS && limit + 1 < segments.size() && segments[limit + 1].time_from_start_ns +
                        segments[limit + 1].duration_ns - segments[first].time_from_start_ns <= longest) ++limit;
                std::size_t best = first;
                robotkit::TrajectorySegment best_segment = segments[first];
                auto accepts = [&](std::size_t last) {
                    auto candidate = through_run(segments, first, last);
                    if (!follows(candidate, segments, first, last, layout, step_tick_hz, blueprint)) return false;
                    best = last; best_segment = candidate;
                    return true;
                };
                std::size_t step = 1, failed = limit + 1;
                while (best + step <= limit) {
                    if (accepts(best + step)) step *= 2; else { failed = best + step; break; }
                }
                // Between the last run that worked and the first that did not.
                while (failed > best + 1) {
                    const auto middle = best + (failed - best) / 2;
                    if (middle == best) break;
                    if (accepts(middle)) continue;
                    failed = middle;
                }
                merged.push_back(best_segment);
                is_merged.push_back(best > first);
                first = best + 1;
            }
            segments = merged;
        }
    }
    if (is_merged.empty()) is_merged.assign(segments.size(), false);
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
                // A merged segment's follower is the same combination of its leaders to rounding, which a high power's
                // coefficient (the position over a power of the duration) magnifies past the check's absolute bound.
                if (!std::isfinite(expected) ||
                    !std::isfinite(source.coefficients[follower].value[degree]) ||
                    (!is_merged[i] && std::abs(source.coefficients[follower].value[degree] - expected) > 1e-6))
                    return reject("coupled follower trajectory mismatch");
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
        if (mapped_end <= mapped_start) return reject("device tick mapping collapsed segment");
        const auto mapped_duration = mapped_end - mapped_start;
        const long double minimum_ticks = std::ceil(
            static_cast<long double>(source.duration_ns) * device_tick_hz / 1'000'000'000.0L);
        if (minimum_ticks > UINT64_MAX) return reject("device duration overflow");
        const auto safe_duration = std::max(mapped_duration, static_cast<std::uint64_t>(minimum_ticks));
        if (safe_duration > UINT64_MAX - start_ticks) return reject("device time overflow");
        const auto end_ticks = start_ticks + safe_duration;
        if (end_ticks <= start_ticks || start_ticks < base_ticks)
            return reject("device tick mapping collapsed segment");
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
                const auto actual = static_cast<double>(evaluate_f32(wire.coefficients[actuator],
                    wire.header.degree, static_cast<float>(device_elapsed_seconds)));
                const auto exact = evaluate_f64(reference.coefficients[joint], reference.degree, reference_tau);
                const auto error = std::abs(actual / ratio + offset - exact);
                result.worst_position_error = std::max(result.worst_position_error, error);
                // The conversion may be off by the declared error, or by a tenth of a step where that is more (a position
                // far from zero, an extruder that has turned a thousand times, is held in a float that cannot say better);
                // a merged segment keeps to a tenth of a step of the originals on top of that.
                const auto one_step = layout.empty() ? 0.0 : 1.0 / (std::abs(ratio) * layout[actuator].steps_per_unit);
                const auto allowed = std::max(target_error, CONVERSION_STEPS * one_step) +
                    (is_merged[i] ? MERGE_POSITION_STEPS * one_step : 0.0);
                if (error > allowed) return reject("converted trajectory exceeds target_error");
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
    double seam = target_error;
    for (const auto &actuator : layout)
        seam = std::max(seam, CONVERSION_STEPS / (std::abs(actuator.ratio) * actuator.steps_per_unit));
    limits.max_continuity_jump[0] = 2.0 * seam;
    for (std::uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        const auto &source = blueprint.joints[joint];
        limits.position_claimed[joint] = 1;
        const bool has_overtravel = blueprint.struct_size >=
            offsetof(rk_robot_runtime_blueprint, joint_overtravel) + sizeof(blueprint.joint_overtravel);
        const auto travel = homing && has_overtravel ? blueprint.joint_overtravel[joint] : 0.0;
        limits.position_lower[joint] = source.lower_limit - travel;
        limits.position_upper[joint] = source.upper_limit + travel;
        if (start_tolerance != nullptr) limits.position_start_tolerance[joint] = start_tolerance[joint];
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

} // namespace

CompiledDevicePlan6 compile_device_segments6(
    std::span<const robotkit::TrajectorySegment> segments, std::uint64_t plan_id,
    bool ends_at_rest, std::uint64_t host_plan_start_ns,
    const ClockEstimator6 &clock, const rk_robot_runtime_blueprint &blueprint,
    std::uint64_t device_tick_hz, std::uint64_t step_tick_hz,
    std::uint8_t max_degree, double target_error,
    std::span<const DeviceActuator6> layout, std::uint64_t anchor_ticks, bool homing,
    const double *start_tolerance) {
    // Merged segments are the first try; a plan they would take past a deployment limit goes as it came.
    auto merged = compile_once(segments, plan_id, ends_at_rest, host_plan_start_ns, clock, blueprint, device_tick_hz,
        step_tick_hz, max_degree, target_error, layout, anchor_ticks, homing, start_tolerance, true);
    if (merged.ok) return merged;
    return compile_once(segments, plan_id, ends_at_rest, host_plan_start_ns, clock, blueprint, device_tick_hz,
        step_tick_hz, max_degree, target_error, layout, anchor_ticks, homing, start_tolerance, false);
}

} // namespace robotkit
