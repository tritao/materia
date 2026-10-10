#include "trajectory_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <unordered_map>
#include <vector>

namespace {

std::mutex registry_mutex;
std::unordered_map<uint32_t, std::shared_ptr<motionkit::Trajectory>> trajectories;
uint32_t next_id = 1;

struct Plan {
    motionkit::Trajectory trajectory;
    mk_plan_spec spec;
    mk_validation_report report;
    // The segments as flat arrays, read in place through mk_plan_segment_*.
    std::vector<int64_t> starts, durations;
    std::vector<int32_t> degrees;
    std::vector<double> coefficients;
};

void flatten_segments(Plan &plan) {
    const auto &trajectory = plan.trajectory;
    const uint32_t count = trajectory.segment_count(), joints = trajectory.joint_count(),
        stride = MK_MAX_DEGREE + 1;
    plan.starts.resize(count);
    plan.durations.resize(count);
    plan.degrees.resize(count);
    plan.coefficients.assign(static_cast<size_t>(count) * joints * stride, 0.0);
    for (uint32_t index = 0; index < count; ++index) {
        const auto &segment = trajectory.segment(index);
        plan.starts[index] = segment.t0_ns;
        plan.durations[index] = segment.duration_ns;
        plan.degrees[index] = static_cast<int32_t>(segment.degree);
        for (uint32_t joint = 0; joint < joints; ++joint)
            for (uint32_t power = 0; power <= segment.degree; ++power)
                plan.coefficients[(static_cast<size_t>(index) * joints + joint) * stride + power] =
                    segment.coefficients[joint].value[power];
    }
}
std::unordered_map<uint32_t, std::shared_ptr<const Plan>> plans;
uint32_t next_plan_id = 1;

// The registry lock guards only these maps: a handle's object is shared out of
// them, so work on it (validation, evaluation) runs without the lock, and a
// destroy while another thread uses an object frees it when that use ends.
// Each trajectory is used by one thread at a time; plans are immutable.
std::shared_ptr<motionkit::Trajectory> find(mk_trajectory_handle handle) {
    std::lock_guard lock(registry_mutex);
    const auto found = trajectories.find(handle.id);
    return found == trajectories.end() ? nullptr : found->second;
}

mk_result register_trajectory(std::unique_ptr<motionkit::Trajectory> trajectory,
                              mk_trajectory_handle *out) {
    if (trajectories.size() >= UINT32_MAX - 1) return MK_ERROR_OUT_OF_MEMORY;
    while (next_id == 0 || trajectories.count(next_id) != 0) ++next_id;
    const uint32_t id = next_id++;
    trajectories.emplace(id, std::move(trajectory));
    out->id = id;
    return MK_OK;
}

bool valid_sample(const mk_sample &sample, uint32_t joint_count) {
    if (sample.struct_size < sizeof(mk_sample) || sample.joint_count != joint_count)
        return false;
    for (uint32_t joint = 0; joint < joint_count; ++joint)
        if (!std::isfinite(sample.position[joint])) return false;
    return true;
}

bool valid_plan_spec(const mk_plan_spec &spec, const mk_limits &limits,
                     uint32_t joint_count) {
    if (spec.struct_size < offsetof(mk_plan_spec, event_count) ||
        (spec.struct_size > offsetof(mk_plan_spec, event_count) &&
         spec.struct_size < sizeof(mk_plan_spec)) ||
        spec.start_state.struct_size < sizeof(mk_start_state) ||
        spec.start_state.joint_count != joint_count || spec.plan_id == 0 ||
        spec.model_revision != limits.model_revision ||
        spec.calibration_revision != limits.calibration_revision ||
        (spec.required_capabilities & ~static_cast<uint64_t>(
            MK_CAP_TIMED_TRAJECTORY | MK_CAP_EVENTS)) != 0)
        return false;
    if (spec.struct_size >= sizeof(mk_plan_spec)) {
        if (spec.event_count > MK_MAX_PLAN_EVENTS ||
            (spec.event_count != 0 && (spec.required_capabilities & MK_CAP_EVENTS) == 0))
            return false;
        uint64_t previous = 0;
        for (uint32_t i = 0; i < spec.event_count; ++i) {
            const auto &event = spec.events[i];
            if (event.channel[0] == '\0' ||
                std::memchr(event.channel, '\0', sizeof(event.channel)) == nullptr ||
                event.hold_policy > MK_EVENT_RESTORE_ON_RESUME ||
                (i != 0 && event.time_ns < previous)) return false;
            const auto &value = event.value;
            if (value.kind == MK_EVENT_DIGITAL && value.digital > 1) return false;
            if (value.kind == MK_EVENT_ANALOG && !std::isfinite(value.analog)) return false;
            if (value.kind == MK_EVENT_PROCESS &&
                (value.command[0] == '\0' ||
                 std::memchr(value.command, '\0', sizeof(value.command)) == nullptr ||
                 !std::isfinite(value.argument))) return false;
            if (value.kind < MK_EVENT_DIGITAL || value.kind > MK_EVENT_PROCESS) return false;
            previous = event.time_ns;
        }
    }
    const auto &start = spec.start_state;
    for (uint32_t joint = 0; joint < joint_count; ++joint) {
        if (!std::isfinite(start.position[joint]) ||
            !std::isfinite(start.velocity[joint]) ||
            !std::isfinite(start.acceleration[joint]) ||
            !std::isfinite(start.position_tolerance[joint]) ||
            !std::isfinite(start.velocity_tolerance[joint]) ||
            !std::isfinite(start.acceleration_tolerance[joint]) ||
            start.position_tolerance[joint] < 0.0 ||
            start.velocity_tolerance[joint] < 0.0 ||
            start.acceleration_tolerance[joint] < 0.0)
            return false;
    }
    return true;
}

std::shared_ptr<const Plan> find(mk_plan_handle handle) {
    std::lock_guard lock(registry_mutex);
    const auto found = plans.find(handle.id);
    return found == plans.end() ? nullptr : found->second;
}

} // namespace

extern "C" {

mk_result MK_CALL mk_trajectory_create(uint32_t joint_count,
                                        mk_trajectory_handle *out_trajectory) {
    if (out_trajectory == nullptr || joint_count == 0 || joint_count > MK_MAX_JOINTS)
        return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    try {
        auto trajectory = std::make_unique<motionkit::Trajectory>(joint_count);
        std::lock_guard lock(registry_mutex);
        return register_trajectory(std::move(trajectory), out_trajectory);
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

void MK_CALL mk_trajectory_destroy(mk_trajectory_handle trajectory) {
    try {
        std::shared_ptr<motionkit::Trajectory> released;
        {
            std::lock_guard lock(registry_mutex);
            const auto found = trajectories.find(trajectory.id);
            if (found == trajectories.end()) return;
            released = std::move(found->second);
            trajectories.erase(found);
        }
    } catch (...) {
    }
}

mk_result MK_CALL mk_trajectory_append_segment(mk_trajectory_handle trajectory,
                                                const mk_segment *segment) {
    if (segment == nullptr) return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        return value->append(*segment) ? MK_OK : MK_ERROR_INVALID_ARGUMENT;
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_evaluate(mk_trajectory_handle trajectory,
                                          int64_t time_ns, mk_trajectory_state *out_state) {
    if (out_state == nullptr || out_state->struct_size < sizeof(mk_trajectory_state))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        return value->evaluate(time_ns, *out_state) ? MK_OK : MK_ERROR_INVALID_ARGUMENT;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_estimate_path_derivatives(
    mk_trajectory_handle trajectory, int64_t time_ns, uint64_t window_ns,
    mk_path_derivative_estimate *out_estimate) {
    if (out_estimate == nullptr || out_estimate->struct_size < sizeof(*out_estimate) ||
        time_ns < 0 || window_ns > static_cast<uint64_t>(INT64_MAX))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        std::vector<mk_segment> segments;
        segments.reserve(value->segment_count());
        for (uint32_t index = 0; index < value->segment_count(); ++index)
            segments.push_back(value->segment(index));
        return motionkit::estimate_path_derivatives(segments, time_ns, window_ns,
            *out_estimate) ? MK_OK : MK_ERROR_INVALID_ARGUMENT;
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_duration_ns(mk_trajectory_handle trajectory,
                                             int64_t *out_duration_ns) {
    if (out_duration_ns == nullptr) return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        *out_duration_ns = value->duration_ns();
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_joint_count(mk_trajectory_handle trajectory,
                                             uint32_t *out_joint_count) {
    if (out_joint_count == nullptr) return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        *out_joint_count = value->joint_count();
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_segment_count(mk_trajectory_handle trajectory,
                                               uint32_t *out_segment_count) {
    if (out_segment_count == nullptr) return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        *out_segment_count = value->segment_count();
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_get_segment(mk_trajectory_handle trajectory,
                                             uint32_t index, mk_segment *out_segment) {
    if (out_segment == nullptr || out_segment->struct_size < sizeof(*out_segment))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        if (index >= value->segment_count()) return MK_ERROR_INVALID_ARGUMENT;
        *out_segment = value->segment(index);
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_boundary_continuity(mk_trajectory_handle trajectory,
    uint32_t boundary_index, mk_continuity *out_continuity) {
    if (out_continuity == nullptr || out_continuity->struct_size < sizeof(mk_continuity))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        if (value->segment_count() < 2 || boundary_index >= value->segment_count() - 1)
            return MK_ERROR_INVALID_ARGUMENT;
        const auto &before_segment = value->segment(boundary_index);
        const auto &after_segment = value->segment(boundary_index + 1);
        mk_trajectory_state before{};
        mk_trajectory_state after{};
        motionkit::evaluate_segment(before_segment,
            static_cast<double>(before_segment.duration_ns) / 1'000'000'000.0, before);
        motionkit::evaluate_segment(after_segment, 0.0, after);
        mk_continuity report{};
        report.struct_size = out_continuity->struct_size;
        report.boundary_index = boundary_index;
        report.time_ns = after_segment.t0_ns;
        for (uint32_t joint = 0; joint < value->joint_count(); ++joint) {
            const double c0 = std::abs(before.position[joint] - after.position[joint]);
            const double c1 = std::abs(before.velocity[joint] - after.velocity[joint]);
            const double c2 = std::abs(before.acceleration[joint] - after.acceleration[joint]);
            if (c0 > report.c0_jump) { report.c0_jump = c0; report.c0_joint = joint; }
            if (c1 > report.c1_jump) { report.c1_jump = c1; report.c1_joint = joint; }
            if (c2 > report.c2_jump) { report.c2_jump = c2; report.c2_joint = joint; }
        }
        *out_continuity = report;
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_from_samples(uint32_t joint_count,
    const mk_sample *samples, uint32_t sample_count, mk_trajectory_handle *out_trajectory) {
    if (out_trajectory == nullptr || samples == nullptr || sample_count < 2 ||
        joint_count == 0 || joint_count > MK_MAX_JOINTS)
        return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    try {
        auto trajectory = std::make_unique<motionkit::Trajectory>(joint_count);
        for (uint32_t index = 0; index < sample_count; ++index) {
            if (!valid_sample(samples[index], joint_count)) return MK_ERROR_INVALID_ARGUMENT;
            if (index == 0) continue;
            const auto &before = samples[index - 1];
            const auto &after = samples[index];
            if (after.time_ns <= before.time_ns) return MK_ERROR_INVALID_ARGUMENT;
            const uint64_t duration = static_cast<uint64_t>(after.time_ns) -
                static_cast<uint64_t>(before.time_ns);
            if (duration > static_cast<uint64_t>(INT64_MAX)) return MK_ERROR_INVALID_ARGUMENT;
            mk_segment segment{};
            segment.struct_size = sizeof(segment);
            segment.t0_ns = before.time_ns;
            segment.duration_ns = static_cast<int64_t>(duration);
            segment.degree = 1;
            segment.joint_count = joint_count;
            const double seconds = static_cast<double>(duration) / 1'000'000'000.0;
            for (uint32_t joint = 0; joint < joint_count; ++joint) {
                segment.coefficients[joint].value[0] = before.position[joint];
                segment.coefficients[joint].value[1] =
                    (after.position[joint] - before.position[joint]) / seconds;
            }
            if (!trajectory->append(segment)) return MK_ERROR_INVALID_ARGUMENT;
        }
        std::lock_guard lock(registry_mutex);
        return register_trajectory(std::move(trajectory), out_trajectory);
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_validate(mk_trajectory_handle trajectory, const mk_limits *limits,
                              mk_validation_report *out_report) {
    if (limits == nullptr || out_report == nullptr ||
        out_report->struct_size < sizeof(mk_validation_report))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        return motionkit::validate(*value, *limits, *out_report);
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_report_set_task_space(mk_validation_report *report,
    uint32_t status, double worst, double time_seconds, double tolerance,
    uint64_t resolution_ns) {
    if (report == nullptr || report->struct_size < sizeof(mk_validation_report) ||
        (status != MK_CHECK_PASSED && status != MK_CHECK_FAILED) ||
        !std::isfinite(worst) || worst < 0.0 ||
        !std::isfinite(time_seconds) || time_seconds < 0.0 ||
        !std::isfinite(tolerance) || tolerance < 0.0 || resolution_ns == 0)
        return MK_ERROR_INVALID_ARGUMENT;
    mk_validation_check check{};
    check.status = status;
    check.joint = UINT32_MAX;
    check.method = MK_CHECK_METHOD_SAMPLED;
    check.value = worst;
    check.time_seconds = time_seconds;
    check.limit = tolerance;
    check.margin = tolerance - worst;
    check.resolution_ns = resolution_ns;
    report->checks[MK_CHECK_TASK_SPACE] = check;
    return MK_OK;
}

mk_result MK_CALL mk_report_set_collision(mk_validation_report *report,
    uint32_t status, uint32_t method, double distance, double required, double time_seconds,
    double segment_start, double segment_end, uint64_t resolution_ns, uint32_t object_a, uint32_t object_b,
    const uint8_t *name_a, uint32_t name_a_length, const uint8_t *name_b, uint32_t name_b_length) {
    if (report == nullptr || report->struct_size < sizeof(mk_validation_report) ||
        (status != MK_CHECK_PASSED && status != MK_CHECK_FAILED) ||
        (method != MK_CHECK_METHOD_BOUND && method != MK_CHECK_METHOD_SAMPLED) ||
        !std::isfinite(distance) || !std::isfinite(required) || required < 0.0 ||
        !std::isfinite(time_seconds) || time_seconds < 0.0 || !std::isfinite(segment_start) ||
        !std::isfinite(segment_end) || segment_start < 0.0 || segment_end < segment_start ||
        (method == MK_CHECK_METHOD_SAMPLED) != (resolution_ns != 0) ||
        (name_a_length && name_a == nullptr) || (name_b_length && name_b == nullptr))
        return MK_ERROR_INVALID_ARGUMENT;
    mk_validation_check check{};
    check.status = status;
    check.joint = UINT32_MAX;
    check.method = method;
    check.value = distance;
    check.time_seconds = time_seconds;
    check.limit = required;
    check.margin = distance - required;
    check.resolution_ns = resolution_ns;
    report->collision = check;
    mk_collision_pair pair{};
    pair.object_a = object_a;
    pair.object_b = object_b;
    pair.segment_start = segment_start;
    pair.segment_end = segment_end;
    const uint32_t a = std::min<uint32_t>(name_a_length, MK_ASSUMPTION_LENGTH - 1);
    const uint32_t b = std::min<uint32_t>(name_b_length, MK_ASSUMPTION_LENGTH - 1);
    for (uint32_t i = 0; i < a; ++i) pair.name_a[i] = char(name_a[i]);
    for (uint32_t i = 0; i < b; ++i) pair.name_b[i] = char(name_b[i]);
    report->collision_pair = pair;
    return MK_OK;
}

mk_result MK_CALL mk_report_set_task_space_bound(mk_validation_report *report,
    double upper_bound, double tolerance) {
    if (report == nullptr || report->struct_size < sizeof(mk_validation_report) ||
        !std::isfinite(upper_bound) || upper_bound < 0.0 ||
        !std::isfinite(tolerance) || tolerance < upper_bound)
        return MK_ERROR_INVALID_ARGUMENT;
    mk_validation_check check{};
    check.status = MK_CHECK_PASSED;
    check.joint = UINT32_MAX;
    check.method = MK_CHECK_METHOD_BOUND;
    check.value = upper_bound;
    check.limit = tolerance;
    check.margin = tolerance - upper_bound;
    report->checks[MK_CHECK_TASK_SPACE] = check;
    return MK_OK;
}

mk_result MK_CALL mk_plan_create(mk_trajectory_handle trajectory, const mk_plan_spec *spec,
    const mk_limits *limits, mk_plan_handle *out_plan, mk_validation_report *out_report) {
    if (out_plan == nullptr || out_report == nullptr ||
        out_report->struct_size < sizeof(mk_validation_report) ||
        spec == nullptr || limits == nullptr)
        return MK_ERROR_INVALID_ARGUMENT;
    out_plan->id = 0;
    try {
        const auto value = find(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        if (!valid_plan_spec(*spec, *limits, value->joint_count()))
            return MK_ERROR_INVALID_ARGUMENT;
        const auto event_count = spec->struct_size >= sizeof(mk_plan_spec) ?
            spec->event_count : 0u;
        for (uint32_t index = 0; index < event_count; ++index)
            if (spec->events[index].time_ns > static_cast<uint64_t>(value->duration_ns()))
                return MK_ERROR_INVALID_ARGUMENT;
        if (spec->planning_authority != MK_AUTHORITY_MATERIA)
            return spec->planning_authority == MK_AUTHORITY_BACKEND ?
                MK_ERROR_UNSUPPORTED : MK_ERROR_INVALID_ARGUMENT;
        const auto result = motionkit::validate(*value, *limits, *out_report);
        if (result != MK_OK) return result;
        for (const auto &check : out_report->checks)
            if (check.status == MK_CHECK_FAILED) return MK_ERROR_LIMIT;
        mk_plan_spec normalized{};
        std::memcpy(&normalized, spec, std::min<size_t>(spec->struct_size, sizeof(normalized)));
        normalized.struct_size = sizeof(normalized);
        auto plan = std::make_unique<Plan>(Plan{*value, normalized, *out_report, {}, {}, {}, {}});
        flatten_segments(*plan);
        std::lock_guard lock(registry_mutex);
        if (plans.size() >= UINT32_MAX - 1) return MK_ERROR_OUT_OF_MEMORY;
        while (next_plan_id == 0 || plans.count(next_plan_id) != 0) ++next_plan_id;
        const uint32_t id = next_plan_id++;
        plans.emplace(id, std::move(plan));
        out_plan->id = id;
        return MK_OK;
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

void MK_CALL mk_plan_destroy(mk_plan_handle plan) {
    try {
        std::shared_ptr<const Plan> released;
        {
            std::lock_guard lock(registry_mutex);
            const auto found = plans.find(plan.id);
            if (found == plans.end()) return;
            released = std::move(found->second);
            plans.erase(found);
        }
    } catch (...) {
    }
}

size_t MK_CALL mk_plan_segment_array_count(mk_plan_handle plan) {
    const auto value = find(plan);
    return value == nullptr ? 0 : value->starts.size();
}

const int64_t *MK_CALL mk_plan_segment_starts(mk_plan_handle plan) {
    const auto value = find(plan);
    return value == nullptr ? nullptr : value->starts.data();
}

const int64_t *MK_CALL mk_plan_segment_durations(mk_plan_handle plan) {
    const auto value = find(plan);
    return value == nullptr ? nullptr : value->durations.data();
}

const int32_t *MK_CALL mk_plan_segment_degrees(mk_plan_handle plan) {
    const auto value = find(plan);
    return value == nullptr ? nullptr : value->degrees.data();
}

size_t MK_CALL mk_plan_coefficient_array_count(mk_plan_handle plan) {
    const auto value = find(plan);
    return value == nullptr ? 0 : value->coefficients.size();
}

const double *MK_CALL mk_plan_segment_coefficients(mk_plan_handle plan) {
    const auto value = find(plan);
    return value == nullptr ? nullptr : value->coefficients.data();
}

mk_result MK_CALL mk_plan_get_info(mk_plan_handle plan, mk_plan_info *out_info) {
    if (out_info == nullptr || out_info->struct_size < offsetof(mk_plan_info, event_count))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(plan);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        mk_plan_info info{};
        info.struct_size = sizeof(info);
        info.joint_count = value->trajectory.joint_count();
        info.plan_id = value->spec.plan_id;
        info.model_revision = value->spec.model_revision;
        info.calibration_revision = value->spec.calibration_revision;
        info.trajectory_revision = value->trajectory.revision();
        info.required_capabilities = value->spec.required_capabilities;
        info.planning_authority = value->spec.planning_authority;
        info.duration_ns = value->trajectory.duration_ns();
        info.event_count = value->spec.event_count;
        std::memcpy(out_info, &info, std::min<size_t>(out_info->struct_size, sizeof(info)));
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_plan_get_event(mk_plan_handle plan, uint32_t index,
    mk_timed_event *out_event) {
    if (out_event == nullptr) return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(plan);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        if (index >= value->spec.event_count) return MK_ERROR_INVALID_ARGUMENT;
        *out_event = value->spec.events[index];
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_plan_get_start_state(mk_plan_handle plan,
                                           mk_start_state *out_start_state) {
    if (out_start_state == nullptr || out_start_state->struct_size < sizeof(mk_start_state))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(plan);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        *out_start_state = value->spec.start_state;
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_plan_get_report(mk_plan_handle plan,
                                      mk_validation_report *out_report) {
    if (out_report == nullptr || out_report->struct_size < sizeof(mk_validation_report))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(plan);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        *out_report = value->report;
        return MK_OK;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_plan_evaluate(mk_plan_handle plan, int64_t time_ns,
                                    mk_trajectory_state *out_state) {
    if (out_state == nullptr || out_state->struct_size < sizeof(mk_trajectory_state))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto value = find(plan);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        return value->trajectory.evaluate(time_ns, *out_state) ? MK_OK : MK_ERROR_INVALID_ARGUMENT;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

} // extern "C"
