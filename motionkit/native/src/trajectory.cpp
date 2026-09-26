#include "motionkit.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <new>
#include <unordered_map>

namespace {

std::mutex registry_mutex;
std::unordered_map<uint32_t, std::unique_ptr<motionkit::Trajectory>> trajectories;
uint32_t next_id = 1;

struct Plan {
    motionkit::Trajectory trajectory;
    mk_plan_spec spec;
    mk_validation_report report;
};
std::unordered_map<uint32_t, std::unique_ptr<Plan>> plans;
uint32_t next_plan_id = 1;

motionkit::Trajectory *get(mk_trajectory_handle handle) {
    const auto found = trajectories.find(handle.id);
    return found == trajectories.end() ? nullptr : found->second.get();
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
    if (spec.struct_size < sizeof(mk_plan_spec) ||
        spec.start_state.struct_size < sizeof(mk_start_state) ||
        spec.start_state.joint_count != joint_count || spec.plan_id == 0 ||
        spec.model_revision != limits.model_revision ||
        spec.calibration_revision != limits.calibration_revision ||
        (spec.required_capabilities & ~static_cast<uint64_t>(MK_CAP_TIMED_TRAJECTORY)) != 0)
        return false;
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

Plan *get(mk_plan_handle handle) {
    const auto found = plans.find(handle.id);
    return found == plans.end() ? nullptr : found->second.get();
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
        std::lock_guard lock(registry_mutex);
        trajectories.erase(trajectory.id);
    } catch (...) {
    }
}

mk_result MK_CALL mk_trajectory_append_segment(mk_trajectory_handle trajectory,
                                                const mk_segment *segment) {
    if (segment == nullptr) return MK_ERROR_INVALID_ARGUMENT;
    try {
        std::lock_guard lock(registry_mutex);
        auto *value = get(trajectory);
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
        std::lock_guard lock(registry_mutex);
        auto *value = get(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        return value->evaluate(time_ns, *out_state) ? MK_OK : MK_ERROR_INVALID_ARGUMENT;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_trajectory_duration_ns(mk_trajectory_handle trajectory,
                                             int64_t *out_duration_ns) {
    if (out_duration_ns == nullptr) return MK_ERROR_INVALID_ARGUMENT;
    try {
        std::lock_guard lock(registry_mutex);
        auto *value = get(trajectory);
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
        std::lock_guard lock(registry_mutex);
        auto *value = get(trajectory);
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
        std::lock_guard lock(registry_mutex);
        auto *value = get(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        *out_segment_count = value->segment_count();
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
        std::lock_guard lock(registry_mutex);
        auto *value = get(trajectory);
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

mk_result MK_CALL mk_generate_state_to_state(const mk_state_to_state_request *request,
    mk_trajectory_handle *out_trajectory, int32_t *out_ruckig_result) {
    if (out_trajectory != nullptr) out_trajectory->id = 0;
    if (out_ruckig_result != nullptr) *out_ruckig_result = -100; // Ruckig ErrorInvalidInput.
    if (request == nullptr || out_trajectory == nullptr || out_ruckig_result == nullptr ||
        request->struct_size < sizeof(mk_state_to_state_request) ||
        request->joint_count == 0 || request->joint_count > MK_MAX_JOINTS)
        return MK_ERROR_INVALID_ARGUMENT;
    *out_ruckig_result = 0;
    try {
        auto generated = std::make_unique<motionkit::Trajectory>(request->joint_count);
        const auto result = motionkit::generate(*request, *generated, *out_ruckig_result);
        if (result != MK_OK) return result;
        std::lock_guard lock(registry_mutex);
        return register_trajectory(std::move(generated), out_trajectory);
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_GENERATION;
    }
}

mk_result MK_CALL mk_validate(mk_trajectory_handle trajectory, const mk_limits *limits,
                              mk_validation_report *out_report) {
    if (limits == nullptr || out_report == nullptr ||
        out_report->struct_size < sizeof(mk_validation_report))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        std::lock_guard lock(registry_mutex);
        const auto *value = get(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        return motionkit::validate(*value, *limits, *out_report);
    } catch (const std::bad_alloc &) {
        return MK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

mk_result MK_CALL mk_plan_create(mk_trajectory_handle trajectory, const mk_plan_spec *spec,
    const mk_limits *limits, mk_plan_handle *out_plan, mk_validation_report *out_report) {
    if (out_plan == nullptr || out_report == nullptr ||
        out_report->struct_size < sizeof(mk_validation_report) ||
        spec == nullptr || limits == nullptr)
        return MK_ERROR_INVALID_ARGUMENT;
    out_plan->id = 0;
    try {
        std::lock_guard lock(registry_mutex);
        const auto *value = get(trajectory);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        if (!valid_plan_spec(*spec, *limits, value->joint_count()))
            return MK_ERROR_INVALID_ARGUMENT;
        if (spec->planning_authority != MK_AUTHORITY_MATERIA)
            return spec->planning_authority == MK_AUTHORITY_BACKEND ?
                MK_ERROR_UNSUPPORTED : MK_ERROR_INVALID_ARGUMENT;
        const auto result = motionkit::validate(*value, *limits, *out_report);
        if (result != MK_OK) return result;
        for (const auto &check : out_report->checks)
            if (check.status == MK_CHECK_FAILED) return MK_ERROR_LIMIT;
        if (plans.size() >= UINT32_MAX - 1) return MK_ERROR_OUT_OF_MEMORY;
        auto plan = std::make_unique<Plan>(Plan{*value, *spec, *out_report});
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
        std::lock_guard lock(registry_mutex);
        plans.erase(plan.id);
    } catch (...) {
    }
}

mk_result MK_CALL mk_plan_get_info(mk_plan_handle plan, mk_plan_info *out_info) {
    if (out_info == nullptr || out_info->struct_size < sizeof(mk_plan_info))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        std::lock_guard lock(registry_mutex);
        const auto *value = get(plan);
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
        *out_info = info;
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
        std::lock_guard lock(registry_mutex);
        const auto *value = get(plan);
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
        std::lock_guard lock(registry_mutex);
        const auto *value = get(plan);
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
        std::lock_guard lock(registry_mutex);
        const auto *value = get(plan);
        if (value == nullptr) return MK_ERROR_INVALID_HANDLE;
        return value->trajectory.evaluate(time_ns, *out_state) ? MK_OK : MK_ERROR_INVALID_ARGUMENT;
    } catch (...) {
        return MK_ERROR_INVALID_ARGUMENT;
    }
}

} // extern "C"
