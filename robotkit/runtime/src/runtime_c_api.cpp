#include "robotkit_runtime.h"
#include "robotkit_runtime.hpp"
#include "coupling_terms.hpp"
#if defined(RK_HAS_SERIAL_DEVICE)
#include "robotkit_device_serial_endpoint.hpp"
#endif
#include "rkd6_endpoint.hpp"
#include "runtime_registry.hpp"
#include "runtime_abi.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace {

std::mutex registry_mutex;
std::unordered_map<rk_robot_runtime, std::shared_ptr<robotkit::RobotRuntime>> runtimes;
rk_robot_runtime next_runtime = 1;

std::chrono::nanoseconds owner_period(const rk_robot_runtime_blueprint &blueprint) {
    return std::chrono::nanoseconds(blueprint.owner_period_ns == 0
        ? 10'000'000 : static_cast<int64_t>(blueprint.owner_period_ns));
}

} // namespace

namespace robotkit::internal {

std::shared_ptr<RobotRuntime> resolve_runtime(rk_robot_runtime handle) {
    std::lock_guard lock(registry_mutex);
    const auto found = runtimes.find(handle);
    return found == runtimes.end() ? nullptr : found->second;
}

rk_robot_runtime register_runtime(std::shared_ptr<RobotRuntime> runtime) {
    std::lock_guard lock(registry_mutex);
    while (next_runtime == RK_INVALID_ROBOT_RUNTIME || runtimes.count(next_runtime) != 0)
        ++next_runtime;
    const auto handle = next_runtime++;
    runtimes.emplace(handle, std::move(runtime));
    return handle;
}

void destroy_runtime(rk_robot_runtime handle) {
    std::shared_ptr<RobotRuntime> released;
    {
        std::lock_guard lock(registry_mutex);
        const auto found = runtimes.find(handle);
        if (found == runtimes.end())
            return;
        released = std::move(found->second);
        runtimes.erase(found);
    }
}

} // namespace robotkit::internal

extern "C" {

rk_result RK_CALL rk_robot_runtime_create(const rk_robot_runtime_blueprint *blueprint,
                                    rk_robot_runtime *out_runtime) {
    if (!out_runtime || rk_robot_runtime_blueprint_validate(blueprint) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    *out_runtime = RK_INVALID_ROBOT_RUNTIME;
    try {
        const auto copied = robotkit::internal::copy_blueprint(blueprint);
        std::shared_ptr<robotkit::RobotEndpoint> endpoint =
            std::make_shared<robotkit::InMemoryRobot>(blueprint->joint_count);
        auto runtime = std::make_shared<robotkit::RobotRuntime>(
            *copied, endpoint, owner_period(*copied));
        const auto handle = robotkit::internal::register_runtime(std::move(runtime));
        *out_runtime = handle;
        return RK_OK;
    } catch (...) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

rk_result RK_CALL rk_robot_runtime_create_serial6(const rk_robot_runtime_blueprint *blueprint,
                                                    const char *device_path, uint32_t baud,
                                                    const rk_serial_device_desc *device,
                                                    double max_target_error,
                                                    uint32_t step_tick_hz,
                                                    uint64_t link_loss_timeout_ns,
                                                    uint64_t clock_bound_ns,
                                                    uint64_t link_latency_ns,
                                                    rk_robot_runtime *out_runtime) {
    if (!out_runtime || !blueprint || rk_robot_runtime_blueprint_validate(blueprint) != RK_OK ||
        !device_path || !*device_path || blueprint->joint_count > RK_MAX_SERIAL_JOINTS ||
        !device || device->struct_size < sizeof(*device) || device->actuator_count == 0 ||
        device->actuator_count > RK_MAX_SERIAL_JOINTS ||
        !std::isfinite(max_target_error) || max_target_error < 0.0 ||
        step_tick_hz == 0 || link_loss_timeout_ns == 0 || clock_bound_ns == 0)
        return RK_ERROR_INVALID_ARGUMENT;
    std::array<std::uint8_t, 16> controller{};
    std::copy_n(device->controller, controller.size(), controller.begin());
    std::vector<robotkit::DeviceActuator6> layout;
    for (std::uint32_t i = 0; i < device->actuator_count; ++i) {
        robotkit::DeviceActuator6 actuator;
        actuator.joint = device->actuator_joint[i];
        actuator.ratio = device->actuator_ratio[i];
        actuator.offset = device->actuator_offset[i];
        actuator.steps_per_unit = device->actuator_steps_per_unit[i];
        actuator.max_rate = device->actuator_max_rate[i];
        actuator.direction_setup_ticks = device->actuator_direction_setup_ticks[i];
        actuator.dual_drive_skew_bound = device->actuator_skew_bound[i];
        const auto *id = device->actuator_ids + i * 64;
        const auto *end = std::find(id, id + 64, 0);
        if (end == id || end == id + 64) return RK_ERROR_INVALID_ARGUMENT;
        actuator.id.assign(reinterpret_cast<const char *>(id), reinterpret_cast<const char *>(end));
        layout.push_back(std::move(actuator));
    }
    *out_runtime = RK_INVALID_ROBOT_RUNTIME;
#if !defined(RK_HAS_SERIAL_DEVICE)
    // Built without serial ports (RK_BUILD_SERIAL_DEVICE).
    (void)step_tick_hz; (void)link_loss_timeout_ns; (void)clock_bound_ns; (void)link_latency_ns; (void)baud;
    return RK_ERROR_UNSUPPORTED;
#else
    try {
        const auto copied = robotkit::internal::copy_blueprint(blueprint);
        const auto period = owner_period(*copied);
        rk_result endpoint_error = RK_ERROR_BACKEND;
        auto endpoint = robotkit::DeviceSerialEndpoint::open(device_path, baud, *copied,
            controller, max_target_error, step_tick_hz, link_loss_timeout_ns,
            clock_bound_ns, link_latency_ns, layout, &endpoint_error);
        if (!endpoint) return endpoint_error;
        auto runtime = std::make_shared<robotkit::RobotRuntime>(
            *copied, std::static_pointer_cast<robotkit::RobotEndpoint>(endpoint), period);
        *out_runtime = robotkit::internal::register_runtime(std::move(runtime));
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
#endif
}

rk_result RK_CALL rk_serial_device_identify(const char *device_path, uint32_t baud,
                                            rk_controller_id *out_controller) {
    if (!device_path || !*device_path || !out_controller) return RK_ERROR_INVALID_ARGUMENT;
    try {
        std::array<std::uint8_t, 16> controller{};
        const auto result = robotkit::DeviceSerialEndpoint::identify(device_path, baud, controller);
        if (result == RK_OK) std::copy(controller.begin(), controller.end(), out_controller->bytes);
        return result;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
}

void RK_CALL rk_robot_runtime_destroy(rk_robot_runtime runtime) {
    robotkit::internal::destroy_runtime(runtime);
}

rk_result RK_CALL rk_robot_runtime_start(rk_robot_runtime runtime) {
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->start() : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_stop(rk_robot_runtime runtime) {
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->stop() : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_require_reference(rk_robot_runtime runtime, uint32_t joint, uint32_t required) {
    if (required > 1) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->require_reference(joint, required != 0) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_latch_reference(rk_robot_runtime runtime, uint32_t joint) {
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->latch_reference(joint) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_reference_status(rk_robot_runtime runtime, uint32_t joint, uint32_t *out_referenced) {
    if (!out_referenced) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->reference_status(joint, *out_referenced) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_submit(rk_robot_runtime runtime, const rk_robot_command *command) {
    if (!command)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->submit(*command) : RK_ERROR_INVALID_HANDLE;
}

namespace {
/**
 * Copies segment arrays into a batch over every robot joint: once, the runtime's one copy.
 * Source joint j drives robot joint joint_map[j], or joint j without a map. A robot joint no
 * source joint drives follows its leaders when the blueprint couples it to some (a lead screw
 * turning with its axis, a CoreXY motor turning with two), and otherwise holds its held position.
 */
rk_result copy_segments(const int64_t *starts_ns, const int64_t *durations_ns,
    const int32_t *degrees, uint32_t segment_count, const double *coefficients,
    uint32_t coefficient_count, const int32_t *joint_map, uint32_t source_joint_count,
    uint32_t robot_joint_count, const double *held_positions,
    const rk_robot_runtime_blueprint *blueprint, robotkit::SegmentBatch &batch) {
    constexpr uint32_t stride = RK_TRAJECTORY_COEFFICIENT_STRIDE;
    if (segment_count == 0 || segment_count > RK_MAX_TRAJECTORY_QUEUE_POINTS ||
        source_joint_count == 0 || source_joint_count > robot_joint_count ||
        robot_joint_count > RK_MAX_TRAJECTORY_JOINTS ||
        !starts_ns || !durations_ns || !degrees || !coefficients ||
        static_cast<uint64_t>(coefficient_count) !=
            static_cast<uint64_t>(segment_count) * source_joint_count * stride)
        return RK_ERROR_INVALID_ARGUMENT;
    uint32_t targets[RK_MAX_TRAJECTORY_JOINTS];
    bool driven[RK_MAX_TRAJECTORY_JOINTS]{};
    for (uint32_t joint = 0; joint < source_joint_count; ++joint) {
        const int64_t target = joint_map ? joint_map[joint] : joint;
        if (target < 0 || target >= robot_joint_count || driven[target])
            return RK_ERROR_INVALID_ARGUMENT;
        targets[joint] = static_cast<uint32_t>(target);
        driven[target] = true;
    }
    if (source_joint_count < robot_joint_count && !held_positions)
        return RK_ERROR_INVALID_ARGUMENT;
    // Followers no source joint drives, ordered so each leader is final before the followers
    // derived from it.
    uint32_t followed[RK_MAX_JOINTS];
    uint32_t follow_count = 0;
    if (blueprint && blueprint->struct_size >= sizeof(*blueprint) &&
        source_joint_count < robot_joint_count &&
        !robotkit::internal::order_followers(blueprint->couplings, blueprint->coupling_count,
            robot_joint_count, driven, followed, follow_count))
        return RK_ERROR_INVALID_ARGUMENT;
    batch.segments.resize(segment_count);
    for (uint32_t index = 0; index < segment_count; ++index) {
        if (starts_ns[index] < starts_ns[0] || durations_ns[index] <= 0 || degrees[index] < 0 ||
            degrees[index] >= static_cast<int32_t>(stride))
            return RK_ERROR_INVALID_ARGUMENT;
        auto &segment = batch.segments[index];
        segment.time_from_start_ns = static_cast<uint64_t>(starts_ns[index] - starts_ns[0]);
        segment.duration_ns = static_cast<uint64_t>(durations_ns[index]);
        segment.degree = static_cast<uint32_t>(degrees[index]);
        segment.joint_count = robot_joint_count;
        for (uint32_t joint = 0; joint < robot_joint_count; ++joint)
            if (!driven[joint]) segment.coefficients[joint].value[0] = held_positions[joint];
        const double *source = coefficients + static_cast<size_t>(index) * source_joint_count * stride;
        for (uint32_t joint = 0; joint < source_joint_count; ++joint)
            for (uint32_t power = 0; power <= segment.degree; ++power)
                segment.coefficients[targets[joint]].value[power] = source[joint * stride + power];
        for (uint32_t i = 0; i < follow_count; ++i) {
            auto &follower = segment.coefficients[followed[i]].value;
            for (uint32_t power = 0; power <= segment.degree; ++power) {
                double sum = 0.0;
                for (uint32_t k = 0; k < blueprint->coupling_count; ++k)
                    if (blueprint->couplings[k].follower == followed[i])
                        sum += blueprint->couplings[k].ratio *
                                segment.coefficients[blueprint->couplings[k].leader].value[power] +
                            (power == 0 ? blueprint->couplings[k].offset : 0.0);
                follower[power] = sum;
            }
        }
    }
    return RK_OK;
}
}

rk_result RK_CALL rk_robot_runtime_submit_segments(rk_robot_runtime runtime,
    const rk_robot_command *command, uint64_t tag, const int64_t *starts_ns,
    const int64_t *durations_ns, const int32_t *degrees, uint32_t segment_count,
    const double *coefficients, uint32_t coefficient_count) {
    if (!command) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    try {
        robotkit::SegmentBatch batch;
        batch.tag = tag;
        const auto joints = value->blueprint().joint_count;
        const auto copied = copy_segments(starts_ns, durations_ns, degrees, segment_count,
            coefficients, coefficient_count, nullptr, joints, joints, nullptr, nullptr, batch);
        return copied == RK_OK ? value->submit_segments(*command, std::move(batch)) : copied;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

rk_result RK_CALL rk_robot_runtime_submit_plan(rk_robot_runtime runtime,
    const rk_plan_header *header, const int64_t *starts_ns, const int64_t *durations_ns,
    const int32_t *degrees, uint32_t segment_count, const double *coefficients,
    uint32_t coefficient_count, const int32_t *joint_map, uint32_t source_joint_count,
    const rk_timed_event *events, uint32_t event_count) {
    if (!header || header->struct_size < offsetof(rk_plan_header, control_acceleration) || !joint_map ||
        header->ends_at_rest > 1 || (event_count > 0 && !events) ||
        event_count > RK_MAX_TRAJECTORY_QUEUE_POINTS)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    try {
        robotkit::PlanRequest plan;
        plan.sequence = header->sequence;
        plan.plan_id = header->plan_id;
        plan.model_revision = header->model_revision;
        plan.calibration_revision = header->calibration_revision;
        plan.required_capabilities = header->required_capabilities;
        plan.flags = header->flags;
        plan.replace_after_plan_id = header->replace_after_plan_id;
        plan.replace_after_time_ns = header->replace_after_time_ns;
        plan.ends_at_rest = header->ends_at_rest != 0;
        std::copy_n(header->start_position, RK_MAX_TRAJECTORY_JOINTS, plan.start_position);
        std::copy_n(header->start_velocity, RK_MAX_TRAJECTORY_JOINTS, plan.start_velocity);
        std::copy_n(header->start_acceleration, RK_MAX_TRAJECTORY_JOINTS, plan.start_acceleration);
        std::copy_n(header->position_tolerance, RK_MAX_TRAJECTORY_JOINTS, plan.position_tolerance);
        std::copy_n(header->velocity_tolerance, RK_MAX_TRAJECTORY_JOINTS, plan.velocity_tolerance);
        std::copy_n(header->acceleration_tolerance, RK_MAX_TRAJECTORY_JOINTS,
            plan.acceleration_tolerance);
        if (header->struct_size >= sizeof(*header))
            std::copy_n(header->control_acceleration, RK_MAX_TRAJECTORY_JOINTS, plan.control_acceleration);
        plan.segments.tag = header->tag;
        const auto copied = copy_segments(starts_ns, durations_ns, degrees, segment_count,
            coefficients, coefficient_count, joint_map, source_joint_count,
            value->blueprint().joint_count, header->start_position, &value->blueprint(),
            plan.segments);
        if (copied != RK_OK) return copied;
        plan.events.assign(events, events + event_count);
        return value->submit_plan(plan);
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

rk_result RK_CALL rk_robot_runtime_poll_events(
    rk_robot_runtime runtime, rk_event_record_batch *out_batch) {
    if (!out_batch || out_batch->struct_size < sizeof(*out_batch))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->poll_events(*out_batch) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_get_channel_value(
    rk_robot_runtime runtime, const char *channel, rk_event_value *out_value) {
    if (!channel || !out_value) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->channel_value(channel, *out_value) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_snapshot(rk_robot_runtime runtime, rk_robot_state *out_state) {
    if (!out_state || out_state->struct_size < sizeof(*out_state))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->snapshot(*out_state) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_snapshot_full(rk_robot_runtime runtime,
                                           rk_robot_snapshot *out_snapshot) {
    if (!out_snapshot ||
        out_snapshot->struct_size < offsetof(rk_robot_snapshot, calibration_revision))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    const auto caller_size = out_snapshot->struct_size;
    rk_robot_snapshot complete{};
    const auto result = value->snapshot_full(complete);
    if (result != RK_OK) return result;
    std::memcpy(out_snapshot, &complete,
        std::min<std::size_t>(caller_size, sizeof(complete)));
    out_snapshot->struct_size = caller_size;
    return RK_OK;
}

rk_result RK_CALL rk_robot_runtime_capabilities(rk_robot_runtime runtime,
                                           rk_robot_capabilities *out_capabilities) {
    if (!out_capabilities || out_capabilities->struct_size < sizeof(*out_capabilities))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    if (!value)
        return RK_ERROR_INVALID_HANDLE;
    out_capabilities->joint_count = 0;
    out_capabilities->supports_position_targets = 1;
    out_capabilities->supports_velocity_targets = 1;
    out_capabilities->supports_effort_targets = 1;
    out_capabilities->supports_prediction = 0;
    out_capabilities->supports_trajectory_queue = value->supports_trajectory_queue();
    out_capabilities->supports_execution_plans = value->supports_trajectory_queue();
    for (auto &reserved : out_capabilities->reserved)
        reserved = 0;
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    const auto result = value->snapshot(state);
    if (result != RK_OK)
        return result;
    out_capabilities->joint_count = state.joint_count;
    return RK_OK;
}

} // extern "C"
