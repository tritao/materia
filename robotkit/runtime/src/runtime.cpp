#include "robotkit_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <new>
#include <utility>

namespace robotkit {

namespace {

uint64_t monotonic_now_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

bool lifecycle_kind(rk_command_kind kind) {
    return kind != RK_COMMAND_NONE && kind != RK_COMMAND_JOINT_TARGETS &&
        kind != RK_COMMAND_TRAJECTORY_CHUNK && kind != RK_COMMAND_TRAJECTORY_SEGMENTS;
}

uint32_t queued_knot_count(const std::deque<RobotRuntime::RuntimeTrajectoryPoint> &trajectory) {
    uint32_t count = 0;
    for (const auto &knot : trajectory)
        count += knot.counted_knot ? 1u : 0u;
    return count;
}

mk_segment chord_segment(const rk_trajectory_point &before,
                         const rk_trajectory_point &after) {
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.t0_ns = static_cast<int64_t>(before.time_from_start_ns);
    segment.duration_ns = static_cast<int64_t>(
        after.time_from_start_ns - before.time_from_start_ns);
    segment.degree = 1;
    segment.joint_count = before.joint_count;
    const double seconds = static_cast<double>(segment.duration_ns) * 1e-9;
    for (uint32_t joint = 0; joint < segment.joint_count; ++joint) {
        segment.coefficients[joint].value[0] = before.positions[joint];
        segment.coefficients[joint].value[1] =
            (after.positions[joint] - before.positions[joint]) / seconds;
    }
    return segment;
}

mk_segment native_segment(const rk_trajectory_segment &input, uint64_t base_time) {
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.t0_ns = static_cast<int64_t>(base_time + input.time_from_start_ns);
    segment.duration_ns = static_cast<int64_t>(input.duration_ns);
    segment.degree = input.degree;
    segment.joint_count = input.joint_count;
    for (uint32_t joint = 0; joint < input.joint_count; ++joint)
        std::copy_n(input.coefficients[joint].value, input.degree + 1,
            segment.coefficients[joint].value);
    return segment;
}

void evaluate_knot(const RobotRuntime::RuntimeTrajectoryPoint &knot, uint64_t time_ns,
                   double *positions, double *velocities = nullptr,
                   double *accelerations = nullptr) {
    mk_trajectory_state evaluated{};
    if (knot.has_segment) {
        const auto elapsed = time_ns > knot.point.time_from_start_ns
            ? std::min(time_ns - knot.point.time_from_start_ns,
                static_cast<uint64_t>(knot.segment.duration_ns)) : 0;
        motionkit::evaluate_segment(knot.segment, static_cast<double>(elapsed) * 1e-9,
            evaluated);
    } else {
        evaluated.joint_count = knot.point.joint_count;
        std::copy_n(knot.point.positions, evaluated.joint_count, evaluated.position);
    }
    std::copy_n(evaluated.position, evaluated.joint_count, positions);
    if (velocities) std::copy_n(evaluated.velocity, evaluated.joint_count, velocities);
    if (accelerations) std::copy_n(evaluated.acceleration, evaluated.joint_count, accelerations);
}

rk_result validate_queued_path(
    const std::deque<RobotRuntime::RuntimeTrajectoryPoint> &trajectory,
    const rk_robot_runtime_blueprint &blueprint) {
    if (trajectory.empty()) return RK_OK;
    for (const auto &knot : trajectory) {
        if (!knot.legacy_point) continue;
        for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
            const auto &limit = blueprint.joints[joint];
            if (knot.point.positions[joint] < limit.lower_limit ||
                knot.point.positions[joint] > limit.upper_limit)
                return RK_ERROR_LIMIT;
        }
    }
    for (std::size_t index = 1; index < trajectory.size(); ++index) {
        if (trajectory[index - 1].point.time_from_start_ns ==
            trajectory[index].point.time_from_start_ns) {
            for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint)
                if (std::abs(trajectory[index - 1].point.positions[joint] -
                        trajectory[index].point.positions[joint]) > 1e-9)
                    return RK_ERROR_LIMIT;
        }
    }
    mk_trajectory_handle handle{};
    if (mk_trajectory_create(blueprint.joint_count, &handle) != MK_OK)
        return RK_ERROR_OUT_OF_MEMORY;
    uint32_t segments = 0;
    for (const auto &knot : trajectory) {
        if (!knot.has_segment) continue;
        if (mk_trajectory_append_segment(handle, &knot.segment) != MK_OK) {
            mk_trajectory_destroy(handle);
            return RK_ERROR_LIMIT;
        }
        ++segments;
    }
    if (segments == 0) {
        mk_trajectory_destroy(handle);
        return RK_OK;
    }
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = blueprint.joint_count;
    limits.max_continuity_jump[0] = 1e-9;
    for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        const auto &source = blueprint.joints[joint];
        limits.position_claimed[joint] = 1;
        limits.position_lower[joint] = source.lower_limit;
        limits.position_upper[joint] = source.upper_limit;
        limits.max_velocity[joint] = source.max_velocity;
        limits.max_acceleration[joint] = source.max_acceleration;
    }
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    const auto result = mk_validate(handle, &limits, &report);
    mk_trajectory_destroy(handle);
    if (result != MK_OK) return RK_ERROR_LIMIT;
    for (const auto &check : report.checks)
        if (check.status == MK_CHECK_FAILED) return RK_ERROR_LIMIT;
    return RK_OK;
}

std::vector<mk_segment> path_region(
    const std::deque<RobotRuntime::RuntimeTrajectoryPoint> &trajectory,
    const RobotRuntime::RuntimeTrajectoryPoint *history = nullptr) {
    std::vector<mk_segment> region;
    region.reserve(trajectory.size() + (history != nullptr ? 1 : 0));
    if (history != nullptr && history->has_segment)
        region.push_back(history->segment);
    for (const auto &knot : trajectory)
        if (knot.has_segment) region.push_back(knot.segment);
    return region;
}

} // namespace

InMemoryRobot::InMemoryRobot(uint32_t joint_count)
    : joint_count_(std::min(joint_count, static_cast<uint32_t>(RK_MAX_JOINTS))) {}

rk_result InMemoryRobot::apply(const rk_robot_command &command) {
    if (stopped_ && command.kind != RK_COMMAND_STOP && command.kind != RK_COMMAND_RESET_SAFETY)
        return RK_ERROR_SAFETY_STOPPED;

    if (command.kind == RK_COMMAND_EMERGENCY_STOP) {
        stopped_ = true;
        return RK_OK;
    }
    if (command.kind == RK_COMMAND_STOP) {
        stopped_ = false;
        for (uint32_t index = 0; index < joint_count_; ++index)
            has_target_[index] = false;
        return RK_OK;
    }
    if (command.kind == RK_COMMAND_RESET_SAFETY) {
        stopped_ = false;
        return RK_OK;
    }
    if (command.kind != RK_COMMAND_JOINT_TARGETS && command.kind != RK_COMMAND_NONE)
        return RK_ERROR_UNSUPPORTED;

    for (uint32_t index = 0; index < command.target_count; ++index) {
        const auto &target = command.targets[index];
        if (target.joint >= joint_count_ || target.mode < RK_TARGET_POSITION ||
            target.mode > RK_TARGET_EFFORT)
            return RK_ERROR_UNSUPPORTED;
    }
    for (uint32_t index = 0; index < command.target_count; ++index) {
        const auto &target = command.targets[index];
        targets_[target.joint] = target;
        has_target_[target.joint] = true;
    }
    return RK_OK;
}

rk_result InMemoryRobot::sample(uint64_t timestamp_ns, rk_robot_state &state) {
    const auto elapsed_seconds = has_sample_timestamp_ &&
        timestamp_ns > last_sample_timestamp_ns_
        ? static_cast<double>(timestamp_ns - last_sample_timestamp_ns_) / 1'000'000'000.0
        : 0.0;
    state.struct_size = sizeof(state);
    state.source_timestamp_ns = timestamp_ns;
    state.received_timestamp_ns = timestamp_ns;
    state.joint_count = joint_count_;
    for (uint32_t index = 0; index < joint_count_; ++index) {
        const double old_position = state.position[index];
        state.velocity[index] = 0.0;
        state.effort[index] = 0.0;
        if (!stopped_ && has_target_[index]) {
            const auto &target = targets_[index];
            if (target.mode == RK_TARGET_POSITION) {
                state.position[index] = target.target;
                state.velocity[index] = elapsed_seconds > 0.0
                    ? (state.position[index] - old_position) / elapsed_seconds
                    : 0.0;
            } else if (target.mode == RK_TARGET_VELOCITY) {
                state.velocity[index] = target.target;
                if (elapsed_seconds > 0.0)
                    state.position[index] += target.target * elapsed_seconds;
            } else if (target.mode == RK_TARGET_EFFORT) {
                state.effort[index] = target.target;
            }
        }
    }
    last_sample_timestamp_ns_ = timestamp_ns;
    has_sample_timestamp_ = true;
    return RK_OK;
}

RobotRuntime::RobotRuntime(const rk_robot_runtime_blueprint &blueprint,
                           std::shared_ptr<RobotEndpoint> endpoint,
                 std::chrono::nanoseconds period)
    : blueprint_(blueprint), endpoint_(std::move(endpoint)), period_(period) {
    state_.struct_size = sizeof(state_);
    state_.joint_count = blueprint_.joint_count;
    state_.safety = endpoint_ ? endpoint_->initial_safety_state() : RK_SAFETY_READY;
    state_.mode = state_.safety == RK_SAFETY_EMERGENCY_STOP ||
        state_.safety == RK_SAFETY_FAULT ? RK_ROBOT_MODE_FAULT : RK_ROBOT_MODE_IDLE;
}

RobotRuntime::~RobotRuntime() {
    stop();
}

rk_result RobotRuntime::start() {
    std::lock_guard lock(queue_mutex_);
    if (externally_driven_ || running_ || stopping_ || endpoint_ == nullptr ||
        rk_robot_runtime_blueprint_validate(&blueprint_) != RK_OK)
        return RK_ERROR_INVALID_STATE;
    running_ = true;
    worker_ = std::thread([this] { run(); });
    return RK_OK;
}

rk_result RobotRuntime::stop() {
    {
        std::lock_guard lock(queue_mutex_);
        if (!running_ && !worker_.joinable())
            return RK_OK;
        stopping_ = true;
        queue_condition_.notify_all();
    }
    if (worker_.joinable())
        worker_.join();
    std::lock_guard lock(queue_mutex_);
    running_ = false;
    stopping_ = false;
    return RK_OK;
}

rk_result RobotRuntime::submit(const rk_robot_command &command) {
    if (command.kind == RK_COMMAND_TRAJECTORY_CHUNK ||
        command.kind == RK_COMMAND_TRAJECTORY_SEGMENTS)
        return RK_ERROR_INVALID_ARGUMENT;
    if (rk_robot_command_validate_for_blueprint(&command, &blueprint_) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    try {
        std::lock_guard lock(queue_mutex_);
        if (command.sequence == 0 || command.sequence <= last_command_sequence_)
            return RK_ERROR_STALE_COMMAND;
        if (commands_.size() >= 128)
            return RK_ERROR_QUEUE_FULL;
        last_command_sequence_ = command.sequence;
        QueuedCommand queued;
        queued.command = command;
        commands_.push_back(std::move(queued));
        queue_condition_.notify_all();
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

rk_result RobotRuntime::submit_trajectory(const rk_robot_command &command,
                                          const rk_trajectory_chunk &chunk) {
    if (command.kind != RK_COMMAND_TRAJECTORY_CHUNK ||
        rk_robot_command_validate_for_blueprint(&command, &blueprint_) != RK_OK ||
        rk_trajectory_chunk_validate_for_blueprint(&chunk, &blueprint_) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    if (!supports_trajectory_queue())
        return RK_ERROR_UNSUPPORTED;
    // The published depth may lag the owner, which only ever shrinks the
    // queue between cycles, so this bound is conservative.
    uint32_t queued_points = 0;
    {
        std::lock_guard state_lock(state_mutex_);
        queued_points = state_.trajectory_queue_depth;
    }
    try {
        std::lock_guard lock(queue_mutex_);
        if (command.sequence == 0 || command.sequence <= last_command_sequence_)
            return RK_ERROR_STALE_COMMAND;
        if (commands_.size() >= 128)
            return RK_ERROR_QUEUE_FULL;
        uint64_t pending_points = queued_points;
        for (const auto &pending : commands_)
            if (pending.trajectory != nullptr)
                pending_points += pending.trajectory->point_count;
            else if (pending.segments != nullptr)
                pending_points += pending.segments->segment_count;
        if (pending_points + chunk.point_count > RK_MAX_TRAJECTORY_QUEUE_POINTS)
            return RK_ERROR_QUEUE_FULL;
        auto payload = std::make_shared<rk_trajectory_chunk>(chunk);
        last_command_sequence_ = command.sequence;
        QueuedCommand queued;
        queued.command = command;
        queued.trajectory = std::move(payload);
        commands_.push_back(std::move(queued));
        queue_condition_.notify_all();
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

rk_result RobotRuntime::submit_segments(const rk_robot_command &command,
                                        const rk_trajectory_segment_chunk &chunk) {
    if (command.kind != RK_COMMAND_TRAJECTORY_SEGMENTS ||
        rk_robot_command_validate_for_blueprint(&command, &blueprint_) != RK_OK ||
        rk_trajectory_segment_chunk_validate_for_blueprint(&chunk, &blueprint_) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    if (!supports_trajectory_queue()) return RK_ERROR_UNSUPPORTED;
    uint32_t queued_knots = 0;
    {
        std::lock_guard state_lock(state_mutex_);
        queued_knots = state_.trajectory_queue_depth;
    }
    try {
        std::lock_guard lock(queue_mutex_);
        if (command.sequence == 0 || command.sequence <= last_command_sequence_)
            return RK_ERROR_STALE_COMMAND;
        if (commands_.size() >= 128) return RK_ERROR_QUEUE_FULL;
        uint64_t pending_knots = queued_knots;
        for (const auto &pending : commands_)
            if (pending.trajectory != nullptr)
                pending_knots += pending.trajectory->point_count;
            else if (pending.segments != nullptr)
                pending_knots += pending.segments->segment_count;
        if (pending_knots + chunk.segment_count > RK_MAX_TRAJECTORY_QUEUE_POINTS)
            return RK_ERROR_QUEUE_FULL;
        auto payload = std::make_shared<rk_trajectory_segment_chunk>(chunk);
        last_command_sequence_ = command.sequence;
        QueuedCommand queued;
        queued.command = command;
        queued.segments = std::move(payload);
        commands_.push_back(std::move(queued));
        queue_condition_.notify_all();
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

rk_result RobotRuntime::submit_plan(const rk_plan_submission &plan) {
    if (rk_plan_submission_validate_for_blueprint(&plan, &blueprint_) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    if (plan.model_revision != blueprint_.revision ||
        plan.calibration_revision != blueprint_.calibration_revision)
        return RK_ERROR_MODEL_MISMATCH;
    if ((plan.required_capabilities & ~RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE) != 0 ||
        !supports_trajectory_queue())
        return RK_ERROR_UNSUPPORTED;
    try {
        std::lock_guard owner_lock(owner_mutex_);
        std::lock_guard queue_lock(queue_mutex_);
        if (plan.sequence <= last_command_sequence_) return RK_ERROR_STALE_COMMAND;
        // A pending legacy command could change the anchor before the owner
        // applies it. Reject rather than accepting a plan against stale state.
        if (!commands_.empty()) return RK_ERROR_INVALID_STATE;
        std::lock_guard state_lock(state_mutex_);
        const bool stopped_and_held = state_.safety == RK_SAFETY_STOPPING &&
            !control_.trajectory_active && !control_.stop_ramp_active;
        if ((state_.safety != RK_SAFETY_READY && !stopped_and_held) ||
            control_.stop_ramp_active)
            return RK_ERROR_INVALID_STATE;
        auto candidate = control_.trajectory;
        const bool replace = plan.replace_after_plan_id != 0;
        uint64_t base_time = 0;
        const uint64_t lead = blueprint_.commit_lead_ns != 0 ? blueprint_.commit_lead_ns :
            static_cast<uint64_t>(std::max<int64_t>(0, period_.count())) * 2;
        const uint64_t committed = control_.trajectory_time_ns > UINT64_MAX - lead
            ? UINT64_MAX : control_.trajectory_time_ns + lead;
        if (replace) {
            if (!control_.trajectory_active || candidate.empty() ||
                plan.replace_after_plan_id != control_.active_plan_id ||
                plan.replace_after_time_ns < committed ||
                plan.replace_after_time_ns > candidate.back().point.time_from_start_ns)
                return RK_ERROR_INVALID_STATE;
            base_time = plan.replace_after_time_ns;
        } else if (!candidate.empty()) {
            base_time = candidate.back().point.time_from_start_ns;
        }
        double anchor_position[RK_MAX_TRAJECTORY_JOINTS]{};
        double anchor_velocity[RK_MAX_TRAJECTORY_JOINTS]{};
        double anchor_acceleration[RK_MAX_TRAJECTORY_JOINTS]{};
        bool check_anchor_velocity = candidate.empty();
        bool check_anchor_acceleration = candidate.empty();
        if (candidate.empty()) {
            // A drained path or completed stop holds its last commanded
            // setpoint. Endpoint samples are observations, not plan anchors.
            for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
                if (control_.active[joint] &&
                    control_.targets[joint].mode != RK_TARGET_POSITION)
                    return RK_ERROR_INVALID_STATE;
                const double bound = blueprint_.following_error_bound[joint];
                if (bound > 0.0 &&
                    std::abs(state_.position[joint] - commanded_position_[joint]) > bound)
                    return RK_ERROR_FOLLOWING_ERROR;
                anchor_position[joint] = commanded_position_[joint];
            }
        } else {
            const RuntimeTrajectoryPoint *anchor = &candidate.front();
            if (replace) {
                anchor = nullptr;
                for (const auto &knot : candidate) {
                    if (!knot.has_segment || knot.plan_id != plan.replace_after_plan_id)
                        continue;
                    const auto start = knot.point.time_from_start_ns;
                    const auto end = start + static_cast<uint64_t>(knot.segment.duration_ns);
                    if (start <= base_time && base_time <= end) {
                        anchor = &knot;
                        break;
                    }
                }
                if (anchor == nullptr) return RK_ERROR_INVALID_STATE;
            } else {
                for (std::size_t index = 0; index < candidate.size(); ++index) {
                    if (candidate[index].point.time_from_start_ns > base_time) break;
                    anchor = &candidate[index];
                    if (!anchor->has_segment && index > 0)
                        anchor = &candidate[index - 1];
                }
            }
            evaluate_knot(*anchor, base_time, anchor_position, anchor_velocity,
                anchor_acceleration);
            check_anchor_velocity = anchor->has_segment;
            check_anchor_acceleration = anchor->has_segment && anchor->segment.degree != 1;
            if (replace && (!anchor->has_segment || anchor->segment.degree < 2))
                return RK_ERROR_INVALID_STATE;
        }
        // The assumption is checked against the actual anchor, and the
        // submitted polynomial must start at that same state. Degree-1 paths
        // may only promise chord velocity, never stored sample derivatives.
        constexpr double default_tolerance = 1e-6;
        const bool has_tolerances = plan.struct_size >= sizeof(rk_plan_submission);
        const auto &first = plan.segments.segments[0];
        if (replace && first.degree < 2)
            return RK_ERROR_INVALID_STATE;
        for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
            const double position_tolerance = has_tolerances && plan.position_tolerance[joint] > 0.0
                ? plan.position_tolerance[joint] : default_tolerance;
            const double velocity_tolerance = has_tolerances && plan.velocity_tolerance[joint] > 0.0
                ? plan.velocity_tolerance[joint] : default_tolerance;
            const double acceleration_tolerance = has_tolerances && plan.acceleration_tolerance[joint] > 0.0
                ? plan.acceleration_tolerance[joint] : default_tolerance;
            if (std::abs(plan.start_position[joint] - anchor_position[joint]) > position_tolerance ||
                (check_anchor_velocity &&
                    std::abs(plan.start_velocity[joint] - anchor_velocity[joint]) > velocity_tolerance) ||
                (check_anchor_acceleration &&
                    std::abs(plan.start_acceleration[joint] - anchor_acceleration[joint]) > acceleration_tolerance) ||
                std::abs(first.coefficients[joint].value[0] - plan.start_position[joint]) > position_tolerance ||
                (first.degree >= 2 &&
                    (std::abs(first.coefficients[joint].value[1] - plan.start_velocity[joint]) > velocity_tolerance ||
                     std::abs(2.0 * first.coefficients[joint].value[2] - plan.start_acceleration[joint]) > acceleration_tolerance)))
                return RK_ERROR_INVALID_STATE;
        }
        if (replace) {
            while (!candidate.empty() && candidate.back().point.time_from_start_ns >= base_time)
                candidate.pop_back();
            if (!candidate.empty() && candidate.back().has_segment) {
                const auto end = candidate.back().point.time_from_start_ns +
                    static_cast<uint64_t>(candidate.back().segment.duration_ns);
                if (end > base_time)
                    candidate.back().segment.duration_ns = static_cast<int64_t>(
                        base_time - candidate.back().point.time_from_start_ns);
            }
        }
        for (uint32_t index = 0; index < plan.segments.segment_count; ++index) {
            const auto &source = plan.segments.segments[index];
            if (source.time_from_start_ns > static_cast<uint64_t>(INT64_MAX) - base_time ||
                source.duration_ns > static_cast<uint64_t>(INT64_MAX) - base_time - source.time_from_start_ns)
                return RK_ERROR_INVALID_ARGUMENT;
            RuntimeTrajectoryPoint knot{};
            knot.segment = native_segment(source, base_time);
            knot.has_segment = true;
            knot.legacy_point = false;
            knot.point.time_from_start_ns = static_cast<uint64_t>(knot.segment.t0_ns);
            knot.point.joint_count = source.joint_count;
            for (uint32_t joint = 0; joint < source.joint_count; ++joint)
                knot.point.positions[joint] = source.coefficients[joint].value[0];
            knot.chunk_base_time_ns = base_time;
            knot.tag = plan.segments.tag;
            knot.plan_id = plan.plan_id;
            candidate.push_back(std::move(knot));
        }
        RuntimeTrajectoryPoint end{};
        const auto &last = candidate.back();
        end.point.time_from_start_ns = static_cast<uint64_t>(last.segment.t0_ns + last.segment.duration_ns);
        end.point.joint_count = blueprint_.joint_count;
        evaluate_knot(last, end.point.time_from_start_ns, end.point.positions);
        end.counted_knot = false;
        end.legacy_point = false;
        end.chunk_base_time_ns = base_time;
        end.tag = plan.segments.tag;
        end.plan_id = plan.plan_id;
        candidate.push_back(std::move(end));
        if (queued_knot_count(candidate) > RK_MAX_TRAJECTORY_QUEUE_POINTS)
            return RK_ERROR_QUEUE_FULL;
        const auto checked = validate_queued_path(candidate, blueprint_);
        if (checked != RK_OK) return checked;
        const bool was_idle = !control_.trajectory_active;
        control_.trajectory = std::move(candidate);
        control_.trajectory_active = true;
        control_.plan_just_submitted = was_idle;
        if (was_idle) control_.trajectory_time_ns = 0;
        control_.active_plan_id = replace ? control_.active_plan_id : plan.plan_id;
        state_.trajectory_active = 1;
        state_.trajectory_queue_depth = queued_knot_count(control_.trajectory);
        state_.trajectory_time_ns = control_.trajectory_time_ns;
        state_.trajectory_duration_ns = control_.trajectory.back().point.time_from_start_ns;
        state_.queue_end_time_ns = state_.trajectory_duration_ns;
        state_.committed_until_ns = committed;
        state_.active_plan_id = control_.active_plan_id;
        state_.session_state = RK_SESSION_EXECUTING;
        state_.mode = RK_ROBOT_MODE_TRACKING;
        state_.safety = RK_SAFETY_READY;
        last_command_sequence_ = plan.sequence;
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

rk_result RobotRuntime::snapshot(rk_robot_state &out_state) const {
    std::lock_guard lock(state_mutex_);
    out_state = state_;
    return RK_OK;
}

rk_result RobotRuntime::snapshot_full(rk_robot_snapshot &out_snapshot) const {
    std::lock_guard lock(state_mutex_);
    out_snapshot = {};
    out_snapshot.struct_size = sizeof(out_snapshot);
    out_snapshot.revision = blueprint_.revision;
    out_snapshot.calibration_revision = blueprint_.calibration_revision;
    out_snapshot.sequence = state_.sequence;
    out_snapshot.source_timestamp_ns = state_.source_timestamp_ns;
    out_snapshot.mode = state_.mode;
    out_snapshot.safety = state_.safety;
    out_snapshot.endpoint = state_.safety == RK_SAFETY_FAULT
        ? RK_ENDPOINT_FAULT : RK_ENDPOINT_CONNECTED;
    out_snapshot.joint_count = state_.joint_count;
    for (uint32_t index = 0; index < state_.joint_count; ++index) {
        out_snapshot.position[index] = state_.position[index];
        out_snapshot.velocity[index] = state_.velocity[index];
        out_snapshot.effort[index] = state_.effort[index];
    }
    out_snapshot.fault_code = state_.safety == RK_SAFETY_FAULT ? 1 : 0;
    out_snapshot.received_timestamp_ns = state_.received_timestamp_ns;
    out_snapshot.trajectory_queue_depth = state_.trajectory_queue_depth;
    out_snapshot.trajectory_active = state_.trajectory_active;
    out_snapshot.trajectory_time_ns = state_.trajectory_time_ns;
    out_snapshot.trajectory_duration_ns = state_.trajectory_duration_ns;
    out_snapshot.trajectory_tag = state_.trajectory_tag;
    out_snapshot.trajectory_tag_time_ns = state_.trajectory_tag_time_ns;
    out_snapshot.session_state = state_.session_state;
    out_snapshot.active_plan_id = state_.active_plan_id;
    out_snapshot.committed_until_ns = state_.committed_until_ns;
    out_snapshot.queue_end_time_ns = state_.queue_end_time_ns;
    out_snapshot.sensor_count = state_.sensor_count;
    std::copy_n(state_.sensors, state_.sensor_count, out_snapshot.sensors);
    return RK_OK;
}

bool RobotRuntime::running() const {
    std::lock_guard lock(queue_mutex_);
    return running_;
}

void RobotRuntime::run() {
    auto next_tick = std::chrono::steady_clock::now();
    for (;;) {
        {
            std::lock_guard lock(queue_mutex_);
            if (stopping_)
                break;
        }
        next_tick += period_;
        step_owner(monotonic_now_ns());
        std::unique_lock lock(queue_mutex_);
        queue_condition_.wait_until(lock, next_tick, [this] { return stopping_; });
        if (stopping_)
            break;
    }
}

rk_result RobotRuntime::step_owner(uint64_t timestamp_ns) {
    const auto apply_result = apply_pending_commands();
    if (apply_result != RK_OK)
        return apply_result;
    return publish_sample(timestamp_ns);
}

void RobotRuntime::latch_fault(bool clear_control) {
    rk_robot_command emergency_stop{};
    emergency_stop.struct_size = sizeof(emergency_stop);
    emergency_stop.sequence = ++endpoint_command_sequence_;
    emergency_stop.kind = RK_COMMAND_EMERGENCY_STOP;
    endpoint_->apply(emergency_stop);
    if (clear_control)
        control_ = {};
    std::lock_guard state_lock(state_mutex_);
    if (clear_control) {
        state_.trajectory_queue_depth = 0;
        state_.trajectory_active = 0;
        state_.trajectory_time_ns = 0;
        state_.trajectory_duration_ns = 0;
        state_.trajectory_tag = 0;
        state_.trajectory_tag_time_ns = 0;
        state_.active_plan_id = 0;
        state_.committed_until_ns = 0;
        state_.queue_end_time_ns = 0;
    }
    state_.mode = RK_ROBOT_MODE_FAULT;
    state_.safety = RK_SAFETY_FAULT;
    state_.session_state = RK_SESSION_FAULTED;
    state_backup_valid_ = false;
}

rk_result RobotRuntime::apply_pending_commands() {
    std::lock_guard owner_lock(owner_mutex_);
    std::deque<QueuedCommand> commands;
    {
        std::lock_guard queue_lock(queue_mutex_);
        commands.swap(commands_);
    }
    {
        std::lock_guard state_lock(state_mutex_);
        state_backup_ = state_;
        state_backup_valid_ = true;
    }
    control_backup_ = control_;
    std::copy_n(commanded_position_, RK_MAX_JOINTS, commanded_position_backup_);

    bool trajectory_stop_completed = false;
    // Set when a path-following stop reaches the end of the queued path this
    // cycle while still moving, so it can hand over to a straight ramp with
    // the same speed instead of stopping dead.
    bool stop_crossed_queue_end = false;
    double stop_crossing_rate = 0.0;
    double stop_after_crossing_seconds = 0.0;
    double stop_path_velocities[RK_MAX_TRAJECTORY_JOINTS]{};
    if (control_.trajectory_active && !control_.trajectory.empty()) {
        const auto period_count = period_.count();
        const auto period_ns = period_count > 0 ? static_cast<uint64_t>(period_count) : 0;
        const double period_seconds = static_cast<double>(period_ns) / 1'000'000'000.0;
        double time_ns = static_cast<double>(control_.trajectory_time_ns) +
            control_.trajectory_time_remainder_ns;
        if (control_.plan_just_submitted) {
            control_.plan_just_submitted = false;
        } else if (!control_.stop_ramp_active) {
            time_ns += static_cast<double>(period_ns);
        } else {
            // Path-following stop. A joint moves at rate * v and accelerates
            // at rate' * v + rate^2 * a, where v and a belong to the queued
            // trajectory itself. Lower the rate as fast as every joint's limit
            // allows, so braking already present in the trajectory is counted
            // rather than stacked on top of the stop. The rate is integrated
            // in substeps because at low speed one period can change it a lot.
            constexpr int substeps = 16;
            const double step_seconds = period_seconds / substeps;
            const double end_ns =
                static_cast<double>(control_.trajectory.back().point.time_from_start_ns);
            const auto region = path_region(control_.trajectory,
                control_.trajectory_history_valid ? &control_.trajectory_history : nullptr);
            double rate = control_.trajectory_rate;
            for (int step = 0; step < substeps && rate > 0.0; ++step) {
                mk_path_derivative_estimate estimate{};
                motionkit::estimate_path_derivatives(region, static_cast<int64_t>(time_ns),
                    period_ns, estimate);
                const auto *velocities = estimate.velocity;
                if (estimate.has_forward_acceleration)
                    std::copy_n(estimate.acceleration, blueprint_.joint_count,
                        control_.stop_path_accelerations);
                const auto *accelerations = control_.stop_path_accelerations;
                const auto *recent = estimate.recent_acceleration;
                double max_decrease = std::numeric_limits<double>::infinity();
                for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
                    const double speed = std::abs(velocities[joint]);
                    if (speed <= 1e-12)
                        continue;
                    const double direction = velocities[joint] > 0.0 ? 1.0 : -1.0;
                    // With rate' = -k, the path's own braking consumes part
                    // of the joint acceleration budget. MotionKit gives
                    // analytic derivatives for degree >= 2 and conservative
                    // chord differences at degree 1, including the recent
                    // knot behind the clock. Budgeting rate * |a| rather than
                    // the continuous rate^2 * |a| covers a whole velocity
                    // step when the clock is slowed and is conservative for
                    // smooth segments too. Path speed-ups are not credited.
                    const double braking = std::max(0.0, std::max(
                        -direction * accelerations[joint], -direction * recent[joint]));
                    max_decrease = std::min(max_decrease,
                        (blueprint_.joints[joint].max_acceleration - rate * braking) / speed);
                }
                // No joint moving: the trajectory is at rest and can stop now.
                // A non-positive bound means the trajectory already brakes at
                // the limit, so it keeps its rate and brakes by itself.
                const double next_rate = !std::isfinite(max_decrease) ? 0.0 :
                    std::clamp(rate - std::max(0.0, max_decrease) * step_seconds, 0.0, rate);
                const double step_ns = 0.5 * (rate + next_rate) * step_seconds * 1'000'000'000.0;
                if (time_ns + step_ns >= end_ns && next_rate > 0.0) {
                    const double fraction = step_ns > 0.0
                        ? std::clamp((end_ns - time_ns) / step_ns, 0.0, 1.0) : 1.0;
                    stop_crossed_queue_end = true;
                    stop_crossing_rate = rate + (next_rate - rate) * fraction;
                    stop_after_crossing_seconds =
                        ((1.0 - fraction) + (substeps - step - 1)) * step_seconds;
                    const auto &last = region.back();
                    if (last.degree >= 2) {
                        mk_trajectory_state endpoint{};
                        motionkit::evaluate_segment(last,
                            static_cast<double>(last.duration_ns) * 1e-9, endpoint);
                        std::copy_n(endpoint.velocity, blueprint_.joint_count,
                            stop_path_velocities);
                    } else {
                        // A chord velocity belongs at its midpoint. Continue
                        // its finite-difference braking estimate to the end.
                        const double half_span_seconds =
                            static_cast<double>(last.duration_ns) * 0.5e-9;
                        for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
                            const double extrapolated = velocities[joint] +
                                accelerations[joint] * half_span_seconds;
                            stop_path_velocities[joint] =
                                extrapolated * velocities[joint] > 0.0 ? extrapolated : 0.0;
                        }
                    }
                    time_ns = end_ns;
                    rate = next_rate;
                    break;
                }
                time_ns += step_ns;
                rate = next_rate;
            }
            control_.trajectory_rate = rate;
            trajectory_stop_completed = rate <= 0.0 && !stop_crossed_queue_end;
        }
        const double max_time = static_cast<double>(std::numeric_limits<uint64_t>::max());
        if (time_ns >= max_time) {
            control_.trajectory_time_ns = std::numeric_limits<uint64_t>::max();
            control_.trajectory_time_remainder_ns = 0.0;
        } else {
            const auto whole = static_cast<uint64_t>(time_ns);
            control_.trajectory_time_ns = std::max(control_.trajectory_time_ns, whole);
            control_.trajectory_time_remainder_ns = time_ns - static_cast<double>(whole);
        }
    }
    if (control_.stop_ramp_active) {
        const auto period_count = period_.count();
        const auto period_ns = period_count > 0 ? static_cast<uint64_t>(period_count) : 0;
        if (std::numeric_limits<uint64_t>::max() - control_.stop_ramp_time_ns < period_ns)
            control_.stop_ramp_time_ns = control_.stop_ramp_duration_ns;
        else
            control_.stop_ramp_time_ns = std::min(control_.stop_ramp_duration_ns,
                control_.stop_ramp_time_ns + period_ns);
    }

    const bool has_command = !commands.empty();
    rk_command_kind final_kind = RK_COMMAND_NONE;
    uint64_t final_timestamp_ns = 0;
    rk_safety_state safety = RK_SAFETY_READY;
    rk_robot_state current{};
    {
        std::lock_guard state_lock(state_mutex_);
        current = state_;
        safety = state_.safety;
    }
    // Emergency stop remains the one intentionally non-sequential operation:
    // it wins over every other command in the drained owner cycle. All other
    // commands are then applied in mailbox order so a reset/flush/chunk
    // sequence and multiple trajectory chunks retain their meaning.
    const rk_robot_command *emergency = nullptr;
    for (const auto &queued : commands) {
        const auto &value = queued.command;
        if (value.kind == RK_COMMAND_EMERGENCY_STOP) {
            emergency = &value;
            break;
        }
    }

    std::size_t last_reset_index = commands.size();
    for (std::size_t index = 0; index < commands.size(); ++index)
        if (commands[index].command.kind == RK_COMMAND_RESET_SAFETY)
            last_reset_index = index;

    bool controlled_stop = false;
    auto refresh_trajectory_progress = [&]() {
        if (control_.trajectory.empty()) return;
        const auto *selected = &control_.trajectory.front();
        for (const auto &candidate : control_.trajectory) {
            if (candidate.point.time_from_start_ns > control_.trajectory_time_ns)
                break;
            selected = &candidate;
        }
        // The selected point identifies the chunk; the reported time is the
        // trajectory clock itself, clamped to the queued end.
        const auto selected_time = std::min(control_.trajectory_time_ns,
            control_.trajectory.back().point.time_from_start_ns);
        control_.trajectory_tag = selected->tag;
        control_.active_plan_id = selected->plan_id;
        control_.trajectory_tag_time_ns = selected_time >= selected->chunk_base_time_ns
            ? selected_time - selected->chunk_base_time_ns : 0;
    };
    // Straight-line ramp from the given pose and velocity to rest, used when
    // the stop cannot follow the queued path. It lasts at least two periods,
    // and long enough for every joint with a limit to stay within it. A ramp
    // travels half its duration at the starting speed, so when a joint's
    // travel limit is closer than that, the ramp is shortened to stop at the
    // limit instead of running into it; if that means braking harder than a
    // joint allows, the fault is latched once the ramp reaches rest.
    auto start_stop_ramp = [&](const double *positions, const double *velocities) {
        const auto period_count = period_.count();
        const auto period_ns = period_count > 0 ? static_cast<uint64_t>(period_count) : 1;
        double duration_seconds = 2.0 * static_cast<double>(period_ns) / 1'000'000'000.0;
        double limited_duration = 0.0;
        for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
            const auto acceleration = blueprint_.joints[joint].max_acceleration;
            if (std::isfinite(acceleration) && acceleration > 0.0)
                limited_duration = std::max(limited_duration,
                    std::abs(velocities[joint]) / acceleration);
        }
        duration_seconds = std::max(duration_seconds, limited_duration);
        for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
            const double speed = std::abs(velocities[joint]);
            if (speed <= 1e-12)
                continue;
            const auto &limits = blueprint_.joints[joint];
            const double room = std::max(0.0, velocities[joint] > 0.0
                ? limits.upper_limit - positions[joint] : positions[joint] - limits.lower_limit);
            duration_seconds = std::min(duration_seconds, 2.0 * room / speed);
        }
        control_.stop_ramp_exceeds_limits = duration_seconds < limited_duration * (1.0 - 1e-9);
        control_.stop_ramp_duration_ns =
            static_cast<uint64_t>(std::ceil(duration_seconds * 1'000'000'000.0));
        control_.stop_ramp_time_ns = 0;
        std::copy_n(positions, blueprint_.joint_count, control_.stop_ramp_positions);
        std::copy_n(velocities, blueprint_.joint_count, control_.stop_ramp_velocities);
        control_.trajectory.clear();
        control_.trajectory_history_valid = false;
        control_.trajectory_time_ns = 0;
        control_.trajectory_active = false;
        control_.trajectory_rate = 1.0;
        control_.trajectory_time_remainder_ns = 0.0;
        control_.stop_ramp_active = true;
    };
    auto begin_controlled_stop = [&]() {
        // A stop that is already running keeps its progress; restarting it
        // would jump back to full trajectory speed.
        if (control_.stop_ramp_active)
            return true;
        if (!control_.trajectory_active || control_.trajectory.empty()) {
            control_ = {};
            return false;
        }
        refresh_trajectory_progress();
        bool has_acceleration_limits = true;
        for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
            const auto acceleration = blueprint_.joints[joint].max_acceleration;
            if (!std::isfinite(acceleration) || acceleration <= 0.0) {
                has_acceleration_limits = false;
                break;
            }
        }
        if (has_acceleration_limits) {
            control_.trajectory_rate = 1.0;
            control_.trajectory_time_remainder_ns = 0.0;
            std::fill_n(control_.stop_path_accelerations, RK_MAX_TRAJECTORY_JOINTS, 0.0);
            control_.stop_ramp_active = true;
        } else {
            double positions[RK_MAX_TRAJECTORY_JOINTS]{};
            double velocities[RK_MAX_TRAJECTORY_JOINTS]{};
            const RuntimeTrajectoryPoint *active = &control_.trajectory.front();
            for (const auto &knot : control_.trajectory) {
                if (knot.point.time_from_start_ns > control_.trajectory_time_ns) break;
                active = &knot;
            }
            evaluate_knot(*active, control_.trajectory_time_ns, positions);
            const auto region = path_region(control_.trajectory,
                control_.trajectory_history_valid ? &control_.trajectory_history : nullptr);
            mk_path_derivative_estimate estimate{};
            if (motionkit::estimate_path_derivatives(region,
                    static_cast<int64_t>(control_.trajectory_time_ns), 0, estimate))
                std::copy_n(estimate.velocity, blueprint_.joint_count, velocities);
            start_stop_ramp(positions, velocities);
        }
        return true;
    };

    auto apply_intermediate_lifecycle = [&](rk_robot_command &value) {
        // The caller sequence belongs to the runtime mailbox. Lifecycle
        // commands sent through an endpoint also need the endpoint-local
        // monotonic sequence, otherwise a few owner cycles with no host
        // command can make an intermediate stop/reset look stale to a device.
        value.sequence = ++endpoint_command_sequence_;
        const auto result = endpoint_->apply(value);
        if (result != RK_OK)
            latch_fault();
        return result;
    };

    if (emergency != nullptr) {
        control_ = {};
        final_kind = RK_COMMAND_EMERGENCY_STOP;
        final_timestamp_ns = emergency->timestamp_ns;
        safety = RK_SAFETY_EMERGENCY_STOP;
    } else {
        if (!commands.empty() &&
            (safety == RK_SAFETY_EMERGENCY_STOP || safety == RK_SAFETY_FAULT) &&
            last_reset_index == commands.size()) {
            std::lock_guard state_lock(state_mutex_);
            state_backup_valid_ = false;
            return RK_ERROR_SAFETY_STOPPED;
        }
        for (std::size_t index = 0; index < commands.size(); ++index) {
            auto &queued = commands[index];
            auto &value = queued.command;
            bool has_later_effective_command = false;
            for (std::size_t next = index + 1; next < commands.size(); ++next) {
                if (commands[next].command.kind != RK_COMMAND_NONE) {
                    has_later_effective_command = true;
                    break;
                }
            }
            final_timestamp_ns = value.timestamp_ns;

            if ((safety == RK_SAFETY_EMERGENCY_STOP || safety == RK_SAFETY_FAULT) &&
                value.kind != RK_COMMAND_RESET_SAFETY) {
                if (index < last_reset_index)
                    continue;
                std::lock_guard state_lock(state_mutex_);
                state_backup_valid_ = false;
                return RK_ERROR_SAFETY_STOPPED;
            }

            if (value.kind == RK_COMMAND_NONE)
                continue;
            final_kind = value.kind;

            if (value.kind == RK_COMMAND_RESET_SAFETY) {
                control_ = {};
                controlled_stop = false;
                safety = RK_SAFETY_READY;
                if (has_later_effective_command) {
                    const auto result = apply_intermediate_lifecycle(value);
                    if (result != RK_OK)
                        return result;
                }
                continue;
            }

            if (value.kind == RK_COMMAND_STOP) {
                controlled_stop = begin_controlled_stop();
                safety = RK_SAFETY_READY;
                if (has_later_effective_command) {
                    const auto result = apply_intermediate_lifecycle(value);
                    if (result != RK_OK)
                        return result;
                }
                continue;
            }

            if (value.kind == RK_COMMAND_JOINT_TARGETS) {
                // A target batch is a replacement when trajectory execution
                // is active, but remains a partial update during ordinary
                // target control. This preserves independent drive/arm
                // updates without appending a new move behind old motion.
                const bool replace_active_motion = control_.trajectory_active ||
                    !control_.trajectory.empty() || control_.stop_ramp_active;
                if (replace_active_motion) {
                    control_.trajectory.clear();
                    control_.trajectory_history_valid = false;
                    control_.trajectory_time_ns = 0;
                    control_.trajectory_active = false;
                    control_.stop_ramp_active = false;
                    control_.trajectory_rate = 1.0;
                    control_.trajectory_time_remainder_ns = 0.0;
                    std::fill_n(control_.active, RK_MAX_JOINTS, false);
                    std::fill_n(control_.reference_initialized, RK_MAX_JOINTS, false);
                }
                for (uint32_t target_index = 0; target_index < value.target_count; ++target_index) {
                    const auto &target = value.targets[target_index];
                    const auto &joint = blueprint_.joints[target.joint];
                    if (target.mode == RK_TARGET_POSITION &&
                        (target.target < joint.lower_limit || target.target > joint.upper_limit)) {
                        latch_fault();
                        return RK_ERROR_LIMIT;
                    }
                    if (target.mode == RK_TARGET_EFFORT && joint.max_effort > 0.0 &&
                        std::abs(target.target) > joint.max_effort) {
                        latch_fault();
                        return RK_ERROR_LIMIT;
                    }
                }
                for (uint32_t target_index = 0; target_index < value.target_count; ++target_index) {
                    const auto &target = value.targets[target_index];
                    const auto joint = target.joint;
                    if (target.mode == RK_TARGET_POSITION &&
                        (!control_.active[joint] || control_.targets[joint].mode != RK_TARGET_POSITION)) {
                        control_.position_reference[joint] = current.position[joint];
                        control_.reference_initialized[joint] = true;
                    }
                    control_.targets[joint] = target;
                    if (target.mode == RK_TARGET_EFFORT && target.max_effort > 0.0 &&
                        std::abs(control_.targets[joint].target) > target.max_effort)
                        control_.targets[joint].target = std::copysign(
                            target.max_effort, control_.targets[joint].target);
                    control_.active[joint] = true;
                }
                controlled_stop = false;
                continue;
            }

            if (value.kind == RK_COMMAND_TRAJECTORY_CHUNK ||
                value.kind == RK_COMMAND_TRAJECTORY_SEGMENTS) {
                const bool sampled = value.kind == RK_COMMAND_TRAJECTORY_CHUNK;
                if (sampled ? queued.trajectory == nullptr : queued.segments == nullptr)
                    return RK_ERROR_INVALID_ARGUMENT;
                if (!supports_trajectory_queue())
                    return RK_ERROR_UNSUPPORTED;
                const uint64_t tag = sampled ? queued.trajectory->tag : queued.segments->tag;
                const uint64_t splice_tag = sampled ? queued.trajectory->splice_tag
                    : queued.segments->splice_tag;
                const uint64_t splice_offset = sampled ? queued.trajectory->splice_time_ns
                    : queued.segments->splice_time_ns;
                const bool splice = splice_tag != 0;
                auto candidate = control_.trajectory;
                uint64_t splice_time = 0;
                if (splice) {
                    const RuntimeTrajectoryPoint *anchor = nullptr;
                    for (const auto &knot : control_.trajectory)
                        if (knot.tag == splice_tag) {
                            anchor = &knot;
                            break;
                        }
                    if (anchor == nullptr || !control_.trajectory_active || control_.stop_ramp_active ||
                        splice_offset > std::numeric_limits<uint64_t>::max() - anchor->chunk_base_time_ns)
                        continue;
                    splice_time = anchor->chunk_base_time_ns + splice_offset;
                    if (splice_time <= control_.trajectory_time_ns ||
                        splice_time > control_.trajectory.back().point.time_from_start_ns)
                        continue;
                    while (!candidate.empty() &&
                           candidate.back().point.time_from_start_ns >= splice_time)
                        candidate.pop_back();
                    if (!candidate.empty() && candidate.back().has_segment) {
                        const auto &last = candidate.back();
                        const auto end = last.point.time_from_start_ns +
                            static_cast<uint64_t>(last.segment.duration_ns);
                        if (end > splice_time)
                            candidate.back().segment.duration_ns = static_cast<int64_t>(
                                splice_time - last.point.time_from_start_ns);
                    }
                }
                const auto base_time = splice ? splice_time : candidate.empty()
                    ? uint64_t{0} : candidate.back().point.time_from_start_ns;
                if (sampled) {
                    for (uint32_t index = 0; index < queued.trajectory->point_count; ++index) {
                        auto point = queued.trajectory->points[index];
                        if (point.time_from_start_ns >
                            static_cast<uint64_t>(INT64_MAX) - base_time)
                            return RK_ERROR_INVALID_ARGUMENT;
                        point.time_from_start_ns += base_time;
                        if (!candidate.empty() && !candidate.back().has_segment &&
                            candidate.back().point.time_from_start_ns < point.time_from_start_ns) {
                            candidate.back().segment = chord_segment(candidate.back().point, point);
                            candidate.back().has_segment = true;
                        }
                        RuntimeTrajectoryPoint knot{};
                        knot.point = point;
                        knot.chunk_base_time_ns = base_time;
                        knot.tag = tag;
                        candidate.push_back(std::move(knot));
                    }
                } else {
                    for (uint32_t index = 0; index < queued.segments->segment_count; ++index) {
                        const auto &source = queued.segments->segments[index];
                        if (source.time_from_start_ns > static_cast<uint64_t>(INT64_MAX) - base_time ||
                            source.duration_ns > static_cast<uint64_t>(INT64_MAX) - base_time -
                                source.time_from_start_ns)
                            return RK_ERROR_INVALID_ARGUMENT;
                        RuntimeTrajectoryPoint knot{};
                        knot.segment = native_segment(source, base_time);
                        knot.has_segment = true;
                        knot.legacy_point = false;
                        knot.point.time_from_start_ns = static_cast<uint64_t>(knot.segment.t0_ns);
                        knot.point.joint_count = source.joint_count;
                        for (uint32_t joint = 0; joint < source.joint_count; ++joint)
                            knot.point.positions[joint] = source.coefficients[joint].value[0];
                        knot.chunk_base_time_ns = base_time;
                        knot.tag = tag;
                        candidate.push_back(std::move(knot));
                    }
                    RuntimeTrajectoryPoint end{};
                    const auto &last = candidate.back();
                    end.point.time_from_start_ns = static_cast<uint64_t>(
                        last.segment.t0_ns + last.segment.duration_ns);
                    end.point.joint_count = blueprint_.joint_count;
                    evaluate_knot(last, end.point.time_from_start_ns, end.point.positions);
                    end.counted_knot = false;
                    end.legacy_point = false;
                    end.chunk_base_time_ns = base_time;
                    end.tag = tag;
                    candidate.push_back(std::move(end));
                }
                if (queued_knot_count(candidate) > RK_MAX_TRAJECTORY_QUEUE_POINTS)
                    return RK_ERROR_QUEUE_FULL;
                const auto checked = validate_queued_path(candidate, blueprint_);
                if (checked == RK_ERROR_LIMIT) {
                    latch_fault();
                    return RK_ERROR_LIMIT;
                }
                if (checked != RK_OK) return checked;
                control_.trajectory = std::move(candidate);
                // While a path-following stop runs, a chunk only extends the
                // path the stop may use; the stop keeps its current rate and
                // still ends at rest. Resuming means waiting for the stop to
                // finish, or flushing with a target batch first.
                const bool stopping_along_path =
                    control_.stop_ramp_active && control_.trajectory_active;
                if (!stopping_along_path) {
                    std::fill_n(control_.active, RK_MAX_JOINTS, false);
                    std::fill_n(control_.reference_initialized, RK_MAX_JOINTS, false);
                    control_.stop_ramp_active = false;
                    control_.trajectory_rate = 1.0;
                    control_.trajectory_time_remainder_ns = 0.0;
                    controlled_stop = false;
                }
                control_.trajectory_active = true;
            }
        }
    }

    const bool lifecycle_command = has_command && lifecycle_kind(final_kind);

    // Emits the straight stop ramp's position at its current time and ends
    // the ramp once it reaches rest.
    bool stop_ramp_finished = false;
    auto write_stop_ramp_targets = [&](rk_robot_command &value) {
        value.kind = RK_COMMAND_JOINT_TARGETS;
        value.target_count = blueprint_.joint_count;
        const double duration = static_cast<double>(control_.stop_ramp_duration_ns) /
            1'000'000'000.0;
        const double elapsed = static_cast<double>(control_.stop_ramp_time_ns) /
            1'000'000'000.0;
        const double t = std::min(duration, elapsed);
        const double blend = duration <= 0.0 ? 0.0 : t - 0.5 * t * t / duration;
        for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
            value.targets[joint].joint = joint;
            value.targets[joint].mode = RK_TARGET_POSITION;
            const auto &limits = blueprint_.joints[joint];
            value.targets[joint].target = std::clamp(
                control_.stop_ramp_positions[joint] +
                    control_.stop_ramp_velocities[joint] * blend,
                limits.lower_limit, limits.upper_limit);
            value.targets[joint].max_rate = 0.0;
            value.targets[joint].max_effort = 0.0;
        }
        if (control_.stop_ramp_time_ns >= control_.stop_ramp_duration_ns) {
            control_.stop_ramp_active = false;
            stop_ramp_finished = true;
        }
    };

    rk_robot_command output{};
    output.struct_size = sizeof(output);
    output.timestamp_ns = has_command ? final_timestamp_ns : 0;
    if (lifecycle_command && !controlled_stop) {
        output.kind = final_kind;
        output.target_count = 0;
    } else if (!control_.trajectory.empty()) {
        auto point = control_.trajectory.front().point;
        while (control_.trajectory.size() > 1 &&
               control_.trajectory[1].point.time_from_start_ns <= control_.trajectory_time_ns) {
            control_.trajectory_history = control_.trajectory.front();
            control_.trajectory_history_valid = true;
            control_.trajectory.pop_front();
        }
        point = control_.trajectory.front().point;
        evaluate_knot(control_.trajectory.front(), control_.trajectory_time_ns,
            point.positions);
        output.kind = RK_COMMAND_JOINT_TARGETS;
        output.target_count = point.joint_count;
        for (uint32_t joint = 0; joint < point.joint_count; ++joint) {
            output.targets[joint].joint = joint;
            output.targets[joint].mode = RK_TARGET_POSITION;
            output.targets[joint].target = point.positions[joint];
            output.targets[joint].max_rate = 0.0;
            output.targets[joint].max_effort = 0.0;
        }
        const bool queue_exhausted =
            control_.trajectory_time_ns >= control_.trajectory.back().point.time_from_start_ns;
        if (queue_exhausted || trajectory_stop_completed) {
            // Report the chunk and time where motion actually ended; a stop
            // can finish inside an earlier chunk than the last one queued.
            refresh_trajectory_progress();
            const bool stop_still_moving = queue_exhausted && !trajectory_stop_completed &&
                control_.stop_ramp_active;
            if (stop_still_moving) {
                // The stop ran out of queued path before reaching rest. Finish
                // with a straight ramp, still within the acceleration limits,
                // instead of stopping dead. The ramp starts where the clock
                // crossed the queue end, at the speed reached there, and is
                // credited with the rest of the cycle so speed carries over.
                double velocities[RK_MAX_JOINTS]{};
                const double crossing_rate = stop_crossed_queue_end
                    ? stop_crossing_rate : control_.trajectory_rate;
                for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint)
                    velocities[joint] = crossing_rate * stop_path_velocities[joint];
                start_stop_ramp(point.positions, velocities);
                control_.stop_ramp_time_ns = std::min(control_.stop_ramp_duration_ns,
                    static_cast<uint64_t>(stop_after_crossing_seconds * 1'000'000'000.0));
                write_stop_ramp_targets(output);
            } else {
                control_.trajectory.clear();
                control_.trajectory_history_valid = false;
                control_.trajectory_time_ns = 0;
                control_.trajectory_active = false;
                control_.stop_ramp_active = false;
                control_.trajectory_rate = 1.0;
                control_.trajectory_time_remainder_ns = 0.0;
            }
        }
    } else if (control_.stop_ramp_active) {
        write_stop_ramp_targets(output);
    } else {
        output.kind = RK_COMMAND_JOINT_TARGETS;
        const auto period_seconds = std::chrono::duration<double>(period_).count();
        uint32_t active_count = 0;
        rk_robot_state current{};
        {
            std::lock_guard state_lock(state_mutex_);
            current = state_;
        }
        for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
            if (!control_.active[joint])
                continue;
            auto target = control_.targets[joint];
            if (target.mode == RK_TARGET_VELOCITY && target.max_rate > 0.0)
                target.target = std::clamp(target.target, -target.max_rate, target.max_rate);
            if (target.mode == RK_TARGET_POSITION) {
                if (!control_.reference_initialized[joint]) {
                    control_.position_reference[joint] = current.position[joint];
                    control_.reference_initialized[joint] = true;
                }
                const double delta = target.target - control_.position_reference[joint];
                if (target.max_rate > 0.0 && std::isfinite(period_seconds)) {
                    const double maximum_delta = target.max_rate * period_seconds;
                    control_.position_reference[joint] += std::clamp(
                        delta, -maximum_delta, maximum_delta);
                } else {
                    control_.position_reference[joint] = target.target;
                }
                target.target = control_.position_reference[joint];
            }
            output.targets[active_count++] = target;
        }
        output.target_count = active_count;
        if (active_count == 0 && has_command && final_kind == RK_COMMAND_NONE)
            output.kind = RK_COMMAND_NONE;
        else if (active_count == 0) {
            state_backup_valid_ = false;
            return RK_OK;
        }
    }

    output.sequence = ++endpoint_command_sequence_;
    const auto result = endpoint_->apply(output);
    if (result != RK_OK) {
        latch_fault();
        return result;
    }
    if (output.kind == RK_COMMAND_JOINT_TARGETS)
        for (uint32_t index = 0; index < output.target_count; ++index)
            if (output.targets[index].mode == RK_TARGET_POSITION)
                commanded_position_[output.targets[index].joint] = output.targets[index].target;
    if (stop_ramp_finished && control_.stop_ramp_exceeds_limits) {
        // The ramp stopped at a travel limit only by braking harder than a
        // joint's acceleration limit allows; now at rest, report it.
        latch_fault();
        return RK_ERROR_LIMIT;
    }
    refresh_trajectory_progress();
    std::lock_guard state_lock(state_mutex_);
    state_.trajectory_queue_depth = queued_knot_count(control_.trajectory);
    state_.trajectory_active = control_.trajectory_active ? 1u : 0u;
    state_.trajectory_time_ns = control_.trajectory_active ? control_.trajectory_time_ns : 0;
    state_.trajectory_duration_ns = control_.trajectory_active && !control_.trajectory.empty()
        ? control_.trajectory.back().point.time_from_start_ns : 0;
    state_.trajectory_tag = control_.trajectory_tag;
    state_.trajectory_tag_time_ns = control_.trajectory_tag_time_ns;
    state_.queue_end_time_ns = state_.trajectory_duration_ns;
    state_.active_plan_id = control_.trajectory_active ? control_.active_plan_id : 0;
    const uint64_t lead = blueprint_.commit_lead_ns != 0 ? blueprint_.commit_lead_ns :
        static_cast<uint64_t>(std::max<int64_t>(0, period_.count())) * 2;
    state_.committed_until_ns = control_.trajectory_active
        ? (control_.trajectory_time_ns > UINT64_MAX - lead ? UINT64_MAX :
            control_.trajectory_time_ns + lead) : 0;
    if (lifecycle_command && final_kind == RK_COMMAND_EMERGENCY_STOP) {
        state_.mode = RK_ROBOT_MODE_FAULT;
        state_.safety = RK_SAFETY_EMERGENCY_STOP;
    } else if (lifecycle_command && final_kind == RK_COMMAND_STOP) {
        state_.mode = RK_ROBOT_MODE_STOPPING;
        state_.safety = RK_SAFETY_STOPPING;
    } else if (lifecycle_command && final_kind == RK_COMMAND_RESET_SAFETY) {
        state_.mode = RK_ROBOT_MODE_IDLE;
        state_.safety = RK_SAFETY_READY;
    } else if (controlled_stop || control_.stop_ramp_active) {
        state_.mode = RK_ROBOT_MODE_STOPPING;
        state_.safety = RK_SAFETY_STOPPING;
    } else if (has_command && (final_kind == RK_COMMAND_JOINT_TARGETS ||
                               final_kind == RK_COMMAND_TRAJECTORY_CHUNK ||
                               final_kind == RK_COMMAND_TRAJECTORY_SEGMENTS)) {
        state_.mode = RK_ROBOT_MODE_TRACKING;
        state_.safety = RK_SAFETY_READY;
    } else if (state_.safety == RK_SAFETY_STOPPING && !stop_ramp_finished &&
               !control_.trajectory_active) {
        state_.mode = RK_ROBOT_MODE_IDLE;
        state_.safety = RK_SAFETY_READY;
    }
    state_.session_state = state_.safety == RK_SAFETY_FAULT ||
        state_.safety == RK_SAFETY_EMERGENCY_STOP ? RK_SESSION_FAULTED :
        state_.safety == RK_SAFETY_STOPPING || control_.stop_ramp_active ? RK_SESSION_STOPPING :
        control_.trajectory_active ? RK_SESSION_EXECUTING : RK_SESSION_IDLE;

    return RK_OK;
}

rk_result RobotRuntime::publish_sample(uint64_t timestamp_ns) {
    std::lock_guard owner_lock(owner_mutex_);
    rk_robot_state next;
    {
        std::lock_guard state_lock(state_mutex_);
        next = state_;
    }
    const auto runtime_mode = next.mode;
    const auto runtime_safety = next.safety;
    const auto trajectory_queue_depth = next.trajectory_queue_depth;
    const auto trajectory_active = next.trajectory_active;
    const auto trajectory_time_ns = next.trajectory_time_ns;
    const auto trajectory_duration_ns = next.trajectory_duration_ns;
    const auto trajectory_tag = next.trajectory_tag;
    const auto trajectory_tag_time_ns = next.trajectory_tag_time_ns;
    const auto session_state = next.session_state;
    const auto active_plan_id = next.active_plan_id;
    const auto committed_until_ns = next.committed_until_ns;
    const auto queue_end_time_ns = next.queue_end_time_ns;
    next.source_timestamp_ns = 0;
    next.received_timestamp_ns = 0;
    next.sensor_count = 0;
    const auto result = endpoint_->sample(timestamp_ns, next);
    next.mode = runtime_mode;
    next.trajectory_queue_depth = trajectory_queue_depth;
    next.trajectory_active = trajectory_active;
    next.trajectory_time_ns = trajectory_time_ns;
    next.trajectory_duration_ns = trajectory_duration_ns;
    next.trajectory_tag = trajectory_tag;
    next.trajectory_tag_time_ns = trajectory_tag_time_ns;
    next.session_state = session_state;
    next.active_plan_id = active_plan_id;
    next.committed_until_ns = committed_until_ns;
    next.queue_end_time_ns = queue_end_time_ns;
    if (!endpoint_->reports_safety_state())
        next.safety = runtime_safety;
    else if (next.safety == RK_SAFETY_EMERGENCY_STOP || next.safety == RK_SAFETY_FAULT)
        next.mode = RK_ROBOT_MODE_FAULT;
    if (next.safety == RK_SAFETY_EMERGENCY_STOP || next.safety == RK_SAFETY_FAULT)
        next.session_state = RK_SESSION_FAULTED;
    const auto sample_validation = result == RK_OK
        ? rk_robot_state_validate(&next) : RK_OK;
    const auto configured_sensor_count = blueprint_.sensor_count == 0 ? 3 : blueprint_.sensor_count;
    if (result != RK_OK || sample_validation != RK_OK ||
        next.joint_count != blueprint_.joint_count ||
        next.sensor_count > configured_sensor_count ||
        (next.sensor_count != 0 && next.sensor_count != configured_sensor_count)) {
        latch_fault();
        return result != RK_OK ? result : RK_ERROR_BACKEND;
    }
    for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
        const auto &limits = blueprint_.joints[joint];
        if (next.position[joint] < limits.lower_limit ||
            next.position[joint] > limits.upper_limit) {
            latch_fault();
            return RK_ERROR_LIMIT;
        }
    }
    {
        std::lock_guard state_lock(state_mutex_);
        next.sequence = state_.sequence + 1;
        // Source epoch zero is valid. Receipt is always the local monotonic
        // acceptance clock, never the caller's simulation/source tick.
        next.received_timestamp_ns = monotonic_now_ns();
        for (uint32_t i = 0; i < next.sensor_count; ++i) {
            auto &sample = next.sensors[i];
            if (!sample.sequence) continue;
            const auto &old = state_.sensors[i];
            sample.received_timestamp_ns = i < state_.sensor_count && old.sequence == sample.sequence
                && old.source_timestamp_ns == sample.source_timestamp_ns
                ? old.received_timestamp_ns : next.received_timestamp_ns;
        }
        next.struct_size = sizeof(next);
        state_ = next;
        state_backup_valid_ = false;
    }
    return RK_OK;
}

void RobotRuntime::set_externally_driven(bool value) noexcept {
    std::lock_guard lock(queue_mutex_);
    externally_driven_ = value;
}

void RobotRuntime::discard_pending_commands() noexcept {
    std::lock_guard owner_lock(owner_mutex_);
    endpoint_->discard_pending();
    std::lock_guard state_lock(state_mutex_);
    if (state_backup_valid_) {
        state_ = state_backup_;
        control_ = control_backup_;
        std::copy_n(commanded_position_backup_, RK_MAX_JOINTS, commanded_position_);
        state_backup_valid_ = false;
    }
}

void RobotRuntime::reset_state() noexcept {
    std::lock_guard owner_lock(owner_mutex_);
    {
        std::lock_guard queue_lock(queue_mutex_);
        commands_.clear();
        last_command_sequence_ = 0;
    }
    std::lock_guard state_lock(state_mutex_);
    state_ = {};
    state_.struct_size = sizeof(state_);
    state_.joint_count = blueprint_.joint_count;
    state_.safety = endpoint_ ? endpoint_->initial_safety_state() : RK_SAFETY_READY;
    state_.mode = state_.safety == RK_SAFETY_EMERGENCY_STOP ||
        state_.safety == RK_SAFETY_FAULT ? RK_ROBOT_MODE_FAULT : RK_ROBOT_MODE_IDLE;
    control_ = {};
    std::fill_n(commanded_position_, RK_MAX_JOINTS, 0.0);
    std::fill_n(commanded_position_backup_, RK_MAX_JOINTS, 0.0);
    control_backup_ = {};
    state_backup_valid_ = false;
}

} // namespace robotkit
