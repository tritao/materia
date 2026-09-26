#include "robotkit_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
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
        kind != RK_COMMAND_TRAJECTORY_CHUNK;
}

void sample_trajectory(const std::deque<RobotRuntime::RuntimeTrajectoryPoint> &trajectory,
                       uint64_t time_ns, double *positions, double *velocities) {
    const auto &first = trajectory.front().point;
    const auto &last = trajectory.back().point;
    const auto joint_count = first.joint_count;
    std::fill_n(positions, joint_count, 0.0);
    std::fill_n(velocities, joint_count, 0.0);
    if (time_ns <= first.time_from_start_ns) {
        for (uint32_t joint = 0; joint < joint_count; ++joint)
            positions[joint] = first.positions[joint];
        return;
    }
    if (time_ns >= last.time_from_start_ns) {
        for (uint32_t joint = 0; joint < joint_count; ++joint)
            positions[joint] = last.positions[joint];
        return;
    }
    auto before = trajectory.begin();
    auto after = std::next(before);
    while (after != trajectory.end() && after->point.time_from_start_ns < time_ns) {
        ++before;
        ++after;
    }
    const auto span = after->point.time_from_start_ns - before->point.time_from_start_ns;
    const auto elapsed = time_ns - before->point.time_from_start_ns;
    const double alpha = span == 0 ? 0.0 :
        static_cast<double>(elapsed) / static_cast<double>(span);
    const double seconds = span == 0 ? 0.0 :
        static_cast<double>(span) / 1'000'000'000.0;
    for (uint32_t joint = 0; joint < joint_count; ++joint) {
        positions[joint] = before->point.positions[joint] +
            (after->point.positions[joint] - before->point.positions[joint]) * alpha;
        velocities[joint] = seconds <= 0.0 ? 0.0 :
            (after->point.positions[joint] - before->point.positions[joint]) / seconds;
    }
}

/**
 * Chord velocity of the queued segment that is being entered at time_ns.
 * Unlike sample_trajectory, a time exactly on a sample uses the following
 * segment, so a trajectory whose clock lands on its front sample still
 * reports its motion. The queue end reports zero velocity. midpoint_ns, when
 * given, receives the segment's middle time, where a chord velocity is exact
 * for constant acceleration.
 */
void forward_velocity(const std::deque<RobotRuntime::RuntimeTrajectoryPoint> &trajectory,
                      uint64_t time_ns, double *velocities, double *midpoint_ns = nullptr) {
    const auto joint_count = trajectory.front().point.joint_count;
    std::fill_n(velocities, joint_count, 0.0);
    if (midpoint_ns)
        *midpoint_ns = static_cast<double>(time_ns);
    for (std::size_t index = 0; index + 1 < trajectory.size(); ++index) {
        const auto &before = trajectory[index].point;
        const auto &after = trajectory[index + 1].point;
        if (after.time_from_start_ns <= time_ns ||
            after.time_from_start_ns == before.time_from_start_ns)
            continue;
        const double seconds = static_cast<double>(
            after.time_from_start_ns - before.time_from_start_ns) / 1'000'000'000.0;
        for (uint32_t joint = 0; joint < joint_count; ++joint)
            velocities[joint] = (after.positions[joint] - before.positions[joint]) / seconds;
        if (midpoint_ns)
            *midpoint_ns = 0.5 * (static_cast<double>(before.time_from_start_ns) +
                static_cast<double>(after.time_from_start_ns));
        return;
    }
}

/**
 * Estimates the queued trajectory's own velocity and acceleration at time_ns.
 * Acceleration is the difference between this segment's chord velocity and
 * the one window_ns ahead, over the time between their midpoints, so uneven
 * sample spacing (such as a short final segment) is handled. Returns false
 * when there is no later segment to compare with, in which case
 * accelerations is left untouched so the caller can keep its last estimate:
 * samples behind the clock have already been dropped from the queue.
 */
bool trajectory_motion(const std::deque<RobotRuntime::RuntimeTrajectoryPoint> &trajectory,
                       uint64_t time_ns, uint64_t window_ns, double *velocities,
                       double *accelerations) {
    const auto joint_count = trajectory.front().point.joint_count;
    double midpoint_ns = 0.0;
    forward_velocity(trajectory, time_ns, velocities, &midpoint_ns);
    const auto last_ns = trajectory.back().point.time_from_start_ns;
    if (window_ns == 0 || time_ns >= last_ns)
        return false;
    double ahead[RK_MAX_TRAJECTORY_JOINTS]{};
    double ahead_midpoint_ns = 0.0;
    forward_velocity(trajectory, std::min(time_ns + window_ns, last_ns - 1), ahead,
        &ahead_midpoint_ns);
    const double spacing_seconds = (ahead_midpoint_ns - midpoint_ns) / 1'000'000'000.0;
    if (spacing_seconds <= 0.0)
        return false;
    for (uint32_t joint = 0; joint < joint_count; ++joint)
        accelerations[joint] = (ahead[joint] - velocities[joint]) / spacing_seconds;
    return true;
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
    if (command.kind == RK_COMMAND_TRAJECTORY_CHUNK)
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
    }
    state_.mode = RK_ROBOT_MODE_FAULT;
    state_.safety = RK_SAFETY_FAULT;
    state_backup_valid_ = false;
}

rk_result RobotRuntime::apply_pending_commands() {
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
        if (!control_.stop_ramp_active) {
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
            double rate = control_.trajectory_rate;
            for (int step = 0; step < substeps && rate > 0.0; ++step) {
                double velocities[RK_MAX_TRAJECTORY_JOINTS]{};
                const auto *accelerations = control_.stop_path_accelerations;
                trajectory_motion(control_.trajectory, static_cast<uint64_t>(time_ns), period_ns,
                    velocities, control_.stop_path_accelerations);
                double max_decrease = std::numeric_limits<double>::infinity();
                for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
                    const double speed = std::abs(velocities[joint]);
                    if (speed <= 1e-12)
                        continue;
                    const double direction = velocities[joint] > 0.0 ? 1.0 : -1.0;
                    // With rate' = -k, keeping the joint's acceleration within
                    // its limit against its direction of travel gives this
                    // bound. The queued path is piecewise linear, so the
                    // trajectory's own speed changes arrive as steps at its
                    // samples rather than smoothly. A speed-up cannot be relied
                    // on to offset the stop within any one period, so it is not
                    // credited. A slow-down is budgeted at rate * |a| rather
                    // than the continuous rate^2 * |a|, which covers a period
                    // that catches one whole step while the clock runs slowed.
                    const double braking = std::max(0.0, -direction * accelerations[joint]);
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
                    // A chord velocity belongs to the middle of its segment;
                    // extrapolate to the queue end so a path that ends at
                    // rest hands over zero speed rather than overshooting.
                    double half_span_seconds = 0.0;
                    if (control_.trajectory.size() >= 2)
                        half_span_seconds = 0.5 * static_cast<double>(
                            control_.trajectory.back().point.time_from_start_ns -
                            control_.trajectory[control_.trajectory.size() - 2]
                                .point.time_from_start_ns) / 1'000'000'000.0;
                    for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
                        const double extrapolated =
                            velocities[joint] + accelerations[joint] * half_span_seconds;
                        stop_path_velocities[joint] =
                            extrapolated * velocities[joint] > 0.0 ? extrapolated : 0.0;
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
        control_.trajectory_tag_time_ns = selected_time >= selected->chunk_base_time_ns
            ? selected_time - selected->chunk_base_time_ns : 0;
    };
    // Straight-line ramp from the given pose and velocity to rest, used when
    // the stop cannot follow the queued path. It lasts at least two periods,
    // and long enough for every joint with a limit to stay within it.
    auto start_stop_ramp = [&](const double *positions, const double *velocities) {
        const auto period_count = period_.count();
        const auto period_ns = period_count > 0 ? static_cast<uint64_t>(period_count) : 1;
        double duration_seconds = 2.0 * static_cast<double>(period_ns) / 1'000'000'000.0;
        for (uint32_t joint = 0; joint < blueprint_.joint_count; ++joint) {
            const auto acceleration = blueprint_.joints[joint].max_acceleration;
            if (std::isfinite(acceleration) && acceleration > 0.0)
                duration_seconds = std::max(duration_seconds,
                    std::abs(velocities[joint]) / acceleration);
        }
        control_.stop_ramp_duration_ns =
            static_cast<uint64_t>(std::ceil(duration_seconds * 1'000'000'000.0));
        control_.stop_ramp_time_ns = 0;
        std::copy_n(positions, blueprint_.joint_count, control_.stop_ramp_positions);
        std::copy_n(velocities, blueprint_.joint_count, control_.stop_ramp_velocities);
        control_.trajectory.clear();
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
            double unused[RK_MAX_TRAJECTORY_JOINTS]{};
            double velocities[RK_MAX_TRAJECTORY_JOINTS]{};
            sample_trajectory(control_.trajectory, control_.trajectory_time_ns, positions, unused);
            forward_velocity(control_.trajectory, control_.trajectory_time_ns, velocities);
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

            if (value.kind == RK_COMMAND_TRAJECTORY_CHUNK) {
                if (queued.trajectory == nullptr)
                    return RK_ERROR_INVALID_ARGUMENT;
                const auto base_time = control_.trajectory.empty()
                    ? uint64_t{0} : control_.trajectory.back().point.time_from_start_ns;
                for (uint32_t point_index = 0;
                     point_index < queued.trajectory->point_count; ++point_index) {
                    const auto &point = queued.trajectory->points[point_index];
                    if (base_time > std::numeric_limits<uint64_t>::max() - point.time_from_start_ns)
                        return RK_ERROR_INVALID_ARGUMENT;
                    for (uint32_t joint = 0; joint < point.joint_count; ++joint) {
                        const auto &limits = blueprint_.joints[joint];
                        if (point.positions[joint] < limits.lower_limit ||
                            point.positions[joint] > limits.upper_limit) {
                            // A rejected chunk must not partially append, but
                            // the limit fault must still stop any trajectory
                            // that was already queued. Keeping that queue
                            // executable after latching the fault lets the
                            // next owner cycle drive the endpoint again.
                            latch_fault();
                            return RK_ERROR_LIMIT;
                        }
                    }
                }
                // The speed implied between consecutive points, including the
                // step from the end of the queued path into this chunk, must
                // stay within each joint's velocity limit: a jump between
                // points at the same time is an unbounded speed.
                const rk_trajectory_point *previous = control_.trajectory.empty()
                    ? nullptr : &control_.trajectory.back().point;
                uint64_t previous_time = base_time;
                for (uint32_t point_index = 0;
                     point_index < queued.trajectory->point_count; ++point_index) {
                    const auto &point = queued.trajectory->points[point_index];
                    const auto time = base_time + point.time_from_start_ns;
                    if (previous != nullptr) {
                        const double seconds =
                            static_cast<double>(time - previous_time) / 1'000'000'000.0;
                        for (uint32_t joint = 0; joint < point.joint_count; ++joint) {
                            const double limit = blueprint_.joints[joint].max_velocity;
                            if (limit <= 0.0)
                                continue;
                            const double distance =
                                std::abs(point.positions[joint] - previous->positions[joint]);
                            if (distance > limit * seconds * (1.0 + 1e-6) + 1e-9) {
                                latch_fault();
                                return RK_ERROR_LIMIT;
                            }
                        }
                    }
                    previous = &point;
                    previous_time = time;
                }
                // Submission already bounds the queue; this guards the owner.
                if (control_.trajectory.size() + queued.trajectory->point_count >
                    RK_MAX_TRAJECTORY_QUEUE_POINTS)
                    return RK_ERROR_QUEUE_FULL;
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
                const auto chunk_base_time = base_time;
                for (uint32_t point_index = 0;
                     point_index < queued.trajectory->point_count; ++point_index) {
                    auto point = queued.trajectory->points[point_index];
                    point.time_from_start_ns += base_time;
                    control_.trajectory.push_back({point, chunk_base_time, queued.trajectory->tag});
                }
                control_.trajectory_active = true;
            }
        }
    }

    const bool lifecycle_command = has_command && lifecycle_kind(final_kind);

    // Emits the straight stop ramp's position at its current time and ends
    // the ramp once it reaches rest.
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
        if (control_.stop_ramp_time_ns >= control_.stop_ramp_duration_ns)
            control_.stop_ramp_active = false;
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
               control_.trajectory[1].point.time_from_start_ns <= control_.trajectory_time_ns)
            control_.trajectory.pop_front();
        point = control_.trajectory.front().point;
        if (control_.trajectory.size() > 1 &&
            control_.trajectory_time_ns > point.time_from_start_ns) {
            const auto &after = control_.trajectory[1].point;
            const auto span = after.time_from_start_ns - point.time_from_start_ns;
            const auto elapsed = control_.trajectory_time_ns - point.time_from_start_ns;
            const double alpha = span == 0 ? 0.0 :
                static_cast<double>(elapsed) / static_cast<double>(span);
            for (uint32_t joint = 0; joint < point.joint_count; ++joint)
                point.positions[joint] +=
                    (after.positions[joint] - point.positions[joint]) * alpha;
        }
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
    refresh_trajectory_progress();
    std::lock_guard state_lock(state_mutex_);
    state_.trajectory_queue_depth = static_cast<uint32_t>(control_.trajectory.size());
    state_.trajectory_active = control_.trajectory_active ? 1u : 0u;
    state_.trajectory_time_ns = control_.trajectory_active ? control_.trajectory_time_ns : 0;
    state_.trajectory_duration_ns = control_.trajectory_active && !control_.trajectory.empty()
        ? control_.trajectory.back().point.time_from_start_ns : 0;
    state_.trajectory_tag = control_.trajectory_tag;
    state_.trajectory_tag_time_ns = control_.trajectory_tag_time_ns;
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
                               final_kind == RK_COMMAND_TRAJECTORY_CHUNK)) {
        state_.mode = RK_ROBOT_MODE_TRACKING;
        state_.safety = RK_SAFETY_READY;
    }

    return RK_OK;
}

rk_result RobotRuntime::publish_sample(uint64_t timestamp_ns) {
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
    if (!endpoint_->reports_safety_state())
        next.safety = runtime_safety;
    else if (next.safety == RK_SAFETY_EMERGENCY_STOP || next.safety == RK_SAFETY_FAULT)
        next.mode = RK_ROBOT_MODE_FAULT;
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
    endpoint_->discard_pending();
    std::lock_guard state_lock(state_mutex_);
    if (state_backup_valid_) {
        state_ = state_backup_;
        control_ = control_backup_;
        state_backup_valid_ = false;
    }
}

void RobotRuntime::reset_state() noexcept {
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
    control_backup_ = {};
    state_backup_valid_ = false;
}

} // namespace robotkit
