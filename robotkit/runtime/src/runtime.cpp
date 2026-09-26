#include "robotkit_runtime.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace robotkit {

namespace {

uint64_t monotonic_now_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

bool lifecycle_kind(rk_command_kind kind) {
    return kind != RK_COMMAND_NONE && kind != RK_COMMAND_JOINT_TARGETS;
}

/**
 * Reduce one mailbox drain to one owner-cycle intent. Submission accepts only
 * increasing sequences, so the drain is already in sequence order. Emergency
 * stop always wins. A newest stop or safety reset wins alone, so a stale
 * target earlier in the same cycle never runs after it. Otherwise every
 * joint-target batch newer than the latest stop or reset merges per joint, a
 * newer batch replacing an older one's target for the same joint: batches are
 * partial updates, so a caller may command a drive and an arm with separate
 * batches in one cycle without the later batch discarding the earlier one's
 * joints.
 */
rk_robot_command arbitrate(const std::deque<rk_robot_command> &commands) {
    const rk_robot_command *emergency = nullptr;
    for (const auto &command : commands)
        if (command.kind == RK_COMMAND_EMERGENCY_STOP)
            emergency = &command;
    if (emergency)
        return *emergency;
    const auto &newest = commands.back();
    if (lifecycle_kind(newest.kind))
        return newest;

    std::size_t first = 0;
    for (std::size_t index = 0; index < commands.size(); ++index)
        if (lifecycle_kind(commands[index].kind))
            first = index + 1;
    rk_robot_command merged = newest;
    merged.kind = RK_COMMAND_NONE;
    merged.target_count = 0;
    std::array<int, RK_MAX_JOINTS> slot;
    slot.fill(-1);
    for (std::size_t index = first; index < commands.size(); ++index) {
        const auto &batch = commands[index];
        if (batch.kind != RK_COMMAND_JOINT_TARGETS)
            continue;
        merged.kind = RK_COMMAND_JOINT_TARGETS;
        for (uint32_t target = 0; target < batch.target_count; ++target) {
            const auto &value = batch.targets[target];
            // Submission validation bounds joints by the blueprint, itself
            // within RK_MAX_JOINTS.
            auto &position = slot[value.joint];
            if (position < 0)
                position = static_cast<int>(merged.target_count++);
            merged.targets[position] = value;
        }
    }
    return merged;
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
    if (rk_robot_command_validate_for_blueprint(&command, &blueprint_) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    std::lock_guard lock(queue_mutex_);
    if (command.sequence == 0 || command.sequence <= last_command_sequence_)
        return RK_ERROR_STALE_COMMAND;
    if (commands_.size() >= 128)
        return RK_ERROR_QUEUE_FULL;
    last_command_sequence_ = command.sequence;
    commands_.push_back(command);
    queue_condition_.notify_all();
    return RK_OK;
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

void RobotRuntime::latch_fault() {
    rk_robot_command emergency_stop{};
    emergency_stop.struct_size = sizeof(emergency_stop);
    emergency_stop.sequence = ++endpoint_command_sequence_;
    emergency_stop.kind = RK_COMMAND_EMERGENCY_STOP;
    endpoint_->apply(emergency_stop);
    control_ = {};
    std::lock_guard state_lock(state_mutex_);
    state_.mode = RK_ROBOT_MODE_FAULT;
    state_.safety = RK_SAFETY_FAULT;
    state_backup_valid_ = false;
}

rk_result RobotRuntime::apply_pending_commands() {
    std::deque<rk_robot_command> commands;
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

    const bool has_command = !commands.empty();
    rk_robot_command command{};
    if (has_command)
        command = arbitrate(commands);

    rk_safety_state safety = RK_SAFETY_READY;
    {
        std::lock_guard state_lock(state_mutex_);
        safety = state_.safety;
    }
    if (has_command &&
        (safety == RK_SAFETY_EMERGENCY_STOP || safety == RK_SAFETY_FAULT) &&
        command.kind != RK_COMMAND_RESET_SAFETY &&
        command.kind != RK_COMMAND_EMERGENCY_STOP) {
        std::lock_guard state_lock(state_mutex_);
        state_backup_valid_ = false;
        return RK_ERROR_SAFETY_STOPPED;
    }

    if (has_command && command.kind == RK_COMMAND_JOINT_TARGETS) {
        rk_robot_state current{};
        {
            std::lock_guard state_lock(state_mutex_);
            current = state_;
        }
        for (uint32_t index = 0; index < command.target_count; ++index) {
            const auto &target = command.targets[index];
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
        for (uint32_t index = 0; index < command.target_count; ++index) {
            const auto &target = command.targets[index];
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
    }

    const bool lifecycle_command = has_command && command.kind != RK_COMMAND_NONE &&
        command.kind != RK_COMMAND_JOINT_TARGETS;
    if (lifecycle_command) {
        control_ = {};
    }

    rk_robot_command output{};
    output.struct_size = sizeof(output);
    output.timestamp_ns = has_command ? command.timestamp_ns : 0;
    if (lifecycle_command) {
        output.kind = command.kind;
        output.target_count = 0;
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
        if (active_count == 0 && has_command && command.kind == RK_COMMAND_NONE)
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
    std::lock_guard state_lock(state_mutex_);
    if (lifecycle_command && command.kind == RK_COMMAND_EMERGENCY_STOP) {
        state_.mode = RK_ROBOT_MODE_FAULT;
        state_.safety = RK_SAFETY_EMERGENCY_STOP;
    } else if (lifecycle_command && command.kind == RK_COMMAND_STOP) {
        state_.mode = RK_ROBOT_MODE_STOPPING;
        state_.safety = RK_SAFETY_STOPPING;
    } else if (lifecycle_command && command.kind == RK_COMMAND_RESET_SAFETY) {
        state_.mode = RK_ROBOT_MODE_IDLE;
        state_.safety = RK_SAFETY_READY;
    } else if (has_command && command.kind == RK_COMMAND_JOINT_TARGETS) {
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
    next.source_timestamp_ns = 0;
    next.received_timestamp_ns = 0;
    next.sensor_count = 0;
    const auto result = endpoint_->sample(timestamp_ns, next);
    next.mode = runtime_mode;
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
