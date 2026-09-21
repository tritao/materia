#include "robotkit_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace robotkit {

namespace {

uint64_t monotonic_now_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

bool newer_command(const rk_robot_command &candidate, const rk_robot_command &current) {
    return candidate.sequence > current.sequence;
}

/**
 * Reduce one mailbox drain to one owner-cycle intent. Emergency stop always
 * wins; otherwise the newest command wins. This prevents a stale target earlier
 * in the same cycle from running after a stop, while preserving explicit
 * command ordering for a controller that deliberately submits a later target.
 */
rk_robot_command arbitrate(const std::deque<rk_robot_command> &commands) {
    rk_robot_command selected{};
    bool has_selected = false;
    bool has_emergency = false;
    for (const auto &command : commands) {
        if (command.kind == RK_COMMAND_EMERGENCY_STOP) {
            if (!has_emergency || newer_command(command, selected)) {
                selected = command;
                has_emergency = true;
            }
            continue;
        }
        if (!has_emergency && (!has_selected || newer_command(command, selected))) {
            selected = command;
            has_selected = true;
        }
    }
    return selected;
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
        if (target.joint >= joint_count_ || target.mode != RK_TARGET_POSITION)
            return RK_ERROR_UNSUPPORTED;
        targets_[target.joint] = target.target;
        has_target_[target.joint] = true;
    }
    return RK_OK;
}

rk_result InMemoryRobot::sample(uint64_t timestamp_ns, rk_robot_state &state) {
    state.struct_size = sizeof(state);
    state.source_timestamp_ns = timestamp_ns;
    state.received_timestamp_ns = timestamp_ns;
    state.joint_count = joint_count_;
    for (uint32_t index = 0; index < joint_count_; ++index) {
        const double old_position = state.position[index];
        if (!stopped_ && has_target_[index])
            state.position[index] += (targets_[index] - state.position[index]) * 0.25;
        state.velocity[index] = state.position[index] - old_position;
        state.effort[index] = 0.0;
    }
    return RK_OK;
}

RobotRuntime::RobotRuntime(const rk_robot_runtime_blueprint &blueprint,
                           std::shared_ptr<RobotEndpoint> endpoint,
                 std::chrono::nanoseconds period)
    : blueprint_(blueprint), endpoint_(std::move(endpoint)), period_(period) {
    state_.struct_size = sizeof(state_);
    state_.joint_count = blueprint_.joint_count;
    state_.mode = RK_ROBOT_MODE_IDLE;
    state_.safety = RK_SAFETY_READY;
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

rk_result RobotRuntime::apply_pending_commands() {
    std::deque<rk_robot_command> commands;
    {
        std::lock_guard queue_lock(queue_mutex_);
        commands.swap(commands_);
    }
    if (commands.empty())
        return RK_OK;

    auto command = arbitrate(commands);
    {
        std::lock_guard state_lock(state_mutex_);
        state_backup_ = state_;
        state_backup_valid_ = true;
    }

    rk_safety_state safety = RK_SAFETY_READY;
    {
        std::lock_guard state_lock(state_mutex_);
        safety = state_.safety;
    }
    if ((safety == RK_SAFETY_EMERGENCY_STOP || safety == RK_SAFETY_FAULT) &&
        command.kind != RK_COMMAND_RESET_SAFETY &&
        command.kind != RK_COMMAND_EMERGENCY_STOP) {
        std::lock_guard state_lock(state_mutex_);
        state_backup_valid_ = false;
        return RK_ERROR_SAFETY_STOPPED;
    }

    if (command.kind == RK_COMMAND_JOINT_TARGETS) {
        rk_robot_state current{};
        {
            std::lock_guard state_lock(state_mutex_);
            current = state_;
        }
        const auto period_seconds = std::chrono::duration<double>(period_).count();
        for (uint32_t index = 0; index < command.target_count; ++index) {
            auto &target = command.targets[index];
            const auto &joint = blueprint_.joints[target.joint];
            if (target.mode == RK_TARGET_POSITION &&
                (target.target < joint.lower_limit || target.target > joint.upper_limit)) {
                std::lock_guard state_lock(state_mutex_);
                state_.mode = RK_ROBOT_MODE_FAULT;
                state_.safety = RK_SAFETY_FAULT;
                state_backup_valid_ = false;
                return RK_ERROR_LIMIT;
            }
            if (target.mode == RK_TARGET_EFFORT && joint.max_effort > 0.0 &&
                std::abs(target.target) > joint.max_effort) {
                std::lock_guard state_lock(state_mutex_);
                state_.mode = RK_ROBOT_MODE_FAULT;
                state_.safety = RK_SAFETY_FAULT;
                state_backup_valid_ = false;
                return RK_ERROR_LIMIT;
            }
            if (target.mode == RK_TARGET_POSITION && target.max_rate > 0.0 &&
                std::isfinite(period_seconds)) {
                const auto maximum_delta = target.max_rate * period_seconds;
                const auto minimum = current.position[target.joint] - maximum_delta;
                const auto maximum = current.position[target.joint] + maximum_delta;
                target.target = std::clamp(target.target, minimum, maximum);
            }
            if (target.mode == RK_TARGET_EFFORT && target.max_effort > 0.0 &&
                std::abs(target.target) > target.max_effort)
                target.target = std::copysign(target.max_effort, target.target);
        }
    }

    const auto result = endpoint_->apply(command);
    if (result != RK_OK) {
        std::lock_guard state_lock(state_mutex_);
        state_.mode = RK_ROBOT_MODE_FAULT;
        state_.safety = RK_SAFETY_FAULT;
        state_backup_valid_ = false;
        return result;
    }
    std::lock_guard state_lock(state_mutex_);
    if (command.kind == RK_COMMAND_EMERGENCY_STOP) {
        state_.mode = RK_ROBOT_MODE_FAULT;
        state_.safety = RK_SAFETY_EMERGENCY_STOP;
    } else if (command.kind == RK_COMMAND_STOP) {
        state_.mode = RK_ROBOT_MODE_STOPPING;
        state_.safety = RK_SAFETY_STOPPING;
    } else if (command.kind == RK_COMMAND_RESET_SAFETY) {
        state_.mode = RK_ROBOT_MODE_IDLE;
        state_.safety = RK_SAFETY_READY;
    } else if (command.kind == RK_COMMAND_JOINT_TARGETS) {
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
    const auto result = endpoint_->sample(timestamp_ns, next);
    if (result != RK_OK) {
        std::lock_guard state_lock(state_mutex_);
        state_.mode = RK_ROBOT_MODE_FAULT;
        state_.safety = RK_SAFETY_FAULT;
        state_backup_valid_ = false;
        return result;
    }
    {
        std::lock_guard state_lock(state_mutex_);
        next.sequence = state_.sequence + 1;
        if (next.source_timestamp_ns == 0)
            next.source_timestamp_ns = timestamp_ns;
        next.received_timestamp_ns = timestamp_ns;
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
    state_.mode = RK_ROBOT_MODE_IDLE;
    state_.safety = RK_SAFETY_READY;
    state_backup_valid_ = false;
}

} // namespace robotkit
