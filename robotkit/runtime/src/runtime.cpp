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

} // namespace

InMemoryEndpoint::InMemoryEndpoint(uint32_t joint_count)
    : joint_count_(std::min(joint_count, static_cast<uint32_t>(RK_MAX_JOINTS))) {}

rk_result InMemoryEndpoint::apply(const rk_robot_command &command) {
    if (stopped_ && command.kind != RK_COMMAND_STOP)
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

rk_result InMemoryEndpoint::step(uint64_t timestamp_ns, rk_robot_state &state) {
    state.struct_size = sizeof(state);
    state.timestamp_ns = timestamp_ns;
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

Runtime::Runtime(const rk_runtime_layout &layout, std::unique_ptr<Endpoint> endpoint,
                 std::chrono::nanoseconds period)
    : layout_(layout), endpoint_(std::move(endpoint)), period_(period) {
    state_.struct_size = sizeof(state_);
    state_.joint_count = layout_.joint_count;
    state_.mode = RK_ROBOT_MODE_IDLE;
    state_.safety = RK_SAFETY_READY;
}

Runtime::~Runtime() {
    stop();
}

rk_result Runtime::start() {
    std::lock_guard lock(queue_mutex_);
    if (running_ || stopping_ || endpoint_ == nullptr ||
        rk_runtime_layout_validate(&layout_) != RK_OK)
        return RK_ERROR_INVALID_STATE;
    running_ = true;
    worker_ = std::thread([this] { run(); });
    return RK_OK;
}

rk_result Runtime::stop() {
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

rk_result Runtime::submit(const rk_robot_command &command) {
    if (rk_robot_command_validate_for_layout(&command, &layout_) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    std::lock_guard lock(queue_mutex_);
    if (commands_.size() >= 128)
        return RK_ERROR_QUEUE_FULL;
    commands_.push_back(command);
    queue_condition_.notify_all();
    return RK_OK;
}

rk_result Runtime::step_once(uint64_t timestamp_ns) {
    {
        std::lock_guard lock(queue_mutex_);
        if (running_ || stopping_ || endpoint_ == nullptr ||
            rk_runtime_layout_validate(&layout_) != RK_OK)
            return RK_ERROR_INVALID_STATE;
    }
    return step_owner(timestamp_ns);
}

rk_result Runtime::snapshot(rk_robot_state &out_state) const {
    std::lock_guard lock(state_mutex_);
    out_state = state_;
    return RK_OK;
}

rk_result Runtime::snapshot_full(rk_robot_snapshot &out_snapshot) const {
    std::lock_guard lock(state_mutex_);
    out_snapshot = {};
    out_snapshot.struct_size = sizeof(out_snapshot);
    out_snapshot.revision = layout_.revision;
    out_snapshot.sequence = state_.sequence;
    out_snapshot.timestamp_ns = state_.timestamp_ns;
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
    return RK_OK;
}

bool Runtime::running() const {
    std::lock_guard lock(queue_mutex_);
    return running_;
}

void Runtime::run() {
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

rk_result Runtime::step_owner(uint64_t timestamp_ns) {
    std::deque<rk_robot_command> commands;
    {
        std::lock_guard queue_lock(queue_mutex_);
        commands.swap(commands_);
    }
    for (const auto &command : commands) {
        const auto result = endpoint_->apply(command);
        if (result != RK_OK) {
            std::lock_guard state_lock(state_mutex_);
            state_.mode = RK_ROBOT_MODE_FAULT;
            state_.safety = RK_SAFETY_FAULT;
            return result;
        }
        std::lock_guard state_lock(state_mutex_);
        if (command.kind == RK_COMMAND_EMERGENCY_STOP) {
            state_.mode = RK_ROBOT_MODE_FAULT;
            state_.safety = RK_SAFETY_EMERGENCY_STOP;
        } else if (command.kind == RK_COMMAND_STOP) {
            state_.mode = RK_ROBOT_MODE_STOPPING;
            state_.safety = RK_SAFETY_STOPPING;
        } else if (command.kind == RK_COMMAND_JOINT_TARGETS) {
            state_.mode = RK_ROBOT_MODE_TRACKING;
            state_.safety = RK_SAFETY_READY;
        }
    }

    rk_robot_state next;
    {
        std::lock_guard state_lock(state_mutex_);
        next = state_;
    }
    const auto result = endpoint_->step(timestamp_ns, next);
    if (result != RK_OK)
        return result;
    {
        std::lock_guard state_lock(state_mutex_);
        next.sequence = state_.sequence + 1;
        next.timestamp_ns = timestamp_ns;
        next.struct_size = sizeof(next);
        state_ = next;
    }
    return RK_OK;
}

} // namespace robotkit
