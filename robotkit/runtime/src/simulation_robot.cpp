#include "simulation_robot.hpp"

#include "simulation.hpp"

namespace robotkit {

rk_result SimulationRobot::apply(const rk_robot_command &command) {
    if (command.kind == RK_COMMAND_EMERGENCY_STOP) {
        stopped_ = true;
        pending_targets_.clear();
        return RK_OK;
    }
    if (command.kind == RK_COMMAND_STOP) {
        stopped_ = false;
        pending_targets_.clear();
        return RK_OK;
    }
    if (stopped_)
        return RK_ERROR_SAFETY_STOPPED;
    if (command.kind == RK_COMMAND_NONE)
        return RK_OK;
    if (command.kind != RK_COMMAND_JOINT_TARGETS)
        return RK_ERROR_UNSUPPORTED;
    for (uint32_t index = 0; index < command.target_count; ++index) {
        const auto &source = command.targets[index];
        if (source.joint >= joints_.size())
            return RK_ERROR_INVALID_ARGUMENT;
        nksim_joint_target target{};
        target.struct_size = sizeof(target);
        target.joint = joints_[source.joint];
        target.mode = source.mode;
        target.target = source.target;
        target.max_force = source.max_effort;
        pending_targets_.push_back(target);
    }
    return RK_OK;
}

std::vector<nksim_joint_target> SimulationRobot::take_pending_targets() {
    auto result = std::move(pending_targets_);
    pending_targets_.clear();
    return result;
}

rk_result SimulationRobot::sample(uint64_t timestamp_ns, rk_robot_state &state) {
    if (simulation_.snapshot_ == 0)
        return RK_ERROR_INVALID_STATE;
    state.struct_size = sizeof(state);
    state.source_timestamp_ns = static_cast<uint64_t>(
        simulation_.simulation_time_ * 1'000'000'000.0);
    state.joint_count = static_cast<uint32_t>(joints_.size());
    for (uint32_t index = 0; index < state.joint_count; ++index)
        state.position[index] = state.velocity[index] = state.effort[index] = 0.0;
    uint64_t count = 0;
    if (nksim_snapshot_get_joint_count(simulation_.snapshot_, &count) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    for (uint64_t index = 0; index < count; ++index) {
        nksim_joint_state source{};
        source.struct_size = sizeof(source);
        if (nksim_snapshot_get_joint(simulation_.snapshot_, index, &source) != NKSIM_OK)
            return RK_ERROR_BACKEND;
        for (uint32_t target = 0; target < state.joint_count; ++target) {
            if (source.joint == joints_[target]) {
                state.position[target] = source.position;
                state.velocity[target] = source.velocity;
                state.effort[target] = source.effort;
                break;
            }
        }
    }
    return RK_OK;
}

} // namespace robotkit
