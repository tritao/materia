#include "simulation_robot.hpp"

#include "simulation.hpp"
#include "sensor_math.hpp"
#include <algorithm>
#include <cmath>

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
    if (command.kind == RK_COMMAND_RESET_SAFETY) {
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
    // All measurements use the same immutable physics snapshot as encoders.
    uint64_t body_count = 0;
    if (nksim_snapshot_get_body_count(simulation_.snapshot_, &body_count) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    std::vector<nksim_body_state> bodies(body_count);
    const nksim_body_state *base = nullptr;
    for (uint64_t index = 0; index < body_count; ++index) {
        bodies[index].struct_size = sizeof(nksim_body_state);
        if (nksim_snapshot_get_body(simulation_.snapshot_, index, &bodies[index]) != NKSIM_OK)
            return RK_ERROR_BACKEND;
        if (bodies[index].body == base_body_) base = &bodies[index];
    }
    if (!base) return RK_ERROR_BACKEND;
    const double now = simulation_.simulation_time_;
    // A first sample primes the derivative. Do not fabricate acceleration.
    state.sensor_flags = 2;
    if (previous_time_ >= 0.0 && now > previous_time_) {
        double acceleration[3];
        for (int i = 0; i < 3; ++i)
            acceleration[i] = (base->linear_velocity[i] - previous_velocity_[i]) / (now - previous_time_);
        sensors::imu(base->rotation, base->angular_velocity, acceleration,
                     simulation_.gravity_, state.imu);
        state.sensor_flags |= 1;
    }
    previous_time_ = now;
    std::copy_n(base->linear_velocity, 3, previous_velocity_);
    for (int ray = 0; ray < 8; ++ray) {
        const double angle = ray * 0.7853981633974483;
        const double local[3] = {std::cos(angle), std::sin(angle), 0.0};
        double direction[3];
        sensors::rotate(base->rotation, local, direction);
        double range = 10.0;
        for (const auto &body : bodies) {
            // Exclude every link of this robot, but observe other robots.
            if (std::find(bodies_.begin(), bodies_.end(), body.body) != bodies_.end()) continue;
            const double robot_extents[3] = {0.05, 0.05, 0.05};
            const double *extents = robot_extents;
            for (const auto &object : simulation_.objects_)
                if (object.active && object.body == body.body) { extents = object.half_extents; break; }
            range = sensors::ray_box(base->position, direction, body.position,
                                     body.rotation, extents, range);
        }
        state.lidar[ray] = range;
    }
    return RK_OK;
}

} // namespace robotkit
