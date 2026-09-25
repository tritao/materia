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
        for (std::size_t index = 0; index < joints_.size(); ++index) {
            if (index >= actuated_joints_.size() || !actuated_joints_[index])
                continue;
            nksim_joint_target target{};
            target.struct_size = sizeof(target);
            target.joint = joints_[index];
            target.mode = NKSIM_JOINT_TARGET_VELOCITY;
            target.target = 0.0;
            pending_targets_.push_back(target);
        }
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
    for (uint64_t index = 0; index < body_count; ++index) {
        bodies[index].struct_size = sizeof(nksim_body_state);
        if (nksim_snapshot_get_body(simulation_.snapshot_, index, &bodies[index]) != NKSIM_OK)
            return RK_ERROR_BACKEND;
    }
    const double now = simulation_.simulation_time_;
    state.sensor_count = static_cast<uint32_t>(sensors_.size());
    for (uint32_t slot = 0; slot < sensors_.size(); ++slot) {
        auto &sensor = sensors_[slot];
        const auto &config = sensor.config;
        const nksim_body_state *base = nullptr;
        for (const auto &body : bodies)
            if (body.body == bodies_[config.link]) { base = &body; break; }
        if (!base) return RK_ERROR_BACKEND;
        double offset[3], origin[3], rotation[4], velocity[3];
        sensors::rotate(base->rotation, config.position, offset);
        sensors::multiply(base->rotation, config.rotation, rotation);
        for (int i = 0; i < 3; ++i) {
            origin[i] = base->position[i] + offset[i];
            const int j = (i+1)%3, k = (i+2)%3;
            velocity[i] = base->linear_velocity[i] + base->angular_velocity[j]*offset[k]
                - base->angular_velocity[k]*offset[j];
        }
        double acceleration[3];
        const bool derivative_valid = sensor.previous_time >= 0.0 && now > sensor.previous_time;
        if (derivative_valid)
            for (int i = 0; i < 3; ++i)
                acceleration[i] = (velocity[i] - sensor.previous_velocity[i]) / (now - sensor.previous_time);
        sensor.previous_time = now;
        std::copy_n(velocity, 3, sensor.previous_velocity);
        const bool due = now + 1e-12 >= sensor.next_due;
        if (due && (config.kind != RK_SENSOR_IMU || derivative_valid)) {
            auto &sample = sensor.sample;
            ++sample.sequence;
            sample.source_timestamp_ns = state.source_timestamp_ns;
            if (config.kind == RK_SENSOR_ENCODER) {
                sample.value_count = state.joint_count;
                std::copy_n(state.position, state.joint_count, sample.values);
            } else if (config.kind == RK_SENSOR_IMU) {
                sample.value_count = 6;
                sensors::imu(rotation, base->angular_velocity, acceleration, simulation_.gravity_, sample.values);
            } else {
                sample.value_count = config.ray_count;
                const double field_of_view = config.field_of_view > 0.0
                    ? config.field_of_view : 6.283185307179586;
                const double angular_step = config.ray_count <= 1 ? 0.0
                    : field_of_view / (field_of_view >= 6.283185307179586 - 1e-9
                        ? config.ray_count : config.ray_count - 1);
                for (uint32_t ray = 0; ray < config.ray_count; ++ray) {
                    const double angle = config.start_angle + ray * angular_step;
                    const double local[3] = {std::cos(angle), std::sin(angle), 0.0};
                    double direction[3];
                    sensors::rotate(rotation, local, direction);
                    double range = config.max_range;
                    for (const auto &body : bodies) {
                        if (std::find(bodies_.begin(), bodies_.end(), body.body) != bodies_.end()) continue;
                        const double robot_extents[3] = {0.05, 0.05, 0.05};
                        const double *extents = robot_extents;
                        for (const auto &object : simulation_.objects_)
                            if (object.active && object.body == body.body) { extents = object.half_extents; break; }
                        range = sensors::ray_box(origin, direction, body.position, body.rotation, extents, range);
                    }
                    sample.values[ray] = range;
                }
            }
            for (uint32_t i = 0; i < sample.value_count; ++i) {
                if (config.noise_stddev > 0.0)
                    sample.values[i] += config.noise_stddev * sensors::gaussian(sensor.random);
                if (config.kind == RK_SENSOR_LIDAR)
                    sample.values[i] = std::clamp(sample.values[i], 0.0, config.max_range);
            }
            if (config.update_rate > 0.0) {
                // Acquisition cannot outpace physics. Capping also prevents
                // overflow for finite but excessively high requested rates.
                const double rate = std::min(config.update_rate, 1.0 / simulation_.fixed_timestep_);
                sensor.next_due = (std::floor(now * rate + 1e-9) + 1.0) / rate;
            }
        }
        state.sensors[slot] = sensor.sample;
    }
    return RK_OK;
}

} // namespace robotkit
