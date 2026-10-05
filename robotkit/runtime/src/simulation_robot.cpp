#include "simulation_robot.hpp"

#include "simulation.hpp"
#include "sensor_math.hpp"
#include <algorithm>
#include <cmath>
#include <memory>

namespace robotkit {

rk_result SimulationRobot::rebase_counters(const uint32_t *joints, const double *deltas,
                                          uint32_t count) {
    if (!joints || !deltas || count == 0 || count > counter_origin_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    if (staged_valid_ || !pending_targets_.empty()) return RK_ERROR_INVALID_STATE;
    for (uint32_t i = 0; i < count; ++i) {
        const auto joint = joints[i];
        if (joint >= counter_origin_.size() || !std::isfinite(deltas[i]) ||
            !actuated_joints_[joint] || passive_[joint]) return RK_ERROR_INVALID_ARGUMENT;
        if (squaring_hold_[joint]) return RK_ERROR_INVALID_STATE;
        for (uint32_t k = 0; k < i; ++k)
            if (joints[k] == joint) return RK_ERROR_INVALID_ARGUMENT;
        if (!std::isfinite(counter_origin_[joint] + deltas[i]) ||
            !std::isfinite(squaring_offset_[joint] - deltas[i])) return RK_ERROR_INVALID_ARGUMENT;
    }
    // All sides are validated before writing; preserve each physical target sum.
    for (uint32_t i = 0; i < count; ++i) {
        counter_origin_[joints[i]] += deltas[i];
        squaring_offset_[joints[i]] -= deltas[i];
    }
    return RK_OK;
}

std::vector<SimulationRobot::JointCommand> &SimulationRobot::staged_commands() {
    if (!staged_valid_) {
        commanded_.resize(joints_.size());
        staged_ = commanded_;
        staged_squaring_offset_ = squaring_offset_;
        staged_stopped_ = stopped_;
        staged_valid_ = true;
    }
    return staged_;
}

void SimulationRobot::queue_rest_holds() noexcept {
    for (std::size_t joint = 0; joint < held_at_rest_.size() && joint < joints_.size(); ++joint) {
        if (!held_at_rest_[joint]) continue;
        nksim_joint_target target{};
        target.struct_size = sizeof(target);
        target.joint = joints_[joint];
        target.mode = NKSIM_JOINT_TARGET_POSITION;
        target.target = joint < counter_origin_.size() ? counter_origin_[joint] : 0.0;
        if (joint < servo_.size() && servo_[joint].stiffness > 0.0) {
            target.mode = NKSIM_JOINT_TARGET_SERVO;
            target.stiffness = servo_[joint].stiffness;
            target.damping = servo_[joint].damping;
        }
        pending_targets_.push_back(target);
        staged_commands()[joint] = {target.mode, 0.0};
    }
}

void SimulationRobot::queue_velocity_hold(std::size_t joint) {
    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joints_[joint];
    target.mode = NKSIM_JOINT_TARGET_VELOCITY;
    target.target = 0.0;
    pending_targets_.push_back(target);
    staged_commands()[joint] = {NKSIM_JOINT_TARGET_VELOCITY, 0.0};
}

rk_result SimulationRobot::apply(const rk_robot_command &command) {
    // Lifecycle commands such as reset do not necessarily stage a target,
    // but their safety state must still roll back if another robot rejects the
    // shared simulation tick.
    staged_commands();
    if (command.kind == RK_COMMAND_EMERGENCY_STOP) {
        stopped_ = true;
        pending_targets_.clear();
        for (std::size_t index = 0; index < joints_.size(); ++index) {
            if (index >= actuated_joints_.size() || !actuated_joints_[index])
                continue;
            if (index < brake_group_.size() && brake_group_[index]) {
                nksim_joint_target target{};
                target.struct_size = sizeof(target);
                target.joint = joints_[index];
                target.mode = NKSIM_JOINT_TARGET_EFFORT;
                target.target = 0.0;
                pending_targets_.push_back(target);
                staged_commands()[index] = {NKSIM_JOINT_TARGET_EFFORT, 0.0};
            } else {
                queue_velocity_hold(index);
            }
        }
        return RK_OK;
    }
    if (command.kind == RK_COMMAND_STOP) {
        // A normal stop is a non-latching stop. The runtime normally converts
        // an active buffered trajectory into a short position-target ramp;
        // this branch handles direct velocity/effort targets that have no
        // runtime trajectory to ramp.
        // Position- and servo-held joints keep their current, already
        // rate-limited reference, which is where they stop; never-commanded
        // joints stay passive.
        stopped_ = false;
        pending_targets_.clear();
        const auto staged = staged_commands();
        for (std::size_t index = 0; index < joints_.size(); ++index) {
            if (index >= actuated_joints_.size() || !actuated_joints_[index] ||
                staged[index].mode == 0 ||
                staged[index].mode == NKSIM_JOINT_TARGET_POSITION ||
                staged[index].mode == NKSIM_JOINT_TARGET_SERVO)
                continue;
            queue_velocity_hold(index);
        }
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
    // Full plans already name followers. Sparse direct commands still need the
    // old coupling propagation, now applied before per-shaft physical offsets.
    bool needs_projection = false;
    for (const auto &term : kinematic_couplings_) {
        bool leader = false, follower = false;
        for (uint32_t i = 0; i < command.target_count; ++i) {
            leader |= command.targets[i].joint == term.leader;
            follower |= command.targets[i].joint == term.follower;
        }
        needs_projection |= leader && !follower;
    }
    if (needs_projection) {
        auto expanded = std::make_unique<rk_robot_command>(command);
        bool added = false;
        for (std::size_t pass = 0; pass < joints_.size(); ++pass) {
            bool changed = false;
            for (const auto &term : kinematic_couplings_) {
                const rk_joint_target *leader = nullptr;
                bool follower = false;
                for (uint32_t i = 0; i < expanded->target_count; ++i) {
                    if (expanded->targets[i].joint == term.leader) leader = &expanded->targets[i];
                    if (expanded->targets[i].joint == term.follower) follower = true;
                }
                if (!leader || follower) continue;
                if (expanded->target_count >= RK_MAX_JOINTS) return RK_ERROR_INVALID_ARGUMENT;
                rk_joint_target target = *leader;
                target.joint = term.follower;
                if (target.mode == RK_TARGET_POSITION || target.mode == RK_TARGET_SERVO) {
                    target.mode = RK_TARGET_POSITION;
                    target.target = term.ratio * leader->target + term.offset;
                } else if (target.mode == RK_TARGET_VELOCITY) target.target *= term.ratio;
                else return RK_ERROR_UNSUPPORTED;
                if (!std::isfinite(target.target)) return RK_ERROR_INVALID_ARGUMENT;
                expanded->targets[expanded->target_count++] = target;
                changed = added = true;
            }
            if (!changed) break;
        }
        if (added) return apply(*expanded);
    }
    for (uint32_t index = 0; index < command.target_count; ++index)
        if (command.targets[index].joint >= joints_.size())
            return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < command.target_count; ++index) {
        const auto &source = command.targets[index];
        if (squaring_hold_[source.joint] && source.mode != RK_TARGET_POSITION &&
            source.mode != RK_TARGET_SERVO) return RK_ERROR_INVALID_STATE;
    }
    auto &staged = staged_commands();
    for (uint32_t index = 0; index < command.target_count; ++index) {
        const auto &source = command.targets[index];
        // A fixed joint cannot move, so its target is already met. Planners that
        // command a whole robot, fixed mounting joints included, rely on this.
        if (source.joint >= actuated_joints_.size() || !actuated_joints_[source.joint])
            continue;
        const bool positional = source.mode == RK_TARGET_POSITION || source.mode == RK_TARGET_SERVO;
        // A joint a servo motor moves through couplings is not commanded; the coupling carries it.
        if (positional && source.joint < passive_.size() && passive_[source.joint])
            continue;
        nksim_joint_target target{};
        target.struct_size = sizeof(target);
        target.joint = joints_[source.joint];
        target.mode = source.mode;
        target.target = source.target;
        target.max_force = source.max_effort;
        if (positional && squaring_hold_[source.joint])
            staged_squaring_offset_[source.joint] = squaring_position_[source.joint] - source.target -
                counter_origin_[source.joint] - slip_[source.joint];
        if (positional && source.joint < slip_.size())
            target.target += slip_[source.joint] + staged_squaring_offset_[source.joint] +
                counter_origin_[source.joint];
        if (source.mode == RK_TARGET_POSITION && source.joint < servo_.size() &&
            servo_[source.joint].stiffness > 0.0) {
            // The drive interpolates the analytic trajectory reference within this cycle.
            const auto &gains = servo_[source.joint];
            target.mode = NKSIM_JOINT_TARGET_SERVO;
            target.stiffness = gains.stiffness;
            target.damping = gains.damping;
            if (index < RK_MAX_SERVO_JOINTS) {
                target.velocity = command.servos[index].velocity;
            }
        }
        if (source.mode == RK_TARGET_SERVO) {
            const auto &servo = command.servos[index];
            target.velocity = servo.velocity;
            target.stiffness = servo.stiffness;
            target.damping = servo.damping;
            target.feedforward = servo.feedforward;
        }
        if (target.mode == NKSIM_JOINT_TARGET_SERVO && index < RK_MAX_SERVO_JOINTS) {
            target.end_position = command.reference_end_position[index] + slip_[source.joint] +
                staged_squaring_offset_[source.joint] + counter_origin_[source.joint];
            target.end_velocity = command.reference_end_velocity[index];
            target.reference_duration = command.reference_duration;
        }
        if (squaring_hold_[source.joint]) {
            target.target = squaring_position_[source.joint];
            target.velocity = 0.0;
            if (target.mode == NKSIM_JOINT_TARGET_SERVO) {
                target.end_position = target.target;
                target.end_velocity = 0.0;
                target.feedforward = 0.0;
            }
        }
        pending_targets_.push_back(target);
        staged[source.joint] = {target.mode, target.target};
    }
    return RK_OK;
}

std::vector<nksim_joint_target> SimulationRobot::take_pending_targets() {
    auto result = std::move(pending_targets_);
    pending_targets_.clear();
    if (staged_valid_) {
        commanded_ = staged_;
        squaring_offset_ = staged_squaring_offset_;
        staged_valid_ = false;
    }
    return result;
}

std::optional<ProcessChannelFault> SimulationRobot::apply_pneumatic_valves(const RobotRuntime &runtime,
                                        std::vector<nksim_joint_target> &targets, bool deenergized) {
    const ProcessChannelReader reader([&runtime](const char *channel, rk_event_value &value) {
        return runtime.channel_value(channel, value);
    });
    // Resolve every coil before committing any valve latch state.
    std::vector<bool> spool_states;
    for (const auto &drive : pneumatic_drives_) {
        const auto a = deenergized ? ProcessChannelReading<bool>{} : reader.digital(drive.channel_a);
        if (a.fault) return a.fault;
        bool to_a = drive.to_a;
        if (!drive.has_channel_b) {
            to_a = a.value ? !drive.normally_to_a : drive.normally_to_a;
        } else {
            const auto b = deenergized ? ProcessChannelReading<bool>{} : reader.digital(drive.channel_b);
            if (b.fault) return b.fault;
            // Both coils off or on hold the last double-solenoid spool state.
            if (a.value != b.value) to_a = a.value;
        }
        spool_states.push_back(to_a);
    }
    for (std::size_t index = 0; index < pneumatic_drives_.size(); ++index) {
        auto &drive = pneumatic_drives_[index];
        drive.to_a = spool_states[index];
        nksim_joint_target target{};
        target.struct_size = sizeof(target);
        target.joint = joints_[drive.joint];
        target.mode = NKSIM_JOINT_TARGET_EFFORT;
        target.target = drive.to_a ? drive.extension_force * drive.extend_sign
            : -drive.retraction_force * drive.extend_sign;
        target.max_force = std::max(drive.extension_force, drive.retraction_force);
        targets.push_back(target);
    }
    return std::nullopt;
}

std::optional<ProcessChannelFault> SimulationRobot::apply_velocity_drives(const RobotRuntime &runtime,
                                        std::vector<nksim_joint_target> &targets, bool deenergized) {
    const ProcessChannelReader reader([&runtime](const char *channel, rk_event_value &value) {
        return runtime.channel_value(channel, value);
    });
    for (const auto &drive : velocity_drives_) {
        const auto speed = deenergized ? ProcessChannelReading<double>{} : reader.analog(drive.speed_channel);
        if (speed.fault) return speed.fault;
        const auto direction = deenergized ? ProcessChannelReading<double>{} : reader.analog(drive.direction_channel);
        if (direction.fault) return direction.fault;
        const auto velocity = std::clamp(speed.value * std::clamp(direction.value, -1.0, 1.0) *
            drive.radians_per_speed_unit, -drive.max_rate, drive.max_rate);
        nksim_joint_target target{};
        target.struct_size = sizeof(target);
        target.joint = joints_[drive.joint];
        target.mode = NKSIM_JOINT_TARGET_VELOCITY;
        target.target = velocity;
        target.max_force = drive.max_effort;
        targets.push_back(target);
        commanded_[drive.joint] = {target.mode, target.target};
    }
    return std::nullopt;
}

rk_result SimulationRobot::apply_brake_coils(const RobotRuntime &runtime,
    std::optional<ProcessChannelFault> &fault, bool deenergized) {
    const ProcessChannelReader reader([&runtime](const char *channel, rk_event_value &value) {
        return runtime.channel_value(channel, value);
    });
    nksim_brake_output outputs[RK_MAX_JOINTS]{};
    for (std::size_t i = 0; i < brakes_.size(); ++i) {
        const auto &brake = brakes_[i];
        const auto coil = deenergized ? ProcessChannelReading<bool>{} : reader.digital(brake.release_channel);
        if (coil.fault) { fault = coil.fault; return RK_OK; }
        outputs[i].joint = joints_[brake.joint];
        outputs[i].energized = coil.value ? 1 : 0;
    }
    return nksim_session_submit_brake_outputs(simulation_.session_, outputs,
        static_cast<uint32_t>(brakes_.size())) == NKSIM_OK ? RK_OK : RK_ERROR_BACKEND;
}

rk_result SimulationRobot::sample(uint64_t timestamp_ns, rk_robot_state &state) {
    const auto snapshot = nksim_session_latest_snapshot(simulation_.session_);
    nksim_session_status status{};
    status.struct_size = sizeof(status);
    if (snapshot == 0 || nksim_session_get_status(simulation_.session_, &status) != NKSIM_OK)
        return RK_ERROR_INVALID_STATE;
    state.struct_size = sizeof(state);
    state.source_timestamp_ns = static_cast<uint64_t>(status.simulation_time * 1'000'000'000.0);
    state.joint_count = static_cast<uint32_t>(joints_.size());
    for (uint32_t index = 0; index < state.joint_count; ++index)
        state.position[index] = state.velocity[index] = state.effort[index] = 0.0;
    uint64_t count = 0;
    if (nksim_snapshot_get_joint_count(snapshot, &count) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    for (uint64_t index = 0; index < count; ++index) {
        nksim_joint_state source{};
        source.struct_size = sizeof(source);
        if (nksim_snapshot_get_joint(snapshot, index, &source) != NKSIM_OK)
            return RK_ERROR_BACKEND;
        for (uint32_t target = 0; target < state.joint_count; ++target) {
            if (source.joint == joints_[target]) {
                state.position[target] = source.position - counter_origin_[target];
                state.velocity[target] = source.velocity;
                state.effort[target] = source.effort;
                joint_brake_state_[target] = {source.brake_configured != 0, source.brake_released != 0};
                break;
            }
        }
    }
    // All measurements use the same immutable physics snapshot as encoders.
    uint64_t body_count = 0;
    if (nksim_snapshot_get_body_count(snapshot, &body_count) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    std::vector<nksim_body_state> bodies(body_count);
    for (uint64_t index = 0; index < body_count; ++index) {
        bodies[index].struct_size = sizeof(nksim_body_state);
        if (nksim_snapshot_get_body(snapshot, index, &bodies[index]) != NKSIM_OK)
            return RK_ERROR_BACKEND;
    }
    const double now = status.simulation_time;
    state.sensor_count = static_cast<uint32_t>(sensors_.size());
    uint32_t value_cursor = 0;
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
                std::copy_n(state.position, state.joint_count, sensor.values);
            } else if (config.kind == RK_SENSOR_IMU) {
                sample.value_count = 6;
                sensors::imu(rotation, base->angular_velocity, acceleration, simulation_.gravity_, sensor.values);
            } else if (config.kind == RK_SENSOR_BRAKE_RELEASED) {
                if (sensor.joint >= joint_brake_state_.size() || !joint_brake_state_[sensor.joint].configured)
                    return RK_ERROR_BACKEND;
                sample.value_count = 1;
                sensor.values[0] = joint_brake_state_[sensor.joint].released ? 1.0 : 0.0;
            } else if (config.kind == RK_SENSOR_JOINT_SWITCH) {
                if (sensor.joint >= state.joint_count) return RK_ERROR_BACKEND;
                const double position = state.position[sensor.joint];
                if (sensor.active) {
                    if (position < sensor.window_lower - sensor.hysteresis ||
                        position > sensor.window_upper + sensor.hysteresis)
                        sensor.active = false;
                } else if (position >= sensor.window_lower && position <= sensor.window_upper) {
                    sensor.active = true;
                }
                sample.value_count = 1;
                sensor.values[0] = sensor.active ? 1.0 : 0.0;
            } else if (config.kind == RK_SENSOR_AT_SPEED) {
                if (sensor.joint >= state.joint_count) return RK_ERROR_BACKEND;
                const double target = std::abs(applied_velocity(sensor.speed_reference_joint) * sensor.speed_reference_scale);
                const double error = std::abs(std::abs(state.velocity[sensor.joint]) - target);
                // window_lower and hysteresis are fractions of the live command, not a fixed rpm.
                const double tolerance = target * sensor.window_lower;
                if (target <= 0.0) sensor.active = false;
                else if (sensor.active) {
                    if (error > target * (sensor.window_lower + sensor.hysteresis)) sensor.active = false;
                } else if (error <= tolerance) sensor.active = true;
                sample.value_count = 1;
                sensor.values[0] = sensor.active ? 1.0 : 0.0;
            } else if (config.kind == RK_SENSOR_PRESENCE) {
                sample.value_count = 1;
                const double local[3] = {0.0, 0.0, 1.0};
                double direction[3];
                sensors::rotate(rotation, local, direction);
                nksim_pose volume{};
                volume.struct_size = sizeof(volume);
                for (int axis = 0; axis < 3; ++axis)
                    volume.position[axis] = origin[axis] + direction[axis] * config.max_range * 0.5;
                std::copy_n(rotation, 4, volume.rotation);
                const double half_extents[3] = {config.max_range, config.max_range,
                                                config.max_range * 0.5};
                uint32_t present = 0;
                if (nksim_session_overlap_free_boxes(simulation_.session_, &volume,
                                                     half_extents, &present) != NKSIM_OK)
                    return RK_ERROR_BACKEND;
                sensor.values[0] = present ? 1.0 : 0.0;
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
                    // Session objects and actors (props, people) by their own
                    // shapes; other robots' links as their 10 cm boxes.
                    nksim_ray cast{};
                    cast.struct_size = sizeof(cast);
                    std::copy_n(origin, 3, cast.origin);
                    std::copy_n(direction, 3, cast.direction);
                    cast.max_distance = config.max_range;
                    double range = config.max_range;
                    if (nksim_session_raycast(simulation_.session_, &cast, &range) != NKSIM_OK)
                        return RK_ERROR_BACKEND;
                    const double robot_extents[3] = {0.05, 0.05, 0.05};
                    for (const auto &body : bodies) {
                        if (std::find(bodies_.begin(), bodies_.end(), body.body) != bodies_.end() ||
                            std::find(simulation_.bodies_.begin(), simulation_.bodies_.end(),
                                      body.body) == simulation_.bodies_.end())
                            continue;
                        range = sensors::ray_box(origin, direction, body.position, body.rotation,
                                                 robot_extents, range);
                    }
                    sensor.values[ray] = range;
                }
            }
            for (uint32_t i = 0; i < sample.value_count; ++i) {
                if (config.noise_stddev > 0.0 && config.kind != RK_SENSOR_JOINT_SWITCH &&
                    config.kind != RK_SENSOR_AT_SPEED && config.kind != RK_SENSOR_PRESENCE)
                    sensor.values[i] += config.noise_stddev * sensors::gaussian(sensor.random);
                if (config.kind == RK_SENSOR_LIDAR)
                    sensor.values[i] = std::clamp(sensor.values[i], 0.0, config.max_range);
            }
            if (config.update_rate > 0.0) {
                // Acquisition cannot outpace physics. Capping also prevents
                // overflow for finite but excessively high requested rates.
                const double rate = std::min(config.update_rate, 1.0 / simulation_.fixed_timestep_);
                sensor.next_due = (std::floor(now * rate + 1e-9) + 1.0) / rate;
            }
        }
        // Pack this slot's values after the previous slots' in the state's pool.
        if (value_cursor + sensor.sample.value_count > RK_SENSOR_VALUE_POOL) return RK_ERROR_BACKEND;
        state.sensors[slot] = sensor.sample;
        state.sensors[slot].value_offset = value_cursor;
        std::copy_n(sensor.values, sensor.sample.value_count, state.sensor_values + value_cursor);
        value_cursor += sensor.sample.value_count;
    }
    return RK_OK;
}

} // namespace robotkit
