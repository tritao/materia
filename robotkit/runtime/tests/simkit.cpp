#include "robotkit_simkit.h"

#include <cassert>
#include <cmath>

namespace {

rk_runtime_blueprint blueprint(uint64_t revision) {
    rk_runtime_blueprint value{};
    value.struct_size = sizeof(value);
    value.revision = revision;
    value.joint_count = 1;
    value.link_count = 2;
    value.joints[0] = {
        0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -3.14, 3.14, 100.0,
    };
    assert(rk_runtime_blueprint_validate(&value) == RK_OK);
    return value;
}

rk_robot_command target(double position, uint64_t sequence) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = RK_COMMAND_JOINT_TARGETS;
    value.target_count = 1;
    value.targets[0] = {0, RK_TARGET_POSITION, position, 0.0, 0.0};
    return value;
}

rk_robot_state snapshot(rk_runtime runtime) {
    rk_robot_state value{};
    value.struct_size = sizeof(value);
    assert(rk_runtime_snapshot(runtime, &value) == RK_OK);
    return value;
}

void shared_world_steps_once() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    rk_simulation simulation = RK_INVALID_SIMULATION;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);

    const auto first_model = blueprint(11);
    const auto second_model = blueprint(12);
    rk_runtime first = RK_INVALID_RUNTIME;
    rk_runtime second = RK_INVALID_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &first_model, &first) == RK_OK);
    assert(rk_simulation_add_robot(simulation, &second_model, &second) == RK_OK);

    const auto first_command = target(0.4, 1);
    const auto second_command = target(-0.3, 1);
    assert(rk_runtime_submit(first, &first_command) == RK_OK);
    assert(rk_runtime_submit(second, &second_command) == RK_OK);

    // Participating runtimes cannot pretend to advance shared simulation time.
    assert(rk_runtime_step(first, 1000) == RK_ERROR_INVALID_STATE);
    assert(rk_simulation_step(simulation, 1000) == RK_OK);
    rk_simulation_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 1);
    assert(std::abs(clock.simulation_time - 0.01) < 1e-12);

    const auto first_state = snapshot(first);
    const auto second_state = snapshot(second);
    assert(first_state.sequence == 1);
    assert(second_state.sequence == 1);
    assert(first_state.timestamp_ns == 1000);
    assert(second_state.timestamp_ns == 1000);
    assert(std::abs(first_state.position[0] - 0.4) < 1e-12);
    assert(std::abs(second_state.position[0] + 0.3) < 1e-12);

    rk_runtime late = RK_INVALID_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &first_model, &late) ==
           RK_ERROR_INVALID_STATE);

    assert(rk_runtime_step(second, 2000) == RK_ERROR_INVALID_STATE);
    assert(rk_simulation_step(simulation, 2000) == RK_OK);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 2);
    assert(snapshot(first).sequence == 2);
    assert(snapshot(second).sequence == 2);

    rk_simulation_destroy(simulation);
    rk_robot_state destroyed{};
    destroyed.struct_size = sizeof(destroyed);
    assert(rk_runtime_snapshot(first, &destroyed) == RK_ERROR_INVALID_HANDLE);
}

void failed_command_phase_does_not_advance() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    rk_simulation simulation = RK_INVALID_SIMULATION;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    const auto model = blueprint(21);
    rk_runtime first = RK_INVALID_RUNTIME;
    rk_runtime second = RK_INVALID_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, &first) == RK_OK);
    assert(rk_simulation_add_robot(simulation, &model, &second) == RK_OK);

    const auto first_target = target(0.8, 1);
    rk_robot_command emergency{};
    emergency.struct_size = sizeof(emergency);
    emergency.sequence = 1;
    emergency.kind = RK_COMMAND_EMERGENCY_STOP;
    const auto rejected_target = target(-0.8, 2);
    assert(rk_runtime_submit(first, &first_target) == RK_OK);
    assert(rk_runtime_submit(second, &emergency) == RK_OK);
    assert(rk_runtime_submit(second, &rejected_target) == RK_OK);
    assert(rk_simulation_step(simulation, 100) == RK_ERROR_SAFETY_STOPPED);

    rk_simulation_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 0);

    rk_robot_command clear_stop{};
    clear_stop.struct_size = sizeof(clear_stop);
    clear_stop.sequence = 3;
    clear_stop.kind = RK_COMMAND_STOP;
    assert(rk_runtime_submit(second, &clear_stop) == RK_OK);
    assert(rk_simulation_step(simulation, 200) == RK_OK);
    assert(std::abs(snapshot(first).position[0]) < 1e-12);
    rk_simulation_destroy(simulation);
}

} // namespace

int main() {
    shared_world_steps_once();
    failed_command_phase_does_not_advance();
    return 0;
}
