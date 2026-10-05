#include "robotkit_simkit.h"
#include "nativekit_sim_mujoco.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <memory>
#include <cstdio>

void check_brake(double holding_force, bool holds) {
    nkscene_scene scene = 0;
    nksim_world world = 0;
    nksim_session session = 0;
    rk_simulation simulation = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc physics{};
    physics.struct_size = sizeof(physics);
    physics.scene = scene;
    physics.fixed_timestep = 0.01;
    physics.physics_substeps = 20;
    physics.gravity[2] = -9.81;
    assert(nksim_mujoco_world_create(&physics, &world) == NKSIM_OK);
    nksim_session_desc owner{};
    owner.struct_size = sizeof(owner);
    owner.scene = scene;
    owner.world = world;
    assert(nksim_session_create(&owner, &session) == NKSIM_OK);
    assert(rk_simulation_create_in_session(session, &simulation) == RK_OK);

    auto blueprint = std::make_unique<rk_robot_runtime_blueprint>();
    blueprint->struct_size = sizeof(*blueprint);
    blueprint->revision = 1;
    blueprint->joint_count = 1;
    blueprint->link_count = 2;
    blueprint->collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (uint32_t i = 0; i < 2; ++i) {
        blueprint->links[i].mass = i == 0 ? 1.0 : 10.0;
        blueprint->links[i].inertia_tensor[0] = blueprint->links[i].inertia_tensor[4] = blueprint->links[i].inertia_tensor[8] = 0.01;
    }
    blueprint->joints[0] = {0, RK_RUNTIME_JOINT_PRISMATIC, 0, 1, -5.0, 5.0, 1000.0};
    blueprint->joints[0].limit_flags |= RK_LIMIT_EFFORT;
    blueprint->joints[0].axis[2] = 1.0;
    blueprint->joints[0].parent_frame_rotation[3] = blueprint->joints[0].child_frame_rotation[3] = 1.0;
    blueprint->joint_servo[0] = {1.0e6, 7000.0};
    blueprint->channel_count = 1;
    std::strcpy(blueprint->channels[0].id, "z.brake.release");
    blueprint->channels[0].kind = blueprint->channels[0].safe_value.kind = RK_EVENT_DIGITAL;
    blueprint->channels[0].stop_policy = RK_CHANNEL_KEEP_ON_STOP;
    blueprint->sensor_count = 1;
    blueprint->sensors[0].kind = RK_SENSOR_BRAKE_RELEASED;
    blueprint->sensors[0].rotation[3] = 1.0;
    blueprint->sensor_joint[0] = 0;
    blueprint->brake_count = 1;
    blueprint->brakes[0] = {0, 0, 0, 0, 250'000'000, 30'000'000};
    assert(rk_robot_runtime_blueprint_validate(blueprint.get()) == RK_OK);
    rk_robot_runtime runtime = 0;
    assert(rk_simulation_add_robot(simulation, blueprint.get(), nullptr, &runtime) == RK_ERROR_INVALID_ARGUMENT);
    auto description = std::make_unique<rk_simulation_robot_desc>();
    description->struct_size = sizeof(*description);
    description->initial_pose.struct_size = sizeof(description->initial_pose);
    description->initial_pose.position[2] = 1.0;
    description->initial_pose.rotation[3] = 1.0;
    description->brake_count = 1;
    description->brakes[0] = {0, 0, holding_force, 0.05, 0.03};
    assert(rk_simulation_add_robot(simulation, blueprint.get(), description.get(), &runtime) == RK_OK);
    rk_robot_snapshot state{};
    state.struct_size = sizeof(state);
    uint64_t tick = 0;
    auto step = [&]() {
        tick += 10'000'000;
        const auto result = nksim_session_step(session, tick, nullptr);
        if (result != NKSIM_OK) {
            const auto snapshot_result = rk_robot_runtime_snapshot_full(runtime, &state);
            std::fprintf(stderr, "brake step %llu: result %d, snapshot %d, fault %d, safety %u\n",
                static_cast<unsigned long long>(tick), result, snapshot_result, state.fault_code, state.safety);
        }
        assert(result == NKSIM_OK);
        assert(rk_robot_runtime_snapshot_full(runtime, &state) == RK_OK);
    };
    for (int i = 0; i < 12; ++i) {
        step();
        assert(state.sensor_count == 1 && state.sensors[0].value_count == 1);
        if (i < 3) assert(RK_SENSOR_VALUE(state, 0, 0) == 0.0);
        assert(std::abs(state.position[0]) < 0.002); // idle servo hold precedes release
    }
    assert(RK_SENSOR_VALUE(state, 0, 0) == 1.0);
    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_SERVO, 0.02, 1.0, 1000.0};
    command.servos[0] = {0.0, 1.0e6, 7000.0, 0.0};
    assert(rk_robot_runtime_submit(runtime, &command) == RK_OK);
    for (int i = 0; i < 30; ++i) step();
    assert(std::abs(state.position[0] - 0.02) < 0.002);
    command.sequence = 2;
    command.kind = RK_COMMAND_STOP;
    command.target_count = 0;
    assert(rk_robot_runtime_submit(runtime, &command) == RK_OK);
    const double stopped = state.position[0];
    for (int i = 0; i < 30; ++i) step();
    assert(RK_SENSOR_VALUE(state, 0, 0) == 1.0);
    assert(std::abs(state.position[0] - stopped) < 0.002);
    command.sequence = 3;
    command.kind = RK_COMMAND_EMERGENCY_STOP;
    assert(rk_robot_runtime_submit(runtime, &command) == RK_OK);
    for (int i = 0; i < 30; ++i) step();
    assert(RK_SENSOR_VALUE(state, 0, 0) == 0.0);
    const double applied = state.position[0];
    for (int i = 0; i < 30; ++i) step();
    std::fprintf(stderr, "brake %.1f N: powered %.9f m, applied %.9f m, drift %.9f m\n",
        holding_force, stopped, applied, state.position[0] - applied);
    if (holds) assert(std::abs(state.position[0] - applied) < 1.0e-5);
    else assert(applied - state.position[0] > 0.1); // motor torque is off; an undersized brake slips
    if (holds) {
        command.sequence = 4;
        command.kind = RK_COMMAND_RESET_SAFETY;
        assert(rk_robot_runtime_submit(runtime, &command) == RK_OK);
        const double reset_position = state.position[0];
        for (int i = 0; i < 20; ++i) step();
        assert(RK_SENSOR_VALUE(state, 0, 0) == 1.0);
        assert(std::abs(state.position[0] - reset_position) < 0.002);
    }
    rk_simulation_destroy(simulation);
    nksim_session_destroy(session);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

int main() {
    check_brake(150.0, true);
    check_brake(50.0, false);
}
