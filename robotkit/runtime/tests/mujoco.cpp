#include "robotkit_simkit.h"
#include <cassert>
#include <cmath>
#include <cstdio>

static rk_robot_state state(rk_robot_runtime robot) {
    rk_robot_state value{};
    value.struct_size = sizeof(value);
    assert(rk_robot_runtime_snapshot(robot, &value) == RK_OK);
    return value;
}

int main() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.005;
    desc.physics_substeps = 2;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 2;
    model.joint_count = 1;
    model.joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -3.14, 3.14, 100.0};
    model.sensor_count = 2;
    model.sensors[0].kind = RK_SENSOR_IMU;
    model.sensors[0].link = 1; // Articulated link, not the static base.
    model.sensors[0].position[0] = 0.5; // Nonzero mount produces centripetal acceleration.
    model.sensors[0].rotation[3] = 1.0;
    model.sensors[1].kind = RK_SENSOR_LIDAR;
    model.sensors[1].rotation[3] = 1.0;
    model.sensors[1].ray_count = 16;
    model.sensors[1].max_range = 20.0;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot) == RK_OK);

    // Falling box crosses and leaves the LiDAR plane. Contact response is
    // checked independently by the sim_mujoco backend's plane-drop fixture.
    rk_simulation_object_desc box{};
    box.struct_size = sizeof(box);
    box.position[0] = 2.0;
    box.position[2] = 1.0;
    box.rotation[3] = 1.0;
    box.half_extents[0] = box.half_extents[1] = box.half_extents[2] = 0.25;
    box.mass = 1.0;
    box.motion_type = 2;
    rk_simulation_object falling = 0;
    assert(rk_simulation_spawn_object(simulation, &box, &falling) == RK_OK);

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 0.5, 0.0, 100.0};
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    bool saw_motion = false, saw_specific_force = false, saw_occlusion = false;
    for (int tick = 0; tick < 400; ++tick) {
        assert(rk_simulation_step(simulation, tick) == RK_OK);
        const auto sample = state(robot);
        if (tick == 0) {
            assert(sample.sensors[0].sequence == 0);
            assert(sample.sensors[1].values[0] == 20.0);
        }
        if (tick > 0) {
            const auto &imu = sample.sensors[0];
            assert(imu.value_count == 6);
            // MuJoCo's hinge velocity and the mounted gyroscope agree.
            assert(std::abs(imu.values[2] - sample.velocity[0]) < 1e-8);
            if (std::abs(imu.values[2]) > 0.1) saw_motion = true;
            if (imu.values[3] < -0.01) saw_specific_force = true;
            assert(std::abs(imu.values[5] - 9.81) < 1e-6);
        }
        if (sample.sensors[1].values[0] < 2.0) saw_occlusion = true;
    }
    const auto final = state(robot);
    assert(saw_motion && saw_specific_force && saw_occlusion);
    assert(std::abs(final.position[0] - 0.5) < 0.05);
    assert(final.sensors[1].values[0] == 20.0);
    assert(rk_simulation_stop(simulation) == RK_OK);
    assert(rk_simulation_remove_object(simulation, falling) == RK_OK);
    assert(rk_simulation_step(simulation, 500) == RK_OK);
    assert(state(robot).sensors[1].values[0] == 20.0);
    assert(rk_simulation_stop(simulation) == RK_OK);
    assert(rk_simulation_reset(simulation) == RK_OK);
    assert(rk_simulation_step(simulation, 0) == RK_OK);
    assert(std::abs(state(robot).position[0]) < 1e-9);
    assert(state(robot).sensors[0].sequence == 0);
    rk_simulation_destroy(simulation);
    std::puts("MuJoCo articulated IMU and moving-occluder LiDAR tests passed");
}
