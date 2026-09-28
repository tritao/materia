// Mixed-scene safety (robotkit/plans/MIXED_SCENE.md): robot arms, a CNC gantry
// and a floating-base legged robot share one Simulation on the MuJoCo backend
// without disturbing each other. Every check names the property it protects.
#include "robotkit_simkit.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

int failures = 0;

#define EXPECT(condition, ...) \
    do { \
        if (!(condition)) { \
            ++failures; \
            std::printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #condition); \
            std::printf(__VA_ARGS__); \
            std::printf("\n"); \
        } \
    } while (0)

using Blueprint = std::unique_ptr<rk_robot_runtime_blueprint>;
using RobotDesc = std::unique_ptr<rk_simulation_robot_desc>;

Blueprint new_blueprint(uint32_t links, uint32_t approximation) {
    Blueprint value(new rk_robot_runtime_blueprint{});
    value->struct_size = sizeof(*value);
    value->link_count = links;
    value->collision_approximation = approximation;
    for (uint32_t index = 0; index < links; ++index) {
        value->links[index].mass = 1.0;
        value->links[index].inertia_tensor[0] = value->links[index].inertia_tensor[4] =
            value->links[index].inertia_tensor[8] = 0.02;
    }
    return value;
}

RobotDesc new_desc(double x, double y, double z) {
    RobotDesc value(new rk_simulation_robot_desc{});
    value->struct_size = sizeof(*value);
    value->initial_pose.struct_size = sizeof(value->initial_pose);
    value->initial_pose.position[0] = x;
    value->initial_pose.position[1] = y;
    value->initial_pose.position[2] = z;
    value->initial_pose.rotation[3] = 1.0;
    return value;
}

void add_joint(rk_robot_runtime_blueprint &model, uint32_t type, uint32_t parent, uint32_t child,
               double lower, double upper, double effort, const double offset[3],
               const double axis[3]) {
    auto &joint = model.joints[model.joint_count];
    joint.joint = model.joint_count;
    joint.type = type;
    joint.parent_link = parent;
    joint.child_link = child;
    joint.lower_limit = lower;
    joint.upper_limit = upper;
    joint.max_effort = effort;
    for (int i = 0; i < 3; ++i) {
        joint.parent_frame_position[i] = offset[i];
        joint.axis[i] = axis[i];
    }
    joint.parent_frame_rotation[3] = joint.child_frame_rotation[3] = 1.0;
    ++model.joint_count;
}

// A 3R arm on a kinematic base, hung out along +X; links collide as boxes.
Blueprint make_arm() {
    auto model = new_blueprint(4, RK_COLLISION_APPROXIMATION_BOUNDS_BOX);
    const double base[3] = {0, 0, 0}, reach[3] = {0.4, 0, 0};
    const double y[3] = {0, 1, 0}, z[3] = {0, 0, 1};
    for (uint32_t link = 1; link < 4; ++link) model->links[link].center_of_mass[0] = 0.2;
    add_joint(*model, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -3.0, 3.0, 300.0, base, z);
    add_joint(*model, RK_RUNTIME_JOINT_REVOLUTE, 1, 2, -3.0, 3.0, 200.0, reach, y);
    add_joint(*model, RK_RUNTIME_JOINT_REVOLUTE, 2, 3, -3.0, 3.0, 100.0, reach, y);
    return model;
}

// An XYZ gantry, the CNC machine of the toolpath scenarios: three prismatic axes.
Blueprint make_gantry() {
    auto model = new_blueprint(4, RK_COLLISION_APPROXIMATION_BOUNDS_BOX);
    const double origin[3] = {0, 0, 0};
    const double x[3] = {1, 0, 0}, y[3] = {0, 1, 0}, z[3] = {0, 0, 1};
    add_joint(*model, RK_RUNTIME_JOINT_PRISMATIC, 0, 1, -0.3, 0.3, 500.0, origin, x);
    add_joint(*model, RK_RUNTIME_JOINT_PRISMATIC, 1, 2, -0.3, 0.3, 500.0, origin, y);
    add_joint(*model, RK_RUNTIME_JOINT_PRISMATIC, 2, 3, -0.3, 0.3, 500.0, origin, z);
    return model;
}

// A floating torso with two legs, each a hip and a knee hinged about Y with a
// narrow range, so a robot that falls onto its legs presses them onto stops.
// `filter` is the contact filter of the leg shapes (NKSIM_CONTACT_*).
struct Humanoid {
    Blueprint blueprint;
    RobotDesc desc;
};
Humanoid make_humanoid(double x, double y, double z, uint32_t filter, double tolerance,
                       double pitch) {
    Humanoid value{new_blueprint(5, RK_COLLISION_APPROXIMATION_NONE), new_desc(x, y, z)};
    value.desc->initial_pose.rotation[1] = std::sin(0.5 * pitch);
    value.desc->initial_pose.rotation[3] = std::cos(0.5 * pitch);
    auto &model = *value.blueprint;
    model.floating_base = 1;
    model.observed_limit_tolerance = tolerance;
    model.links[0].mass = 12.0;
    const double hip_l[3] = {0, 0.1, -0.15}, hip_r[3] = {0, -0.1, -0.15}, knee[3] = {0, 0, -0.25};
    const double axis[3] = {0, 1, 0};
    for (uint32_t link = 1; link < 5; ++link) model.links[link].mass = 2.0;
    add_joint(model, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -0.05, 0.05, 50.0, hip_l, axis);
    add_joint(model, RK_RUNTIME_JOINT_REVOLUTE, 1, 2, -0.05, 0.05, 50.0, knee, axis);
    add_joint(model, RK_RUNTIME_JOINT_REVOLUTE, 0, 3, -0.05, 0.05, 50.0, hip_r, axis);
    add_joint(model, RK_RUNTIME_JOINT_REVOLUTE, 3, 4, -0.05, 0.05, 50.0, knee, axis);
    auto &desc = *value.desc;
    desc.link_shape_count = 5;
    for (uint32_t link = 0; link < 5; ++link) {
        auto &shape = desc.link_shapes[link];
        shape.link = link;
        shape.rotation[3] = 1.0;
        shape.contact_filter = filter;
        if (link == 0) {
            shape.type = RK_LINK_SHAPE_BOX;
            shape.size[0] = 0.08;
            shape.size[1] = 0.15;
            shape.size[2] = 0.15;
        } else {
            shape.type = RK_LINK_SHAPE_CAPSULE;
            shape.size[0] = 0.04;
            shape.size[1] = 0.1;
            shape.position[2] = -0.12;
        }
    }
    return value;
}

rk_robot_command position_command(uint64_t sequence, uint32_t joints, const double *targets) {
    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = sequence;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = joints;
    for (uint32_t joint = 0; joint < joints; ++joint)
        command.targets[joint] = {joint, RK_TARGET_POSITION, targets[joint], 0.0, 0.0};
    return command;
}

rk_robot_command servo_hold(uint64_t sequence, uint32_t joints, double stiffness, double damping) {
    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = sequence;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = joints;
    for (uint32_t joint = 0; joint < joints; ++joint) {
        command.targets[joint] = {joint, RK_TARGET_SERVO, 0.0, 0.0, 0.0};
        command.servos[joint] = {0.0, stiffness, damping, 0.0};
    }
    return command;
}

struct Options {
    bool arm = true, gantry = true, humanoid = false, humanoid_first = false;
    uint32_t humanoid_filter = 0;
    double humanoid_tolerance = 0.0;
    double humanoid_height = 0.8;
    double humanoid_pitch = 0.0; // Radians about Y: a tilted start topples forward.
    double humanoid_x = 10.0;
    double timestep = 0.01;
    uint32_t substeps = 5;
    uint32_t integrator = 0, solver_iterations = 0, line_search_iterations = 0;
    bool floor_box = false; // A floor under the humanoid only.
    bool plane = false;     // An infinite ground plane through z = 0.
    double gantry_z = 0.5;
};

// One robot's observable trace: joint state and every link pose, each tick.
struct Trace {
    std::vector<double> values;
    bool operator==(const Trace &other) const {
        return values.size() == other.values.size() &&
            (values.empty() || std::memcmp(values.data(), other.values.data(),
                                           values.size() * sizeof(double)) == 0);
    }
    double max_difference(const Trace &other) const {
        double worst = 0.0;
        if (values.size() != other.values.size()) return 1e300;
        for (std::size_t i = 0; i < values.size(); ++i)
            worst = std::fmax(worst, std::fabs(values[i] - other.values[i]));
        return worst;
    }
};

struct Scene {
    rk_simulation simulation = 0;
    rk_robot_runtime arm = 0, gantry = 0, humanoid = 0;
    int arm_index = -1, gantry_index = -1, humanoid_index = -1;
    uint32_t arm_links = 4, gantry_links = 4, humanoid_links = 5;
    rk_simulation_object floor = 0;
    std::vector<rk_result> results;
    Trace arm_trace, gantry_trace;
    uint64_t sequence = 0;
    bool policy = false; // The humanoid is sent a fresh servo batch every tick, like a policy.
    ~Scene() { if (simulation) rk_simulation_destroy(simulation); }
};

void record(Scene &scene, Trace &trace, rk_robot_runtime runtime, int index, uint32_t links) {
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    if (rk_robot_runtime_snapshot(runtime, &state) != RK_OK) {
        trace.values.push_back(std::nan(""));
        return;
    }
    for (uint32_t joint = 0; joint < state.joint_count; ++joint) {
        trace.values.push_back(state.position[joint]);
        trace.values.push_back(state.velocity[joint]);
        trace.values.push_back(state.effort[joint]);
    }
    for (uint32_t link = 0; link < links; ++link) {
        rk_simulation_pose pose{};
        pose.struct_size = sizeof(pose);
        rk_simulation_get_link_pose(scene.simulation, static_cast<uint32_t>(index), link, &pose);
        for (int i = 0; i < 3; ++i) trace.values.push_back(pose.position[i]);
        for (int i = 0; i < 4; ++i) trace.values.push_back(pose.rotation[i]);
    }
}

void build(Scene &scene, const Options &options) {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = options.timestep;
    desc.physics_substeps = options.substeps;
    desc.backend = 1;
    desc.integrator = options.integrator;
    desc.solver_iterations = options.solver_iterations;
    desc.line_search_iterations = options.line_search_iterations;
    assert(rk_simulation_create(&desc, &scene.simulation) == RK_OK);
    int next = 0;
    const auto add_humanoid = [&] {
        auto humanoid = make_humanoid(options.humanoid_x, 0.0, options.humanoid_height,
                                      options.humanoid_filter, options.humanoid_tolerance,
                                      options.humanoid_pitch);
        scene.humanoid_index = next++;
        assert(rk_simulation_add_robot(scene.simulation, humanoid.blueprint.get(),
                                       humanoid.desc.get(), &scene.humanoid) == RK_OK);
    };
    if (options.humanoid && options.humanoid_first) add_humanoid();
    if (options.arm) {
        auto model = make_arm();
        auto desc_arm = new_desc(0.0, 0.0, 1.0);
        scene.arm_index = next++;
        assert(rk_simulation_add_robot(scene.simulation, model.get(), desc_arm.get(),
                                       &scene.arm) == RK_OK);
    }
    if (options.gantry) {
        auto model = make_gantry();
        auto desc_gantry = new_desc(3.0, 0.0, options.gantry_z);
        scene.gantry_index = next++;
        assert(rk_simulation_add_robot(scene.simulation, model.get(), desc_gantry.get(),
                                       &scene.gantry) == RK_OK);
    }
    if (options.humanoid && !options.humanoid_first) add_humanoid();
    if (options.floor_box) {
        rk_simulation_object_desc floor{};
        floor.struct_size = sizeof(floor);
        floor.rotation[3] = 1.0;
        floor.position[0] = options.humanoid_x;
        floor.position[2] = -0.5;
        floor.half_extents[0] = floor.half_extents[1] = 2.0;
        floor.half_extents[2] = 0.5;
        assert(rk_simulation_spawn_object(scene.simulation, &floor, &scene.floor) == RK_OK);
    }
    if (options.plane) {
        rk_simulation_object_desc plane{};
        plane.struct_size = sizeof(plane);
        plane.rotation[3] = 1.0;
        plane.shape = 1;
        assert(rk_simulation_spawn_object(scene.simulation, &plane, &scene.floor) == RK_OK);
    }
    if (scene.humanoid) {
        const auto hold = servo_hold(1, 4, 8.0, 0.5);
        assert(rk_robot_runtime_submit(scene.humanoid, &hold) == RK_OK);
    }
}

// Steps `ticks` times. The arm and the gantry follow a scripted job, the way a
// motion program feeds them, one new position batch per tick.
void run(Scene &scene, uint32_t ticks, uint32_t first_tick = 0) {
    for (uint32_t tick = first_tick; tick < first_tick + ticks; ++tick) {
        const double t = tick * 0.01;
        if (scene.arm) {
            const double targets[3] = {0.8 * std::sin(1.3 * t), 0.5 * std::sin(2.1 * t) - 0.2,
                                       0.6 * std::sin(0.9 * t)};
            const auto command = position_command(++scene.sequence, 3, targets);
            EXPECT(rk_robot_runtime_submit(scene.arm, &command) == RK_OK, "arm submit tick %u", tick);
        }
        if (scene.gantry) {
            const double targets[3] = {0.2 * std::sin(1.1 * t), 0.2 * std::sin(1.7 * t),
                                       0.1 * std::sin(2.3 * t) + 0.1};
            const auto command = position_command(++scene.sequence, 3, targets);
            EXPECT(rk_robot_runtime_submit(scene.gantry, &command) == RK_OK, "gantry submit tick %u", tick);
        }
        if (scene.humanoid && scene.policy) {
            const auto command = servo_hold(++scene.sequence, 4, 8.0, 0.5);
            (void)rk_robot_runtime_submit(scene.humanoid, &command);
        }
        scene.results.push_back(rk_simulation_step(scene.simulation,
                                                   static_cast<uint64_t>(tick) * 10'000'000u));
        if (scene.arm) record(scene, scene.arm_trace, scene.arm, scene.arm_index, scene.arm_links);
        if (scene.gantry)
            record(scene, scene.gantry_trace, scene.gantry, scene.gantry_index, scene.gantry_links);
    }
}

std::size_t count_failed(const Scene &scene) {
    std::size_t failed = 0;
    for (const auto result : scene.results) failed += result != RK_OK;
    return failed;
}

rk_robot_state snapshot(rk_robot_runtime robot) {
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    assert(rk_robot_runtime_snapshot(robot, &state) == RK_OK);
    return state;
}

double height_of(const Scene &scene, int robot) {
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(rk_simulation_get_robot_pose(scene.simulation, static_cast<uint32_t>(robot), &pose) == RK_OK);
    return pose.position[2];
}

constexpr uint32_t TICKS = 400;

} // namespace

// Baseline: the arm and the gantry with the humanoid absent.
static Trace baseline_arm, baseline_gantry;

// Timestep, integrator and solver limits belong to the world. With the
// humanoid added at the same settings, robots that never touch it run the same,
// whether it stands still or topples onto its joint stops.
static void far_humanoid_does_not_change_arm_and_gantry(bool humanoid_first, double pitch,
                                                        bool policy = false) {
    Scene alone;
    build(alone, {});
    run(alone, TICKS);
    EXPECT(count_failed(alone) == 0, "arm and gantry alone step cleanly");
    baseline_arm = alone.arm_trace;
    baseline_gantry = alone.gantry_trace;

    Options options;
    options.humanoid = true;
    options.humanoid_first = humanoid_first;
    options.humanoid_pitch = pitch;
    options.floor_box = true;
    options.humanoid_filter = 2;
    Scene mixed;
    build(mixed, options);
    mixed.policy = policy;
    run(mixed, TICKS);
    const auto humanoid = snapshot(mixed.humanoid);
    std::printf("far humanoid (%s, pitch %.1f, policy %d): arm max diff %.3g, gantry max diff %.3g, humanoid safety %d, failed steps %zu of %u\n",
                humanoid_first ? "first" : "last", pitch, policy, mixed.arm_trace.max_difference(baseline_arm),
                mixed.gantry_trace.max_difference(baseline_gantry), humanoid.safety,
                count_failed(mixed), TICKS);
    EXPECT(pitch == 0.0 || humanoid.safety == RK_SAFETY_FAULT, "the toppled humanoid latches its own fault");
    EXPECT(count_failed(mixed) == 0, "a humanoid fault fails %zu of %u session steps",
           count_failed(mixed), TICKS);
    EXPECT(mixed.arm_trace == baseline_arm, "arm trace changed by a far floating robot, max diff %.3g",
           mixed.arm_trace.max_difference(baseline_arm));
    EXPECT(mixed.gantry_trace == baseline_gantry,
           "gantry trace changed by a far floating robot, max diff %.3g",
           mixed.gantry_trace.max_difference(baseline_gantry));
    const auto arm = snapshot(mixed.arm), gantry = snapshot(mixed.gantry);
    EXPECT(arm.safety != RK_SAFETY_FAULT && gantry.safety != RK_SAFETY_FAULT,
           "arm safety %d gantry safety %d", arm.safety, gantry.safety);
}

// The humanoid alone, unpowered enough to fall: report how it ends.
[[maybe_unused]] static void probe_fall(uint32_t filter, double tolerance) {
    Options options;
    options.arm = options.gantry = false;
    options.humanoid = true;
    options.humanoid_x = 0.0;
    options.humanoid_pitch = 0.5;
    options.floor_box = true;
    options.humanoid_filter = filter;
    options.humanoid_tolerance = tolerance;
    Scene scene;
    build(scene, options);
    uint32_t first_failure = 0;
    for (uint32_t tick = 0; tick < 400; ++tick) {
        const auto result = rk_simulation_step(scene.simulation, tick * 10'000'000ull);
        if (result != RK_OK && first_failure == 0) {
            first_failure = tick + 1;
            std::printf("  step %u -> %d\n", tick, result);
        }
    }
    const auto state = snapshot(scene.humanoid);
    std::printf("humanoid alone filter %u tol %.2f: height %.3f safety %d mode %d first failure tick %u q = %.3f %.3f %.3f %.3f\n",
                filter, tolerance, height_of(scene, 0), state.safety, state.mode, first_failure,
                state.position[0], state.position[1], state.position[2], state.position[3]);
}

int main() {
    far_humanoid_does_not_change_arm_and_gantry(false, 0.0);
    far_humanoid_does_not_change_arm_and_gantry(true, 0.0);
    far_humanoid_does_not_change_arm_and_gantry(false, 0.5);
    far_humanoid_does_not_change_arm_and_gantry(true, 0.5);
    far_humanoid_does_not_change_arm_and_gantry(false, 0.5, true);
    far_humanoid_does_not_change_arm_and_gantry(true, 0.5, true);
    if (failures != 0) {
        std::printf("%d mixed-scene check(s) failed\n", failures);
        return 1;
    }
    std::puts("mixed-scene tests passed");
    return 0;
}
