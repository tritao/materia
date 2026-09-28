// Mixed-scene safety (robotkit/plans/MIXED_SCENE.md): robot arms, a CNC gantry
// and a floating-base legged robot share one Simulation on the MuJoCo backend
// without disturbing each other. Every check names the property it protects.
#include "robotkit_simkit.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
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
    bool bed = false;       // The gantry's base link is a 1 m x 1 m x 10 cm machine bed.
    double gantry_z = 0.5;
    double arm_position[3] = {0.0, 0.0, 1.0};
    double gantry_position[3] = {3.0, 0.0, 0.5};
    // Extra bodies, at a position, for contact probes.
    const double *static_box = nullptr, *dynamic_box = nullptr, *shapeless_robot = nullptr;
};

// One robot's observable trace: joint state and every link pose, each tick.
struct Trace {
    std::vector<double> values;
    bool operator==(const Trace &other) const {
        return values.size() == other.values.size() &&
            (values.empty() || std::memcmp(values.data(), other.values.data(),
                                           values.size() * sizeof(double)) == 0);
    }
    // Largest difference in joint positions and link poses, ignoring joint
    // velocities and efforts, which chatter at the level of the integrator.
    double pose_difference(const Trace &other, uint32_t joints, uint32_t links) const {
        const std::size_t record = joints * 3 + links * 7;
        double worst = 0.0;
        if (values.size() != other.values.size() || record == 0) return 1e300;
        for (std::size_t base = 0; base + record <= values.size(); base += record) {
            for (uint32_t joint = 0; joint < joints; ++joint)
                worst = std::fmax(worst, std::fabs(values[base + joint * 3] - other.values[base + joint * 3]));
            for (std::size_t i = joints * 3; i < record; ++i)
                worst = std::fmax(worst, std::fabs(values[base + i] - other.values[base + i]));
        }
        return worst;
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
    rk_simulation_object floor = 0, box = 0;
    std::vector<rk_result> results;
    Trace arm_trace, gantry_trace;
    uint64_t sequence = 0;
    bool policy = false; // The humanoid is sent a fresh servo batch every tick, like a policy.
    double timestep = 0.01;
    uint32_t stride = 1; // Record every stride-th tick.
    double arm_error = 0.0, gantry_error = 0.0; // Worst distance from the commanded position.
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
    scene.timestep = options.timestep;
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
        auto desc_arm = new_desc(options.arm_position[0], options.arm_position[1], options.arm_position[2]);
        scene.arm_index = next++;
        assert(rk_simulation_add_robot(scene.simulation, model.get(), desc_arm.get(),
                                       &scene.arm) == RK_OK);
    }
    if (options.gantry) {
        auto model = make_gantry();
        auto desc_gantry = new_desc(options.gantry_position[0], options.gantry_position[1],
                                    options.gantry_position[2]);
        scene.gantry_index = next++;
        if (options.bed) {
            desc_gantry->collision_half_extents[0] = desc_gantry->collision_half_extents[1] = 0.5;
            desc_gantry->collision_half_extents[2] = 0.05;
        }
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
    const auto spawn_box = [&](const double *at, uint32_t motion) {
        rk_simulation_object_desc box{};
        box.struct_size = sizeof(box);
        box.motion_type = motion;
        box.mass = motion == 2 ? 1.0 : 0.0;
        box.rotation[3] = 1.0;
        std::copy(at, at + 3, box.position);
        box.half_extents[0] = box.half_extents[1] = box.half_extents[2] = 0.05;
        rk_simulation_object object = 0;
        assert(rk_simulation_spawn_object(scene.simulation, &box, &object) == RK_OK);
        scene.box = object;
    };
    if (options.static_box) spawn_box(options.static_box, 0);
    if (options.dynamic_box) spawn_box(options.dynamic_box, 2);
    if (options.shapeless_robot) {
        // One link, no joints, no collision shape, and the "none" approximation:
        // a robot that is not meant to touch anything.
        auto model = new_blueprint(1, RK_COLLISION_APPROXIMATION_NONE);
        auto robot_desc = new_desc(options.shapeless_robot[0], options.shapeless_robot[1],
                                   options.shapeless_robot[2]);
        rk_robot_runtime ghost = 0;
        assert(rk_simulation_add_robot(scene.simulation, model.get(), robot_desc.get(), &ghost) == RK_OK);
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

rk_robot_state snapshot(rk_robot_runtime robot) {
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    assert(rk_robot_runtime_snapshot(robot, &state) == RK_OK);
    return state;
}

// Steps `ticks` times. The arm and the gantry follow a scripted job, the way a
// motion program feeds them, one new position batch per tick.
void run(Scene &scene, uint32_t ticks, uint32_t first_tick = 0) {
    for (uint32_t tick = first_tick; tick < first_tick + ticks; ++tick) {
        const double t = tick * scene.timestep;
        double arm_targets[3] = {}, gantry_targets[3] = {};
        if (scene.arm) {
            const double targets[3] = {0.8 * std::sin(1.3 * t), 0.5 * std::sin(2.1 * t) - 0.2,
                                       0.6 * std::sin(0.9 * t)};
            std::copy(targets, targets + 3, arm_targets);
            const auto command = position_command(++scene.sequence, 3, targets);
            EXPECT(rk_robot_runtime_submit(scene.arm, &command) == RK_OK, "arm submit tick %u", tick);
        }
        if (scene.gantry) {
            const double targets[3] = {0.2 * std::sin(1.1 * t), 0.2 * std::sin(1.7 * t),
                                       0.1 * std::sin(2.3 * t) + 0.1};
            std::copy(targets, targets + 3, gantry_targets);
            const auto command = position_command(++scene.sequence, 3, targets);
            EXPECT(rk_robot_runtime_submit(scene.gantry, &command) == RK_OK, "gantry submit tick %u", tick);
        }
        if (scene.humanoid && scene.policy) {
            const auto command = servo_hold(++scene.sequence, 4, 8.0, 0.5);
            (void)rk_robot_runtime_submit(scene.humanoid, &command);
        }
        scene.results.push_back(rk_simulation_step(scene.simulation,
                                                   static_cast<uint64_t>(tick) * 10'000'000u));
        if (scene.arm) {
            const auto state = snapshot(scene.arm);
            for (int joint = 0; joint < 3; ++joint)
                scene.arm_error = std::fmax(scene.arm_error, std::fabs(state.position[joint] - arm_targets[joint]));
        }
        if (scene.gantry) {
            const auto state = snapshot(scene.gantry);
            for (int joint = 0; joint < 3; ++joint)
                scene.gantry_error = std::fmax(scene.gantry_error,
                                               std::fabs(state.position[joint] - gantry_targets[joint]));
        }
        if ((tick + 1) % scene.stride != 0) continue;
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

// The world owns timestep, integrator and solver limits, so a scene that adds
// a humanoid at 2 ms with G1's solver settings runs the arm and the gantry at
// those settings too. They must stay valid, and a humanoid that does not touch
// them must not change them at a given setting.
static void world_settings_are_shared() {
    struct Config {
        const char *name;
        double timestep;
        uint32_t substeps, integrator, iterations, line_search;
    };
    const Config configs[] = {
        {"10 ms x5 euler (arm and CNC scenes)", 0.01, 5, 0, 0, 0},
        {"2 ms x2 implicitfast", 0.002, 2, 2, 0, 0},
        {"2 ms x2 implicitfast, 5 and 8 iterations (MJX G1)", 0.002, 2, 2, 5, 8},
    };
    Trace reference_arm, reference_gantry;
    for (const auto &config : configs) {
        Options options;
        options.timestep = config.timestep;
        options.substeps = config.substeps;
        options.integrator = config.integrator;
        options.solver_iterations = config.iterations;
        options.line_search_iterations = config.line_search;
        const auto ticks = static_cast<uint32_t>(std::lround(4.0 / config.timestep));
        const auto stride = static_cast<uint32_t>(std::lround(0.02 / config.timestep));
        Scene alone;
        build(alone, options);
        alone.stride = stride;
        run(alone, ticks);
        options.humanoid = true;
        options.humanoid_pitch = 0.5;
        options.floor_box = true;
        options.humanoid_filter = 2;
        Scene mixed;
        build(mixed, options);
        mixed.stride = stride;
        run(mixed, ticks);
        if (reference_arm.values.empty()) {
            reference_arm = alone.arm_trace;
            reference_gantry = alone.gantry_trace;
        }
        std::printf("%s: tracking error arm %.4f rad gantry %.5f m; vs 10 ms poses: arm %.4f gantry %.5f; humanoid changes them by %.3g\n",
                    config.name, alone.arm_error, alone.gantry_error,
                    alone.arm_trace.pose_difference(reference_arm, 3, 4),
                    alone.gantry_trace.pose_difference(reference_gantry, 3, 4),
                    std::fmax(mixed.arm_trace.max_difference(alone.arm_trace),
                              mixed.gantry_trace.max_difference(alone.gantry_trace)));
        EXPECT(count_failed(alone) == 0 && count_failed(mixed) == 0, "steps fail under %s", config.name);
        EXPECT(mixed.arm_trace == alone.arm_trace && mixed.gantry_trace == alone.gantry_trace,
               "a far humanoid changes the arm or gantry under %s", config.name);
    }
}


// What a standing humanoid's leg touches when something else overlaps it.
enum class Target { None, ArmLink, GantryLink, StaticBox, DynamicBox, ShapelessRobot };
const char *target_name(Target target) {
    switch (target) {
    case Target::None: return "nothing";
    case Target::ArmLink: return "arm link";
    case Target::GantryLink: return "gantry link";
    case Target::StaticBox: return "static box";
    case Target::DynamicBox: return "dynamic box";
    case Target::ShapelessRobot: return "shapeless none-approximation robot";
    }
    return "";
}

struct Touch {
    uint32_t contacts = 0;   // Active contacts of the leg links with anything but the floor.
    double displacement = 0; // How far the humanoid ended from where it ends with nothing there.
    double pose[7] = {};
};

Touch leg_touch(Target target, uint32_t filter, const Touch *reference) {
    // The shank capsule of the left leg is centred (hx, 0.1, 0.16) at rest.
    const double at[3] = {0.0, 0.18, 0.16};
    Options options;
    options.arm = target == Target::ArmLink;
    options.gantry = target == Target::GantryLink;
    options.humanoid = true;
    options.humanoid_first = true;
    options.humanoid_x = 0.0;
    options.humanoid_height = 0.68;
    options.humanoid_filter = filter;
    options.floor_box = true;
    std::copy(at, at + 3, options.arm_position);
    std::copy(at, at + 3, options.gantry_position);
    if (target == Target::StaticBox) options.static_box = at;
    if (target == Target::DynamicBox) options.dynamic_box = at;
    if (target == Target::ShapelessRobot) options.shapeless_robot = at;
    Scene scene;
    build(scene, options);
    Touch touch;
    for (uint32_t tick = 0; tick < 100; ++tick) {
        assert(rk_simulation_step(scene.simulation, tick * 10'000'000ull) == RK_OK);
        rk_robot_contact contacts[256]{};
        uint32_t count = 0;
        assert(rk_simulation_get_robot_contacts(scene.simulation, scene.humanoid, contacts, 256, &count) == RK_OK);
        uint32_t now = 0;
        for (uint32_t i = 0; i < count && i < 256; ++i)
            if (contacts[i].active && contacts[i].other_object != scene.floor && contacts[i].link_index >= 1)
                ++now;
        touch.contacts = std::max(touch.contacts, now);
    }
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(rk_simulation_get_robot_pose(scene.simulation, 0, &pose) == RK_OK);
    std::copy(pose.position, pose.position + 3, touch.pose);
    std::copy(pose.rotation, pose.rotation + 4, touch.pose + 3);
    if (reference)
        for (int i = 0; i < 3; ++i)
            touch.displacement = std::fmax(touch.displacement, std::fabs(touch.pose[i] - reference->pose[i]));
    return touch;
}

// Contact between a humanoid's leg shapes and what overlaps them follows the
// shapes' contact filter: by layers with everything, only through pairs (so
// with nothing outside the robot), or through pairs and with everything the
// layers allow outside the robot. A link that collides with nothing is never
// touched.
static void contacts_follow_the_contact_filter() {
    struct Row {
        Target target;
        bool touched[3]; // Filters 0, 1, 2.
    };
    const Row rows[] = {
        {Target::ArmLink, {true, false, true}},
        {Target::GantryLink, {true, false, true}},
        {Target::StaticBox, {true, false, true}},
        {Target::DynamicBox, {true, false, true}},
        {Target::ShapelessRobot, {false, false, false}},
    };
    for (const uint32_t filter : {0u, 1u, 2u}) {
        const auto reference = leg_touch(Target::None, filter, nullptr);
        EXPECT(reference.contacts == 0, "filter %u: a lone humanoid touches something", filter);
        for (const auto &row : rows) {
            const auto touch = leg_touch(row.target, filter, &reference);
            std::printf("filter %u leg vs %-36s: max contacts %u, humanoid moved %.4f m\n", filter,
                        target_name(row.target), touch.contacts, touch.displacement);
            EXPECT((touch.contacts > 0) == row.touched[filter],
                   "filter %u vs %s: %u contacts", filter, target_name(row.target), touch.contacts);
            EXPECT((touch.displacement > 1e-3) == row.touched[filter] ||
                       row.target == Target::DynamicBox,
                   "filter %u vs %s: humanoid moved %.4f m", filter, target_name(row.target),
                   touch.displacement);
        }
    }
}

// A humanoid dropped onto a machine bed, a link of a CNC robot, stands on it as
// it stands on the floor, and its feet do not pass through into the floor below.
static void humanoid_stands_on_a_machine_bed(uint32_t filter) {
    Options options;
    options.arm = false;
    options.humanoid = true;
    options.humanoid_first = true;
    options.humanoid_filter = filter;
    options.humanoid_x = 3.0;
    options.humanoid_height = 1.23; // Feet 2 cm above a bed top at 0.55 m.
    options.floor_box = true;       // The workshop floor, 0.55 m below the bed.
    options.bed = true;
    Scene scene;
    build(scene, options);
    for (uint32_t tick = 0; tick < 150; ++tick)
        assert(rk_simulation_step(scene.simulation, tick * 10'000'000ull) == RK_OK);
    const auto height = height_of(scene, scene.humanoid_index);
    std::printf("filter %u humanoid dropped on a machine bed: torso height %.3f (bed top 0.55, floor 0)\n",
                filter, height);
    // Standing on the bed puts the torso near 0.55 + 0.66; through it, near 0.66.
    EXPECT(filter == 1 ? height < 1.0 : height > 1.15,
           "filter %u: torso ended at %.3f m", filter, height);
}

// Robots step together and repeatably: every robot publishes from the same
// tick, the run does not depend on the order robots were added in, and the
// same scene run twice gives the same trace.
static void mixed_scene_steps_coherently_and_repeatably() {
    Options options;
    options.humanoid = true;
    options.humanoid_pitch = 0.5;
    options.floor_box = true;
    options.humanoid_filter = 2;
    Scene first, second;
    build(first, options);
    build(second, options);
    // Both sessions get the same policy stream; robots' clocks match every tick.
    first.policy = second.policy = true;
    bool clocks_match = true;
    uint64_t humanoid_stalled = 0;
    for (uint32_t tick = 0; tick < 200; ++tick) {
        run(first, 1, tick);
        run(second, 1, tick);
        const auto a = snapshot(first.arm), g = snapshot(first.gantry), h = snapshot(first.humanoid);
        clocks_match &= a.source_timestamp_ns == g.source_timestamp_ns && a.sequence == g.sequence;
        humanoid_stalled += h.source_timestamp_ns != a.source_timestamp_ns;
    }
    EXPECT(clocks_match, "arm and gantry publish from different ticks");
    EXPECT(first.arm_trace == second.arm_trace && first.gantry_trace == second.gantry_trace,
           "the same mixed scene run twice gave different traces");
    const auto h1 = snapshot(first.humanoid), h2 = snapshot(second.humanoid);
    EXPECT(std::memcmp(h1.position, h2.position, 4 * sizeof(double)) == 0 && h1.safety == h2.safety,
           "the humanoid is not repeatable");
    // A faulted robot's own sample stops while a joint is past its limit;
    // that is its own state and does not touch the others.
    std::printf("humanoid state lagged the session on %llu of 200 ticks after its fault\n",
                static_cast<unsigned long long>(humanoid_stalled));
}

// A realtime session keeps ticking when one robot faults.
static void realtime_session_survives_a_humanoid_fault() {
    Options options;
    options.humanoid = true;
    options.humanoid_pitch = 0.5;
    options.floor_box = true;
    options.humanoid_filter = 2;
    Scene scene;
    build(scene, options);
    const double targets[3] = {0.5, -0.4, 0.3};
    const auto command = position_command(1, 3, targets);
    assert(rk_robot_runtime_submit(scene.arm, &command) == RK_OK);
    assert(rk_simulation_start(scene.simulation) == RK_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    rk_simulation_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(rk_simulation_get_clock(scene.simulation, &clock) == RK_OK);
    const auto before = clock.step_index;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    assert(rk_simulation_get_clock(scene.simulation, &clock) == RK_OK);
    const auto humanoid = snapshot(scene.humanoid), arm = snapshot(scene.arm);
    std::printf("realtime: humanoid safety %d, session ticks %llu -> %llu, arm q0 %.3f\n", humanoid.safety,
                static_cast<unsigned long long>(before), static_cast<unsigned long long>(clock.step_index),
                arm.position[0]);
    EXPECT(humanoid.safety == RK_SAFETY_FAULT, "the toppled humanoid faults");
    EXPECT(clock.step_index >= before + 30, "the session stopped ticking after the fault: %llu -> %llu",
           static_cast<unsigned long long>(before), static_cast<unsigned long long>(clock.step_index));
    EXPECT(std::fabs(arm.position[0] - 0.5) < 0.05, "the arm is at %.3f", arm.position[0]);
    assert(rk_simulation_stop(scene.simulation) == RK_OK);
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
    mixed_scene_steps_coherently_and_repeatably();
    realtime_session_survives_a_humanoid_fault();
    contacts_follow_the_contact_filter();
    humanoid_stands_on_a_machine_bed(0);
    humanoid_stands_on_a_machine_bed(1);
    humanoid_stands_on_a_machine_bed(2);
    world_settings_are_shared();
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
