#include "simulation.hpp"
#include "virtual_device_endpoint.hpp"
#include "simulation_robot.hpp"
#include "runtime_registry.hpp"
#include "sensor_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace robotkit {
namespace {

bool valid_pose(const double position[3], const double rotation[4]);

bool valid_contact_surface(const double friction[3], uint32_t friction_dimensions,
                           double contact_time_constant, double contact_damping_ratio,
                           uint32_t contact_filter) {
    if (friction_dimensions != 0 && friction_dimensions != 1 && friction_dimensions != 3 &&
        friction_dimensions != 4 && friction_dimensions != 6)
        return false;
    for (uint32_t axis = 0; axis < 3; ++axis)
        if (!std::isfinite(friction[axis]) || friction[axis] < 0.0) return false;
    return std::isfinite(contact_time_constant) && contact_time_constant >= 0.0 &&
        std::isfinite(contact_damping_ratio) && contact_damping_ratio >= 0.0 &&
        contact_filter <= NKSIM_CONTACT_PAIRS_AND_ENVIRONMENT;
}

void require_sim(nksim_result result, const char *operation) {
    if (result != NKSIM_OK)
        throw std::runtime_error(operation);
}

void require_scene(nkscene_result result, const char *operation) {
    if (result != NKS_OK)
        throw std::runtime_error(operation);
}

/** A coupling the physics simulates as a spring: any stiffness. Only a stiffness of zero is an ideal constraint. */
bool spring_coupling(const rk_robot_runtime_blueprint &blueprint, uint32_t index) {
    return blueprint.coupling_stiffness[index] > 0.0;
}

nksim_shape create_collision_shape(nksim_world world, const rk_simulation_link_shape &source) {
    nksim_shape primitive = 0;
    switch (source.type) {
    case RK_LINK_SHAPE_BOX:
        require_sim(nksim_shape_create_box(world, source.size, &primitive),
                    "nksim_shape_create_box(link primitive)");
        break;
    case RK_LINK_SHAPE_SPHERE:
        require_sim(nksim_shape_create_sphere(world, source.size[0], &primitive),
                    "nksim_shape_create_sphere(link primitive)");
        break;
    case RK_LINK_SHAPE_CAPSULE:
        require_sim(nksim_shape_create_capsule(world, source.size[0],
            2.0 * source.size[1], &primitive), "nksim_shape_create_capsule(link primitive)");
        break;
    case RK_LINK_SHAPE_CONVEX:
        require_sim(nksim_shape_create_convex(world, source.vertices,
            source.vertex_count * 3, &primitive), "nksim_shape_create_convex(link shape)");
        break;
    default:
        require_sim(nksim_shape_create_cylinder(world, source.size[0],
            2.0 * source.size[1], &primitive), "nksim_shape_create_cylinder(link primitive)");
        break;
    }
    nksim_surface surface{};
    surface.struct_size = sizeof(surface);
    surface.friction_dimensions = source.friction_dimensions;
    std::copy_n(source.friction, 3, surface.friction);
    surface.contact_time_constant = source.contact_time_constant;
    surface.contact_damping_ratio = source.contact_damping_ratio;
    surface.contact_filter = source.contact_filter;
    require_sim(nksim_shape_set_surface(world, primitive, &surface),
                "nksim_shape_set_surface(link primitive)");
    return primitive;
}

bool valid_link_shape(const rk_simulation_link_shape &shape, uint32_t link_count) {
    if (shape.link >= link_count || shape.type < RK_LINK_SHAPE_BOX ||
        shape.type > RK_LINK_SHAPE_CONVEX || !valid_pose(shape.position, shape.rotation) ||
        !valid_contact_surface(shape.friction, shape.friction_dimensions,
            shape.contact_time_constant, shape.contact_damping_ratio, shape.contact_filter))
        return false;
    if (shape.type == RK_LINK_SHAPE_CONVEX) {
        if (shape.vertex_count < 4 || shape.vertex_count > 64) return false;
        for (uint32_t i = 0; i < shape.vertex_count * 3; ++i)
            if (!std::isfinite(shape.vertices[i])) return false;
        return true;
    }
    const uint32_t sized = shape.type == RK_LINK_SHAPE_BOX ? 3
        : shape.type == RK_LINK_SHAPE_SPHERE ? 1 : 2;
    for (uint32_t axis = 0; axis < sized; ++axis)
        if (!std::isfinite(shape.size[axis]) || shape.size[axis] <= 0.0) return false;
    return true;
}

// Minimal rigid-transform (translation + xyzw quaternion) helper for
// evaluating each link's rest pose. Mirrors robotkit.spatial.Transform3's
// compose/inverse exactly (ARCHITECTURE.md's a_T_b convention), so this is
// the C++ side of the same math, not a parallel convention.
struct Xform {
    double pos[3] = {0.0, 0.0, 0.0};
    double rot[4] = {0.0, 0.0, 0.0, 1.0};
};

void quat_rotate(const double q[4], const double v[3], double out[3]) {
    const double t[3] = {
        2.0 * (q[1] * v[2] - q[2] * v[1]),
        2.0 * (q[2] * v[0] - q[0] * v[2]),
        2.0 * (q[0] * v[1] - q[1] * v[0])
    };
    out[0] = v[0] + q[3] * t[0] + q[1] * t[2] - q[2] * t[1];
    out[1] = v[1] + q[3] * t[1] + q[2] * t[0] - q[0] * t[2];
    out[2] = v[2] + q[3] * t[2] + q[0] * t[1] - q[1] * t[0];
}

void quat_multiply(const double a[4], const double b[4], double out[4]) {
    out[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    out[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    out[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
    out[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
}

Xform xform_compose(const Xform &a, const Xform &b) {
    Xform result;
    double rotated[3];
    quat_rotate(a.rot, b.pos, rotated);
    for (int i = 0; i < 3; ++i) result.pos[i] = a.pos[i] + rotated[i];
    quat_multiply(a.rot, b.rot, result.rot);
    return result;
}

Xform xform_inverse(const Xform &a) {
    Xform result;
    result.rot[0] = -a.rot[0]; result.rot[1] = -a.rot[1];
    result.rot[2] = -a.rot[2]; result.rot[3] = a.rot[3];
    const double neg[3] = {-a.pos[0], -a.pos[1], -a.pos[2]};
    quat_rotate(result.rot, neg, result.pos);
    return result;
}

Xform xform_from(const double pos[3], const double rot[4]) {
    Xform x;
    std::copy_n(pos, 3, x.pos);
    std::copy_n(rot, 4, x.rot);
    return x;
}

nkscene_transform to_scene_transform(const Xform &x) {
    nkscene_transform transform{};
    for (int column = 0; column < 3; ++column) {
        double axis[3] = {0.0, 0.0, 0.0};
        axis[column] = 1.0;
        double rotated[3];
        quat_rotate(x.rot, axis, rotated);
        for (int row = 0; row < 3; ++row)
            transform.matrix[column * 4 + row] = static_cast<float>(rotated[row]);
    }
    transform.matrix[12] = static_cast<float>(x.pos[0]);
    transform.matrix[13] = static_cast<float>(x.pos[1]);
    transform.matrix[14] = static_cast<float>(x.pos[2]);
    transform.matrix[15] = 1.0f;
    return transform;
}

/**
 * Rest pose (world_T_link at q = 0) for every link, walking the joint tree
 * from the root. Per ARCHITECTURE.md's joint-frame convention:
 * world_T_child = world_T_parent . T(parent_frame_position, parent_frame_rotation)
 *               . T(child_frame_position, child_frame_rotation)^-1
 * (joint motion is identity at q = 0). robot_index offsets the whole robot
 * so multiple robots' links don't start stacked at one point either.
 */
std::vector<Xform> link_rest_poses(const rk_robot_runtime_blueprint &blueprint, uint32_t robot_index,
                                    uint32_t root) {
    std::vector<Xform> world(blueprint.link_count);
    std::vector<bool> resolved(blueprint.link_count, false);
    world[root].pos[0] = static_cast<double>(robot_index);
    resolved[root] = true;
    uint32_t remaining = blueprint.link_count > 0 ? blueprint.link_count - 1 : 0;
    for (uint32_t pass = 0; pass < blueprint.link_count && remaining > 0; ++pass) {
        for (uint32_t j = 0; j < blueprint.joint_count; ++j) {
            const auto &joint = blueprint.joints[j];
            if (resolved[joint.child_link] || !resolved[joint.parent_link])
                continue;
            const auto parent_T_jointFrame = xform_from(joint.parent_frame_position, joint.parent_frame_rotation);
            const auto child_T_jointFrame = xform_from(joint.child_frame_position, joint.child_frame_rotation);
            world[joint.child_link] = xform_compose(
                xform_compose(world[joint.parent_link], parent_T_jointFrame),
                xform_inverse(child_T_jointFrame));
            resolved[joint.child_link] = true;
            --remaining;
        }
    }
    if (remaining > 0)
        throw std::runtime_error("robot topology has unreachable links");
    return world;
}

rk_result from_sim(nksim_result result) {
    switch (result) {
    case NKSIM_OK: return RK_OK;
    case NKSIM_ERROR_INVALID_ARGUMENT: return RK_ERROR_INVALID_ARGUMENT;
    case NKSIM_ERROR_INVALID_HANDLE: return RK_ERROR_INVALID_HANDLE;
    case NKSIM_ERROR_INVALID_STATE: return RK_ERROR_INVALID_STATE;
    case NKSIM_ERROR_WRONG_THREAD: return RK_ERROR_INVALID_STATE;
    case NKSIM_ERROR_OUT_OF_MEMORY: return RK_ERROR_OUT_OF_MEMORY;
    default: return RK_ERROR_BACKEND;
    }
}

} // namespace

Simulation::Simulation(nksim_session session) : session_(session) {
    nksim_session_status status{};
    status.struct_size = sizeof(status);
    require_sim(nksim_session_get_status(session, &status), "nksim_session_get_status");
    fixed_timestep_ = status.fixed_timestep;
    std::copy_n(status.gravity, 3, gravity_);
    period_ = std::chrono::nanoseconds(static_cast<int64_t>(fixed_timestep_ * 1'000'000'000.0));
    try {
        attach();
    } catch (...) {
        cleanup();
        throw;
    }
}

void Simulation::attach() {
    require_sim(nksim_session_get_scene(session_, &scene_), "nksim_session_get_scene");
    Lock lock(session_);
    const auto world = stopped_world();
    if (world == 0) throw std::runtime_error("robots attach to a stopped session");
    require_sim(nksim_world_get_position_precision(world, &position_precision_), "nksim_world_get_position_precision");
    const double half_extents[] = {0.05, 0.05, 0.05};
    require_sim(nksim_shape_create_box(world, half_extents, &shape_), "nksim_shape_create_box");
    nksim_participant_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.user = this;
    // A failed callback records its RobotKit error for the caller of step().
    desc.prepare = [](void *user, const nksim_tick *tick) -> nksim_result {
        auto &self = *static_cast<Simulation *>(user);
        return (self.rejected_ = self.prepare(*tick)) == RK_OK ? NKSIM_OK : NKSIM_ERROR_BACKEND;
    };
    desc.discard = [](void *user) { static_cast<Simulation *>(user)->discard(); };
    desc.submit = [](void *user, const nksim_tick *tick) -> nksim_result {
        auto &self = *static_cast<Simulation *>(user);
        return (self.rejected_ = self.submit(*tick)) == RK_OK ? NKSIM_OK : NKSIM_ERROR_BACKEND;
    };
    desc.publish = [](void *user, const nksim_tick *tick) -> nksim_result {
        auto &self = *static_cast<Simulation *>(user);
        return (self.rejected_ = self.publish(*tick)) == RK_OK ? NKSIM_OK : NKSIM_ERROR_BACKEND;
    };
    desc.reset = [](void *user) -> nksim_result {
        auto &self = *static_cast<Simulation *>(user);
        return (self.rejected_ = self.reset_robots()) == RK_OK ? NKSIM_OK : NKSIM_ERROR_BACKEND;
    };
    require_sim(nksim_session_add_participant(session_, &desc, &participant_),
                "nksim_session_add_participant");
}

nksim_world Simulation::stopped_world() const {
    nksim_world world = 0;
    return nksim_session_get_world(session_, &world) == NKSIM_OK ? world : 0;
}

Simulation::~Simulation() {
    // No tick may call back into this object once teardown starts.
    if (participant_ != 0) nksim_session_remove_participant(session_, participant_);
    participant_ = 0;
    runtimes_.clear();
    for (const auto handle : handles_)
        internal::destroy_runtime(handle);
    handles_.clear();
    cleanup();
}

rk_result Simulation::add_robot(const rk_robot_runtime_blueprint &blueprint,
                                rk_robot_runtime &out_runtime,
                                const rk_simulation_robot_desc *robot_desc) {
    if (robot_desc && robot_desc->struct_size != sizeof(*robot_desc))
        return RK_ERROR_INVALID_ARGUMENT;
    if (rk_robot_runtime_blueprint_validate(&blueprint) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t i = 0; i < blueprint.brake_count; ++i) {
        const auto &brake = blueprint.brakes[i];
        if (!internal::brake_feedback_cadence_valid(blueprint.sensors[brake.released_sensor].update_rate,
                static_cast<uint64_t>(period_.count()), brake.feedback_max_age_ns))
            return RK_ERROR_INVALID_ARGUMENT;
    }
    // A zero-sized initial pose keeps the default placement, so a descriptor
    // can carry collision or device settings without choosing a pose.
    const auto *initial_pose = robot_desc && robot_desc->initial_pose.struct_size != 0
        ? &robot_desc->initial_pose : nullptr;
    Lock lock(session_);
    bool topology_update = false;
    nksim_session_status status{};
    status.struct_size = sizeof(status);
    if (nksim_session_get_status(session_, &status) != NKSIM_OK) return RK_ERROR_BACKEND;
    // The first tick seals the world layout for every participant.
    const auto world = status.sealed ? 0 : stopped_world();
    if (world == 0)
        return RK_ERROR_INVALID_STATE;
    if (rk_robot_runtime_blueprint_validate(&blueprint) != RK_OK || blueprint.link_count == 0)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto brake_count = robot_desc ? robot_desc->brake_count : 0;
    if (brake_count != blueprint.brake_count) return RK_ERROR_INVALID_ARGUMENT;
    if (brake_count && robot_desc->virtual_device_enabled) return RK_ERROR_UNSUPPORTED;
    for (uint32_t i = 0; i < brake_count; ++i) {
        const auto &brake = robot_desc->brakes[i];
        if (brake.joint >= blueprint.joint_count || brake.reserved ||
            !std::isfinite(brake.holding_effort) || brake.holding_effort <= 0.0 ||
            !std::isfinite(brake.release_delay) || brake.release_delay < 0.0 ||
            !std::isfinite(brake.application_delay) || brake.application_delay < 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
        bool matched = false;
        for (uint32_t j = 0; j < blueprint.brake_count; ++j)
            if (blueprint.brakes[j].joint == brake.joint) matched = true;
        if (!matched) return RK_ERROR_INVALID_ARGUMENT;
        for (uint32_t j = 0; j < i; ++j)
            if (robot_desc->brakes[j].joint == brake.joint) return RK_ERROR_INVALID_ARGUMENT;
    }
    const bool has_cylinder_desc = robot_desc != nullptr;
    const auto cylinder_count = has_cylinder_desc ? robot_desc->cylinder_drive_count : 0;
    if (cylinder_count > RK_MAX_JOINTS) return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < cylinder_count; ++index) {
        const auto &drive = robot_desc->cylinder_drives[index];
        if (drive.joint >= blueprint.joint_count ||
            blueprint.joints[drive.joint].type != RK_RUNTIME_JOINT_PRISMATIC ||
            !blueprint.external_joint[drive.joint] ||
            drive.channel_a >= blueprint.channel_count ||
            (drive.channel_b != UINT32_MAX && drive.channel_b >= blueprint.channel_count) ||
            drive.channel_a == drive.channel_b || drive.normally_to_a > 1 ||
            !std::isfinite(drive.extension_force) || drive.extension_force < 0.0 ||
            !std::isfinite(drive.retraction_force) || drive.retraction_force < 0.0 ||
            !std::isfinite(drive.extend_sign) ||
            (drive.extend_sign != 1.0 && drive.extend_sign != -1.0) ||
            !std::isfinite(drive.rated_speed) || drive.rated_speed <= 0.0 ||
            blueprint.channels[drive.channel_a].kind != RK_EVENT_DIGITAL ||
            (drive.channel_b != UINT32_MAX &&
             blueprint.channels[drive.channel_b].kind != RK_EVENT_DIGITAL))
            return RK_ERROR_INVALID_ARGUMENT;
        if (drive.control != RK_CYLINDER_PRESSURE_EFFORT && drive.control != RK_CYLINDER_FLOW_LIMITED)
            return RK_ERROR_INVALID_ARGUMENT;
        if (drive.control == RK_CYLINDER_FLOW_LIMITED &&
            (drive.channel_b == UINT32_MAX || !std::isfinite(drive.stroke) || drive.stroke <= 0.0 ||
             !std::isfinite(drive.approach_speed) || drive.approach_speed <= 0.0 ||
             !std::isfinite(drive.working_speed) || drive.working_speed <= 0.0 ||
             !std::isfinite(drive.return_speed) || drive.return_speed <= 0.0 ||
             !std::isfinite(drive.change_point) || drive.change_point < 0.0 || drive.change_point > drive.stroke ||
             !std::isfinite(drive.retracted_position) ||
             std::max({drive.approach_speed, drive.working_speed, drive.return_speed}) > drive.rated_speed))
            return RK_ERROR_INVALID_ARGUMENT;
        for (uint32_t earlier = 0; earlier < index; ++earlier)
            if (robot_desc->cylinder_drives[earlier].joint == drive.joint)
                return RK_ERROR_INVALID_ARGUMENT;
    }
    const bool has_velocity_desc = robot_desc != nullptr;
    const auto velocity_count = has_velocity_desc ? robot_desc->velocity_drive_count : 0;
    if (velocity_count > RK_MAX_JOINTS) return RK_ERROR_INVALID_ARGUMENT;
    for (uint32_t index = 0; index < velocity_count; ++index) {
        const auto &drive = robot_desc->velocity_drives[index];
        if (drive.joint >= blueprint.joint_count ||
            !blueprint.external_joint[drive.joint] ||
            drive.speed_channel >= blueprint.channel_count ||
            drive.direction_channel >= blueprint.channel_count ||
            drive.speed_channel == drive.direction_channel ||
            !std::isfinite(drive.radians_per_speed_unit) || drive.radians_per_speed_unit <= 0.0 ||
            !std::isfinite(drive.max_effort) || drive.max_effort < 0.0 ||
            !std::isfinite(drive.max_rate) || drive.max_rate <= 0.0 ||
            blueprint.channels[drive.speed_channel].kind != RK_EVENT_ANALOG ||
            blueprint.channels[drive.direction_channel].kind != RK_EVENT_ANALOG)
            return RK_ERROR_INVALID_ARGUMENT;
        for (uint32_t earlier = 0; earlier < index; ++earlier)
            if (robot_desc->velocity_drives[earlier].joint == drive.joint)
                return RK_ERROR_INVALID_ARGUMENT;
        for (uint32_t cylinder = 0; cylinder < cylinder_count; ++cylinder)
            if (robot_desc->cylinder_drives[cylinder].joint == drive.joint)
                return RK_ERROR_INVALID_ARGUMENT;
    }
    if (initial_pose && (initial_pose->struct_size < sizeof(*initial_pose) ||
                         !valid_pose(initial_pose->position, initial_pose->rotation)))
        return RK_ERROR_INVALID_ARGUMENT;
    std::shared_ptr<VirtualDeviceEndpoint> virtual_endpoint;
    std::vector<std::size_t> device_measured_joints;
    // How stiffly the device's actuators hold the joints they measure (zero: the default position loop).
    std::vector<double> device_hold_stiffness(blueprint.joint_count, 0.0), device_hold_damping(blueprint.joint_count, 0.0);
    if (robot_desc &&
        robot_desc->virtual_device_enabled) {
#if !defined(RK_HAS_VIRTUAL_DEVICE)
        // Built without the simulated board (RK_BUILD_VIRTUAL_DEVICE).
        return RK_ERROR_UNSUPPORTED;
#else
        VirtualDeviceConfig6 config;
        config.device_tick_hz = robot_desc->virtual_device_tick_hz;
        config.step_tick_hz = robot_desc->virtual_device_step_tick_hz;
        config.offset_ticks = robot_desc->virtual_device_offset_ticks;
        config.drift_ppm = robot_desc->virtual_device_drift_ppm;
        config.baud = robot_desc->virtual_device_baud;
        config.latency_ns = robot_desc->virtual_device_latency_ns;
        config.jitter_ns = robot_desc->virtual_device_jitter_ns;
        config.frame_drop_rate = robot_desc->virtual_device_drop_rate;
        config.corruption_rate = robot_desc->virtual_device_corruption_rate;
        config.seed = robot_desc->virtual_device_seed;
        // An all-zero id names no board, so a caller that leaves it unset gets the default.
        if (std::any_of(robot_desc->virtual_device_controller,
                robot_desc->virtual_device_controller + 16, [](auto byte) { return byte != 0; }))
            std::copy_n(robot_desc->virtual_device_controller, 16, config.controller.begin());
        config.steps_per_unit.assign(robot_desc->virtual_device_steps_per_unit,
            robot_desc->virtual_device_steps_per_unit + blueprint.joint_count);
        config.target_error = robot_desc->virtual_device_target_error;
        config.clock_bound_ns = robot_desc->virtual_device_clock_bound_ns;
        config.link_loss_timeout_ns = robot_desc->virtual_device_link_loss_timeout_ns;
        {
            if (robot_desc->virtual_peripheral_parameter_count > 64) return RK_ERROR_INVALID_ARGUMENT;
            config.peripheral_kind = robot_desc->virtual_peripheral_kind;
            config.external_sensor_mask = robot_desc->virtual_external_sensor_mask;
            config.peripheral_parameters.assign(robot_desc->virtual_peripheral_parameters,
                robot_desc->virtual_peripheral_parameters + robot_desc->virtual_peripheral_parameter_count);
        }
        if (robot_desc->struct_size >= offsetof(rk_simulation_robot_desc, virtual_device_profile) + sizeof(robot_desc->virtual_device_profile) && robot_desc->virtual_device_profile)
            config.profile = robot_desc->virtual_device_profile;
        if (robot_desc->virtual_device_actuator_count > 0) {
            if (robot_desc->virtual_device_actuator_count > 64) return RK_ERROR_INVALID_ARGUMENT;
            for (std::uint32_t i = 0; i < robot_desc->virtual_device_actuator_count; ++i) {
                DeviceActuator6 actuator;
                actuator.joint = robot_desc->virtual_device_actuator_joint[i];
                actuator.ratio = robot_desc->virtual_device_actuator_ratio[i];
                actuator.offset = robot_desc->virtual_device_actuator_offset[i];
                if (robot_desc->struct_size >= sizeof(*robot_desc) && robot_desc->virtual_device_feedback_count) {
                    if (robot_desc->virtual_device_feedback_count != robot_desc->virtual_device_actuator_count)
                        return RK_ERROR_INVALID_ARGUMENT;
                    actuator.feedback_joint = robot_desc->virtual_device_feedback_joint[i];
                    actuator.feedback_ratio = robot_desc->virtual_device_feedback_ratio[i];
                    actuator.feedback_offset = robot_desc->virtual_device_feedback_offset[i];
                }
                actuator.steps_per_unit = robot_desc->virtual_device_actuator_steps_per_unit[i];
                actuator.min_step_ticks = robot_desc->virtual_device_actuator_min_step_ticks[i];
                actuator.direction_setup_ticks =
                    robot_desc->virtual_device_actuator_direction_setup_ticks[i];
                actuator.dual_drive_skew_bound = robot_desc->virtual_device_actuator_skew_bound[i];
                const auto *id = robot_desc->virtual_device_actuator_ids + i * 64;
                const auto *end = std::find(id, id + 64, 0);
                if (end == id || end == id + 64) return RK_ERROR_INVALID_ARGUMENT;
                actuator.id.assign(reinterpret_cast<const char *>(id),
                    reinterpret_cast<const char *>(end));
                config.actuators.push_back(actuator);
            }
        }
        if (robot_desc->struct_size >= offsetof(rk_simulation_robot_desc, virtual_device_feedback_count)) {
            if (robot_desc->virtual_device_input_count > 64) return RK_ERROR_INVALID_ARGUMENT;
            for (std::uint32_t i = 0; i < robot_desc->virtual_device_input_count; ++i) {
                const auto *id = robot_desc->virtual_device_input_switch_ids + i * 64;
                const auto *end = std::find(id, id + 64, 0);
                if (end == id || end == id + 64 ||
                    robot_desc->virtual_device_input_active_high[i] > 1 ||
                    robot_desc->virtual_device_input_active_above[i] > 1) return RK_ERROR_INVALID_ARGUMENT;
                VirtualInputSwitch6 input;
                input.wiring.actuator = robot_desc->virtual_device_input_actuator[i];
                input.wiring.active_high = robot_desc->virtual_device_input_active_high[i] != 0;
                input.wiring.switch_id.assign(reinterpret_cast<const char *>(id), reinterpret_cast<const char *>(end));
                input.active_above = robot_desc->virtual_device_input_active_above[i] != 0;
                input.threshold_steps = robot_desc->virtual_device_input_threshold_steps[i];
                config.inputs.push_back(std::move(input));
            }
        }
        for (std::size_t index = 0; index < config.actuators.size(); ++index) {
            const auto joint = config.actuators[index].measured_joint();
            device_measured_joints.push_back(joint);
            const auto stiffness = robot_desc->virtual_device_actuator_hold_stiffness[index];
            const auto damping = robot_desc->virtual_device_actuator_hold_damping[index];
            if (!std::isfinite(stiffness) || stiffness < 0.0 || !std::isfinite(damping) || damping < 0.0)
                return RK_ERROR_INVALID_ARGUMENT;
            if (joint < blueprint.joint_count) {
                device_hold_stiffness[joint] = stiffness;
                device_hold_damping[joint] = damping;
            }
        }
        virtual_endpoint = VirtualDeviceEndpoint::create(blueprint, config);
        if (!virtual_endpoint) return RK_ERROR_INVALID_ARGUMENT;
#endif
    }
    try {
        auto binding = std::shared_ptr<SimulationRobot>(new SimulationRobot(*this));
        for (uint32_t index = 0; index < cylinder_count; ++index) {
            const auto &source = robot_desc->cylinder_drives[index];
            SimulationRobot::CylinderDriveState drive;
            drive.joint = source.joint;
            drive.control = source.control;
            drive.approach_speed = source.approach_speed;
            drive.working_speed = source.working_speed;
            drive.return_speed = source.return_speed;
            drive.change_point = source.change_point;
            drive.retracted_position = source.retracted_position;
            drive.channel_a = blueprint.channels[source.channel_a].id;
            drive.has_channel_b = source.channel_b != UINT32_MAX;
            if (drive.has_channel_b) drive.channel_b = blueprint.channels[source.channel_b].id;
            drive.normally_to_a = source.normally_to_a != 0;
            drive.to_a = drive.normally_to_a;
            drive.extension_force = source.extension_force;
            drive.retraction_force = source.retraction_force;
            drive.extend_sign = source.extend_sign;
            binding->cylinder_drives_.push_back(std::move(drive));
        }
        for (uint32_t index = 0; index < velocity_count; ++index) {
            const auto &source = robot_desc->velocity_drives[index];
            SimulationRobot::VelocityDriveState drive;
            drive.joint = source.joint;
            drive.speed_channel = blueprint.channels[source.speed_channel].id;
            drive.direction_channel = blueprint.channels[source.direction_channel].id;
            drive.radians_per_speed_unit = source.radians_per_speed_unit;
            drive.max_effort = source.max_effort;
            drive.max_rate = source.max_rate;
            binding->velocity_drives_.push_back(std::move(drive));
        }
        const auto robot_index = static_cast<uint32_t>(bindings_.size());
        uint32_t root = 0;
        for (uint32_t candidate = 0; candidate < blueprint.link_count; ++candidate) {
            bool child = false;
            for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint)
                child = child || blueprint.joints[joint].child_link == candidate;
            if (!child) { root = candidate; break; }
        }
        auto rest_poses = link_rest_poses(blueprint, robot_index, root);
        rk_simulation_pose chosen_initial{};
        chosen_initial.struct_size = sizeof(chosen_initial);
        chosen_initial.position[0] = static_cast<double>(robot_index);
        chosen_initial.rotation[3] = 1.0;
        if (initial_pose) {
            chosen_initial = *initial_pose;
            const auto authored_root = xform_from(rest_poses[root].pos, rest_poses[root].rot);
            const auto requested_root = xform_from(chosen_initial.position, chosen_initial.rotation);
            const auto root_delta = xform_compose(requested_root, xform_inverse(authored_root));
            for (auto &rest_pose : rest_poses)
                rest_pose = xform_compose(root_delta, rest_pose);
        }
        nkscene_transaction transaction = 0;
        require_scene(nkscene_transaction_begin(scene_, &transaction),
                      "nkscene_transaction_begin");
        for (uint32_t index = 0; index < blueprint.link_count; ++index) {
            nkscene_node_id node{};
            if (nkscene_tx_create_node(transaction, &node) != NKS_OK) {
                nkscene_transaction_cancel(transaction);
                throw std::runtime_error("nkscene_tx_create_node");
            }
            const auto transform = to_scene_transform(rest_poses[index]);
            if (nkscene_tx_set_transform(transaction, node, &transform) != NKS_OK) {
                nkscene_transaction_cancel(transaction);
                throw std::runtime_error("nkscene_tx_set_transform");
            }
            binding->nodes_.push_back(node);
            nodes_.push_back(node);
        }
        nkscene_change_set changes = 0;
        require_scene(nkscene_transaction_commit_with_changes(transaction, &changes),
                      "nkscene_transaction_commit_with_changes");
        if (changes != 0)
            nkscene_change_set_destroy(changes);

        require_sim(nksim_world_begin_topology_update(world),
                    "nksim_world_begin_topology_update");
        topology_update = true;
        const bool has_tool = robot_desc &&
            robot_desc->tool_shape_count > 0;
        if (has_tool && (robot_desc->tool_link_index >= blueprint.link_count ||
                         robot_desc->tool_shape_count > 16))
            throw std::invalid_argument("invalid tool attachment link or piece count");
        const auto link_shape_count = robot_desc
            ? robot_desc->link_shape_count : 0;
        if (link_shape_count > RK_MAX_LINK_SHAPES)
            throw std::invalid_argument("too many link collision shapes");
        for (uint32_t index = 0; index < link_shape_count; ++index)
            if (!valid_link_shape(robot_desc->link_shapes[index], blueprint.link_count))
                throw std::invalid_argument("invalid link collision shape");
        const auto contact_pair_count = robot_desc
            ? robot_desc->contact_pair_count : 0;
        if (contact_pair_count > RK_MAX_CONTACT_PAIRS)
            throw std::invalid_argument("too many contact pairs");
        for (uint32_t index = 0; index < contact_pair_count; ++index) {
            const auto &pair = robot_desc->contact_pairs[index];
            if (pair.shape_a >= link_shape_count || pair.shape_b >= link_shape_count ||
                robot_desc->link_shapes[pair.shape_a].link == robot_desc->link_shapes[pair.shape_b].link)
                throw std::invalid_argument("invalid contact pair");
        }
        const auto link_hull_count = robot_desc
            ? robot_desc->link_hull_count : 0;
        if (link_hull_count > RK_MAX_LINK_HULLS)
            throw std::invalid_argument("too many link hulls");
        for (uint32_t index = 0; index < link_hull_count; ++index) {
            const auto &hull = robot_desc->link_hulls[index];
            if (hull.link >= blueprint.link_count || hull.vertex_count < 4 || hull.vertex_count > 64)
                throw std::invalid_argument("invalid link hull");
            if (!valid_contact_surface(hull.friction, hull.friction_dimensions,
                    hull.contact_time_constant, hull.contact_damping_ratio, hull.contact_filter))
                throw std::invalid_argument("invalid link hull contact surface");
            for (uint32_t coordinate = 0; coordinate < hull.vertex_count * 3; ++coordinate)
                if (!std::isfinite(hull.vertices[coordinate]))
                    throw std::invalid_argument("link hull vertex is not finite");
        }
        // The compound part each link shape becomes, for its contact pairs.
        std::vector<uint32_t> shape_parts(link_shape_count, 0);
        for (uint32_t index = 0; index < blueprint.link_count; ++index) {
            nksim_body_desc desc{};
            desc.struct_size = sizeof(desc);
            desc.node = binding->nodes_[index];
            desc.motion_type = index == root && !blueprint.floating_base
                ? NKSIM_MOTION_KINEMATIC : NKSIM_MOTION_DYNAMIC;
            desc.mass = blueprint.links[index].mass;
            desc.has_inertial_properties = 1;
            std::copy_n(blueprint.links[index].center_of_mass, 3, desc.center_of_mass);
            std::copy_n(blueprint.links[index].inertia_tensor, 9, desc.inertia_tensor);
            desc.shape = shape_;
            desc.rest_contact_group = robot_desc ? robot_desc->rest_contact_group : 0;
            bool has_link_shape = robot_desc;
            const auto hull_count = robot_desc
                ? robot_desc->collision_hull_count[index] : 0;
            if (hull_count != 0 && (hull_count < 4 || hull_count > 64))
                throw std::invalid_argument("invalid link collision hull vertex count");
            if (hull_count > 0) {
                nksim_shape link_shape = 0;
                const auto *vertices = robot_desc->collision_hull_vertices + index * 64 * 3;
                require_sim(nksim_shape_create_convex(world, vertices, hull_count * 3,
                                                      &link_shape), "nksim_shape_create_convex(link)");
                link_shapes_.push_back(link_shape);
                desc.shape = link_shape;
                has_link_shape = true;
            } else if (has_link_shape) {
                double extents[3]{};
                for (int axis = 0; axis < 3; ++axis)
                    extents[axis] = robot_desc->collision_half_extents[index * 3 + axis];
                has_link_shape = extents[0] > 0.0 && extents[1] > 0.0 && extents[2] > 0.0;
                if (has_link_shape) {
                    nksim_shape link_shape = 0;
                    require_sim(nksim_shape_create_box(world, extents, &link_shape),
                                "nksim_shape_create_box(link)");
                    link_shapes_.push_back(link_shape);
                    desc.shape = link_shape;
                }
            }
            // A link with primitive shapes or a tool collides through one
            // compound of its explicit hull or box, the primitives, and the
            // tool pieces, each posed in the link frame.
            std::vector<nksim_shape> children;
            std::vector<nksim_shape_pose> poses;
            const auto add_child = [&](nksim_shape child, const double position[3],
                                       const double rotation[4]) {
                nksim_shape_pose pose{};
                std::copy_n(position, 3, pose.position);
                std::copy_n(rotation, 4, pose.rotation);
                children.push_back(child);
                poses.push_back(pose);
            };
            constexpr double origin[3] = {0.0, 0.0, 0.0};
            constexpr double identity[4] = {0.0, 0.0, 0.0, 1.0};
            // A link made of several parts carries one hull per part, already in
            // the link frame.
            for (uint32_t hull_index = 0; hull_index < link_hull_count; ++hull_index) {
                const auto &hull = robot_desc->link_hulls[hull_index];
                if (hull.link != index) continue;
                if (children.empty() && has_link_shape) add_child(desc.shape, origin, identity);
                nksim_shape piece = 0;
                require_sim(nksim_shape_create_convex(world, hull.vertices, hull.vertex_count * 3, &piece),
                            "nksim_shape_create_convex(link hull)");
                nksim_surface surface{};
                surface.struct_size = sizeof(surface);
                std::copy_n(hull.friction, 3, surface.friction);
                surface.friction_dimensions = hull.friction_dimensions;
                surface.contact_time_constant = hull.contact_time_constant;
                surface.contact_damping_ratio = hull.contact_damping_ratio;
                surface.contact_filter = hull.contact_filter;
                require_sim(nksim_shape_set_surface(world, piece, &surface),
                            "nksim_shape_set_surface(link hull)");
                link_shapes_.push_back(piece);
                add_child(piece, origin, identity);
                has_link_shape = true;
            }
            for (uint32_t shape_index = 0; shape_index < link_shape_count; ++shape_index) {
                const auto &source = robot_desc->link_shapes[shape_index];
                if (source.link != index) continue;
                if (children.empty() && has_link_shape) add_child(desc.shape, origin, identity);
                nksim_shape primitive = create_collision_shape(world, source);
                link_shapes_.push_back(primitive);
                shape_parts[shape_index] = static_cast<uint32_t>(children.size());
                add_child(primitive, source.position, source.rotation);
                has_link_shape = true;
            }
            if (has_tool && index == robot_desc->tool_link_index) {
                if (!std::isfinite(robot_desc->tool_margin) ||
                    robot_desc->tool_margin < 0.0)
                    throw std::invalid_argument("invalid tool contact margin or gap");
                if (children.empty()) add_child(desc.shape, origin, identity);
                binding->tool_link_ = index;
                binding->tool_part_start_ = static_cast<uint32_t>(children.size());
                binding->tool_part_count_ = robot_desc->tool_shape_count;
                for (uint32_t piece = 0; piece < robot_desc->tool_shape_count; ++piece) {
                    const auto &source = robot_desc->tool_shapes[piece];
                    if (!valid_link_shape(source, blueprint.link_count) || source.link != index ||
                        !std::isfinite(robot_desc->tool_shape_gap[piece]) || robot_desc->tool_shape_gap[piece] < 0)
                        throw std::invalid_argument("invalid tool collision shape");
                    nksim_shape tool_shape = create_collision_shape(world, source);
                    require_sim(nksim_shape_set_contact(world, tool_shape,
                        robot_desc->tool_margin, robot_desc->tool_shape_gap[piece]), "nksim_shape_set_contact(tool)");
                    link_shapes_.push_back(tool_shape);
                    add_child(tool_shape, source.position, source.rotation);
                }
                has_link_shape = true;
            }
            if (!children.empty()) {
                nksim_shape compound = 0;
                require_sim(nksim_shape_create_compound(world, children.data(), poses.data(),
                    static_cast<uint32_t>(children.size()), &compound),
                    "nksim_shape_create_compound(link)");
                link_shapes_.push_back(compound);
                desc.shape = compound;
            }
            // Layer 2 is an opt-out category for a robot's own links. It
            // still collides with ordinary layer-1 environment geometry,
            // while two opt-out links do not collide with one another.
            // A link with no shape under the "none" approximation collides
            // with nothing. Clearing only its mask is not enough: a contact
            // forms when either body's layer meets the other's mask, so the
            // environment's own mask would still catch it.
            const bool self_collision_disabled = blueprint.self_collision == RK_SELF_COLLISION_DISABLED;
            const bool collides = has_link_shape ||
                blueprint.collision_approximation != RK_COLLISION_APPROXIMATION_NONE;
            desc.collision_layer = !collides ? 0 : self_collision_disabled ? 2 : 1;
            desc.collision_mask = collides ? 1 : 0;
            nksim_body body = 0;
            require_sim(nksim_body_create(world, &desc, &body), "nksim_body_create");
            binding->bodies_.push_back(body);
            bodies_.push_back(body);
        }
        for (uint32_t index = 0; index < blueprint.joint_count; ++index) {
            const auto &source = blueprint.joints[index];
            nksim_joint_desc desc{};
            desc.struct_size = sizeof(desc);
            desc.type = source.type;
            desc.body_a = binding->bodies_[source.parent_link];
            desc.body_b = binding->bodies_[source.child_link];
            std::copy_n(source.parent_frame_position, 3, desc.anchor_a);
            std::copy_n(source.child_frame_position, 3, desc.anchor_b);
            const auto *q = source.parent_frame_rotation;
            const auto *a = source.axis;
            const double t[3] = {
                2.0 * (q[1]*a[2] - q[2]*a[1]),
                2.0 * (q[2]*a[0] - q[0]*a[2]),
                2.0 * (q[0]*a[1] - q[1]*a[0])
            };
            desc.axis_a[0] = a[0] + q[3]*t[0] + q[1]*t[2] - q[2]*t[1];
            desc.axis_a[1] = a[1] + q[3]*t[1] + q[2]*t[0] - q[0]*t[2];
            desc.axis_a[2] = a[2] + q[3]*t[2] + q[0]*t[1] - q[1]*t[0];
            std::copy_n(source.parent_frame_rotation, 4, desc.rotation_a);
            std::copy_n(source.child_frame_rotation, 4, desc.rotation_b);
            // A cylinder's overtravel is its compressible end cushion: keep
            // the physics stop at the stroke boundary while the runtime tolerates
            // that small penetration in the observed joint position.
            const double overtravel = blueprint.external_joint[index] ? 0.0 : blueprint.joint_overtravel[index];
            desc.lower_limit = source.lower_limit - overtravel;
            desc.upper_limit = source.upper_limit + overtravel;
            // SimKit uses zero for an unspecified force ceiling. A negative
            // sentinel preserves an explicit zero-effort RobotKit limit.
            desc.max_force = (source.limit_flags & RK_LIMIT_EFFORT) && source.max_effort == 0.0
                ? -1.0 : source.max_effort;
            {
                const auto &dynamics = blueprint.joint_dynamics[index];
                desc.armature = dynamics.armature;
                desc.damping = dynamics.damping;
                desc.friction_loss = dynamics.friction_loss;
                desc.limit_time_constant = dynamics.limit_time_constant;
                desc.limit_damping_ratio = dynamics.limit_damping_ratio;
                std::copy_n(dynamics.limit_impedance, 5, desc.limit_impedance);
            }
            for (uint32_t drive_index = 0; drive_index < cylinder_count; ++drive_index) {
                const auto &drive = robot_desc->cylinder_drives[drive_index];
                if (drive.joint != index) continue;
                desc.max_force = std::max(desc.max_force,
                    std::max(drive.extension_force, drive.retraction_force));
                if (drive.control == RK_CYLINDER_PRESSURE_EFFORT)
                    desc.damping += std::max(drive.extension_force, drive.retraction_force) /
                        drive.rated_speed;
            }
            nksim_joint joint = 0;
            require_sim(nksim_joint_create(world, &desc, &joint), "nksim_joint_create");
            binding->joints_.push_back(joint);
            const bool external_joint = blueprint.external_joint[index] != 0;
            binding->actuated_joints_.push_back(source.type != RK_RUNTIME_JOINT_FIXED && !external_joint);
            joints_.push_back(joint);
        }
        for (uint32_t i = 0; i < brake_count; ++i) {
            const auto &source = robot_desc->brakes[i];
            require_sim(nksim_joint_configure_brake(world, binding->joints_[source.joint],
                source.holding_effort, source.release_delay, source.application_delay), "nksim_joint_configure_brake");
            for (uint32_t j = 0; j < blueprint.brake_count; ++j)
                if (blueprint.brakes[j].joint == source.joint)
                    binding->brakes_.push_back({source.joint, blueprint.channels[blueprint.brakes[j].release_channel].id});
        }
        // A process drive may command its joint before any controller joint
        // target has staged a command. Keep its applied state from the first tick.
        binding->commanded_.resize(blueprint.joint_count);
        // Parallel powered drive shafts need independent targets for squaring.
        // An unpowered idler following the same axis is not a second drive. Keep
        // ordinary single-output transmissions physically constrained so their
        // load and shaft cannot drift apart while tracking a moving target.
        std::vector<uint8_t> independent_coupling(blueprint.joint_count, 0);
        for (uint32_t i = 0; i < blueprint.coupling_count; ++i)
            for (uint32_t k = 0; k < i; ++k)
                if (blueprint.couplings[i].leader == blueprint.couplings[k].leader &&
                    blueprint.couplings[i].follower != blueprint.couplings[k].follower &&
                    (blueprint.joints[blueprint.couplings[i].follower].limit_flags & RK_LIMIT_EFFORT) &&
                    blueprint.joints[blueprint.couplings[i].follower].max_effort > 0.0 &&
                    (blueprint.joints[blueprint.couplings[k].follower].limit_flags & RK_LIMIT_EFFORT) &&
                    blueprint.joints[blueprint.couplings[k].follower].max_effort > 0.0)
                    independent_coupling[blueprint.couplings[i].leader] = 1;
        for (uint32_t pass = 0; pass < blueprint.joint_count; ++pass) {
            bool changed = false;
            for (uint32_t i = 0; i < blueprint.coupling_count; ++i) {
                const auto &term = blueprint.couplings[i];
                if (independent_coupling[term.leader] || independent_coupling[term.follower]) {
                    changed |= !independent_coupling[term.leader] || !independent_coupling[term.follower];
                    independent_coupling[term.leader] = independent_coupling[term.follower] = 1;
                }
            }
            if (!changed) break;
        }
        std::vector<uint8_t> physical_coupling(blueprint.joint_count, 0);
        for (uint32_t i = 0; i < blueprint.joint_count; ++i)
            physical_coupling[i] = !independent_coupling[i];
        if (blueprint.struct_size >= offsetof(rk_robot_runtime_blueprint, joint_servo) +
                sizeof(blueprint.joint_servo))
            for (uint32_t i = 0; i < blueprint.joint_count; ++i)
                physical_coupling[i] |= blueprint.joint_servo[i].stiffness > 0.0;
        // Multiple-input mechanisms keep their physical summed constraint.
        for (uint32_t i = 0; i < blueprint.coupling_count; ++i)
            for (uint32_t k = 0; k < i; ++k)
                if (blueprint.couplings[i].follower == blueprint.couplings[k].follower)
                    physical_coupling[blueprint.couplings[i].follower] = 1;
        for (uint32_t pass = 0; pass < blueprint.joint_count; ++pass) {
            bool changed = false;
            for (uint32_t i = 0; i < blueprint.coupling_count; ++i) {
                const auto &term = blueprint.couplings[i];
                if (physical_coupling[term.leader] || physical_coupling[term.follower]) {
                    changed |= !physical_coupling[term.leader] || !physical_coupling[term.follower];
                    physical_coupling[term.leader] = physical_coupling[term.follower] = 1;
                }
            }
            if (!changed) break;
        }
        binding->joint_brake_state_.assign(blueprint.joint_count, SimulationRobot::BrakeContact{});
        binding->brake_group_.assign(blueprint.joint_count, 0);
        for (const auto &brake : binding->brakes_) binding->brake_group_[brake.joint] = 1;
        bool expanded = true;
        while (expanded) {
            expanded = false;
            for (uint32_t i = 0; i < blueprint.coupling_count; ++i) {
                const auto &coupling = blueprint.couplings[i];
                if (!(binding->brake_group_[coupling.leader] || binding->brake_group_[coupling.follower])) continue;
                if (!binding->brake_group_[coupling.leader] || !binding->brake_group_[coupling.follower]) expanded = true;
                binding->brake_group_[coupling.leader] = binding->brake_group_[coupling.follower] = 1;
            }
        }
        for (uint32_t index = 0; index < blueprint.coupling_count; ++index) {
            const auto &source = blueprint.couplings[index];
            if (!physical_coupling[source.follower]) {
                binding->kinematic_couplings_.push_back(source);
                continue;
            }
            nksim_joint_coupling_desc coupling{};
            coupling.struct_size = sizeof(coupling);
            coupling.leader = binding->joints_[source.leader];
            coupling.follower = binding->joints_[source.follower];
            coupling.ratio = source.ratio;
            coupling.offset = source.offset;
            coupling.stiffness = blueprint.coupling_stiffness[index];
            coupling.damping_ratio = blueprint.coupling_damping_ratio[index];
            require_sim(nksim_joint_couple(world, &coupling), "nksim_joint_couple");
        }
        for (uint32_t index = 0; index < contact_pair_count; ++index) {
            const auto &source = robot_desc->contact_pairs[index];
            nksim_contact_pair_desc pair{};
            pair.struct_size = sizeof(pair);
            pair.body_a = binding->bodies_[robot_desc->link_shapes[source.shape_a].link];
            pair.part_a = shape_parts[source.shape_a];
            pair.body_b = binding->bodies_[robot_desc->link_shapes[source.shape_b].link];
            pair.part_b = shape_parts[source.shape_b];
            pair.surface.struct_size = sizeof(pair.surface);
            pair.surface.friction_dimensions = source.friction_dimensions;
            std::copy_n(source.friction, 3, pair.surface.friction);
            pair.surface.contact_time_constant = source.contact_time_constant;
            pair.surface.contact_damping_ratio = source.contact_damping_ratio;
            require_sim(nksim_contact_pair_create(world, &pair), "nksim_contact_pair_create");
        }
        const auto closure_count = robot_desc
            ? robot_desc->closure_count : 0;
        if (closure_count > 64) throw std::invalid_argument("too many assembly closures");
        for (uint32_t index = 0; index < closure_count; ++index) {
            const auto &source = robot_desc->closures[index];
            if (source.parent_link >= blueprint.link_count ||
                source.child_link >= blueprint.link_count)
                throw std::invalid_argument("assembly closure references an invalid link");
            nksim_closure_desc closure{};
            closure.struct_size = sizeof(closure);
            closure.type = source.type;
            closure.body_a = binding->bodies_[source.parent_link];
            closure.body_b = binding->bodies_[source.child_link];
            std::copy_n(source.anchor_parent, 3, closure.anchor_a);
            std::copy_n(source.axis_parent, 3, closure.axis_a);
            require_sim(nksim_closure_create(world, &closure), "nksim_closure_create");
        }
        // Servo motor joints (see rk_robot_joint_servo) carry the joints coupled to them, which then
        // take no commands of their own.
        binding->slip_.assign(blueprint.joint_count, 0.0);
        binding->squaring_offset_.assign(blueprint.joint_count, 0.0);
        binding->counter_origin_.assign(blueprint.joint_count, 0.0);
        binding->squaring_hold_.assign(blueprint.joint_count, 0);
        binding->squaring_position_.assign(blueprint.joint_count, 0.0);
        binding->servo_.assign(blueprint.joint_count, rk_robot_joint_servo{});
        binding->servo_feedforward_.assign(blueprint.joint_count,0.0);

        binding->passive_.assign(blueprint.joint_count, 0);
        binding->step_ = fixed_timestep_;
        {
            std::copy_n(blueprint.joint_servo, blueprint.joint_count, binding->servo_.begin());
            std::vector<uint32_t> group(blueprint.joint_count);
            for (uint32_t index = 0; index < blueprint.joint_count; ++index) group[index] = index;
            const auto find = [&group](uint32_t joint) {
                while (group[joint] != joint) joint = group[joint] = group[group[joint]];
                return joint;
            };
            for (uint32_t index = 0; index < blueprint.coupling_count; ++index)
                group[find(blueprint.couplings[index].leader)] = find(blueprint.couplings[index].follower);
            std::vector<uint8_t> driven(blueprint.joint_count, 0);
            for (uint32_t index = 0; index < blueprint.joint_count; ++index)
                if (binding->servo_[index].stiffness > 0.0) driven[find(index)] = 1;
            for (uint32_t index = 0; index < blueprint.coupling_count; ++index)
                for (const auto joint : {blueprint.couplings[index].leader, blueprint.couplings[index].follower})
                    if (driven[find(joint)] && binding->servo_[joint].stiffness <= 0.0)
                        binding->passive_[joint] = 1;
        }
        if (robot_desc && (robot_desc->flags & RK_SIMULATION_ROBOT_EXPLICIT_EXCLUDES)) {
            if (robot_desc->struct_size < offsetof(rk_simulation_robot_desc, link_excludes) +
                    sizeof(robot_desc->link_excludes) ||
                robot_desc->link_exclude_count > RK_MAX_LINK_EXCLUDES)
                throw std::invalid_argument("explicit link excludes need the descriptor's exclude tail");
            std::vector<uint32_t> pairs;
            for (uint32_t index = 0; index < robot_desc->link_exclude_count; ++index) {
                const auto a = robot_desc->link_excludes[2 * index], b = robot_desc->link_excludes[2 * index + 1];
                if (a >= blueprint.link_count || b >= blueprint.link_count || a == b)
                    throw std::invalid_argument("invalid link exclude");
                pairs.push_back(a);
                pairs.push_back(b);
            }
            require_sim(nksim_world_set_exclusions(world, binding->bodies_.data(), blueprint.link_count,
                                                   pairs.data(), uint32_t(pairs.size())),
                        "nksim_world_set_exclusions");
        }
        const auto topology_result = nksim_world_end_topology_update(world);
        topology_update = false;
        require_sim(topology_result, "nksim_world_end_topology_update");
        binding->device_free_.assign(blueprint.joint_count, 0);
        binding->device_previous_.assign(blueprint.joint_count, std::nullopt);
        binding->device_hold_stiffness_ = std::move(device_hold_stiffness);
        binding->device_hold_damping_ = std::move(device_hold_damping);
        uint32_t world_features = 0;
        require_sim(nksim_world_get_features(world, &world_features), "nksim_world_get_features");
        if (virtual_endpoint && robot_desc && (world_features & NKSIM_WORLD_FEATURE_COMPLIANT_COUPLINGS)) {
            // A device drives the joints its actuators measure. The rest of a spring coupling is left to the physics,
            // where the physics simulates springs: a backend with ideal couplings sets a follower from its leaders, so
            // there the device keeps driving every joint it reconstructs.
            std::vector<uint8_t> measured(blueprint.joint_count, 0);
            for (const auto joint : device_measured_joints)
                if (joint < blueprint.joint_count) measured[joint] = 1;
            for (uint32_t index = 0; index < blueprint.coupling_count; ++index) {
                if (!spring_coupling(blueprint, index)) continue;
                for (const auto joint : {blueprint.couplings[index].leader, blueprint.couplings[index].follower})
                    if (!measured[joint]) binding->device_free_[joint] = 1;
            }
            // The load's rigid partners (the idlers that turn with the carriage) are free with it.
            for (bool changed = true; changed;) {
                changed = false;
                for (uint32_t index = 0; index < blueprint.coupling_count; ++index) {
                    const auto leader = blueprint.couplings[index].leader, follower = blueprint.couplings[index].follower;
                    if (binding->device_free_[leader] == binding->device_free_[follower]) continue;
                    const auto other = binding->device_free_[leader] ? follower : leader;
                    if (measured[other]) continue;
                    binding->device_free_[other] = 1;
                    changed = true;
                }
            }
        }
        if (robot_desc && (robot_desc->flags & RK_SIMULATION_ROBOT_HOLD_AT_REST)) {
            std::vector<uint8_t> held(blueprint.joint_count, 0);
            for (uint32_t index = 0; index < blueprint.joint_count; ++index)
                held[index] = binding->actuated_joints_[index];
            for (uint32_t index = 0; index < blueprint.coupling_count; ++index)
                held[blueprint.couplings[index].follower] = 0;
            // A servo motor is held at rest by its servo, wherever it sits in its couplings.
            for (uint32_t index = 0; index < blueprint.joint_count; ++index) {
                if (binding->servo_[index].stiffness > 0.0) held[index] = binding->actuated_joints_[index];
                // An applied brake holds its joint mechanically until explicitly released.
                if (binding->passive_[index] || binding->device_free_[index] || binding->brake_group_[index]) held[index] = 0;
            }
            for (uint32_t index = 0; index < blueprint.joint_count; ++index)
                    if (blueprint.external_joint[index]) held[index] = 0;
            binding->hold_at_rest(std::move(held));
        }
        bindings_.push_back(binding);
        virtual_bindings_.push_back(virtual_endpoint ? binding : nullptr);
        virtual_devices_.push_back(virtual_endpoint);
        binding->base_body_ = binding->bodies_[root];
        robot_base_bodies_.push_back(binding->base_body_);
        robot_floating_.push_back(blueprint.floating_base != 0);
        robot_initial_poses_.push_back(chosen_initial);
        robot_base_poses_.push_back(chosen_initial);
        robot_tick_poses_.push_back(chosen_initial);
        drives_.emplace_back();
        for (uint32_t i = 0; i < (blueprint.sensor_count ? blueprint.sensor_count : 3); ++i) {
            SimulationRobot::SensorState sensor;
            if (blueprint.sensor_count) sensor.config = blueprint.sensors[i];
            else {
                sensor.config.kind = i + 1;
                sensor.config.link = root;
                sensor.config.rotation[3] = 1.0;
                sensor.config.ray_count = 8;
                sensor.config.max_range = 10.0;
            }
            {
                sensor.joint = blueprint.sensor_joint[i];
                sensor.window_lower = blueprint.sensor_window_lower[i];
                sensor.window_upper = blueprint.sensor_window_upper[i];
                sensor.hysteresis = blueprint.sensor_hysteresis[i];
            }
            if (sensor.config.kind == RK_SENSOR_AT_SPEED) {
                // Resolve the drive's live setpoint into the measured shaft coordinate once.
                // A sum coupling cannot be inverted from one drive's command.
                struct Reference { uint32_t joint; double scale; };
                std::vector<Reference> pending{{sensor.joint, 1.0}};
                std::vector<bool> visited(blueprint.joint_count, false);
                for (std::size_t next = 0; next < pending.size(); ++next) {
                    const auto reference = pending[next];
                    if (reference.joint >= visited.size() || visited[reference.joint]) continue;
                    visited[reference.joint] = true;
                    for (const auto &drive : binding->velocity_drives_)
                        if (drive.joint == reference.joint) {
                            if (sensor.speed_reference_joint != UINT32_MAX)
                                throw std::invalid_argument("At-speed sensor has multiple drive references");
                            sensor.speed_reference_joint = reference.joint;
                            sensor.speed_reference_scale = reference.scale;
                        }
                    for (uint32_t coupling = 0; coupling < blueprint.coupling_count; ++coupling) {
                        const auto &relation = blueprint.couplings[coupling];
                        uint32_t terms = 0;
                        for (uint32_t term = 0; term < blueprint.coupling_count; ++term)
                            if (blueprint.couplings[term].follower == relation.follower) ++terms;
                        if (terms != 1 || relation.ratio == 0.0) continue;
                        if (relation.leader == reference.joint)
                            pending.push_back({relation.follower, reference.scale / relation.ratio});
                        else if (relation.follower == reference.joint)
                            pending.push_back({relation.leader, reference.scale * relation.ratio});
                    }
                }
                // Direct velocity commands also supply a live reference without a process drive.
                if (sensor.speed_reference_joint == UINT32_MAX) sensor.speed_reference_joint = sensor.joint;
            }
            binding->sensors_.push_back(sensor);
        }
        binding->reset_sensors();

        auto runtime = std::make_shared<RobotRuntime>(blueprint,
            virtual_endpoint ? std::static_pointer_cast<RobotEndpoint>(virtual_endpoint)
                             : std::static_pointer_cast<RobotEndpoint>(binding), period_);
        runtime->set_externally_driven(true);
        const auto handle = internal::register_runtime(runtime);
        runtimes_.push_back(std::move(runtime));
        handles_.push_back(handle);
        out_runtime = handle;
        return RK_OK;
    } catch (const std::bad_alloc &) {
        if (topology_update)
            (void)nksim_world_end_topology_update(world);
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        if (topology_update)
            (void)nksim_world_end_topology_update(world);
        return RK_ERROR_BACKEND;
    }
}

namespace { rk_result set_body_pose(nksim_world,nksim_body,const double[3],const double[4]); }

rk_result Simulation::reset_robots() {
    const auto world = stopped_world();
    if (world == 0) return RK_ERROR_INVALID_STATE;
    for (std::size_t index = 0; index < runtimes_.size(); ++index) {
        if(set_body_pose(world,robot_base_bodies_[index],robot_initial_poses_[index].position,
            robot_initial_poses_[index].rotation)!=RK_OK)return RK_ERROR_BACKEND;
        // Kinematic bases follow their scene node on every tick, so a pose
        // driven by drive_robot_base() or a drive plant is restored there too.
        drives_[index].yaw = 0.0;
        std::fill(std::begin(drives_[index].rates), std::end(drives_[index].rates), 0.0);
        if (set_robot_base_node_pose(static_cast<uint32_t>(index),
                                     robot_initial_poses_[index]) != RK_OK)
            return RK_ERROR_BACKEND;
        if (virtual_devices_[index] && !virtual_devices_[index]->reset()) return RK_ERROR_BACKEND;
        if (auto binding = bindings_[index].lock()) binding->reset();
        runtimes_[index]->reset_state();
    }
    return RK_OK;
}

rk_result Simulation::set_power_up_offsets(uint32_t robot_index, const double *offsets, uint32_t count,
                                         const uint32_t *side_drives, uint32_t side_count) {
    Lock lock(session_);
    const auto world = stopped_world();
    if (!world || !offsets || robot_index >= bindings_.size()) return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    if (std::find(virtual_bindings_.begin(), virtual_bindings_.end(), binding) != virtual_bindings_.end())
        return RK_ERROR_UNSUPPORTED;
    const auto &blueprint = runtimes_[robot_index]->blueprint();
    if (count != blueprint.joint_count) return RK_ERROR_INVALID_ARGUMENT;
    if (side_count > count || (side_count != 0 && !side_drives)) return RK_ERROR_INVALID_ARGUMENT;
    std::array<bool, RK_MAX_JOINTS> independent_sides{};
    for (uint32_t i = 0; i < side_count; ++i) {
        const auto joint = side_drives[i];
        if (joint >= count || independent_sides[joint] || !binding->actuated_joints_[joint] ||
            binding->passive_[joint]) return RK_ERROR_INVALID_ARGUMENT;
        uint32_t terms = 0;
        for (uint32_t k = 0; k < blueprint.coupling_count; ++k)
            if (blueprint.couplings[k].follower == joint) ++terms;
        if (terms != 1) return RK_ERROR_INVALID_ARGUMENT;
        independent_sides[joint] = true;
    }
    rk_robot_state observed{};
    runtimes_[robot_index]->snapshot(observed);
    if (observed.sequence != 0) return RK_ERROR_INVALID_STATE;
    std::array<nksim_joint_state, RK_MAX_JOINTS> before{};
    for (uint32_t joint = 0; joint < count; ++joint) {
        if (!std::isfinite(offsets[joint]) || binding->counter_origin_[joint] != 0.0 ||
            binding->slip_[joint] != 0.0 || binding->squaring_offset_[joint] != 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
        if (blueprint.joints[joint].type == RK_RUNTIME_JOINT_FIXED) {
            if (offsets[joint] != 0.0) return RK_ERROR_INVALID_ARGUMENT;
            continue;
        }
        before[joint].struct_size = sizeof(before[joint]);
        if (nksim_joint_get_state(world, binding->joints_[joint], &before[joint]) != NKSIM_OK)
            return RK_ERROR_BACKEND;
        const double position = before[joint].position + offsets[joint];
        const auto &limits = blueprint.joints[joint];
        if (!std::isfinite(position) || position < limits.lower_limit || position > limits.upper_limit)
            return RK_ERROR_LIMIT;
    }
    for (uint32_t joint = 0; joint < count; ++joint) {
        bool coupled = false; double expected = 0.0;
        for (uint32_t k = 0; k < blueprint.coupling_count; ++k)
            if (blueprint.couplings[k].follower == joint) {
                coupled = true;
                expected += blueprint.couplings[k].ratio * offsets[blueprint.couplings[k].leader];
            }
        if (coupled && !independent_sides[joint] &&
            (!std::isfinite(expected) || std::abs(expected - offsets[joint]) > 1e-6))
            return RK_ERROR_INVALID_ARGUMENT;
    }
    for (uint32_t joint = 0; joint < count; ++joint) {
        if (blueprint.joints[joint].type == RK_RUNTIME_JOINT_FIXED) continue;
        if (nksim_joint_set_state(world, binding->joints_[joint],
                before[joint].position + offsets[joint], 0.0) != NKSIM_OK) {
            for (uint32_t restore = 0; restore <= joint; ++restore)
                if (blueprint.joints[restore].type != RK_RUNTIME_JOINT_FIXED)
                    nksim_joint_set_state(world, binding->joints_[restore],
                        before[restore].position, before[restore].velocity);
            return RK_ERROR_BACKEND;
        }
    }
    std::copy_n(offsets, count, binding->counter_origin_.begin());
    // Replace initial holds with physical holds at the offset pose.
    binding->pending_targets_.clear();
    binding->queue_rest_holds();
    return RK_OK;
}

rk_result Simulation::set_coupling_law(uint32_t robot_index, const rk_simulation_coupling_law_desc &desc) {
    Lock lock(session_);
    if (robot_index >= bindings_.size() || desc.struct_size != sizeof(desc) ||
        desc.spring_count == 0 || desc.spring_count > NKSIM_MAX_COMPLIANCE_SPRINGS) return RK_ERROR_INVALID_ARGUMENT;
    const auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    if (desc.leader >= binding->joints_.size() || desc.follower >= binding->joints_.size()) return RK_ERROR_INVALID_ARGUMENT;
    const auto world = stopped_world();
    if (!world) return RK_ERROR_INVALID_STATE;
    nksim_coupling_law_desc law{};
    law.struct_size = sizeof(law);
    law.leader = binding->joints_[desc.leader]; law.follower = binding->joints_[desc.follower];
    law.spring_count = desc.spring_count;
    for (uint32_t spring = 0; spring < law.spring_count; ++spring) {
        if (desc.coordinate_count[spring] > NKSIM_MAX_COMPLIANCE_COORDINATES) return RK_ERROR_INVALID_ARGUMENT;
        law.compliance[spring] = desc.compliance[spring]; law.coordinate_count[spring] = desc.coordinate_count[spring];
        for (uint32_t c = 0; c < law.coordinate_count[spring]; ++c) {
            const auto index = spring * NKSIM_MAX_COMPLIANCE_COORDINATES + c;
            if (desc.coordinate_joint[index] >= binding->joints_.size()) return RK_ERROR_INVALID_ARGUMENT;
            law.coordinate_joint[index] = binding->joints_[desc.coordinate_joint[index]];
            law.gradient[index] = desc.gradient[index];
        }
    }
    // Independent fan-out drives are represented by kinematic targets, not a
    // backend transmission. Like their constant stiffness, a constitutive law
    // has no physical response until the coupling is simulated physically.
    for (const auto &coupling : binding->kinematic_couplings_)
        if (coupling.leader == desc.leader && coupling.follower == desc.follower)
            return RK_OK;
    uint32_t features = 0;
    if (nksim_world_get_features(world, &features) != NKSIM_OK) return RK_ERROR_BACKEND;
    // The reference backend is deliberately kinematic: it ignores physical spring responses, including variable ones.
    if (!(features & NKSIM_WORLD_FEATURE_COMPLIANT_COUPLINGS)) return RK_OK;
    return from_sim(nksim_joint_set_coupling_law(world, &law));
}

rk_result Simulation::set_joint_slip(uint32_t robot_index, uint32_t joint, double offset) {
    // The tick lock orders this between completed host steps, like a base drive.
    Lock lock(session_);
    if (robot_index >= bindings_.size() || !std::isfinite(offset))
        return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding)
        return RK_ERROR_INVALID_HANDLE;
    if (joint >= binding->slip_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    binding->slip_[joint] = offset;
    return RK_OK;
}

rk_result Simulation::set_squaring_hold(uint32_t robot_index, uint32_t joint, bool active,
                                       double position) {
    Lock lock(session_);
    if (robot_index >= bindings_.size() || !std::isfinite(position))
        return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    if (std::find(virtual_bindings_.begin(), virtual_bindings_.end(), binding) != virtual_bindings_.end())
        return RK_ERROR_UNSUPPORTED;
    if (joint >= binding->squaring_hold_.size() || !binding->actuated_joints_[joint] ||
        binding->passive_[joint]) return RK_ERROR_INVALID_ARGUMENT;
    binding->squaring_position_[joint] = position;
    binding->squaring_hold_[joint] = active ? 1 : 0;
    // Releasing preserves alignment separately from subsequent lost-step slip.
    return RK_OK;
}

rk_result Simulation::reset_robot(uint32_t robot_index) {
    Lock lock(session_);
    const auto world = stopped_world();
    if (world == 0 || robot_index >= bindings_.size())
        return RK_ERROR_INVALID_STATE;
    auto binding = bindings_[robot_index].lock();
    if (!binding)
        return RK_ERROR_INVALID_HANDLE;
    for (auto body : binding->bodies_)
        if (nksim_body_reset(world, body) != NKSIM_OK)
            return RK_ERROR_BACKEND;
    if(set_body_pose(world,robot_base_bodies_[robot_index],robot_initial_poses_[robot_index].position,
        robot_initial_poses_[robot_index].rotation)!=RK_OK)return RK_ERROR_BACKEND;
    auto &drive = drives_[robot_index];
    drive.yaw = 0.0;
    std::fill(std::begin(drive.rates), std::end(drive.rates), 0.0);
    if (set_robot_base_node_pose(robot_index, robot_initial_poses_[robot_index]) != RK_OK)
        return RK_ERROR_BACKEND;
    if (virtual_devices_[robot_index] && !virtual_devices_[robot_index]->reset()) return RK_ERROR_BACKEND;
    binding->reset();
    runtimes_[robot_index]->reset_state();
    return RK_OK;
}

namespace {

bool valid_pose(const double position[3], const double rotation[4]) {
    if (!position || !rotation)
        return false;
    for (int index = 0; index < 3; ++index)
        if (!std::isfinite(position[index])) return false;
    for (int index = 0; index < 4; ++index)
        if (!std::isfinite(rotation[index])) return false;
    double norm = 0.0;
    for (int index = 0; index < 4; ++index) norm += rotation[index] * rotation[index];
    return std::abs(norm - 1.0) < 1e-6;
}

rk_result set_body_pose(nksim_world world, nksim_body body, const double position[3],
                        const double rotation[4]) {
    if (!valid_pose(position, rotation))
        return RK_ERROR_INVALID_ARGUMENT;
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    if (nksim_body_get_state(world, body, &state) != NKSIM_OK)
        return RK_ERROR_INVALID_HANDLE;
    std::copy(position, position + 3, state.position);
    std::copy(rotation, rotation + 4, state.rotation);
    std::fill(std::begin(state.linear_velocity), std::end(state.linear_velocity), 0.0);
    std::fill(std::begin(state.angular_velocity), std::end(state.angular_velocity), 0.0);
    state.sleeping = 1;
    return nksim_body_set_state(world, body, &state) == NKSIM_OK
        ? RK_OK : RK_ERROR_BACKEND;
}

rk_result set_node_pose(nkscene_scene scene, nkscene_node_id node,
                              const double position[3], const double rotation[4]) {
    nkscene_transform transform{};
    for (int column = 0; column < 3; ++column) {
        double axis[3]{};
        axis[column] = 1.0;
        double rotated[3];
        sensors::rotate(rotation, axis, rotated);
        for (int row = 0; row < 3; ++row)
            transform.matrix[column * 4 + row] = static_cast<float>(rotated[row]);
    }
    transform.matrix[12] = static_cast<float>(position[0]);
    transform.matrix[13] = static_cast<float>(position[1]);
    transform.matrix[14] = static_cast<float>(position[2]);
    transform.matrix[15] = 1.0f;
    nkscene_transaction transaction = 0;
    if (nkscene_transaction_begin(scene, &transaction) != NKS_OK) return RK_ERROR_BACKEND;
    if (nkscene_tx_set_transform(transaction, node, &transform) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        return RK_ERROR_BACKEND;
    }
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
        return RK_ERROR_BACKEND;
    if (changes != 0) nkscene_change_set_destroy(changes);
    return RK_OK;
}

} // namespace

namespace {

// Heading of the body x axis projected on the floor.
double yaw_of(const double rotation[4]) {
    const double x = rotation[0], y = rotation[1], z = rotation[2], w = rotation[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void yaw_rotation(double yaw, double out[4]) {
    out[0] = out[1] = 0.0;
    out[2] = std::sin(yaw * 0.5);
    out[3] = std::cos(yaw * 0.5);
}

// World-frame angular velocity that rotates `from` onto `to` in `dt`, the
// shorter way around.
void angular_velocity_between(const double from[4], const double to[4], double dt,
                              double out[3]) {
    const double inverse[4] = {-from[0], -from[1], -from[2], from[3]};
    double relative[4];
    sensors::multiply(to, inverse, relative);
    if (relative[3] < 0.0)
        for (double &component : relative) component = -component;
    const double sine = std::sqrt(relative[0] * relative[0] + relative[1] * relative[1] +
                                  relative[2] * relative[2]);
    const double scale = sine > 1e-12 ? 2.0 * std::atan2(sine, relative[3]) / sine : 2.0;
    for (int axis = 0; axis < 3; ++axis) out[axis] = relative[axis] * scale / dt;
}

bool valid_drive_geometry(double value) {
    return std::isfinite(value) && value > 0.0;
}

} // namespace

rk_result Simulation::write_robot_base_node(uint32_t robot_index,
                                            const rk_simulation_pose &pose) {
    if (robot_index >= bindings_.size() || robot_index >= robot_base_bodies_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    if (!valid_pose(pose.position, pose.rotation)) return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    const auto root = std::find(binding->bodies_.begin(), binding->bodies_.end(),
                                robot_base_bodies_[robot_index]);
    if (root == binding->bodies_.end()) return RK_ERROR_INVALID_HANDLE;
    const auto root_index = static_cast<std::size_t>(root - binding->bodies_.begin());
    const auto result = set_node_pose(scene_, binding->nodes_[root_index], pose.position,
                                      pose.rotation);
    if (result == RK_OK) robot_base_poses_[robot_index] = pose;
    return result;
}

rk_result Simulation::set_robot_base_node_pose(uint32_t robot_index,
                                               const rk_simulation_pose &pose) {
    const auto result = write_robot_base_node(robot_index, pose);
    if (result != RK_OK) return result;
    robot_tick_poses_[robot_index] = pose;
    seed_drive(robot_index, pose);
    return RK_OK;
}

void Simulation::seed_drive(uint32_t robot_index, const rk_simulation_pose &pose) {
    auto &drive = drives_[robot_index];
    drive.x = pose.position[0];
    drive.y = pose.position[1];
    drive.height = pose.position[2];
    // Keep the plant heading continuous: choose the turn count nearest to the
    // previous unwrapped heading so a re-seed at the same pose is a no-op.
    constexpr double tau = 6.283185307179586;
    const double yaw = yaw_of(pose.rotation);
    drive.yaw = yaw + tau * std::round((drive.yaw - yaw) / tau);
    double unyaw[4];
    yaw_rotation(-yaw, unyaw);
    sensors::multiply(unyaw, pose.rotation, drive.tilt);
    double norm = 0.0;
    for (const double component : drive.tilt) norm += component * component;
    norm = std::sqrt(norm);
    for (double &component : drive.tilt) component /= norm;
}

rk_result Simulation::drive_robot_base_body(uint32_t robot_index, const rk_simulation_pose &pose,
                                            const double linear_velocity[3],
                                            const double angular_velocity[3]) {
    const auto result = write_robot_base_node(robot_index, pose);
    if (result != RK_OK) return result;
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    state.body = robot_base_bodies_[robot_index];
    std::copy_n(pose.position, 3, state.position);
    std::copy_n(pose.rotation, 4, state.rotation);
    std::copy_n(linear_velocity, 3, state.linear_velocity);
    std::copy_n(angular_velocity, 3, state.angular_velocity);
    const auto driven = nksim_session_drive_bodies(session_, &state, 1);
    return driven == NKSIM_OK ? RK_OK : RK_ERROR_BACKEND;
}

rk_result Simulation::advance_drives() {
    for (uint32_t index = 0; index < drives_.size(); ++index) {
        auto &drive = drives_[index];
        if (drive.kind == DrivePlant::Kind::None) continue;
        const auto binding = bindings_[index].lock();
        if (!binding) return RK_ERROR_INVALID_HANDLE;
        for (uint32_t wheel = 0; wheel < drive.wheel_count; ++wheel)
            drive.rates[wheel] = binding->applied_velocity(drive.joints[wheel]);
        // Body twist (forward, lateral, yaw rate) decoded from the applied rates.
        double forward = 0.0, lateral = 0.0, turn_rate = 0.0;
        if (drive.kind == DrivePlant::Kind::Differential) {
            const double left = drive.rates[0] * drive.directions[0] * drive.wheel_radius;
            const double right = drive.rates[1] * drive.directions[1] * drive.wheel_radius;
            forward = (left + right) * 0.5;
            turn_rate = (right - left) / drive.track_width;
        } else {
            double body[3] = {0.0, 0.0, 0.0};
            for (int row = 0; row < 3; ++row)
                for (int wheel = 0; wheel < 3; ++wheel)
                    body[row] += drive.inverse[row][wheel] * drive.rates[wheel] * drive.wheel_radius;
            forward = body[0];
            lateral = body[1];
            turn_rate = body[2];
        }
        // A stopped plant sends nothing: the base holds its pose at rest.
        if (forward == 0.0 && lateral == 0.0 && turn_rate == 0.0) continue;
        // Exact motion under a constant body twist over the tick: the twist
        // integrated along the turning heading (an arc when lateral is zero).
        const double turn = turn_rate * fixed_timestep_;
        double along = forward * fixed_timestep_, across = lateral * fixed_timestep_;
        if (std::abs(turn) >= 1e-9) {
            const double sine = std::sin(turn), versine = 1.0 - std::cos(turn);
            along = (forward * sine - lateral * versine) / turn_rate;
            across = (forward * versine + lateral * sine) / turn_rate;
        }
        drive.x += along * std::cos(drive.yaw) - across * std::sin(drive.yaw);
        drive.y += along * std::sin(drive.yaw) + across * std::cos(drive.yaw);
        drive.yaw += turn;
        // World-frame twist at the end of the tick.
        const double linear[3] = {forward * std::cos(drive.yaw) - lateral * std::sin(drive.yaw),
                                  forward * std::sin(drive.yaw) + lateral * std::cos(drive.yaw),
                                  0.0};
        // The wheels roll on the level floor, so the base translates in the
        // world XY plane at its seeded height and turns about world Z; its
        // authored roll and pitch (the tilt) turn with the heading.
        rk_simulation_pose pose{};
        pose.struct_size = sizeof(pose);
        pose.position[0] = drive.x;
        pose.position[1] = drive.y;
        pose.position[2] = drive.height;
        double heading[4];
        yaw_rotation(drive.yaw, heading);
        sensors::multiply(heading, drive.tilt, pose.rotation);
        const double angular[3] = {0.0, 0.0, turn_rate};
        const auto result = drive_robot_base_body(index, pose, linear, angular);
        if (result != RK_OK) return result;
    }
    return RK_OK;
}

rk_result Simulation::set_differential_drive(
    uint32_t robot_index, const rk_simulation_differential_drive_desc &desc) {
    Lock lock(session_);
    if (desc.struct_size < sizeof(desc) || robot_index >= drives_.size() ||
        !valid_drive_geometry(desc.wheel_radius) || !valid_drive_geometry(desc.track_width) ||
        desc.left_wheel_joint == desc.right_wheel_joint ||
        (desc.reversed_wheels & ~(RK_DRIVE_REVERSED_LEFT | RK_DRIVE_REVERSED_RIGHT)) != 0)
        return RK_ERROR_INVALID_ARGUMENT;
    if (robot_floating_[robot_index]) return RK_ERROR_INVALID_STATE;
    const uint32_t joints[2] = {desc.left_wheel_joint, desc.right_wheel_joint};
    const auto valid = valid_wheel_joints(robot_index, joints, 2);
    if (valid != RK_OK) return valid;
    auto &drive = drives_[robot_index];
    drive.kind = DrivePlant::Kind::Differential;
    drive.wheel_count = 2;
    std::copy_n(joints, 2, drive.joints);
    drive.wheel_radius = desc.wheel_radius;
    drive.track_width = desc.track_width;
    drive.directions[0] = (desc.reversed_wheels & RK_DRIVE_REVERSED_LEFT) != 0 ? -1.0 : 1.0;
    drive.directions[1] = (desc.reversed_wheels & RK_DRIVE_REVERSED_RIGHT) != 0 ? -1.0 : 1.0;
    std::fill(std::begin(drive.rates), std::end(drive.rates), 0.0);
    return set_robot_base_node_pose(robot_index, robot_base_poses_[robot_index]);
}

rk_result Simulation::set_omni_drive(uint32_t robot_index,
                                     const rk_simulation_omni_drive_desc &desc) {
    Lock lock(session_);
    if (desc.struct_size < sizeof(desc) || robot_index >= drives_.size() ||
        !valid_drive_geometry(desc.wheel_radius) || !valid_drive_geometry(desc.base_radius))
        return RK_ERROR_INVALID_ARGUMENT;
    if (robot_floating_[robot_index]) return RK_ERROR_INVALID_STATE;
    for (const double angle : desc.wheel_angles)
        if (!std::isfinite(angle)) return RK_ERROR_INVALID_ARGUMENT;
    const auto &j = desc.wheel_joints;
    if (j[0] == j[1] || j[0] == j[2] || j[1] == j[2]) return RK_ERROR_INVALID_ARGUMENT;
    const auto valid = valid_wheel_joints(robot_index, j, 3);
    if (valid != RK_OK) return valid;
    // Wheel i's rim speed is -sin(a_i) vx + cos(a_i) vy + base_radius * omega
    // for a body twist (vx, vy, omega); invert that map once here.
    double forward[3][3];
    for (int wheel = 0; wheel < 3; ++wheel) {
        forward[wheel][0] = -std::sin(desc.wheel_angles[wheel]);
        forward[wheel][1] = std::cos(desc.wheel_angles[wheel]);
        forward[wheel][2] = desc.base_radius;
    }
    const double determinant =
        forward[0][0] * (forward[1][1] * forward[2][2] - forward[1][2] * forward[2][1]) -
        forward[0][1] * (forward[1][0] * forward[2][2] - forward[1][2] * forward[2][0]) +
        forward[0][2] * (forward[1][0] * forward[2][1] - forward[1][1] * forward[2][0]);
    // The wheels must span every planar motion (no two parallel, not all
    // through one point); scale the check by base_radius, the matrix's units.
    if (!(std::abs(determinant) > 1e-6 * desc.base_radius)) return RK_ERROR_INVALID_ARGUMENT;
    auto &drive = drives_[robot_index];
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column) {
            const int r0 = (column + 1) % 3, r1 = (column + 2) % 3;
            const int c0 = (row + 1) % 3, c1 = (row + 2) % 3;
            drive.inverse[row][column] =
                (forward[r0][c0] * forward[r1][c1] - forward[r0][c1] * forward[r1][c0]) /
                determinant;
        }
    drive.kind = DrivePlant::Kind::Omni;
    drive.wheel_count = 3;
    std::copy_n(j, 3, drive.joints);
    drive.wheel_radius = desc.wheel_radius;
    drive.track_width = 0.0;
    std::fill(std::begin(drive.rates), std::end(drive.rates), 0.0);
    return set_robot_base_node_pose(robot_index, robot_base_poses_[robot_index]);
}

rk_result Simulation::valid_wheel_joints(uint32_t robot_index, const uint32_t *joints,
                                         uint32_t count) const {
    const auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    for (uint32_t index = 0; index < count; ++index)
        if (joints[index] >= binding->joints_.size() ||
            joints[index] >= binding->actuated_joints_.size() ||
            !binding->actuated_joints_[joints[index]])
            return RK_ERROR_INVALID_ARGUMENT;
    return RK_OK;
}

rk_result Simulation::clear_drive(uint32_t robot_index, int kind) {
    Lock lock(session_);
    if (robot_index >= drives_.size()) return RK_ERROR_INVALID_ARGUMENT;
    auto &drive = drives_[robot_index];
    if (static_cast<int>(drive.kind) != kind) return RK_OK;
    drive.kind = DrivePlant::Kind::None;
    std::fill(std::begin(drive.rates), std::end(drive.rates), 0.0);
    return RK_OK;
}

rk_result Simulation::clear_differential_drive(uint32_t robot_index) {
    return clear_drive(robot_index, static_cast<int>(DrivePlant::Kind::Differential));
}

rk_result Simulation::clear_omni_drive(uint32_t robot_index) {
    return clear_drive(robot_index, static_cast<int>(DrivePlant::Kind::Omni));
}

rk_result Simulation::get_differential_drive_state(
    uint32_t robot_index, rk_simulation_differential_drive_state &out_state) const {
    Lock lock(session_);
    if (out_state.struct_size < sizeof(out_state) || robot_index >= drives_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    const auto &drive = drives_[robot_index];
    out_state.enabled = drive.kind == DrivePlant::Kind::Differential ? 1u : 0u;
    out_state.x = drive.x;
    out_state.y = drive.y;
    out_state.yaw = drive.yaw;
    out_state.height = drive.height;
    out_state.left_wheel_rate = out_state.enabled ? drive.rates[0] : 0.0;
    out_state.right_wheel_rate = out_state.enabled ? drive.rates[1] : 0.0;
    return RK_OK;
}

rk_result Simulation::get_omni_drive_state(uint32_t robot_index,
                                           rk_simulation_omni_drive_state &out_state) const {
    Lock lock(session_);
    if (out_state.struct_size < sizeof(out_state) || robot_index >= drives_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    const auto &drive = drives_[robot_index];
    out_state.enabled = drive.kind == DrivePlant::Kind::Omni ? 1u : 0u;
    out_state.x = drive.x;
    out_state.y = drive.y;
    out_state.yaw = drive.yaw;
    out_state.height = drive.height;
    for (int wheel = 0; wheel < 3; ++wheel)
        out_state.wheel_rates[wheel] = out_state.enabled ? drive.rates[wheel] : 0.0;
    return RK_OK;
}

rk_result Simulation::drive_robot_base(uint32_t robot_index, const rk_simulation_pose &pose) {
    // The tick lock orders this scene edit between completed host steps. The
    // world refreshes every kinematic body from its scene node at the start of
    // a step and infers its twist from the motion, so the owner thread, host,
    // sensors, and reset pose stay intact.
    Lock lock(session_);
    if (robot_index >= robot_base_bodies_.size()) return RK_ERROR_INVALID_ARGUMENT;
    if (pose.struct_size < sizeof(pose) || !valid_pose(pose.position, pose.rotation))
        return RK_ERROR_INVALID_ARGUMENT;
    // A floating base moves only under physics; place or teleport it instead.
    if (robot_floating_[robot_index]) return RK_ERROR_INVALID_STATE;
    // The drive's twist is the motion from the pose the base held at the end
    // of the previous tick, differenced in double precision.
    const auto &from = robot_tick_poses_[robot_index];
    double linear[3], angular[3];
    for (int axis = 0; axis < 3; ++axis)
        linear[axis] = (pose.position[axis] - from.position[axis]) / fixed_timestep_;
    angular_velocity_between(from.rotation, pose.rotation, fixed_timestep_, angular);
    const auto result = drive_robot_base_body(robot_index, pose, linear, angular);
    if (result == RK_OK) seed_drive(robot_index, pose);
    return result;
}

rk_result Simulation::place_robot_base(uint32_t robot_index, const rk_simulation_pose &pose) {
    Lock lock(session_);
    if (robot_index >= robot_base_bodies_.size() || pose.struct_size < sizeof(pose) ||
        !valid_pose(pose.position, pose.rotation))
        return RK_ERROR_INVALID_ARGUMENT;
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    if (nksim_session_get_body_state(session_, robot_base_bodies_[robot_index], &state) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    // Carry the body-frame twist across the jump so the new heading keeps the
    // motion the base had instead of registering a velocity spike or a stop.
    double linear[3], angular[3];
    sensors::rotate(state.rotation, state.linear_velocity, linear, true);
    sensors::rotate(state.rotation, state.angular_velocity, angular, true);
    sensors::rotate(pose.rotation, linear, state.linear_velocity);
    sensors::rotate(pose.rotation, angular, state.angular_velocity);
    std::copy_n(pose.position, 3, state.position);
    std::copy_n(pose.rotation, 4, state.rotation);
    state.sleeping = 0;
    if (nksim_session_set_body_states(session_, &state, 1) != NKSIM_OK) return RK_ERROR_BACKEND;
    return set_robot_base_node_pose(robot_index, pose);
}

rk_result Simulation::teleport_robot(uint32_t robot_index, const rk_simulation_pose &pose) {
    Lock lock(session_);
    const auto world = stopped_world();
    if (world == 0 || robot_index >= robot_base_bodies_.size())
        return RK_ERROR_INVALID_STATE;
    if (pose.struct_size < sizeof(pose)) return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    auto result = set_robot_base_node_pose(robot_index, pose);
    if (result == RK_OK)
        result = set_body_pose(world, robot_base_bodies_[robot_index], pose.position, pose.rotation);
    if (result == RK_OK) binding->reset_sensors();
    return result;
}

rk_result Simulation::set_joint_positions(uint32_t robot_index, const double *positions,
                                          uint32_t count) {
    Lock lock(session_);
    const auto world = stopped_world();
    if (world == 0 || robot_index >= bindings_.size())
        return RK_ERROR_INVALID_STATE;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    if (!positions || count != binding->joints_.size()) return RK_ERROR_INVALID_ARGUMENT;
    const auto &blueprint = runtimes_[robot_index]->blueprint();
    for (uint32_t joint = 0; joint < count; ++joint) {
        const auto &limits = blueprint.joints[joint];
        if (!std::isfinite(positions[joint]) ||
            (limits.type == RK_RUNTIME_JOINT_FIXED && positions[joint] != 0.0) ||
            positions[joint] < limits.lower_limit || positions[joint] > limits.upper_limit)
            return RK_ERROR_INVALID_ARGUMENT;
    }
    for (uint32_t joint = 0; joint < count; ++joint) {
        // A pose seeds passive/coupled coordinates too; motor ownership only
        // controls drive commands. Leaving a follower at zero can spawn it
        // through a workpiece and generate a large first-step contact impulse.
        if (blueprint.joints[joint].type == RK_RUNTIME_JOINT_FIXED) continue;
        if (nksim_joint_set_state(world, binding->joints_[joint], positions[joint], 0.0) != NKSIM_OK)
            return RK_ERROR_BACKEND;
    }
    binding->reset_sensors();
    return RK_OK;
}

rk_result Simulation::set_servo_feedforward(uint32_t robot_index, uint32_t joint, double effort) {
    Lock lock(session_);
    if (robot_index >= bindings_.size() || !std::isfinite(effort)) return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    const auto &blueprint = runtimes_[robot_index]->blueprint();
    if (joint >= binding->servo_.size() || binding->servo_[joint].stiffness <= 0.0) return RK_ERROR_INVALID_ARGUMENT;
    const auto &limits = blueprint.joints[joint];
    if ((limits.limit_flags & RK_LIMIT_EFFORT) && std::abs(effort) > limits.max_effort)
        return RK_ERROR_INVALID_ARGUMENT;
    binding->servo_feedforward_[joint] = effort;
    return RK_OK;
}

rk_result Simulation::drive_process_joint(uint32_t robot_index, uint32_t joint,
                                            double position, double velocity, double max_effort) {
    Lock lock(session_);
    if (robot_index >= bindings_.size()) return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    const auto &blueprint = runtimes_[robot_index]->blueprint();
    if (joint >= binding->joints_.size() || !std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(max_effort) || max_effort < 0.0 || !blueprint.process_joint[joint])
        return RK_ERROR_INVALID_ARGUMENT;
    const auto &limits = blueprint.joints[joint];
    const bool rated = (limits.limit_flags & RK_LIMIT_EFFORT) && limits.max_effort > 0.0;
    if ((limits.type != RK_RUNTIME_JOINT_PRISMATIC && limits.type != RK_RUNTIME_JOINT_REVOLUTE) ||
        position < limits.lower_limit || position > limits.upper_limit ||
        ((limits.limit_flags & RK_LIMIT_VELOCITY) && std::abs(velocity) > limits.max_velocity) ||
        (rated && max_effort > limits.max_effort))
        return RK_ERROR_INVALID_ARGUMENT;
    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = binding->joints_[joint];
    target.mode = NKSIM_JOINT_TARGET_POSITION;
    target.target = position;
    target.max_force = max_effort > 0.0 ? max_effort
        : (limits.limit_flags & RK_LIMIT_EFFORT) && limits.max_effort == 0.0 ? -1.0 : limits.max_effort;
    const auto &gains = binding->servo_[joint];
    if (gains.stiffness > 0.0) {
        target.mode = NKSIM_JOINT_TARGET_SERVO;
        target.stiffness = gains.stiffness;
        target.damping = gains.damping;
        target.feedforward = binding->servo_feedforward_[joint];
        target.velocity = velocity;
    }
    return nksim_session_submit_joint_targets(session_, &target, 1) == NKSIM_OK
        ? RK_OK : RK_ERROR_BACKEND;
}

rk_result Simulation::spring_process_joint(uint32_t robot_index, uint32_t joint, double rest_position,
                                          double stiffness, double damping) {
    Lock lock(session_);
    if (robot_index >= bindings_.size()) return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    const auto &blueprint = runtimes_[robot_index]->blueprint();
    if (joint >= binding->joints_.size() || !std::isfinite(rest_position) || !std::isfinite(stiffness) ||
        !(stiffness > 0.0) || !std::isfinite(damping) || damping < 0.0 || !blueprint.process_joint[joint])
        return RK_ERROR_INVALID_ARGUMENT;
    const auto &limits = blueprint.joints[joint];
    if ((limits.type != RK_RUNTIME_JOINT_PRISMATIC && limits.type != RK_RUNTIME_JOINT_REVOLUTE) ||
        rest_position < limits.lower_limit || rest_position > limits.upper_limit)
        return RK_ERROR_INVALID_ARGUMENT;
    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = binding->joints_[joint];
    target.mode = NKSIM_JOINT_TARGET_SERVO;
    target.target = rest_position;
    target.max_force = (limits.limit_flags & RK_LIMIT_EFFORT) && limits.max_effort == 0.0
        ? -1.0 : limits.max_effort;
    target.stiffness = stiffness;
    target.damping = damping;
    return nksim_session_submit_joint_targets(session_, &target, 1) == NKSIM_OK
        ? RK_OK : RK_ERROR_BACKEND;
}

rk_result Simulation::get_robot_pose(uint32_t robot_index,rk_simulation_pose &out_pose) const {
    Lock lock(session_);
    if(out_pose.struct_size<sizeof(out_pose)||robot_index>=robot_base_bodies_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    return read_body_pose(robot_base_bodies_[robot_index],out_pose);
}

rk_result Simulation::get_robot_base_velocity(uint32_t robot_index,
                                             rk_simulation_twist &out_twist) const {
    Lock lock(session_);
    if (out_twist.struct_size < sizeof(out_twist) || robot_index >= robot_base_bodies_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    const auto result = read_body_state(robot_base_bodies_[robot_index], state);
    if (result != RK_OK) return result;
    std::copy_n(state.linear_velocity, 3, out_twist.linear);
    std::copy_n(state.angular_velocity, 3, out_twist.angular);
    return RK_OK;
}

rk_result Simulation::apply_robot_force(uint32_t robot_index, const rk_simulation_wrench &wrench) {
    Lock lock(session_);
    if (wrench.struct_size < sizeof(wrench) || robot_index >= robot_base_bodies_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    for (int axis = 0; axis < 3; ++axis)
        if (!std::isfinite(wrench.force[axis]) || !std::isfinite(wrench.torque[axis]))
            return RK_ERROR_INVALID_ARGUMENT;
    nksim_body_force push{};
    push.struct_size = sizeof(push);
    push.body = robot_base_bodies_[robot_index];
    std::copy_n(wrench.force, 3, push.force);
    std::copy_n(wrench.torque, 3, push.torque);
    return nksim_session_submit_forces(session_, &push, 1) == NKSIM_OK ? RK_OK : RK_ERROR_BACKEND;
}

rk_result Simulation::get_link_pose(uint32_t robot_index,uint32_t link_index,
                                     rk_simulation_pose &out_pose) const {
    Lock lock(session_);
    if(out_pose.struct_size<sizeof(out_pose)||robot_index>=bindings_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    const auto binding=bindings_[robot_index].lock();
    if(!binding)return RK_ERROR_INVALID_HANDLE;
    if(link_index>=binding->bodies_.size())return RK_ERROR_INVALID_ARGUMENT;
    return read_body_pose(binding->bodies_[link_index],out_pose);
}

rk_result Simulation::get_link_body(uint32_t robot_index,uint32_t link_index,
                                     nksim_body &out_body) const {
    Lock lock(session_);
    if(robot_index>=bindings_.size())return RK_ERROR_INVALID_ARGUMENT;
    const auto binding=bindings_[robot_index].lock();
    if(!binding)return RK_ERROR_INVALID_HANDLE;
    if(link_index>=binding->bodies_.size())return RK_ERROR_INVALID_ARGUMENT;
    out_body=binding->bodies_[link_index];
    return RK_OK;
}

rk_result Simulation::bind_object_to_link(nksim_object object, uint32_t robot_index,
        uint32_t link_index) {
    Lock lock(session_);
    if (object == 0 || robot_index >= bindings_.size()) return RK_ERROR_INVALID_ARGUMENT;
    const auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    if (link_index >= binding->bodies_.size()) return RK_ERROR_INVALID_ARGUMENT;
    nksim_body body = 0;
    if (nksim_session_get_object_body(session_, object, &body) != NKSIM_OK || body == 0)
        return RK_ERROR_INVALID_HANDLE;
    for (auto &owner : object_carriers_) if (owner.object == object) {
        owner.body = body;
        owner.robot_index = robot_index;
        owner.link_index = link_index;
        return RK_OK;
    }
    try {
        object_carriers_.push_back({object, body, robot_index, link_index});
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
    return RK_OK;
}

rk_result Simulation::unbind_object(nksim_object object) {
    Lock lock(session_);
    if (object == 0) return RK_ERROR_INVALID_ARGUMENT;
    const auto found = std::find_if(object_carriers_.begin(), object_carriers_.end(),
        [object](const ObjectCarrierOwner &owner) { return owner.object == object; });
    if (found != object_carriers_.end()) object_carriers_.erase(found);
    return RK_OK;
}

rk_result Simulation::create_object_screw_joint(const rk_simulation_object_screw_desc &request,
        nksim_joint &out_joint) {
    Lock lock(session_);
    out_joint = 0;
    const auto finite_positive = [](double value) { return std::isfinite(value) && value > 0.0; };
    if (request.struct_size < sizeof(request) || request.reserved[0] || request.reserved[1] ||
        request.member_body == 0 || request.screw_body == 0 || request.member_body == request.screw_body ||
        !std::isfinite(request.lead) || request.lead == 0.0 ||
        !std::isfinite(request.lower_limit) || !std::isfinite(request.upper_limit) ||
        !(request.lower_limit < request.upper_limit) ||
        (request.socket_ratio != 1.0 && request.socket_ratio != -1.0) ||
        !finite_positive(request.thread_stiffness) || !finite_positive(request.thread_damping_ratio) ||
        !std::isfinite(request.advance_armature) || request.advance_armature < 0.0 ||
        request.robot_index >= bindings_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    double axis_norm = 0.0;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(request.axis_point[i]) || !std::isfinite(request.axis[i]))
            return RK_ERROR_INVALID_ARGUMENT;
        axis_norm += request.axis[i] * request.axis[i];
    }
    if (std::abs(axis_norm - 1.0) > 1e-8) return RK_ERROR_INVALID_ARGUMENT;
    const auto world = stopped_world();
    if (world == 0) return RK_ERROR_INVALID_STATE;
    const auto binding = bindings_[request.robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    const auto &blueprint = runtimes_[request.robot_index]->blueprint();
    if (request.spindle_joint_index >= binding->joints_.size() ||
        request.spindle_joint_index >= blueprint.joint_count ||
        blueprint.joints[request.spindle_joint_index].type != RK_RUNTIME_JOINT_REVOLUTE)
        return RK_ERROR_INVALID_ARGUMENT;
    try {
        object_joints_.reserve(object_joints_.size() + 1);
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }

    nksim_body_state member_state{}, screw_state{};
    member_state.struct_size = screw_state.struct_size = sizeof(nksim_body_state);
    nksim_joint_state spindle_state{};
    spindle_state.struct_size = sizeof(spindle_state);
    const auto spindle = binding->joints_[request.spindle_joint_index];
    if (nksim_body_get_state(world, request.member_body, &member_state) != NKSIM_OK ||
        nksim_body_get_state(world, request.screw_body, &screw_state) != NKSIM_OK ||
        nksim_joint_get_state(world, spindle, &spindle_state) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    // The joint frame is the screw's own at this measured boundary, so creating the helix moves nothing.
    const auto member = xform_from(member_state.position, member_state.rotation);
    const auto screw = xform_from(screw_state.position, screw_state.rotation);
    const auto member_inverse = xform_inverse(member), screw_inverse = xform_inverse(screw);
    const auto relative = xform_compose(member_inverse, screw);

    const auto began = nksim_world_begin_topology_update(world);
    if (began != NKSIM_OK) return from_sim(began);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_HELICAL;
    joint_desc.body_a = request.member_body;
    joint_desc.body_b = request.screw_body;
    std::copy_n(relative.rot, 4, joint_desc.rotation_a);
    joint_desc.rotation_b[3] = 1.0;
    double anchor_a[3], anchor_b[3], axis_a[3], rotated[3];
    quat_rotate(member_inverse.rot, request.axis_point, rotated);
    for (int i = 0; i < 3; ++i) anchor_a[i] = rotated[i] + member_inverse.pos[i];
    quat_rotate(screw_inverse.rot, request.axis_point, rotated);
    for (int i = 0; i < 3; ++i) anchor_b[i] = rotated[i] + screw_inverse.pos[i];
    quat_rotate(member_inverse.rot, request.axis, axis_a);
    std::copy_n(anchor_a, 3, joint_desc.anchor_a);
    std::copy_n(anchor_b, 3, joint_desc.anchor_b);
    std::copy_n(axis_a, 3, joint_desc.axis_a);
    joint_desc.lower_limit = request.lower_limit;
    joint_desc.upper_limit = request.upper_limit;
    joint_desc.max_force = -1.0;  // Unpowered: only the socket turns it.
    joint_desc.lead = request.lead;
    joint_desc.lead_stiffness = request.thread_stiffness;
    joint_desc.lead_damping_ratio = request.thread_damping_ratio;
    joint_desc.lead_armature = request.advance_armature;
    nksim_joint helix = 0;
    auto result = nksim_joint_create(world, &joint_desc, &helix);
    if (result == NKSIM_OK) {
        // The socket: turn = ratio * spindle + offset, zero turn at the spindle's position now.
        nksim_joint_coupling_desc socket{};
        socket.struct_size = sizeof(socket);
        socket.leader = spindle;
        socket.follower = helix;
        socket.ratio = request.socket_ratio;
        socket.offset = -request.socket_ratio * spindle_state.position;
        result = nksim_joint_couple(world, &socket);
    }
    if (result != NKSIM_OK && helix != 0) nksim_joint_destroy(world, helix);
    const auto committed = nksim_world_end_topology_update(world);
    if (result != NKSIM_OK) return from_sim(result);
    if (committed != NKSIM_OK) return from_sim(committed);
    object_joints_.push_back({helix, request.screw_body, request.robot_index, 0, false});
    out_joint = helix;
    return RK_OK;
}

rk_result Simulation::create_object_fixed_follower(uint32_t robot_index,
        uint32_t parent_link_index, nksim_body follower_body, nksim_joint &out_joint) {
    Lock lock(session_);
    out_joint = 0;
    if (follower_body == 0 || robot_index >= bindings_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    const auto world = stopped_world();
    if (world == 0) return RK_ERROR_INVALID_STATE;
    const auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    if (parent_link_index >= binding->bodies_.size()) return RK_ERROR_INVALID_ARGUMENT;
    try {
        object_joints_.reserve(object_joints_.size() + 1);
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }

    nksim_body_state parent_state{}, follower_state{};
    parent_state.struct_size = follower_state.struct_size = sizeof(nksim_body_state);
    if (nksim_body_get_state(world, binding->bodies_[parent_link_index], &parent_state) != NKSIM_OK ||
        nksim_body_get_state(world, follower_body, &follower_state) != NKSIM_OK)
        return RK_ERROR_BACKEND;

    // Capture the measured transform at the interlocked mount. Creating this
    // fixed joint therefore preserves the seated CAD body instead of snapping
    // it to a nominal connector or the link origin.
    const auto parent = xform_from(parent_state.position, parent_state.rotation);
    const auto follower = xform_from(follower_state.position, follower_state.rotation);
    const auto relative = xform_compose(xform_inverse(parent), follower);

    const auto began = nksim_world_begin_topology_update(world);
    if (began != NKSIM_OK) return from_sim(began);
    nksim_joint_desc description{};
    description.struct_size = sizeof(description);
    description.type = NKSIM_JOINT_FIXED;
    description.body_a = binding->bodies_[parent_link_index];
    description.body_b = follower_body;
    std::copy_n(relative.pos, 3, description.anchor_a);
    std::copy_n(relative.rot, 4, description.rotation_a);
    description.rotation_b[3] = 1.0;
    nksim_joint joint = 0;
    const auto created = nksim_joint_create(world, &description, &joint);
    if (created != NKSIM_OK && joint != 0) nksim_joint_destroy(world, joint);
    const auto committed = nksim_world_end_topology_update(world);
    if (created != NKSIM_OK) return from_sim(created);
    if (committed != NKSIM_OK) return from_sim(committed);
    object_joints_.push_back({joint, follower_body, robot_index, parent_link_index});
    out_joint = joint;
    return RK_OK;
}

rk_result Simulation::create_robot_fixed_follower(uint32_t parent_robot_index,
        uint32_t parent_link_index, uint32_t child_robot_index,
        uint32_t child_root_link_index, nksim_joint &out_joint) {
    out_joint = 0;
    nksim_body parent_body = 0;
    {
        Lock lock(session_);
        if (parent_robot_index >= bindings_.size()) return RK_ERROR_INVALID_ARGUMENT;
        const auto parent = bindings_[parent_robot_index].lock();
        if (!parent) return RK_ERROR_INVALID_HANDLE;
        if (parent_link_index >= parent->bodies_.size()) return RK_ERROR_INVALID_ARGUMENT;
        parent_body = parent->bodies_[parent_link_index];
    }
    return create_robot_body_fixed_follower(parent_body, child_robot_index,
        child_root_link_index, out_joint);
}

rk_result Simulation::create_robot_body_fixed_follower(nksim_body parent_body,
        uint32_t child_robot_index, uint32_t child_root_link_index, nksim_joint &out_joint) {
    Lock lock(session_);
    out_joint = 0;
    if (parent_body == 0 || child_robot_index >= bindings_.size() ||
        child_robot_index >= robot_floating_.size() || !robot_floating_[child_robot_index])
        return RK_ERROR_INVALID_ARGUMENT;
    const auto world = stopped_world();
    if (world == 0) return RK_ERROR_INVALID_STATE;
    const auto child = bindings_[child_robot_index].lock();
    if (!child) return RK_ERROR_INVALID_HANDLE;
    if (child_root_link_index >= child->bodies_.size() ||
        child->bodies_[child_root_link_index] != robot_base_bodies_[child_robot_index])
        return RK_ERROR_INVALID_ARGUMENT;
    for (const auto &owner : robot_joints_)
        if (owner.child_robot_index == child_robot_index)
            return RK_ERROR_INVALID_STATE;

    // Environment bodies can hold a tool at a dock. Robot-body parents also
    // keep their mount graph acyclic, including a robot already mounted to a dock.
    const auto no_parent_robot = std::numeric_limits<uint32_t>::max();
    uint32_t parent_robot_index = no_parent_robot;
    for (uint32_t robot = 0; robot < bindings_.size(); ++robot) {
        const auto binding = bindings_[robot].lock();
        if (!binding) continue;
        for (uint32_t link = 0; link < binding->bodies_.size(); ++link)
            if (binding->bodies_[link] == parent_body) {
                parent_robot_index = robot;
                break;
            }
        if (parent_robot_index != no_parent_robot) break;
    }
    if (parent_robot_index == child_robot_index) return RK_ERROR_INVALID_ARGUMENT;
    if (parent_robot_index != no_parent_robot) {
        auto ancestor = parent_robot_index;
        while (ancestor != no_parent_robot) {
            if (ancestor == child_robot_index) return RK_ERROR_INVALID_ARGUMENT;
            const auto mounted = std::find_if(robot_joints_.begin(), robot_joints_.end(),
                [ancestor](const RobotJointOwner &owner) {
                    return owner.child_robot_index == ancestor;
                });
            if (mounted == robot_joints_.end()) break;
            ancestor = mounted->parent_robot_index;
        }
    }
    try {
        robot_joints_.reserve(robot_joints_.size() + 1);
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }

    const auto child_body = child->bodies_[child_root_link_index];
    nksim_body_state parent_state{}, child_state{};
    parent_state.struct_size = child_state.struct_size = sizeof(nksim_body_state);
    if (nksim_body_get_state(world, parent_body, &parent_state) != NKSIM_OK ||
        nksim_body_get_state(world, child_body, &child_state) != NKSIM_OK)
        return RK_ERROR_BACKEND;

    // Capture the measured connector pose so docking never moves either
    // articulated runtime when its fixed mount is created.
    const auto parent_pose = xform_from(parent_state.position, parent_state.rotation);
    const auto child_pose = xform_from(child_state.position, child_state.rotation);
    const auto relative = xform_compose(xform_inverse(parent_pose), child_pose);
    const auto began = nksim_world_begin_topology_update(world);
    if (began != NKSIM_OK) return from_sim(began);
    nksim_joint_desc description{};
    description.struct_size = sizeof(description);
    description.type = NKSIM_JOINT_FIXED;
    description.body_a = parent_body;
    description.body_b = child_body;
    std::copy_n(relative.pos, 3, description.anchor_a);
    std::copy_n(relative.rot, 4, description.rotation_a);
    description.rotation_b[3] = 1.0;
    nksim_joint joint = 0;
    const auto created = nksim_joint_create(world, &description, &joint);
    if (created != NKSIM_OK && joint != 0) nksim_joint_destroy(world, joint);
    const auto committed = nksim_world_end_topology_update(world);
    if (created != NKSIM_OK) return from_sim(created);
    if (committed != NKSIM_OK) return from_sim(committed);
    robot_joints_.push_back({joint, parent_body, parent_robot_index, child_robot_index,
        child_root_link_index});
    out_joint = joint;
    return RK_OK;
}

rk_result Simulation::destroy_robot_joint(nksim_joint joint) {
    Lock lock(session_);
    const auto found = std::find_if(robot_joints_.begin(), robot_joints_.end(),
        [joint](const RobotJointOwner &owner) { return owner.joint == joint; });
    if (found == robot_joints_.end()) return RK_ERROR_INVALID_HANDLE;
    const auto world = stopped_world();
    if (world == 0) return RK_ERROR_INVALID_STATE;
    nksim_joint_state state{};
    state.struct_size = sizeof(state);
    if (nksim_joint_get_state(world, joint, &state) != NKSIM_OK) return RK_ERROR_INVALID_HANDLE;
    if (nksim_world_begin_topology_update(world) != NKSIM_OK) return RK_ERROR_INVALID_STATE;
    nksim_joint_destroy(world, joint);
    if (nksim_world_end_topology_update(world) != NKSIM_OK) return RK_ERROR_BACKEND;
    robot_joints_.erase(found);
    return RK_OK;
}

rk_result Simulation::destroy_object_joint(nksim_joint joint) {
    Lock lock(session_);
    const auto found = std::find_if(object_joints_.begin(), object_joints_.end(),
        [joint](const ObjectJointOwner &owner) { return owner.joint == joint; });
    if (found == object_joints_.end()) return RK_ERROR_INVALID_HANDLE;
    const auto world = stopped_world();
    if (world == 0) return RK_ERROR_INVALID_STATE;
    nksim_joint_state state{};
    state.struct_size = sizeof(state);
    if (nksim_joint_get_state(world, joint, &state) != NKSIM_OK) return RK_ERROR_INVALID_HANDLE;
    if (nksim_world_begin_topology_update(world) != NKSIM_OK) return RK_ERROR_INVALID_STATE;
    nksim_joint_destroy(world, joint);
    if (nksim_world_end_topology_update(world) != NKSIM_OK) return RK_ERROR_BACKEND;
    object_joints_.erase(found);
    return RK_OK;
}

rk_result Simulation::get_robot_contacts(rk_robot_runtime runtime,
                                         std::vector<rk_robot_contact> &out,
                                         uint64_t *step_index) const {
    Lock lock(session_);
    out.clear();
    const auto handle = std::find(handles_.begin(), handles_.end(), runtime);
    if (handle == handles_.end()) return RK_ERROR_INVALID_HANDLE;
    const auto robot_index = static_cast<std::size_t>(handle - handles_.begin());
    if (robot_index >= bindings_.size()) return RK_ERROR_INVALID_STATE;
    const auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    // The latest tick's contacts while hosted; a stopped session's world holds them otherwise.
    nksim_snapshot snapshot = nksim_session_latest_snapshot(session_), owned = 0;
    if (snapshot == 0) {
        const auto world = stopped_world();
        if (world == 0 || nksim_world_snapshot(world, &owned) != NKSIM_OK)
            return RK_ERROR_INVALID_STATE;
        snapshot = owned;
    }
    if (step_index) {
        nksim_clock clock{};
        clock.struct_size = sizeof(clock);
        if (nksim_snapshot_get_clock(snapshot, &clock) != NKSIM_OK) {
            if (owned) nksim_snapshot_destroy(owned);
            return RK_ERROR_BACKEND;
        }
        *step_index = clock.step_index;
    }
    rk_result result_code = RK_OK;
    uint64_t count = 0;
    if (nksim_snapshot_get_contact_count(snapshot, &count) != NKSIM_OK)
        result_code = RK_ERROR_BACKEND;
    auto append_contact = [&](const nksim_contact &source, nksim_body body,
                              uint32_t link_index, int32_t tool_piece_index, nksim_body carried_body = 0) {
        if (source.body_a != body && source.body_b != body) return;
        const bool body_is_a = source.body_a == body;
        const auto other_body = body_is_a ? source.body_b : source.body_a;
        rk_robot_contact result{};
        result.struct_size = sizeof(result);
        result.other_robot = UINT32_MAX;
        result.other_link = UINT32_MAX;
        result.link_index = link_index;
        result.tool_piece_index = tool_piece_index;
        result.other_piece_index = body_is_a ? source.part_b : source.part_a;
        nksim_object other_object = 0;
        if (nksim_session_find_object(session_, other_body, &other_object) == NKSIM_OK) {
            result.other_object = other_object;
            result.other_kind = RK_CONTACT_OTHER_OBJECT;
        } else {
            result.other_kind = RK_CONTACT_OTHER_WORLD;
            for (uint32_t other_robot = 0; other_robot < bindings_.size(); ++other_robot) {
                const auto other_binding = bindings_[other_robot].lock();
                if (!other_binding) continue;
                const auto &other_bodies = other_binding->bodies_;
                const auto found = std::find(other_bodies.begin(), other_bodies.end(), other_body);
                if (found == other_bodies.end()) continue;
                result.other_kind = RK_CONTACT_OTHER_ROBOT_LINK;
                result.other_robot = other_robot;
                result.other_link = static_cast<uint32_t>(found - other_bodies.begin());
                break;
            }
        }
        result.distance = source.distance;
        std::copy_n(source.position, 3, result.position);
        std::copy_n(source.normal, 3, result.normal);
        if (source.wrench_valid) {
            result.wrench_valid = 1;
            std::copy_n(source.force_on_body_a, 3, result.force_on_link);
            std::copy_n(source.torque_on_body_a, 3, result.torque_on_link);
        }
        if (!body_is_a) {
            for (double &axis : result.normal) axis = -axis;
            for (double &axis : result.force_on_link) axis = -axis;
            for (double &axis : result.torque_on_link) axis = -axis;
        }
        result.active = source.active;
        // A held payload's contact loads the link through it; name the payload so callers can tell it apart.
        if (carried_body != 0 && nksim_session_find_object(session_, carried_body, &result.carried_object) != NKSIM_OK)
            result.carried_object = 0;
        out.push_back(result);
    };
    // Resolve the current carrier graph, not just the directly bound payloads. A gripped plate can
    // carry a fastened housing and screws, and their external contacts load the same robot link.
    std::unordered_map<nksim_body, int32_t> owner_links;
    auto owning_link = [&](nksim_body body) -> int32_t {
        std::vector<nksim_body> path;
        int32_t result = -1;
        while (body != 0) {
            const auto cached = owner_links.find(body);
            if (cached != owner_links.end()) { result = cached->second; break; }
            if (std::find(path.begin(), path.end(), body) != path.end()) break;
            path.push_back(body);
            const auto link = std::find(binding->bodies_.begin(), binding->bodies_.end(), body);
            if (link != binding->bodies_.end()) {
                result = static_cast<int32_t>(link - binding->bodies_.begin()); break;
            }
            for (const auto &owner : object_joints_)
                if (owner.tool_load && owner.robot_index == robot_index && owner.follower == body) {
                    result = static_cast<int32_t>(owner.parent_link_index); break;
                }
            if (result >= 0) break;
            for (const auto &owner : object_carriers_)
                if (owner.robot_index == robot_index && owner.body == body) {
                    result = static_cast<int32_t>(owner.link_index); break;
                }
            if (result >= 0) break;
            nksim_object object = 0;
            nksim_body parent = 0;
            if (nksim_session_find_object(session_, body, &object) != NKSIM_OK ||
                nksim_session_get_object_carrier(session_, object, &parent) != NKSIM_OK) break;
            body = parent;
        }
        for (const auto part : path) owner_links.emplace(part, result);
        return result;
    };
    auto internal_to_tool = [&](nksim_body body, uint32_t link_index) {
        return owning_link(body) == static_cast<int32_t>(link_index);
    };
    for (uint64_t i = 0; result_code == RK_OK && i < count; ++i) {
        nksim_contact source{};
        source.struct_size = sizeof(source);
        if (nksim_snapshot_get_contact(snapshot, i, &source) != NKSIM_OK) {
            result_code = RK_ERROR_BACKEND;
            break;
        }
        for (uint32_t link = 0; link < binding->bodies_.size(); ++link) {
            const auto body = binding->bodies_[link];
            if (source.body_a == body && !internal_to_tool(source.body_b, link)) {
                const auto part = source.part_a;
                const auto tool_piece = link == binding->tool_link_ && part >= binding->tool_part_start_ &&
                    part < binding->tool_part_start_ + binding->tool_part_count_
                    ? static_cast<int32_t>(part - binding->tool_part_start_) : -1;
                append_contact(source, body, link, tool_piece);
            } else if (source.body_b == body && !internal_to_tool(source.body_a, link)) {
                const auto part = source.part_b;
                const auto tool_piece = link == binding->tool_link_ && part >= binding->tool_part_start_ &&
                    part < binding->tool_part_start_ + binding->tool_part_count_
                    ? static_cast<int32_t>(part - binding->tool_part_start_) : -1;
                append_contact(source, body, link, tool_piece);
            }
        }
        for (const auto body : {source.body_a, source.body_b}) {
            // Direct robot-link contacts were already appended above. For retained objects and
            // removable tool subtrees, report the exact contacting leaf as the carried object.
            if (std::find(binding->bodies_.begin(), binding->bodies_.end(), body) != binding->bodies_.end())
                continue;
            const auto link = owning_link(body);
            if (link < 0) continue;
            const auto other_body = source.body_a == body ? source.body_b : source.body_a;
            if (!internal_to_tool(other_body, static_cast<uint32_t>(link)))
                append_contact(source, body, static_cast<uint32_t>(link), -1, body);
        }
    }
    if (owned != 0) nksim_snapshot_destroy(owned);
    return result_code;
}

rk_result Simulation::read_body_pose(nksim_body body,rk_simulation_pose &out_pose) const {
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    const auto result = read_body_state(body, state);
    if (result != RK_OK) return result;
    std::copy_n(state.position,3,out_pose.position);
    std::copy_n(state.rotation,4,out_pose.rotation);
    return RK_OK;
}

rk_result Simulation::read_body_state(nksim_body body, nksim_body_state &state) const {
    state.struct_size = sizeof(state);
    return nksim_session_get_body_state(session_, body, &state) == NKSIM_OK ? RK_OK : RK_ERROR_BACKEND;
}

rk_result Simulation::capture_presentation(rk_simulation_presentation_info &out_info,
        std::vector<rk_simulation_presentation_pose> &out_poses) const {
    if (out_info.struct_size < sizeof(out_info)) return RK_ERROR_INVALID_ARGUMENT;
    Lock lock(session_);
    nksim_frame frame = 0;
    if (nksim_session_capture(session_, &frame) != NKSIM_OK) return RK_ERROR_BACKEND;
    const auto result = present_frame(frame, out_info, out_poses);
    nksim_frame_destroy(frame);
    return result;
}

rk_result Simulation::present_frame(nksim_frame frame, rk_simulation_presentation_info &out_info,
        std::vector<rk_simulation_presentation_pose> &out_poses) const {
    if (out_info.struct_size < sizeof(out_info)) return RK_ERROR_INVALID_ARGUMENT;
    Lock lock(session_);
    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    if (nksim_frame_get_clock(frame, &clock) != NKSIM_OK) return RK_ERROR_INVALID_HANDLE;
    auto add = [&](uint32_t kind, uint32_t robot_index, uint32_t link_index,
                   const double position[3], const double rotation[4]) {
        rk_simulation_presentation_pose item{};
        item.struct_size = sizeof(item);
        item.kind = kind;
        item.robot_index = robot_index;
        item.link_index = link_index;
        std::copy_n(position, 3, item.position);
        std::copy_n(rotation, 4, item.rotation);
        out_poses.push_back(item);
    };
    auto body = [&](nksim_body handle, nksim_body_state &state) {
        state = {};
        state.struct_size = sizeof(state);
        return nksim_frame_get_body_state(frame, handle, &state) == NKSIM_OK;
    };
    out_poses.clear();
    for (uint32_t robot_index = 0; robot_index < robot_base_bodies_.size(); ++robot_index) {
        nksim_body_state state{};
        if (!body(robot_base_bodies_[robot_index], state)) return RK_ERROR_BACKEND;
        add(RK_SIMULATION_PRESENTATION_ROBOT_BASE, robot_index, 0, state.position,
            state.rotation);
        const auto binding = bindings_[robot_index].lock();
        if (!binding) return RK_ERROR_INVALID_HANDLE;
        for (uint32_t link_index = 0; link_index < binding->bodies_.size(); ++link_index) {
            if (!body(binding->bodies_[link_index], state)) return RK_ERROR_BACKEND;
            add(RK_SIMULATION_PRESENTATION_ROBOT_LINK, robot_index, link_index,
                state.position, state.rotation);
        }
    }
    if (out_poses.size() > std::numeric_limits<uint32_t>::max()) return RK_ERROR_OUT_OF_MEMORY;
    out_info.step_index = clock.step_index;
    out_info.simulation_time = clock.time;
    out_info.pose_count = static_cast<uint32_t>(out_poses.size());
    return RK_OK;
}

rk_result Simulation::virtual_device_ready(uint32_t robot_index, uint32_t &ready) {
    Lock lock(session_);
    if (robot_index >= virtual_devices_.size() || !virtual_devices_[robot_index]) return RK_ERROR_INVALID_ARGUMENT;
    ready = virtual_devices_[robot_index]->ready_for_plans() ? 1 : 0;
    return RK_OK;
}

rk_result Simulation::stop_virtual_device(uint32_t robot_index) {
    Lock lock(session_);
    if (robot_index >= virtual_devices_.size() || !virtual_devices_[robot_index]) return RK_ERROR_INVALID_ARGUMENT;
    return virtual_devices_[robot_index]->stop_device() ? RK_OK : RK_ERROR_BACKEND;
}
rk_result Simulation::set_virtual_device_input(uint32_t robot_index, uint32_t kind, uint32_t input, double value) {
    Lock lock(session_);
    if (robot_index >= virtual_devices_.size() || !virtual_devices_[robot_index]) return RK_ERROR_INVALID_ARGUMENT;
    return virtual_devices_[robot_index]->set_peripheral_input(kind, input, value) ? RK_OK : RK_ERROR_INVALID_ARGUMENT;
}
rk_result Simulation::get_virtual_device_sensor(uint32_t robot_index, uint32_t slot, rk_simulation_device_sensor &sensor) {
    Lock lock(session_);
    if (robot_index >= virtual_devices_.size() || !virtual_devices_[robot_index]) return RK_ERROR_INVALID_ARGUMENT;
    return virtual_devices_[robot_index]->sensor_sample(slot, sensor.sample, sensor.values) ? RK_OK : RK_ERROR_INVALID_ARGUMENT;
}

rk_result Simulation::get_joint_sample(uint32_t robot_index, uint32_t joint, rk_simulation_joint_sample &sample) {
    Lock lock(session_);
    if (robot_index >= bindings_.size()) return RK_ERROR_INVALID_ARGUMENT;
    const auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    if (joint >= binding->joints_.size()) return RK_ERROR_INVALID_ARGUMENT;
    const auto snapshot = nksim_session_latest_snapshot(session_);
    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    uint64_t count = 0;
    if (!snapshot || nksim_snapshot_get_clock(snapshot, &clock) != NKSIM_OK ||
        nksim_snapshot_get_joint_count(snapshot, &count) != NKSIM_OK) return RK_ERROR_INVALID_STATE;
    for (uint64_t index = 0; index < count; ++index) {
        nksim_joint_state state{};
        state.struct_size = sizeof(state);
        if (nksim_snapshot_get_joint(snapshot, index, &state) != NKSIM_OK) return RK_ERROR_BACKEND;
        if (state.joint != binding->joints_[joint]) continue;
        if (!std::isfinite(state.position) || !std::isfinite(state.velocity) || !std::isfinite(state.effort))
            return RK_ERROR_BACKEND;
        sample = {clock.step_index, clock.time, state.position, state.velocity, state.effort};
        return RK_OK;
    }
    return RK_ERROR_BACKEND;
}

rk_result Simulation::get_virtual_device_channel(uint32_t robot_index, uint32_t channel, double &value) {
    Lock lock(session_);
    if (robot_index >= virtual_devices_.size() || !virtual_devices_[robot_index] ||
        channel >= runtimes_[robot_index]->blueprint().channel_count) return RK_ERROR_INVALID_ARGUMENT;
    const auto channels = virtual_devices_[robot_index]->channel_values();
    if (channel >= channels.size() || !std::isfinite(channels[channel])) return RK_ERROR_BACKEND;
    value = channels[channel];
    return RK_OK;
}

rk_result Simulation::cut_virtual_device_link(uint32_t robot_index, bool cut) {
    Lock lock(session_);
    if (robot_index >= virtual_devices_.size() || !virtual_devices_[robot_index])
        return RK_ERROR_INVALID_ARGUMENT;
    virtual_devices_[robot_index]->cut_link(cut);
    return RK_OK;
}

rk_result Simulation::prepare(const nksim_tick &tick) {
    // Simulated endpoints apply commands and publish feedback on the fixed
    // simulation clock, including when the owner paces ticks using wall time.
    // Brake deadlines and feedback freshness must advance with physics.
    // A robot whose commands cannot be applied faults alone: the others still
    // apply theirs and the tick goes ahead. Rejecting the whole tick would let
    // one robot, for instance a fallen humanoid whose policy keeps commanding
    // it, stop every machine that shares the session.
    const auto simulation_ns = tick.step_index * static_cast<std::uint64_t>(period_.count());
    for (std::size_t index = 0; index < runtimes_.size(); ++index) {
        if (runtimes_[index]->apply_pending_commands(simulation_ns) != RK_OK)
            runtimes_[index]->fail_tick();
    }
    return RK_OK;
}

void Simulation::discard() noexcept {
    for (const auto &runtime : runtimes_)
        runtime->discard_pending_commands();
}

rk_result Simulation::submit(const nksim_tick &tick) {
    const auto simulation_ns = tick.step_index * static_cast<std::uint64_t>(period_.count());
    std::size_t binding_index = 0;
    virtual_samples_.assign(runtimes_.size(), rk_robot_state{});
    virtual_sample_results_.assign(runtimes_.size(), RK_OK);
    for (auto it = bindings_.begin(); it != bindings_.end();) {
        const auto binding = it->lock();
        if (!binding) {
            it = bindings_.erase(it);
            virtual_bindings_.erase(virtual_bindings_.begin() + binding_index);
            virtual_devices_.erase(virtual_devices_.begin() + binding_index);
            continue;
        }
        auto targets = binding->take_pending_targets();
        if (binding_index >= runtimes_.size()) return RK_ERROR_BACKEND;
        auto &runtime = *runtimes_[binding_index];
        auto process_fault = binding->apply_cylinder_valves(runtime, targets);
        if (!process_fault) process_fault = binding->apply_velocity_drives(runtime, targets);
        rk_result brake_result = RK_OK;
        if (!process_fault) brake_result = binding->apply_brake_coils(runtime, process_fault);
        if (process_fault || brake_result != RK_OK) {
            // Reject this robot's staged motion, keep the shared world advancing, and
            // apply physical de-energized behavior (latched valves retain supply air).
            runtime.fail_tick(process_fault ? process_fault->code() : RK_FAULT_ENDPOINT);
            targets = binding->take_pending_targets();
            const auto safe_valves = binding->apply_cylinder_valves(runtime, targets, true);
            const auto safe_velocity = binding->apply_velocity_drives(runtime, targets, true);
            std::optional<ProcessChannelFault> safe_brake_fault;
            if (binding->apply_brake_coils(runtime, safe_brake_fault, true) != RK_OK) return RK_ERROR_BACKEND;
        }
        if (virtual_devices_[binding_index]) {
            rk_robot_state state{};
            const auto result = virtual_devices_[binding_index]->sample(simulation_ns, state);
            if (result != RK_OK && result != RK_ERROR_STALE_STATE) return result;
            virtual_samples_[binding_index] = state;
            virtual_sample_results_[binding_index] = result;
            const auto positions = virtual_devices_[binding_index]->joint_positions();
            if (positions.size() != binding->joints_.size()) return RK_ERROR_BACKEND;
            for (std::size_t joint = 0; joint < positions.size(); ++joint) {
                if (!binding->actuated_joints_[joint] || binding->device_free_[joint]) continue;
                nksim_joint_target target{};
                target.struct_size = sizeof(target);
                target.joint = binding->joints_[joint];
                target.mode = NKSIM_JOINT_TARGET_POSITION;
                target.target = positions[joint];
                target.max_force = 1e9;
                // A stepper that holds a belt's pulley holds it as a spring of its own stiffness to where the device says
                // the pulley is, not as the default position loop (a softer spring than the belt it drives).
                if (binding->device_hold_stiffness_[joint] > 0.0) {
                    target.mode = NKSIM_JOINT_TARGET_SERVO;
                    target.stiffness = binding->device_hold_stiffness_[joint];
                    target.damping = binding->device_hold_damping_[joint];
                    target.velocity = binding->device_previous_[joint].has_value()
                        ? (positions[joint] - *binding->device_previous_[joint]) / binding->step_ : 0.0;
                    binding->device_previous_[joint] = positions[joint];
                }
                targets.push_back(target);
            }
        }
        if (nksim_session_submit_joint_targets(session_, targets.data(),
                                               static_cast<uint32_t>(targets.size())) != NKSIM_OK)
            return RK_ERROR_BACKEND;
        ++it;
        ++binding_index;
    }
    // Targets taken above are the ones every robot applied for this tick.
    return advance_drives();
}

rk_result Simulation::publish(const nksim_tick &tick) {
    robot_tick_poses_ = robot_base_poses_;
    const auto simulation_ns = tick.step_index * static_cast<std::uint64_t>(period_.count());
    // Every robot publishes from the one snapshot. One that cannot, for
    // instance because a joint is beyond its limit, faults on its own; the
    // robots after it in the list are neither skipped nor failed.
    for (std::size_t index = 0; index < runtimes_.size(); ++index) {
        const auto sample_result = virtual_devices_[index]
            ? runtimes_[index]->publish_presampled(simulation_ns, virtual_samples_[index],
                                                  virtual_sample_results_[index])
            : runtimes_[index]->publish_sample(simulation_ns);
        if (sample_result != RK_OK)
            runtimes_[index]->fail_tick();
    }
    return RK_OK;
}

uint64_t Simulation::step_index() const {
    nksim_session_status status{};
    status.struct_size = sizeof(status);
    return nksim_session_get_status(session_, &status) == NKSIM_OK ? status.step_index : 0;
}

double Simulation::simulation_time() const {
    nksim_session_status status{};
    status.struct_size = sizeof(status);
    return nksim_session_get_status(session_, &status) == NKSIM_OK ? status.simulation_time : 0.0;
}

void Simulation::cleanup() noexcept {
    if (participant_ != 0) nksim_session_remove_participant(session_, participant_);
    participant_ = 0;
    if (session_ == 0) return;
    // A borrowed session keeps its world: remove this simulation's robots from
    // it when it is stopped, or leave them to the world's owner otherwise.
    Lock lock(session_);
    const auto world = stopped_world();
    if (world != 0) {
        // One model recompile for the whole robot, not one per joint and body.
        const bool batched = nksim_world_begin_topology_update(world) == NKSIM_OK;
        for (const auto &owner : object_joints_) nksim_joint_destroy(world, owner.joint);
        for (const auto &owner : robot_joints_) nksim_joint_destroy(world, owner.joint);
        for (const auto joint : joints_) nksim_joint_destroy(world, joint);
        for (const auto body : bodies_) nksim_body_destroy(world, body);
        if (batched) nksim_world_end_topology_update(world);
        for (const auto shape : link_shapes_) nksim_shape_destroy(world, shape);
        if (shape_ != 0) nksim_shape_destroy(world, shape_);
        nkscene_transaction transaction = 0;
        if (!nodes_.empty() && nkscene_transaction_begin(scene_, &transaction) == NKS_OK) {
            for (const auto node : nodes_) nkscene_tx_destroy_node(transaction, node);
            nkscene_change_set changes = 0;
            if (nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK &&
                changes != 0)
                nkscene_change_set_destroy(changes);
        }
    }
    joints_.clear();
    object_joints_.clear();
    robot_joints_.clear();
    object_carriers_.clear();
    bodies_.clear();
    link_shapes_.clear();
    nodes_.clear();
    shape_ = 0;
}

} // namespace robotkit
