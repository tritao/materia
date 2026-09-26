#include "simulation.hpp"
#include "simulation_robot.hpp"
#include "runtime_registry.hpp"
#include "sensor_math.hpp"
#ifdef RK_HAS_MUJOCO
#include "nativekit_sim_mujoco.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace robotkit {
namespace {

void require_sim(nksim_result result, const char *operation) {
    if (result != NKSIM_OK)
        throw std::runtime_error(operation);
}

void require_scene(nkscene_result result, const char *operation) {
    if (result != NKS_OK)
        throw std::runtime_error(operation);
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

uint64_t monotonic_now_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

} // namespace

Simulation::Simulation(double fixed_timestep, uint32_t physics_substeps, uint32_t backend)
    : fixed_timestep_(fixed_timestep), physics_substeps_(physics_substeps),
      period_(static_cast<int64_t>(fixed_timestep * 1'000'000'000.0)) {
    if (fixed_timestep <= 0.0 || physics_substeps == 0)
        throw std::invalid_argument("invalid simulation timing");
    try {
        require_scene(nkscene_scene_create(&scene_), "nkscene_scene_create");
        nksim_world_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.scene = scene_;
        desc.fixed_timestep = fixed_timestep_;
        desc.physics_substeps = physics_substeps_;
        std::copy_n(gravity_, 3, desc.gravity);
        if (backend == 0)
            require_sim(nksim_world_create(&desc, &world_), "nksim_world_create");
#ifdef RK_HAS_MUJOCO
        else if (backend == 1)
            require_sim(nksim_mujoco_world_create(&desc, &world_), "nksim_mujoco_world_create");
#endif
        else throw std::invalid_argument("simulation backend unavailable");
        const double half_extents[] = {0.05, 0.05, 0.05};
        require_sim(nksim_shape_create_box(world_, half_extents, &shape_),
                    "nksim_shape_create_box");
    } catch (...) {
        cleanup();
        throw;
    }
}

Simulation::~Simulation() {
    stop();
    runtimes_.clear();
    for (const auto handle : handles_)
        internal::destroy_runtime(handle);
    handles_.clear();
    cleanup();
}

rk_result Simulation::add_robot(const rk_robot_runtime_blueprint &blueprint,
                                rk_robot_runtime &out_runtime) {
    std::lock_guard tick_lock(tick_mutex_);
    bool topology_update = false;
    {
        std::lock_guard state_lock(state_mutex_);
        if (sealed_ || running_)
            return RK_ERROR_INVALID_STATE;
    }
    if (topology_frozen_ || rk_robot_runtime_blueprint_validate(&blueprint) != RK_OK ||
        blueprint.link_count == 0)
        return RK_ERROR_INVALID_ARGUMENT;
    try {
        auto binding = std::shared_ptr<SimulationRobot>(new SimulationRobot(*this));
        const auto robot_index = static_cast<uint32_t>(bindings_.size());
        uint32_t root = 0;
        for (uint32_t candidate = 0; candidate < blueprint.link_count; ++candidate) {
            bool child = false;
            for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint)
                child = child || blueprint.joints[joint].child_link == candidate;
            if (!child) { root = candidate; break; }
        }
        const auto rest_poses = link_rest_poses(blueprint, robot_index, root);
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

        require_sim(nksim_world_begin_topology_update(world_),
                    "nksim_world_begin_topology_update");
        topology_update = true;
        for (uint32_t index = 0; index < blueprint.link_count; ++index) {
            nksim_body_desc desc{};
            desc.struct_size = sizeof(desc);
            desc.node = binding->nodes_[index];
            desc.motion_type = index == root ? NKSIM_MOTION_KINEMATIC : NKSIM_MOTION_DYNAMIC;
            desc.mass = blueprint.links[index].mass;
            desc.has_inertial_properties = 1;
            std::copy_n(blueprint.links[index].center_of_mass, 3, desc.center_of_mass);
            std::copy_n(blueprint.links[index].inertia_tensor, 9, desc.inertia_tensor);
            desc.shape = shape_;
            desc.collision_layer = 1;
            desc.collision_mask = blueprint.collision_approximation == RK_COLLISION_APPROXIMATION_NONE ? 0 : 1;
            nksim_body body = 0;
            require_sim(nksim_body_create(world_, &desc, &body), "nksim_body_create");
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
            desc.lower_limit = source.lower_limit;
            desc.upper_limit = source.upper_limit;
            desc.max_force = source.max_effort;
            nksim_joint joint = 0;
            require_sim(nksim_joint_create(world_, &desc, &joint), "nksim_joint_create");
            binding->joints_.push_back(joint);
            binding->actuated_joints_.push_back(source.type != RK_RUNTIME_JOINT_FIXED);
            joints_.push_back(joint);
        }
        const auto topology_result = nksim_world_end_topology_update(world_);
        topology_update = false;
        require_sim(topology_result, "nksim_world_end_topology_update");
        bindings_.push_back(binding);
        binding->base_body_ = binding->bodies_[root];
        robot_base_bodies_.push_back(binding->base_body_);
        rk_simulation_pose initial_pose{};initial_pose.struct_size=sizeof(initial_pose);
        // Matches robot_transform(), the authored node pose kinematic bases start from.
        initial_pose.position[0]=static_cast<double>(robot_index);
        initial_pose.rotation[3]=1.0;robot_initial_poses_.push_back(initial_pose);
        robot_base_poses_.push_back(initial_pose);
        robot_tick_poses_.push_back(initial_pose);
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
            binding->sensors_.push_back(sensor);
        }
        binding->reset_sensors();

        auto runtime = std::make_shared<RobotRuntime>(
            blueprint, std::static_pointer_cast<RobotEndpoint>(binding), period_);
        runtime->set_externally_driven(true);
        const auto handle = internal::register_runtime(runtime);
        runtimes_.push_back(std::move(runtime));
        handles_.push_back(handle);
        out_runtime = handle;
        return RK_OK;
    } catch (const std::bad_alloc &) {
        if (topology_update)
            (void)nksim_world_end_topology_update(world_);
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        if (topology_update)
            (void)nksim_world_end_topology_update(world_);
        return RK_ERROR_BACKEND;
    }
}

namespace { rk_result set_body_pose(nksim_world,nksim_body,const double[3],const double[4]); }

rk_result Simulation::reset() {
    std::lock_guard tick_lock(tick_mutex_);
    {
        std::lock_guard state_lock(state_mutex_);
        if (running_ || stopping_)
            return RK_ERROR_INVALID_STATE;
    }
    if (host_ != 0)
        return RK_ERROR_INVALID_STATE;
    if (nksim_world_reset(world_) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    if (snapshot_ != 0)
        nksim_snapshot_destroy(snapshot_);
    snapshot_ = 0;
    step_index_ = 0;
    simulation_time_ = 0.0;
    for (std::size_t index = 0; index < runtimes_.size(); ++index) {
        if(set_body_pose(world_,robot_base_bodies_[index],robot_initial_poses_[index].position,
            robot_initial_poses_[index].rotation)!=RK_OK)return RK_ERROR_BACKEND;
        // Kinematic bases follow their scene node on every tick, so a pose
        // driven by drive_robot_base() or a drive plant is restored there too.
        drives_[index].yaw = 0.0;
        std::fill(std::begin(drives_[index].rates), std::end(drives_[index].rates), 0.0);
        if (set_robot_base_node_pose(static_cast<uint32_t>(index),
                                     robot_initial_poses_[index]) != RK_OK)
            return RK_ERROR_BACKEND;
        if (auto binding = bindings_[index].lock()) binding->reset();
        runtimes_[index]->reset_state();
    }
    for (auto &object : objects_)
        if (object.active && set_body_pose(world_, object.body, object.initial_pose.position,
                                          object.initial_pose.rotation) != RK_OK)
            return RK_ERROR_BACKEND;
    return RK_OK;
}

rk_result Simulation::reset_robot(uint32_t robot_index) {
    std::lock_guard tick_lock(tick_mutex_);
    if (running_ || stopping_ || host_ != 0 || robot_index >= bindings_.size())
        return RK_ERROR_INVALID_STATE;
    auto binding = bindings_[robot_index].lock();
    if (!binding)
        return RK_ERROR_INVALID_HANDLE;
    for (auto body : binding->bodies_)
        if (nksim_body_reset(world_, body) != NKSIM_OK)
            return RK_ERROR_BACKEND;
    if(set_body_pose(world_,robot_base_bodies_[robot_index],robot_initial_poses_[robot_index].position,
        robot_initial_poses_[robot_index].rotation)!=RK_OK)return RK_ERROR_BACKEND;
    auto &drive = drives_[robot_index];
    drive.yaw = 0.0;
    std::fill(std::begin(drive.rates), std::end(drive.rates), 0.0);
    if (set_robot_base_node_pose(robot_index, robot_initial_poses_[robot_index]) != RK_OK)
        return RK_ERROR_BACKEND;
    binding->reset();
    runtimes_[robot_index]->reset_state();
    if (snapshot_ != 0) { nksim_snapshot_destroy(snapshot_); snapshot_ = 0; }
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
    const auto driven = host_ != 0 ? nksim_host_submit_body_drives(host_, &state, 1)
                                   : nksim_body_drive(world_, state.body, &state);
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
            const double left = drive.rates[0] * drive.wheel_radius;
            const double right = drive.rates[1] * drive.wheel_radius;
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
    std::lock_guard tick_lock(tick_mutex_);
    if (desc.struct_size < sizeof(desc) || robot_index >= drives_.size() ||
        !valid_drive_geometry(desc.wheel_radius) || !valid_drive_geometry(desc.track_width) ||
        desc.left_wheel_joint == desc.right_wheel_joint)
        return RK_ERROR_INVALID_ARGUMENT;
    const uint32_t joints[2] = {desc.left_wheel_joint, desc.right_wheel_joint};
    const auto valid = valid_wheel_joints(robot_index, joints, 2);
    if (valid != RK_OK) return valid;
    auto &drive = drives_[robot_index];
    drive.kind = DrivePlant::Kind::Differential;
    drive.wheel_count = 2;
    std::copy_n(joints, 2, drive.joints);
    drive.wheel_radius = desc.wheel_radius;
    drive.track_width = desc.track_width;
    std::fill(std::begin(drive.rates), std::end(drive.rates), 0.0);
    return set_robot_base_node_pose(robot_index, robot_base_poses_[robot_index]);
}

rk_result Simulation::set_omni_drive(uint32_t robot_index,
                                     const rk_simulation_omni_drive_desc &desc) {
    std::lock_guard tick_lock(tick_mutex_);
    if (desc.struct_size < sizeof(desc) || robot_index >= drives_.size() ||
        !valid_drive_geometry(desc.wheel_radius) || !valid_drive_geometry(desc.base_radius))
        return RK_ERROR_INVALID_ARGUMENT;
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
    std::lock_guard tick_lock(tick_mutex_);
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
    std::lock_guard tick_lock(tick_mutex_);
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
    std::lock_guard tick_lock(tick_mutex_);
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
    std::lock_guard tick_lock(tick_mutex_);
    if (robot_index >= robot_base_bodies_.size()) return RK_ERROR_INVALID_ARGUMENT;
    if (pose.struct_size < sizeof(pose) || !valid_pose(pose.position, pose.rotation))
        return RK_ERROR_INVALID_ARGUMENT;
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
    std::lock_guard tick_lock(tick_mutex_);
    if (robot_index >= robot_base_bodies_.size() || pose.struct_size < sizeof(pose) ||
        !valid_pose(pose.position, pose.rotation))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto body = robot_base_bodies_[robot_index];
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    bool found = false;
    if (host_ != 0) {
        nksim_snapshot latest = 0;
        if (nksim_host_get_snapshot(host_, &latest) != NKSIM_OK) return RK_ERROR_BACKEND;
        uint64_t count = 0;
        if (nksim_snapshot_get_body_count(latest, &count) == NKSIM_OK)
            for (uint64_t index = 0; index < count && !found; ++index)
                found = nksim_snapshot_get_body(latest, index, &state) == NKSIM_OK &&
                    state.body == body;
        nksim_snapshot_destroy(latest);
    } else {
        found = nksim_body_get_state(world_, body, &state) == NKSIM_OK;
    }
    if (!found) return RK_ERROR_BACKEND;
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
    const auto written = host_ != 0
        ? nksim_host_submit_body_states(host_, &state, 1)
        : nksim_body_set_state(world_, body, &state);
    if (written != NKSIM_OK) return RK_ERROR_BACKEND;
    return set_robot_base_node_pose(robot_index, pose);
}

rk_result Simulation::teleport_robot(uint32_t robot_index, const rk_simulation_pose &pose) {
    std::lock_guard tick_lock(tick_mutex_);
    if (running_ || stopping_ || host_ != 0 || robot_index >= robot_base_bodies_.size())
        return RK_ERROR_INVALID_STATE;
    if (pose.struct_size < sizeof(pose)) return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    auto result = set_robot_base_node_pose(robot_index, pose);
    if (result == RK_OK)
        result = set_body_pose(world_, robot_base_bodies_[robot_index], pose.position, pose.rotation);
    if (result == RK_OK) {
        binding->reset_sensors();
        robot_initial_poses_[robot_index] = pose;
        if (snapshot_ != 0) { nksim_snapshot_destroy(snapshot_); snapshot_ = 0; }
    }
    return result;
}

rk_result Simulation::get_robot_pose(uint32_t robot_index,rk_simulation_pose &out_pose) const {
    std::lock_guard tick_lock(tick_mutex_);
    if(out_pose.struct_size<sizeof(out_pose)||robot_index>=robot_base_bodies_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    return read_body_pose(robot_base_bodies_[robot_index],out_pose);
}

rk_result Simulation::get_link_pose(uint32_t robot_index,uint32_t link_index,
                                     rk_simulation_pose &out_pose) const {
    std::lock_guard tick_lock(tick_mutex_);
    if(out_pose.struct_size<sizeof(out_pose)||robot_index>=bindings_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    const auto binding=bindings_[robot_index].lock();
    if(!binding)return RK_ERROR_INVALID_HANDLE;
    if(link_index>=binding->bodies_.size())return RK_ERROR_INVALID_ARGUMENT;
    return read_body_pose(binding->bodies_[link_index],out_pose);
}

rk_result Simulation::read_body_pose(nksim_body body,rk_simulation_pose &out_pose) const {
    nksim_body_state state{};state.struct_size=sizeof(state);
    bool found=false;
    if(snapshot_!=0){
        uint64_t count=0;
        if(nksim_snapshot_get_body_count(snapshot_,&count)==NKSIM_OK)for(uint64_t index=0;index<count;++index){
            nksim_body_state candidate{};candidate.struct_size=sizeof(candidate);
            if(nksim_snapshot_get_body(snapshot_,index,&candidate)!=NKSIM_OK)break;
            if(candidate.body==body){state=candidate;found=true;break;}
        }
    } else if(nksim_body_get_state(world_,body,&state)==NKSIM_OK)found=true;
    if(!found)return RK_ERROR_BACKEND;
    std::copy_n(state.position,3,out_pose.position);
    std::copy_n(state.rotation,4,out_pose.rotation);
    return RK_OK;
}

rk_result Simulation::spawn_object(const rk_simulation_object_desc &desc,
                                   rk_simulation_object &out_object) {
    std::lock_guard tick_lock(tick_mutex_);
    out_object = RK_INVALID_SIMULATION_OBJECT;
    if (running_ || stopping_ || host_ != 0 || desc.struct_size < sizeof(desc) ||
        desc.motion_type > NKSIM_MOTION_DYNAMIC ||
        (desc.motion_type == NKSIM_MOTION_DYNAMIC && desc.mass <= 0.0) ||
        !valid_pose(desc.position, desc.rotation))
        return RK_ERROR_INVALID_ARGUMENT;
    for (int index = 0; index < 3; ++index)
        if (!std::isfinite(desc.half_extents[index]) || desc.half_extents[index] <= 0.0)
            return RK_ERROR_INVALID_ARGUMENT;
    nkscene_transaction transaction = 0;
    if (nkscene_transaction_begin(scene_, &transaction) != NKS_OK)
        return RK_ERROR_BACKEND;
    EnvironmentObject object;
    std::copy_n(desc.half_extents, 3, object.half_extents);
    object.initial_pose.struct_size = sizeof(object.initial_pose);
    std::copy_n(desc.position, 3, object.initial_pose.position);
    std::copy_n(desc.rotation, 4, object.initial_pose.rotation);
    nkscene_transform transform{};
    transform.matrix[0] = transform.matrix[5] = transform.matrix[10] = transform.matrix[15] = 1.0f;
    for (int column = 0; column < 3; ++column) {
        double axis[3]{};
        axis[column] = 1.0;
        double rotated[3];
        sensors::rotate(desc.rotation, axis, rotated);
        for (int row = 0; row < 3; ++row)
            transform.matrix[column * 4 + row] = static_cast<float>(rotated[row]);
    }
    transform.matrix[12] = static_cast<float>(desc.position[0]);
    transform.matrix[13] = static_cast<float>(desc.position[1]);
    transform.matrix[14] = static_cast<float>(desc.position[2]);
    if (nkscene_tx_create_node(transaction, &object.node) != NKS_OK ||
        nkscene_tx_set_transform(transaction, object.node, &transform) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        return RK_ERROR_BACKEND;
    }
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
        return RK_ERROR_BACKEND;
    if (changes != 0) nkscene_change_set_destroy(changes);
    const double half_extents[] = {desc.half_extents[0], desc.half_extents[1], desc.half_extents[2]};
    if (nksim_shape_create_box(world_, half_extents, &object.shape) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.node = object.node;
    body_desc.motion_type = desc.motion_type;
    body_desc.mass = desc.mass;
    body_desc.shape = object.shape;
    body_desc.collision_layer = body_desc.collision_mask = 1;
    if (nksim_body_create(world_, &body_desc, &object.body) != NKSIM_OK) {
        nksim_shape_destroy(world_, object.shape);
        return RK_ERROR_BACKEND;
    }
    object.active = true;
    objects_.push_back(object);
    out_object = static_cast<rk_simulation_object>(objects_.size());
    if (snapshot_ != 0) { nksim_snapshot_destroy(snapshot_); snapshot_ = 0; }
    return RK_OK;
}

rk_result Simulation::remove_object(rk_simulation_object object) {
    std::lock_guard tick_lock(tick_mutex_);
    if (running_ || stopping_ || host_ != 0 || object == 0 || object > objects_.size())
        return RK_ERROR_INVALID_STATE;
    auto &value = objects_[object - 1];
    if (!value.active)
        return RK_ERROR_INVALID_HANDLE;
    nksim_body_destroy(world_, value.body);
    nksim_shape_destroy(world_, value.shape);
    nkscene_transaction transaction = 0;
    if (nkscene_transaction_begin(scene_, &transaction) != NKS_OK)
        return RK_ERROR_BACKEND;
    if (nkscene_tx_destroy_node(transaction, value.node) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        return RK_ERROR_BACKEND;
    }
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
        return RK_ERROR_BACKEND;
    if (changes != 0) nkscene_change_set_destroy(changes);
    value.active = false;
    if (snapshot_ != 0) { nksim_snapshot_destroy(snapshot_); snapshot_ = 0; }
    return RK_OK;
}

rk_result Simulation::teleport_object(rk_simulation_object object,
                                       const rk_simulation_pose &pose) {
    std::lock_guard tick_lock(tick_mutex_);
    if (running_ || stopping_ || host_ != 0)
        return RK_ERROR_INVALID_STATE;
    if (object == 0 || object > objects_.size() || !objects_[object - 1].active)
        return RK_ERROR_INVALID_HANDLE;
    if (pose.struct_size < sizeof(pose)) return RK_ERROR_INVALID_ARGUMENT;
    const auto result = set_body_pose(world_, objects_[object - 1].body,
                                      pose.position, pose.rotation);
    if (result == RK_OK) {
        objects_[object - 1].initial_pose = pose;
        if (snapshot_ != 0) { nksim_snapshot_destroy(snapshot_); snapshot_ = 0; }
    }
    return result;
}

rk_result Simulation::get_object_pose(rk_simulation_object object,
                                      rk_simulation_pose &out_pose) const {
    std::lock_guard tick_lock(tick_mutex_);
    if(out_pose.struct_size<sizeof(out_pose)||object==0||object>objects_.size())
        return RK_ERROR_INVALID_ARGUMENT;
    const auto &value=objects_[object-1];
    return value.active?read_body_pose(value.body,out_pose):RK_ERROR_INVALID_HANDLE;
}

rk_result Simulation::capture_presentation(rk_simulation_presentation_info &out_info,
        std::vector<rk_simulation_presentation_pose> &out_poses) const {
    std::lock_guard tick_lock(tick_mutex_);
    if (out_info.struct_size < sizeof(out_info)) return RK_ERROR_INVALID_ARGUMENT;
    std::unordered_map<nksim_body, rk_simulation_pose> body_poses;
    if (snapshot_ != 0) {
        uint64_t count = 0;
        if (nksim_snapshot_get_body_count(snapshot_, &count) != NKSIM_OK)
            return RK_ERROR_BACKEND;
        body_poses.reserve(static_cast<std::size_t>(count));
        for (uint64_t index = 0; index < count; ++index) {
            nksim_body_state state{};
            state.struct_size = sizeof(state);
            if (nksim_snapshot_get_body(snapshot_, index, &state) != NKSIM_OK)
                return RK_ERROR_BACKEND;
            rk_simulation_pose pose{};
            pose.struct_size = sizeof(pose);
            std::copy_n(state.position, 3, pose.position);
            std::copy_n(state.rotation, 4, pose.rotation);
            body_poses.emplace(state.body, pose);
        }
    }
    auto readPose = [&](nksim_body body, rk_simulation_pose &pose) -> rk_result {
        if (snapshot_ == 0) return read_body_pose(body, pose);
        const auto found = body_poses.find(body);
        if (found == body_poses.end()) return RK_ERROR_BACKEND;
        pose = found->second;
        return RK_OK;
    };
    std::size_t pose_count = robot_base_bodies_.size();
    for (const auto &binding : bindings_) {
        const auto robot = binding.lock();
        if (robot) pose_count += robot->bodies_.size();
    }
    for (const auto &object : objects_) if (object.active) ++pose_count;
    if (pose_count > std::numeric_limits<uint32_t>::max()) return RK_ERROR_OUT_OF_MEMORY;
    out_poses.clear();
    out_poses.reserve(pose_count);
    for (uint32_t robot_index = 0; robot_index < robot_base_bodies_.size(); ++robot_index) {
        rk_simulation_presentation_pose item{};
        item.struct_size = sizeof(item);
        item.kind = RK_SIMULATION_PRESENTATION_ROBOT_BASE;
        item.robot_index = robot_index;
        rk_simulation_pose pose{};
        pose.struct_size = sizeof(pose);
        auto result = readPose(robot_base_bodies_[robot_index], pose);
        if (result != RK_OK) return result;
        std::copy_n(pose.position, 3, item.position);
        std::copy_n(pose.rotation, 4, item.rotation);
        out_poses.push_back(item);
        const auto binding = bindings_[robot_index].lock();
        if (!binding) return RK_ERROR_INVALID_HANDLE;
        for (uint32_t link_index = 0; link_index < binding->bodies_.size(); ++link_index) {
            item = {};
            item.struct_size = sizeof(item);
            item.kind = RK_SIMULATION_PRESENTATION_ROBOT_LINK;
            item.robot_index = robot_index;
            item.link_index = link_index;
            pose = {};
            pose.struct_size = sizeof(pose);
            result = readPose(binding->bodies_[link_index], pose);
            if (result != RK_OK) return result;
            std::copy_n(pose.position, 3, item.position);
            std::copy_n(pose.rotation, 4, item.rotation);
            out_poses.push_back(item);
        }
    }
    for (uint32_t index = 0; index < objects_.size(); ++index) {
        const auto &object = objects_[index];
        if (!object.active) continue;
        rk_simulation_presentation_pose item{};
        item.struct_size = sizeof(item);
        item.kind = RK_SIMULATION_PRESENTATION_ENVIRONMENT;
        item.object_id = index + 1;
        rk_simulation_pose pose{};
        pose.struct_size = sizeof(pose);
        const auto result = readPose(object.body, pose);
        if (result != RK_OK) return result;
        std::copy_n(pose.position, 3, item.position);
        std::copy_n(pose.rotation, 4, item.rotation);
        out_poses.push_back(item);
    }
    out_info.step_index = step_index_;
    out_info.simulation_time = simulation_time_;
    out_info.pose_count = static_cast<uint32_t>(out_poses.size());
    return RK_OK;
}

rk_result Simulation::step(uint64_t timestamp_ns) {
    std::lock_guard tick_lock(tick_mutex_);
    {
        std::lock_guard state_lock(state_mutex_);
        if (running_ || stopping_ || runtimes_.empty())
            return RK_ERROR_INVALID_STATE;
        sealed_ = true;
    }
    for (const auto &runtime : runtimes_) {
        const auto result = runtime->apply_pending_commands();
        if (result != RK_OK) {
            for (const auto &participant : runtimes_)
                participant->discard_pending_commands();
            return result;
        }
    }
    return advance(timestamp_ns);
}

rk_result Simulation::advance(uint64_t timestamp_ns) {
    if (ensure_host() != RK_OK)
        return RK_ERROR_BACKEND;
    for (auto it = bindings_.begin(); it != bindings_.end();) {
        const auto binding = it->lock();
        if (!binding) {
            it = bindings_.erase(it);
            continue;
        }
        auto targets = binding->take_pending_targets();
        if (!targets.empty() && nksim_host_submit_joint_targets(
                                    host_, targets.data(),
                                    static_cast<uint32_t>(targets.size())) != NKSIM_OK)
            return RK_ERROR_BACKEND;
        ++it;
    }
    // Targets taken above are the ones every robot applied for this tick.
    const auto drive_result = advance_drives();
    if (drive_result != RK_OK)
        return drive_result;
    nksim_step_result result{};
    result.struct_size = sizeof(result);
    if (nksim_host_step(host_, &result) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    robot_tick_poses_ = robot_base_poses_;
    if (result.scene_changes != 0)
        nkscene_change_set_destroy(result.scene_changes);
    if (snapshot_ != 0)
        nksim_snapshot_destroy(snapshot_);
    snapshot_ = 0;
    if (nksim_host_get_snapshot(host_, &snapshot_) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    step_index_ = result.step_index;
    simulation_time_ = result.simulation_time;
    for (const auto &runtime : runtimes_) {
        const auto sample_result = runtime->publish_sample(timestamp_ns);
        if (sample_result != RK_OK)
            return sample_result;
    }
    return RK_OK;
}

rk_result Simulation::start() {
    std::lock_guard tick_lock(tick_mutex_);
    std::lock_guard state_lock(state_mutex_);
    if (running_ || stopping_ || runtimes_.empty())
        return RK_ERROR_INVALID_STATE;
    if (ensure_host() != RK_OK)
        return RK_ERROR_BACKEND;
    sealed_ = true;
    running_ = true;
    worker_ = std::thread([this] { run(); });
    return RK_OK;
}

rk_result Simulation::ensure_host() {
    if (host_ != 0)
        return RK_OK;
    nksim_host_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.world = world_;
    desc.mode = NKSIM_HOST_MODE_EXTERNAL;
    if (nksim_host_create(&desc, &host_) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    if (nksim_host_start(host_) != NKSIM_OK) {
        nksim_host_destroy(host_);
        host_ = 0;
        return RK_ERROR_BACKEND;
    }
    topology_frozen_ = true;
    return RK_OK;
}

rk_result Simulation::stop() {
    {
        std::lock_guard state_lock(state_mutex_);
        if (!running_ && !worker_.joinable() && host_ == 0)
            return RK_OK;
        stopping_ = true;
    }
    if (worker_.joinable())
        worker_.join();
    std::lock_guard state_lock(state_mutex_);
    running_ = false;
    stopping_ = false;
    if (host_ == 0)
        return RK_OK;
    const auto result = nksim_host_stop(host_);
    nksim_host_destroy(host_);
    host_ = 0;
    return result == NKSIM_OK ? RK_OK : RK_ERROR_BACKEND;
}

void Simulation::run() {
    auto next_tick = std::chrono::steady_clock::now();
    for (;;) {
        {
            std::lock_guard state_lock(state_mutex_);
            if (stopping_)
                break;
        }
        next_tick += period_;
        {
            std::lock_guard tick_lock(tick_mutex_);
            bool commands_valid = true;
            for (const auto &runtime : runtimes_) {
                if (runtime->apply_pending_commands() != RK_OK) {
                    commands_valid = false;
                    break;
                }
            }
            if (!commands_valid) {
                for (const auto &runtime : runtimes_)
                    runtime->discard_pending_commands();
            } else if (advance(monotonic_now_ns()) != RK_OK) {
                // The owner thread remains alive; the published runtime state
                // records the endpoint fault and callers can stop the session.
                break;
            }
        }
        std::this_thread::sleep_until(next_tick);
    }
}

uint64_t Simulation::step_index() const {
    std::lock_guard lock(tick_mutex_);
    return step_index_;
}

double Simulation::simulation_time() const {
    std::lock_guard lock(tick_mutex_);
    return simulation_time_;
}

void Simulation::cleanup() noexcept {
    stop();
    if (snapshot_ != 0)
        nksim_snapshot_destroy(snapshot_);
    snapshot_ = 0;
    if (world_ != 0) {
        // Destroying the world tears down the complete backend at once. Per-
        // handle destruction here recompiles whole-model backends for every
        // link immediately before the model itself is discarded.
        if (shape_ != 0)
            nksim_shape_destroy(world_, shape_);
        nksim_world_destroy(world_);
        world_ = 0;
    }
    if (scene_ != 0)
        nkscene_scene_destroy(scene_);
    scene_ = 0;
}

} // namespace robotkit
