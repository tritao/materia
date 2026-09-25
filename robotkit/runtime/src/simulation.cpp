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

nkscene_transform robot_transform(uint32_t robot_index) {
    nkscene_transform transform{};
    transform.matrix[0] = transform.matrix[5] = transform.matrix[10] =
        transform.matrix[15] = 1.0f;
    transform.matrix[12] = static_cast<float>(robot_index);
    return transform;
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
        nkscene_transaction transaction = 0;
        require_scene(nkscene_transaction_begin(scene_, &transaction),
                      "nkscene_transaction_begin");
        for (uint32_t index = 0; index < blueprint.link_count; ++index) {
            nkscene_node_id node{};
            if (nkscene_tx_create_node(transaction, &node) != NKS_OK) {
                nkscene_transaction_cancel(transaction);
                throw std::runtime_error("nkscene_tx_create_node");
            }
            const auto transform = robot_transform(robot_index);
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
        initial_pose.rotation[3]=1.0;robot_initial_poses_.push_back(initial_pose);
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

rk_result Simulation::teleport_robot(uint32_t robot_index, const rk_simulation_pose &pose) {
    std::lock_guard tick_lock(tick_mutex_);
    if (running_ || stopping_ || host_ != 0 || robot_index >= robot_base_bodies_.size())
        return RK_ERROR_INVALID_STATE;
    if (pose.struct_size < sizeof(pose)) return RK_ERROR_INVALID_ARGUMENT;
    auto binding = bindings_[robot_index].lock();
    if (!binding) return RK_ERROR_INVALID_HANDLE;
    const auto root = std::find(binding->bodies_.begin(), binding->bodies_.end(),
                                robot_base_bodies_[robot_index]);
    if (root == binding->bodies_.end()) return RK_ERROR_INVALID_HANDLE;
    const auto root_index = static_cast<std::size_t>(root - binding->bodies_.begin());
    auto result = set_node_pose(scene_, binding->nodes_[root_index],
                                      pose.position, pose.rotation);
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
    nksim_step_result result{};
    result.struct_size = sizeof(result);
    if (nksim_host_step(host_, &result) != NKSIM_OK)
        return RK_ERROR_BACKEND;
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
