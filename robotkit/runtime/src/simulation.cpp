#include "simulation.hpp"
#include "simulation_robot.hpp"
#include "runtime_registry.hpp"
#include "sensor_math.hpp"
#ifdef RK_HAS_MUJOCO
#include "nativekit_sim_mujoco.h"
#endif

#include <algorithm>
#include <cmath>
#include <stdexcept>

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
            nkscene_occurrence_id occurrence{};
            if (nkscene_tx_create_occurrence(transaction, &occurrence) != NKS_OK) {
                nkscene_transaction_cancel(transaction);
                throw std::runtime_error("nkscene_tx_create_occurrence");
            }
            const auto transform = robot_transform(robot_index);
            if (nkscene_tx_set_transform(transaction, occurrence, &transform) != NKS_OK) {
                nkscene_transaction_cancel(transaction);
                throw std::runtime_error("nkscene_tx_set_transform");
            }
            binding->occurrences_.push_back(occurrence);
            occurrences_.push_back(occurrence);
        }
        nkscene_change_set changes = 0;
        require_scene(nkscene_transaction_commit_with_changes(transaction, &changes),
                      "nkscene_transaction_commit_with_changes");
        if (changes != 0)
            nkscene_change_set_destroy(changes);

        for (uint32_t index = 0; index < blueprint.link_count; ++index) {
            nksim_body_desc desc{};
            desc.struct_size = sizeof(desc);
            desc.occurrence = binding->occurrences_[index];
            desc.motion_type = index == root ? NKSIM_MOTION_STATIC : NKSIM_MOTION_DYNAMIC;
            desc.mass = 1.0;
            desc.shape = shape_;
            desc.collision_layer = desc.collision_mask = 1;
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
            desc.axis_a[2] = 1.0;
            desc.lower_limit = source.lower_limit;
            desc.upper_limit = source.upper_limit;
            desc.max_force = source.max_effort;
            nksim_joint joint = 0;
            require_sim(nksim_joint_create(world_, &desc, &joint), "nksim_joint_create");
            binding->joints_.push_back(joint);
            joints_.push_back(joint);
        }
        bindings_.push_back(binding);
        binding->base_body_ = binding->bodies_[root];
        robot_base_bodies_.push_back(binding->base_body_);
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
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
}

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
        if (auto binding = bindings_[index].lock()) binding->reset();
        runtimes_[index]->reset_state();
    }
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

} // namespace

rk_result Simulation::teleport_robot(uint32_t robot_index, const rk_simulation_pose &pose) {
    std::lock_guard tick_lock(tick_mutex_);
    if (running_ || stopping_ || host_ != 0 || robot_index >= robot_base_bodies_.size())
        return RK_ERROR_INVALID_STATE;
    if (pose.struct_size < sizeof(pose)) return RK_ERROR_INVALID_ARGUMENT;
    const auto result = set_body_pose(world_, robot_base_bodies_[robot_index], pose.position, pose.rotation);
    if (result == RK_OK)
        if (auto binding = bindings_[robot_index].lock()) binding->reset_sensors();
    return result;
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
    if (nkscene_tx_create_occurrence(transaction, &object.occurrence) != NKS_OK ||
        nkscene_tx_set_transform(transaction, object.occurrence, &transform) != NKS_OK) {
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
    body_desc.occurrence = object.occurrence;
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
    if (nkscene_tx_destroy_occurrence(transaction, value.occurrence) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        return RK_ERROR_BACKEND;
    }
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
        return RK_ERROR_BACKEND;
    if (changes != 0) nkscene_change_set_destroy(changes);
    value.active = false;
    return RK_OK;
}

rk_result Simulation::teleport_object(rk_simulation_object object,
                                       const rk_simulation_pose &pose) {
    std::lock_guard tick_lock(tick_mutex_);
    if (running_ || stopping_ || host_ != 0)
        return RK_ERROR_INVALID_STATE;
    if (object == 0 || object > objects_.size() || !objects_[object - 1].active)
        return RK_ERROR_INVALID_HANDLE;
    return pose.struct_size < sizeof(pose) ? RK_ERROR_INVALID_ARGUMENT
        : set_body_pose(world_, objects_[object - 1].body, pose.position, pose.rotation);
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
        for (auto joint : joints_)
            nksim_joint_destroy(world_, joint);
        for (auto body : bodies_)
            nksim_body_destroy(world_, body);
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
