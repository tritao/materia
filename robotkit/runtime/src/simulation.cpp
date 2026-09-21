#include "simulation.hpp"
#include "simulation_robot.hpp"
#include "runtime_registry.hpp"

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

Simulation::Simulation(double fixed_timestep, uint32_t physics_substeps)
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
        require_sim(nksim_world_create(&desc, &world_), "nksim_world_create");
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
            desc.motion_type = index == 0 ? NKSIM_MOTION_STATIC : NKSIM_MOTION_DYNAMIC;
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

        rk_robot_runtime_layout layout{};
        layout.struct_size = sizeof(layout);
        layout.revision = blueprint.revision;
        layout.joint_count = blueprint.joint_count;
        layout.link_count = blueprint.link_count;
        layout.frame_count = blueprint.frame_count;
        auto runtime = std::make_shared<RobotRuntime>(
            layout, std::static_pointer_cast<RobotEndpoint>(binding));
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
