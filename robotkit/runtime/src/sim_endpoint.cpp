#include "sim_endpoint.hpp"

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
} // namespace

SimWorld::SimWorld(double fixed_timestep, uint32_t physics_substeps)
    : fixed_timestep_(fixed_timestep), physics_substeps_(physics_substeps) {
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

SimWorld::~SimWorld() { cleanup(); }

std::shared_ptr<SimRobotBinding>
SimWorld::add_robot(const rk_runtime_blueprint &blueprint) {
    if (topology_frozen_ || rk_runtime_blueprint_validate(&blueprint) != RK_OK ||
        blueprint.link_count == 0)
        throw std::invalid_argument("frozen topology or invalid blueprint");

    auto binding = std::shared_ptr<SimRobotBinding>(new SimRobotBinding(*this));
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
    return binding;
}

rk_result SimWorld::start() {
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

rk_result SimWorld::stop() {
    if (host_ == 0)
        return RK_OK;
    const auto result = nksim_host_stop(host_);
    nksim_host_destroy(host_);
    host_ = 0;
    return result == NKSIM_OK ? RK_OK : RK_ERROR_BACKEND;
}

rk_result SimWorld::step() {
    if (start() != RK_OK)
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
    return RK_OK;
}

void SimWorld::cleanup() noexcept {
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

rk_result SimRobotBinding::apply(const rk_robot_command &command) {
    if (command.kind == RK_COMMAND_EMERGENCY_STOP) {
        stopped_ = true;
        pending_targets_.clear();
        return RK_OK;
    }
    if (command.kind == RK_COMMAND_STOP) {
        stopped_ = false;
        pending_targets_.clear();
        return RK_OK;
    }
    if (stopped_)
        return RK_ERROR_SAFETY_STOPPED;
    if (command.kind == RK_COMMAND_NONE)
        return RK_OK;
    if (command.kind != RK_COMMAND_JOINT_TARGETS)
        return RK_ERROR_UNSUPPORTED;
    for (uint32_t index = 0; index < command.target_count; ++index) {
        const auto &source = command.targets[index];
        if (source.joint >= joints_.size())
            return RK_ERROR_INVALID_ARGUMENT;
        nksim_joint_target target{};
        target.struct_size = sizeof(target);
        target.joint = joints_[source.joint];
        target.mode = source.mode;
        target.target = source.target;
        target.max_force = source.max_effort;
        pending_targets_.push_back(target);
    }
    return RK_OK;
}

std::vector<nksim_joint_target> SimRobotBinding::take_pending_targets() {
    auto result = std::move(pending_targets_);
    pending_targets_.clear();
    return result;
}

rk_result SimRobotBinding::sample(uint64_t timestamp_ns, rk_robot_state &state) const {
    if (world_.snapshot_ == 0)
        return RK_ERROR_INVALID_STATE;
    state.struct_size = sizeof(state);
    state.timestamp_ns = timestamp_ns;
    state.joint_count = static_cast<uint32_t>(joints_.size());
    for (uint32_t index = 0; index < state.joint_count; ++index)
        state.position[index] = state.velocity[index] = state.effort[index] = 0.0;
    uint64_t count = 0;
    if (nksim_snapshot_get_joint_count(world_.snapshot_, &count) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    for (uint64_t index = 0; index < count; ++index) {
        nksim_joint_state source{};
        source.struct_size = sizeof(source);
        if (nksim_snapshot_get_joint(world_.snapshot_, index, &source) != NKSIM_OK)
            return RK_ERROR_BACKEND;
        for (uint32_t target = 0; target < state.joint_count; ++target) {
            if (source.joint == joints_[target]) {
                state.position[target] = source.position;
                state.velocity[target] = source.velocity;
                state.effort[target] = source.effort;
                break;
            }
        }
    }
    return RK_OK;
}

rk_result SimEndpoint::apply(const rk_robot_command &command) {
    return binding_->apply(command);
}
rk_result SimEndpoint::sample(uint64_t timestamp_ns, rk_robot_state &state) {
    return binding_->sample(timestamp_ns, state);
}

} // namespace robotkit
