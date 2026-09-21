#include "sim_endpoint.hpp"

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

nkscene_transform identity_transform() {
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[15] = 1.0f;
    return transform;
}

} // namespace

SimEndpoint::SimEndpoint(const rk_runtime_blueprint &blueprint) {
    try {
        initialize(blueprint);
    } catch (...) {
        cleanup();
        throw;
    }
}

SimEndpoint::~SimEndpoint() {
    cleanup();
}

void SimEndpoint::initialize(const rk_runtime_blueprint &blueprint) {
    if (rk_runtime_blueprint_validate(&blueprint) != RK_OK)
        throw std::invalid_argument("invalid RobotKit runtime blueprint");
    if (blueprint.link_count == 0)
        throw std::invalid_argument("simulated robot requires at least one link");

    require_scene(nkscene_scene_create(&scene_), "nkscene_scene_create");
    occurrences_.reserve(blueprint.link_count);
    nkscene_transaction transaction = 0;
    require_scene(nkscene_transaction_begin(scene_, &transaction),
                  "nkscene_transaction_begin");
    for (uint32_t index = 0; index < blueprint.link_count; ++index) {
        nkscene_occurrence_id occurrence{};
        if (nkscene_tx_create_occurrence(transaction, &occurrence) != NKS_OK) {
            nkscene_transaction_cancel(transaction);
            throw std::runtime_error("nkscene_tx_create_occurrence");
        }
        const auto transform = identity_transform();
        if (nkscene_tx_set_transform(transaction, occurrence, &transform) != NKS_OK) {
            nkscene_transaction_cancel(transaction);
            throw std::runtime_error("nkscene_tx_set_transform");
        }
        occurrences_.push_back(occurrence);
    }
    nkscene_change_set changes = 0;
    require_scene(nkscene_transaction_commit_with_changes(transaction, &changes),
                  "nkscene_transaction_commit_with_changes");
    if (changes != 0)
        nkscene_change_set_destroy(changes);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene_;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = 0.0;
    require_sim(nksim_world_create(&world_desc, &world_), "nksim_world_create");

    const double half_extents[] = {0.05, 0.05, 0.05};
    require_sim(nksim_shape_create_box(world_, half_extents, &shape_),
                "nksim_shape_create_box");

    bodies_.reserve(blueprint.link_count);
    for (uint32_t index = 0; index < blueprint.link_count; ++index) {
        nksim_body_desc body_desc{};
        body_desc.struct_size = sizeof(body_desc);
        body_desc.occurrence = occurrences_[index];
        body_desc.motion_type = index == 0 ? NKSIM_MOTION_STATIC : NKSIM_MOTION_DYNAMIC;
        body_desc.mass = 1.0;
        body_desc.shape = shape_;
        body_desc.collision_layer = 1;
        body_desc.collision_mask = 1;
        nksim_body body = 0;
        require_sim(nksim_body_create(world_, &body_desc, &body), "nksim_body_create");
        bodies_.push_back(body);
    }

    joints_.reserve(blueprint.joint_count);
    for (uint32_t index = 0; index < blueprint.joint_count; ++index) {
        const auto &source = blueprint.joints[index];
        nksim_joint_desc joint_desc{};
        joint_desc.struct_size = sizeof(joint_desc);
        joint_desc.type = source.type;
        joint_desc.body_a = bodies_[source.parent_link];
        joint_desc.body_b = bodies_[source.child_link];
        joint_desc.axis_a[2] = 1.0;
        joint_desc.lower_limit = source.lower_limit;
        joint_desc.upper_limit = source.upper_limit;
        joint_desc.max_force = source.max_effort;
        nksim_joint joint = 0;
        require_sim(nksim_joint_create(world_, &joint_desc, &joint), "nksim_joint_create");
        joints_.push_back(joint);
    }

    nksim_host_desc host_desc{};
    host_desc.struct_size = sizeof(host_desc);
    host_desc.world = world_;
    host_desc.mode = NKSIM_HOST_MODE_EXTERNAL;
    require_sim(nksim_host_create(&host_desc, &host_), "nksim_host_create");
    require_sim(nksim_host_start(host_), "nksim_host_start");
}

void SimEndpoint::cleanup() noexcept {
    if (host_ != 0) {
        nksim_host_stop(host_);
        nksim_host_destroy(host_);
        host_ = 0;
    }
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
    joints_.clear();
    bodies_.clear();
    occurrences_.clear();
    if (scene_ != 0) {
        nkscene_scene_destroy(scene_);
        scene_ = 0;
    }
}

rk_result SimEndpoint::apply(const rk_robot_command &command) {
    if (command.kind == RK_COMMAND_EMERGENCY_STOP) {
        stopped_ = true;
        return RK_OK;
    }
    if (command.kind == RK_COMMAND_STOP) {
        stopped_ = false;
        return RK_OK;
    }
    if (stopped_)
        return RK_ERROR_SAFETY_STOPPED;
    if (command.kind == RK_COMMAND_NONE)
        return RK_OK;

    std::vector<nksim_joint_target> targets;
    targets.reserve(command.target_count);
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
        targets.push_back(target);
    }
    if (targets.empty())
        return RK_OK;
    return nksim_host_submit_joint_targets(host_, targets.data(),
                                           static_cast<uint32_t>(targets.size())) == NKSIM_OK
               ? RK_OK
               : RK_ERROR_BACKEND;
}

rk_result SimEndpoint::step(uint64_t timestamp_ns, rk_robot_state &state) {
    nksim_step_result step_result{};
    step_result.struct_size = sizeof(step_result);
    if (nksim_host_step(host_, &step_result) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    if (step_result.scene_changes != 0)
        nkscene_change_set_destroy(step_result.scene_changes);

    nksim_snapshot snapshot = 0;
    if (nksim_host_get_snapshot(host_, &snapshot) != NKSIM_OK)
        return RK_ERROR_BACKEND;
    state.struct_size = sizeof(state);
    state.timestamp_ns = timestamp_ns;
    state.joint_count = static_cast<uint32_t>(joints_.size());
    for (uint32_t index = 0; index < state.joint_count; ++index) {
        state.position[index] = 0.0;
        state.velocity[index] = 0.0;
        state.effort[index] = 0.0;
    }

    uint64_t joint_count = 0;
    if (nksim_snapshot_get_joint_count(snapshot, &joint_count) != NKSIM_OK) {
        nksim_snapshot_destroy(snapshot);
        return RK_ERROR_BACKEND;
    }
    for (uint64_t index = 0; index < joint_count; ++index) {
        nksim_joint_state joint_state{};
        joint_state.struct_size = sizeof(joint_state);
        if (nksim_snapshot_get_joint(snapshot, index, &joint_state) != NKSIM_OK)
            continue;
        for (uint32_t target = 0; target < state.joint_count; ++target) {
            if (joint_state.joint == joints_[target]) {
                state.position[target] = joint_state.position;
                state.velocity[target] = joint_state.velocity;
                state.effort[target] = joint_state.effort;
                break;
            }
        }
    }
    nksim_snapshot_destroy(snapshot);
    return RK_OK;
}

} // namespace robotkit
