#include "nativekit_scene.h"
#include "nativekit_sim.h"
#include "nativekit_sim_host.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <thread>

namespace {

nkscene_node_id make_node(nkscene_scene scene, double z) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_node_id node{};
    assert(nkscene_tx_create_node(transaction, &node) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[15] = 1.0f;
    transform.matrix[14] = static_cast<float>(z);
    assert(nkscene_tx_set_transform(transaction, node, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
    return node;
}

nksim_world make_world(nkscene_scene scene, double timestep) {
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = timestep;
    desc.physics_substeps = 1;
    desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_world_create(&desc, &world) == NKSIM_OK);
    return world;
}

void external_host_owns_world_and_publishes_snapshots() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto node = make_node(scene, 10.0);
    const auto world = make_world(scene, 0.01);

    const double half_extents[] = {0.5, 0.5, 0.5};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half_extents, &shape) == NKSIM_OK);
    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.node = node;
    body_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    body_desc.mass = 1.0;
    body_desc.shape = shape;
    nksim_body body = 0;
    assert(nksim_body_create(world, &body_desc, &body) == NKSIM_OK);

    nksim_host_desc host_desc{};
    host_desc.struct_size = sizeof(host_desc);
    host_desc.world = world;
    host_desc.mode = NKSIM_HOST_MODE_EXTERNAL;
    nksim_host host = 0;
    assert(nksim_host_create(&host_desc, &host) == NKSIM_OK);
    assert(nksim_host_start(host) == NKSIM_OK);

    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(nksim_world_get_clock(world, &clock) == NKSIM_ERROR_WRONG_THREAD);
    assert(nksim_host_get_clock(host, &clock) == NKSIM_OK);
    assert(clock.step_index == 0);

    nksim_snapshot initial_snapshot = 0;
    assert(nksim_host_get_snapshot(host, &initial_snapshot) == NKSIM_OK);
    uint64_t initial_count = 0;
    assert(nksim_snapshot_get_body_count(initial_snapshot, &initial_count) == NKSIM_OK);
    assert(initial_count == 1);
    nksim_body_state initial_state{};
    initial_state.struct_size = sizeof(initial_state);
    assert(nksim_snapshot_get_body(initial_snapshot, 0, &initial_state) == NKSIM_OK);

    nksim_body_force force{};
    force.struct_size = sizeof(force);
    force.body = body;
    force.force[0] = 2.0;
    assert(nksim_host_submit_forces(host, &force, 1) == NKSIM_OK);

    nksim_snapshot old_snapshot = initial_snapshot;
    for (int index = 0; index < 100; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        assert(nksim_host_step(host, &step) == NKSIM_OK);
        assert(step.step_index == static_cast<uint64_t>(index + 1));
        assert(step.scene_changes != 0);
        nkscene_change_set_destroy(step.scene_changes);
    }

    nksim_body_state old_state{};
    old_state.struct_size = sizeof(old_state);
    assert(nksim_snapshot_get_body(old_snapshot, 0, &old_state) == NKSIM_OK);
    assert(std::abs(old_state.position[2] - initial_state.position[2]) < 1e-12);
    nksim_snapshot_destroy(old_snapshot);

    assert(nksim_host_get_clock(host, &clock) == NKSIM_OK);
    assert(clock.step_index == 100);
    nksim_snapshot latest_snapshot = 0;
    assert(nksim_host_get_snapshot(host, &latest_snapshot) == NKSIM_OK);
    nksim_body_state latest_state{};
    latest_state.struct_size = sizeof(latest_state);
    assert(nksim_snapshot_get_body(latest_snapshot, 0, &latest_state) == NKSIM_OK);
    assert(latest_state.position[2] < initial_state.position[2]);
    nksim_snapshot_destroy(latest_snapshot);

    nksim_host_status status{};
    status.struct_size = sizeof(status);
    assert(nksim_host_get_status(host, &status) == NKSIM_OK);
    assert(status.running != 0);
    assert(status.mode == NKSIM_HOST_MODE_EXTERNAL);

    assert(nksim_host_stop(host) == NKSIM_OK);
    assert(nksim_world_get_clock(world, &clock) == NKSIM_OK);
    assert(clock.step_index == 100);
    nksim_host_destroy(host);
    nksim_body_destroy(world, body);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void realtime_host_can_pause_and_resume() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto world = make_world(scene, 0.001);

    nksim_host_desc host_desc{};
    host_desc.struct_size = sizeof(host_desc);
    host_desc.world = world;
    host_desc.mode = NKSIM_HOST_MODE_REALTIME;
    nksim_host host = 0;
    assert(nksim_host_create(&host_desc, &host) == NKSIM_OK);
    assert(nksim_host_start(host) == NKSIM_OK);

    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    do {
        assert(nksim_host_get_clock(host, &clock) == NKSIM_OK);
        if (clock.step_index > 0)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    assert(clock.step_index > 0);

    assert(nksim_host_pause(host) == NKSIM_OK);
    nksim_clock paused_clock{};
    paused_clock.struct_size = sizeof(paused_clock);
    assert(nksim_host_get_clock(host, &paused_clock) == NKSIM_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    nksim_clock stable_clock{};
    stable_clock.struct_size = sizeof(stable_clock);
    assert(nksim_host_get_clock(host, &stable_clock) == NKSIM_OK);
    assert(stable_clock.step_index == paused_clock.step_index);

    assert(nksim_host_resume(host) == NKSIM_OK);
    do {
        assert(nksim_host_get_clock(host, &clock) == NKSIM_OK);
        if (clock.step_index > stable_clock.step_index)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    assert(clock.step_index > stable_clock.step_index);

    assert(nksim_host_stop(host) == NKSIM_OK);
    nksim_host_destroy(host);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void set_node_x(nkscene_scene scene, nkscene_node_id node, double x) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[15] = 1.0f;
    transform.matrix[12] = static_cast<float>(x);
    assert(nkscene_tx_set_transform(transaction, node, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
}

nksim_body_state host_step_and_read(nksim_host host) {
    nksim_step_result step{};
    step.struct_size = sizeof(step);
    assert(nksim_host_step(host, &step) == NKSIM_OK);
    if (step.scene_changes) nkscene_change_set_destroy(step.scene_changes);
    nksim_snapshot snapshot = 0;
    assert(nksim_host_get_snapshot(host, &snapshot) == NKSIM_OK);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_snapshot_get_body(snapshot, 0, &state) == NKSIM_OK);
    nksim_snapshot_destroy(snapshot);
    return state;
}

void host_body_state_writes_teleport_kinematic_bodies() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto node = make_node(scene, 0.0);
    const auto world = make_world(scene, 0.01);
    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.node = node;
    body_desc.motion_type = NKSIM_MOTION_KINEMATIC;
    body_desc.mass = 1.0;
    nksim_body body = 0;
    assert(nksim_body_create(world, &body_desc, &body) == NKSIM_OK);
    nksim_host_desc host_desc{};
    host_desc.struct_size = sizeof(host_desc);
    host_desc.world = world;
    host_desc.mode = NKSIM_HOST_MODE_EXTERNAL;
    nksim_host host = 0;
    assert(nksim_host_create(&host_desc, &host) == NKSIM_OK);
    assert(nksim_host_start(host) == NKSIM_OK);

    (void)host_step_and_read(host);
    set_node_x(scene, node, 0.02);
    auto state = host_step_and_read(host);
    assert(std::abs(state.linear_velocity[0] - 2.0) < 1e-4);

    // A queued state write applies before the next tick on the owner thread
    // and marks the jump as a teleport that keeps the written twist.
    set_node_x(scene, node, 5.0);
    auto teleport = state;
    teleport.position[0] = 5.0;
    assert(nksim_host_submit_body_states(host, &teleport, 1) == NKSIM_OK);
    state = host_step_and_read(host);
    assert(std::abs(state.position[0] - 5.0) < 1e-6);
    assert(state.linear_velocity[0] == teleport.linear_velocity[0]);
    set_node_x(scene, node, 5.02);
    state = host_step_and_read(host);
    assert(std::abs(state.linear_velocity[0] - 2.0) < 1e-3);

    nksim_body_state invalid = teleport;
    invalid.struct_size = 0;
    assert(nksim_host_submit_body_states(host, &invalid, 1) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(nksim_host_submit_body_states(host, nullptr, 1) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(nksim_host_submit_body_states(host, nullptr, 0) == NKSIM_OK);

    // A queued drive is continuous motion with the exact supplied twist.
    auto drive = state;
    drive.position[0] = 5.05;
    drive.linear_velocity[0] = 3.0;
    set_node_x(scene, node, 5.05);
    assert(nksim_host_submit_body_drives(host, &drive, 1) == NKSIM_OK);
    state = host_step_and_read(host);
    assert(state.position[0] == 5.05 && state.linear_velocity[0] == 3.0);
    assert(nksim_host_submit_body_drives(host, nullptr, 1) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(nksim_host_stop(host) == NKSIM_OK);
    nksim_host_destroy(host);
    nksim_body_destroy(world, body);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

} // namespace

int main() {
    external_host_owns_world_and_publishes_snapshots();
    realtime_host_can_pause_and_resume();
    host_body_state_writes_teleport_kinematic_bodies();
    return 0;
}
