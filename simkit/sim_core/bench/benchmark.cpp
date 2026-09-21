#include "nativekit_scene.h"
#include "nativekit_sim.h"

#include <chrono>
#include <cstdio>

int main() {
    nkscene_scene scene = 0;
    nkscene_scene_create(&scene);
    nkscene_transaction transaction = 0;
    nkscene_transaction_begin(scene, &transaction);
    nkscene_occurrence_id occurrence{};
    nkscene_tx_create_occurrence(transaction, &occurrence);
    nkscene_change_set changes = 0;
    nkscene_transaction_commit_with_changes(transaction, &changes);
    nkscene_change_set_destroy(changes);

    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 1.0 / 1000.0;
    desc.physics_substeps = 1;
    desc.gravity[2] = -9.81;
    nksim_world world = 0;
    nksim_world_create(&desc, &world);
    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.occurrence = occurrence;
    body_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    body_desc.mass = 1.0;
    nksim_body body = 0;
    nksim_body_create(world, &body_desc, &body);

    const auto start = std::chrono::steady_clock::now();
    for (int index = 0; index < 10000; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        nksim_world_step(world, &step);
        nkscene_change_set_destroy(step.scene_changes);
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    std::printf("10000 fixed steps: %lld us\n", static_cast<long long>(micros));
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return 0;
}
