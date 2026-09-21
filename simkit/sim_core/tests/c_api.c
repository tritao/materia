#include "nativekit_sim.h"
#include "nativekit_sim_host.h"

#include <assert.h>

int main(void) {
    nkscene_scene scene = {0};
    assert(nkscene_scene_create(&scene) == NKS_OK);

    nksim_world_desc desc = {0};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 1.0 / 1000.0;
    desc.physics_substeps = 1;
    desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_world_create(&desc, &world) == NKSIM_OK);

    nksim_clock clock = {0};
    clock.struct_size = sizeof(clock);
    assert(nksim_world_get_clock(world, &clock) == NKSIM_OK);
    assert(clock.step_index == 0);

    nksim_step_result step = {0};
    step.struct_size = sizeof(step);
    assert(nksim_world_step(world, &step) == NKSIM_OK);
    assert(step.scene_changes != 0);
    nkscene_change_set_destroy(step.scene_changes);

    nksim_host_desc host_desc = {0};
    host_desc.struct_size = sizeof(host_desc);
    host_desc.world = world;
    host_desc.mode = NKSIM_HOST_MODE_EXTERNAL;
    nksim_host host = 0;
    assert(nksim_host_create(&host_desc, &host) == NKSIM_OK);
    assert(nksim_host_start(host) == NKSIM_OK);
    assert(nksim_world_get_clock(world, &clock) == NKSIM_ERROR_WRONG_THREAD);
    assert(nksim_host_get_clock(host, &clock) == NKSIM_OK);
    assert(clock.step_index == 1);
    step = (nksim_step_result){0};
    step.struct_size = sizeof(step);
    assert(nksim_host_step(host, &step) == NKSIM_OK);
    nkscene_change_set_destroy(step.scene_changes);
    assert(nksim_host_stop(host) == NKSIM_OK);
    nksim_host_destroy(host);

    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return 0;
}
