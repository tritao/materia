#include "nativekit_scene.h"
#include "nativekit_sim.h"
#include "nativekit_sim_mujoco.h"
#include "nativekit_sim_session.h"

#include <cassert>
#include <cmath>

namespace {

nksim_pose pose_at(double x, double y, double z) {
    nksim_pose pose{};
    pose.struct_size = sizeof(pose);
    pose.position[0] = x;
    pose.position[1] = y;
    pose.position[2] = z;
    pose.rotation[3] = 1.0;
    return pose;
}

nksim_object make_box(nksim_session session, uint32_t motion, double mass, const nksim_pose &pose,
                      double x, double y, double z) {
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.motion_type = motion;
    desc.shape.type = NKSIM_SHAPE_BOX;
    desc.shape.parameters[0] = x;
    desc.shape.parameters[1] = y;
    desc.shape.parameters[2] = z;
    desc.pose = pose;
    desc.mass = mass;
    nksim_object object = 0;
    assert(nksim_session_create_object(session, &desc, &object) == NKSIM_OK);
    return object;
}

nksim_pose object_pose(nksim_frame frame, nksim_object object) {
    nksim_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(nksim_frame_get_object_pose(frame, object, &pose) == NKSIM_OK);
    return pose;
}

// A walking person's kinematic proxy shoves a crate along the floor and is
// never shoved back.
void actor_pushes_dynamic_object_without_being_pushed() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.005;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    nksim_session_desc session_desc{};
    session_desc.struct_size = sizeof(session_desc);
    session_desc.scene = scene;
    session_desc.world = world;
    nksim_session session = 0;
    assert(nksim_session_create(&session_desc, &session) == NKSIM_OK);

    make_box(session, NKSIM_MOTION_STATIC, 0.0, pose_at(0.0, 0.0, -0.5), 10.0, 10.0, 0.5);
    const auto crate = make_box(session, NKSIM_MOTION_DYNAMIC, 5.0, pose_at(1.0, 0.0, 0.2),
                                0.2, 0.2, 0.2);
    // A torso-sized upright capsule walking 2 m along +X in 2 s.
    nksim_actor_part part{};
    part.struct_size = sizeof(part);
    part.shape.type = NKSIM_SHAPE_CAPSULE;
    part.shape.parameters[0] = 0.15;
    part.shape.parameters[1] = 0.8;
    part.pose = pose_at(0.0, 0.0, 0.6);
    nksim_actor actor = 0;
    assert(nksim_session_create_actor(session, &part, 1, &actor) == NKSIM_OK);
    const auto start = pose_at(0.0, 0.0, 0.6), end = pose_at(2.0, 0.0, 0.6);
    assert(nksim_session_push_actor_keyframe(session, actor, 0.0, &start, 1) == NKSIM_OK);
    assert(nksim_session_push_actor_keyframe(session, actor, 2.0, &end, 1) == NKSIM_OK);

    for (int tick = 0; tick < 300; ++tick)
        assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    nksim_frame frame = 0;
    assert(nksim_session_capture(session, &frame) == NKSIM_OK);
    nksim_pose walker{};
    walker.struct_size = sizeof(walker);
    assert(nksim_frame_get_actor_pose(frame, actor, 0, &walker) == NKSIM_OK);
    // 1.5 s in: exactly on its trajectory despite the contact.
    assert(std::abs(walker.position[0] - 1.5) < 1e-6 && std::abs(walker.position[1]) < 1e-9);
    const auto shoved = object_pose(frame, crate);
    // The crate stays ahead of the capsule's front surface and on the floor.
    assert(shoved.position[0] > 1.5 + 0.15 + 0.2 - 0.05);
    assert(std::abs(shoved.position[2] - 0.2) < 0.05);
    nksim_frame_destroy(frame);

    nksim_session_destroy(session);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void held_object_tracks_pushes_and_releases() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.005;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    nksim_session_desc session_desc{};
    session_desc.struct_size = sizeof(session_desc);
    session_desc.scene = scene;
    session_desc.world = world;
    nksim_session session = 0;
    assert(nksim_session_create(&session_desc, &session) == NKSIM_OK);
    make_box(session, NKSIM_MOTION_STATIC, 0.0, pose_at(0, 0, -0.5), 10, 10, 0.5);
    make_box(session, NKSIM_MOTION_STATIC, 0.0, pose_at(1.0, 0, 0.45),
             0.5, 0.5, 0.45);
    const auto payload = make_box(session, NKSIM_MOTION_DYNAMIC, 1.0,
                                  pose_at(0.4, 0, 1), 0.1, 0.1, 0.1);
    const auto target = make_box(session, NKSIM_MOTION_DYNAMIC, 1.0,
                                 pose_at(1.0, 0, 1.0), 0.1, 0.1, 0.1);
    nksim_actor_part part{};
    part.struct_size = sizeof(part);
    part.shape.type = NKSIM_SHAPE_SPHERE;
    part.shape.parameters[0] = 0.05;
    part.pose = pose_at(0, 0, 1);
    nksim_actor actor = 0;
    assert(nksim_session_create_actor(session, &part, 1, &actor) == NKSIM_OK);
    nksim_body carrier = 0, body = 0, reported = 0;
    assert(nksim_session_get_actor_body(session, actor, 0, &carrier) == NKSIM_OK);
    assert(nksim_session_get_object_body(session, payload, &body) == NKSIM_OK);
    const auto offset = pose_at(0.4, 0, 0);
    assert(nksim_session_hold_object(session, payload, carrier, &offset) == NKSIM_OK);
    assert(nksim_session_get_object_carrier(session, payload, &reported) == NKSIM_OK &&
           reported == carrier);
    const auto start = pose_at(0, 0, 1), end = pose_at(2, 0, 1);
    assert(nksim_session_push_actor_keyframe(session, actor, 0, &start, 1) == NKSIM_OK);
    assert(nksim_session_push_actor_keyframe(session, actor, 2, &end, 1) == NKSIM_OK);
    for (int tick = 1; tick <= 100; ++tick) {
        assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
        nksim_body_state state{};
        state.struct_size = sizeof(state);
        assert(nksim_session_get_body_state(session, body, &state) == NKSIM_OK);
        assert(std::abs(state.position[0] - (0.4 + tick * 0.005)) < 1e-6);
        assert(std::abs(state.position[2] - 1.0) < 1e-6);
        assert(std::abs(state.linear_velocity[0] - 1.0) < 1e-5);
    }
    // The payload's path crossed the free box, which must have moved.
    nksim_frame frame = 0;
    assert(nksim_session_capture(session, &frame) == NKSIM_OK);
    const auto pushed = object_pose(frame, target);
    assert(pushed.position[0] > 1.05);
    nksim_frame_destroy(frame);
    for (int tick = 0; tick < 300; ++tick)
        assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_release_object(session, payload) == NKSIM_OK);
    assert(nksim_session_release_object(session, payload) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    nksim_body_state released{};
    released.struct_size = sizeof(released);
    assert(nksim_session_get_body_state(session, body, &released) == NKSIM_OK);
    assert(released.linear_velocity[0] > 0.8 && released.position[2] < 1.0);
    for (int tick = 0; tick < 500; ++tick)
        assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    nksim_body_state rested{};
    rested.struct_size = sizeof(rested);
    assert(nksim_session_get_body_state(session, body, &rested) == NKSIM_OK);
    assert(std::abs(rested.position[2] - 0.1) < 0.02);
    assert(nksim_session_stop(session) == NKSIM_OK);
    assert(nksim_session_reset(session) == NKSIM_OK);
    assert(nksim_session_get_object_carrier(session, payload, &reported) == NKSIM_OK &&
           reported == 0);
    nksim_body_state reset{};
    reset.struct_size = sizeof(reset);
    assert(nksim_session_get_body_state(session, body, &reset) == NKSIM_OK);
    assert(std::abs(reset.position[0] - 0.4) < 1e-6);
    assert(std::abs(reset.position[2] - 1.0) < 1e-6);
    for (double velocity : reset.linear_velocity)
        assert(std::abs(velocity) < 1e-6);
    assert(nksim_session_hold_object(session, payload, carrier, &offset) == NKSIM_OK);
    assert(nksim_session_reset(session) == NKSIM_OK);
    assert(nksim_session_get_object_carrier(session, payload, &reported) == NKSIM_OK &&
           reported == 0);
    assert(nksim_session_hold_object(session, payload, carrier, &offset) == NKSIM_OK);
    assert(nksim_session_destroy_actor(session, actor) == NKSIM_OK);
    assert(nksim_session_get_object_carrier(session, payload, &reported) == NKSIM_OK &&
           reported == 0);
    // Rebuilding while held must retain the cargo's future floor contacts.
    nksim_actor_part rebuild_carrier_part{};
    rebuild_carrier_part.struct_size = sizeof(rebuild_carrier_part);
    rebuild_carrier_part.shape.type = NKSIM_SHAPE_SPHERE;
    rebuild_carrier_part.shape.parameters[0] = 0.02;
    rebuild_carrier_part.pose = pose_at(3.0, 0, 1.0);
    nksim_actor rebuild_carrier = 0;
    assert(nksim_session_create_actor(session, &rebuild_carrier_part, 1, &rebuild_carrier) == NKSIM_OK);
    nksim_body rebuild_body = 0;
    assert(nksim_session_get_actor_body(session, rebuild_carrier, 0, &rebuild_body) == NKSIM_OK);
    const auto rebuilt_payload = make_box(session, NKSIM_MOTION_DYNAMIC, 1.0,
                                          pose_at(3.4, 0, 1.0), 0.1, 0.1, 0.1);
    const auto rebuild_offset = pose_at(0.4, 0, 0);
    assert(nksim_session_hold_object(session, rebuilt_payload, rebuild_body,
                                     &rebuild_offset) == NKSIM_OK);
    assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_stop(session) == NKSIM_OK);
    make_box(session, NKSIM_MOTION_STATIC, 0.0, pose_at(-3.0, 0, 0.5), 0.2, 0.2, 0.5);
    assert(nksim_session_release_object(session, rebuilt_payload) == NKSIM_OK);
    for (int tick = 0; tick < 500; ++tick)
        assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_capture(session, &frame) == NKSIM_OK);
    const auto rebuilt_rest = object_pose(frame, rebuilt_payload);
    assert(std::abs(rebuilt_rest.position[2] - 0.1) < 0.03);
    nksim_frame_destroy(frame);
    nksim_session_destroy(session);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

} // namespace

int main() {
    actor_pushes_dynamic_object_without_being_pushed();
    held_object_tracks_pushes_and_releases();
    return 0;
}
