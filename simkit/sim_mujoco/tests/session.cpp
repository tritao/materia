#undef NDEBUG
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

} // namespace

int main() {
    actor_pushes_dynamic_object_without_being_pushed();
    return 0;
}
