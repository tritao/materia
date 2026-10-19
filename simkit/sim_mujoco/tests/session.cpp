#include "nativekit_scene.h"
#include "nativekit_sim.h"
#include "nativekit_sim_mujoco.h"
#include "nativekit_sim_session.h"

#include <cassert>
#include <cmath>
#include <cstdio>

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
    uint64_t contact_count = 0;
    assert(nksim_frame_get_contact_count(frame, &contact_count) == NKSIM_OK && contact_count > 0);
    nksim_contact contact{};contact.struct_size = sizeof(contact);
    assert(nksim_frame_get_contact(frame, 0, &contact) == NKSIM_OK);
    assert(contact.wrench_valid && contact.body_a != contact.body_b);
    const auto retained_count = contact_count;
    assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    assert(nksim_frame_get_contact_count(frame, &contact_count) == NKSIM_OK && contact_count == retained_count);
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

void fixed_object_tree_survives_free_motion_and_child_rebuild() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.002;
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
    make_box(session, NKSIM_MOTION_STATIC, 0, pose_at(0, 0, -0.05), 10, 10, 0.05);
    const auto parent = make_box(session, NKSIM_MOTION_DYNAMIC, 0.2, pose_at(0, 0, 0.5), 0.05, 0.05, 0.05);
    auto child = make_box(session, NKSIM_MOTION_DYNAMIC, 0.2, pose_at(0.4, 0, 0.5), 0.05, 0.05, 0.05);
    auto moved = pose_at(1, 2, 0.5);
    moved.rotation[2] = moved.rotation[3] = std::sqrt(0.5);
    assert(nksim_session_teleport_object(session, parent, &moved) == NKSIM_OK);
    nksim_joint joint = 0;
    assert(nksim_session_create_fixed_joint(session, parent, child, &joint) == NKSIM_OK);
    for (int tick = 0; tick < 1000; ++tick)
        assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    nksim_frame frame = 0;
    assert(nksim_session_capture(session, &frame) == NKSIM_OK);
    const auto a = object_pose(frame, parent), b = object_pose(frame, child);
    assert(std::abs(a.position[2] - 0.05) < 0.01);
    const double x = a.rotation[0], y = a.rotation[1], z = a.rotation[2], w = a.rotation[3];
    assert(std::abs(b.position[0] - a.position[0] - 0.4 * (1 - 2 * (y*y + z*z))) < 1e-6);
    assert(std::abs(b.position[1] - a.position[1] - 0.8 * (x*y + z*w)) < 1e-6);
    assert(std::abs(b.position[2] - a.position[2] - 0.8 * (x*z - y*w)) < 1e-6);
    nksim_frame_destroy(frame);
    assert(nksim_session_stop(session) == NKSIM_OK);
    nksim_body root_body = 0;
    assert(nksim_session_get_object_body(session, parent, &root_body) == NKSIM_OK);
    assert(nksim_session_destroy_fixed_joint(session, joint) == NKSIM_OK);
    assert(nksim_session_destroy_object(session, child) == NKSIM_OK);
    // Rebuild a child's geometry in the stable root's original reference frame.
    child = make_box(session, NKSIM_MOTION_DYNAMIC, 0.2, pose_at(0.4, 0, 0.7), 0.05, 0.05, 0.05);
    assert(nksim_session_create_fixed_joint(session, parent, child, &joint) == NKSIM_OK);
    nksim_body preserved_body = 0;
    assert(nksim_session_get_object_body(session, parent, &preserved_body) == NKSIM_OK);
    assert(preserved_body == root_body);
    nksim_session_status status{};
    status.struct_size = sizeof(status);
    assert(nksim_session_get_status(session, &status) == NKSIM_OK);
    assert(status.step_index == 1000);
    assert(nksim_session_capture(session, &frame) == NKSIM_OK);
    const auto preserved = object_pose(frame, parent);
    for (int axis = 0; axis < 3; ++axis)
        assert(std::abs(preserved.position[axis] - a.position[axis]) < 1e-8);
    nksim_frame_destroy(frame);
    assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_capture(session, &frame) == NKSIM_OK);
    const auto current = object_pose(frame, parent), rebuilt = object_pose(frame, child);
    const double qx = current.rotation[0], qy = current.rotation[1], qz = current.rotation[2], qw = current.rotation[3];
    assert(std::abs(rebuilt.position[0] - current.position[0] -
        (0.4 * (1 - 2 * (qy*qy + qz*qz)) + 0.4 * (qx*qz + qy*qw))) < 1e-6);
    assert(std::abs(rebuilt.position[1] - current.position[1] -
        (0.8 * (qx*qy + qz*qw) + 0.4 * (qy*qz - qx*qw))) < 1e-6);
    assert(std::abs(rebuilt.position[2] - current.position[2] -
        (0.8 * (qx*qz - qy*qw) + 0.2 * (1 - 2 * (qx*qx + qy*qy)))) < 1e-6);
    nksim_frame_destroy(frame);
    assert(nksim_session_stop(session) == NKSIM_OK);
    assert(nksim_session_reset(session) == NKSIM_OK);
    assert(nksim_session_capture(session, &frame) == NKSIM_OK);
    const auto reset_parent = object_pose(frame, parent), reset_child = object_pose(frame, child);
    assert(std::abs(reset_parent.position[0] - 1) < 1e-6);
    assert(std::abs(reset_parent.position[1] - 2) < 1e-6);
    assert(std::abs(reset_parent.position[2] - 0.5) < 1e-6);
    assert(std::abs(reset_child.position[0] - 1) < 1e-6);
    assert(std::abs(reset_child.position[1] - 2.4) < 1e-6);
    assert(std::abs(reset_child.position[2] - 0.7) < 1e-6);
    nksim_frame_destroy(frame);
    nksim_session_destroy(session);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void object_inertia_and_local_center_drive_measured_dynamics() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.005;
    world_desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    nksim_session_desc session_desc{};
    session_desc.struct_size = sizeof(session_desc); session_desc.scene = scene; session_desc.world = world;
    nksim_session session = 0;
    assert(nksim_session_create(&session_desc, &session) == NKSIM_OK);
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc); desc.motion_type = NKSIM_MOTION_DYNAMIC;
    desc.shape.type = NKSIM_SHAPE_BOX;
    desc.shape.parameters[0] = desc.shape.parameters[1] = desc.shape.parameters[2] = 0.05;
    desc.pose = pose_at(1, 0, 1); desc.mass = 2;
    desc.has_inertial_properties = 1; desc.center_of_mass[0] = 0.01;
    desc.inertia_tensor[0] = 0.02; desc.inertia_tensor[4] = 0.03; desc.inertia_tensor[8] = 0.04;
    nksim_object object = 0;
    assert(nksim_session_create_object(session, &desc, &object) == NKSIM_OK);
    nksim_body body = 0;
    assert(nksim_session_get_object_body(session, object, &body) == NKSIM_OK);
    nksim_body_force force{};
    force.struct_size = sizeof(force); force.body = body; force.torque[2] = 0.4;
    assert(nksim_world_apply_forces(world, &force, 1) == NKSIM_OK);
    nksim_step_result step{}; step.struct_size = sizeof(step);
    assert(nksim_world_step(world, &step) == NKSIM_OK);
    nksim_body_state state{}; state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    assert(std::abs(state.angular_velocity[2] - 0.05) < 1e-8);
    assert(std::abs(state.position[1] + 0.01 * std::sin(0.00025)) < 1e-9);
    assert(nksim_session_stop(session) == NKSIM_OK);
    assert(nksim_session_reset(session) == NKSIM_OK);
    const auto carrier = make_box(session, NKSIM_MOTION_STATIC, 0, pose_at(3, 0, 1), 0.05, 0.05, 0.05);
    nksim_body carrier_body = 0;
    assert(nksim_session_get_object_body(session, carrier, &carrier_body) == NKSIM_OK);
    const auto offset = pose_at(-2, 0, 0);
    assert(nksim_session_hold_object(session, object, carrier_body, &offset) == NKSIM_OK);
    assert(nksim_world_begin_topology_update(world) == NKSIM_OK);
    const auto child = make_box(session, NKSIM_MOTION_DYNAMIC, 0.2, pose_at(1.2, 0, 1), 0.05, 0.05, 0.05);
    nksim_joint joint = 0;
    assert(nksim_session_create_fixed_joint(session, object, child, &joint) == NKSIM_OK);
    assert(nksim_world_end_topology_update(world) == NKSIM_OK);
    assert(nksim_world_begin_topology_update(world) == NKSIM_OK);
    assert(nksim_session_destroy_fixed_joint(session, joint) == NKSIM_OK);
    assert(nksim_session_destroy_object(session, child) == NKSIM_OK);
    assert(nksim_world_end_topology_update(world) == NKSIM_OK);
    assert(nksim_session_release_object(session, object) == NKSIM_OK);
    // A held topology rebuild must retain the physical centre and principal axes,
    // not just the three sorted principal moments of the kinematic proxy.
    assert(nksim_world_apply_forces(world, &force, 1) == NKSIM_OK);
    assert(nksim_world_step(world, &step) == NKSIM_OK);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    assert(std::abs(state.angular_velocity[2] - 0.05) < 1e-8);
    assert(std::abs(state.position[1] + 0.01 * std::sin(0.00025)) < 1e-9);
    desc.inertia_tensor[8] = -0.04;
    nksim_object invalid = 0;
    assert(nksim_session_create_object(session, &desc, &invalid) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(invalid == 0);
    nksim_session_destroy(session); nksim_world_destroy(world); nkscene_scene_destroy(scene);
}

void held_rigid_tree_rebuilds_and_releases_as_one_free_part() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{}; desc.struct_size = sizeof(desc); desc.scene = scene;
    desc.fixed_timestep = 0.001; desc.physics_substeps = 5; desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    nksim_session_desc session_desc{}; session_desc.struct_size = sizeof(session_desc);
    session_desc.scene = scene; session_desc.world = world;
    nksim_session session = 0;
    assert(nksim_session_create(&session_desc, &session) == NKSIM_OK);
    const auto root = make_box(session, NKSIM_MOTION_DYNAMIC, 0.2, pose_at(0, 0, 1), 0.05, 0.05, 0.05);
    const auto child = make_box(session, NKSIM_MOTION_DYNAMIC, 0.2, pose_at(0.4, 0, 1), 0.05, 0.05, 0.05);
    const auto carrier = make_box(session, NKSIM_MOTION_STATIC, 0, pose_at(3, 0, 2), 0.05, 0.05, 0.05);
    nksim_body carrier_body = 0, root_body = 0;
    assert(nksim_session_get_object_body(session, carrier, &carrier_body) == NKSIM_OK);
    assert(nksim_session_get_object_body(session, root, &root_body) == NKSIM_OK);
    nksim_joint first = 0;
    assert(nksim_session_create_fixed_joint(session, root, child, &first) == NKSIM_OK);
    const auto offset = pose_at(-2, 0, -1);
    assert(nksim_session_hold_object(session, root, carrier_body, &offset) == NKSIM_OK);
    assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_stop(session) == NKSIM_OK);
    assert(nksim_world_begin_topology_update(world) == NKSIM_OK);
    const auto other = make_box(session, NKSIM_MOTION_DYNAMIC, 0.2, pose_at(0, 0.4, 1), 0.05, 0.05, 0.05);
    nksim_joint second = 0;
    assert(nksim_session_create_fixed_joint(session, root, other, &second) == NKSIM_OK);
    assert(nksim_world_end_topology_update(world) == NKSIM_OK);
    assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    auto observed = pose_at(0, 0, 0);
    assert(nksim_session_get_body_pose(session, root_body, &observed) == NKSIM_OK);
    assert(std::abs(observed.position[0] - 1) < 1e-6);
    assert(std::abs(observed.position[2] - 1) < 1e-6);
    assert(nksim_session_release_object(session, root) == NKSIM_OK);
    for (int i = 0; i < 100; ++i) assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_get_body_pose(session, root_body, &observed) == NKSIM_OK);
    assert(observed.position[2] < 0.97);
    assert(nksim_session_hold_object(session, root, carrier_body, &offset) == NKSIM_OK);
    assert(nksim_session_step(session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_get_body_pose(session, root_body, &observed) == NKSIM_OK);
    assert(std::abs(observed.position[2] - 1) < 1e-6);
    nksim_body retained = 0;
    assert(nksim_session_get_object_body(session, root, &retained) == NKSIM_OK && retained == root_body);
    nksim_session_destroy(session); nksim_world_destroy(world); nkscene_scene_destroy(scene);
}

// Runtime fastening keeps the seated transform and both masses. A force on
// the retained child accelerates the free assembly rather than a mocap proxy
// pushing its own parent through the worktable.
void measured_fastening_transfers_force_through_combined_mass() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{}; desc.struct_size = sizeof(desc); desc.scene = scene;
    desc.fixed_timestep = 0.001; desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    nksim_session_desc session_desc{}; session_desc.struct_size = sizeof(session_desc);
    session_desc.scene = scene; session_desc.world = world;
    nksim_session session = 0;
    assert(nksim_session_create(&session_desc, &session) == NKSIM_OK);
    const auto parent = make_box(session, NKSIM_MOTION_DYNAMIC, 1, pose_at(0,0,1), .05,.05,.05);
    const auto child = make_box(session, NKSIM_MOTION_DYNAMIC, 2, pose_at(0,0,3), .05,.05,.05);
    auto a = pose_at(.3,.4,1), b = pose_at(.3,.4,1.1);
    a.rotation[2] = a.rotation[3] = b.rotation[2] = b.rotation[3] = std::sqrt(.5);
    assert(nksim_session_teleport_object(session,parent,&a) == NKSIM_OK);
    assert(nksim_session_teleport_object(session,child,&b) == NKSIM_OK);
    nksim_joint joint = 0;
    const auto connected = nksim_session_create_measured_fixed_joint(session,parent,child,&joint);
    if (connected != NKSIM_OK) std::fprintf(stderr,"Measured fixed connection: %d\n",connected);
    assert(connected == NKSIM_OK);
    nksim_body parent_body = 0, child_body = 0, owner = 0;
    assert(nksim_session_get_object_body(session,parent,&parent_body) == NKSIM_OK);
    assert(nksim_session_get_object_body(session,child,&child_body) == NKSIM_OK);
    assert(nksim_session_get_object_carrier(session,child,&owner) == NKSIM_OK && owner == parent_body);
    assert(nksim_session_step(session,0,nullptr) == NKSIM_OK);
    nksim_frame frame = 0;
    assert(nksim_session_capture(session,&frame) == NKSIM_OK);
    auto first = object_pose(frame,parent), second = object_pose(frame,child);
    assert(std::abs(first.position[0]-.3) < 1e-8 && std::abs(first.position[1]-.4) < 1e-8);
    assert(std::abs(first.position[2]-1) < 1e-8 && std::abs(second.position[2]-1.1) < 1e-8);
    nksim_frame_destroy(frame);
    nksim_body_force force{}; force.struct_size = sizeof(force); force.body = child_body; force.force[2] = 3;
    for (int tick=0; tick<100; ++tick) {
        assert(nksim_session_submit_forces(session,&force,1) == NKSIM_OK);
        assert(nksim_session_step(session,0,nullptr) == NKSIM_OK);
    }
    assert(nksim_session_capture(session,&frame) == NKSIM_OK);
    first = object_pose(frame,parent); second = object_pose(frame,child);
    assert(std::abs(first.position[2]-1.00505) < 1e-5); // 3 N / (1+2) kg for 0.1 s.
    assert(std::abs(second.position[2]-first.position[2]-.1) < 1e-8);
    nksim_frame_destroy(frame);
    assert(nksim_session_stop(session) == NKSIM_OK);
    assert(nksim_session_reset(session) == NKSIM_OK);
    assert(nksim_session_get_object_carrier(session,child,&owner) == NKSIM_OK && owner == 0);
    nksim_session_destroy(session); nksim_world_destroy(world); nkscene_scene_destroy(scene);
}

} // namespace

int main() {
    actor_pushes_dynamic_object_without_being_pushed();
    held_object_tracks_pushes_and_releases();
    fixed_object_tree_survives_free_motion_and_child_rebuild();
    object_inertia_and_local_center_drive_measured_dynamics();
    held_rigid_tree_rebuilds_and_releases_as_one_free_part();
    measured_fastening_transfers_force_through_combined_mass();
    return 0;
}
