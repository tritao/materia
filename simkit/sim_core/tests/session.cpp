#include "nativekit_scene.h"
#include "nativekit_sim.h"
#include "nativekit_sim_session.h"

#include <cassert>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>
#include <memory>
#include <limits>
#include <cstddef>

namespace {

struct Space {
    nkscene_scene scene = 0;
    nksim_world world = 0;
    nksim_session session = 0;

    explicit Space(double timestep = 0.01) {
        assert(nkscene_scene_create(&scene) == NKS_OK);
        nksim_world_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.scene = scene;
        desc.fixed_timestep = timestep;
        desc.physics_substeps = 1;
        desc.gravity[2] = -9.81;
        assert(nksim_world_create(&desc, &world) == NKSIM_OK);
        nksim_session_desc session_desc{};
        session_desc.struct_size = sizeof(session_desc);
        session_desc.scene = scene;
        session_desc.world = world;
        assert(nksim_session_create(&session_desc, &session) == NKSIM_OK);
    }
    ~Space() {
        nksim_session_destroy(session);
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
    }
};

nksim_pose pose_at(double x, double y, double z) {
    nksim_pose pose{};
    pose.struct_size = sizeof(pose);
    pose.position[0] = x;
    pose.position[1] = y;
    pose.position[2] = z;
    pose.rotation[3] = 1.0;
    return pose;
}

nksim_shape_desc capsule(double radius, double length) {
    nksim_shape_desc shape{};
    shape.struct_size = sizeof(shape);
    shape.type = NKSIM_SHAPE_CAPSULE;
    shape.parameters[0] = radius;
    shape.parameters[1] = length;
    return shape;
}

nksim_pose actor_pose(nksim_session session, nksim_actor actor, uint32_t part) {
    nksim_frame frame = 0;
    assert(nksim_session_capture(session, &frame) == NKSIM_OK);
    nksim_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(nksim_frame_get_actor_pose(frame, actor, part, &pose) == NKSIM_OK);
    nksim_frame_destroy(frame);
    return pose;
}

bool near(double a, double b, double tolerance = 1e-5) { return std::abs(a - b) < tolerance; }

void captured_contacts_are_immutable_and_checked() {
    Space space;
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.motion_type = NKSIM_MOTION_STATIC;
    desc.shape.type = NKSIM_SHAPE_BOX;
    desc.shape.parameters[0] = desc.shape.parameters[1] = desc.shape.parameters[2] = 0.5;
    desc.pose = pose_at(0, 0, -0.5);
    nksim_object floor = 0, stock = 0;
    assert(nksim_session_create_object(space.session, &desc, &floor) == NKSIM_OK);
    desc.motion_type = NKSIM_MOTION_KINEMATIC;desc.mass = 0;
    desc.pose = pose_at(0, 0, 0.49);
    assert(nksim_session_create_object(space.session, &desc, &stock) == NKSIM_OK);
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    nksim_frame frame = 0;
    assert(nksim_session_capture(space.session, &frame) == NKSIM_OK);
    uint64_t count = 0;
    assert(nksim_frame_get_contact_count(frame, &count) == NKSIM_OK && count > 0);
    const auto original = count;
    nksim_contact contact{};contact.struct_size = sizeof(contact);
    assert(nksim_frame_get_contact(frame, 0, &contact) == NKSIM_OK);
    assert(contact.body_a != 0 && contact.body_b != 0 && contact.body_a != contact.body_b);
    assert(nksim_frame_get_contact(frame, count, &contact) != NKSIM_OK);
    assert(nksim_frame_get_contact_count(frame, nullptr) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(nksim_session_stop(space.session) == NKSIM_OK);
    auto away = pose_at(0, 0, 5);
    assert(nksim_session_teleport_object(space.session, stock, &away) == NKSIM_OK);
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    assert(nksim_frame_get_contact_count(frame, &count) == NKSIM_OK && count == original);
    nksim_frame_destroy(frame);
    assert(nksim_frame_get_contact_count(frame, &count) == NKSIM_ERROR_INVALID_HANDLE);
}

void objects_follow_their_motion_type() {
    Space space;
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.motion_type = NKSIM_MOTION_DYNAMIC;
    desc.shape.type = NKSIM_SHAPE_BOX;
    desc.shape.parameters[0] = desc.shape.parameters[1] = desc.shape.parameters[2] = 0.25;
    desc.pose = pose_at(1.0, 0.0, 5.0);
    nksim_object falling = 0;
    // Dynamic objects need a mass.
    assert(nksim_session_create_object(space.session, &desc, &falling) ==
           NKSIM_ERROR_INVALID_ARGUMENT);
    desc.mass = 2.0;
    assert(nksim_session_create_object(space.session, &desc, &falling) == NKSIM_OK);
    desc.motion_type = NKSIM_MOTION_KINEMATIC;
    desc.pose = pose_at(0.0, 0.0, 1.0);
    nksim_object carried = 0;
    assert(nksim_session_create_object(space.session, &desc, &carried) == NKSIM_OK);
    // Only kinematic objects can be driven.
    auto target = pose_at(0.1, 0.0, 1.0);
    assert(nksim_session_drive_object(space.session, falling, &target) ==
           NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_drive_object(space.session, carried, &target) == NKSIM_OK);

    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(nksim_session_step(space.session, 0, &clock) == NKSIM_OK);
    assert(clock.step_index == 1 && near(clock.time, 0.01));
    nksim_frame frame = 0;
    assert(nksim_session_capture(space.session, &frame) == NKSIM_OK);
    nksim_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(nksim_frame_get_object_pose(frame, falling, &pose) == NKSIM_OK);
    assert(pose.position[2] < 5.0);
    assert(nksim_frame_get_object_pose(frame, carried, &pose) == NKSIM_OK);
    assert(near(pose.position[0], 0.1));
    nksim_body body = 0;
    assert(nksim_session_get_object_body(space.session, carried, &body) == NKSIM_OK);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_frame_get_body_state(frame, body, &state) == NKSIM_OK);
    assert(near(state.linear_velocity[0], 10.0, 1e-6));
    nksim_frame_destroy(frame);

    // Topology is fixed while hosted; stopping returns the world.
    nksim_object late = 0;
    assert(nksim_session_create_object(space.session, &desc, &late) ==
           NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_stop(space.session) == NKSIM_OK);
    assert(nksim_session_reset(space.session) == NKSIM_OK);
    assert(nksim_session_capture(space.session, &frame) == NKSIM_OK);
    assert(nksim_frame_get_object_pose(frame, falling, &pose) == NKSIM_OK);
    assert(near(pose.position[2], 5.0));
    assert(nksim_frame_get_object_pose(frame, carried, &pose) == NKSIM_OK);
    assert(near(pose.position[0], 0.0));
    nksim_frame_destroy(frame);
}

void presence_counts_only_free_boxes_in_its_volume() {
    Space space;
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.shape.type = NKSIM_SHAPE_BOX;
    desc.shape.parameters[0] = desc.shape.parameters[1] = desc.shape.parameters[2] = 0.1;
    desc.pose = pose_at(0.0, 0.0, 2.0);
    desc.motion_type = NKSIM_MOTION_STATIC;
    nksim_object fixture = 0;
    assert(nksim_session_create_object(space.session, &desc, &fixture) == NKSIM_OK);
    const double half_extents[3] = {0.5, 0.5, 0.5};
    const auto query = pose_at(0.0, 0.0, 2.0);
    uint32_t present = 1;
    assert(nksim_session_overlap_free_objects(space.session, &query, half_extents, &present) == NKSIM_OK);
    assert(present == 0);
    desc.motion_type = NKSIM_MOTION_DYNAMIC;
    desc.mass = 1.0;
    desc.pose = pose_at(0.4, 0.0, 2.0);
    nksim_object blank = 0;
    assert(nksim_session_create_object(space.session, &desc, &blank) == NKSIM_OK);
    assert(nksim_session_overlap_free_objects(space.session, &query, half_extents, &present) == NKSIM_OK);
    assert(present == 1);
    const auto far = pose_at(1.2, 0.0, 2.0);
    assert(nksim_session_overlap_free_objects(space.session, &far, half_extents, &present) == NKSIM_OK);
    assert(present == 0);
}

void presence_uses_loose_object_shapes() {
    for (const auto type : {NKSIM_SHAPE_SPHERE, NKSIM_SHAPE_CAPSULE, NKSIM_SHAPE_CYLINDER}) {
        Space space;
        nksim_object_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.shape.type = type;
        desc.shape.parameters[0] = 0.2;
        desc.shape.parameters[1] = 0.6;
        desc.pose = pose_at(0.65, 0.65, 2.0);
        desc.motion_type = NKSIM_MOTION_DYNAMIC;
        desc.mass = 1.0;
        nksim_object object = 0;
        assert(nksim_session_create_object(space.session, &desc, &object) == NKSIM_OK);
        const double half_extents[3] = {0.5, 0.5, 0.5};
        uint32_t present = 1;
        const auto query = pose_at(0.0, 0.0, 2.0);
        assert(nksim_session_overlap_free_objects(space.session, &query, half_extents, &present) == NKSIM_OK);
        assert(present == 0); // The primitive's bounding box overlaps, but its rounded corner does not.
        const auto closer = pose_at(0.01, 0.01, 2.0);
        assert(nksim_session_overlap_free_objects(space.session, &closer, half_extents, &present) == NKSIM_OK);
        assert(present == 1); // Off-axis contact with the volume, not its centre ray.
    }
}

void measured_fixed_connections_preserve_seated_pose_and_reset() {
    Space space;
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.motion_type = NKSIM_MOTION_DYNAMIC;
    desc.mass = 0.2;
    desc.shape.type = NKSIM_SHAPE_BOX;
    for (double &extent : desc.shape.parameters) extent = 0.05;
    desc.pose = pose_at(0, 0, 5);
    nksim_object parent = 0, child = 0, carrier = 0;
    assert(nksim_session_create_object(space.session, &desc, &parent) == NKSIM_OK);
    desc.pose = pose_at(0.4, 0, 5);
    assert(nksim_session_create_object(space.session, &desc, &child) == NKSIM_OK);
    auto seated_parent = pose_at(1, 2, 5), seated_child = pose_at(1, 2.3, 5);
    seated_parent.rotation[2] = seated_parent.rotation[3] = std::sqrt(0.5);
    seated_child.rotation[2] = seated_child.rotation[3] = std::sqrt(0.5);
    assert(nksim_session_teleport_object(space.session, parent, &seated_parent) == NKSIM_OK);
    assert(nksim_session_teleport_object(space.session, child, &seated_child) == NKSIM_OK);
    nksim_joint joint = 0;
    assert(nksim_session_create_measured_fixed_joint(space.session, parent, child, &joint) == NKSIM_OK);
    nksim_body parent_body = 0, owner = 0;
    assert(nksim_session_get_object_body(space.session, parent, &parent_body) == NKSIM_OK);
    assert(nksim_session_get_object_carrier(space.session, child, &owner) == NKSIM_OK && owner == parent_body);
    // Carry the parent through a different turn: the measured 0.3 m offset, not
    // the authored 0.4 m separation, must follow it on the very first tick.
    desc.motion_type = NKSIM_MOTION_STATIC;
    desc.pose = pose_at(0, 0, 0);
    assert(nksim_session_create_object(space.session, &desc, &carrier) == NKSIM_OK);
    nksim_body carrier_body = 0;
    assert(nksim_session_get_object_body(space.session, carrier, &carrier_body) == NKSIM_OK);
    auto carried = pose_at(2, 3, 5);
    carried.rotation[2] = 1; carried.rotation[3] = 0;
    assert(nksim_session_hold_object(space.session, parent, carrier_body, &carried) == NKSIM_OK);
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    nksim_frame frame = 0;
    assert(nksim_session_capture(space.session, &frame) == NKSIM_OK);
    nksim_pose a{}, b{}; a.struct_size = b.struct_size = sizeof(a);
    assert(nksim_frame_get_object_pose(frame, parent, &a) == NKSIM_OK);
    assert(nksim_frame_get_object_pose(frame, child, &b) == NKSIM_OK);
    assert(near(a.position[0], 2) && near(a.position[1], 3));
    assert(near(b.position[0], 1.7) && near(b.position[1], 3));
    assert(near(std::abs(b.rotation[2]), 1));
    nksim_frame_destroy(frame);
    assert(nksim_session_stop(space.session) == NKSIM_OK);
    assert(nksim_session_reset(space.session) == NKSIM_OK);
    assert(nksim_session_get_object_carrier(space.session, child, &owner) == NKSIM_OK && owner == 0);
    assert(nksim_session_capture(space.session, &frame) == NKSIM_OK);
    assert(nksim_frame_get_object_pose(frame, child, &b) == NKSIM_OK);
    assert(near(b.position[0], 1) && near(b.position[1], 2.3));
    nksim_frame_destroy(frame);
    assert(nksim_session_destroy_object(space.session, child) == NKSIM_OK);
}

void fixed_connections_use_reference_frames_and_session_ownership() {
    Space space;
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.motion_type = NKSIM_MOTION_DYNAMIC;
    desc.mass = 0.2;
    desc.shape.type = NKSIM_SHAPE_BOX;
    for (double &extent : desc.shape.parameters) extent = 0.05;
    desc.pose = pose_at(0, 0, 5);
    nksim_object parent = 0, child = 0;
    assert(nksim_session_create_object(space.session, &desc, &parent) == NKSIM_OK);
    desc.pose = pose_at(0.4, 0, 5);
    assert(nksim_session_create_object(space.session, &desc, &child) == NKSIM_OK);
    auto moved = pose_at(1, 2, 5);
    moved.rotation[2] = moved.rotation[3] = std::sqrt(0.5);
    assert(nksim_session_teleport_object(space.session, parent, &moved) == NKSIM_OK);
    nksim_joint joint = 0, invalid = 99;
    assert(nksim_session_create_fixed_joint(space.session, parent, child, &joint) == NKSIM_OK);
    assert(nksim_session_create_fixed_joint(space.session, child, parent, &invalid) == NKSIM_ERROR_INVALID_STATE);
    assert(invalid == 0);
    nksim_body parent_body = 0, child_body = 0;
    assert(nksim_session_get_object_body(space.session, parent, &parent_body) == NKSIM_OK);
    assert(nksim_session_get_object_body(space.session, child, &child_body) == NKSIM_OK);
    const auto offset = pose_at(0, 0, 0);
    assert(nksim_session_hold_object(space.session, child, parent_body, &offset) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_hold_object(space.session, parent, child_body, &offset) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_destroy_object(space.session, child) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    nksim_frame frame = 0;
    assert(nksim_session_capture(space.session, &frame) == NKSIM_OK);
    nksim_pose a{}, b{};
    a.struct_size = b.struct_size = sizeof(a);
    assert(nksim_frame_get_object_pose(frame, parent, &a) == NKSIM_OK);
    assert(nksim_frame_get_object_pose(frame, child, &b) == NKSIM_OK);
    assert(near(b.position[0] - a.position[0], 0));
    assert(near(b.position[1] - a.position[1], 0.4));
    assert(near(b.position[2] - a.position[2], 0));
    assert(near(std::abs(b.rotation[2]), std::sqrt(0.5)));
    nksim_frame_destroy(frame);
    assert(nksim_session_destroy_fixed_joint(space.session, joint) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_stop(space.session) == NKSIM_OK);
    assert(nksim_session_reset(space.session) == NKSIM_OK);
    assert(nksim_session_capture(space.session, &frame) == NKSIM_OK);
    assert(nksim_frame_get_object_pose(frame, parent, &a) == NKSIM_OK);
    assert(nksim_frame_get_object_pose(frame, child, &b) == NKSIM_OK);
    assert(near(a.position[0], 1) && near(a.position[1], 2) && near(a.position[2], 5));
    assert(near(b.position[0], 1) && near(b.position[1], 2.4) && near(b.position[2], 5));
    nksim_frame_destroy(frame);
    assert(nksim_session_destroy_fixed_joint(space.session, joint) == NKSIM_OK);
    assert(nksim_session_hold_object(space.session, parent, child_body, &offset) == NKSIM_OK);
    assert(nksim_session_create_fixed_joint(space.session, parent, child, &invalid) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_release_object(space.session, parent) == NKSIM_OK);
    assert(nksim_session_destroy_object(space.session, child) == NKSIM_OK);
    // A session tears its remaining joints down before their body resources.
    desc.pose = pose_at(0.4, 0, 5);
    assert(nksim_session_create_object(space.session, &desc, &child) == NKSIM_OK);
    assert(nksim_session_create_fixed_joint(space.session, parent, child, &joint) == NKSIM_OK);
    desc.motion_type = NKSIM_MOTION_STATIC;
    desc.pose = pose_at(0, 0, 5);
    nksim_object carrier = 0;
    nksim_body carrier_body = 0;
    assert(nksim_session_create_object(space.session, &desc, &carrier) == NKSIM_OK);
    assert(nksim_session_get_object_body(space.session, carrier, &carrier_body) == NKSIM_OK);
    auto carried_offset = pose_at(1, 2, 0);
    carried_offset.rotation[2] = carried_offset.rotation[3] = std::sqrt(0.5);
    assert(nksim_session_hold_object(space.session, parent, carrier_body, &carried_offset) == NKSIM_OK);
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_capture(space.session, &frame) == NKSIM_OK);
    assert(nksim_frame_get_object_pose(frame, child, &b) == NKSIM_OK);
    assert(near(b.position[0], 1) && near(b.position[1], 2.4) && near(b.position[2], 5));
    nksim_frame_destroy(frame);
}

void compound_objects_preserve_empty_gaps() {
    Space space;
    std::vector<double> vertices;
    // Two separated bars share one body, with a genuinely empty central gap.
    for (const double centre : {-0.3, 0.3})
        for (const double x : {-0.1, 0.1})
            for (const double y : {-0.2, 0.2})
                for (const double z : {-0.05, 0.05}) {
                    vertices.push_back(centre + x);
                    vertices.push_back(y);
                    vertices.push_back(z);
                }
    const uint32_t counts[] = {24, 24};
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.motion_type = NKSIM_MOTION_DYNAMIC;
    desc.mass = 0.8;
    desc.shape.type = NKSIM_SHAPE_COMPOUND;
    desc.pose = pose_at(0, 0, 2);
    nksim_object object = 0;
    assert(nksim_session_create_compound_convex_object(space.session, &desc,
        vertices.data(), vertices.size(), counts, 2, &object) == NKSIM_OK);
    uint32_t overlap = 1;
    auto query = pose_at(0, 0, 2);
    assert(nksim_session_object_overlaps_volume(space.session, object, &query, 0.02, 0.02, 0.02, &overlap) == NKSIM_OK);
    assert(overlap == 0);
    query = pose_at(0.3, 0, 2);
    assert(nksim_session_object_overlaps_volume(space.session, object, &query, 0.02, 0.02, 0.02, &overlap) == NKSIM_OK);
    assert(overlap == 1);
    nksim_ray ray{};
    ray.struct_size = sizeof(ray);
    ray.origin[2] = 3;
    ray.direction[2] = -1;
    ray.max_distance = 3;
    double hit = 0;
    assert(nksim_session_raycast(space.session, &ray, &hit) == NKSIM_OK);
    assert(near(hit, 3));
    ray.origin[0] = 0.3;
    assert(nksim_session_raycast(space.session, &ray, &hit) == NKSIM_OK);
    assert(near(hit, 0.95));
    nksim_object invalid = 99;
    assert(nksim_session_create_compound_convex_object(space.session, &desc,
        vertices.data(), vertices.size() - 1, counts, 2, &invalid) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(invalid == 0);
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    assert(nksim_session_stop(space.session) == NKSIM_OK);
    assert(nksim_session_destroy_object(space.session, object) == NKSIM_OK);
    assert(nksim_session_raycast(space.session, &ray, &hit) == NKSIM_OK);
    assert(near(hit, 3));
}

void convex_objects_keep_their_actual_volume() {
    Space space;
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.shape.type = NKSIM_SHAPE_CONVEX;
    desc.pose = pose_at(0, 0, 0);
    desc.motion_type = NKSIM_MOTION_DYNAMIC;
    desc.mass = 1.0;
    double vertices[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
    nksim_object object = 0;
    assert(nksim_session_create_convex_object(space.session, &desc, vertices, 12, &object) == NKSIM_OK);
    std::fill(std::begin(vertices), std::end(vertices), 0.0); // Creation owns a copy.
    const double half_extents[] = {0.05, 0.05, 0.05};
    uint32_t present = 1;
    const auto outside = pose_at(0.8, 0.8, 0.8);
    assert(nksim_session_overlap_free_objects(space.session, &outside, half_extents, &present) == NKSIM_OK);
    assert(present == 0); // Inside its bounding box, outside its convex volume.
    const auto inside = pose_at(0.1, 0.1, 0.1);
    assert(nksim_session_overlap_free_objects(space.session, &inside, half_extents, &present) == NKSIM_OK);
    assert(present == 1);
    nksim_ray ray{};
    ray.struct_size = sizeof(ray);
    ray.origin[0] = ray.origin[1] = 0.1;
    ray.origin[2] = 2;
    ray.direction[2] = -1;
    ray.max_distance = 3;
    double distance = 0;
    assert(nksim_session_raycast(space.session, &ray, &distance) == NKSIM_OK);
    assert(near(distance, 1.2)); // The sloped face, not the box's top at distance 1.
    ray.origin[0] = ray.origin[1] = 0.8;
    assert(nksim_session_raycast(space.session, &ray, &distance) == NKSIM_OK);
    assert(near(distance, 3));
    assert(nksim_session_create_convex_object(space.session, &desc, vertices, 12, &object) == NKSIM_ERROR_INVALID_ARGUMENT);
}

void compound_objects_preserve_gaps_and_identity() {
    Space space;
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.shape.type = NKSIM_SHAPE_COMPOUND;
    desc.pose = pose_at(3, 4, 5);
    desc.pose.rotation[2] = desc.pose.rotation[3] = std::sqrt(0.5);
    desc.motion_type = NKSIM_MOTION_DYNAMIC;
    desc.mass = 1.0;
    std::vector<double> vertices;
    for (double side : {-1.0, 1.0})
        for (double x : {1.0, 2.0}) for (double y : {-0.5, 0.5}) for (double z : {0.0, 1.0}) {
            vertices.push_back(side*x); vertices.push_back(y); vertices.push_back(z);
        }
    uint32_t counts[] = {24, 24};
    nksim_object object = 0;
    assert(nksim_session_create_compound_convex_object(space.session, &desc, vertices.data(), 47,
        counts, 2, &object) == NKSIM_ERROR_INVALID_ARGUMENT && object == 0);
    assert(nksim_session_create_compound_convex_object(space.session, &desc, vertices.data(), 48,
        counts, 0, &object) == NKSIM_ERROR_INVALID_ARGUMENT && object == 0);
    assert(nksim_session_create_compound_convex_object(space.session, &desc, vertices.data(), 48,
        counts, 2, &object) == NKSIM_OK);
    std::fill(vertices.begin(),vertices.end(),0.0);
    counts[0] = counts[1] = 0; // Both arrays are copied.
    const double half[] = {0.05,0.05,0.05};
    uint32_t present = 1;
    auto query = pose_at(3,4,5.5);
    assert(nksim_session_overlap_free_objects(space.session,&query,half,&present) == NKSIM_OK && present == 0);
    assert(nksim_session_object_overlaps_volume(space.session,object,&query,0.05,0.05,0.05,&present) == NKSIM_OK && present == 0);
    query = pose_at(3,5.5,5.5);
    assert(nksim_session_object_overlaps_volume(space.session,object,&query,0.05,0.05,0.05,&present) == NKSIM_OK && present == 1);
    nksim_ray ray{};
    ray.struct_size = sizeof(ray);
    ray.origin[0] = 3; ray.origin[1] = 4; ray.origin[2] = 7;
    ray.direction[2] = -1; ray.max_distance = 3;
    double distance = 0;
    assert(nksim_session_raycast(space.session,&ray,&distance) == NKSIM_OK && near(distance,3));
    ray.origin[1] = 5.5;
    assert(nksim_session_raycast(space.session,&ray,&distance) == NKSIM_OK && near(distance,1));
    nksim_body body = 0;
    assert(nksim_session_get_object_body(space.session,object,&body) == NKSIM_OK && body != 0);
    assert(nksim_session_find_object(space.session,body,&object) == NKSIM_OK);
    assert(nksim_session_step(space.session,0,nullptr) == NKSIM_OK);
    assert(nksim_session_get_object_body(space.session,object,&body) == NKSIM_OK);
}

void compound_contacts_support_parts_without_filling_the_gap() {
    Space space;
    std::vector<double> vertices;
    for (const auto &bounds : std::vector<std::array<double,6>>{
        {-2,-2,-0.1,2,2,0}, {-2,-2,0,-1,2,2}, {1,-2,0,2,2,2}})
        for (double x : {bounds[0],bounds[3]}) for (double y : {bounds[1],bounds[4]})
            for (double z : {bounds[2],bounds[5]}) { vertices.push_back(x); vertices.push_back(y); vertices.push_back(z); }
    const uint32_t counts[] = {24,24,24};
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.shape.type = NKSIM_SHAPE_COMPOUND;
    desc.motion_type = NKSIM_MOTION_STATIC;
    desc.pose = pose_at(0.1,0.2,0.3);
    desc.pose.rotation[2] = desc.pose.rotation[3] = std::sqrt(0.5);
    nksim_object fixture = 0;
    assert(nksim_session_create_compound_convex_object(space.session,&desc,vertices.data(),72,counts,3,&fixture) == NKSIM_OK);
    auto box = [&](const nksim_pose &pose) {
        nksim_object_desc moving{};
        moving.struct_size = sizeof(moving);
        moving.shape.type = NKSIM_SHAPE_BOX;
        moving.shape.parameters[0] = moving.shape.parameters[1] = moving.shape.parameters[2] = 0.2;
        moving.motion_type = NKSIM_MOTION_DYNAMIC;
        moving.mass = 1;
        moving.pose = pose;
        nksim_object object = 0;
        assert(nksim_session_create_object(space.session,&moving,&object) == NKSIM_OK);
        return object;
    };
    const auto gap = box(pose_at(0.1,0.2,1.3));
    const auto wall = box(pose_at(0.1,1.7,3.3));
    for (int i = 0; i < 100; ++i) assert(nksim_session_step(space.session,0,nullptr) == NKSIM_OK);
    nksim_frame frame = 0;
    assert(nksim_session_capture(space.session,&frame) == NKSIM_OK);
    auto pose = pose_at(0,0,0);
    assert(nksim_frame_get_object_pose(frame,gap,&pose) == NKSIM_OK);
    assert(near(pose.position[0],0.1) && near(pose.position[1],0.2) && near(pose.position[2],0.5));
    assert(nksim_frame_get_object_pose(frame,wall,&pose) == NKSIM_OK);
    assert(near(pose.position[0],0.1) && near(pose.position[1],1.7) && near(pose.position[2],2.5));
    nksim_frame_destroy(frame);
}

void actors_follow_timed_keyframes() {
    Space space;
    nksim_actor_part parts[2]{};
    for (auto &part : parts) {
        part.struct_size = sizeof(part);
        part.shape = capsule(0.1, 0.4);
    }
    parts[0].pose = pose_at(0.0, 0.0, 1.0);
    parts[1].pose = pose_at(0.0, 0.0, 0.5);
    nksim_actor actor = 0;
    assert(nksim_session_create_actor(space.session, parts, 2, &actor) == NKSIM_OK);

    // One second of motion 1 m along +X, pushed ahead of the clock.
    nksim_pose start[] = {pose_at(0.0, 0.0, 1.0), pose_at(0.0, 0.0, 0.5)};
    nksim_pose end[] = {pose_at(1.0, 0.0, 1.0), pose_at(1.0, 0.0, 0.5)};
    assert(nksim_session_push_actor_keyframe(space.session, actor, 0.0, start, 2) == NKSIM_OK);
    assert(nksim_session_push_actor_keyframe(space.session, actor, 1.0, end, 1) ==
           NKSIM_ERROR_INVALID_ARGUMENT);
    assert(nksim_session_push_actor_keyframe(space.session, actor, 1.0, end, 2) == NKSIM_OK);
    for (int tick = 0; tick < 25; ++tick)
        assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    auto pose = actor_pose(space.session, actor, 1);
    assert(near(pose.position[0], 0.25) && near(pose.position[2], 0.5));
    nksim_body body = 0;
    assert(nksim_session_get_actor_body(space.session, actor, 1, &body) == NKSIM_OK);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_session_get_body_state(space.session, body, &state) == NKSIM_OK);
    assert(near(state.linear_velocity[0], 1.0, 1e-6));

    // Replanning replaces the future: stop where it is by t = 0.5.
    nksim_pose hold[] = {pose_at(0.5, 0.0, 1.0), pose_at(0.5, 0.0, 0.5)};
    assert(nksim_session_push_actor_keyframe(space.session, actor, 0.5, hold, 2) == NKSIM_OK);
    for (int tick = 0; tick < 50; ++tick)
        assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    pose = actor_pose(space.session, actor, 0);
    assert(near(pose.position[0], 0.5) && near(pose.position[2], 1.0));
    assert(nksim_session_get_body_state(space.session, body, &state) == NKSIM_OK);
    assert(near(state.linear_velocity[0], 0.0, 1e-9));

    // A ray down the X axis at hip height meets the lower capsule's side.
    nksim_ray ray{};
    ray.struct_size = sizeof(ray);
    ray.origin[0] = -2.0;
    ray.origin[2] = 0.5;
    ray.direction[0] = 1.0;
    ray.max_distance = 10.0;
    double distance = 0.0;
    assert(nksim_session_raycast(space.session, &ray, &distance) == NKSIM_OK);
    assert(near(distance, 2.4));
    // Straight down onto the upper capsule's top hemisphere.
    ray.origin[0] = 0.5;
    ray.origin[2] = 3.0;
    ray.direction[0] = 0.0;
    ray.direction[2] = -1.0;
    assert(nksim_session_raycast(space.session, &ray, &distance) == NKSIM_OK);
    assert(near(distance, 3.0 - 1.3));
    // Nothing within reach reports the reach.
    ray.max_distance = 1.0;
    assert(nksim_session_raycast(space.session, &ray, &distance) == NKSIM_OK);
    assert(distance == 1.0);

    assert(nksim_session_stop(space.session) == NKSIM_OK);
    assert(nksim_session_reset(space.session) == NKSIM_OK);
    pose = actor_pose(space.session, actor, 0);
    assert(near(pose.position[0], 0.0));
}

struct Recorder {
    std::string log;
    bool fail_prepare = false;
    nksim_session session = 0;
    uint64_t last_step = 0;
    uint64_t owner_time = 0;
};

void participants_share_every_tick() {
    Space space;
    Recorder first, second;
    first.session = second.session = space.session;
    auto describe = [](Recorder &recorder) {
        nksim_participant_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.user = &recorder;
        desc.prepare = [](void *user, const nksim_tick *tick) -> nksim_result {
            auto &self = *static_cast<Recorder *>(user);
            self.log += "p";
            self.owner_time = tick->owner_time_ns;
            return self.fail_prepare ? NKSIM_ERROR_INVALID_ARGUMENT : NKSIM_OK;
        };
        desc.discard = [](void *user) { static_cast<Recorder *>(user)->log += "d"; };
        desc.submit = [](void *user, const nksim_tick *tick) -> nksim_result {
            auto &self = *static_cast<Recorder *>(user);
            assert(tick->owner_time_ns == self.owner_time);
            self.log += "s";
            return NKSIM_OK;
        };
        desc.publish = [](void *user, const nksim_tick *tick) -> nksim_result {
            auto &self = *static_cast<Recorder *>(user);
            self.log += "u";
            assert(tick->owner_time_ns == self.owner_time);
            // Callbacks may re-enter the session under its lock.
            assert(nksim_session_latest_snapshot(self.session) != 0);
            self.last_step = tick->step_index;
            return NKSIM_OK;
        };
        desc.reset = [](void *user) -> nksim_result {
            static_cast<Recorder *>(user)->log += "r";
            return NKSIM_OK;
        };
        return desc;
    };
    nksim_participant a = 0, b = 0;
    auto desc_a = describe(first), desc_b = describe(second);
    assert(nksim_session_add_participant(space.session, &desc_a, &a) == NKSIM_OK);
    assert(nksim_session_add_participant(space.session, &desc_b, &b) == NKSIM_OK);
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    assert(first.log == "psu" && second.log == "psu" && second.last_step == 1);
    assert(first.owner_time == 10'000'000 && second.owner_time == first.owner_time);

    // One participant's rejected commands discard everyone's and skip the tick.
    second.fail_prepare = true;
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(first.log == "psupd" && second.log == "psupd");
    nksim_session_status status{};
    status.struct_size = sizeof(status);
    assert(nksim_session_get_status(space.session, &status) == NKSIM_OK);
    assert(status.step_index == 1 && status.hosted && status.sealed);

    second.fail_prepare = false;
    assert(nksim_session_remove_participant(space.session, b) == NKSIM_OK);
    assert(nksim_session_start(space.session) == NKSIM_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_stop(space.session) == NKSIM_OK);
    assert(first.last_step > 2 && second.last_step == 1);
    assert(nksim_session_reset(space.session) == NKSIM_OK);
    assert(first.log.back() == 'r' && second.log == "psupd");
    assert(nksim_session_get_status(space.session, &status) == NKSIM_OK);
    assert(status.step_index == 0 && !status.hosted && !status.running);
    assert(near(status.gravity[2], -9.81) && near(status.fixed_timestep, 0.01));
}

void owner_paced_ticks() {
    Space space;
    constexpr uint64_t tick_ns = 10'000'000;
    uint32_t due = 99;
    assert(nksim_session_due_ticks(space.session, 0, 5, nullptr) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(nksim_session_due_ticks(space.session, 0, 5, &due) == NKSIM_OK && due == 0);
    // A partial tick carries over to the next call.
    assert(nksim_session_due_ticks(space.session, tick_ns * 3 / 2, 5, &due) == NKSIM_OK && due == 1);
    assert(nksim_session_due_ticks(space.session, tick_ns / 2, 5, &due) == NKSIM_OK && due == 1);
    // A stalled owner loses the excess instead of bursting to catch up.
    assert(nksim_session_due_ticks(space.session, tick_ns * 20, 5, &due) == NKSIM_OK && due == 5);
    assert(nksim_session_due_ticks(space.session, 0, 5, &due) == NKSIM_OK && due == 0);

    // A paced step advances the clock like an explicit one, and participants
    // see a realtime tick where an explicit step is not.
    struct Seen {
        int realtime = -1;
    } seen;
    nksim_participant_desc participant{};
    participant.struct_size = sizeof(participant);
    participant.user = &seen;
    participant.submit = [](void *user, const nksim_tick *tick) -> nksim_result {
        static_cast<Seen *>(user)->realtime = static_cast<int>(tick->realtime);
        return NKSIM_OK;
    };
    nksim_participant id = 0;
    assert(nksim_session_add_participant(space.session, &participant, &id) == NKSIM_OK);
    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(nksim_session_step_paced(space.session, &clock) == NKSIM_OK);
    assert(clock.step_index == 1 && near(clock.time, 0.01) && seen.realtime == 1);
    assert(nksim_session_step(space.session, 0, &clock) == NKSIM_OK);
    assert(clock.step_index == 2 && seen.realtime == 0);

    // The owner loop and the paced calls are exclusive.
    assert(nksim_session_remove_participant(space.session, id) == NKSIM_OK);
    assert(nksim_session_start(space.session) == NKSIM_OK);
    assert(nksim_session_due_ticks(space.session, tick_ns, 5, &due) == NKSIM_ERROR_INVALID_STATE &&
           due == 0);
    assert(nksim_session_step_paced(space.session, nullptr) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_session_stop(space.session) == NKSIM_OK);
}

void owned_carriers_use_current_targets_in_dependency_order() {
    Space space;
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc); desc.motion_type = NKSIM_MOTION_DYNAMIC; desc.mass = 1;
    desc.shape.type = NKSIM_SHAPE_BOX;
    desc.shape.parameters[0] = desc.shape.parameters[1] = desc.shape.parameters[2] = 0.05;
    desc.pose = pose_at(0, 0, 3);
    nksim_object leaf = 0, parent = 0, carrier = 0;
    assert(nksim_session_create_object(space.session, &desc, &leaf) == NKSIM_OK);
    desc.pose = pose_at(1, 0, 3);
    assert(nksim_session_create_object(space.session, &desc, &parent) == NKSIM_OK);
    desc.motion_type = NKSIM_MOTION_KINEMATIC; desc.pose = pose_at(0, 0, 3);
    assert(nksim_session_create_object(space.session, &desc, &carrier) == NKSIM_OK);
    nksim_body parent_body = 0, carrier_body = 0, leaf_body = 0;
    assert(nksim_session_get_object_body(space.session, parent, &parent_body) == NKSIM_OK);
    assert(nksim_session_get_object_body(space.session, carrier, &carrier_body) == NKSIM_OK);
    assert(nksim_session_get_object_body(space.session, leaf, &leaf_body) == NKSIM_OK);
    const auto parent_offset = pose_at(0.2, 0, 0), leaf_offset = pose_at(0.4, 0, 0);
    assert(nksim_session_hold_object(space.session, parent, carrier_body, &parent_offset) == NKSIM_OK);
    assert(nksim_session_hold_object(space.session, leaf, parent_body, &leaf_offset) == NKSIM_OK);
    for (int i = 1; i <= 3; ++i) {
        const double angle = i * std::acos(-1.0) / 6;
        auto target = pose_at(1, 2, 3);
        target.rotation[2] = std::sin(angle / 2); target.rotation[3] = std::cos(angle / 2);
        assert(nksim_session_drive_object(space.session, carrier, &target) == NKSIM_OK);
        assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
        auto actual = pose_at(0, 0, 0);
        assert(nksim_session_get_body_pose(space.session, leaf_body, &actual) == NKSIM_OK);
        assert(std::abs(actual.position[0] - (1 + 0.6 * std::cos(angle))) < 1e-6);
        assert(std::abs(actual.position[1] - (2 + 0.6 * std::sin(angle))) < 1e-6);
        assert(std::abs(actual.position[2] - 3) < 1e-6);
    }
    assert(nksim_session_stop(space.session) == NKSIM_OK);
    assert(nksim_session_release_object(space.session, parent) == NKSIM_OK);
    nksim_joint fixed = 0;
    assert(nksim_session_create_fixed_joint(space.session, carrier, parent, &fixed) == NKSIM_OK);
    const auto target = pose_at(2, 3, 3);
    assert(nksim_session_drive_object(space.session, carrier, &target) == NKSIM_OK);
    assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    auto actual = pose_at(0, 0, 0);
    assert(nksim_session_get_body_pose(space.session, leaf_body, &actual) == NKSIM_OK);
    assert(std::abs(actual.position[0] - 3.4) < 1e-6);
    assert(std::abs(actual.position[1] - 3) < 1e-6);
}

} // namespace

// Ground planes and cylinders are session objects too: a plane must be static,
// and rays hit a plane's surface and a cylinder's side and caps.
void planes_and_cylinders_are_objects() {
    Space space;
    nksim_object_desc plane{};
    plane.struct_size = sizeof(plane);
    plane.motion_type = NKSIM_MOTION_DYNAMIC;
    plane.mass = 1.0;
    plane.shape.type = NKSIM_SHAPE_PLANE;
    plane.shape.parameters[2] = 1.0;
    plane.pose = pose_at(0.0, 0.0, 0.0);
    nksim_object ground = 0;
    assert(nksim_session_create_object(space.session, &plane, &ground) == NKSIM_ERROR_INVALID_ARGUMENT);
    plane.motion_type = NKSIM_MOTION_STATIC;
    plane.mass = 0.0;
    assert(nksim_session_create_object(space.session, &plane, &ground) == NKSIM_OK);
    nksim_object_desc column{};
    column.struct_size = sizeof(column);
    column.motion_type = NKSIM_MOTION_STATIC;
    column.shape.type = NKSIM_SHAPE_CYLINDER;
    column.shape.parameters[0] = 0.5; // radius
    column.shape.parameters[1] = 2.0; // full height
    column.pose = pose_at(3.0, 0.0, 1.0);
    nksim_object cylinder = 0;
    assert(nksim_session_create_object(space.session, &column, &cylinder) == NKSIM_OK);

    nksim_ray ray{};
    ray.struct_size = sizeof(ray);
    ray.origin[2] = 1.5;
    ray.direction[2] = -1.0;
    ray.max_distance = 10.0;
    double distance = 0.0;
    assert(nksim_session_raycast(space.session, &ray, &distance) == NKSIM_OK);
    assert(near(distance, 1.5)); // Down onto the ground plane.
    ray.direction[2] = 0.0;
    ray.direction[0] = 1.0;
    assert(nksim_session_raycast(space.session, &ray, &distance) == NKSIM_OK);
    assert(near(distance, 2.5)); // Sideways into the cylinder's side.
    ray.origin[0] = 3.2;
    ray.origin[2] = 4.0;
    ray.direction[0] = 0.0;
    ray.direction[2] = -1.0;
    assert(nksim_session_raycast(space.session, &ray, &distance) == NKSIM_OK);
    assert(near(distance, 2.0)); // Down onto the cylinder's top cap at z = 2.
}

int main() {
    // Solver extensions consume reserved slots without changing the existing ABI.
    static_assert(sizeof(nksim_world_desc) == 96);
    static_assert(offsetof(nksim_world_desc, integrator) == 80);
    {
        nkscene_scene scene = 0;
        assert(nkscene_scene_create(&scene) == NKS_OK);
        nksim_world_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.scene = scene;
        desc.fixed_timestep = 0.01;
        desc.physics_substeps = 1;
        nksim_world world = 0;
        for (double ratio : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
            desc.friction_impedance_ratio = ratio;
            assert(nksim_world_create(&desc, &world) == NKSIM_ERROR_INVALID_ARGUMENT);
        }
        desc.friction_impedance_ratio = 10.0;
        desc.noslip_iterations = 2;
        for (double tolerance : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()}) {
            desc.solver_tolerance = tolerance;
            assert(nksim_world_create(&desc, &world) == NKSIM_ERROR_INVALID_ARGUMENT);
        }
        desc.solver_tolerance = 1e-10;
        desc.arena_memory_bytes = 128u * 1024u * 1024u;
        assert(nksim_world_create(&desc, &world) == NKSIM_OK);
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
    }
    // A completed background builder hands an unstarted world to the UI owner.
    {
        std::unique_ptr<Space> prepared;
        nksim_object object = 0;
        std::thread builder([&] {
            prepared = std::make_unique<Space>();
            nksim_object_desc desc{};
            desc.struct_size = sizeof(desc);
            desc.motion_type = NKSIM_MOTION_DYNAMIC;
            desc.mass = 1.0;
            desc.shape = capsule(0.1, 0.2);
            desc.pose = pose_at(0.0, 0.0, 1.0);
            assert(nksim_session_create_object(prepared->session, &desc, &object) == NKSIM_OK);
        });
        builder.join();
        assert(nksim_world_begin_topology_update(prepared->world) == NKSIM_ERROR_WRONG_THREAD);
        assert(nksim_session_stop(prepared->session) == NKSIM_OK);
        assert(nksim_world_begin_topology_update(prepared->world) == NKSIM_OK);
        assert(nksim_world_end_topology_update(prepared->world) == NKSIM_OK);
        assert(nksim_session_step(prepared->session, 0, nullptr) == NKSIM_OK);
        assert(nksim_session_reset(prepared->session) == NKSIM_OK);
    }
    owner_paced_ticks();
    {
        Space space;
        nksim_object_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.motion_type = NKSIM_MOTION_DYNAMIC;
        desc.mass = 1.0;
        desc.shape.type = NKSIM_SHAPE_BOX;
        desc.shape.parameters[0] = desc.shape.parameters[1] = desc.shape.parameters[2] = 0.1;
        desc.pose = pose_at(0.0, 0.0, 2.0);
        nksim_object payload = 0;
        assert(nksim_session_create_object(space.session, &desc, &payload) == NKSIM_OK);
        desc.motion_type = NKSIM_MOTION_STATIC;
        desc.mass = 0.0;
        desc.pose = pose_at(5.0, 0.0, 0.0);
        nksim_object fixed = 0;
        assert(nksim_session_create_object(space.session, &desc, &fixed) == NKSIM_OK);
        desc.shape.parameters[0] = desc.shape.parameters[1] = 10.0;
        desc.shape.parameters[2] = 0.5;
        desc.pose = pose_at(0.0, 0.0, -0.5);
        nksim_object floor = 0;
        assert(nksim_session_create_object(space.session, &desc, &floor) == NKSIM_OK);
        nksim_actor_part part{};
        part.struct_size = sizeof(part);
        part.shape.type = NKSIM_SHAPE_BOX;
        part.shape.parameters[0] = part.shape.parameters[1] = part.shape.parameters[2] = 0.1;
        part.pose = pose_at(0.0, 0.0, 2.0);
        nksim_actor actor = 0;
        assert(nksim_session_create_actor(space.session, &part, 1, &actor) == NKSIM_OK);
        nksim_body carrier = 0, body = 0, reported = 0;
        assert(nksim_session_get_actor_body(space.session, actor, 0, &carrier) == NKSIM_OK);
        assert(nksim_session_get_object_body(space.session, payload, &body) == NKSIM_OK);
        const auto offset = pose_at(0.0, 0.0, 0.3);
        assert(nksim_session_hold_object(space.session, fixed, carrier, &offset) ==
               NKSIM_ERROR_INVALID_STATE);
        assert(nksim_session_hold_object(space.session, payload, 0, &offset) ==
               NKSIM_ERROR_INVALID_ARGUMENT);
        nksim_body fixed_body = 0;
        assert(nksim_session_get_object_body(space.session, fixed, &fixed_body) == NKSIM_OK);
        assert(nksim_session_hold_object(space.session, payload, fixed_body, &offset) == NKSIM_OK);
        assert(nksim_session_get_object_carrier(space.session, payload, &reported) == NKSIM_OK &&
               reported == fixed_body);
        assert(nksim_session_hold_object(space.session, payload, carrier, &offset) == NKSIM_OK);
        assert(nksim_session_get_object_carrier(space.session, payload, &reported) == NKSIM_OK &&
               reported == carrier);
        assert(nksim_session_hold_object(space.session, payload, body, &offset) ==
               NKSIM_ERROR_INVALID_ARGUMENT);
        const auto start = pose_at(0.0, 0.0, 2.0);
        const auto end = pose_at(1.0, 0.0, 2.0);
        assert(nksim_session_push_actor_keyframe(space.session, actor, 0.0, &start, 1) == NKSIM_OK);
        assert(nksim_session_push_actor_keyframe(space.session, actor, 1.0, &end, 1) == NKSIM_OK);
        for (int tick = 1; tick <= 20; ++tick) {
            assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
            nksim_body_state state{};
            state.struct_size = sizeof(state);
            assert(nksim_session_get_body_state(space.session, body, &state) == NKSIM_OK);
            assert(near(state.position[0], tick * 0.01, 1e-9));
            assert(near(state.position[2], 2.3, 1e-9));
            assert(near(state.linear_velocity[0], 1.0, 1e-9));
        }
        assert(nksim_session_release_object(space.session, payload) == NKSIM_OK);
        assert(nksim_session_release_object(space.session, payload) == NKSIM_ERROR_INVALID_STATE);
        assert(nksim_session_stop(space.session) == NKSIM_OK);
        assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
        nksim_body_state released{};
        released.struct_size = sizeof(released);
        assert(nksim_session_get_body_state(space.session, body, &released) == NKSIM_OK);
        assert(released.linear_velocity[0] > 0.9 && released.position[2] < 2.3);
        for (int tick = 0; tick < 300; ++tick)
            assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
        assert(nksim_session_get_body_state(space.session, body, &released) == NKSIM_OK);
        assert(near(released.position[2], 0.1, 1e-4));
        assert(nksim_session_stop(space.session) == NKSIM_OK);
        assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
        assert(nksim_session_hold_object(space.session, payload, carrier, &offset) == NKSIM_OK);
        assert(nksim_session_stop(space.session) == NKSIM_OK);
        assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
        nksim_body_state held_again{};
        held_again.struct_size = sizeof(held_again);
        assert(nksim_session_get_body_state(space.session, body, &held_again) == NKSIM_OK);
        assert(near(held_again.position[0], 1.0, 1e-9));
        assert(near(held_again.position[2], 2.3, 1e-9));
        assert(nksim_session_stop(space.session) == NKSIM_OK);
        assert(nksim_session_reset(space.session) == NKSIM_OK);
        assert(nksim_session_get_object_carrier(space.session, payload, &reported) == NKSIM_OK &&
               reported == 0);
        nksim_body_state reset{};
        reset.struct_size = sizeof(reset);
        assert(nksim_session_get_body_state(space.session, body, &reset) == NKSIM_OK);
        assert(near(reset.position[0], 0.0) && near(reset.position[2], 2.0));
        assert(near(reset.linear_velocity[0], 0.0) &&
               near(reset.linear_velocity[1], 0.0) &&
               near(reset.linear_velocity[2], 0.0));
        assert(nksim_session_hold_object(space.session, payload, carrier, &offset) == NKSIM_OK);
        assert(nksim_session_reset(space.session) == NKSIM_OK);
        assert(nksim_session_get_object_carrier(space.session, payload, &reported) == NKSIM_OK &&
               reported == 0);
        assert(nksim_session_hold_object(space.session, payload, carrier, &offset) == NKSIM_OK);
        assert(nksim_session_destroy_actor(space.session, actor) == NKSIM_OK);
        assert(nksim_session_get_object_carrier(space.session, payload, &reported) == NKSIM_OK &&
               reported == 0);
        assert(nksim_session_hold_object(space.session, payload, fixed_body, &offset) == NKSIM_OK);
        assert(nksim_session_destroy_object(space.session, payload) == NKSIM_OK);
        assert(nksim_session_get_object_carrier(space.session, payload, &reported) ==
               NKSIM_ERROR_INVALID_HANDLE);
        // An object body is also a valid carrier. Destroying it must clear
        // dependent holds before removing the body handle.
        desc.motion_type = NKSIM_MOTION_DYNAMIC;
        desc.mass = 1.0;
        desc.pose = pose_at(0.0, 0.0, 2.0);
        nksim_object parent = 0, child = 0;
        assert(nksim_session_create_object(space.session, &desc, &parent) == NKSIM_OK);
        desc.pose = pose_at(0.0, 0.0, 2.3);
        assert(nksim_session_create_object(space.session, &desc, &child) == NKSIM_OK);
        nksim_body parent_body = 0;
        assert(nksim_session_get_object_body(space.session, parent, &parent_body) == NKSIM_OK);
        assert(nksim_session_hold_object(space.session, child, parent_body, &offset) == NKSIM_OK);
        assert(nksim_session_destroy_object(space.session, parent) == NKSIM_OK);
        assert(nksim_session_get_object_carrier(space.session, child, &reported) == NKSIM_OK &&
               reported == 0);
        assert(nksim_session_step(space.session, 0, nullptr) == NKSIM_OK);
    }
    captured_contacts_are_immutable_and_checked();
    planes_and_cylinders_are_objects();
    objects_follow_their_motion_type();
    presence_counts_only_free_boxes_in_its_volume();
    presence_uses_loose_object_shapes();
    convex_objects_keep_their_actual_volume();
    compound_objects_preserve_empty_gaps();
    fixed_connections_use_reference_frames_and_session_ownership();
    measured_fixed_connections_preserve_seated_pose_and_reset();
    compound_objects_preserve_gaps_and_identity();
    compound_contacts_support_parts_without_filling_the_gap();
    actors_follow_timed_keyframes();
    participants_share_every_tick();
    owned_carriers_use_current_targets_in_dependency_order();
    return 0;
}
