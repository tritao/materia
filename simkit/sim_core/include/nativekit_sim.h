#ifndef NATIVEKIT_SIM_H
#define NATIVEKIT_SIM_H

#include "nativekit.h"
#include "nativekit_scene.h"

#include <stdint.h>

#if defined(_WIN32)
#if defined(NKSIM_STATIC)
#define NKSIM_API
#elif defined(NKSIM_BUILDING_LIBRARY)
#define NKSIM_API __declspec(dllexport)
#else
#define NKSIM_API __declspec(dllimport)
#endif
#define NKSIM_CALL __cdecl
#else
#define NKSIM_API __attribute__((visibility("default")))
#define NKSIM_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t nksim_world NK_HANDLE NK_HANDLE_DESTROY(nksim_world_destroy);
typedef uint32_t nksim_body NK_HANDLE;
typedef uint32_t nksim_joint NK_HANDLE;
typedef uint32_t nksim_shape NK_HANDLE;
typedef uint32_t nksim_snapshot NK_HANDLE NK_HANDLE_DESTROY(nksim_snapshot_destroy);

typedef int32_t nksim_result;

enum {
    NKSIM_OK = 0,
    NKSIM_ERROR_INVALID_ARGUMENT = -1,
    NKSIM_ERROR_INVALID_HANDLE = -2,
    NKSIM_ERROR_INVALID_STATE = -3,
    NKSIM_ERROR_OUT_OF_MEMORY = -4,
    NKSIM_ERROR_UNSUPPORTED = -5,
    NKSIM_ERROR_STALE_ID = -6,
    NKSIM_ERROR_WRONG_THREAD = -7,
    NKSIM_ERROR_BACKEND = -8,
    NKSIM_ERROR_SCENE = -9
};

#define NKSIM_INVALID_WORLD ((nksim_world)0)
#define NKSIM_INVALID_BODY ((nksim_body)0)
#define NKSIM_INVALID_JOINT ((nksim_joint)0)
#define NKSIM_INVALID_SHAPE ((nksim_shape)0)
#define NKSIM_INVALID_SNAPSHOT ((nksim_snapshot)0)

enum {
    NKSIM_MOTION_STATIC = 0,
    NKSIM_MOTION_KINEMATIC = 1,
    NKSIM_MOTION_DYNAMIC = 2
};

enum {
    NKSIM_SHAPE_BOX = 1,
    NKSIM_SHAPE_SPHERE = 2,
    NKSIM_SHAPE_CAPSULE = 3,
    NKSIM_SHAPE_PLANE = 4
};

enum {
    NKSIM_JOINT_FIXED = 1,
    NKSIM_JOINT_REVOLUTE = 2,
    NKSIM_JOINT_PRISMATIC = 3
};

enum {
    NKSIM_JOINT_TARGET_POSITION = 1,
    NKSIM_JOINT_TARGET_VELOCITY = 2,
    NKSIM_JOINT_TARGET_EFFORT = 3
};

typedef struct nksim_world_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    nkscene_scene scene;
    double fixed_timestep;
    uint32_t physics_substeps;
    double gravity[3];
    uint64_t reserved[4];
} nksim_world_desc;

typedef struct nksim_clock {
    uint32_t struct_size NK_STRUCT_SIZE;
    double time;
    double fixed_timestep;
    uint64_t step_index;
} nksim_clock;

typedef struct nksim_step_result {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint64_t step_index;
    double simulation_time;
    uint32_t physics_substeps;
    nkscene_change_set scene_changes;
    uint64_t reserved[4];
} nksim_step_result;

typedef struct nksim_shape_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t type;
    double parameters[4];
    uint64_t reserved[2];
} nksim_shape_desc;

typedef struct nksim_body_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    nkscene_node_id node;
    uint32_t motion_type;
    double mass;
    nksim_shape shape;
    uint32_t collision_layer;
    uint32_t collision_mask;
    uint64_t reserved[4];
} nksim_body_desc;

typedef struct nksim_body_state {
    uint32_t struct_size NK_STRUCT_SIZE;
    nksim_body body;
    nkscene_node_id node;
    double position[3];
    /* Quaternion in x, y, z, w order. */
    double rotation[4];
    double linear_velocity[3];
    double angular_velocity[3];
    uint32_t sleeping NK_BOOL32;
    uint32_t reserved0;
    uint64_t reserved[2];
} nksim_body_state;

typedef struct nksim_body_force {
    uint32_t struct_size NK_STRUCT_SIZE;
    nksim_body body;
    double force[3];
    double torque[3];
} nksim_body_force;

typedef struct nksim_joint_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t type;
    nksim_body body_a;
    nksim_body body_b;
    double anchor_a[3];
    double anchor_b[3];
    double axis_a[3];
    double lower_limit;
    double upper_limit;
    double max_force;
    uint64_t reserved[4];
} nksim_joint_desc;

typedef struct nksim_joint_state {
    uint32_t struct_size NK_STRUCT_SIZE;
    nksim_joint joint;
    double position;
    double velocity;
    double effort;
    uint32_t reserved0;
    uint64_t reserved[2];
} nksim_joint_state;

typedef struct nksim_joint_target {
    uint32_t struct_size NK_STRUCT_SIZE;
    nksim_joint joint;
    uint32_t mode;
    double target;
    double max_force;
} nksim_joint_target;

enum { NKSIM_SNAPSHOT_PAGE_CAPACITY = 64u };

typedef struct nksim_snapshot_body_page {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint64_t start_index;
    uint32_t count;
    nksim_body_state bodies[NKSIM_SNAPSHOT_PAGE_CAPACITY];
} nksim_snapshot_body_page;

typedef struct nksim_snapshot_joint_page {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint64_t start_index;
    uint32_t count;
    nksim_joint_state joints[NKSIM_SNAPSHOT_PAGE_CAPACITY];
} nksim_snapshot_joint_page;

NKSIM_API nksim_result NKSIM_CALL nksim_world_create(
    const nksim_world_desc *desc, nksim_world *out_world NK_OUT NK_OWNED);
NKSIM_API void NKSIM_CALL nksim_world_destroy(nksim_world world);
NKSIM_API nksim_result NKSIM_CALL nksim_world_get_clock(nksim_world world,
                                                         nksim_clock *out_clock NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_world_step(nksim_world world,
                                                   nksim_step_result *out_result NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_world_apply_forces(
    nksim_world world, const nksim_body_force *forces NK_IN_ARRAY(count), uint32_t count);
NKSIM_API nksim_result NKSIM_CALL nksim_world_set_joint_targets(
    nksim_world world, const nksim_joint_target *targets NK_IN_ARRAY(count), uint32_t count);
NKSIM_API nksim_result NKSIM_CALL nksim_world_snapshot(
    nksim_world world, nksim_snapshot *out_snapshot NK_OUT NK_OWNED);

NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_box(
    nksim_world world, const double half_extents[3],
    nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_sphere(
    nksim_world world, double radius, nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_capsule(
    nksim_world world, double radius, double height, nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_plane(
    nksim_world world, const double normal[3], double offset,
    nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create(
    nksim_world world, const nksim_shape_desc *desc, nksim_shape *out_shape NK_OUT);
NKSIM_API void NKSIM_CALL nksim_shape_destroy(nksim_world world, nksim_shape shape);

NKSIM_API nksim_result NKSIM_CALL nksim_body_create(
    nksim_world world, const nksim_body_desc *desc, nksim_body *out_body NK_OUT);
NKSIM_API void NKSIM_CALL nksim_body_destroy(nksim_world world, nksim_body body);
NKSIM_API nksim_result NKSIM_CALL nksim_body_get_state(
    nksim_world world, nksim_body body, nksim_body_state *out_state NK_INOUT);
/** Replaces a body's pose and velocities on the world owner thread. */
NKSIM_API nksim_result NKSIM_CALL nksim_body_set_state(
    nksim_world world, nksim_body body, const nksim_body_state *state);
NKSIM_API nksim_result NKSIM_CALL nksim_body_reset(nksim_world world, nksim_body body);
/** Restores body state and the fixed-step clock to their initial values. */
NKSIM_API nksim_result NKSIM_CALL nksim_world_reset(nksim_world world);

NKSIM_API nksim_result NKSIM_CALL nksim_joint_create(
    nksim_world world, const nksim_joint_desc *desc, nksim_joint *out_joint NK_OUT);
NKSIM_API void NKSIM_CALL nksim_joint_destroy(nksim_world world, nksim_joint joint);
NKSIM_API nksim_result NKSIM_CALL nksim_joint_get_state(
    nksim_world world, nksim_joint joint, nksim_joint_state *out_state NK_INOUT);

NKSIM_API void NKSIM_CALL nksim_snapshot_destroy(nksim_snapshot snapshot);
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_clock(
    nksim_snapshot snapshot, nksim_clock *out_clock NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_body_count(
    nksim_snapshot snapshot, uint64_t *out_count NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_body(
    nksim_snapshot snapshot, uint64_t index, nksim_body_state *out_state NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_body_page(
    nksim_snapshot snapshot, uint64_t start_index,
    nksim_snapshot_body_page *out_page NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_joint_count(
    nksim_snapshot snapshot, uint64_t *out_count NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_joint(
    nksim_snapshot snapshot, uint64_t index, nksim_joint_state *out_state NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_joint_page(
    nksim_snapshot snapshot, uint64_t start_index,
    nksim_snapshot_joint_page *out_page NK_INOUT);

#ifdef __cplusplus
}
#endif

#endif /* NATIVEKIT_SIM_H */
