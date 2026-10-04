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
    // Follows its scene node's world pose each tick. The body's velocity is
    // the node's motion over the tick (finite difference of the previous and
    // new pose), except on the first tick after nksim_body_set_state(),
    // nksim_body_reset(), or nksim_world_reset(), which carries the twist
    // those writes supplied so a teleport does not read as a velocity.
    // nksim_body_drive() instead supplies the next tick's pose and twist
    // exactly in double precision, free of the single-precision scene node's
    // rounding.
    NKSIM_MOTION_KINEMATIC = 1,
    NKSIM_MOTION_DYNAMIC = 2
};

enum {
    NKSIM_SHAPE_BOX = 1,
    NKSIM_SHAPE_SPHERE = 2,
    NKSIM_SHAPE_CAPSULE = 3,
    NKSIM_SHAPE_PLANE = 4,
    NKSIM_SHAPE_CONVEX = 5,
    NKSIM_SHAPE_COMPOUND = 6,
    /** Solid cylinder along local Z: parameters radius, full height. */
    NKSIM_SHAPE_CYLINDER = 7
};

enum {
    NKSIM_JOINT_FIXED = 1,
    NKSIM_JOINT_REVOLUTE = 2,
    NKSIM_JOINT_PRISMATIC = 3,
    /* Closures only (see nksim_closure_desc): */
    NKSIM_JOINT_SPHERICAL = 4,   /**< A shared point. */
    NKSIM_JOINT_CYLINDRICAL = 5, /**< A shared axis line, free to slide and turn along it. */
    NKSIM_JOINT_PLANAR = 6       /**< body_b slides and turns in body_a's plane through anchor_a with normal axis_a. */
};

enum {
    NKSIM_JOINT_TARGET_POSITION = 1,
    NKSIM_JOINT_TARGET_VELOCITY = 2,
    NKSIM_JOINT_TARGET_EFFORT = 3,
    /**
     * Joint-space servo evaluated every physics substep:
     * effort = stiffness * (target - q) + damping * (velocity - qdot) + feedforward,
     * clamped to max_force. This is the command legged-robot controllers emit.
     */
    NKSIM_JOINT_TARGET_SERVO = 4
};

/** Numerical integrators a backend may offer; the default is the backend's own. */
enum {
    NKSIM_INTEGRATOR_DEFAULT = 0,
    NKSIM_INTEGRATOR_EULER = 1,       /**< Semi-implicit Euler. */
    NKSIM_INTEGRATOR_IMPLICIT_FAST = 2, /**< Implicit in velocity-dependent forces. */
    NKSIM_INTEGRATOR_RK4 = 3
};

/** Friction cone approximations. */
enum {
    NKSIM_FRICTION_CONE_DEFAULT = 0,
    NKSIM_FRICTION_CONE_PYRAMIDAL = 1,
    NKSIM_FRICTION_CONE_ELLIPTIC = 2
};

typedef struct nksim_world_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    nkscene_scene scene;
    double fixed_timestep;
    uint32_t physics_substeps;
    double gravity[3];
    uint64_t reserved[4];
    /* Optional when struct_size includes this tail; zero keeps backend defaults. */
    uint32_t integrator; /**< NKSIM_INTEGRATOR_*. */
    uint32_t friction_cone; /**< NKSIM_FRICTION_CONE_*. */
    /** Constraint solver iteration limits; zero keeps the backend default. */
    uint32_t solver_iterations;
    uint32_t line_search_iterations;
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

/** Local pose of one compound child, quaternion in x, y, z, w order. */
typedef struct nksim_shape_pose {
    double position[3];
    double rotation[4];
} nksim_shape_pose;

typedef struct nksim_body_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    nkscene_node_id node;
    uint32_t motion_type;
    double mass;
    nksim_shape shape;
    uint32_t collision_layer;
    uint32_t collision_mask;
    uint32_t has_inertial_properties;
    double center_of_mass[3];
    double inertia_tensor[9]; /**< Row-major symmetric tensor in kg m². */
    uint64_t reserved[2];
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
    double axis_a[3]; /**< Unit vector in body_a's own frame. */
    double lower_limit;
    double upper_limit;
    double max_force; /**< 0: unspecified; negative: explicitly unpowered; positive: force ceiling. */
    /**
     * Joint-frame orientation relative to body_a/body_b (x, y, z, w),
     * appended after the original fields so an old struct prefix remains
     * valid. A struct_size that ends before rotation_b means the
     * caller predates these fields; backends must treat them as identity
     * rather than reading past the caller's actual struct.
     */
    double rotation_a[4];
    double rotation_b[4];
    uint64_t reserved[4];
    /*
     * Optional joint dynamics, read when struct_size includes them. Zero is
     * none. armature is rotor inertia reflected to the joint (kg m^2, or kg for
     * a prismatic joint); damping is effort per unit velocity; friction_loss
     * is dry friction effort.
     */
    double armature;
    double damping;
    double friction_loss;
    /*
     * Optional limit softness, read when struct_size includes it; zeros keep
     * the backend default. The limit acts like a soft contact:
     * limit_time_constant (s) and limit_damping_ratio set its stiffness and
     * damping, limit_impedance its impedance curve (MuJoCo's solimp: dmin,
     * dmax, width, midpoint, power).
     */
    double limit_time_constant;
    double limit_damping_ratio;
    double limit_impedance[5];
} nksim_joint_desc;

/**
 * One term of a follower's value, in joint coordinates: ratio * leader + offset. A follower with
 * several couplings is the sum of their terms (a CoreXY motor follows both axes). Each leader and
 * follower pair is coupled once, and the couplings may not form a cycle.
 */
typedef struct nksim_joint_coupling_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    nksim_joint leader;
    nksim_joint follower;
    double ratio;
    double offset;
    /** Follower-coordinate stiffness; zero preserves the backend default. */
    double stiffness;
} nksim_joint_coupling_desc;

/** Closed-loop fixed, revolute, or prismatic joint between tree bodies. */
typedef struct nksim_closure_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t type; /**< NKSIM_JOINT_FIXED, REVOLUTE, PRISMATIC, SPHERICAL, CYLINDRICAL or PLANAR. */
    nksim_body body_a;
    nksim_body body_b;
    double anchor_a[3];
    double axis_a[3]; /**< Unit axis (hinge, slide, or plane normal) in body_a's local frame. */
} nksim_closure_desc;

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
    double max_force; /**< 0: inherit joint ceiling; negative: unpowered; positive: override ceiling. */
    /* NKSIM_JOINT_TARGET_SERVO terms, read when struct_size includes them. */
    double velocity;
    double stiffness;
    double damping;
    double feedforward;
    /* Drive reference advanced from the command epoch at every physics substep. */
    double end_position;
    double end_velocity;
    double reference_duration;
    double reflected_inertia;
} nksim_joint_target;

/**
 * Contact surface of a shape. Zero fields keep the backend default. Friction
 * coefficients are sliding, torsional and rolling; friction_dimensions is 1
 * (frictionless), 3 (sliding), 4 (with torsional) or 6 (with rolling).
 * contact_time_constant (s) and contact_damping_ratio set how stiff and how
 * damped the soft contact is.
 */
typedef struct nksim_surface {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t friction_dimensions;
    double friction[3];
    double contact_time_constant;
    double contact_damping_ratio;
    uint32_t contact_filter; /**< NKSIM_CONTACT_*; zero is layer collision. */
    uint32_t reserved0;
} nksim_surface;

/** Which contacts a shape takes part in. */
enum {
    /** Collides by its body's collision layer and mask. */
    NKSIM_CONTACT_LAYERS = 0,
    /** Collides only through explicit contact pairs. */
    NKSIM_CONTACT_PAIRS_ONLY = 1,
    /**
     * Collides through explicit pairs and with every environment body (one no
     * joint connects to anything), using this surface for those contacts.
     */
    NKSIM_CONTACT_PAIRS_AND_ENVIRONMENT = 2
};

/**
 * A contact between one part of each of two bodies' shapes (the part index
 * within a compound, or 0), with its own surface, whatever either shape's
 * contact filter says.
 */
typedef struct nksim_contact_pair_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t part_a;
    nksim_body body_a;
    nksim_body body_b;
    uint32_t part_b;
    uint32_t reserved0;
    nksim_surface surface; /**< contact_filter is ignored. */
} nksim_contact_pair_desc;

/** A detected contact. Distance is signed surface separation in metres;
 * active is false when the backend reports it without a force constraint.
 * Body handles are zero for backend-owned world geometry.
 */
typedef struct nksim_contact {
    uint32_t struct_size NK_STRUCT_SIZE;
    nksim_body body_a;
    nksim_body body_b;
    int32_t part_a;
    int32_t part_b;
    double position[3];
    double normal[3];
    double distance;
    uint32_t active NK_BOOL32;
} nksim_contact;

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
/** Stages body and joint additions before the first world step. */
NKSIM_API nksim_result NKSIM_CALL nksim_world_begin_topology_update(nksim_world world);
/** Commits a staged topology update with one backend rebuild. */
NKSIM_API nksim_result NKSIM_CALL nksim_world_end_topology_update(nksim_world world);
/** Configures drive-required integration before the first world step. */
NKSIM_API nksim_result NKSIM_CALL nksim_world_configure_integration(
    nksim_world world, uint32_t substeps, uint32_t integrator);
NKSIM_API nksim_result NKSIM_CALL nksim_world_step(nksim_world world,
                                                   nksim_step_result *out_result NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_world_apply_forces(
    nksim_world world, const nksim_body_force *forces NK_IN_ARRAY(count), uint32_t count);
NKSIM_API nksim_result NKSIM_CALL nksim_world_set_joint_targets(
    nksim_world world, const nksim_joint_target *targets NK_IN_ARRAY(count), uint32_t count);
NKSIM_API nksim_result NKSIM_CALL nksim_world_snapshot(
    nksim_world world, nksim_snapshot *out_snapshot NK_OUT NK_OWNED);
/** Copies at most capacity contacts and reports the full available count.
 * Pass capacity 0 and out NULL to size a buffer. Read after stepping.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_world_get_contacts(
    nksim_world world, nksim_contact *out,
    uint32_t capacity, uint32_t *out_count NK_OUT);

NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_box(
    nksim_world world, const double half_extents[3],
    nksim_shape *out_shape NK_OUT);
/** Copies 12..192 XYZ coordinates (4..64 vertices); MuJoCo compiles their convex polytope. */
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_convex(
    nksim_world world, const double *vertices NK_IN_ARRAY(coordinate_count),
    uint32_t coordinate_count, nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_sphere(
    nksim_world world, double radius, nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_capsule(
    nksim_world world, double radius, double height, nksim_shape *out_shape NK_OUT);
/** A solid cylinder along local Z; height is the full length between its flat ends. */
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_cylinder(
    nksim_world world, double radius, double height, nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_plane(
    nksim_world world, const double normal[3], double offset,
    nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create(
    nksim_world world, const nksim_shape_desc *desc, nksim_shape *out_shape NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_shape_create_compound(
    nksim_world world, const nksim_shape *children, const nksim_shape_pose *poses,
    uint32_t count, nksim_shape *out_shape NK_OUT);
/** Adds an explicit contact pair between two existing bodies' shape parts. */
NKSIM_API nksim_result NKSIM_CALL nksim_contact_pair_create(
    nksim_world world, const nksim_contact_pair_desc *desc);
/** Sets a shape's contact surface; like its margin, only before a body uses it. */
NKSIM_API nksim_result NKSIM_CALL nksim_shape_set_surface(
    nksim_world world, nksim_shape shape, const nksim_surface *surface);
/** margin is the distance at which contact force begins. gap is the extra
 * band beyond margin where contacts are detected without force. */
NKSIM_API nksim_result NKSIM_CALL nksim_shape_set_contact(
    nksim_world world, nksim_shape shape, double margin, double gap);
NKSIM_API void NKSIM_CALL nksim_shape_destroy(nksim_world world, nksim_shape shape);

NKSIM_API nksim_result NKSIM_CALL nksim_body_create(
    nksim_world world, const nksim_body_desc *desc, nksim_body *out_body NK_OUT);
NKSIM_API void NKSIM_CALL nksim_body_destroy(nksim_world world, nksim_body body);
NKSIM_API nksim_result NKSIM_CALL nksim_body_get_state(
    nksim_world world, nksim_body body, nksim_body_state *out_state NK_INOUT);
/**
 * Replaces a body's pose and velocities on the world owner thread. For a
 * kinematic body this is a discontinuity: the next tick keeps the supplied
 * velocities instead of inferring them from the scene-node pose change.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_body_set_state(
    nksim_world world, nksim_body body, const nksim_body_state *state);
/**
 * Drives a kinematic body continuously on the world owner thread: the next
 * tick moves the body from its current pose to the state's pose and reports
 * the state's linear and angular velocity as its twist, both exactly as given
 * in double precision. Unlike nksim_body_set_state() this is not a
 * discontinuity. Keep the body's scene node at the same pose; while a drive is
 * pending it takes precedence over the node. After the driven tick the body
 * follows its node again: a node left where the drive put it holds the body
 * at the driven pose with zero twist, and a node that moves is differenced
 * against its previous pose. Drive every tick for continuous exact motion.
 * Returns NKSIM_ERROR_INVALID_STATE for a body that is not kinematic.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_body_drive(
    nksim_world world, nksim_body body, const nksim_body_state *state);
NKSIM_API nksim_result NKSIM_CALL nksim_body_reset(nksim_world world, nksim_body body);
/** Restores body state and the fixed-step clock to their initial values. */
NKSIM_API nksim_result NKSIM_CALL nksim_world_reset(nksim_world world);

NKSIM_API nksim_result NKSIM_CALL nksim_joint_create(
    nksim_world world, const nksim_joint_desc *desc, nksim_joint *out_joint NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_joint_couple(
    nksim_world world, const nksim_joint_coupling_desc *desc);
NKSIM_API nksim_result NKSIM_CALL nksim_closure_create(
    nksim_world world, const nksim_closure_desc *desc);
NKSIM_API void NKSIM_CALL nksim_joint_destroy(nksim_world world, nksim_joint joint);
NKSIM_API nksim_result NKSIM_CALL nksim_joint_get_state(
    nksim_world world, nksim_joint joint, nksim_joint_state *out_state NK_INOUT);
/**
 * Places a one-DOF joint at a position and velocity, moving the bodies it
 * carries, as a pose to start from; it is not a target. Coupled followers are
 * re-derived by the backend at its next step.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_joint_set_state(
    nksim_world world, nksim_joint joint, double position, double velocity);

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
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_contact_count(
    nksim_snapshot snapshot, uint64_t *out_count NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_snapshot_get_contact(
    nksim_snapshot snapshot, uint64_t index, nksim_contact *out_contact NK_INOUT);

#ifdef __cplusplus
}
#endif

#endif /* NATIVEKIT_SIM_H */
