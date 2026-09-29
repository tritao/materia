#ifndef NATIVEKIT_SIM_SESSION_H
#define NATIVEKIT_SIM_SESSION_H

#include "nativekit_sim.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * One shared simulated space: a fixed-step clock and owner loop around a
 * borrowed world and its scene, which every participant (robots, people,
 * environment props) attaches to instead of owning physics itself.
 *
 * The session borrows its world and scene; destroy the session before them.
 * While the session is hosted (from its first tick or start until stop or
 * reset), the world belongs to the session's physics owner thread, and the
 * session's calls may come from any thread; it serializes them against ticks.
 * While it is not hosted, calls that touch the world must come from the
 * world's owner thread, as direct world calls must.
 */
typedef uint32_t nksim_session NK_HANDLE NK_HANDLE_DESTROY(nksim_session_destroy);
/** An environment body the session owns: static, kinematic, or dynamic. */
typedef uint32_t nksim_object NK_HANDLE;
/** A group of kinematic bodies moved together along a timed trajectory. */
typedef uint32_t nksim_actor NK_HANDLE;
/** Immutable consistent view of the session after one tick. */
typedef uint32_t nksim_frame NK_HANDLE NK_HANDLE_DESTROY(nksim_frame_destroy);

#define NKSIM_INVALID_SESSION ((nksim_session)0)
#define NKSIM_INVALID_OBJECT ((nksim_object)0)
#define NKSIM_INVALID_ACTOR ((nksim_actor)0)
#define NKSIM_INVALID_FRAME ((nksim_frame)0)

typedef struct nksim_session_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    /** The scene the world was created with. Borrowed. */
    nkscene_scene scene;
    /** Borrowed; must be created on the thread that creates the session. */
    nksim_world world;
    uint64_t reserved[4];
} nksim_session_desc;

typedef struct nksim_session_status {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t running NK_BOOL32; /**< A realtime owner loop is ticking. */
    uint32_t hosted NK_BOOL32; /**< The physics owner thread holds the world. */
    /** Set by the first tick or start; topology participants add before it. */
    uint32_t sealed NK_BOOL32;
    uint64_t step_index;
    double simulation_time;
    double fixed_timestep;
    double gravity[3];
    uint64_t reserved[2];
} nksim_session_status;

typedef struct nksim_pose {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t reserved0;
    double position[3];
    double rotation[4]; /**< Quaternion in x, y, z, w order. */
} nksim_pose;

typedef struct nksim_object_desc {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t motion_type; /**< NKSIM_MOTION_STATIC, _KINEMATIC, or _DYNAMIC. */
    /** Box, sphere, or capsule, as nksim_shape_create() takes it. */
    nksim_shape_desc shape;
    nksim_pose pose;
    double mass; /**< Positive for a dynamic object; ignored otherwise. */
    /** Both zero selects layer 1 and mask 1, the ordinary environment category. */
    uint32_t collision_layer;
    uint32_t collision_mask;
    uint64_t reserved[2];
} nksim_object_desc;

/** One kinematic body of an actor. */
typedef struct nksim_actor_part {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t reserved0;
    /** Box, sphere, or capsule, as nksim_shape_create() takes it. */
    nksim_shape_desc shape;
    /** Pose at creation, and the pose reset restores. */
    nksim_pose pose;
} nksim_actor_part;

NKSIM_API nksim_result NKSIM_CALL nksim_session_create(
    const nksim_session_desc *desc, nksim_session *out_session NK_OUT NK_OWNED);
/** Stops the session, destroys what it created, and leaves the world and scene. */
NKSIM_API void NKSIM_CALL nksim_session_destroy(nksim_session session);
NKSIM_API nksim_result NKSIM_CALL nksim_session_get_status(
    nksim_session session, nksim_session_status *out_status NK_INOUT);

/**
 * Advances exactly one fixed tick on the caller's thread: every participant
 * prepares, then submits, physics advances, and every participant publishes.
 * owner_time_ns is the timestamp participants that follow the owner's clock
 * receive. Returns NKSIM_ERROR_INVALID_STATE while running realtime.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_step(
    nksim_session session, uint64_t owner_time_ns, nksim_clock *out_clock NK_INOUT);
/** Starts a realtime owner loop that ticks at the fixed timestep. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_start(nksim_session session);
/** Stops the realtime loop and returns the world to the caller's thread. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_stop(nksim_session session);
/**
 * Restores the world, the clock, every object and actor, and every
 * participant to their initial state. The session must be stopped.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_reset(nksim_session session);

/** Adds one environment body while stopped. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_create_object(
    nksim_session session, const nksim_object_desc *desc, nksim_object *out_object NK_OUT);
/** Removes one environment body while stopped. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_destroy_object(
    nksim_session session, nksim_object object);
/** Moves an object at rest while stopped; the pose also becomes its reset pose. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_teleport_object(
    nksim_session session, nksim_object object, const nksim_pose *pose);
/**
 * Moves a kinematic object for the next tick while stopped or running; its
 * velocity over the tick is the motion from where the previous tick left it.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_drive_object(
    nksim_session session, nksim_object object, const nksim_pose *pose);
/**
 * Attaches a DYNAMIC object to any session body while stopped or running.
 * From the next tick it follows carrier * offset without being displaced by
 * contacts. Holding an already-held object replaces its carrier and offset.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_hold_object(
    nksim_session session, nksim_object object, nksim_body carrier,
    const nksim_pose *offset);
/**
 * Releases a held object as DYNAMIC from the next tick, retaining the
 * carrier point's velocity. Releasing a free object is INVALID_STATE.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_release_object(
    nksim_session session, nksim_object object);
/** Returns zero in out_carrier when the object is free. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_get_object_carrier(
    nksim_session session, nksim_object object, nksim_body *out_carrier NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_session_get_object_body(
    nksim_session session, nksim_object object, nksim_body *out_body NK_OUT);
/**
 * The session object bound to a body, for a body a session participant
 * observes in a contact or elsewhere. Returns NKSIM_ERROR_INVALID_HANDLE if
 * the body is not a current session object's body (for instance, a robot
 * link or an actor part).
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_find_object(
    nksim_session session, nksim_body body, nksim_object *out_object NK_OUT);

/**
 * Adds a group of 1..256 kinematic bodies while stopped. Actor bodies push
 * dynamic bodies they move into and are never pushed back.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_create_actor(
    nksim_session session, const nksim_actor_part *parts NK_IN_ARRAY(part_count),
    uint32_t part_count, nksim_actor *out_actor NK_OUT);
NKSIM_API nksim_result NKSIM_CALL nksim_session_destroy_actor(
    nksim_session session, nksim_actor actor);
/**
 * Adds a keyframe to an actor's trajectory: the pose of every part at
 * simulation time `time`, while stopped or running. Each tick moves every
 * part to its pose interpolated at the tick's end time (linear position,
 * shortest-arc rotation), holding the last keyframe after it and the first
 * before it, with the velocity of that motion. A keyframe at or before an
 * existing one's time replaces it and every later keyframe, so a writer can
 * replan; keyframes older than the current tick are discarded. Writers that
 * keep a lead of a few ticks see smooth motion in realtime; a single
 * keyframe at the current time moves the actor on the next tick.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_push_actor_keyframe(
    nksim_session session, nksim_actor actor, double time,
    const nksim_pose *poses NK_IN_ARRAY(pose_count), uint32_t pose_count);
NKSIM_API nksim_result NKSIM_CALL nksim_session_get_actor_body(
    nksim_session session, nksim_actor actor, uint32_t part, nksim_body *out_body NK_OUT);

typedef struct nksim_ray {
    uint32_t struct_size NK_STRUCT_SIZE;
    uint32_t reserved0;
    double origin[3];
    double direction[3]; /**< Unit vector. */
    double max_distance;
} nksim_ray;

/**
 * Casts a ray against the session's objects and actors in the latest state
 * and returns the distance to the nearest hit, or max_distance when nothing
 * is nearer.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_raycast(
    nksim_session session, const nksim_ray *ray, double *out_distance NK_OUT);

/** Captures the clock and every body's state under one session lock. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_capture(
    nksim_session session, nksim_frame *out_frame NK_OUT NK_OWNED);
NKSIM_API void NKSIM_CALL nksim_frame_destroy(nksim_frame frame);
NKSIM_API nksim_result NKSIM_CALL nksim_frame_get_clock(
    nksim_frame frame, nksim_clock *out_clock NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_frame_get_body_state(
    nksim_frame frame, nksim_body body, nksim_body_state *out_state NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_frame_get_object_pose(
    nksim_frame frame, nksim_object object, nksim_pose *out_pose NK_INOUT);
NKSIM_API nksim_result NKSIM_CALL nksim_frame_get_actor_pose(
    nksim_frame frame, nksim_actor actor, uint32_t part, nksim_pose *out_pose NK_INOUT);

#ifndef NKSIM_HAXEON_IMPORT

/**
 * Participant interface: native systems that take part in every tick, such as
 * a robot runtime. Callbacks run under the session lock on the ticking
 * thread and may call any session function from it.
 */
typedef uint32_t nksim_participant;
#define NKSIM_INVALID_PARTICIPANT ((nksim_participant)0)

typedef struct nksim_tick {
    uint32_t struct_size;
    /** Nonzero for a tick the realtime owner loop scheduled. */
    uint32_t realtime;
    /** The step index and simulation time this tick completes. */
    uint64_t step_index;
    double simulation_time;
    double fixed_timestep;
    /** nksim_session_step()'s timestamp, or monotonic time in realtime. */
    uint64_t owner_time_ns;
} nksim_tick;

typedef struct nksim_participant_desc {
    uint32_t struct_size;
    uint32_t reserved0;
    void *user;
    /**
     * Stages this participant's commands for the tick. If any participant
     * fails, every participant's discard runs and the tick does not advance.
     */
    nksim_result (*prepare)(void *user, const nksim_tick *tick);
    void (*discard)(void *user);
    /** Submits the tick's physics inputs: joint targets, body drives. */
    nksim_result (*submit)(void *user, const nksim_tick *tick);
    /** Reads the completed tick from nksim_session_latest_snapshot(). */
    nksim_result (*publish)(void *user, const nksim_tick *tick);
    /** Restores participant state during nksim_session_reset(). */
    nksim_result (*reset)(void *user);
} nksim_participant_desc;

NKSIM_API nksim_result NKSIM_CALL nksim_session_add_participant(
    nksim_session session, const nksim_participant_desc *desc,
    nksim_participant *out_participant);
/** After this returns, the participant's callbacks are never called again. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_remove_participant(
    nksim_session session, nksim_participant participant);

/** Recursive session lock; participant code takes it around multi-call edits. */
NKSIM_API void NKSIM_CALL nksim_session_lock(nksim_session session);
NKSIM_API void NKSIM_CALL nksim_session_unlock(nksim_session session);
/**
 * The world and scene for topology and direct state edits while the session
 * is not hosted, under the session lock. NKSIM_ERROR_INVALID_STATE while
 * hosted.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_get_world(
    nksim_session session, nksim_world *out_world);
NKSIM_API nksim_result NKSIM_CALL nksim_session_get_scene(
    nksim_session session, nkscene_scene *out_scene);
/**
 * The latest published snapshot, borrowed: valid under the session lock
 * until the next tick, stop, or reset. Zero before the session is hosted.
 */
NKSIM_API nksim_snapshot NKSIM_CALL nksim_session_latest_snapshot(nksim_session session);
/** Reads a body from the latest snapshot while hosted, else from the world. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_get_body_state(
    nksim_session session, nksim_body body, nksim_body_state *out_state);
/** Joint targets, kinematic drives, and state writes for the next tick. */
NKSIM_API nksim_result NKSIM_CALL nksim_session_submit_joint_targets(
    nksim_session session, const nksim_joint_target *targets, uint32_t count);
/**
 * Forces and torques, world frame, applied at bodies' centres of mass for the
 * next tick only, on top of any others submitted for it. Repeat every tick to
 * push for longer.
 */
NKSIM_API nksim_result NKSIM_CALL nksim_session_submit_forces(
    nksim_session session, const nksim_body_force *forces NK_IN_ARRAY(count), uint32_t count);
NKSIM_API nksim_result NKSIM_CALL nksim_session_drive_bodies(
    nksim_session session, const nksim_body_state *states, uint32_t count);
NKSIM_API nksim_result NKSIM_CALL nksim_session_set_body_states(
    nksim_session session, const nksim_body_state *states, uint32_t count);
/** The frame's snapshot, borrowed for the frame's lifetime. */
NKSIM_API nksim_snapshot NKSIM_CALL nksim_frame_snapshot(nksim_frame frame);

#endif /* NKSIM_HAXEON_IMPORT */

#ifdef __cplusplus
}
#endif

#endif /* NATIVEKIT_SIM_SESSION_H */
