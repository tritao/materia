#ifndef MOTIONKIT_H
#define MOTIONKIT_H

#include <stdint.h>

#if defined(__clang__)
#define MK_OUT __attribute__((annotate("hxi:out")))
#define MK_OWNED __attribute__((annotate("hxi:owned")))
#define MK_HANDLE __attribute__((annotate("hxi:handle")))
#define MK_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#define MK_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#define MK_STRUCT_SIZE __attribute__((annotate("hxi:struct_size")))
#else
#define MK_OUT
#define MK_OWNED
#define MK_HANDLE
#define MK_HANDLE_DESTROY(symbol)
#define MK_IN_ARRAY(count)
#define MK_STRUCT_SIZE
#endif

#if defined(_WIN32)
#if defined(MK_STATIC)
#define MK_API
#elif defined(MK_BUILDING_LIBRARY)
#define MK_API __declspec(dllexport)
#else
#define MK_API __declspec(dllimport)
#endif
#define MK_CALL __cdecl
#else
#define MK_API __attribute__((visibility("default")))
#define MK_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum { MK_API_VERSION = 1, MK_MAX_JOINTS = 64, MK_MAX_DEGREE = 5 };
typedef int32_t mk_result;
enum {
    MK_OK = 0,
    MK_ERROR_INVALID_ARGUMENT = -1,
    MK_ERROR_INVALID_HANDLE = -2,
    MK_ERROR_OUT_OF_MEMORY = -3
};

/** Opaque registry identity; only MotionKit may interpret id. */
typedef struct mk_trajectory_handle { uint32_t id; } mk_trajectory_handle
    MK_HANDLE MK_HANDLE_DESTROY(mk_trajectory_destroy);

typedef struct mk_joint_coefficients {
    double value[MK_MAX_DEGREE + 1];
} mk_joint_coefficients;

/** Coefficients are joint-major, with seconds from t0 as the polynomial variable. */
typedef struct mk_segment {
    uint32_t struct_size MK_STRUCT_SIZE;
    int64_t t0_ns;
    int64_t duration_ns;
    uint32_t degree;
    uint32_t joint_count;
    mk_joint_coefficients coefficients[MK_MAX_JOINTS];
} mk_segment;

typedef struct mk_sample {
    uint32_t struct_size MK_STRUCT_SIZE;
    int64_t time_ns;
    uint32_t joint_count;
    double position[MK_MAX_JOINTS];
} mk_sample;

typedef struct mk_trajectory_state {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t joint_count;
    double position[MK_MAX_JOINTS];
    double velocity[MK_MAX_JOINTS];
    double acceleration[MK_MAX_JOINTS];
    double jerk[MK_MAX_JOINTS];
} mk_trajectory_state;

/** Maximum absolute jump for each derivative across one segment boundary. */
typedef struct mk_continuity {
    uint32_t struct_size MK_STRUCT_SIZE;
    uint32_t boundary_index;
    int64_t time_ns;
    double c0_jump;
    double c1_jump;
    double c2_jump;
    uint32_t c0_joint;
    uint32_t c1_joint;
    uint32_t c2_joint;
} mk_continuity;

MK_API mk_result MK_CALL mk_trajectory_create(uint32_t joint_count,
    mk_trajectory_handle *out_trajectory MK_OUT MK_OWNED);
MK_API void MK_CALL mk_trajectory_destroy(mk_trajectory_handle trajectory);
MK_API mk_result MK_CALL mk_trajectory_append_segment(mk_trajectory_handle trajectory,
    const mk_segment *segment);
MK_API mk_result MK_CALL mk_trajectory_evaluate(mk_trajectory_handle trajectory,
    int64_t time_ns, mk_trajectory_state *out_state);
MK_API mk_result MK_CALL mk_trajectory_duration_ns(mk_trajectory_handle trajectory,
    int64_t *out_duration_ns MK_OUT);
MK_API mk_result MK_CALL mk_trajectory_joint_count(mk_trajectory_handle trajectory,
    uint32_t *out_joint_count MK_OUT);
MK_API mk_result MK_CALL mk_trajectory_segment_count(mk_trajectory_handle trajectory,
    uint32_t *out_segment_count MK_OUT);
MK_API mk_result MK_CALL mk_trajectory_boundary_continuity(mk_trajectory_handle trajectory,
    uint32_t boundary_index, mk_continuity *out_continuity);
/** Requires at least two strictly increasing samples. Positions are interpolated linearly. */
MK_API mk_result MK_CALL mk_trajectory_from_samples(uint32_t joint_count,
    const mk_sample *samples MK_IN_ARRAY(sample_count), uint32_t sample_count,
    mk_trajectory_handle *out_trajectory MK_OUT MK_OWNED);

#ifdef __cplusplus
}
#endif

#endif
