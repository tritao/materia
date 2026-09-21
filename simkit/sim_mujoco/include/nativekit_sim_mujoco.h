#ifndef NATIVEKIT_SIM_MUJOCO_H
#define NATIVEKIT_SIM_MUJOCO_H

#include "nativekit_sim.h"

#if defined(_WIN32)
#if defined(NKSIMMUJOCO_STATIC)
#define NKSIMMUJOCO_API
#elif defined(NKSIMMUJOCO_BUILDING_LIBRARY)
#define NKSIMMUJOCO_API __declspec(dllexport)
#else
#define NKSIMMUJOCO_API __declspec(dllimport)
#endif
#define NKSIMMUJOCO_CALL __cdecl
#else
#define NKSIMMUJOCO_API __attribute__((visibility("default")))
#define NKSIMMUJOCO_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Creates a Sim world whose internal backend is MuJoCo. */
NKSIMMUJOCO_API nksim_result NKSIMMUJOCO_CALL nksim_mujoco_world_create(
    const nksim_world_desc *desc, nksim_world *out_world NK_OUT NK_OWNED);

#ifdef __cplusplus
}
#endif

#endif /* NATIVEKIT_SIM_MUJOCO_H */
