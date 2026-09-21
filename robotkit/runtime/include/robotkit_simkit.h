#ifndef ROBOTKIT_SIMKIT_H
#define ROBOTKIT_SIMKIT_H

#include "robotkit_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Creates a RobotKit runtime backed by SimKit's owner-thread host. */
RK_API rk_result RK_CALL rk_runtime_create_sim(
    const rk_runtime_blueprint *blueprint,
    rk_runtime *out_runtime RK_OUT RK_OWNED);

#ifdef __cplusplus
}
#endif

#endif
