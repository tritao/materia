#ifndef MOTIONKIT_HPP
#define MOTIONKIT_HPP
#include "motionkit.h"
#include "trajectory_core.hpp"
namespace motionkit {
mk_result generate(const mk_state_to_state_request &request, Trajectory &trajectory,
                   int32_t &ruckig_result);
}
#endif
