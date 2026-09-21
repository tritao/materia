#pragma once

#include "nativekit_sim.h"

#include <array>
#include <cstdint>

namespace nksim {

using World = nksim_world;
using Body = nksim_body;
using Joint = nksim_joint;
using Shape = nksim_shape;
using Snapshot = nksim_snapshot;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline std::array<double, 3> array(Vec3 value) noexcept {
    return {value.x, value.y, value.z};
}

} // namespace nksim
