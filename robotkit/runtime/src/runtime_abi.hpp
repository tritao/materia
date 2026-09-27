#ifndef ROBOTKIT_RUNTIME_ABI_HPP
#define ROBOTKIT_RUNTIME_ABI_HPP

#include "robotkit_runtime.h"

#include <algorithm>
#include <cstring>

namespace robotkit::internal {

/** Copy only fields supplied by the caller; absent calibration revision stays zero. */
inline rk_robot_runtime_blueprint copy_blueprint(const rk_robot_runtime_blueprint *source) {
    rk_robot_runtime_blueprint copy{};
    std::memcpy(&copy, source, std::min<std::size_t>(source->struct_size, sizeof(copy)));
    copy.struct_size = sizeof(copy);
    return copy;
}

} // namespace robotkit::internal

#endif
