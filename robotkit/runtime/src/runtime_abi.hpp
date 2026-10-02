#ifndef ROBOTKIT_RUNTIME_ABI_HPP
#define ROBOTKIT_RUNTIME_ABI_HPP

#include "robotkit_runtime.h"

#include <algorithm>
#include <cstring>
#include <memory>

namespace robotkit::internal {

/**
 * Copy only fields supplied by the caller; absent calibration revision stays zero. The copy is on the heap: a
 * blueprint is about 260 KiB, more than some threads' whole stack.
 */
inline std::unique_ptr<rk_robot_runtime_blueprint> copy_blueprint(const rk_robot_runtime_blueprint *source) {
    auto copy = std::make_unique<rk_robot_runtime_blueprint>();
    std::memcpy(copy.get(), source, std::min<std::size_t>(source->struct_size, sizeof(*copy)));
    copy->struct_size = sizeof(*copy);
    return copy;
}

} // namespace robotkit::internal

#endif
