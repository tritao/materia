#ifndef ROBOTKIT_RUNTIME_ABI_HPP
#define ROBOTKIT_RUNTIME_ABI_HPP

#include "robotkit_runtime.h"

#include <cstring>
#include <memory>

namespace robotkit::internal {

/**
 * Copy a validated current-layout blueprint onto the heap: its size exceeds some
 * threads' whole stack. Every entry point validates the complete layout first.
 */
inline std::unique_ptr<rk_robot_runtime_blueprint> copy_blueprint(const rk_robot_runtime_blueprint *source) {
    auto copy = std::make_unique<rk_robot_runtime_blueprint>();
    std::memcpy(copy.get(), source, sizeof(*copy));
    return copy;
}

} // namespace robotkit::internal

#endif
