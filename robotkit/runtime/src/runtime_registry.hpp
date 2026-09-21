#ifndef ROBOTKIT_RUNTIME_REGISTRY_HPP
#define ROBOTKIT_RUNTIME_REGISTRY_HPP

#include "robotkit_runtime.hpp"

#include <memory>

namespace robotkit::internal {

std::shared_ptr<RobotRuntime> resolve_runtime(rk_robot_runtime handle);
rk_robot_runtime register_runtime(std::shared_ptr<RobotRuntime> runtime);
void destroy_runtime(rk_robot_runtime handle);

} // namespace robotkit::internal

#endif
