#ifndef ROBOTKIT_RUNTIME_REGISTRY_HPP
#define ROBOTKIT_RUNTIME_REGISTRY_HPP

#include "robotkit_runtime.hpp"

#include <memory>

namespace robotkit::internal {

std::shared_ptr<Runtime> resolve_runtime(rk_runtime handle);
rk_runtime register_runtime(std::shared_ptr<Runtime> runtime);
void destroy_runtime(rk_runtime handle);

} // namespace robotkit::internal

#endif
