#ifndef ROBOTKIT_RUNTIME_REGISTRY_HPP
#define ROBOTKIT_RUNTIME_REGISTRY_HPP

#include "robotkit_runtime.hpp"

#include <memory>

namespace robotkit::internal {

class RuntimeCoordinator {
public:
    virtual ~RuntimeCoordinator() = default;
    virtual rk_result start() = 0;
    virtual rk_result stop() = 0;
};

std::shared_ptr<Runtime> resolve_runtime(rk_runtime handle);
std::shared_ptr<RuntimeCoordinator> resolve_runtime_coordinator(rk_runtime handle);
rk_runtime register_runtime(
    std::shared_ptr<Runtime> runtime,
    std::shared_ptr<RuntimeCoordinator> coordinator = {});
void destroy_runtime(rk_runtime handle);

} // namespace robotkit::internal

#endif
