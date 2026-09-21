#include "robotkit_runtime.h"
#include "robotkit_runtime.hpp"
#include "runtime_registry.hpp"

#include <memory>
#include <mutex>
#include <unordered_map>

namespace {

std::mutex registry_mutex;
std::unordered_map<rk_runtime, std::shared_ptr<robotkit::Runtime>> runtimes;
std::unordered_map<rk_runtime, std::weak_ptr<robotkit::internal::RuntimeCoordinator>> coordinators;
std::unordered_map<rk_runtime, std::shared_ptr<void>> runtime_owners;
rk_runtime next_runtime = 1;

} // namespace

namespace robotkit::internal {

std::shared_ptr<Runtime> resolve_runtime(rk_runtime handle) {
    std::lock_guard lock(registry_mutex);
    const auto found = runtimes.find(handle);
    return found == runtimes.end() ? nullptr : found->second;
}

std::shared_ptr<RuntimeCoordinator> resolve_runtime_coordinator(rk_runtime handle) {
    std::lock_guard lock(registry_mutex);
    const auto found = coordinators.find(handle);
    return found == coordinators.end() ? nullptr : found->second.lock();
}

rk_runtime register_runtime(std::shared_ptr<Runtime> runtime,
                            std::shared_ptr<RuntimeCoordinator> coordinator) {
    std::lock_guard lock(registry_mutex);
    while (next_runtime == RK_INVALID_RUNTIME || runtimes.count(next_runtime) != 0)
        ++next_runtime;
    const auto handle = next_runtime++;
    runtimes.emplace(handle, std::move(runtime));
    if (coordinator)
        coordinators.emplace(handle, coordinator);
    return handle;
}

void destroy_runtime(rk_runtime handle) {
    std::shared_ptr<Runtime> released;
    std::shared_ptr<void> owner;
    {
        std::lock_guard lock(registry_mutex);
        const auto found = runtimes.find(handle);
        if (found == runtimes.end())
            return;
        released = std::move(found->second);
        runtimes.erase(found);
        coordinators.erase(handle);
        const auto owner_found = runtime_owners.find(handle);
        if (owner_found != runtime_owners.end()) {
            owner = std::move(owner_found->second);
            runtime_owners.erase(owner_found);
        }
    }
}

void attach_runtime_owner(rk_runtime handle, std::shared_ptr<void> owner) {
    std::lock_guard lock(registry_mutex);
    if (runtimes.count(handle) != 0)
        runtime_owners[handle] = std::move(owner);
}

} // namespace robotkit::internal

extern "C" {

rk_result RK_CALL rk_runtime_create(const rk_runtime_layout *layout,
                                    rk_runtime *out_runtime) {
    if (!out_runtime || rk_runtime_layout_validate(layout) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    *out_runtime = RK_INVALID_RUNTIME;
    try {
        auto endpoint = std::make_unique<robotkit::InMemoryEndpoint>(layout->joint_count);
        auto runtime = std::make_shared<robotkit::Runtime>(*layout, std::move(endpoint));
        const auto handle = robotkit::internal::register_runtime(std::move(runtime));
        *out_runtime = handle;
        return RK_OK;
    } catch (...) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

void RK_CALL rk_runtime_destroy(rk_runtime runtime) {
    robotkit::internal::destroy_runtime(runtime);
}

rk_result RK_CALL rk_runtime_start(rk_runtime runtime) {
    if (const auto coordinator = robotkit::internal::resolve_runtime_coordinator(runtime))
        return coordinator->start();
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->start() : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_runtime_stop(rk_runtime runtime) {
    if (const auto coordinator = robotkit::internal::resolve_runtime_coordinator(runtime))
        return coordinator->stop();
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->stop() : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_runtime_submit(rk_runtime runtime, const rk_robot_command *command) {
    if (!command)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->submit(*command) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_runtime_step(rk_runtime runtime, uint64_t timestamp_ns) {
    if (const auto coordinator = robotkit::internal::resolve_runtime_coordinator(runtime))
        return coordinator->step(timestamp_ns);
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->step_once(timestamp_ns) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_runtime_snapshot(rk_runtime runtime, rk_robot_state *out_state) {
    if (!out_state || out_state->struct_size < sizeof(*out_state))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->snapshot(*out_state) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_runtime_snapshot_full(rk_runtime runtime,
                                           rk_robot_snapshot *out_snapshot) {
    if (!out_snapshot || out_snapshot->struct_size < sizeof(*out_snapshot))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->snapshot_full(*out_snapshot) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_runtime_capabilities(rk_runtime runtime,
                                           rk_robot_capabilities *out_capabilities) {
    if (!out_capabilities || out_capabilities->struct_size < sizeof(*out_capabilities))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    if (!value)
        return RK_ERROR_INVALID_HANDLE;
    out_capabilities->joint_count = 0;
    out_capabilities->supports_position_targets = 1;
    out_capabilities->supports_velocity_targets = 0;
    out_capabilities->supports_effort_targets = 0;
    out_capabilities->supports_prediction = 0;
    for (auto &reserved : out_capabilities->reserved)
        reserved = 0;
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    const auto result = value->snapshot(state);
    if (result != RK_OK)
        return result;
    out_capabilities->joint_count = state.joint_count;
    return RK_OK;
}

} // extern "C"
