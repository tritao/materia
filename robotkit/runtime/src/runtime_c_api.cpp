#include "robotkit_runtime.h"
#include "robotkit_runtime.hpp"
#include "robotkit_device_serial_endpoint.hpp"
#include "runtime_registry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace {

std::mutex registry_mutex;
std::unordered_map<rk_robot_runtime, std::shared_ptr<robotkit::RobotRuntime>> runtimes;
rk_robot_runtime next_runtime = 1;

int hex_nibble(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

bool parse_fingerprint(const char *hex, std::array<std::uint8_t, 16> &result) {
    if (!hex) return false;
    for (std::size_t index = 0; index < result.size(); ++index) {
        if (!hex[2 * index] || !hex[2 * index + 1]) return false;
        const int high = hex_nibble(hex[2 * index]);
        const int low = hex_nibble(hex[2 * index + 1]);
        if (high < 0 || low < 0) return false;
        result[index] = static_cast<std::uint8_t>((high << 4) | low);
    }
    return hex[32] == '\0' &&
        !std::all_of(result.begin(), result.end(), [](auto byte) { return byte == 0; });
}

} // namespace

namespace robotkit::internal {

std::shared_ptr<RobotRuntime> resolve_runtime(rk_robot_runtime handle) {
    std::lock_guard lock(registry_mutex);
    const auto found = runtimes.find(handle);
    return found == runtimes.end() ? nullptr : found->second;
}

rk_robot_runtime register_runtime(std::shared_ptr<RobotRuntime> runtime) {
    std::lock_guard lock(registry_mutex);
    while (next_runtime == RK_INVALID_ROBOT_RUNTIME || runtimes.count(next_runtime) != 0)
        ++next_runtime;
    const auto handle = next_runtime++;
    runtimes.emplace(handle, std::move(runtime));
    return handle;
}

void destroy_runtime(rk_robot_runtime handle) {
    std::shared_ptr<RobotRuntime> released;
    {
        std::lock_guard lock(registry_mutex);
        const auto found = runtimes.find(handle);
        if (found == runtimes.end())
            return;
        released = std::move(found->second);
        runtimes.erase(found);
    }
}

} // namespace robotkit::internal

extern "C" {

rk_result RK_CALL rk_robot_runtime_create(const rk_robot_runtime_blueprint *blueprint,
                                    rk_robot_runtime *out_runtime) {
    if (!out_runtime || rk_robot_runtime_blueprint_validate(blueprint) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    *out_runtime = RK_INVALID_ROBOT_RUNTIME;
    try {
        std::shared_ptr<robotkit::RobotEndpoint> endpoint =
            std::make_shared<robotkit::InMemoryRobot>(blueprint->joint_count);
        auto runtime = std::make_shared<robotkit::RobotRuntime>(*blueprint, endpoint);
        const auto handle = robotkit::internal::register_runtime(std::move(runtime));
        *out_runtime = handle;
        return RK_OK;
    } catch (...) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

rk_result RK_CALL rk_robot_runtime_create_serial(const rk_robot_runtime_blueprint *blueprint,
                                                    const char *device_path, uint32_t baud,
                                                    const char *fingerprint_hex,
                                                    double max_target_error,
                                                    rk_robot_runtime *out_runtime) {
    std::array<std::uint8_t, 16> fingerprint{};
    if (!out_runtime || !blueprint || rk_robot_runtime_blueprint_validate(blueprint) != RK_OK ||
        !device_path || !*device_path || blueprint->joint_count > RK_MAX_SERIAL_JOINTS ||
        !std::isfinite(max_target_error) || max_target_error < 0.0 ||
        !parse_fingerprint(fingerprint_hex, fingerprint))
        return RK_ERROR_INVALID_ARGUMENT;
    *out_runtime = RK_INVALID_ROBOT_RUNTIME;
    try {
        std::uint8_t session_status = 0;
        auto endpoint = robotkit::DeviceSerialEndpoint::open(device_path, baud, fingerprint,
            static_cast<std::uint8_t>(blueprint->joint_count), max_target_error, &session_status);
        if (!endpoint)
            return session_status == 2 ? RK_ERROR_MODEL_MISMATCH : RK_ERROR_BACKEND;
        auto runtime = std::make_shared<robotkit::RobotRuntime>(*blueprint,
            std::static_pointer_cast<robotkit::RobotEndpoint>(endpoint));
        *out_runtime = robotkit::internal::register_runtime(std::move(runtime));
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
}

void RK_CALL rk_robot_runtime_destroy(rk_robot_runtime runtime) {
    robotkit::internal::destroy_runtime(runtime);
}

rk_result RK_CALL rk_robot_runtime_start(rk_robot_runtime runtime) {
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->start() : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_stop(rk_robot_runtime runtime) {
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->stop() : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_submit(rk_robot_runtime runtime, const rk_robot_command *command) {
    if (!command)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->submit(*command) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_submit_trajectory(
    rk_robot_runtime runtime, const rk_robot_command *command,
    const rk_trajectory_chunk *chunk) {
    if (!command || !chunk)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->submit_trajectory(*command, *chunk) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_snapshot(rk_robot_runtime runtime, rk_robot_state *out_state) {
    if (!out_state || out_state->struct_size < sizeof(*out_state))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->snapshot(*out_state) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_snapshot_full(rk_robot_runtime runtime,
                                           rk_robot_snapshot *out_snapshot) {
    if (!out_snapshot || out_snapshot->struct_size < sizeof(*out_snapshot))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    return value ? value->snapshot_full(*out_snapshot) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_robot_runtime_capabilities(rk_robot_runtime runtime,
                                           rk_robot_capabilities *out_capabilities) {
    if (!out_capabilities || out_capabilities->struct_size < sizeof(*out_capabilities))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = robotkit::internal::resolve_runtime(runtime);
    if (!value)
        return RK_ERROR_INVALID_HANDLE;
    out_capabilities->joint_count = 0;
    out_capabilities->supports_position_targets = 1;
    out_capabilities->supports_velocity_targets = 1;
    out_capabilities->supports_effort_targets = 1;
    out_capabilities->supports_prediction = 0;
    out_capabilities->supports_trajectory_queue = value->supports_trajectory_queue();
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
