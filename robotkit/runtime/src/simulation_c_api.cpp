#include "robotkit_simkit.h"
#include "simulation.hpp"

#include <memory>
#include <cmath>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace {
std::mutex simulations_mutex;
std::unordered_map<rk_simulation, std::shared_ptr<robotkit::Simulation>> simulations;
rk_simulation next_simulation = 1;
struct Presentation {
    rk_simulation_presentation_info info{};
    std::vector<rk_simulation_presentation_pose> poses;
};
std::mutex presentations_mutex;
std::unordered_map<rk_simulation_presentation, std::shared_ptr<const Presentation>> presentations;
rk_simulation_presentation next_presentation = 1;

std::shared_ptr<robotkit::Simulation> resolve(rk_simulation handle) {
    std::lock_guard lock(simulations_mutex);
    const auto found = simulations.find(handle);
    return found == simulations.end() ? nullptr : found->second;
}

rk_simulation store(std::shared_ptr<robotkit::Simulation> simulation) {
    std::lock_guard lock(simulations_mutex);
    while (next_simulation == RK_INVALID_SIMULATION ||
           simulations.count(next_simulation) != 0)
        ++next_simulation;
    const auto handle = next_simulation++;
    simulations.emplace(handle, std::move(simulation));
    return handle;
}

rk_simulation_presentation store_presentation(std::shared_ptr<const Presentation> presentation) {
    std::lock_guard lock(presentations_mutex);
    while (next_presentation == RK_INVALID_SIMULATION_PRESENTATION ||
           presentations.count(next_presentation) != 0)
        ++next_presentation;
    const auto handle = next_presentation++;
    presentations.emplace(handle, std::move(presentation));
    return handle;
}

std::shared_ptr<const Presentation> resolve_presentation(rk_simulation_presentation handle) {
    std::lock_guard lock(presentations_mutex);
    const auto found = presentations.find(handle);
    return found == presentations.end() ? nullptr : found->second;
}
} // namespace

extern "C" {

rk_result RK_CALL rk_simulation_create(const rk_simulation_desc *desc,
                                       rk_simulation *out_simulation) {
    if (!desc || desc->struct_size < sizeof(*desc) || !out_simulation ||
        !std::isfinite(desc->fixed_timestep) || desc->fixed_timestep <= 0.0 ||
        desc->physics_substeps == 0 || desc->backend > 1)
        return RK_ERROR_INVALID_ARGUMENT;
    *out_simulation = RK_INVALID_SIMULATION;
#ifndef RK_HAS_MUJOCO
    if (desc->backend == 1) return RK_ERROR_UNSUPPORTED;
#endif
    try {
        *out_simulation = store(std::make_shared<robotkit::Simulation>(
            desc->fixed_timestep, desc->physics_substeps, desc->backend));
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
}

void RK_CALL rk_simulation_destroy(rk_simulation simulation) {
    std::shared_ptr<robotkit::Simulation> released;
    {
        std::lock_guard lock(simulations_mutex);
        const auto found = simulations.find(simulation);
        if (found == simulations.end())
            return;
        released = std::move(found->second);
        simulations.erase(found);
    }
}

rk_result RK_CALL rk_simulation_add_robot(rk_simulation simulation,
                                          const rk_robot_runtime_blueprint *blueprint,
                                          rk_robot_runtime *out_runtime) {
    if (!out_runtime || rk_robot_runtime_blueprint_validate(blueprint) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    *out_runtime = RK_INVALID_ROBOT_RUNTIME;
    const auto value = resolve(simulation);
    return value ? value->add_robot(*blueprint, *out_runtime) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_step(rk_simulation simulation, uint64_t timestamp_ns) {
    const auto value = resolve(simulation);
    return value ? value->step(timestamp_ns) : RK_ERROR_INVALID_HANDLE;
}
rk_result RK_CALL rk_simulation_start(rk_simulation simulation) {
    const auto value = resolve(simulation);
    return value ? value->start() : RK_ERROR_INVALID_HANDLE;
}
rk_result RK_CALL rk_simulation_stop(rk_simulation simulation) {
    const auto value = resolve(simulation);
    return value ? value->stop() : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_clock(rk_simulation simulation,
                                          rk_simulation_clock *out_clock) {
    if (!out_clock || out_clock->struct_size < sizeof(*out_clock))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    if (!value)
        return RK_ERROR_INVALID_HANDLE;
    out_clock->step_index = value->step_index();
    out_clock->simulation_time = value->simulation_time();
    return RK_OK;
}

rk_result RK_CALL rk_simulation_reset(rk_simulation simulation) {
    const auto value = resolve(simulation);
    return value ? value->reset() : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_reset_robot(rk_simulation simulation, uint32_t robot_index) {
    const auto value = resolve(simulation);
    return value ? value->reset_robot(robot_index) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_teleport_robot(rk_simulation simulation, uint32_t robot_index,
                                                const rk_simulation_pose *pose) {
    if (!pose)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->teleport_robot(robot_index, *pose)
                 : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_robot_pose(rk_simulation simulation,uint32_t robot_index,
                                                rk_simulation_pose *out_pose) {
    if(!out_pose)return RK_ERROR_INVALID_ARGUMENT;
    const auto value=resolve(simulation);
    return value?value->get_robot_pose(robot_index,*out_pose):RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_link_pose(rk_simulation simulation,uint32_t robot_index,
                                               uint32_t link_index,rk_simulation_pose *out_pose) {
    if(!out_pose)return RK_ERROR_INVALID_ARGUMENT;
    const auto value=resolve(simulation);
    return value?value->get_link_pose(robot_index,link_index,*out_pose):RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_spawn_object(rk_simulation simulation,
                                             const rk_simulation_object_desc *desc,
                                             rk_simulation_object *out_object) {
    if (!desc || !out_object)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->spawn_object(*desc, *out_object) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_remove_object(rk_simulation simulation,
                                              rk_simulation_object object) {
    const auto value = resolve(simulation);
    return value ? value->remove_object(object) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_teleport_object(rk_simulation simulation,
                                                rk_simulation_object object,
                                                const rk_simulation_pose *pose) {
    if (!pose)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->teleport_object(object, *pose)
                 : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_object_pose(rk_simulation simulation,
                                                 rk_simulation_object object,
                                                 rk_simulation_pose *out_pose) {
    if(!out_pose)return RK_ERROR_INVALID_ARGUMENT;
    const auto value=resolve(simulation);
    return value?value->get_object_pose(object,*out_pose):RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_capture_presentation(
    rk_simulation simulation, rk_simulation_presentation *out_presentation) {
    if (!out_presentation) return RK_ERROR_INVALID_ARGUMENT;
    *out_presentation = RK_INVALID_SIMULATION_PRESENTATION;
    const auto value = resolve(simulation);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    try {
        auto presentation = std::make_shared<Presentation>();
        presentation->info.struct_size = sizeof(presentation->info);
        const auto result = value->capture_presentation(presentation->info, presentation->poses);
        if (result != RK_OK) return result;
        presentation->info.pose_count = static_cast<uint32_t>(presentation->poses.size());
        *out_presentation = store_presentation(std::move(presentation));
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
}

rk_result RK_CALL rk_simulation_presentation_get_info(
    rk_simulation_presentation presentation,
    rk_simulation_presentation_info *out_info) {
    if (!out_info || out_info->struct_size < sizeof(*out_info))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve_presentation(presentation);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    *out_info = value->info;
    return RK_OK;
}

rk_result RK_CALL rk_simulation_presentation_get_pose(
    rk_simulation_presentation presentation, uint32_t index,
    rk_simulation_presentation_pose *out_pose) {
    if (!out_pose || out_pose->struct_size < sizeof(*out_pose))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve_presentation(presentation);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    if (index >= value->poses.size()) return RK_ERROR_INVALID_ARGUMENT;
    *out_pose = value->poses[index];
    return RK_OK;
}

void RK_CALL rk_simulation_presentation_destroy(rk_simulation_presentation presentation) {
    std::lock_guard lock(presentations_mutex);
    presentations.erase(presentation);
}

} // extern "C"
