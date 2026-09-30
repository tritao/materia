#include "robotkit_simkit.h"
#include "simulation.hpp"
#include "runtime_abi.hpp"

#include <memory>
#include <algorithm>
#include <cstddef>
#include <cstring>
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
struct ContactList {
    uint64_t step_index = 0;
    std::vector<rk_robot_contact> contacts;
};
std::mutex contact_lists_mutex;
std::unordered_map<rk_robot_contact_list, std::shared_ptr<const ContactList>> contact_lists;
rk_robot_contact_list next_contact_list = 1;
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
std::shared_ptr<const ContactList> resolve_contact_list(rk_robot_contact_list handle) {
    std::lock_guard lock(contact_lists_mutex);
    const auto found = contact_lists.find(handle);
    return found == contact_lists.end() ? nullptr : found->second;
}
} // namespace

extern "C" {

rk_result RK_CALL rk_simulation_capture_robot_contacts(rk_simulation simulation,
    rk_robot_runtime runtime, rk_robot_contact_list *out_list) {
    if (!out_list) return RK_ERROR_INVALID_ARGUMENT;
    *out_list = RK_INVALID_ROBOT_CONTACT_LIST;
    const auto value = resolve(simulation);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    try {
        auto list = std::make_shared<ContactList>();
        const auto status = value->get_robot_contacts(runtime, list->contacts, &list->step_index);
        if (status != RK_OK) return status;
        std::lock_guard lock(contact_lists_mutex);
        while (next_contact_list == RK_INVALID_ROBOT_CONTACT_LIST ||
               contact_lists.count(next_contact_list) != 0) ++next_contact_list;
        *out_list = next_contact_list++;
        contact_lists.emplace(*out_list, std::move(list));
        return RK_OK;
    } catch (const std::bad_alloc &) { return RK_ERROR_OUT_OF_MEMORY; }
    catch (...) { return RK_ERROR_BACKEND; }
}

rk_result RK_CALL rk_robot_contact_list_count(rk_robot_contact_list list, uint32_t *out_count) {
    if (!out_count) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve_contact_list(list);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    *out_count = static_cast<uint32_t>(value->contacts.size());
    return RK_OK;
}

rk_result RK_CALL rk_robot_contact_list_get(rk_robot_contact_list list, uint32_t index,
    rk_robot_contact *out_contact) {
    if (!out_contact || out_contact->struct_size < offsetof(rk_robot_contact, other_kind))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve_contact_list(list);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    if (index >= value->contacts.size()) return RK_ERROR_INVALID_ARGUMENT;
    const auto size = std::min<std::size_t>(out_contact->struct_size, sizeof(*out_contact));
    std::memcpy(out_contact, &value->contacts[index], size);
    out_contact->struct_size = static_cast<uint32_t>(size);
    return RK_OK;
}

rk_result RK_CALL rk_robot_contact_list_step_index(rk_robot_contact_list list,
    uint64_t *out_step_index) {
    if (!out_step_index) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve_contact_list(list);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    *out_step_index = value->step_index;
    return RK_OK;
}

void RK_CALL rk_robot_contact_list_destroy(rk_robot_contact_list list) {
    std::lock_guard lock(contact_lists_mutex);
    contact_lists.erase(list);
}

rk_result RK_CALL rk_simulation_create_in_session(nksim_session session,
                                                  rk_simulation *out_simulation) {
    if (!out_simulation)
        return RK_ERROR_INVALID_ARGUMENT;
    *out_simulation = RK_INVALID_SIMULATION;
    nksim_session_status status{};
    status.struct_size = sizeof(status);
    if (nksim_session_get_status(session, &status) != NKSIM_OK)
        return RK_ERROR_INVALID_HANDLE;
    if (status.hosted)
        return RK_ERROR_INVALID_STATE;
    try {
        *out_simulation = store(std::make_shared<robotkit::Simulation>(session));
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
                                          const rk_simulation_robot_desc *robot_desc,
                                          rk_robot_runtime *out_runtime) {
    if (!out_runtime || rk_robot_runtime_blueprint_validate(blueprint) != RK_OK ||
        (robot_desc && (robot_desc->struct_size < offsetof(rk_simulation_robot_desc, virtual_device_enabled) ||
                        (robot_desc->initial_pose.struct_size != 0 &&
                         robot_desc->initial_pose.struct_size < sizeof(robot_desc->initial_pose)))))
        return RK_ERROR_INVALID_ARGUMENT;
    *out_runtime = RK_INVALID_ROBOT_RUNTIME;
    const auto value = resolve(simulation);
    return value ? value->add_robot(robotkit::internal::copy_blueprint(blueprint), *out_runtime,
        robot_desc) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_cut_virtual_device_link(
    rk_simulation simulation, uint32_t robot_index, uint32_t cut) {
    if (cut > 1) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->cut_virtual_device_link(robot_index, cut != 0) :
        RK_ERROR_INVALID_HANDLE;
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

rk_result RK_CALL rk_simulation_get_rejection(rk_simulation simulation,
                                              rk_result *out_result) {
    if (!out_result)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    if (!value)
        return RK_ERROR_INVALID_HANDLE;
    *out_result = value->rejection();
    return RK_OK;
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

rk_result RK_CALL rk_simulation_drive_robot_base(rk_simulation simulation, uint32_t robot_index,
                                                  const rk_simulation_pose *pose) {
    if (!pose)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->drive_robot_base(robot_index, *pose)
                 : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_place_robot_base(rk_simulation simulation, uint32_t robot_index,
                                                  const rk_simulation_pose *pose) {
    if (!pose)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->place_robot_base(robot_index, *pose) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_set_differential_drive(
    rk_simulation simulation, uint32_t robot_index,
    const rk_simulation_differential_drive_desc *desc) {
    if (!desc)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->set_differential_drive(robot_index, *desc) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_clear_differential_drive(rk_simulation simulation,
                                                          uint32_t robot_index) {
    const auto value = resolve(simulation);
    return value ? value->clear_differential_drive(robot_index) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_differential_drive_state(
    rk_simulation simulation, uint32_t robot_index,
    rk_simulation_differential_drive_state *out_state) {
    if (!out_state)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->get_differential_drive_state(robot_index, *out_state)
                 : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_set_omni_drive(rk_simulation simulation, uint32_t robot_index,
                                               const rk_simulation_omni_drive_desc *desc) {
    if (!desc)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->set_omni_drive(robot_index, *desc) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_clear_omni_drive(rk_simulation simulation,
                                                 uint32_t robot_index) {
    const auto value = resolve(simulation);
    return value ? value->clear_omni_drive(robot_index) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_omni_drive_state(rk_simulation simulation,
                                                     uint32_t robot_index,
                                                     rk_simulation_omni_drive_state *out_state) {
    if (!out_state)
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->get_omni_drive_state(robot_index, *out_state)
                 : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_set_joint_positions(rk_simulation simulation,
    uint32_t robot_index, const double *positions, uint32_t count) {
    const auto value = resolve(simulation);
    return value ? value->set_joint_positions(robot_index, positions, count)
                 : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_robot_pose(rk_simulation simulation,uint32_t robot_index,
                                                rk_simulation_pose *out_pose) {
    if(!out_pose)return RK_ERROR_INVALID_ARGUMENT;
    const auto value=resolve(simulation);
    return value?value->get_robot_pose(robot_index,*out_pose):RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_robot_base_velocity(rk_simulation simulation,
    uint32_t robot_index, rk_simulation_twist *out_twist) {
    if (!out_twist) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->get_robot_base_velocity(robot_index, *out_twist)
                 : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_apply_robot_force(rk_simulation simulation, uint32_t robot_index,
                                                  const rk_simulation_wrench *wrench) {
    if (!wrench) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(simulation);
    return value ? value->apply_robot_force(robot_index, *wrench) : RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_link_pose(rk_simulation simulation,uint32_t robot_index,
                                               uint32_t link_index,rk_simulation_pose *out_pose) {
    if(!out_pose)return RK_ERROR_INVALID_ARGUMENT;
    const auto value=resolve(simulation);
    return value?value->get_link_pose(robot_index,link_index,*out_pose):RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_link_body(rk_simulation simulation,uint32_t robot_index,
                                               uint32_t link_index,nksim_body *out_body) {
    if(!out_body)return RK_ERROR_INVALID_ARGUMENT;
    const auto value=resolve(simulation);
    return value?value->get_link_body(robot_index,link_index,*out_body):RK_ERROR_INVALID_HANDLE;
}

rk_result RK_CALL rk_simulation_get_robot_contacts(rk_simulation simulation,
    rk_robot_runtime runtime, rk_robot_contact *out, uint32_t capacity,
    uint32_t *out_count) {
    if (!out_count || (capacity != 0 && !out)) return RK_ERROR_INVALID_ARGUMENT;
    rk_robot_contact_list list = RK_INVALID_ROBOT_CONTACT_LIST;
    const auto status = rk_simulation_capture_robot_contacts(simulation, runtime, &list);
    if (status != RK_OK) return status;
    const auto value = resolve_contact_list(list);
    *out_count = static_cast<uint32_t>(value->contacts.size());
    if (capacity != 0) {
        const auto stride = out->struct_size == 0 ? sizeof(rk_robot_contact) : out->struct_size;
        if (stride < offsetof(rk_robot_contact, other_kind)) {
            rk_robot_contact_list_destroy(list);
            return RK_ERROR_INVALID_ARGUMENT;
        }
        for (uint32_t i = 0; i < std::min(capacity, *out_count); ++i) {
            auto *target = reinterpret_cast<rk_robot_contact *>(
                reinterpret_cast<std::uint8_t *>(out) + i * stride);
            const auto copy_size = std::min<std::size_t>(stride, sizeof(rk_robot_contact));
            std::memcpy(target, &value->contacts[i], copy_size);
            target->struct_size = static_cast<uint32_t>(copy_size);
        }
    }
    rk_robot_contact_list_destroy(list);
    return RK_OK;
}

rk_result RK_CALL rk_simulation_get_robot_contact(rk_simulation simulation,
    rk_robot_runtime runtime, uint32_t index, rk_robot_contact *out_contact) {
    if (!out_contact || out_contact->struct_size < offsetof(rk_robot_contact, other_kind))
        return RK_ERROR_INVALID_ARGUMENT;
    rk_robot_contact_list list = RK_INVALID_ROBOT_CONTACT_LIST;
    const auto status = rk_simulation_capture_robot_contacts(simulation, runtime, &list);
    if (status != RK_OK) return status;
    const auto value = resolve_contact_list(list);
    if (index >= value->contacts.size()) {
        rk_robot_contact_list_destroy(list);
        return RK_ERROR_INVALID_ARGUMENT;
    }
    const auto size = std::min<std::size_t>(out_contact->struct_size, sizeof(*out_contact));
    std::memcpy(out_contact, &value->contacts[index], size);
    out_contact->struct_size = static_cast<uint32_t>(size);
    rk_robot_contact_list_destroy(list);
    return RK_OK;
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

rk_result RK_CALL rk_simulation_present_frame(
    rk_simulation simulation, nksim_frame frame, rk_simulation_presentation *out_presentation) {
    if (!out_presentation) return RK_ERROR_INVALID_ARGUMENT;
    *out_presentation = RK_INVALID_SIMULATION_PRESENTATION;
    const auto value = resolve(simulation);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    try {
        auto presentation = std::make_shared<Presentation>();
        presentation->info.struct_size = sizeof(presentation->info);
        const auto result = value->present_frame(frame, presentation->info, presentation->poses);
        if (result != RK_OK) return result;
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
