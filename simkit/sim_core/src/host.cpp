#include "internal.hpp"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>

namespace nksim {

namespace {

constexpr std::uint32_t default_command_capacity = 256;

struct Command {
    enum class Kind { Forces, JointTargets };

    Kind kind = Kind::Forces;
    std::vector<nksim_body_force> forces;
    std::vector<nksim_joint_target> joint_targets;
};

struct StepRequest {
    std::mutex mutex;
    std::condition_variable condition;
    bool complete = false;
    nksim_result result = NKSIM_ERROR_INVALID_STATE;
    nksim_step_result step{};

    StepRequest() { step.struct_size = sizeof(step); }
};

void complete_request(const std::shared_ptr<StepRequest> &request, nksim_result result,
                      nksim_step_result step) {
    {
        std::lock_guard lock(request->mutex);
        request->result = result;
        request->step = step;
        request->complete = true;
    }
    request->condition.notify_one();
}

bool valid_host_mode(std::uint32_t mode) noexcept {
    return mode == NKSIM_HOST_MODE_REALTIME || mode == NKSIM_HOST_MODE_UNBOUNDED ||
        mode == NKSIM_HOST_MODE_EXTERNAL;
}

} // namespace

class Host {
public:
    Host(std::shared_ptr<World> world, const nksim_host_desc &desc)
        : world(std::move(world)), mode(desc.mode),
          time_scale(desc.time_scale > 0.0 ? desc.time_scale : 1.0),
          command_capacity(desc.command_capacity ? desc.command_capacity
                                                  : default_command_capacity) {}

    ~Host() { stop(); }

    nksim_result start() {
        std::unique_lock lock(mutex);
        if (started)
            return NKSIM_ERROR_INVALID_STATE;
        started = true;
        worker = std::thread([this] { run(); });
        condition.wait(lock, [this] { return ready; });
        const auto result = last_error;
        if (result == NKSIM_OK)
            return NKSIM_OK;

        std::thread failed_worker = std::move(worker);
        lock.unlock();
        if (failed_worker.joinable())
            failed_worker.join();
        world->claim_thread();
        return result;
    }

    nksim_result stop() {
        std::thread worker_to_join;
        {
            std::lock_guard lock(mutex);
            if (!started)
                return NKSIM_ERROR_INVALID_STATE;
            if (worker.joinable()) {
                stop_requested = true;
                condition.notify_all();
                worker_to_join = std::move(worker);
            }
        }
        if (worker_to_join.joinable())
            worker_to_join.join();
        world->claim_thread();
        return NKSIM_OK;
    }

    nksim_result pause() {
        std::lock_guard lock(mutex);
        if (!started || !running)
            return NKSIM_ERROR_INVALID_STATE;
        paused = true;
        condition.notify_all();
        return NKSIM_OK;
    }

    nksim_result resume() {
        std::lock_guard lock(mutex);
        if (!started || !running)
            return NKSIM_ERROR_INVALID_STATE;
        paused = false;
        condition.notify_all();
        return NKSIM_OK;
    }

    nksim_result step(nksim_step_result *out_result) {
        if (!out_result || !valid_struct_size(out_result->struct_size, sizeof(*out_result)))
            return NKSIM_ERROR_INVALID_ARGUMENT;
        auto request = std::make_shared<StepRequest>();
        {
            std::lock_guard lock(mutex);
            if (!started || !running || stop_requested)
                return NKSIM_ERROR_INVALID_STATE;
            step_requests.push_back(request);
        }
        condition.notify_all();

        std::unique_lock lock(request->mutex);
        request->condition.wait(lock, [&request] { return request->complete; });
        *out_result = request->step;
        return request->result;
    }

    nksim_result submit_forces(const nksim_body_force *forces, std::uint32_t count) {
        if (count != 0 && !forces)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        Command command;
        command.kind = Command::Kind::Forces;
        command.forces.reserve(count);
        for (std::uint32_t index = 0; index < count; ++index) {
            if (!valid_struct_size(forces[index].struct_size, sizeof(forces[index])))
                return NKSIM_ERROR_INVALID_ARGUMENT;
            command.forces.push_back(forces[index]);
        }
        return enqueue(std::move(command));
    }

    nksim_result submit_joint_targets(const nksim_joint_target *targets, std::uint32_t count) {
        if (count != 0 && !targets)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        Command command;
        command.kind = Command::Kind::JointTargets;
        command.joint_targets.reserve(count);
        for (std::uint32_t index = 0; index < count; ++index) {
            if (!valid_struct_size(targets[index].struct_size, sizeof(targets[index])))
                return NKSIM_ERROR_INVALID_ARGUMENT;
            command.joint_targets.push_back(targets[index]);
        }
        return enqueue(std::move(command));
    }

    nksim_result get_snapshot(nksim_snapshot *out_snapshot) {
        if (!out_snapshot)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        *out_snapshot = NKSIM_INVALID_SNAPSHOT;
        std::shared_ptr<Snapshot> snapshot;
        {
            std::unique_lock lock(mutex);
            if (!started)
                return NKSIM_ERROR_INVALID_STATE;
            condition.wait(lock, [this] { return ready || stop_requested; });
            snapshot = latest_snapshot;
            if (!snapshot)
                return last_error == NKSIM_OK ? NKSIM_ERROR_INVALID_STATE : last_error;
        }

        auto &state = registry();
        std::lock_guard lock(state.mutex);
        const auto handle = state.snapshots.create(std::move(snapshot));
        if (!handle)
            return NKSIM_ERROR_OUT_OF_MEMORY;
        *out_snapshot = handle;
        return NKSIM_OK;
    }

    nksim_result get_clock(nksim_clock *out_clock) const {
        if (!out_clock || !valid_struct_size(out_clock->struct_size, sizeof(*out_clock)))
            return NKSIM_ERROR_INVALID_ARGUMENT;
        std::lock_guard lock(mutex);
        if (!started)
            return NKSIM_ERROR_INVALID_STATE;
        latest_clock.write(*out_clock);
        return NKSIM_OK;
    }

    nksim_result get_status(nksim_host_status *out_status) const {
        if (!out_status || !valid_struct_size(out_status->struct_size, sizeof(*out_status)))
            return NKSIM_ERROR_INVALID_ARGUMENT;
        std::lock_guard lock(mutex);
        out_status->struct_size = sizeof(*out_status);
        out_status->running = running ? 1u : 0u;
        out_status->paused = paused ? 1u : 0u;
        out_status->mode = mode;
        out_status->reserved0 = 0;
        out_status->step_index = latest_clock.step_index;
        out_status->simulation_time = latest_clock.time;
        out_status->last_error = last_error;
        out_status->queued_commands = static_cast<std::uint32_t>(commands.size());
        out_status->reserved[0] = 0;
        out_status->reserved[1] = 0;
        out_status->reserved[2] = 0;
        return NKSIM_OK;
    }

private:
    nksim_result enqueue(Command command) {
        if (command.forces.empty() && command.joint_targets.empty())
            return NKSIM_OK;
        std::lock_guard lock(mutex);
        if (!started || !running || stop_requested)
            return NKSIM_ERROR_INVALID_STATE;
        if (commands.size() >= command_capacity)
            return NKSIM_ERROR_INVALID_STATE;
        commands.push_back(std::move(command));
        condition.notify_all();
        return NKSIM_OK;
    }

    nksim_result publish_snapshot() {
        std::shared_ptr<Snapshot> snapshot;
        const auto result = world->snapshot(snapshot);
        if (result != NKSIM_OK)
            return result;
        {
            std::lock_guard lock(mutex);
            latest_clock = snapshot->clock;
            latest_snapshot = std::move(snapshot);
        }
        condition.notify_all();
        return NKSIM_OK;
    }

    nksim_result process_commands() {
        std::deque<Command> pending;
        {
            std::lock_guard lock(mutex);
            pending.swap(commands);
        }
        for (auto &command : pending) {
            nksim_result result = NKSIM_OK;
            if (command.kind == Command::Kind::Forces) {
                result = world->apply_forces(command.forces.data(),
                                             static_cast<std::uint32_t>(command.forces.size()));
            } else {
                result = world->set_joint_targets(
                    command.joint_targets.data(),
                    static_cast<std::uint32_t>(command.joint_targets.size()));
            }
            if (result != NKSIM_OK)
                return result;
        }
        return NKSIM_OK;
    }

    void perform_step(const std::shared_ptr<StepRequest> &request) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        auto result = process_commands();
        if (result == NKSIM_OK)
            result = world->step(&step);
        if (result == NKSIM_OK)
            result = publish_snapshot();

        {
            std::lock_guard lock(mutex);
            last_error = result;
        }
        if (request) {
            if (result != NKSIM_OK && step.scene_changes)
                nkscene_change_set_destroy(step.scene_changes);
            complete_request(request, result, step);
        } else if (step.scene_changes) {
            nkscene_change_set_destroy(step.scene_changes);
        }
    }

    void finish() {
        std::deque<std::shared_ptr<StepRequest>> pending;
        {
            std::lock_guard lock(mutex);
            running = false;
            ready = true;
            pending.swap(step_requests);
        }
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        for (const auto &request : pending)
            complete_request(request, NKSIM_ERROR_INVALID_STATE, step);
        condition.notify_all();
    }

    void run() {
        world->claim_thread();
        const auto initial_result = publish_snapshot();
        {
            std::lock_guard lock(mutex);
            last_error = initial_result;
            ready = true;
            running = initial_result == NKSIM_OK;
        }
        condition.notify_all();
        if (initial_result != NKSIM_OK) {
            finish();
            return;
        }

        const auto interval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(world->desc().fixed_timestep / time_scale));
        auto next_deadline = std::chrono::steady_clock::now() + interval;
        for (;;) {
            std::shared_ptr<StepRequest> request;
            {
                std::unique_lock lock(mutex);
                if (mode == NKSIM_HOST_MODE_EXTERNAL) {
                    condition.wait(lock, [this] {
                        return stop_requested || !step_requests.empty();
                    });
                } else if (paused) {
                    condition.wait(lock, [this] {
                        return stop_requested || !step_requests.empty() || !paused;
                    });
                } else if (mode == NKSIM_HOST_MODE_REALTIME && step_requests.empty()) {
                    if (condition.wait_until(lock, next_deadline, [this] {
                            return stop_requested || paused || !step_requests.empty();
                        }) && !stop_requested)
                        continue;
                }

                if (stop_requested)
                    break;
                if (!step_requests.empty()) {
                    request = step_requests.front();
                    step_requests.pop_front();
                } else if (mode == NKSIM_HOST_MODE_EXTERNAL || paused) {
                    continue;
                }
            }

            perform_step(request);
            if (mode == NKSIM_HOST_MODE_REALTIME)
                next_deadline = std::chrono::steady_clock::now() + interval;
        }
        finish();
    }

    std::shared_ptr<World> world;
    std::uint32_t mode = NKSIM_HOST_MODE_EXTERNAL;
    double time_scale = 1.0;
    std::uint32_t command_capacity = default_command_capacity;

    mutable std::mutex mutex;
    std::condition_variable condition;
    std::deque<Command> commands;
    std::deque<std::shared_ptr<StepRequest>> step_requests;
    std::shared_ptr<Snapshot> latest_snapshot;
    Clock latest_clock;
    std::thread worker;
    nksim_result last_error = NKSIM_OK;
    bool started = false;
    bool ready = false;
    bool running = false;
    bool paused = false;
    bool stop_requested = false;
};

std::shared_ptr<Host> resolve_host(nksim_host host) noexcept {
    auto &state = registry();
    std::lock_guard lock(state.mutex);
    return state.hosts.get(host);
}

} // namespace nksim

extern "C" {

nksim_result NKSIM_HOST_CALL nksim_host_create(const nksim_host_desc *desc,
                                               nksim_host *out_host) {
    if (!desc || !out_host || !nksim::valid_struct_size(desc->struct_size, sizeof(*desc)) ||
        !nksim::valid_host_mode(desc->mode) || !desc->world ||
        !std::isfinite(desc->time_scale) || desc->time_scale < 0.0)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    *out_host = NKSIM_INVALID_HOST;
    const auto world = nksim::resolve_world(desc->world);
    if (!world)
        return NKSIM_ERROR_INVALID_HANDLE;
    auto host = std::make_shared<nksim::Host>(world, *desc);
    auto &state = nksim::registry();
    std::lock_guard lock(state.mutex);
    const auto handle = state.hosts.create(std::move(host));
    if (!handle)
        return NKSIM_ERROR_OUT_OF_MEMORY;
    *out_host = handle;
    return NKSIM_OK;
}

void NKSIM_HOST_CALL nksim_host_destroy(nksim_host host) {
    std::shared_ptr<nksim::Host> value;
    {
        auto &state = nksim::registry();
        std::lock_guard lock(state.mutex);
        value = state.hosts.remove(host);
    }
    if (value)
        value->stop();
}

nksim_result NKSIM_HOST_CALL nksim_host_start(nksim_host host) {
    const auto value = nksim::resolve_host(host);
    return value ? value->start() : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_stop(nksim_host host) {
    const auto value = nksim::resolve_host(host);
    return value ? value->stop() : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_pause(nksim_host host) {
    const auto value = nksim::resolve_host(host);
    return value ? value->pause() : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_resume(nksim_host host) {
    const auto value = nksim::resolve_host(host);
    return value ? value->resume() : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_step(nksim_host host, nksim_step_result *out_result) {
    const auto value = nksim::resolve_host(host);
    return value ? value->step(out_result) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_submit_forces(nksim_host host,
                                                      const nksim_body_force *forces,
                                                      uint32_t count) {
    const auto value = nksim::resolve_host(host);
    return value ? value->submit_forces(forces, count) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_submit_joint_targets(
    nksim_host host, const nksim_joint_target *targets, uint32_t count) {
    const auto value = nksim::resolve_host(host);
    return value ? value->submit_joint_targets(targets, count) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_get_snapshot(nksim_host host,
                                                     nksim_snapshot *out_snapshot) {
    const auto value = nksim::resolve_host(host);
    return value ? value->get_snapshot(out_snapshot) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_get_clock(nksim_host host, nksim_clock *out_clock) {
    const auto value = nksim::resolve_host(host);
    return value ? value->get_clock(out_clock) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_HOST_CALL nksim_host_get_status(nksim_host host,
                                                   nksim_host_status *out_status) {
    const auto value = nksim::resolve_host(host);
    return value ? value->get_status(out_status) : NKSIM_ERROR_INVALID_HANDLE;
}

} // extern "C"
