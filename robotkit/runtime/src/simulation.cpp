#include "simulation.hpp"

namespace robotkit {
namespace {
uint64_t monotonic_now_ns() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}
} // namespace

Simulation::Simulation(double fixed_timestep, uint32_t physics_substeps)
    : world_(std::make_shared<SimWorld>(fixed_timestep, physics_substeps)),
      period_(static_cast<int64_t>(fixed_timestep * 1'000'000'000.0)) {}

Simulation::~Simulation() {
    stop();
    world_->stop();
    runtimes_.clear();
    for (const auto handle : handles_)
        internal::destroy_runtime(handle);
    handles_.clear();
}

rk_result Simulation::add_robot(const rk_runtime_blueprint &blueprint,
                                rk_runtime &out_runtime) {
    std::lock_guard tick_lock(tick_mutex_);
    {
        std::lock_guard state_lock(state_mutex_);
        if (sealed_ || running_)
            return RK_ERROR_INVALID_STATE;
    }
    try {
        auto binding = world_->add_robot(blueprint);
        rk_runtime_layout layout{};
        layout.struct_size = sizeof(layout);
        layout.revision = blueprint.revision;
        layout.joint_count = blueprint.joint_count;
        layout.link_count = blueprint.link_count;
        layout.frame_count = blueprint.frame_count;
        auto endpoint = std::make_unique<SimEndpoint>(std::move(binding));
        auto runtime = std::make_shared<Runtime>(layout, std::move(endpoint));
        runtime->set_externally_driven(true);
        const auto handle = internal::register_runtime(runtime, shared_from_this());
        runtimes_.push_back(std::move(runtime));
        handles_.push_back(handle);
        out_runtime = handle;
        return RK_OK;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
}

rk_result Simulation::step(uint64_t timestamp_ns) {
    std::lock_guard tick_lock(tick_mutex_);
    {
        std::lock_guard state_lock(state_mutex_);
        if (running_ || stopping_ || runtimes_.empty())
            return RK_ERROR_INVALID_STATE;
        sealed_ = true;
    }
    for (const auto &runtime : runtimes_) {
        const auto result = runtime->apply_pending_commands();
        if (result != RK_OK) {
            for (const auto &participant : runtimes_)
                participant->discard_pending_commands();
            return result;
        }
    }
    const auto world_result = world_->step();
    if (world_result != RK_OK)
        return world_result;
    for (const auto &runtime : runtimes_) {
        const auto result = runtime->publish_sample(timestamp_ns);
        if (result != RK_OK)
            return result;
    }
    return RK_OK;
}

rk_result Simulation::start() {
    std::lock_guard tick_lock(tick_mutex_);
    std::lock_guard state_lock(state_mutex_);
    if (running_ || stopping_ || runtimes_.empty())
        return RK_ERROR_INVALID_STATE;
    if (world_->start() != RK_OK)
        return RK_ERROR_BACKEND;
    sealed_ = true;
    running_ = true;
    worker_ = std::thread([this] { run(); });
    return RK_OK;
}

rk_result Simulation::stop() {
    {
        std::lock_guard state_lock(state_mutex_);
        if (!running_ && !worker_.joinable())
            return RK_OK;
        stopping_ = true;
    }
    if (worker_.joinable())
        worker_.join();
    std::lock_guard state_lock(state_mutex_);
    running_ = false;
    stopping_ = false;
    return world_->stop();
}

void Simulation::run() {
    auto next_tick = std::chrono::steady_clock::now();
    for (;;) {
        {
            std::lock_guard state_lock(state_mutex_);
            if (stopping_)
                break;
        }
        next_tick += period_;
        {
            std::lock_guard tick_lock(tick_mutex_);
            bool commands_valid = true;
            for (const auto &runtime : runtimes_) {
                if (runtime->apply_pending_commands() != RK_OK) {
                    commands_valid = false;
                    break;
                }
            }
            if (!commands_valid) {
                for (const auto &runtime : runtimes_)
                    runtime->discard_pending_commands();
            } else if (world_->step() == RK_OK) {
                const auto timestamp_ns = monotonic_now_ns();
                for (const auto &runtime : runtimes_)
                    runtime->publish_sample(timestamp_ns);
            }
        }
        std::this_thread::sleep_until(next_tick);
    }
}

uint64_t Simulation::step_index() const {
    std::lock_guard lock(tick_mutex_);
    return world_->step_index();
}
double Simulation::simulation_time() const {
    std::lock_guard lock(tick_mutex_);
    return world_->simulation_time();
}

} // namespace robotkit
