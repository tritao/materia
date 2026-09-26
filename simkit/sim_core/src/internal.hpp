#pragma once

#include "nativekit_sim.h"
#include "nativekit_sim_host.h"
#include "PhysicsBackend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace nksim {

class Host;

constexpr std::uint32_t handle_index_bits = 20;
constexpr std::uint32_t handle_index_mask = (1u << handle_index_bits) - 1u;
constexpr std::uint32_t handle_max_generation = (1u << (32 - handle_index_bits)) - 1u;

struct RuntimeHandle {
    std::uint32_t slot = 0;
    std::uint32_t generation = 0;

    constexpr bool valid() const noexcept { return generation != 0; }
};

inline std::uint32_t pack_handle(RuntimeHandle handle) noexcept {
    if (!handle.valid() || handle.slot >= handle_index_mask ||
        handle.generation > handle_max_generation)
        return 0;
    return (handle.generation << handle_index_bits) | (handle.slot + 1);
}

inline RuntimeHandle unpack_handle(std::uint32_t value) noexcept {
    if (value == 0 || (value & handle_index_mask) == 0)
        return {};
    return {static_cast<std::uint32_t>((value & handle_index_mask) - 1),
            static_cast<std::uint32_t>(value >> handle_index_bits)};
}

template<class T>
class OwnedTable {
public:
    template<class... Args>
    std::pair<std::uint32_t, T *> create(Args &&...args) {
        RuntimeHandle handle;
        if (free_slots.empty()) {
            if (slots.size() >= handle_index_mask)
                return {0, nullptr};
            slots.push_back({nullptr, 1});
            handle = {static_cast<std::uint32_t>(slots.size() - 1), 1};
        } else {
            handle.slot = free_slots.back();
            free_slots.pop_back();
            auto &slot = slots[handle.slot];
            ++slot.generation;
            handle.generation = slot.generation;
        }

        auto value = std::make_unique<T>(std::forward<Args>(args)...);
        auto &slot = slots[handle.slot];
        slot.value = std::move(value);
        return {pack_handle(handle), slot.value.get()};
    }

    T *get(std::uint32_t value) noexcept {
        const auto handle = unpack_handle(value);
        return valid(handle) ? slots[handle.slot].value.get() : nullptr;
    }

    const T *get(std::uint32_t value) const noexcept {
        const auto handle = unpack_handle(value);
        return valid(handle) ? slots[handle.slot].value.get() : nullptr;
    }

    bool empty() const noexcept { return std::all_of(slots.begin(), slots.end(), [](const Slot &slot) {
        return slot.value == nullptr;
    }); }

    std::unique_ptr<T> remove(std::uint32_t value) noexcept {
        const auto handle = unpack_handle(value);
        if (!valid(handle))
            return {};
        auto &slot = slots[handle.slot];
        auto result = std::move(slot.value);
        if (slot.generation < handle_max_generation)
            free_slots.push_back(handle.slot);
        return result;
    }

    template<class Fn>
    void for_each(Fn &&fn) const {
        for (std::uint32_t index = 0; index < slots.size(); ++index) {
            const auto &slot = slots[index];
            if (slot.value)
                fn(pack_handle({index, slot.generation}), *slot.value);
        }
    }

private:
    struct Slot {
        std::unique_ptr<T> value;
        std::uint32_t generation = 1;
    };

    bool valid(RuntimeHandle handle) const noexcept {
        return handle.valid() && handle.slot < slots.size() &&
            slots[handle.slot].generation == handle.generation &&
            slots[handle.slot].value != nullptr;
    }

    std::vector<Slot> slots;
    std::vector<std::uint32_t> free_slots;
};

template<class T>
class SharedTable {
public:
    std::uint32_t create(std::shared_ptr<T> value) {
        RuntimeHandle handle;
        if (free_slots.empty()) {
            if (slots.size() >= handle_index_mask)
                return 0;
            slots.push_back({std::move(value), 1});
            handle = {static_cast<std::uint32_t>(slots.size() - 1), 1};
        } else {
            handle.slot = free_slots.back();
            free_slots.pop_back();
            auto &slot = slots[handle.slot];
            ++slot.generation;
            slot.value = std::move(value);
            handle.generation = slot.generation;
        }
        return pack_handle(handle);
    }

    std::shared_ptr<T> get(std::uint32_t value) const noexcept {
        const auto handle = unpack_handle(value);
        if (!handle.valid() || handle.slot >= slots.size())
            return {};
        const auto &slot = slots[handle.slot];
        if (slot.generation != handle.generation)
            return {};
        return slot.value;
    }

    std::shared_ptr<T> remove(std::uint32_t value) noexcept {
        const auto handle = unpack_handle(value);
        if (!handle.valid() || handle.slot >= slots.size())
            return {};
        auto &slot = slots[handle.slot];
        if (slot.generation != handle.generation || !slot.value)
            return {};
        auto result = std::move(slot.value);
        if (slot.generation < handle_max_generation)
            free_slots.push_back(handle.slot);
        return result;
    }

    template<class Fn>
    void for_each(Fn &&fn) const {
        for (std::uint32_t index = 0; index < slots.size(); ++index) {
            const auto &slot = slots[index];
            if (slot.value)
                fn(pack_handle({index, slot.generation}), *slot.value);
        }
    }

private:
    struct Slot {
        std::shared_ptr<T> value;
        std::uint32_t generation = 1;
    };

    std::vector<Slot> slots;
    std::vector<std::uint32_t> free_slots;
};

struct Clock {
    double time = 0.0;
    double fixed_timestep = 0.0;
    std::uint64_t step_index = 0;

    void write(nksim_clock &out) const noexcept {
        out.struct_size = sizeof(out);
        out.time = time;
        out.fixed_timestep = fixed_timestep;
        out.step_index = step_index;
    }

    void advance() noexcept {
        time += fixed_timestep;
        ++step_index;
    }
};

struct Shape {
    nksim_shape handle = 0;
    nksim_shape_desc desc{};
};

struct Body {
    nksim_body handle = 0;
    nksim_body_desc desc{};
    std::uint64_t backend_body = 0;
    nksim_body_state state{};
    nksim_body_state initial_state{};
    /**
     * Kinematic pose and twist for the step in progress: the scene-node pose
     * the body reaches at the end of the step and the twist that carries it
     * there. Committed to state after the backend step.
     */
    nksim_body_state kinematic_target{};
    /**
     * True when state holds the pose this kinematic body reached on the
     * previous step, so the next scene-node pose is continuous motion whose
     * twist is the finite difference over the step. Cleared by explicit state
     * writes and resets, which are discontinuous: the first step after one
     * keeps the twist carried by that state instead (zero after a reset).
     */
    bool kinematic_continuous = false;
    /**
     * Pending nksim_body_drive(): the exact pose and twist for the next step,
     * which take precedence over the scene node for that step.
     */
    nksim_body_state kinematic_drive{};
    bool kinematic_drive_pending = false;
    /**
     * The scene-node pose seen on the previous step. Scene nodes are single
     * precision, so node motion is differenced against the previous node pose
     * rather than against state, which a drive may have set in double
     * precision.
     */
    std::array<double, 3> kinematic_node_position{};
    std::array<double, 4> kinematic_node_rotation{0.0, 0.0, 0.0, 1.0};
};

struct Joint {
    nksim_joint handle = 0;
    nksim_joint_desc desc{};
    std::uint64_t backend_joint = 0;
    nksim_joint_state state{};
};

struct Snapshot {
    Clock clock;
    std::vector<nksim_body_state> bodies;
    std::vector<nksim_joint_state> joints;
};

class World {
public:
    World(const nksim_world_desc &desc, std::unique_ptr<PhysicsBackend> backend);
    ~World();

    nksim_result initialize();
    bool owns_thread() const noexcept { return owner_thread == std::this_thread::get_id(); }
    void claim_thread() noexcept { owner_thread = std::this_thread::get_id(); }

    nksim_result get_clock(nksim_clock *out_clock) const noexcept;
    nksim_result begin_topology_update();
    nksim_result end_topology_update();
    nksim_result step(nksim_step_result *out_result);
    nksim_result apply_forces(const nksim_body_force *forces, std::uint32_t count);
    nksim_result set_joint_targets(const nksim_joint_target *targets, std::uint32_t count);
    nksim_result snapshot(std::shared_ptr<Snapshot> &out_snapshot) const;

    nksim_result create_shape(const nksim_shape_desc &desc, nksim_shape *out_shape);
    nksim_result destroy_shape(nksim_shape shape);
    nksim_result create_body(const nksim_body_desc &desc, nksim_body *out_body);
    nksim_result destroy_body(nksim_body body);
    nksim_result get_body_state(nksim_body body, nksim_body_state *out_state) const;
    nksim_result set_body_state(nksim_body body, const nksim_body_state &state);
    nksim_result drive_body(nksim_body body, const nksim_body_state &state);
    nksim_result reset_body(nksim_body body);
    nksim_result reset();
    nksim_result create_joint(const nksim_joint_desc &desc, nksim_joint *out_joint);
    nksim_result destroy_joint(nksim_joint joint);
    nksim_result get_joint_state(nksim_joint joint, nksim_joint_state *out_state) const;

    const nksim_world_desc &desc() const noexcept { return world_desc; }

private:
    nksim_result refresh_kinematic_bodies();
    nksim_result read_backend_state();
    nksim_result synchronize_scene(nkscene_change_set *out_changes);
    nksim_result node_pose(nkscene_node_id node,
                                 std::array<double, 3> &position,
                                 std::array<double, 4> &rotation) const;
    nksim_result set_backend_body_state(const Body &body);
    nksim_result set_backend_body_state(std::uint64_t backend_body,
                                        const nksim_body_state &state);
    void commit_kinematic_targets() noexcept;
    void initialize_body_state(Body &body, const std::array<double, 3> &position,
                               const std::array<double, 4> &rotation) noexcept;

    nksim_world_desc world_desc{};
    Clock clock;
    std::thread::id owner_thread;
    std::unique_ptr<PhysicsBackend> backend;
    bool topology_update_open = false;
    OwnedTable<Shape> shapes;
    OwnedTable<Body> bodies;
    OwnedTable<Joint> joints;
};

struct RuntimeRegistry {
    std::mutex mutex;
    SharedTable<World> worlds;
    SharedTable<Snapshot> snapshots;
    SharedTable<Host> hosts;
};

RuntimeRegistry &registry() noexcept;
std::shared_ptr<World> resolve_world(nksim_world world) noexcept;
std::shared_ptr<Snapshot> resolve_snapshot(nksim_snapshot snapshot) noexcept;
std::shared_ptr<Host> resolve_host(nksim_host host) noexcept;

NKSIM_API nksim_result create_world_with_backend(
    const nksim_world_desc &desc, std::unique_ptr<PhysicsBackend> backend,
    nksim_world *out_world);

bool valid_struct_size(std::uint32_t provided, std::size_t required) noexcept;
bool finite_positive(double value) noexcept;

} // namespace nksim
