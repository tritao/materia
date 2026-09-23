#include "nativekit_scene_interaction.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace nkscene {

namespace {

void apply_selection(std::vector<NodeId> &selected, NodeId node,
                     SelectionMode mode) {
    const auto found = std::find(selected.begin(), selected.end(), node);
    if (!node.valid()) {
        if (mode == SelectionMode::Replace)
            selected.clear();
        return;
    }

    switch (mode) {
    case SelectionMode::Replace:
        selected.clear();
        selected.push_back(node);
        break;
    case SelectionMode::Add:
        if (found == selected.end())
            selected.push_back(node);
        break;
    case SelectionMode::Toggle:
        if (found == selected.end())
            selected.push_back(node);
        else
            selected.erase(found);
        break;
    }
}

bool selection_mode(std::uint32_t value, SelectionMode &out_mode) noexcept {
    switch (value) {
    case NKS_INTERACTION_SELECTION_REPLACE:
        out_mode = SelectionMode::Replace;
        return true;
    case NKS_INTERACTION_SELECTION_ADD:
        out_mode = SelectionMode::Add;
        return true;
    case NKS_INTERACTION_SELECTION_TOGGLE:
        out_mode = SelectionMode::Toggle;
        return true;
    default:
        return false;
    }
}

} // namespace

SceneInteraction::~SceneInteraction() = default;
SceneInteraction::SceneInteraction(SceneInteraction &&) noexcept = default;
SceneInteraction &SceneInteraction::operator=(SceneInteraction &&) noexcept = default;

nkgpu_result SceneInteraction::request_hover(NativeKitGpuExecutor &executor, const RenderPlan &plan,
                                             const SceneSnapshot &snapshot, std::uint32_t width,
                                             std::uint32_t height, std::uint32_t x,
                                             std::uint32_t y) {
    hover_request_.reset();
    return executor.begin_pick_pixel(plan, snapshot, width, height, x, y, hover_request_);
}

InteractionHoverState SceneInteraction::poll_hover(NativeKitGpuExecutor &executor,
                                                   const RenderPlan &plan,
                                                   const SceneSnapshot &snapshot,
                                                   nkgpu_result &out_error) {
    out_error = NKGPU_OK;
    if (!hover_request_)
        return InteractionHoverState::Idle;

    PickResult result;
    const auto state =
        executor.poll_pick_pixel(*hover_request_, plan, snapshot, &result, &out_error);
    const auto hover_state = static_cast<InteractionHoverState>(state);
    if (hover_state == InteractionHoverState::Ready) {
        if (result.node.valid())
            hovered_ = result;
        else
            hovered_.reset();
        hover_request_.reset();
    } else if (hover_state == InteractionHoverState::Stale ||
               hover_state == InteractionHoverState::Failed) {
        hover_request_.reset();
    }
    return hover_state;
}

void SceneInteraction::cancel_hover() noexcept {
    hover_request_.reset();
}

void SceneInteraction::apply_pick(const PickResult &pick_result, SelectionMode mode) {
    if (pick_result.node.valid())
        hovered_ = pick_result;
    else
        hovered_.reset();
    apply_selection(selected_, pick_result.node, mode);
}

void SceneInteraction::select(NodeId node, SelectionMode mode) {
    apply_selection(selected_, node, mode);
}

void SceneInteraction::clear_selection() noexcept {
    selected_.clear();
}

void SceneInteraction::synchronize(const SceneSnapshot &snapshot) {
    std::erase_if(selected_, [&snapshot](NodeId node) {
        return snapshot.find(node) == nullptr;
    });
    if (hovered_ && snapshot.find(hovered_->node) == nullptr)
        hovered_.reset();
}

std::optional<PickResult> SceneInteraction::hovered() const {
    return hovered_;
}

std::span<const NodeId> SceneInteraction::selected() const noexcept {
    return selected_;
}

bool SceneInteraction::is_selected(NodeId node) const noexcept {
    return std::find(selected_.begin(), selected_.end(), node) != selected_.end();
}

} // namespace nkscene

namespace {

struct CInteractionState {
    nkscene_render_pick_request hover_request = 0;
    std::optional<nkscene_render_pick_result> hovered;
    std::vector<nkscene_node_id> selected;

    ~CInteractionState() {
        if (hover_request)
            nkscene_render_pick_request_destroy(hover_request);
    }
};

struct RuntimeHandle {
    std::uint32_t slot = 0;
    std::uint32_t generation = 0;

    bool valid() const noexcept { return generation != 0; }
};

constexpr std::uint32_t handle_index_bits = 20;
constexpr std::uint32_t handle_index_mask = (1u << handle_index_bits) - 1u;
constexpr std::uint32_t handle_max_generation = (1u << (32 - handle_index_bits)) - 1u;

std::uint32_t pack_handle(RuntimeHandle handle) noexcept {
    if (!handle.valid() || handle.slot >= handle_index_mask ||
        handle.generation > handle_max_generation)
        return 0;
    return (handle.generation << handle_index_bits) | (handle.slot + 1);
}

RuntimeHandle unpack_handle(std::uint32_t value) noexcept {
    if (value == 0 || (value & handle_index_mask) == 0)
        return {};
    return {static_cast<std::uint32_t>((value & handle_index_mask) - 1),
            static_cast<std::uint32_t>(value >> handle_index_bits)};
}

class InteractionHandles {
  public:
    RuntimeHandle create(std::shared_ptr<CInteractionState> value) {
        if (free_slots_.empty()) {
            if (slots_.size() >= handle_index_mask)
                return {};
            slots_.push_back({std::move(value), 1});
            return {static_cast<std::uint32_t>(slots_.size() - 1), 1};
        }
        const auto slot = free_slots_.back();
        free_slots_.pop_back();
        auto &entry = slots_[slot];
        if (entry.generation < handle_max_generation)
            ++entry.generation;
        entry.value = std::move(value);
        return {slot, entry.generation};
    }

    std::shared_ptr<CInteractionState> get(RuntimeHandle handle) const noexcept {
        if (!valid(handle))
            return {};
        return slots_[handle.slot].value;
    }

    std::shared_ptr<CInteractionState> remove(RuntimeHandle handle) noexcept {
        if (!valid(handle))
            return {};
        auto &entry = slots_[handle.slot];
        auto result = std::move(entry.value);
        if (entry.generation < handle_max_generation)
            free_slots_.push_back(handle.slot);
        return result;
    }

  private:
    struct Entry {
        std::shared_ptr<CInteractionState> value;
        std::uint32_t generation = 1;
    };

    bool valid(RuntimeHandle handle) const noexcept {
        return handle.valid() && handle.slot < slots_.size() &&
               slots_[handle.slot].generation == handle.generation &&
               slots_[handle.slot].value != nullptr;
    }

    std::vector<Entry> slots_;
    std::vector<std::uint32_t> free_slots_;
};

struct InteractionRegistry {
    std::mutex mutex;
    InteractionHandles interactions;
};

InteractionRegistry &registry() {
    static InteractionRegistry value;
    return value;
}

std::shared_ptr<CInteractionState> resolve(std::uint32_t handle) {
    auto &state = registry();
    std::lock_guard lock(state.mutex);
    return state.interactions.get(unpack_handle(handle));
}

void apply_selection(std::vector<nkscene_node_id> &selected, nkscene_node_id node,
                     std::uint32_t mode) {
    const auto found =
        std::find_if(selected.begin(), selected.end(), [node](nkscene_node_id value) {
            return value.value == node.value;
        });
    if (node.value == 0) {
        if (mode == NKS_INTERACTION_SELECTION_REPLACE)
            selected.clear();
        return;
    }

    switch (mode) {
    case NKS_INTERACTION_SELECTION_REPLACE:
        selected.clear();
        selected.push_back(node);
        break;
    case NKS_INTERACTION_SELECTION_ADD:
        if (found == selected.end())
            selected.push_back(node);
        break;
    case NKS_INTERACTION_SELECTION_TOGGLE:
        if (found == selected.end())
            selected.push_back(node);
        else
            selected.erase(found);
        break;
    default:
        break;
    }
}

} // namespace

extern "C" {

nkscene_result NKS_CALL nkscene_interaction_create(nkscene_interaction *out_interaction) {
    if (!out_interaction)
        return NKS_ERROR_INVALID_ARGUMENT;
    *out_interaction = 0;
    auto value = std::make_shared<CInteractionState>();
    auto &state = registry();
    std::lock_guard lock(state.mutex);
    const auto handle = state.interactions.create(std::move(value));
    if (!handle.valid())
        return NKS_ERROR_OUT_OF_MEMORY;
    *out_interaction = pack_handle(handle);
    return NKS_OK;
}

void NKS_CALL nkscene_interaction_destroy(nkscene_interaction interaction) {
    std::shared_ptr<CInteractionState> value;
    auto &state = registry();
    {
        std::lock_guard lock(state.mutex);
        value = state.interactions.remove(unpack_handle(interaction));
    }
}

nkscene_result NKS_CALL nkscene_interaction_request_hover(nkscene_interaction interaction,
                                                          nkscene_render_executor executor,
                                                          nkscene_render_plan plan,
                                                          nkscene_snapshot snapshot, uint32_t width,
                                                          uint32_t height, uint32_t x, uint32_t y) {
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    if (state->hover_request) {
        nkscene_render_pick_request_destroy(state->hover_request);
        state->hover_request = 0;
    }
    nkscene_render_pick_request request = 0;
    const auto result = nkscene_render_executor_pick_pixel_begin(executor, plan, snapshot, width,
                                                                 height, x, y, &request);
    if (result == NKS_OK)
        state->hover_request = request;
    return result;
}

nkscene_result NKS_CALL nkscene_interaction_poll_hover(nkscene_interaction interaction,
                                                       nkscene_render_executor executor,
                                                       nkscene_render_plan plan,
                                                       nkscene_snapshot snapshot,
                                                       uint32_t *out_state, nkgpu_result *out_error,
                                                       nkscene_render_pick_result *out_result) {
    if (!out_state || !out_error || !out_result)
        return NKS_ERROR_INVALID_ARGUMENT;
    *out_state = NKS_INTERACTION_HOVER_FAILED;
    *out_error = NKGPU_ERROR_INVALID_HANDLE;
    *out_result = {};
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    if (!state->hover_request) {
        *out_state = NKS_INTERACTION_HOVER_IDLE;
        *out_error = NKGPU_OK;
        return NKS_OK;
    }

    const auto result = nkscene_render_executor_pick_pixel_poll(
        executor, state->hover_request, plan, snapshot, out_state, out_error, out_result);
    if (result != NKS_OK) {
        nkscene_render_pick_request_destroy(state->hover_request);
        state->hover_request = 0;
        return result;
    }
    if (*out_state == NKS_INTERACTION_HOVER_READY) {
        if (out_result->node.value != 0)
            state->hovered = *out_result;
        else
            state->hovered.reset();
        nkscene_render_pick_request_destroy(state->hover_request);
        state->hover_request = 0;
    } else if (*out_state == NKS_INTERACTION_HOVER_STALE ||
               *out_state == NKS_INTERACTION_HOVER_FAILED) {
        nkscene_render_pick_request_destroy(state->hover_request);
        state->hover_request = 0;
    }
    return NKS_OK;
}

void NKS_CALL nkscene_interaction_cancel_hover(nkscene_interaction interaction) {
    const auto state = resolve(interaction);
    if (!state)
        return;
    if (state->hover_request)
        nkscene_render_pick_request_destroy(state->hover_request);
    state->hover_request = 0;
}

nkscene_result NKS_CALL nkscene_interaction_apply_pick(nkscene_interaction interaction,
                                                       const nkscene_render_pick_result *pick,
                                                       uint32_t mode) {
    if (!pick)
        return NKS_ERROR_INVALID_ARGUMENT;
    nkscene::SelectionMode ignored;
    if (!nkscene::selection_mode(mode, ignored))
        return NKS_ERROR_INVALID_ARGUMENT;
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    if (pick->node.value != 0)
        state->hovered = *pick;
    else
        state->hovered.reset();
    apply_selection(state->selected, pick->node, mode);
    return NKS_OK;
}

nkscene_result NKS_CALL nkscene_interaction_select(nkscene_interaction interaction,
                                                   nkscene_node_id node,
                                                   uint32_t mode) {
    nkscene::SelectionMode ignored;
    if (node.value == 0 || !nkscene::selection_mode(mode, ignored))
        return NKS_ERROR_INVALID_ARGUMENT;
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    apply_selection(state->selected, node, mode);
    return NKS_OK;
}

void NKS_CALL nkscene_interaction_clear_selection(nkscene_interaction interaction) {
    const auto state = resolve(interaction);
    if (state)
        state->selected.clear();
}

nkscene_result NKS_CALL nkscene_interaction_synchronize(nkscene_interaction interaction,
                                                        nkscene_snapshot snapshot) {
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    std::uint64_t count = 0;
    if (nkscene_snapshot_get_node_count(snapshot, &count) != NKS_OK)
        return NKS_ERROR_INVALID_HANDLE;
    std::unordered_set<std::uint64_t> live;
    live.reserve(static_cast<std::size_t>(count));
    for (std::uint64_t index = 0; index < count; ++index) {
        nkscene_snapshot_node value = {};
        value.struct_size = sizeof(value);
        if (nkscene_snapshot_get_node(snapshot, index, &value) != NKS_OK)
            return NKS_ERROR_INVALID_HANDLE;
        live.insert(value.node.value);
    }
    std::erase_if(state->selected, [&live](nkscene_node_id node) {
        return !live.contains(node.value);
    });
    if (state->hovered && !live.contains(state->hovered->node.value))
        state->hovered.reset();
    return NKS_OK;
}

nkscene_result NKS_CALL nkscene_interaction_get_hover(nkscene_interaction interaction,
                                                      uint32_t *out_has_hover,
                                                      nkscene_render_pick_result *out_result) {
    if (!out_has_hover || !out_result)
        return NKS_ERROR_INVALID_ARGUMENT;
    *out_has_hover = 0;
    *out_result = {};
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    if (state->hovered) {
        *out_has_hover = 1;
        *out_result = *state->hovered;
    }
    return NKS_OK;
}

nkscene_result NKS_CALL nkscene_interaction_get_selection_count(nkscene_interaction interaction,
                                                                uint64_t *out_count) {
    if (!out_count)
        return NKS_ERROR_INVALID_ARGUMENT;
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    *out_count = state->selected.size();
    return NKS_OK;
}

nkscene_result NKS_CALL nkscene_interaction_get_selected(nkscene_interaction interaction,
                                                         uint64_t index,
                                                         nkscene_node_id *out_node) {
    if (!out_node)
        return NKS_ERROR_INVALID_ARGUMENT;
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    if (index >= state->selected.size())
        return NKS_ERROR_INVALID_ARGUMENT;
    *out_node = state->selected[static_cast<std::size_t>(index)];
    return NKS_OK;
}

nkscene_result NKS_CALL nkscene_interaction_is_selected(nkscene_interaction interaction,
                                                        nkscene_node_id node,
                                                        uint32_t *out_selected) {
    if (!out_selected)
        return NKS_ERROR_INVALID_ARGUMENT;
    const auto state = resolve(interaction);
    if (!state)
        return NKS_ERROR_INVALID_HANDLE;
    *out_selected = std::any_of(state->selected.begin(), state->selected.end(),
                                [node](nkscene_node_id value) {
                                    return value.value == node.value;
                                })
                        ? 1u
                        : 0u;
    return NKS_OK;
}

} // extern "C"
