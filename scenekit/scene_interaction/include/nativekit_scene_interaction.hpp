#pragma once

/* ------------------------------------------------------------------------- */
/* Dependencies                                                              */
/* ------------------------------------------------------------------------- */

#include "nativekit_scene_interaction.h"
#include "nativekit_scene_render.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

/* ------------------------------------------------------------------------- */
/* C++ interaction API                                                       */
/* ------------------------------------------------------------------------- */

namespace nkscene {

enum class SelectionMode : std::uint32_t {
    Replace = NKS_INTERACTION_SELECTION_REPLACE,
    Add = NKS_INTERACTION_SELECTION_ADD,
    Toggle = NKS_INTERACTION_SELECTION_TOGGLE
};

enum class InteractionHoverState : std::uint32_t {
    Idle = NKS_INTERACTION_HOVER_IDLE,
    Pending = NKS_INTERACTION_HOVER_PENDING,
    Ready = NKS_INTERACTION_HOVER_READY,
    Stale = NKS_INTERACTION_HOVER_STALE,
    Failed = NKS_INTERACTION_HOVER_FAILED
};

/**
 * Backend-neutral hover and selection state for one scene presentation.
 *
 * The interaction object never mutates a Scene. The caller supplies the
 * current immutable snapshot and render plan when polling asynchronous hover.
 */
class NKSINTERACTION_API SceneInteraction {
  public:
    SceneInteraction() = default;
    ~SceneInteraction();
    SceneInteraction(SceneInteraction &&) noexcept;
    SceneInteraction &operator=(SceneInteraction &&) noexcept;
    SceneInteraction(const SceneInteraction &) = delete;
    SceneInteraction &operator=(const SceneInteraction &) = delete;

    nkgpu_result request_hover(NativeKitGpuExecutor &, const RenderPlan &, const SceneSnapshot &,
                               std::uint32_t width, std::uint32_t height, std::uint32_t x,
                               std::uint32_t y);
    InteractionHoverState poll_hover(NativeKitGpuExecutor &, const RenderPlan &,
                                     const SceneSnapshot &, nkgpu_result &out_error);
    void cancel_hover() noexcept;

    void apply_pick(const PickResult &, SelectionMode mode);
    void select(NodeId, SelectionMode mode);
    void clear_selection() noexcept;
    void synchronize(const SceneSnapshot &);

    std::optional<PickResult> hovered() const;
    std::span<const NodeId> selected() const noexcept;
    bool is_selected(NodeId) const noexcept;

  private:
    std::shared_ptr<GpuPickRequest> hover_request_;
    std::optional<PickResult> hovered_;
    std::vector<NodeId> selected_;
};

} // namespace nkscene
