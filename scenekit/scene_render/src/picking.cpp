#include "nativekit_scene_render.hpp"

namespace nkscene {

PickResult pick(const RenderPlan &plan, const SceneSnapshot &snapshot, std::uint32_t primitive,
                Vec3 world_position, float depth) {
    PickResult result;
    result.worldPosition = world_position;
    result.depth = depth;
    if (primitive >= plan.items().size())
        return result;
    const auto &item = plan.items()[primitive];
    const auto *node = snapshot.find(item.node);
    if (!node)
        return result;
    result.node = node->node;
    result.source = node->source;
    /* The first headless executor maps one render item to one pick primitive.
     * Geometry subelement tables are retained in the snapshot so this mapping
     * can become primitive-accurate without changing the public result type. */
    result.subelement = {1};
    return result;
}

} // namespace nkscene

namespace nkscene::render_internal {

/* Picking is introduced after the headless compiler and GPU executor. */

} // namespace nkscene::render_internal
