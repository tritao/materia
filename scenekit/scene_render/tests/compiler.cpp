#include "nativekit_scene_render.hpp"

#include "scene_internal.hpp"
#include "scene_shader_sources.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <memory>
#include <thread>
#include <vector>

namespace {

using nkscene::ChangeSet;
using nkscene::Scene;
using nkscene::Transaction;

void shader_sources_cover_backend_matrix() {
    struct BackendCase {
        nkgpu_backend backend;
        nkgpu_shader_language language;
    };
    const BackendCase backends[] = {
        {NKGPU_BACKEND_GLCORE, NKGPU_SHADERLANGUAGE_GLSL},
        {NKGPU_BACKEND_GLES3, NKGPU_SHADERLANGUAGE_GLSL},
        {NKGPU_BACKEND_D3D11, NKGPU_SHADERLANGUAGE_HLSL5},
        {NKGPU_BACKEND_METAL, NKGPU_SHADERLANGUAGE_MSL},
    };
    for (const auto &backend : backends) {
        const auto regular = nkscene::render_internal::scene_shader_sources(backend.backend, false);
        const auto picking = nkscene::render_internal::scene_shader_sources(backend.backend, true);
        assert(regular.vertex && regular.fragment && regular.language == backend.language);
        assert(picking.vertex && picking.fragment && picking.language == backend.language);
    }
    const auto web = nkscene::render_internal::scene_shader_sources(NKGPU_BACKEND_GLES3, false);
    assert(web.language == NKGPU_SHADERLANGUAGE_GLSL);
}

nkscene::LocalTransform translated(float x) {
    nkscene::LocalTransform transform;
    transform.matrix[12] = x;
    return transform;
}

void geometry_payload_contract_is_validated() {
    auto scene = std::make_shared<Scene>();
    const auto valid_geometry = scene->reserve_geometry_id();
    auto &valid = scene->geometry_store().create(valid_geometry);
    valid.edit_payload().vertices = {
        nkscene::GeometryVertex{{-1.0f, 0.0f, 0.0f}},
        nkscene::GeometryVertex{{1.0f, 0.0f, 0.0f}},
        nkscene::GeometryVertex{{0.0f, 1.0f, 0.0f}}};
    valid.edit_payload().indices = {0, 1, 2};
    valid.edit_subelements().ranges.push_back({0, 1, 7});
    assert(valid.payload->element_count() == 3);
    assert(valid.payload->indexed());
    assert(valid.subelements->id_for_primitive(0) == 7);

    const auto invalid_geometry = scene->reserve_geometry_id();
    auto &invalid = scene->geometry_store().create(invalid_geometry);
    invalid.edit_payload().vertices = valid.payload->vertices;
    invalid.edit_payload().indices = {0, 1, 3};

    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto node = scene->reserve_node_id();
    Transaction create(scene);
    create.add_create(node);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();
    Transaction configure(scene);
    configure.add_geometry(node, invalid_geometry);
    configure.add_material(node, material);
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const auto plan = nkscene::compile(scene->snapshot(), {});
    nkscene::NativeKitGpuExecutor executor;
    const auto stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_ERROR_INVALID_ARGUMENT);
}

void resource_lifecycle_is_cache_safe() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.edit_payload().vertices = {
        nkscene::GeometryVertex{{-1.0f, -1.0f, 0.0f}},
        nkscene::GeometryVertex{{1.0f, -1.0f, 0.0f}},
        nkscene::GeometryVertex{{0.0f, 1.0f, 0.0f}}};
    geometry_resource.edit_payload().indices = {0, 1, 2};

    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto node = scene->reserve_node_id();
    Transaction create(scene);
    create.add_create(node);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();
    Transaction configure(scene);
    configure.add_geometry(node, geometry);
    configure.add_material(node, material);
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const auto view = nkscene::SceneView{};
    auto plan = nkscene::compile(scene->snapshot(), view);
    ChangeSet no_changes;
    nkscene::NativeKitGpuExecutor executor;
    auto stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_OK);
    assert(stats.geometry_resources_created == 1);
    assert(stats.material_resources_created == 1);

    auto &non_indexed = scene->geometry_store().create(geometry);
    non_indexed.edit_payload().indices.clear();
    scene->publish();
    auto update = nkscene::update(plan, scene->snapshot(), no_changes, view);
    assert(!update.plan_rebuilt);
    assert(update.updated_geometry_resources == 1);
    assert(update.updated_material_resources == 0);
    stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_OK);
    assert(stats.geometry_resources_created == 0);
    assert(stats.geometry_resources_updated == 1);

    auto &indexed = scene->geometry_store().create(geometry);
    indexed.edit_payload().indices = {0, 1, 2};
    scene->publish();
    update = nkscene::update(plan, scene->snapshot(), no_changes, view);
    assert(!update.plan_rebuilt);
    assert(update.updated_geometry_resources == 1);
    assert(update.updated_material_resources == 0);
    stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_OK);
    assert(stats.geometry_resources_created == 0);
    assert(stats.geometry_resources_updated == 1);

    const auto vertices = indexed.payload->vertices;
    assert(scene->geometry_store().destroy(geometry));
    scene->publish();
    update = nkscene::update(plan, scene->snapshot(), no_changes, view);
    assert(update.plan_rebuilt);
    assert(update.invalidated_items == 1);
    assert(plan.items().empty());
    stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_OK);
    assert(stats.geometry_resources_created == 0);
    assert(stats.geometry_resources_updated == 0);

    auto &restored_geometry = scene->geometry_store().create(geometry);
    restored_geometry.edit_payload().vertices = vertices;
    restored_geometry.edit_payload().indices.clear();
    scene->publish();
    plan = nkscene::compile(scene->snapshot(), view);
    assert(plan.items().size() == 1);
    stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_OK);
    assert(stats.geometry_resources_created == 1);
    assert(stats.geometry_resources_updated == 0);

    scene->material_store().create(material);
    scene->publish();
    update = nkscene::update(plan, scene->snapshot(), no_changes, view);
    assert(!update.plan_rebuilt);
    assert(update.updated_geometry_resources == 0);
    assert(update.updated_material_resources == 1);
    stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_OK);
    assert(stats.material_resources_created == 0);
    assert(stats.material_resources_updated == 1);

    assert(scene->material_store().destroy(material));
    scene->publish();
    update = nkscene::update(plan, scene->snapshot(), no_changes, view);
    assert(update.plan_rebuilt);
    assert(update.invalidated_items == 1);
    assert(plan.items().empty());
    stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_OK);
    assert(stats.material_resources_created == 0);
    assert(stats.material_resources_updated == 0);

    scene->material_store().create(material);
    scene->publish();
    plan = nkscene::compile(scene->snapshot(), view);
    stats = executor.execute(plan, scene->snapshot());
    assert(stats.result == NKGPU_OK);
    assert(stats.material_resources_created == 1);
    assert(stats.material_resources_updated == 0);

    const auto missing_geometry = scene->reserve_geometry_id();
    Transaction invalid_geometry(scene);
    invalid_geometry.add_geometry(node, missing_geometry);
    assert(scene->commit(invalid_geometry, changes) == NKS_OK);
    invalid_geometry.close();
    update = nkscene::update(plan, scene->snapshot(), changes, view);
    assert(update.plan_rebuilt);
    assert(update.invalidated_items == 1);
    assert(plan.items().empty());

    Transaction restore_geometry(scene);
    restore_geometry.add_geometry(node, geometry);
    assert(scene->commit(restore_geometry, changes) == NKS_OK);
    restore_geometry.close();
    plan = nkscene::compile(scene->snapshot(), view);
    assert(plan.items().size() == 1);

    const auto missing_material = scene->reserve_material_id();
    Transaction invalid_material(scene);
    invalid_material.add_material(node, missing_material);
    assert(scene->commit(invalid_material, changes) == NKS_OK);
    invalid_material.close();
    update = nkscene::update(plan, scene->snapshot(), changes, view);
    assert(update.plan_rebuilt);
    assert(update.invalidated_items == 1);
    assert(plan.items().empty());
}

void scene_views_are_hierarchy_aware() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    scene->geometry_store().create(geometry);
    const auto material_one = scene->reserve_material_id();
    const auto material_two = scene->reserve_material_id();
    scene->material_store().create(material_one);
    scene->material_store().create(material_two);

    const auto group = scene->reserve_node_id();
    const auto leaf = scene->reserve_node_id();
    const auto sibling = scene->reserve_node_id();
    Transaction create(scene);
    create.add_create(group);
    create.add_create(leaf);
    create.add_create(sibling);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    configure.add_parent(leaf, group);
    configure.add_geometry(leaf, geometry);
    configure.add_material(leaf, material_one);
    configure.add_geometry(sibling, geometry);
    configure.add_material(sibling, material_one);
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const auto before = scene->snapshot();
    const auto find_item = [](const nkscene::RenderPlan &plan,
                              nkscene::NodeId node) {
        return std::find_if(plan.items().begin(), plan.items().end(),
                            [node](const nkscene::RenderItem &item) {
                                return item.node == node;
                            });
    };

    nkscene::SceneView full_view;
    auto plan = nkscene::compile(before, full_view);
    assert(plan.items().size() == 2);
    assert(find_item(plan, leaf) != plan.items().end());
    assert(!nkscene::has_render_flag(find_item(plan, leaf)->flags,
                                     nkscene::RenderFlags::Hidden));

    nkscene::SceneView subtree_view;
    subtree_view.root = group;
    plan = nkscene::compile(before, subtree_view);
    assert(plan.items().size() == 1);
    assert(plan.items().front().node == leaf);

    nkscene::SceneView hidden_view = subtree_view;
    hidden_view.visibility_overrides.push_back({group, false});
    plan = nkscene::compile(before, hidden_view);
    assert(plan.items().size() == 1);
    assert(nkscene::has_render_flag(plan.items().front().flags,
                                    nkscene::RenderFlags::Hidden));

    nkscene::SceneView material_view = subtree_view;
    material_view.material_overrides.push_back({leaf, material_two});
    plan = nkscene::compile(before, material_view);
    assert(plan.items().front().material == material_two);
    assert(before.find(leaf)->material == material_one);
    assert(before.revision() == scene->revision());
    assert(scene->revision_counters().material == before.revisions().material);
    assert(scene->material_store().find(material_two)->revision == 1);

    nkscene::SceneView composed_view = full_view;
    composed_view.visibility_overrides.push_back({leaf, false});
    composed_view.material_overrides.push_back({leaf, material_two});
    plan = nkscene::compile(before, full_view);
    const auto composed_update = nkscene::refresh(plan, before, composed_view);
    assert(!composed_update.plan_rebuilt);
    assert(composed_update.patched_visibility == 1);
    assert(composed_update.patched_materials == 1);
    assert(composed_update.patched_culling == 0);
    assert(plan.items().size() == 2);
    assert(nkscene::has_render_flag(find_item(plan, leaf)->flags,
                                    nkscene::RenderFlags::Hidden));
    assert(find_item(plan, leaf)->material == material_two);

    plan = nkscene::compile(before, full_view);
    Transaction hide_parent(scene);
    hide_parent.add_visibility(group, false);
    assert(scene->commit(hide_parent, changes) == NKS_OK);
    hide_parent.close();
    const auto hidden_snapshot = scene->snapshot();
    auto update = nkscene::update(plan, hidden_snapshot, changes, full_view);
    assert(!update.plan_rebuilt);
    assert(update.patched_visibility == 1);
    assert(nkscene::has_render_flag(find_item(plan, leaf)->flags,
                                    nkscene::RenderFlags::Hidden));

    nkscene::SceneView inspection_view;
    inspection_view.include_invisible = true;
    plan = nkscene::compile(hidden_snapshot, inspection_view);
    assert(!nkscene::has_render_flag(find_item(plan, leaf)->flags,
                                     nkscene::RenderFlags::Hidden));

    ChangeSet no_changes;
    update = nkscene::update(plan, hidden_snapshot, no_changes, subtree_view);
    assert(update.plan_rebuilt);
    assert(plan.items().size() == 1);
    assert(plan.items().front().node == leaf);

    plan = nkscene::compile(hidden_snapshot, full_view);
    Transaction reparent_leaf(scene);
    reparent_leaf.add_parent(leaf, sibling);
    assert(scene->commit(reparent_leaf, changes) == NKS_OK);
    reparent_leaf.close();
    const auto reparented_snapshot = scene->snapshot();
    update = nkscene::update(plan, reparented_snapshot, changes, full_view);
    assert(!update.plan_rebuilt);
    assert(update.patched_visibility == 1);
    assert(!nkscene::has_render_flag(find_item(plan, leaf)->flags,
                                     nkscene::RenderFlags::Hidden));

    nkscene::SceneView hide_new_parent = full_view;
    hide_new_parent.visibility_overrides.push_back({sibling, false});
    update = nkscene::refresh(plan, reparented_snapshot, hide_new_parent);
    assert(!update.plan_rebuilt);
    assert(update.patched_visibility == 2);
    assert(nkscene::has_render_flag(find_item(plan, sibling)->flags,
                                    nkscene::RenderFlags::Hidden));
    assert(nkscene::has_render_flag(find_item(plan, leaf)->flags,
                                    nkscene::RenderFlags::Hidden));
}

void scene_view_source_filters_are_incremental() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    scene->geometry_store().create(geometry);
    const auto material_one = scene->reserve_material_id();
    const auto material_two = scene->reserve_material_id();
    scene->material_store().create(material_one);
    scene->material_store().create(material_two);

    const auto first = scene->reserve_node_id();
    const auto second = scene->reserve_node_id();
    Transaction create(scene);
    create.add_create(first);
    create.add_create(second);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    configure.add_geometry(first, geometry);
    configure.add_material(first, material_one);
    configure.add_source_entity(first, nkscene::EntityId{42});
    configure.add_geometry(second, geometry);
    configure.add_material(second, material_one);
    configure.add_source_entity(second, nkscene::EntityId{84});
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const auto snapshot = scene->snapshot();
    nkscene::SceneView base_view;
    auto plan = nkscene::compile(snapshot, base_view);
    assert(plan.item_index(first) != static_cast<std::size_t>(-1));
    assert(plan.items_for_source(nkscene::EntityId{42}).size() == 1);
    assert(plan.items_for_source(nkscene::EntityId{84}).size() == 1);
    nkscene::SceneView source_view = base_view;
    source_view.filter.source_visibility_overrides.push_back(
        {nkscene::EntityId{84}, false});
    source_view.filter.source_material_overrides.push_back(
        {nkscene::EntityId{42}, material_two});
    auto filter_update = nkscene::refresh(plan, snapshot, source_view);
    assert(!filter_update.plan_rebuilt);
    assert(filter_update.patched_visibility == 1);
    assert(filter_update.patched_materials == 1);
    assert(filter_update.updated_geometry_resources == 0);
    assert(filter_update.updated_material_resources == 0);

    nkscene::SceneView composed_view = source_view;
    composed_view.visibility_overrides.push_back({second, true});
    composed_view.material_overrides.push_back({first, material_one});
    auto composed_plan = nkscene::compile(snapshot, base_view);
    const auto composed_update = nkscene::refresh(composed_plan, snapshot, composed_view);
    assert(!composed_update.plan_rebuilt);
    assert(composed_update.patched_visibility == 0);
    assert(composed_update.patched_materials == 0);
    const auto composed_first_item = std::find_if(
        composed_plan.items().begin(), composed_plan.items().end(),
        [first](const nkscene::RenderItem &item) { return item.node == first; });
    const auto composed_second_item = std::find_if(
        composed_plan.items().begin(), composed_plan.items().end(),
        [second](const nkscene::RenderItem &item) { return item.node == second; });
    assert(composed_first_item != composed_plan.items().end());
    assert(composed_second_item != composed_plan.items().end());
    assert(composed_first_item->material == material_one);
    assert(!nkscene::has_render_flag(composed_second_item->flags,
                                     nkscene::RenderFlags::Hidden));

    Transaction change_source(scene);
    change_source.add_source_entity(second, nkscene::EntityId{42});
    assert(scene->commit(change_source, changes) == NKS_OK);
    change_source.close();
    const auto changed_snapshot = scene->snapshot();
    const auto source_update = nkscene::update(plan, changed_snapshot, changes, source_view);
    assert(!source_update.plan_rebuilt);
    assert(source_update.patched_visibility == 1);
    assert(source_update.patched_materials == 1);
    assert(source_update.updated_geometry_resources == 0);
    assert(source_update.updated_material_resources == 0);

    const auto find_item = [&](nkscene::NodeId node) {
        return std::find_if(plan.items().begin(), plan.items().end(),
                            [node](const nkscene::RenderItem &item) {
                                return item.node == node;
                            });
    };
    const auto second_item = find_item(second);
    assert(second_item != plan.items().end());
    assert(second_item->material == material_two);
    assert(!nkscene::has_render_flag(second_item->flags,
                                     nkscene::RenderFlags::Hidden));

    nkscene::SceneView first_isolation;
    first_isolation.filter.isolated_sources.push_back(nkscene::EntityId{42});
    auto isolation_plan = nkscene::compile(snapshot, nkscene::SceneView{});
    auto isolation_update = nkscene::refresh(isolation_plan, snapshot, first_isolation);
    assert(!isolation_update.plan_rebuilt);
    assert(isolation_update.patched_visibility == 1);
    auto second_isolation = first_isolation;
    second_isolation.filter.isolated_sources = {nkscene::EntityId{84}};
    isolation_update = nkscene::refresh(isolation_plan, snapshot, second_isolation);
    assert(!isolation_update.plan_rebuilt);
    assert(isolation_update.patched_visibility == 2);
    assert(isolation_plan.visible_items() == 1);
}

void scene_view_camera_culling_is_incremental() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.25f, -0.25f, -0.25f};
    geometry_resource.bounds.maximum = {0.25f, 0.25f, 0.25f};
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto inside = scene->reserve_node_id();
    const auto outside = scene->reserve_node_id();

    Transaction create(scene);
    create.add_create(inside);
    create.add_create(outside);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    configure.add_geometry(inside, geometry);
    configure.add_material(inside, material);
    configure.add_geometry(outside, geometry);
    configure.add_material(outside, material);
    configure.add_transform(outside, translated(2.0f));
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    nkscene::SceneView view;
    view.camera.enabled = true;
    const auto snapshot = scene->snapshot();
    auto plan = nkscene::compile(snapshot, view);
    assert(plan.items().size() == 2);
    assert(plan.visible_items() == 1);
    assert(plan.culled_items() == 1);
    const auto find_item = [&](nkscene::NodeId id) {
        return std::find_if(plan.items().begin(), plan.items().end(),
                            [id](const nkscene::RenderItem &item) {
                                return item.node == id;
                            });
    };
    assert(!nkscene::has_render_flag(find_item(inside)->flags,
                                     nkscene::RenderFlags::Culled));
    assert(nkscene::has_render_flag(find_item(outside)->flags,
                                    nkscene::RenderFlags::Culled));
    const auto compile_count = plan.compile_count();

    Transaction move_inside(scene);
    move_inside.add_transform(outside, translated(0.5f));
    assert(scene->commit(move_inside, changes) == NKS_OK);
    move_inside.close();
    const auto moved_snapshot = scene->snapshot();
    const auto update = nkscene::update(plan, moved_snapshot, changes, view);
    assert(!update.plan_rebuilt);
    assert(update.patched_instances == 1);
    assert(update.patched_culling == 1);
    assert(update.visible_items == 2);
    assert(update.culled_items == 0);
    assert(plan.compile_count() == compile_count);
    assert(!nkscene::has_render_flag(find_item(outside)->flags,
                                     nkscene::RenderFlags::Culled));
}

void scene_resource_camera_is_used_for_render_view() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.edit_payload().vertices = {
        {{{-0.25f, -0.25f, -0.25f}}},
        {{{0.25f, -0.25f, -0.25f}}},
        {{{0.0f, 0.25f, 0.25f}}}};
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.25f, -0.25f, -0.25f};
    geometry_resource.bounds.maximum = {0.25f, 0.25f, 0.25f};
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto camera = scene->reserve_camera_id();
    auto &camera_resource = scene->camera_store().create(camera);
    camera_resource.fov_y = 1.0f;
    camera_resource.near_plane = 0.1f;
    camera_resource.far_plane = 10.0f;
    camera_resource.aspect_ratio = 1.0f;
    const auto camera_node = scene->reserve_node_id();
    const auto visible = scene->reserve_node_id();
    const auto hidden = scene->reserve_node_id();

    Transaction create(scene);
    create.add_create(camera_node);
    create.add_create(visible);
    create.add_create(hidden);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    configure.add_camera(camera_node, camera);
    configure.add_geometry(visible, geometry);
    configure.add_material(visible, material);
    configure.add_geometry(hidden, geometry);
    configure.add_material(hidden, material);
    configure.add_transform(visible, translated(2.0f));
    configure.add_transform(hidden, translated(-2.0f));
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    nkscene::SceneView view;
    view.camera_node = camera_node;
    const auto snapshot = scene->snapshot();
    const auto plan = nkscene::compile(snapshot, view);
    assert(plan.view_projection() != nkscene::SceneCamera{}.view_projection);
    assert(plan.camera().has_view_pose);
    assert(!plan.camera().orthographic);
    assert(plan.camera().position == (std::array<float, 3>{0.0f, 0.0f, 0.0f}));
    assert(plan.camera().view_direction == (std::array<float, 3>{1.0f, 0.0f, 0.0f}));
    const auto find_item = [&](nkscene::NodeId id) {
        return std::find_if(plan.items().begin(), plan.items().end(),
                            [id](const nkscene::RenderItem &item) {
                                return item.node == id;
                            });
    };
    assert(!nkscene::has_render_flag(find_item(visible)->flags,
                                     nkscene::RenderFlags::Culled));
    assert(nkscene::has_render_flag(find_item(hidden)->flags,
                                    nkscene::RenderFlags::Culled));
    camera_resource.projection = nkscene::CameraProjection::Orthographic;
    scene->publish();
    auto refreshed_plan = plan;
    const auto camera_refresh = nkscene::refresh(refreshed_plan, scene->snapshot(), view);
    assert(camera_refresh.plan_rebuilt);
    assert(refreshed_plan.camera().orthographic);
    const auto orthographic_plan = nkscene::compile(scene->snapshot(), view);
    assert(orthographic_plan.camera().has_view_pose);
    assert(orthographic_plan.camera().orthographic);
    Transaction move_camera(scene);
    move_camera.add_transform(camera_node, translated(3.0f));
    assert(scene->commit(move_camera, changes) == NKS_OK);
    move_camera.close();
    const auto moved_plan = nkscene::compile(scene->snapshot(), view);
    assert(moved_plan.camera().position == (std::array<float, 3>{3.0f, 0.0f, 0.0f}));
}

void scene_view_clip_planes_are_incremental() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.25f, -0.25f, -0.25f};
    geometry_resource.bounds.maximum = {0.25f, 0.25f, 0.25f};
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto inside = scene->reserve_node_id();
    const auto outside = scene->reserve_node_id();

    Transaction create(scene);
    create.add_create(inside);
    create.add_create(outside);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    configure.add_geometry(inside, geometry);
    configure.add_material(inside, material);
    configure.add_geometry(outside, geometry);
    configure.add_material(outside, material);
    configure.add_transform(outside, translated(2.0f));
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    nkscene::SceneView clipped_view;
    clipped_view.clip_planes.push_back({{-1.0f, 0.0f, 0.0f}, 0.0f, true});
    const auto snapshot = scene->snapshot();
    auto plan = nkscene::compile(snapshot, clipped_view);
    assert(plan.items().size() == 2);
    assert(plan.visible_items() == 1);
    assert(plan.culled_items() == 1);
    const auto compile_count = plan.compile_count();

    auto relaxed_view = clipped_view;
    relaxed_view.clip_planes.front().distance = 2.0f;
    const auto update = nkscene::refresh(plan, snapshot, relaxed_view);
    assert(!update.plan_rebuilt);
    assert(update.patched_culling == 1);
    assert(update.visible_items == 2);
    assert(update.culled_items == 0);
    assert(plan.compile_count() == compile_count);

    relaxed_view.clip_planes.clear();
    const auto cleared = nkscene::refresh(plan, snapshot, relaxed_view);
    assert(!cleared.plan_rebuilt);
    assert(cleared.patched_culling == 0);
    assert(cleared.visible_items == 2);

    auto disabled_view = relaxed_view;
    disabled_view.clip_planes.push_back({{1.0f, 0.0f, 0.0f}, -100.0f, false});
    const auto disabled = nkscene::refresh(plan, snapshot, disabled_view);
    assert(!disabled.plan_rebuilt);
    assert(disabled.patched_culling == 0);
    assert(disabled.visible_items == 2);
    assert(plan.clip_planes().empty());

    disabled_view.clip_planes.push_back({{1.0f, 0.0f, 0.0f}, -1.0f, true});
    const auto multiple = nkscene::refresh(plan, snapshot, disabled_view);
    assert(!multiple.plan_rebuilt);
    assert(multiple.patched_culling == 1);
    assert(multiple.visible_items == 1);
    assert(multiple.culled_items == 1);
    assert(plan.clip_planes().size() == 1);

    disabled_view.clip_planes.push_back({{0.0f, 1.0f, 0.0f}, -1.0f, true});
    const auto second_multiple = nkscene::refresh(plan, snapshot, disabled_view);
    assert(!second_multiple.plan_rebuilt);
    assert(second_multiple.patched_culling == 1);
    assert(second_multiple.visible_items == 0);
    assert(second_multiple.culled_items == 2);
    assert(plan.clip_planes().size() == 2);
}

void scene_view_culling_uses_spatial_candidates() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.25f, -0.25f, -0.25f};
    geometry_resource.bounds.maximum = {0.25f, 0.25f, 0.25f};
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);

    std::vector<nkscene::NodeId> nodes;
    nodes.reserve(100);
    Transaction create(scene);
    for (std::size_t index = 0; index < 100; ++index) {
        const auto node = scene->reserve_node_id();
        nodes.push_back(node);
        create.add_create(node);
    }
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        configure.add_geometry(nodes[index], geometry);
        configure.add_material(nodes[index], material);
        configure.add_transform(nodes[index], translated(static_cast<float>(index * 4)));
    }
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    nkscene::SceneView first_view;
    first_view.clip_planes.push_back({{1.0f, 0.0f, 0.0f}, -300.0f, true});
    const auto snapshot = scene->snapshot();
    auto plan = nkscene::compile(snapshot, first_view);
    assert(plan.items().size() == nodes.size());

    auto second_view = first_view;
    second_view.clip_planes.front() = {{-1.0f, 0.0f, 0.0f}, 10.0f, true};
    const auto update = nkscene::refresh(plan, snapshot, second_view);
    assert(!update.plan_rebuilt);
    assert(update.culling_candidates < plan.items().size());
    assert(update.patched_culling > 0);
    assert(update.culling_candidates == 28);
    assert(update.visible_items == 3);
}

void mixed_hierarchy_and_empty_batches_remain_incremental() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.5f, -0.5f, -0.5f};
    geometry_resource.bounds.maximum = {0.5f, 0.5f, 0.5f};
    const auto material_one = scene->reserve_material_id();
    const auto material_two = scene->reserve_material_id();
    scene->material_store().create(material_one);
    scene->material_store().create(material_two);
    const auto root = scene->reserve_node_id();
    const auto group = scene->reserve_node_id();
    const auto first = scene->reserve_node_id();
    const auto second = scene->reserve_node_id();

    Transaction create(scene);
    create.add_create(root);
    create.add_create(group);
    create.add_create(first);
    create.add_create(second);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();
    Transaction configure(scene);
    configure.add_parent(group, root);
    configure.add_parent(first, group);
    configure.add_parent(second, group);
    configure.add_geometry(first, geometry);
    configure.add_material(first, material_one);
    configure.add_geometry(second, geometry);
    configure.add_material(second, material_two);
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const auto snapshot = scene->snapshot();
    nkscene::SceneView base_view;
    auto plan = nkscene::compile(snapshot, base_view);
    assert(plan.items().size() == 2);
    assert(plan.batches().size() == 2);

    Transaction merge_batch(scene);
    merge_batch.add_material(first, material_two);
    assert(scene->commit(merge_batch, changes) == NKS_OK);
    merge_batch.close();
    auto merged = nkscene::update(plan, scene->snapshot(), changes, base_view);
    assert(!merged.plan_rebuilt);
    assert(plan.batches().size() == 1);
    assert(plan.batches().front().instances.size() == 2);

    Transaction split_batch(scene);
    split_batch.add_material(first, material_one);
    assert(scene->commit(split_batch, changes) == NKS_OK);
    split_batch.close();
    auto split = nkscene::update(plan, scene->snapshot(), changes, base_view);
    assert(!split.plan_rebuilt);
    assert(plan.batches().size() == 2);
    assert(plan.batches()[0].instances.size() == 1);
    assert(plan.batches()[1].instances.size() == 1);

    nkscene::SceneView hidden_view = base_view;
    hidden_view.visibility_overrides.push_back({root, false});
    const auto hidden = nkscene::refresh(plan, scene->snapshot(), hidden_view);
    assert(!hidden.plan_rebuilt);
    assert(hidden.patched_visibility == 2);
    assert(plan.visible_items() == 0);
    for (const auto &item : plan.items())
        assert(nkscene::has_render_flag(item.flags, nkscene::RenderFlags::Hidden));
}

void spatial_queries_and_cpu_picking_are_snapshot_bound() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.5f, -0.5f, 0.0f};
    geometry_resource.bounds.maximum = {0.5f, 0.5f, 0.0f};
    geometry_resource.edit_payload().vertices = {
        nkscene::GeometryVertex{{-0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.0f, 0.5f, 0.0f}}};
    geometry_resource.edit_payload().indices = {0, 1, 2};
    geometry_resource.edit_subelements().ranges.push_back({0, 1, 42});
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);

    const auto first = scene->reserve_node_id();
    const auto second = scene->reserve_node_id();
    const auto hidden = scene->reserve_node_id();
    Transaction create(scene);
    create.add_create(first);
    create.add_create(second);
    create.add_create(hidden);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    configure.add_geometry(first, geometry);
    configure.add_material(first, material);
    configure.add_source_entity(first, nkscene::EntityId{42});
    configure.add_transform(first, translated(-2.0f));
    configure.add_geometry(second, geometry);
    configure.add_material(second, material);
    configure.add_source_entity(second, nkscene::EntityId{84});
    configure.add_transform(second, translated(2.0f));
    configure.add_geometry(hidden, geometry);
    configure.add_material(hidden, material);
    configure.add_source_entity(hidden, nkscene::EntityId{126});
    configure.add_visibility(hidden, false);
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const auto snapshot = scene->snapshot();
    nkscene::SceneSpatialIndex index(snapshot);
    assert(index.source_revision() == snapshot.revision());

    nkscene::Bounds left_bounds;
    left_bounds.valid = true;
    left_bounds.minimum = {-3.0f, -1.0f, -1.0f};
    left_bounds.maximum = {-1.0f, 1.0f, 1.0f};
    const auto left = index.query_bounds(left_bounds);
    assert(left.size() == 1);
    assert(left.front() == first);

    const std::array<std::array<float, 4>, 1> left_plane = {{{-1.0f, 0.0f, 0.0f, -1.0f}}};
    const auto frustum_left = index.query_frustum(left_plane);
    assert(frustum_left.size() == 1);
    assert(frustum_left.front() == first);

    nkscene::Bounds all_bounds;
    all_bounds.valid = true;
    all_bounds.minimum = {-3.0f, -1.0f, -1.0f};
    all_bounds.maximum = {3.0f, 1.0f, 1.0f};
    const auto all = index.query_bounds(all_bounds);
    assert(all.size() == 3);
    assert(all[0] == first);
    assert(all[1] == second);
    assert(all[2] == hidden);

    const nkscene::Ray left_ray{{-2.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
    const auto ray_candidates = index.query_ray(left_ray);
    assert(ray_candidates.size() == 1);
    assert(ray_candidates.front() == first);
    const auto left_pick = index.pick_ray(left_ray);
    assert(left_pick.node == first);
    assert(left_pick.source == nkscene::EntityId{42});
    assert(left_pick.subelement.value == 42);
    assert(left_pick.worldPosition.x == -2.0f);
    assert(left_pick.worldPosition.z == 0.0f);
    assert(left_pick.depth == 5.0f);

    const nkscene::Ray hidden_ray{{0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
    assert(index.query_ray(hidden_ray).size() == 1);
    assert(!index.pick_ray(hidden_ray).node.valid());
}

void tagged_strokes_pick_only_when_visible() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &resource = scene->geometry_store().create(geometry);
    resource.bounds.valid = true;
    resource.bounds.minimum = {-1.0f, -1.0f, -1.0f};
    resource.bounds.maximum = {1.0f, 1.0f, 0.0f};
    auto &payload = resource.edit_payload();
    payload.vertices = {{{-1.0f, -1.0f, 0.0f}}, {{1.0f, -1.0f, 0.0f}},
                        {{1.0f, 1.0f, 0.0f}}, {{-1.0f, -1.0f, 0.0f}},
                        {{1.0f, 1.0f, 0.0f}}, {{-1.0f, 1.0f, 0.0f}}};
    payload.stroke_segments.push_back({{-0.8f, -0.8f, 0.0f}, {0.8f, -0.8f, 0.0f},
                                       0x40000001u});
    payload.stroke_segments.push_back({{-0.8f, 0.0f, -1.0f}, {0.8f, 0.0f, -1.0f},
                                       0x40000002u});
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto node = scene->reserve_node_id();
    Transaction transaction(scene);
    transaction.add_create(node);
    ChangeSet changes;
    assert(scene->commit(transaction, changes) == NKS_OK);
    transaction.close();
    Transaction configure(scene);
    configure.add_geometry(node, geometry);
    configure.add_material(node, material);
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();
    nkscene::SceneSpatialIndex index(scene->snapshot());
    nkscene::SceneView view;
    const auto edge = index.pick_ray_with_edges({{0.0f, -0.8f, 5.0f}, {0.0f, 0.0f, -1.0f}},
                                                view, 0.01f);
    assert(edge.node == node && edge.subelement.value == 0x40000001u);
    const auto face = index.pick_ray_with_edges({{0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}},
                                                view, 0.01f);
    assert(face.node == node && face.subelement.value != 0x40000002u);
}

void runtime_pose_overrides_render_and_pick_without_mutating_snapshot() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.5f, -0.5f, 0.0f};
    geometry_resource.bounds.maximum = {0.5f, 0.5f, 0.0f};
    geometry_resource.edit_payload().vertices = {
        nkscene::GeometryVertex{{-0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.0f, 0.5f, 0.0f}}};
    geometry_resource.edit_payload().indices = {0, 1, 2};
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto node = scene->reserve_node_id();

    Transaction create(scene);
    create.add_create(node);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    configure.add_geometry(node, geometry);
    configure.add_material(node, material);
    configure.add_transform(node, translated(-4.0f));
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const auto snapshot = scene->snapshot();
    const auto *snapshot_node = snapshot.find(node);
    assert(snapshot_node);
    assert(snapshot_node->world_transform.transform.matrix[12] == -4.0f);

    nkscene::SceneView view;
    view.pose_overrides.push_back({node, translated(4.0f)});
    const auto plan = nkscene::compile(snapshot, view);
    assert(plan.items().size() == 1);
    const auto &item = plan.items().front();
    assert(plan.transforms()[item.transformIndex].transform.matrix[12] == 4.0f);

    const nkscene::Ray design_ray{{-4.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
    const nkscene::Ray runtime_ray{{4.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
    nkscene::SceneSpatialIndex design_index(snapshot);
    nkscene::SceneSpatialIndex runtime_index(snapshot, &view);
    assert(design_index.pick_ray(design_ray).node == node);
    assert(!design_index.pick_ray(runtime_ray).node.valid());
    assert(!runtime_index.pick_ray(design_ray).node.valid());
    assert(runtime_index.pick_ray(runtime_ray).node == node);

    // Presentation poses affect consumers of the view, while the immutable
    // scene snapshot continues to describe the authored transform.
    assert(snapshot.find(node)->world_transform.transform.matrix[12] == -4.0f);
}

void geometry_edits_refresh_bounds_and_picking_for_new_snapshots() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.5f, -0.5f, 0.0f};
    geometry_resource.bounds.maximum = {0.5f, 0.5f, 0.0f};
    geometry_resource.edit_payload().vertices = {
        nkscene::GeometryVertex{{-0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.0f, 0.5f, 0.0f}}};
    geometry_resource.edit_payload().indices = {0, 1, 2};
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto node = scene->reserve_node_id();

    Transaction create(scene);
    create.add_create(node);
    create.add_geometry(node, geometry);
    create.add_material(node, material);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();
    const auto before = scene->snapshot();
    nkscene::SceneSpatialIndex before_index(before);

    auto &edited_geometry = scene->geometry_store().create(geometry);
    for (auto &vertex : edited_geometry.edit_payload().vertices)
        vertex.position[0] += 5.0f;
    edited_geometry.bounds.minimum[0] = 4.5f;
    edited_geometry.bounds.maximum[0] = 5.5f;
    scene->publish();

    const auto after = scene->snapshot();
    assert(after.find(node)->bounds.minimum[0] == 4.5f);
    assert(after.find(node)->bounds.maximum[0] == 5.5f);
    assert(before.find(node)->bounds.minimum[0] == -0.5f);
    assert(before.find(node)->bounds.maximum[0] == 0.5f);

    nkscene::SceneSpatialIndex after_index(after);
    const nkscene::Ray old_ray{{0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
    const nkscene::Ray moved_ray{{5.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
    assert(before_index.pick_ray(old_ray).node == node);
    assert(!before_index.pick_ray(moved_ray).node.valid());
    assert(!after_index.pick_ray(old_ray).node.valid());
    assert(after_index.pick_ray(moved_ray).node == node);
}

void spatial_index_refits_transform_changes_and_requests_rebuilds() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.5f, -0.5f, -0.5f};
    geometry_resource.bounds.maximum = {0.5f, 0.5f, 0.5f};
    geometry_resource.edit_payload().vertices = {
        nkscene::GeometryVertex{{-0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.0f, 0.5f, 0.0f}}};
    geometry_resource.edit_payload().indices = {0, 1, 2};
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);
    const auto node = scene->reserve_node_id();

    Transaction create(scene);
    create.add_create(node);
    create.add_geometry(node, geometry);
    create.add_material(node, material);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    const auto original = scene->snapshot();
    nkscene::SceneSpatialIndex index(original);

    const auto added = scene->reserve_node_id();
    Transaction add(scene);
    add.add_create(added);
    assert(scene->commit(add, changes) == NKS_OK);
    add.close();
    const auto structural = scene->snapshot();
    assert(!index.update_node_bounds(structural, node));
    assert(index.source_revision() == original.revision());

    nkscene::SceneSpatialIndex refitted(structural);
    for (std::uint32_t update = 1; update <= 128; ++update) {
        Transaction move(scene);
        move.add_transform(node, translated(static_cast<float>(update)));
        assert(scene->commit(move, changes) == NKS_OK);
        move.close();

        const auto current = scene->snapshot();
        assert(refitted.update_node_bounds(current, node));
        assert(refitted.source_revision() == current.revision());
    }

    nkscene::Bounds old_bounds;
    old_bounds.valid = true;
    old_bounds.minimum = {-1.0f, -1.0f, -1.0f};
    old_bounds.maximum = {1.0f, 1.0f, 1.0f};
    assert(refitted.query_bounds(old_bounds).empty());
    nkscene::Bounds moved_bounds;
    moved_bounds.valid = true;
    moved_bounds.minimum = {127.0f, -1.0f, -1.0f};
    moved_bounds.maximum = {129.0f, 1.0f, 1.0f};
    const auto moved_nodes = refitted.query_bounds(moved_bounds);
    assert(moved_nodes.size() == 1 && moved_nodes.front() == node);
    const nkscene::Ray moved_ray{{128.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
    assert(refitted.pick_ray(moved_ray).node == node);

    Transaction over_budget(scene);
    over_budget.add_transform(node, translated(129.0f));
    assert(scene->commit(over_budget, changes) == NKS_OK);
    over_budget.close();
    const auto next = scene->snapshot();
    assert(!refitted.update_node_bounds(next, node));
    assert(refitted.source_revision() != next.revision());
    nkscene::SceneSpatialIndex rebuilt(next);
    assert(rebuilt.pick_ray({{129.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}}).node == node);
}

void render_snapshots_are_concurrent_reader_safe() {
    constexpr std::size_t count = 256;
    constexpr std::size_t reader_count = 3;
    constexpr std::size_t writer_iterations = 48;

    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    scene->geometry_store().create(geometry);
    const auto material = scene->reserve_material_id();
    scene->material_store().create(material);

    std::vector<nkscene::NodeId> nodes;
    nodes.reserve(count);
    Transaction create(scene);
    for (std::size_t index = 0; index < count; ++index) {
        const auto node = scene->reserve_node_id();
        nodes.push_back(node);
        create.add_create(node);
    }
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    for (const auto node : nodes) {
        configure.add_geometry(node, geometry);
        configure.add_material(node, material);
    }
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    std::atomic<bool> stop = false;
    std::atomic<bool> failed = false;
    std::vector<std::thread> readers;
    readers.reserve(reader_count);
    for (std::size_t index = 0; index < reader_count; ++index) {
        readers.emplace_back([&] {
            while (!stop.load(std::memory_order_acquire)) {
                const auto snapshot = scene->snapshot();
                const auto plan = nkscene::compile(snapshot, {});
                if (snapshot.nodes().size() != count || plan.items().size() != count ||
                    !snapshot.find(nodes[count / 2])) {
                    failed.store(true, std::memory_order_release);
                    return;
                }
                std::this_thread::yield();
            }
        });
    }

    for (std::size_t iteration = 0; iteration < writer_iterations; ++iteration) {
        nkscene::LocalTransform transform;
        transform.matrix[12] = static_cast<float>(iteration);
        Transaction move(scene);
        move.add_transform(nodes[iteration % nodes.size()], transform);
        assert(scene->commit(move, changes) == NKS_OK);
        move.close();
    }

    stop.store(true, std::memory_order_release);
    for (auto &reader : readers)
        reader.join();
    assert(!failed.load(std::memory_order_acquire));
}

} // namespace

int main() {
    shader_sources_cover_backend_matrix();
    geometry_payload_contract_is_validated();
    resource_lifecycle_is_cache_safe();
    scene_views_are_hierarchy_aware();
    scene_view_source_filters_are_incremental();
    scene_view_camera_culling_is_incremental();
    scene_resource_camera_is_used_for_render_view();
    scene_view_clip_planes_are_incremental();
    scene_view_culling_uses_spatial_candidates();
    mixed_hierarchy_and_empty_batches_remain_incremental();
    spatial_queries_and_cpu_picking_are_snapshot_bound();
    tagged_strokes_pick_only_when_visible();
    runtime_pose_overrides_render_and_pick_without_mutating_snapshot();
    geometry_edits_refresh_bounds_and_picking_for_new_snapshots();
    spatial_index_refits_transform_changes_and_requests_rebuilds();
    render_snapshots_are_concurrent_reader_safe();
    constexpr std::size_t count = 50000;
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    scene->geometry_store().create(geometry);
    std::vector<nkscene::MaterialId> materials;
    for (int index = 0; index < 4; ++index) {
        const auto material = scene->reserve_material_id();
        materials.push_back(material);
        scene->material_store().create(material);
    }

    std::vector<nkscene::NodeId> nodes;
    nodes.reserve(count);
    Transaction create(scene);
    for (std::size_t index = 0; index < count; ++index) {
        const auto node = scene->reserve_node_id();
        nodes.push_back(node);
        create.add_create(node);
    }
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    for (std::size_t index = 0; index < count; ++index) {
        configure.add_geometry(nodes[index], geometry);
        configure.add_material(nodes[index], materials[index % materials.size()]);
    }
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const auto snapshot = scene->snapshot();
    nkscene::SceneView view;
    auto plan = nkscene::compile(snapshot, view);
    assert(plan.items().size() == count);
    assert(plan.batches().size() == materials.size());
    const auto compile_count = plan.compile_count();
    const auto first = nodes.front();
    const auto first_item = plan.items().front();
    assert(plan.item_index(first) == 0);
    assert(plan.transforms()[first_item.transformIndex].transform.matrix[12] == 0.0f);

    Transaction move(scene);
    move.add_transform(first, translated(5.0f));
    assert(scene->commit(move, changes) == NKS_OK);
    move.close();
    const auto moved_snapshot = scene->snapshot();
    auto update = nkscene::update(plan, moved_snapshot, changes, view);
    assert(!update.plan_rebuilt);
    assert(!update.geometry_rebuilt);
    assert(update.patched_instances == 1);
    assert(plan.compile_count() == compile_count);
    const auto moved_item = plan.items().front();
    assert(plan.transforms()[moved_item.transformIndex].transform.matrix[12] == 5.0f);

    Transaction change_material(scene);
    change_material.add_material(first, materials[1]);
    assert(scene->commit(change_material, changes) == NKS_OK);
    change_material.close();
    update = nkscene::update(plan, scene->snapshot(), changes, view);
    assert(!update.plan_rebuilt);
    assert(update.patched_materials == 1);
    assert(update.rebuilt_batches != 0);

    Transaction hide(scene);
    for (std::size_t index = 0; index < 1000; ++index)
        hide.add_visibility(nodes[index], false);
    assert(scene->commit(hide, changes) == NKS_OK);
    hide.close();
    update = nkscene::update(plan, scene->snapshot(), changes, view);
    assert(!update.plan_rebuilt);
    assert(update.patched_visibility == 1000);
    assert(plan.items().size() == count);

    const auto hidden_snapshot = scene->snapshot();
    const auto scene_revision = scene->revision();
    nkscene::SceneView filtered_view;
    for (std::size_t index = 1000; index < 2000; ++index)
        filtered_view.visibility_overrides.push_back({nodes[index], false});
    nkscene::ChangeSet no_changes;
    update = nkscene::update(plan, hidden_snapshot, no_changes, filtered_view);
    assert(!update.plan_rebuilt);
    assert(update.patched_visibility == 1000);
    assert(update.visible_items == count - 2000);
    assert(plan.items().size() == count);
    assert(scene->revision() == scene_revision);

    auto selected_view = filtered_view;
    selected_view.material_overrides.push_back({nodes[2000], materials[3]});
    update = nkscene::update(plan, hidden_snapshot, no_changes, selected_view);
    assert(!update.plan_rebuilt);
    assert(update.patched_materials == 1);
    assert(update.rebuilt_batches != 0);
    assert(scene->revision() == scene_revision);

    nkscene::NativeKitGpuExecutor executor;
    auto gpu_stats = executor.execute(plan, scene->snapshot());
    assert(gpu_stats.geometry_resources_created == 1);
    assert(gpu_stats.material_resources_created == materials.size());
    assert(gpu_stats.commands == count - 2000);
    assert(executor.commands().size() == count - 2000);
    gpu_stats = executor.execute(plan, scene->snapshot());
    assert(gpu_stats.geometry_resources_created == 0);
    assert(gpu_stats.material_resources_created == 0);
    scene->geometry_store().create(geometry);
    scene->publish();
    gpu_stats = executor.execute(plan, scene->snapshot());
    assert(gpu_stats.geometry_resources_updated == 1);

    const auto pick_result = nkscene::pick(
        plan, scene->snapshot(), 0, nkscene::Vec3{1.0f, 2.0f, 3.0f}, 0.5f);
    assert(pick_result.node.valid());
    assert(pick_result.subelement.valid());
    assert(pick_result.worldPosition.x == 1.0f);
    return 0;
}
