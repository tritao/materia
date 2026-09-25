#include "nativekit.h"
#include "nativekit_scene_render.hpp"
#include "nativekit_window.h"

#include "scene_internal.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <thread>

namespace {

using nkscene::ChangeSet;
using nkscene::Scene;
using nkscene::Transaction;

bool wait_for_surface(nk_surface surface) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        nk_event event{};
        event.struct_size = sizeof(event);
        if (nk_poll_event(&event) != NK_OK)
            return false;
        const bool ready = event.kind == NK_EVENT_SURFACE_READY && event.source == surface;
        nk_event_release(&event);
        if (ready)
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

} // namespace

int main() {
    nk_init_options init{};
    init.struct_size = sizeof(init);
    init.api_version = NK_API_VERSION;
    if (nk_init(&init) != NK_OK)
        return 1;

    nk_window window{};
    nk_surface surface{};
    nkgpu_renderer renderer{};
    int result = 0;
    bool window_created = false;
    bool surface_created = false;

    nk_window_options options{};
    options.struct_size = sizeof(options);
    options.width = 128;
    options.height = 96;
    options.title = "NativeKit scene render GPU test";
    if (nk_window_create(&options, &window) != NK_OK) {
        result = 2;
        goto cleanup;
    }
    window_created = true;
    if (nkgpu_surface_create(window, options.width, options.height, &surface) != NKGPU_OK) {
        result = 2;
        goto cleanup;
    }
    surface_created = true;
    if (!wait_for_surface(surface) || nkgpu_renderer_create(surface, &renderer) != NKGPU_OK) {
        result = 3;
        goto cleanup;
    }

    {
        auto scene = std::make_shared<Scene>();
        const auto geometry = scene->reserve_geometry_id();
        auto &geometry_resource = scene->geometry_store().create(geometry);
        geometry_resource.edit_payload().vertices = {
            {{{-0.6f, -0.6f, 0.0f}}},
            {{{0.6f, -0.6f, 0.0f}}},
            {{{0.0f, 0.6f, 0.0f}}}};
        const std::array<float, 9> normals = {
            0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 1.0f};
        nkscene::GeometryVertexStream normal_stream;
        normal_stream.semantic = nkscene::VertexSemantic::Normal;
        normal_stream.format = nkscene::VertexFormat::Float32x3;
        normal_stream.stride = sizeof(float) * 3;
        normal_stream.count = 3;
        normal_stream.data.resize(sizeof(normals));
        std::memcpy(normal_stream.data.data(), normals.data(), sizeof(normals));
        const std::array<float, 6> texcoords = {
            0.0f, 0.0f,
            1.0f, 0.0f,
            0.5f, 1.0f};
        nkscene::GeometryVertexStream texcoord_stream;
        texcoord_stream.semantic = nkscene::VertexSemantic::Texcoord0;
        texcoord_stream.format = nkscene::VertexFormat::Float32x2;
        texcoord_stream.stride = sizeof(float) * 2;
        texcoord_stream.count = 3;
        texcoord_stream.data.resize(sizeof(texcoords));
        std::memcpy(texcoord_stream.data.data(), texcoords.data(), sizeof(texcoords));
        geometry_resource.edit_payload().streams = {normal_stream, texcoord_stream};
        geometry_resource.edit_payload().indices = {0, 1, 2};
        geometry_resource.edit_subelements().ranges.push_back({0, 1, 42});
        const auto image = scene->reserve_image_id();
        auto &image_resource = scene->image_store().create(image);
        image_resource.width = 1;
        image_resource.height = 1;
        image_resource.format = nkscene::ImageFormat::RGBA8;
        image_resource.data = {std::byte{255}, std::byte{128}, std::byte{64}, std::byte{255}};
        const auto texture = scene->reserve_texture_id();
        auto &texture_resource = scene->texture_store().create(texture);
        texture_resource.image = image;
        const auto sampler = scene->reserve_sampler_id();
        auto &sampler_resource = scene->sampler_store().create(sampler);
        sampler_resource.min_filter = nkscene::SamplerFilter::Nearest;
        sampler_resource.mag_filter = nkscene::SamplerFilter::Nearest;
        sampler_resource.wrap_u = nkscene::SamplerWrap::ClampToEdge;
        sampler_resource.wrap_v = nkscene::SamplerWrap::ClampToEdge;
        const auto light = scene->reserve_light_id();
        auto &light_resource = scene->light_store().create(light);
        light_resource.type = nkscene::LightType::Directional;
        light_resource.intensity = 1.25f;
        const auto material = scene->reserve_material_id();
        auto &material_resource = scene->material_store().create(material);
        material_resource.edit_state().base_color = {0.2f, 0.7f, 1.0f, 1.0f};
        material_resource.edit_state().roughness = 0.0f;
        material_resource.edit_state().base_color_texture = texture;
        material_resource.edit_state().sampler = sampler;

        Transaction create(scene);
        const auto node = scene->reserve_node_id();
        const auto second_node = scene->reserve_node_id();
        const auto light_node = scene->reserve_node_id();
        create.add_create(node);
        create.add_create(second_node);
        create.add_create(light_node);
        ChangeSet changes;
        assert(scene->commit(create, changes) == NKS_OK);
        create.close();
        Transaction configure(scene);
        configure.add_geometry(node, geometry);
        configure.add_material(node, material);
        configure.add_geometry(second_node, geometry);
        configure.add_material(second_node, material);
        configure.add_source_entity(node, nkscene::EntityId{42});
        configure.add_source_entity(second_node, nkscene::EntityId{84});
        configure.add_light(light_node, light);
        nkscene::LocalTransform first_transform;
        first_transform.matrix[12] = -0.8f;
        configure.add_transform(node, first_transform);
        nkscene::LocalTransform second_transform;
        second_transform.matrix[12] = 0.8f;
        configure.add_transform(second_node, second_transform);
        assert(scene->commit(configure, changes) == NKS_OK);
        configure.close();

        nkscene::SceneView view;
        auto plan = nkscene::compile(scene->snapshot(), view);
        nkscene::NativeKitGpuExecutor executor(renderer);
        auto stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.geometry_resources_created == 1);
        assert(stats.instance_buffers_created == 1);
        assert(stats.draw_calls == 1);
        std::vector<std::uint8_t> color_pixels;
        assert(executor.capture_rgba8(plan, scene->snapshot(), options.width, options.height,
                   {0.0f, 0.0f, 0.0f, 1.0f}, color_pixels) == NKGPU_OK);
        assert(color_pixels.size() ==
               static_cast<std::size_t>(options.width * options.height * 4));
        bool found_color = false;
        for (std::size_t index = 0; index + 3 < color_pixels.size(); index += 4) {
            if (color_pixels[index] != 0 || color_pixels[index + 1] != 0 ||
                color_pixels[index + 2] != 0) {
                found_color = true;
                break;
            }
        }
        assert(found_color);

        // View lighting changes the image without changing scene resources or picking.
        nkscene::SceneView studio_view = view;
        studio_view.studio_lighting.enabled = true;
        studio_view.studio_lighting.directions = {{{0.0f, 0.0f, 1.0f, 0.0f},
                                                   {1.0f, 0.0f, 0.0f, 0.0f},
                                                   {0.0f, 1.0f, 0.0f, 0.0f}}};
        studio_view.studio_lighting.ambient_sky = {0.9f, 0.9f, 0.9f, 0.0f};
        studio_view.studio_lighting.ambient_ground = {0.9f, 0.9f, 0.9f, 0.0f};
        auto studio_plan = nkscene::compile(scene->snapshot(), studio_view);
        assert(studio_plan.view_signature() != plan.view_signature());
        std::vector<std::uint8_t> studio_pixels;
        assert(executor.capture_rgba8(studio_plan, scene->snapshot(), options.width,
                   options.height, {0.0f, 0.0f, 0.0f, 1.0f}, studio_pixels) == NKGPU_OK);
        assert(studio_pixels != color_pixels);
        studio_view.studio_lighting.directions[2] = {0.0f, 0.0f, 1.0f, 0.5f};
        studio_view.studio_lighting.colors[2] = {1.0f, 0.0f, 0.0f, 0.0f};
        studio_plan = nkscene::compile(scene->snapshot(), studio_view);
        std::vector<std::uint8_t> rim_pixels;
        assert(executor.capture_rgba8(studio_plan, scene->snapshot(), options.width,
                   options.height, {0.0f, 0.0f, 0.0f, 1.0f}, rim_pixels) == NKGPU_OK);
        assert(rim_pixels != studio_pixels);

        nkscene::PickResult picked;
        assert(executor.pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                  16, options.height / 2, &picked) == NKGPU_OK);
        assert(picked.node == node);
        assert(picked.source == nkscene::EntityId{42});
        assert(picked.subelement.value == 42);
        assert(executor.pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                  112, options.height / 2, &picked) == NKGPU_OK);
        assert(picked.node == second_node);
        assert(picked.source == nkscene::EntityId{84});
        assert(picked.subelement.value == 42);
        nkscene::PickResult miss;
        assert(executor.pick_pixel(plan, scene->snapshot(), options.width, options.height, 0, 0,
                                  &miss) == NKGPU_OK);
        assert(!miss.node.valid());

        std::shared_ptr<nkscene::GpuPickRequest> stale_request;
        assert(executor.begin_pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                         16, options.height / 2, stale_request) == NKGPU_OK);
        nkscene::SceneView changed_view;
        changed_view.include_invisible = true;
        const auto changed_plan = nkscene::compile(scene->snapshot(), changed_view);
        nkscene::PickResult stale_result;
        nkgpu_result stale_error = NKGPU_OK;
        const auto stale_state = executor.poll_pick_pixel(
            *stale_request, changed_plan, scene->snapshot(), &stale_result, &stale_error);
        assert(stale_state == NKS_RENDER_PICK_STALE);
        assert(stale_error == NKGPU_OK);

        std::shared_ptr<nkscene::GpuPickRequest> async_request;
        assert(executor.begin_pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                         16, options.height / 2, async_request) == NKGPU_OK);
        nkscene::PickResult async_picked;
        nkgpu_result async_error = NKGPU_OK;
        std::uint32_t async_state = NKS_RENDER_PICK_PENDING;
        for (int attempt = 0; attempt < 100 && async_state == NKS_RENDER_PICK_PENDING;
             ++attempt) {
            async_state = executor.poll_pick_pixel(
                *async_request, plan, scene->snapshot(), &async_picked, &async_error);
            if (async_state == NKS_RENDER_PICK_PENDING)
                std::this_thread::yield();
        }
        assert(async_state == NKS_RENDER_PICK_READY);
        assert(async_error == NKGPU_OK);
        assert(async_picked.node == node);
        assert(async_picked.source == nkscene::EntityId{42});
        assert(async_picked.subelement.value == 42);
        assert(std::abs(async_picked.worldPosition.x + 0.7421875f) < 0.05f);
        assert(std::abs(async_picked.worldPosition.z) < 0.001f);
        assert(std::abs(async_picked.depth - 0.5f) < 0.01f);

        nkscene::LocalTransform transform;
        transform.matrix[12] = 0.25f;
        Transaction move(scene);
        move.add_transform(node, transform);
        assert(scene->commit(move, changes) == NKS_OK);
        move.close();
        const auto moved_snapshot = scene->snapshot();
        const auto update = nkscene::update(plan, moved_snapshot, changes, view);
        assert(!update.plan_rebuilt);

        std::shared_ptr<nkscene::GpuPickRequest> moved_request;
        assert(executor.begin_pick_pixel(plan, moved_snapshot, options.width, options.height, 80,
                                         options.height / 2, moved_request) == NKGPU_OK);
        nkscene::PickResult moved_picked;
        nkgpu_result moved_error = NKGPU_OK;
        std::uint32_t moved_state = NKS_RENDER_PICK_PENDING;
        for (int attempt = 0; attempt < 100 && moved_state == NKS_RENDER_PICK_PENDING;
             ++attempt) {
            moved_state = executor.poll_pick_pixel(*moved_request, plan, moved_snapshot,
                                                    &moved_picked, &moved_error);
            if (moved_state == NKS_RENDER_PICK_PENDING)
                std::this_thread::yield();
        }
        assert(moved_state == NKS_RENDER_PICK_READY);
        assert(moved_error == NKGPU_OK);
        assert(moved_picked.node == node);
        assert(moved_picked.source == nkscene::EntityId{42});

        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.geometry_resources_created == 0);
        assert(stats.geometry_resources_updated == 0);
        assert(stats.instance_records_updated == 0);
        assert(stats.draw_calls == 1);

        Transaction change_source(scene);
        change_source.add_source_entity(second_node, nkscene::EntityId{142});
        assert(scene->commit(change_source, changes) == NKS_OK);
        change_source.close();
        const auto source_snapshot = scene->snapshot();
        const auto source_update = nkscene::update(plan, source_snapshot, changes, view);
        assert(!source_update.plan_rebuilt);
        assert(source_update.patched_instances == 0);
        stats = executor.execute(plan, source_snapshot);
        assert(stats.result == NKGPU_OK);
        assert(stats.geometry_resources_created == 0);
        assert(stats.geometry_resources_updated == 0);
        assert(stats.instance_records_updated == 0);
        assert(executor.pick_pixel(plan, source_snapshot, options.width, options.height,
                                  112, options.height / 2, &picked) == NKGPU_OK);
        assert(picked.node == second_node);
        assert(picked.source == nkscene::EntityId{142});

        Transaction hide_second(scene);
        hide_second.add_visibility(second_node, false);
        assert(scene->commit(hide_second, changes) == NKS_OK);
        hide_second.close();
        const auto hidden_snapshot = scene->snapshot();
        const auto hidden_update = nkscene::update(plan, hidden_snapshot, changes, view);
        assert(!hidden_update.plan_rebuilt);
        stats = executor.execute(plan, hidden_snapshot);
        assert(stats.result == NKGPU_OK);
        assert(stats.full_rebuilds == 0);
        assert(stats.batches_inspected == 1);
        assert(stats.batch_instances_inspected == 1);
        assert(stats.commands_patched == 1);
        assert(stats.commands == 1);

        Transaction show_second(scene);
        show_second.add_visibility(second_node, true);
        assert(scene->commit(show_second, changes) == NKS_OK);
        show_second.close();
        const auto restored_snapshot = scene->snapshot();
        const auto restored_update = nkscene::update(plan, restored_snapshot, changes, view);
        assert(!restored_update.plan_rebuilt);
        stats = executor.execute(plan, restored_snapshot);
        assert(stats.result == NKGPU_OK);
        assert(stats.full_rebuilds == 0);
        assert(stats.batches_inspected == 1);
        assert(stats.batch_instances_inspected == 2);
        assert(stats.commands_patched == 1);
        assert(stats.commands == 2);

        const auto alternate_material = scene->reserve_material_id();
        scene->material_store().create(alternate_material);
        Transaction change_instance_material(scene);
        change_instance_material.add_material(node, alternate_material);
        assert(scene->commit(change_instance_material, changes) == NKS_OK);
        change_instance_material.close();
        const auto rematerialized_snapshot = scene->snapshot();
        const auto rematerialized_update =
            nkscene::update(plan, rematerialized_snapshot, changes, view);
        assert(!rematerialized_update.plan_rebuilt);
        stats = executor.execute(plan, rematerialized_snapshot);
        assert(stats.result == NKGPU_OK);
        assert(stats.full_rebuilds == 0);
        assert(stats.batches_inspected == 2);
        assert(stats.batch_instances_inspected == 2);
        assert(stats.commands_patched == 1);
        assert(stats.commands == 2);

        Transaction restore_instance_material(scene);
        restore_instance_material.add_material(node, material);
        assert(scene->commit(restore_instance_material, changes) == NKS_OK);
        restore_instance_material.close();
        const auto restored_material_snapshot = scene->snapshot();
        const auto restored_material_update =
            nkscene::update(plan, restored_material_snapshot, changes, view);
        assert(!restored_material_update.plan_rebuilt);
        stats = executor.execute(plan, restored_material_snapshot);
        assert(stats.result == NKGPU_OK);
        assert(stats.full_rebuilds == 0);
        assert(stats.batches_inspected == 2);
        assert(stats.batch_instances_inspected == 2);
        assert(stats.commands_patched == 1);
        assert(stats.commands == 2);

        auto &updated_material = scene->material_store().create(material);
        updated_material.edit_state().base_color = {1.0f, 0.3f, 0.2f, 1.0f};
        scene->publish();
        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.material_resources_updated == 1);

        auto &non_indexed_geometry = scene->geometry_store().create(geometry);
        non_indexed_geometry.edit_payload().indices.clear();
        scene->publish();
        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.geometry_resources_created == 0);
        assert(stats.geometry_resources_updated == 1);
        assert(stats.draw_calls == 1);

        auto &indexed_geometry = scene->geometry_store().create(geometry);
        indexed_geometry.edit_payload().indices = {0, 1, 2};
        scene->publish();
        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.geometry_resources_created == 0);
        assert(stats.geometry_resources_updated == 1);
        assert(stats.draw_calls == 1);

        const auto vertices = indexed_geometry.payload->vertices;
        const auto unused_geometry = scene->reserve_geometry_id();
        auto &unused_resource = scene->geometry_store().create(unused_geometry);
        unused_resource.edit_payload().vertices = vertices;
        unused_resource.edit_payload().indices = {0, 1, 2};
        scene->publish();
        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.geometry_resources_created == 1);
        assert(stats.geometry_resources_updated == 0);
        assert(stats.draw_calls == 1);

        assert(scene->geometry_store().destroy(unused_geometry));
        scene->publish();
        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.geometry_resources_created == 0);
        assert(stats.geometry_resources_updated == 0);
        assert(stats.draw_calls == 1);

        auto &recreated_geometry = scene->geometry_store().create(unused_geometry);
        recreated_geometry.edit_payload().vertices = vertices;
        recreated_geometry.edit_payload().indices.clear();
        scene->publish();
        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.geometry_resources_created == 1);
        assert(stats.geometry_resources_updated == 0);
        assert(stats.draw_calls == 1);

        for (std::size_t iteration = 0; iteration < 70; ++iteration) {
            nkscene::LocalTransform lagged_transform;
            lagged_transform.matrix[12] = 0.25f + static_cast<float>(iteration) * 0.01f;
            Transaction lagged_move(scene);
            lagged_move.add_transform(node, lagged_transform);
            assert(scene->commit(lagged_move, changes) == NKS_OK);
            lagged_move.close();
            const auto lagged_update = nkscene::update(plan, scene->snapshot(), changes, view);
            assert(!lagged_update.plan_rebuilt);
        }
        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.full_rebuilds == 1);
        assert(stats.instance_records_updated == 1);
        assert(stats.draw_calls == 1);
    }

    {
        auto scene = std::make_shared<Scene>();
        const auto geometry = scene->reserve_geometry_id();
        auto &geometry_resource = scene->geometry_store().create(geometry);
        geometry_resource.edit_payload().vertices = {
            {{{-0.6f, -0.6f, 0.0f}}},
            {{{0.6f, -0.6f, 0.0f}}},
            {{{0.0f, 0.6f, 0.0f}}}};
        geometry_resource.edit_payload().indices = {0, 1, 2};
        geometry_resource.edit_subelements().ranges.push_back({0, 1, 7});
        const auto material = scene->reserve_material_id();
        auto &material_resource = scene->material_store().create(material);
        material_resource.edit_state().base_color = {0.8f, 0.8f, 0.8f, 1.0f};

        Transaction create(scene);
        const auto node = scene->reserve_node_id();
        create.add_create(node);
        ChangeSet changes;
        assert(scene->commit(create, changes) == NKS_OK);
        create.close();
        Transaction configure(scene);
        configure.add_geometry(node, geometry);
        configure.add_material(node, material);
        assert(scene->commit(configure, changes) == NKS_OK);
        configure.close();

        nkscene::SceneView view;
        view.clip_planes.push_back({{{1.0f, 0.0f, 0.0f}}, 0.0f, true});
        auto plan = nkscene::compile(scene->snapshot(), view);
        assert(plan.clip_planes().size() == 1);
        assert(plan.visible_items() == 1);
        assert(plan.culled_items() == 0);

        nkscene::NativeKitGpuExecutor executor(renderer);
        const auto stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.draw_calls == 1);

        nkscene::PickResult clipped;
        assert(executor.pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                  48, options.height / 2, &clipped) == NKGPU_OK);
        assert(!clipped.node.valid());

        nkscene::PickResult visible;
        assert(executor.pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                  80, options.height / 2, &visible) == NKGPU_OK);
        assert(visible.node == node);
        assert(visible.subelement.value == 7);
    }

    {
        auto scene = std::make_shared<Scene>();
        const auto geometry = scene->reserve_geometry_id();
        auto &mesh = scene->geometry_store().create(geometry);
        mesh.edit_payload().vertices = {{{-0.65f, -0.65f, 0.0f}},
                                        {{0.65f, -0.65f, 0.0f}},
                                        {{0.0f, 0.65f, 0.0f}}};
        mesh.bounds = {{-0.65f, -0.65f, -0.01f}, {0.65f, 0.65f, 0.01f}, true};
        const auto material = scene->reserve_material_id();
        scene->material_store().create(material);
        const auto mesh_node = scene->reserve_node_id();
        Transaction create(scene);
        create.add_create(mesh_node);
        ChangeSet changes;
        assert(scene->commit(create, changes) == NKS_OK);
        create.close();
        Transaction configure(scene);
        configure.add_geometry(mesh_node, geometry);
        configure.add_material(mesh_node, material);
        assert(scene->commit(configure, changes) == NKS_OK);
        configure.close();

        nkscene::NativeKitGpuExecutor executor(renderer);
        nkscene::SceneView authored_view;
        const auto capture = [&](const nkscene::SceneView &view) {
            auto plan = nkscene::compile(scene->snapshot(), view);
            std::vector<std::uint8_t> pixels;
            assert(executor.capture_rgba8(plan, scene->snapshot(), options.width, options.height,
                       {0.0f, 0.0f, 0.0f, 1.0f}, pixels) == NKGPU_OK);
            return pixels;
        };
        const auto attach_light = [&](nkscene::LightType type, std::array<float, 3> color,
                                      float intensity, float range,
                                      nkscene::LocalTransform transform) {
            const auto id = scene->reserve_light_id();
            auto &light = scene->light_store().create(id);
            light.type = type;
            light.color = color;
            light.intensity = intensity;
            light.range = range;
            scene->publish();
            const auto node = scene->reserve_node_id();
            Transaction add(scene);
            add.add_create(node);
            assert(scene->commit(add, changes) == NKS_OK);
            add.close();
            Transaction set(scene);
            set.add_light(node, id);
            set.add_transform(node, transform);
            assert(scene->commit(set, changes) == NKS_OK);
            set.close();
            return std::pair{id, node};
        };
        nkscene::LocalTransform toward_surface;
        toward_surface.matrix[0] = 0.0f;
        toward_surface.matrix[2] = -1.0f;
        toward_surface.matrix[8] = 1.0f;
        toward_surface.matrix[10] = 0.0f;
        const auto red = attach_light(nkscene::LightType::Directional, {1.0f, 0.0f, 0.0f},
                                      0.4f, 10.0f, toward_surface);
        const auto red_pixels = capture(authored_view);
        const auto green = attach_light(nkscene::LightType::Directional, {0.0f, 1.0f, 0.0f},
                                        0.4f, 10.0f, toward_surface);
        const auto two_directional_pixels = capture(authored_view);
        assert(two_directional_pixels != red_pixels);
        (void)red;
        (void)green;

        nkscene::SceneView studio_view;
        studio_view.studio_lighting.enabled = true;
        studio_view.studio_lighting.directions = {{{0.0f, 0.0f, 1.0f, 0.5f},
                                                   {1.0f, 0.0f, 0.0f, 0.0f},
                                                   {0.0f, 1.0f, 0.0f, 0.0f}}};
        const auto studio_before = capture(studio_view);

        nkscene::LocalTransform point_transform;
        point_transform.matrix[14] = 2.0f;
        const auto [point_id, point_node] =
            attach_light(nkscene::LightType::Point, {0.0f, 0.0f, 1.0f},
                         0.8f, 0.5f, point_transform);
        assert(capture(authored_view) == two_directional_pixels);
        scene->light_store().create(point_id).range = 4.0f;
        scene->publish();
        const auto point_pixels = capture(authored_view);
        assert(point_pixels != two_directional_pixels);
        (void)point_node;

        nkscene::LocalTransform spot_transform;
        spot_transform.matrix[0] = 0.0f;
        spot_transform.matrix[2] = 1.0f;
        spot_transform.matrix[8] = -1.0f;
        spot_transform.matrix[10] = 0.0f;
        spot_transform.matrix[14] = 2.0f;
        const auto [spot_id, spot_node] =
            attach_light(nkscene::LightType::Spot, {1.0f, 1.0f, 1.0f},
                         0.8f, 4.0f, spot_transform);
        scene->light_store().create(spot_id).inner_cone_angle = 0.1f;
        scene->light_store().find(spot_id)->outer_cone_angle = 0.5f;
        scene->publish();
        const auto spot_pixels = capture(authored_view);
        assert(spot_pixels != point_pixels);
        spot_transform.matrix[12] = 4.0f;
        Transaction move_spot(scene);
        move_spot.add_transform(spot_node, spot_transform);
        assert(scene->commit(move_spot, changes) == NKS_OK);
        move_spot.close();
        assert(capture(authored_view) == point_pixels);
        assert(capture(studio_view) == studio_before);
    }

cleanup:
    if (renderer.id)
        nkgpu_renderer_destroy(renderer);
    if (surface_created)
        nkgpu_surface_destroy(surface);
    if (window_created)
        nk_window_destroy(window);
    nk_shutdown();
    return result;
}
