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
#include <vector>

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

        // Padded float streams must produce the same image as tightly packed attributes.
        auto padded_normal = normal_stream;
        padded_normal.stride = 20;
        padded_normal.data.assign(52, std::byte{0x7f});
        for (std::size_t index = 0; index < 3; ++index)
            std::memcpy(padded_normal.data.data() + index * 20,
                        normals.data() + index * 3, 12);
        geometry_resource.edit_payload().streams = {padded_normal, texcoord_stream};
        scene->publish();
        std::vector<std::uint8_t> padded_pixels;
        assert(executor.capture_rgba8(plan, scene->snapshot(), options.width, options.height,
                   {0.0f, 0.0f, 0.0f, 1.0f}, padded_pixels) == NKGPU_OK);
        assert(padded_pixels == color_pixels);
        // A truncated last attribute must fail before any upload, and a corrected revision retries.
        padded_normal.data.pop_back();
        geometry_resource.edit_payload().streams = {padded_normal, texcoord_stream};
        scene->publish();
        assert(executor.execute(plan, scene->snapshot()).result == NKGPU_ERROR_INVALID_ARGUMENT);
        geometry_resource.edit_payload().streams = {normal_stream, texcoord_stream};
        scene->publish();
        assert(executor.execute(plan, scene->snapshot()).result == NKGPU_OK);

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

        // Roughness changes highlight width, and metallic changes the energy split.
        nkscene::SceneView pbr_view = studio_view;
        pbr_view.camera.enabled = true;
        pbr_view.camera.view_projection[10] = -1.0f;
        pbr_view.camera.has_view_pose = true;
        pbr_view.camera.position = {0.0f, 0.0f, 2.0f};
        pbr_view.camera.view_direction = {0.0f, 0.0f, 1.0f};
        pbr_view.studio_lighting.ambient_sky = {0.0f, 0.0f, 0.0f, 0.0f};
        pbr_view.studio_lighting.ambient_ground = {0.0f, 0.0f, 0.0f, 0.0f};
        const auto pbr_plan = nkscene::compile(scene->snapshot(), pbr_view);
        const auto capture_pbr = [&] {
            std::vector<std::uint8_t> pixels;
            assert(executor.capture_rgba8(pbr_plan, scene->snapshot(), options.width,
                       options.height, {0.0f, 0.0f, 0.0f, 1.0f}, pixels) == NKGPU_OK);
            return pixels;
        };
        material_resource.edit_state().roughness = 0.25f;
        scene->publish();
        const auto smooth_pixels = capture_pbr();
        auto shifted_view = pbr_view;
        shifted_view.camera.position[0] = 1.0f;
        const auto shifted_plan = nkscene::compile(scene->snapshot(), shifted_view);
        assert(shifted_plan.view_signature() != pbr_plan.view_signature());
        std::vector<std::uint8_t> shifted_pixels;
        assert(executor.capture_rgba8(shifted_plan, scene->snapshot(), options.width,
                   options.height, {0.0f, 0.0f, 0.0f, 1.0f}, shifted_pixels) == NKGPU_OK);
        assert(shifted_pixels != smooth_pixels);
        auto orthographic_view = pbr_view;
        orthographic_view.camera.orthographic = true;
        const auto ortho_plan = nkscene::compile(scene->snapshot(), orthographic_view);
        std::vector<std::uint8_t> ortho_pixels;
        assert(executor.capture_rgba8(ortho_plan, scene->snapshot(), options.width,
                   options.height, {0.0f, 0.0f, 0.0f, 1.0f}, ortho_pixels) == NKGPU_OK);
        orthographic_view.camera.position[0] = 1.0f;
        const auto shifted_ortho_plan = nkscene::compile(scene->snapshot(), orthographic_view);
        std::vector<std::uint8_t> shifted_ortho_pixels;
        assert(executor.capture_rgba8(shifted_ortho_plan, scene->snapshot(), options.width,
                   options.height, {0.0f, 0.0f, 0.0f, 1.0f}, shifted_ortho_pixels) == NKGPU_OK);
        assert(shifted_ortho_pixels == ortho_pixels);
        material_resource.edit_state().roughness = 0.9f;
        scene->publish();
        const auto rough_pixels = capture_pbr();
        assert(smooth_pixels != rough_pixels);
        material_resource.edit_state().metallic = 1.0f;
        scene->publish();
        assert(capture_pbr() != rough_pixels);
        material_resource.edit_state().metallic = 0.0f;
        material_resource.edit_state().roughness = 0.0f;
        scene->publish();

        nkscene::SceneView reflection_view = pbr_view;
        reflection_view.studio_lighting.directions[2][3] = 0.0f;
        reflection_view.studio_lighting.ambient_sky = {0.4f, 0.4f, 0.4f, 0.0f};
        reflection_view.studio_lighting.ambient_ground = {0.05f, 0.05f, 0.05f, 0.0f};
        const auto reflection_plan = nkscene::compile(scene->snapshot(), reflection_view);
        material_resource.edit_state().metallic = 1.0f;
        material_resource.edit_state().roughness = 0.2f;
        scene->publish();
        std::vector<std::uint8_t> reflected_pixels;
        assert(executor.capture_rgba8(reflection_plan, scene->snapshot(), options.width,
                   options.height, {0.0f, 0.0f, 0.0f, 1.0f}, reflected_pixels) == NKGPU_OK);
        bool has_reflection = false;
        for (std::size_t index = 0; index < reflected_pixels.size(); index += 4)
            has_reflection = has_reflection || reflected_pixels[index] != 0 ||
                             reflected_pixels[index + 1] != 0 || reflected_pixels[index + 2] != 0;
        assert(has_reflection);
        material_resource.edit_state().roughness = 0.9f;
        scene->publish();
        std::vector<std::uint8_t> rough_reflected_pixels;
        assert(executor.capture_rgba8(reflection_plan, scene->snapshot(), options.width,
                   options.height, {0.0f, 0.0f, 0.0f, 1.0f}, rough_reflected_pixels) == NKGPU_OK);
        assert(rough_reflected_pixels != reflected_pixels);
        material_resource.edit_state().metallic = 0.0f;
        material_resource.edit_state().roughness = 0.0f;
        scene->publish();

        const auto make_map = [&](std::array<std::byte, 4> rgba) {
            const auto map_image = scene->reserve_image_id();
            auto &map_resource = scene->image_store().create(map_image);
            map_resource.width = map_resource.height = 1;
            map_resource.format = nkscene::ImageFormat::RGBA8;
            map_resource.data.assign(rgba.begin(), rgba.end());
            const auto map_texture = scene->reserve_texture_id();
            scene->texture_store().create(map_texture).image = map_image;
            return map_texture;
        };
        const auto metal_rough_map = make_map({std::byte{255}, std::byte{48},
                                                std::byte{255}, std::byte{255}});
        const auto normal_map = make_map({std::byte{255}, std::byte{128},
                                          std::byte{128}, std::byte{255}});
        const auto occlusion_map = make_map({std::byte{24}, std::byte{24},
                                             std::byte{24}, std::byte{255}});
        const auto emissive_map = make_map({std::byte{0}, std::byte{255},
                                            std::byte{0}, std::byte{255}});
        material_resource.edit_state().roughness = 0.7f;
        material_resource.edit_state().metallic = 0.3f;
        material_resource.edit_state().emissive = {0.25f, 0.25f, 0.25f};
        scene->publish();
        const auto capture_maps = [&] {
            std::vector<std::uint8_t> pixels;
            assert(executor.capture_rgba8(studio_plan, scene->snapshot(), options.width,
                       options.height, {0.0f, 0.0f, 0.0f, 1.0f}, pixels) == NKGPU_OK);
            return pixels;
        };
        auto previous_map_pixels = capture_maps();
        const auto check_map = [&](nkscene::TextureId map, nkscene::TextureId nkscene::MaterialState::*slot) {
            material_resource.edit_state().*slot = map;
            scene->publish();
            const auto pixels = capture_maps();
            assert(pixels != previous_map_pixels);
            previous_map_pixels = pixels;
        };
        check_map(metal_rough_map, &nkscene::MaterialState::metallic_roughness_texture);
        check_map(normal_map, &nkscene::MaterialState::normal_texture);
        check_map(occlusion_map, &nkscene::MaterialState::occlusion_texture);
        check_map(emissive_map, &nkscene::MaterialState::emissive_texture);
        const auto occlusion_image = scene->texture_store().find(occlusion_map)->image;
        scene->image_store().create(occlusion_image).data = {
            std::byte{255}, std::byte{255}, std::byte{255}, std::byte{255}};
        scene->publish();
        assert(capture_maps() != previous_map_pixels);

        // Emissive color and RGBA8 texels are decoded before multiplication.
        const auto half_red_map = make_map({std::byte{128}, std::byte{0},
                                             std::byte{0}, std::byte{255}});
        auto dark_view = reflection_view;
        dark_view.studio_lighting.ambient_sky = {0.0f, 0.0f, 0.0f, 0.0f};
        dark_view.studio_lighting.ambient_ground = {0.0f, 0.0f, 0.0f, 0.0f};
        const auto dark_plan = nkscene::compile(scene->snapshot(), dark_view);
        material_resource.edit_state().base_color = {0.0f, 0.0f, 0.0f, 1.0f};
        material_resource.edit_state().emissive = {0.5f, 0.0f, 0.0f};
        material_resource.edit_state().emissive_texture = half_red_map;
        scene->publish();
        std::vector<std::uint8_t> linear_pixels;
        assert(executor.capture_rgba8(dark_plan, scene->snapshot(), options.width,
                   options.height, {0.0f, 0.0f, 0.0f, 1.0f}, linear_pixels) == NKGPU_OK);
        const auto srgb_decode = [](float value) {
            return value <= 0.04045f ? value / 12.92f
                                     : std::pow((value + 0.055f) / 1.055f, 2.4f);
        };
        const auto srgb_encode = [](float value) {
            return value <= 0.0031308f ? value * 12.92f
                                       : 1.055f * std::pow(value, 1.0f / 2.4f) - 0.055f;
        };
        const auto expected_red = static_cast<int>(std::round(
            srgb_encode(srgb_decode(0.5f) * srgb_decode(128.0f / 255.0f)) * 255.0f));
        const auto red_pixel = linear_pixels[(48 * options.width + 24) * 4];
        assert(std::abs(static_cast<int>(red_pixel) - expected_red) <= 2);
        material_resource.edit_state().metallic_roughness_texture = {};
        material_resource.edit_state().normal_texture = {};
        material_resource.edit_state().occlusion_texture = {};
        material_resource.edit_state().emissive_texture = {};
        material_resource.edit_state().metallic = 0.0f;
        material_resource.edit_state().roughness = 0.0f;
        material_resource.edit_state().emissive = {};
        material_resource.edit_state().base_color = {0.2f, 0.7f, 1.0f, 1.0f};
        scene->publish();

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

        // Both visible instances share one batch and must use a single persistent upload.
        Transaction move_pair(scene);
        nkscene::LocalTransform pair_transform;
        pair_transform.matrix[12] = 0.1f;
        move_pair.add_transform(node, pair_transform);
        pair_transform.matrix[12] = -0.1f;
        move_pair.add_transform(second_node, pair_transform);
        assert(scene->commit(move_pair, changes) == NKS_OK);
        move_pair.close();
        const auto pair_snapshot = scene->snapshot();
        nkscene::update(plan, pair_snapshot, changes, view);
        stats = executor.execute(plan, pair_snapshot);
        assert(stats.result == NKGPU_OK);
        assert(stats.instance_records_updated == 2);
        assert(stats.instance_buffer_updates == 1);
        stats = executor.execute(plan, pair_snapshot);
        assert(stats.instance_records_updated == 0 && stats.instance_buffer_updates == 0);

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

        // A fresh plan reconciles all resources, including unchanged meshes.
        // Their prepared surface/picking buffers must survive this cache hit.
        const auto unchanged_snapshot = scene->snapshot();
        auto recompiled_plan = nkscene::compile(unchanged_snapshot, view);
        stats = executor.execute(recompiled_plan, unchanged_snapshot);
        assert(stats.result == NKGPU_OK);
        assert(stats.full_rebuilds == 1);
        assert(stats.geometry_resources_created == 0);
        assert(stats.geometry_resources_updated == 0);
        assert(stats.draw_calls == 1);
        std::vector<std::uint8_t> cached_pixels;
        assert(executor.capture_rgba8(recompiled_plan, unchanged_snapshot,
                                     options.width, options.height,
                                     {0.0f, 0.0f, 0.0f, 1.0f}, cached_pixels) == NKGPU_OK);
        assert(cached_pixels.size() == options.width * options.height * 4);
        nkscene::PickResult cached_pick;
        assert(executor.pick_pixel(recompiled_plan, unchanged_snapshot,
                                   options.width, options.height, 80, options.height / 2,
                                   &cached_pick) == NKGPU_OK);
        assert(cached_pick.node == node);
        assert(cached_pick.source == nkscene::EntityId{42});
        assert(cached_pick.subelement.value == 42);
        // Restore the original plan before checking its bounded delta history.
        stats = executor.execute(plan, unchanged_snapshot);
        assert(stats.result == NKGPU_OK);

        for (std::size_t iteration = 0; iteration < 70; ++iteration) {
            nkscene::LocalTransform lagged_transform;
            lagged_transform.matrix[12] = 0.25f + static_cast<float>(iteration) * 0.01f;
            Transaction lagged_move(scene);
            lagged_move.add_transform(node, lagged_transform);
            lagged_transform.matrix[12] = -lagged_transform.matrix[12];
            lagged_move.add_transform(second_node, lagged_transform);
            assert(scene->commit(lagged_move, changes) == NKS_OK);
            lagged_move.close();
            const auto lagged_update = nkscene::update(plan, scene->snapshot(), changes, view);
            assert(!lagged_update.plan_rebuilt);
        }
        stats = executor.execute(plan, scene->snapshot());
        assert(stats.result == NKGPU_OK);
        assert(stats.full_rebuilds == 1);
        assert(stats.instance_records_updated == 2);
        assert(stats.instance_buffer_updates == 1);
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

        // An existing pick buffer must refresh after display-only geometry updates.
        auto &moving_geometry = scene->geometry_store().create(geometry);
        const auto original_vertices = moving_geometry.payload->vertices;
        moving_geometry.edit_payload().indices = {0, 0, 0};
        scene->publish();
        assert(executor.execute(plan, scene->snapshot()).result == NKGPU_OK);
        std::vector<std::uint8_t> degenerate_pixels;
        assert(executor.capture_rgba8(plan, scene->snapshot(), options.width, options.height,
                                     {0.0f, 0.0f, 0.0f, 1.0f}, degenerate_pixels) == NKGPU_OK);
        const auto center_pixel = (options.height / 2 * options.width + 80) * 4;
        assert(degenerate_pixels[center_pixel] == 0 && degenerate_pixels[center_pixel + 1] == 0 &&
               degenerate_pixels[center_pixel + 2] == 0);
        moving_geometry.edit_payload().indices = {0, 1, 2};
        for (auto &vertex : moving_geometry.edit_payload().vertices)
            vertex.position[0] += 3.0f;
        scene->publish();
        assert(executor.execute(plan, scene->snapshot()).result == NKGPU_OK);
        assert(executor.pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                  80, options.height / 2, &visible) == NKGPU_OK);
        assert(!visible.node.valid());
        moving_geometry.edit_payload().vertices = original_vertices;
        moving_geometry.edit_payload().indices.clear();
        scene->publish();
        assert(executor.execute(plan, scene->snapshot()).result == NKGPU_OK);
        assert(executor.pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                  80, options.height / 2, &visible) == NKGPU_OK);
        assert(visible.node == node);
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

    {
        // Transparency: zero opacity draws nothing, partial opacity blends with what lies
        // behind, opaque surfaces still hide translucent ones, and translucent surfaces
        // blend far to near whatever order they were created in.
        struct Quad {
            std::array<float, 3> emissive;
            float alpha;
            float z;
            bool opaque_surface;
            /* Turns the quad about its center, so its edges are not pixel-aligned. */
            float angle = 0.0f;
        };
        struct Capture {
            std::vector<std::uint8_t> pixels;
            /* Whether picking the middle pixel found the first quad created. */
            bool picked_first = false;
        };
        // Renders the quads once for each sample count in `samples`, all through one executor.
        const auto render_samples = [&](const std::vector<Quad> &quads, bool reverse_creation,
                                        std::array<float, 4> clear,
                                        const std::vector<std::uint32_t> &samples, bool outlines = true,
                                        bool grid = false,
                                        const nkscene::RgbaPostProcess &post = {}) -> std::vector<Capture> {
            auto scene = std::make_shared<Scene>();
            const auto geometry = scene->reserve_geometry_id();
            auto &geometry_resource = scene->geometry_store().create(geometry);
            geometry_resource.edit_payload().vertices = {
                {{{-0.5f, -0.5f, 0.0f}}}, {{{0.5f, -0.5f, 0.0f}}},
                {{{0.5f, 0.5f, 0.0f}}}, {{{-0.5f, 0.5f, 0.0f}}}};
            std::array<float, 12> normals{};
            for (std::size_t vertex = 0; vertex < 4; ++vertex)
                normals[vertex * 3 + 2] = 1.0f;
            nkscene::GeometryVertexStream normal_stream;
            normal_stream.semantic = nkscene::VertexSemantic::Normal;
            normal_stream.format = nkscene::VertexFormat::Float32x3;
            normal_stream.stride = sizeof(float) * 3;
            normal_stream.count = 4;
            normal_stream.data.resize(sizeof(normals));
            std::memcpy(normal_stream.data.data(), normals.data(), sizeof(normals));
            geometry_resource.edit_payload().streams = {normal_stream};
            geometry_resource.edit_payload().indices = {0, 1, 2, 0, 2, 3};
            // Outline the quad, so a test can tell whether its edges were drawn.
            const std::array<std::array<float, 3>, 4> corners{
                {{-0.5f, -0.5f, 0.0f}, {0.5f, -0.5f, 0.0f}, {0.5f, 0.5f, 0.0f}, {-0.5f, 0.5f, 0.0f}}};
            for (std::size_t corner = 0; outlines && corner < 4; ++corner)
                geometry_resource.edit_payload().stroke_segments.push_back(
                    {corners[corner], corners[(corner + 1) % 4], 1});

            std::vector<nkscene::NodeId> nodes;
            std::vector<std::size_t> order;
            for (std::size_t index = 0; index < quads.size(); ++index)
                order.push_back(reverse_creation ? quads.size() - 1 - index : index);
            for (const auto index : order) {
                const auto &quad = quads[index];
                const auto material = scene->reserve_material_id();
                auto &resource = scene->material_store().create(material);
                // Emissive only, so lighting cannot change the expected color.
                resource.edit_state().base_color = {0.0f, 0.0f, 0.0f, quad.alpha};
                resource.edit_state().emissive = quad.emissive;
                if (!quad.opaque_surface)
                    resource.edit_state().flags &=
                        ~static_cast<std::uint32_t>(nkscene::MaterialFlags::Opaque);
                Transaction create(scene);
                const auto node = scene->reserve_node_id();
                nodes.push_back(node);
                create.add_create(node);
                ChangeSet changes;
                assert(scene->commit(create, changes) == NKS_OK);
                create.close();
                Transaction configure(scene);
                configure.add_geometry(node, geometry);
                configure.add_material(node, material);
                nkscene::LocalTransform placement;
                placement.matrix[14] = quad.z;
                if (quad.angle != 0.0f) {
                    placement.matrix[0] = std::cos(quad.angle);
                    placement.matrix[1] = std::sin(quad.angle);
                    placement.matrix[4] = -std::sin(quad.angle);
                    placement.matrix[5] = std::cos(quad.angle);
                }
                configure.add_transform(node, placement);
                assert(scene->commit(configure, changes) == NKS_OK);
                configure.close();
            }
            nkscene::SceneView view;
            if (grid) {
                view.workplane_grid.enabled = true;
                view.workplane_grid.eye_spacing = {0.0f, 0.0f, 3.0f, 0.25f};
                view.workplane_grid.forward = {0.0f, 0.0f, -1.0f, 3.0f};
                view.workplane_grid.right = {1.0f, 0.0f, 0.0f, 0.0f};
                view.workplane_grid.up = {0.0f, 1.0f, 0.0f, 0.0f};
            }
            auto plan = nkscene::compile(scene->snapshot(), view);
            nkscene::NativeKitGpuExecutor executor(renderer);
            std::vector<Capture> captures;
            for (const auto count : samples) {
                executor.set_sample_count(count);
                Capture capture;
                assert(executor.capture_rgba8(plan, scene->snapshot(), options.width, options.height,
                           clear, capture.pixels, post) == NKGPU_OK);
                nkscene::PickResult picked;
                capture.picked_first = !nodes.empty() &&
                    executor.pick_pixel(plan, scene->snapshot(), options.width, options.height,
                                        options.width / 2, options.height / 2, &picked) == NKGPU_OK &&
                    picked.node == nodes[0];
                captures.push_back(std::move(capture));
            }
            return captures;
        };
        const auto render_pixels = [&](const std::vector<Quad> &quads, bool reverse_creation,
                                       std::array<float, 4> clear) {
            return render_samples(quads, reverse_creation, clear, {1})[0].pixels;
        };
        const auto render = [&](const std::vector<Quad> &quads,
                                bool reverse_creation) -> std::array<std::uint8_t, 4> {
            const auto pixels = render_pixels(quads, reverse_creation, {0.0f, 0.0f, 0.0f, 1.0f});
            const auto at = (static_cast<std::size_t>(options.height / 2) * options.width +
                             options.width / 2) * 4;
            return {pixels[at], pixels[at + 1], pixels[at + 2], pixels[at + 3]};
        };
        const std::array<float, 3> red{1.0f, 0.0f, 0.0f}, green{0.0f, 1.0f, 0.0f},
            blue{0.0f, 0.0f, 1.0f};

        // Lighting leaves a floor of about 70 in the other channels, so compare against that.
        // Depth convention: the smaller clip z is nearer, so an opaque surface at -0.3 wins.
        const auto near_green = render({{green, 1.0f, -0.3f, true}, {red, 1.0f, 0.3f, true}}, false);
        assert(near_green[1] > 200 && near_green[0] < 100);

        const auto opaque_only = render({{green, 1.0f, 0.3f, true}}, false);
        assert(opaque_only[1] > 200 && opaque_only[0] < 100);
        // A surface with no opacity is invisible, and does not hide what is behind it.
        const auto ghost = render({{green, 1.0f, 0.3f, true}, {red, 0.0f, -0.3f, false}}, false);
        assert(ghost == opaque_only);
        // Half opacity in front blends with what lies behind.
        const auto half = render({{green, 1.0f, 0.3f, true}, {red, 0.5f, -0.3f, false}}, false);
        assert(half[0] > 110 && half[0] < 220 && half[1] > 110 && half[1] < opaque_only[1]);
        // Translucent color over the empty background fades toward the clear color.
        const auto over_black = render({{red, 0.5f, 0.0f, false}}, false);
        assert(over_black[0] > 90 && over_black[0] < 200 && over_black[1] < 60);
        // An opaque surface in front hides a translucent one behind it.
        const auto hidden = render({{green, 1.0f, -0.3f, true}, {red, 0.5f, 0.3f, false}}, false);
        assert(hidden[1] > 200 && hidden[0] < 100);

        // Two translucent surfaces blend far to near, whatever order they were created in.
        const std::vector<Quad> red_in_front{{red, 0.5f, -0.3f, false}, {blue, 0.5f, 0.3f, false}};
        const std::vector<Quad> blue_in_front{{red, 0.5f, 0.3f, false}, {blue, 0.5f, -0.3f, false}};
        const auto red_front = render(red_in_front, false);
        const auto blue_front = render(blue_in_front, false);
        assert(red_front[0] > red_front[2] && blue_front[2] > blue_front[0]);
        assert(render(red_in_front, true) == red_front);
        assert(render(blue_in_front, true) == blue_front);

        // A surface with no alpha shows nothing, its edge strokes included: on a light
        // background the image matches an empty scene, while a visible quad differs.
        const std::array<float, 4> light{0.9f, 0.9f, 0.9f, 1.0f};
        const auto empty_scene = render_pixels({}, false, light);
        assert(render_pixels({{red, 0.0f, 0.0f, false}}, false, light) == empty_scene);
        assert(render_pixels({{red, 1.0f, 0.0f, true}}, false, light) != empty_scene);

        // A surface drawn from an index buffer does not stop its own outline from drawing: the
        // outline's pipeline is not indexed, and a stale index buffer left bound would make the
        // GPU layer reject it.
        assert(render_pixels({{red, 1.0f, 0.0f, true}}, false, light) !=
               render_samples({{red, 1.0f, 0.0f, true}}, false, light, {1}, false)[0].pixels);

        // Multisampling. A capture is single-sample until a count is asked for, and the count
        // is clamped to what the GPU can render and resolve.
        nkscene::NativeKitGpuExecutor probe(renderer);
        const auto max_samples = probe.max_sample_count();
        assert(max_samples >= 1 && probe.sample_count() == 1);
        assert(probe.set_sample_count(1) == 1 && probe.set_sample_count(0) == 1);
        if (max_samples >= 2) {
            for (const std::uint32_t asked : {2u, 3u, 4u, 5u, 1000u}) {
                const auto got = probe.set_sample_count(asked);
                assert(got >= 2 && got <= asked && got <= max_samples && (got & (got - 1)) == 0);
            }
            const auto partial = [](const std::vector<std::uint8_t> &pixels) {
                std::size_t count = 0;
                for (std::size_t at = 3; at < pixels.size(); at += 4)
                    if (pixels[at] > 8 && pixels[at] < 247)
                        ++count;
                return count;
            };
            const auto middle = [&](const std::vector<std::uint8_t> &pixels) {
                const auto at = (static_cast<std::size_t>(options.height / 2) * options.width +
                                 options.width / 2) * 4;
                return std::array<std::uint8_t, 4>{pixels[at], pixels[at + 1], pixels[at + 2],
                                                   pixels[at + 3]};
            };
            // A tilted opaque quad on a transparent background, with its outline off so the only
            // edges are the geometry's own. Changing the count while running rebuilds the
            // targets and pipelines, and returning to one sample reproduces the first image.
            const std::array<float, 4> transparent{0.0f, 0.0f, 0.0f, 0.0f};
            const std::vector<Quad> tilted{{red, 1.0f, 0.0f, true, 0.35f}};
            const auto runs = render_samples(tilted, false, transparent, {1, 4, 1, 2, 4}, false);
            assert(partial(runs[0].pixels) == 0);
            assert(partial(runs[1].pixels) > 8);
            assert(runs[2].pixels == runs[0].pixels);
            assert(partial(runs[3].pixels) > 4);
            assert(runs[4].pixels == runs[1].pixels);
            // The interior is untouched, and an edge pixel never has a channel above its alpha
            // because the resolve of a transparent background is premultiplied.
            assert(middle(runs[1].pixels) == middle(runs[0].pixels));
            for (std::size_t at = 0; at + 3 < runs[1].pixels.size(); at += 4)
                assert(runs[1].pixels[at] <= runs[1].pixels[at + 3] + 2 &&
                       runs[1].pixels[at + 1] <= runs[1].pixels[at + 3] + 2 &&
                       runs[1].pixels[at + 2] <= runs[1].pixels[at + 3] + 2);
            // Picking is exact with multisampling on, since it never resolves.
            assert(runs[0].picked_first && runs[1].picked_first && runs[3].picked_first);

            // A translucent surface over an opaque one blends the same in its interior.
            const std::vector<Quad> layered{{green, 1.0f, 0.3f, true, 0.35f},
                                            {red, 0.5f, -0.3f, false, 0.35f}};
            const auto blended = render_samples(layered, false, transparent, {1, 4}, false);
            const auto plain = middle(blended[0].pixels), smooth = middle(blended[1].pixels);
            for (std::size_t channel = 0; channel < 4; ++channel)
                assert(std::abs(int(plain[channel]) - int(smooth[channel])) <= 2);

            // The workplane grid and the outlines share the multisampled pass, and post-processing
            // reads the resolved image.
            render_samples(tilted, false, light, {4}, true, true);
            nkscene::RgbaPostProcess dim;
            dim.gain = 0.5f;
            const auto undimmed = render_samples(tilted, false, light, {4}, true)[0];
            const auto dimmed = render_samples(tilted, false, light, {4}, true, false, dim)[0];
            assert(middle(dimmed.pixels)[0] + 20 < middle(undimmed.pixels)[0]);
        }
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
