#include "nativekit_sensor_render.hpp"

#include "nativekit.h"
#include "nativekit_window.h"
#include "scene_internal.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

namespace {

using namespace nkscene;
using namespace nksensor;
using namespace nksensor::render;

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

std::shared_ptr<Scene> make_colored_scene() {
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {2.0f, -3.0f, -3.0f};
    geometry_resource.bounds.maximum = {2.0f, 3.0f, 3.0f};
    geometry_resource.edit_payload().vertices = {
        GeometryVertex{{2.0f, -3.0f, -3.0f}},
        GeometryVertex{{2.0f, 3.0f, -3.0f}},
        GeometryVertex{{2.0f, 0.0f, 3.0f}}};

    const auto material = scene->reserve_material_id();
    auto &material_resource = scene->material_store().create(material);
    auto &material_state = material_resource.edit_state();
    material_state.base_color = {0.0f, 0.0f, 0.0f, 1.0f};
    material_state.emissive = {1.0f, 0.0f, 0.0f};

    const auto occurrence = scene->reserve_occurrence_id();
    Transaction create(scene);
    create.add_create(occurrence);
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    configure.add_geometry(occurrence, geometry);
    configure.add_material(occurrence, material);
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();
    return scene;
}

void captures_emissive_triangle() {
    nk_window window{};
    nk_surface surface{};
    nkgpu_renderer renderer{};

    nk_window_options options{};
    options.struct_size = sizeof(options);
    options.width = 32;
    options.height = 32;
    options.title = "NativeKit SensorKit RGB camera test";
    assert(nk_window_create(&options, &window) == NK_OK);
    assert(nkgpu_surface_create(window, options.width, options.height, &surface) == NKGPU_OK);
    assert(wait_for_surface(surface));
    assert(nkgpu_renderer_create(surface, &renderer) == NKGPU_OK);

    auto scene = make_colored_scene();
    SensorConfig sensor_config;
    sensor_config.id = 50;
    sensor_config.frame = 11;
    CameraConfig camera_config;
    camera_config.width = 16;
    camera_config.height = 16;
    camera_config.fov_y = 1.5707963267948966f;
    camera_config.near_plane = 0.1f;
    camera_config.far_plane = 10.0f;
    CameraSensor camera(sensor_config, camera_config);
    const auto tick = camera.trigger(1.25);
    assert(tick.has_value());

    SceneCameraAdapter adapter(renderer);
    std::optional<CameraFrame> frame;
    assert(adapter.capture(camera, *tick, scene->snapshot(), {}, frame) == NKGPU_OK);
    assert(frame.has_value());
    assert(frame->header.capture_time == 1.25);
    assert(frame->width == 16 && frame->height == 16);
    assert(frame->rgba8.size() == 16u * 16u * 4u);
    const auto *center = frame->pixel(8, 8);
    assert(center);
    assert(center[0] > 200 && center[1] < 20 && center[2] < 20 && center[3] > 200);

    SensorConfig depth_sensor_config;
    depth_sensor_config.id = 51;
    depth_sensor_config.frame = 12;
    DepthConfig depth_config;
    depth_config.width = 16;
    depth_config.height = 16;
    depth_config.fov_y = 1.5707963267948966f;
    depth_config.near_plane = 0.1f;
    depth_config.far_plane = 10.0f;
    DepthSensor depth(depth_sensor_config, depth_config);
    const auto depth_tick = depth.trigger(1.25);
    assert(depth_tick.has_value());
    std::optional<DepthFrame> depth_frame;
    assert(adapter.capture_depth(depth, *depth_tick, scene->snapshot(), {}, depth_frame) ==
           NKGPU_OK);
    assert(depth_frame.has_value());
    assert(depth_frame->header.capture_time == 1.25);
    assert(std::abs(*depth_frame->pixel(8, 8) - 2.0f) < 0.05f);
    assert(std::abs(*depth_frame->pixel(0, 0) - 10.0f) < 0.01f);

    assert(nkgpu_renderer_destroy(renderer) == NKGPU_OK);
    assert(nkgpu_surface_destroy(surface) == NKGPU_OK);
    assert(nk_window_destroy(window) == NK_OK);
}

} // namespace

int main() {
    nk_init_options init{};
    init.struct_size = sizeof(init);
    init.api_version = NK_API_VERSION;
    if (nk_init(&init) != NK_OK)
        return 1;
    captures_emissive_triangle();
    nk_shutdown();
    return 0;
}
