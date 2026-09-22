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
    configure.add_source_entity(occurrence, EntityId{101});
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
    camera_config.projection.width = 16;
    camera_config.projection.height = 16;
    camera_config.projection.fov_y = 1.5707963267948966f;
    camera_config.projection.near_plane = 0.1f;
    camera_config.projection.far_plane = 10.0f;
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

    SensorConfig runtime_sensor_config = sensor_config;
    runtime_sensor_config.id = 55;
    runtime_sensor_config.frame = 15;
    runtime_sensor_config.timing.update_rate_hz = 10.0;
    auto runtime_camera = std::make_shared<CameraSensor>(runtime_sensor_config, camera_config);
    SensorRuntime runtime;
    assert(runtime.add(
        runtime_camera,
        [runtime_camera, scene, &adapter](const SensorTick &runtime_tick)
            -> std::optional<SensorMeasurement> {
            std::optional<CameraFrame> runtime_frame;
            if (adapter.capture(*runtime_camera, runtime_tick, scene->snapshot(), {},
                                runtime_frame) != NKGPU_OK ||
                !runtime_frame)
                return std::nullopt;
            return SensorMeasurement{*runtime_frame};
        }));

    auto runtime_measurements = runtime.poll(0.0);
    assert(runtime_measurements.size() == 1);
    const auto &runtime_frame = std::get<CameraFrame>(runtime_measurements.front());
    assert(runtime_frame.header.sensor == 55);
    assert(runtime_frame.header.sequence == 0);
    assert(runtime_frame.header.capture_time == 0.0);
    const auto *runtime_center = runtime_frame.pixel(8, 8);
    assert(runtime_center);
    assert(runtime_center[0] > 200 && runtime_center[1] < 20 && runtime_center[2] < 20);

    runtime_measurements = runtime.poll(0.1);
    assert(runtime_measurements.size() == 1);
    const auto &second_runtime_frame = std::get<CameraFrame>(runtime_measurements.front());
    assert(second_runtime_frame.header.sequence == 1);
    assert(second_runtime_frame.header.capture_time == 0.1);

    SensorConfig processed_sensor_config;
    processed_sensor_config.id = 53;
    processed_sensor_config.frame = 14;
    CameraConfig processed_camera_config = camera_config;
    processed_camera_config.post_process.gain = 0.5f;
    processed_camera_config.post_process.quantization = 0.25f;
    CameraSensor processed_camera(processed_sensor_config, processed_camera_config);
    const auto processed_tick = processed_camera.trigger(1.25);
    assert(processed_tick.has_value());
    std::optional<CameraFrame> processed_frame;
    assert(adapter.capture(processed_camera, *processed_tick, scene->snapshot(), {},
                           processed_frame) == NKGPU_OK);
    assert(processed_frame.has_value());
    const auto *processed_center = processed_frame->pixel(8, 8);
    assert(processed_center);
    auto expected_processed_pixels = frame->rgba8;
    processed_camera.apply_post_process_cpu(expected_processed_pixels,
                                             processed_tick->header.sequence);
    const auto *expected_processed_center =
        expected_processed_pixels.data() + (8u * 16u + 8u) * 4u;
    assert(std::abs(static_cast<int>(processed_center[0]) - expected_processed_center[0]) <= 1);
    assert(processed_center[0] > 90 && processed_center[0] < 160);
    assert(processed_center[1] < 20 && processed_center[2] < 20 && processed_center[3] > 200);

    SensorConfig dropout_sensor_config = processed_sensor_config;
    dropout_sensor_config.id = 54;
    CameraConfig dropout_camera_config = camera_config;
    dropout_camera_config.post_process.dropout_probability = 1.0f;
    CameraSensor dropout_camera(dropout_sensor_config, dropout_camera_config);
    const auto dropout_tick = dropout_camera.trigger(1.25);
    assert(dropout_tick.has_value());
    std::optional<CameraFrame> dropout_frame;
    assert(adapter.capture(dropout_camera, *dropout_tick, scene->snapshot(), {}, dropout_frame) ==
           NKGPU_OK);
    assert(dropout_frame.has_value());
    const auto *dropout_center = dropout_frame->pixel(8, 8);
    assert(dropout_center);
    assert(dropout_center[0] == 0 && dropout_center[1] == 0 && dropout_center[2] == 0 &&
           dropout_center[3] == 255);

    SensorConfig depth_sensor_config;
    depth_sensor_config.id = 51;
    depth_sensor_config.frame = 12;
    DepthConfig depth_config;
    depth_config.projection.width = 16;
    depth_config.projection.height = 16;
    depth_config.projection.fov_y = 1.5707963267948966f;
    depth_config.projection.near_plane = 0.1f;
    depth_config.projection.far_plane = 10.0f;
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

    SensorConfig segmentation_sensor_config;
    segmentation_sensor_config.id = 52;
    segmentation_sensor_config.frame = 13;
    SegmentationConfig segmentation_config;
    segmentation_config.projection.width = 16;
    segmentation_config.projection.height = 16;
    segmentation_config.projection.fov_y = 1.5707963267948966f;
    segmentation_config.projection.near_plane = 0.1f;
    segmentation_config.projection.far_plane = 10.0f;
    segmentation_config.background_label = 999;
    SegmentationSensor segmentation(segmentation_sensor_config, segmentation_config);
    const auto segmentation_tick = segmentation.trigger(1.25);
    assert(segmentation_tick.has_value());
    std::optional<SegmentationFrame> segmentation_frame;
    assert(adapter.capture_segmentation(segmentation, *segmentation_tick, scene->snapshot(), {},
                                        segmentation_frame) == NKGPU_OK);
    assert(segmentation_frame.has_value());
    assert(segmentation_frame->header.capture_time == 1.25);
    assert(*segmentation_frame->pixel(8, 8) == 101);
    assert(*segmentation_frame->pixel(0, 0) == 999);

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
