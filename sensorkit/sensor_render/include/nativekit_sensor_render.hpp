#pragma once

#include "nativekit_scene_render.h"
#include "nativekit_sensor_core.hpp"

#include <optional>

#if defined(_WIN32)
#if defined(NKSENSOR_RENDER_STATIC)
#define NKSENSOR_RENDER_API
#elif defined(NKSENSOR_RENDER_BUILDING_LIBRARY)
#define NKSENSOR_RENDER_API __declspec(dllexport)
#else
#define NKSENSOR_RENDER_API __declspec(dllimport)
#endif
#else
#define NKSENSOR_RENDER_API __attribute__((visibility("default")))
#endif

namespace nksensor::render {

/**
 * Captures RGB frames from one immutable SceneKit snapshot using an existing
 * NativeKit GPU renderer. The renderer is borrowed and must outlive this
 * adapter.
 */
class NKSENSOR_RENDER_API SceneCameraAdapter {
public:
    explicit SceneCameraAdapter(nkgpu_renderer renderer) noexcept : executor_(renderer) {}

    SceneCameraAdapter(const SceneCameraAdapter &) = delete;
    SceneCameraAdapter &operator=(const SceneCameraAdapter &) = delete;
    SceneCameraAdapter(SceneCameraAdapter &&) noexcept = default;
    SceneCameraAdapter &operator=(SceneCameraAdapter &&) noexcept = default;

    void set_renderer(nkgpu_renderer renderer) noexcept { executor_.set_renderer(renderer); }
    nkgpu_renderer renderer() const noexcept { return executor_.renderer(); }
    nkgpu_result last_result() const noexcept { return executor_.last_result(); }

    /** Render one camera sample in the supplied world pose.
     * Camera frames use SceneKit's convention: local +X is forward and +Z is up. */
    nkgpu_result capture(CameraSensor &sensor, const SensorTick &tick,
                         const nkscene::SceneSnapshot &snapshot, const Pose &camera_pose,
                         std::optional<CameraFrame> &out_frame);

    /** Render metric camera-forward depth in the supplied world pose. */
    nkgpu_result capture_depth(DepthSensor &sensor, const SensorTick &tick,
                               const nkscene::SceneSnapshot &snapshot, const Pose &camera_pose,
                               std::optional<DepthFrame> &out_frame);

private:
    nkscene::NativeKitGpuExecutor executor_;
};

} // namespace nksensor::render
