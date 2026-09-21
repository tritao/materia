#include "nativekit_sensor_render.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace nksensor::render {
namespace {

using Matrix4 = std::array<float, 16>;

struct Vec3 {
    float x;
    float y;
    float z;
};

float dot(Vec3 lhs, Vec3 rhs) noexcept {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 cross(Vec3 lhs, Vec3 rhs) noexcept {
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

Vec3 normalize(Vec3 value) noexcept {
    const auto length = std::sqrt(dot(value, value));
    if (length > 1.0e-6f)
        return {value.x / length, value.y / length, value.z / length};
    return value;
}

Vec3 rotate(Quaternion orientation, Vec3 value) noexcept {
    const auto length = std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                                  orientation.z * orientation.z + orientation.w * orientation.w);
    if (!(length > 1.0e-8) || !std::isfinite(length))
        return value;
    const auto x = static_cast<float>(orientation.x / length);
    const auto y = static_cast<float>(orientation.y / length);
    const auto z = static_cast<float>(orientation.z / length);
    const auto w = static_cast<float>(orientation.w / length);
    const auto tx = 2.0f * (y * value.z - z * value.y);
    const auto ty = 2.0f * (z * value.x - x * value.z);
    const auto tz = 2.0f * (x * value.y - y * value.x);
    return {value.x + w * tx + (y * tz - z * ty),
            value.y + w * ty + (z * tx - x * tz),
            value.z + w * tz + (x * ty - y * tx)};
}

Matrix4 multiply(const Matrix4 &lhs, const Matrix4 &rhs) noexcept {
    Matrix4 result{};
    for (std::size_t column = 0; column < 4; ++column)
        for (std::size_t row = 0; row < 4; ++row)
            for (std::size_t index = 0; index < 4; ++index)
                result[column * 4 + row] +=
                    lhs[index * 4 + row] * rhs[column * 4 + index];
    return result;
}

Matrix4 camera_view(const Pose &pose) noexcept {
    const auto forward = normalize(rotate(pose.orientation, {1.0f, 0.0f, 0.0f}));
    const auto up = normalize(rotate(pose.orientation, {0.0f, 0.0f, 1.0f}));
    const auto right = normalize(cross(forward, up));
    const auto position = Vec3{static_cast<float>(pose.position.x),
                               static_cast<float>(pose.position.y),
                               static_cast<float>(pose.position.z)};
    return {right.x,
            up.x,
            forward.x,
            0.0f,
            right.y,
            up.y,
            forward.y,
            0.0f,
            right.z,
            up.z,
            forward.z,
            0.0f,
            -dot(right, position),
            -dot(up, position),
            -dot(forward, position),
            1.0f};
}

Matrix4 camera_projection(std::uint32_t width, std::uint32_t height, float fov_y,
                          float near_plane, float far_plane) noexcept {
    const auto aspect = static_cast<float>(width) / static_cast<float>(height);
    const auto focal = 1.0f / std::tan(fov_y * 0.5f);
    Matrix4 result{};
    result[0] = focal / aspect;
    result[5] = focal;
    result[10] = (far_plane + near_plane) / (far_plane - near_plane);
    result[11] = 1.0f;
    result[14] = -(2.0f * far_plane * near_plane) / (far_plane - near_plane);
    return result;
}

Matrix4 camera_projection(const CameraConfig &camera) noexcept {
    return camera_projection(camera.width, camera.height, camera.fov_y, camera.near_plane,
                             camera.far_plane);
}

Matrix4 depth_projection(const DepthConfig &depth) noexcept {
    return camera_projection(depth.width, depth.height, depth.fov_y, depth.near_plane,
                             depth.far_plane);
}

Matrix4 segmentation_projection(const SegmentationConfig &segmentation) noexcept {
    return camera_projection(segmentation.width, segmentation.height, segmentation.fov_y,
                             segmentation.near_plane, segmentation.far_plane);
}

float metric_depth(float normalized_depth, const DepthConfig &depth) noexcept {
    if (!std::isfinite(normalized_depth))
        return depth.far_plane;
    const auto depth_buffer = std::clamp(normalized_depth, 0.0f, 1.0f);
    const auto ndc_depth = depth_buffer * 2.0f - 1.0f;
    const auto denominator = depth.far_plane + depth.near_plane -
                             ndc_depth * (depth.far_plane - depth.near_plane);
    if (!(denominator > 0.0f) || !std::isfinite(denominator))
        return depth.far_plane;
    return std::clamp(2.0f * depth.far_plane * depth.near_plane / denominator,
                      depth.near_plane, depth.far_plane);
}

} // namespace

nkgpu_result SceneCameraAdapter::capture(CameraSensor &sensor, const SensorTick &tick,
                                         const nkscene::SceneSnapshot &snapshot,
                                         const Pose &camera_pose,
                                         std::optional<CameraFrame> &out_frame) {
    out_frame.reset();
    if (tick.dropped)
        return NKGPU_OK;

    nkscene::SceneView view;
    view.camera.enabled = true;
    view.camera.view_projection = multiply(camera_projection(sensor.camera_config()),
                                          camera_view(camera_pose));
    const auto plan = nkscene::compile(snapshot, view);

    std::vector<std::uint8_t> pixels;
    const auto &camera = sensor.camera_config();
    nkscene::RgbaPostProcess gpu_post_process;
    gpu_post_process.exposure_stops = camera.post_process.exposure_stops;
    gpu_post_process.gain = camera.post_process.gain;
    gpu_post_process.noise_stddev = camera.post_process.noise_stddev;
    gpu_post_process.quantization = camera.post_process.quantization;
    gpu_post_process.distortion_k1 = camera.post_process.distortion_k1;
    gpu_post_process.distortion_k2 = camera.post_process.distortion_k2;
    gpu_post_process.dropout_probability = camera.post_process.dropout_probability;
    gpu_post_process.seed = sensor.config().seed;
    gpu_post_process.sequence = tick.header.sequence;

    const auto backend = nkgpu_query_backend(executor_.renderer());
    const bool gpu_post_process_supported = backend == NKGPU_BACKEND_GLCORE ||
                                            backend == NKGPU_BACKEND_GLES3;
    const bool use_gpu_post_process = camera.post_process.enabled() &&
                                      gpu_post_process_supported;
    auto result = executor_.capture_rgba8(
        plan, snapshot, camera.width, camera.height, camera.clear_color, pixels,
        use_gpu_post_process ? gpu_post_process : nkscene::RgbaPostProcess{});

    /* A backend may advertise a path but reject a particular shader or image
     * format at runtime. In that case retry the raw capture and preserve the
     * sensor contract with the portable CPU model. Other failures still
     * propagate because hiding device or frame errors would be dangerous. */
    if (result == NKGPU_ERROR_UNSUPPORTED && use_gpu_post_process) {
        result = executor_.capture_rgba8(plan, snapshot, camera.width, camera.height,
                                         camera.clear_color, pixels);
        if (result != NKGPU_OK)
            return result;
        sensor.apply_post_process_cpu(pixels, tick.header.sequence);
    } else if (result != NKGPU_OK) {
        return result;
    } else if (camera.post_process.enabled() && !use_gpu_post_process) {
        sensor.apply_post_process_cpu(pixels, tick.header.sequence);
    }
    out_frame = sensor.sample(tick, pixels);
    return out_frame.has_value() ? NKGPU_OK : NKGPU_ERROR_INVALID_ARGUMENT;
}

nkgpu_result SceneCameraAdapter::capture_depth(
    DepthSensor &sensor, const SensorTick &tick, const nkscene::SceneSnapshot &snapshot,
    const Pose &camera_pose, std::optional<DepthFrame> &out_frame) {
    out_frame.reset();
    if (tick.dropped)
        return NKGPU_OK;

    nkscene::SceneView view;
    view.camera.enabled = true;
    view.camera.view_projection =
        multiply(depth_projection(sensor.depth_config()), camera_view(camera_pose));
    const auto plan = nkscene::compile(snapshot, view);

    std::vector<float> normalized_depth;
    const auto &depth = sensor.depth_config();
    const auto result = executor_.capture_depth(plan, snapshot, depth.width, depth.height,
                                                normalized_depth);
    if (result != NKGPU_OK)
        return result;

    std::vector<float> meters;
    meters.reserve(normalized_depth.size());
    for (const auto value : normalized_depth)
        meters.push_back(metric_depth(value, depth));
    out_frame = sensor.sample(tick, meters);
    return out_frame.has_value() ? NKGPU_OK : NKGPU_ERROR_INVALID_ARGUMENT;
}

nkgpu_result SceneCameraAdapter::capture_segmentation(
    SegmentationSensor &sensor, const SensorTick &tick,
    const nkscene::SceneSnapshot &snapshot, const Pose &camera_pose,
    std::optional<SegmentationFrame> &out_frame) {
    out_frame.reset();
    if (tick.dropped)
        return NKGPU_OK;

    nkscene::SceneView view;
    view.camera.enabled = true;
    view.camera.view_projection =
        multiply(segmentation_projection(sensor.segmentation_config()), camera_view(camera_pose));
    const auto plan = nkscene::compile(snapshot, view);

    std::vector<std::uint32_t> pick_ids;
    const auto &segmentation = sensor.segmentation_config();
    const auto result = executor_.capture_pick_ids(plan, snapshot, segmentation.width,
                                                   segmentation.height, pick_ids);
    if (result != NKGPU_OK)
        return result;

    std::vector<std::uint64_t> labels(pick_ids.size(), segmentation.background_label);
    for (std::size_t index = 0; index < pick_ids.size(); ++index) {
        const auto pick_id = pick_ids[index];
        if (pick_id == 0 || pick_id > plan.items().size())
            continue;
        const auto &item = plan.items()[pick_id - 1];
        const auto *occurrence = snapshot.find(item.occurrence);
        if (!occurrence)
            continue;
        labels[index] = occurrence->source.valid() ? occurrence->source.value
                                                   : occurrence->occurrence.value;
    }

    out_frame = sensor.sample(tick, labels);
    return out_frame.has_value() ? NKGPU_OK : NKGPU_ERROR_INVALID_ARGUMENT;
}

} // namespace nksensor::render
