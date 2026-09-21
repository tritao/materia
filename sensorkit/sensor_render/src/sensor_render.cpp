#include "nativekit_sensor_render.hpp"

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

Matrix4 camera_projection(const CameraConfig &camera) noexcept {
    const auto aspect = static_cast<float>(camera.width) / static_cast<float>(camera.height);
    const auto focal = 1.0f / std::tan(camera.fov_y * 0.5f);
    Matrix4 result{};
    result[0] = focal / aspect;
    result[5] = focal;
    result[10] = (camera.far_plane + camera.near_plane) /
                 (camera.far_plane - camera.near_plane);
    result[11] = 1.0f;
    result[14] = -(2.0f * camera.far_plane * camera.near_plane) /
                 (camera.far_plane - camera.near_plane);
    return result;
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
    const auto result = executor_.capture_rgba8(plan, snapshot, camera.width, camera.height,
                                                camera.clear_color, pixels);
    if (result != NKGPU_OK)
        return result;
    out_frame = sensor.sample(tick, pixels);
    return out_frame.has_value() ? NKGPU_OK : NKGPU_ERROR_INVALID_ARGUMENT;
}

} // namespace nksensor::render
