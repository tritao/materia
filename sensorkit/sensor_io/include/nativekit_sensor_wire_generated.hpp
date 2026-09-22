#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace nksensor::wire::generated {

enum class MessageType : std::uint8_t {
    camera_frame = 1,
    depth_frame = 2,
    segmentation_frame = 3,
    imu_sample = 4,
    lidar_scan = 5,
};

enum class PixelFormat : std::uint8_t {
    rgba8 = 1,
    r32f_le = 2,
    r32u_le = 3,
    u64_le = 4,
};

inline constexpr std::uint8_t current_frame_version = 1;

inline constexpr std::size_t imusampledata_size = 192;
inline constexpr std::size_t lidarreturn_size = 12;

inline constexpr std::size_t imu_packed_value_count = 24;
inline constexpr std::size_t imu_packed_data_size = 192;
inline constexpr std::size_t lidar_packed_return_size = 12;

struct ImuSampleData {
    std::array<double, 3> angular_velocity{};
    std::array<double, 3> linear_acceleration{};
    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 9> linear_acceleration_covariance{};
};

struct LidarReturn {
    float range{};
    float intensity{};
    std::uint8_t hit{};
    std::array<std::uint8_t, 3> reserved{};
};

struct PackedFrameMessage {
    std::uint8_t schema_version = 1;
    MessageType message_type = {};
    std::int64_t sensor = {};
    std::int64_t sequence = {};
    double capture_time = {};
    double delivery_time = {};
    std::int64_t frame = {};
    std::uint32_t width = {};
    std::uint32_t height = {};
    std::uint32_t stride = {};
    PixelFormat pixel_format = {};
    std::span<const std::uint8_t> data = {};
};

struct ImuSampleMessage {
    std::uint8_t schema_version = 1;
    MessageType message_type = static_cast<MessageType>(4);
    std::int64_t sensor = {};
    std::int64_t sequence = {};
    double capture_time = {};
    double delivery_time = {};
    std::int64_t frame = {};
    std::span<const std::uint8_t> data = {};
};

struct LidarScanMessage {
    std::uint8_t schema_version = 1;
    MessageType message_type = static_cast<MessageType>(5);
    std::int64_t sensor = {};
    std::int64_t sequence = {};
    double capture_time = {};
    double delivery_time = {};
    std::int64_t frame = {};
    std::uint32_t horizontal_count = {};
    std::uint32_t vertical_count = {};
    std::uint32_t return_stride = 12;
    std::span<const std::uint8_t> data = {};
};

} // namespace nksensor::wire::generated
