#pragma once

#include "nativekit_sensor_core.hpp"
#include "nativekit_sensor_wire_generated.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#if defined(_WIN32)
#if defined(NKSENSOR_WIRE_STATIC)
#define NKSENSOR_WIRE_API
#elif defined(NKSENSOR_WIRE_BUILDING_LIBRARY)
#define NKSENSOR_WIRE_API __declspec(dllexport)
#else
#define NKSENSOR_WIRE_API __declspec(dllimport)
#endif
#else
#define NKSENSOR_WIRE_API __attribute__((visibility("default")))
#endif

namespace nksensor::wire {

/**
 * The MessagePack payload is framed with Haxeon's existing HMPK envelope:
 * magic, version, flags, big-endian payload length, then exactly one value.
 */
using generated::MessageType;
using generated::PixelFormat;

constexpr std::uint8_t current_frame_version = generated::current_frame_version;
constexpr std::size_t frame_header_size = 10;
constexpr std::size_t default_max_messagepack_bytes = 16 * 1024 * 1024;
constexpr std::size_t imu_packed_value_count = generated::imu_packed_value_count;
constexpr std::size_t imu_packed_data_size = generated::imu_packed_data_size;
constexpr std::size_t lidar_packed_return_size = generated::lidar_packed_return_size;

/** A non-owning packed image-like payload view into an encoded HMPK message. */
struct PackedFrameView {
    SensorSampleHeader header;
    MessageType type = MessageType::camera_frame;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t stride = 0;
    PixelFormat format = PixelFormat::rgba8;
    std::span<const std::uint8_t> data;
};

/** An owning packed-frame value detached from its encoded input buffer. */
struct PackedFrame {
    SensorSampleHeader header;
    MessageType type = MessageType::camera_frame;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t stride = 0;
    PixelFormat format = PixelFormat::rgba8;
    std::vector<std::uint8_t> data;
};

/** A non-owning packed IMU payload view into an encoded HMPK message. */
struct ImuSampleView {
    SensorSampleHeader header;
    /** 24 little-endian IEEE-754 binary64 values; see encode_imu_sample. */
    std::span<const std::uint8_t> data;
};

/**
 * A non-owning packed LiDAR payload view. Each return is 12 bytes:
 * little-endian float32 range, little-endian float32 intensity, one hit flag,
 * and three reserved bytes.
 */
struct LidarScanView {
    SensorSampleHeader header;
    std::uint32_t horizontal_count = 0;
    std::uint32_t vertical_count = 0;
    std::uint32_t return_stride = static_cast<std::uint32_t>(lidar_packed_return_size);
    std::span<const std::uint8_t> data;
};

/**
 * Encode one tightly packed packed-frame payload.
 *
 * The MessagePack value is an integer-keyed map so it can be represented by a
 * Haxeon `@:wire` record. The binary payload is field 12; it is never
 * expanded into one MessagePack value per pixel.
 */
NKSENSOR_WIRE_API std::optional<std::vector<std::uint8_t>> encode_packed_frame(
    const PackedFrameView &frame, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/**
 * Validate and inspect one packed-frame message without copying its payload.
 *
 * The returned span remains valid only while `encoded` remains alive and
 * unchanged.
 */
NKSENSOR_WIRE_API std::optional<PackedFrameView> view_packed_frame(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Decode one packed-frame message and own a copy of its binary payload. */
NKSENSOR_WIRE_API std::optional<PackedFrame> decode_packed_frame(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Convenience wrapper for the packed RGBA8 camera representation. */
NKSENSOR_WIRE_API std::optional<std::vector<std::uint8_t>> encode_camera_frame(
    const CameraFrame &frame, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/**
 * Convenience wrapper that requires a camera/RGBA8 packed-frame message.
 */
NKSENSOR_WIRE_API std::optional<PackedFrameView> view_camera_frame(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Decode one camera frame and copy its packed pixels into owned storage. */
NKSENSOR_WIRE_API std::optional<CameraFrame> decode_camera_frame(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Encode metric depth as tightly packed little-endian R32F metres. */
NKSENSOR_WIRE_API std::optional<std::vector<std::uint8_t>> encode_depth_frame(
    const DepthFrame &frame, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Decode a packed little-endian R32F depth frame into owned metric values. */
NKSENSOR_WIRE_API std::optional<DepthFrame> decode_depth_frame(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Encode segmentation labels losslessly as tightly packed little-endian U64. */
NKSENSOR_WIRE_API std::optional<std::vector<std::uint8_t>> encode_segmentation_frame(
    const SegmentationFrame &frame, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Decode a packed little-endian U64 segmentation frame into owned labels. */
NKSENSOR_WIRE_API std::optional<SegmentationFrame> decode_segmentation_frame(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/**
 * Encode an IMU sample as 24 little-endian IEEE-754 binary64 values:
 * angular velocity xyz, linear acceleration xyz, angular-velocity covariance
 * row-major, and linear-acceleration covariance row-major.
 */
NKSENSOR_WIRE_API std::optional<std::vector<std::uint8_t>> encode_imu_sample(
    const ImuSample &sample, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Validate and inspect an IMU packet without copying its packed payload. */
NKSENSOR_WIRE_API std::optional<ImuSampleView> view_imu_sample(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Decode an IMU packet into the core fixed-size measurement type. */
NKSENSOR_WIRE_API std::optional<ImuSample> decode_imu_sample(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Encode LiDAR ranges and intensities as compact little-endian float32 data. */
NKSENSOR_WIRE_API std::optional<std::vector<std::uint8_t>> encode_lidar_scan(
    const LidarScan &scan, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Validate and inspect a LiDAR packet without copying its packed payload. */
NKSENSOR_WIRE_API std::optional<LidarScanView> view_lidar_scan(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Decode a LiDAR packet into the core scan type. */
NKSENSOR_WIRE_API std::optional<LidarScan> decode_lidar_scan(
    std::span<const std::uint8_t> encoded, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

/** Encode any currently supported runtime measurement for external transport. */
NKSENSOR_WIRE_API std::optional<std::vector<std::uint8_t>> encode_sensor_measurement(
    const SensorMeasurement &measurement, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = default_max_messagepack_bytes);

} // namespace nksensor::wire
