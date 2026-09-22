#include "nativekit_sensor_wire.hpp"
#include "msgpack.hpp"
#include "sensor_wire_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>
#include <type_traits>

namespace nksensor::wire {
namespace {

using detail::MessagePackReader;
using detail::MessagePackWriter;

constexpr std::uint8_t hmpk_magic[] = {'H', 'M', 'P', 'K'};
constexpr std::uint64_t signed_int64_max =
    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());

void set_error(std::string *error, std::string_view message) {
    if (error != nullptr)
        *error = std::string(message);
}

bool element_count(std::uint32_t width, std::uint32_t height, std::size_t &count,
                   std::string *error, std::string_view name) {
    if (height != 0 && width > std::numeric_limits<std::size_t>::max() / height) {
        set_error(error, std::string(name) + " dimensions overflow its element count");
        return false;
    }
    count = static_cast<std::size_t>(width) * height;
    return true;
}

void append_le32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8));
    bytes.push_back(static_cast<std::uint8_t>(value >> 16));
    bytes.push_back(static_cast<std::uint8_t>(value >> 24));
}

void append_le64(std::vector<std::uint8_t> &bytes, std::uint64_t value) {
    append_le32(bytes, static_cast<std::uint32_t>(value));
    append_le32(bytes, static_cast<std::uint32_t>(value >> 32));
}

bool read_le32(std::span<const std::uint8_t> bytes, std::size_t offset,
               std::uint32_t &value) {
    if (offset > bytes.size() || bytes.size() - offset < 4)
        return false;
    value = static_cast<std::uint32_t>(bytes[offset]) |
            (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
            (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
            (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
    return true;
}

bool read_le64(std::span<const std::uint8_t> bytes, std::size_t offset,
               std::uint64_t &value) {
    if (offset > bytes.size() || bytes.size() - offset < 8)
        return false;
    std::uint32_t low = 0;
    std::uint32_t high = 0;
    if (!read_le32(bytes, offset, low) || !read_le32(bytes, offset + 4, high))
        return false;
    value = (static_cast<std::uint64_t>(high) << 32) | low;
    return true;
}

bool validate_sensor_header(const SensorSampleHeader &header, std::string *error,
                           std::string_view name) {
    if (header.sensor > signed_int64_max || header.sequence > signed_int64_max ||
        header.frame > signed_int64_max) {
        set_error(error, std::string(name) + " identifiers must fit Haxe Int64");
        return false;
    }
    if (!std::isfinite(header.capture_time) || !std::isfinite(header.delivery_time)) {
        set_error(error, std::string(name) + " timestamps must be finite");
        return false;
    }
    return true;
}

bool validate_imu_data(std::span<const std::uint8_t> data, std::string *error) {
    generated::ImuSampleData values;
    if (!generated::detail::unpack(data, values)) {
        set_error(error, "IMU packed payload does not match the generated layout");
        return false;
    }
    const auto all_finite = [](const auto &items) {
        return std::all_of(items.begin(), items.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!all_finite(values.angular_velocity) || !all_finite(values.linear_acceleration) ||
        !all_finite(values.angular_velocity_covariance) ||
        !all_finite(values.linear_acceleration_covariance)) {
        set_error(error, "IMU packed values must be finite");
        return false;
    }
    return true;
}

bool validate_lidar_data(std::span<const std::uint8_t> data,
                         std::uint32_t horizontal_count,
                         std::uint32_t vertical_count,
                         std::uint32_t return_stride,
                         std::size_t max_bytes,
                         std::string *error) {
    if (return_stride != lidar_packed_return_size) {
        set_error(error, "LiDAR return stride is not the supported packed format");
        return false;
    }

    std::size_t count = 0;
    if (!element_count(horizontal_count, vertical_count, count, error, "LiDAR"))
        return false;
    if (count > std::numeric_limits<std::size_t>::max() / lidar_packed_return_size) {
        set_error(error, "LiDAR dimensions overflow its packed payload size");
        return false;
    }
    const auto expected = count * lidar_packed_return_size;
    if (expected > max_bytes || data.size() != expected) {
        set_error(error, "LiDAR packed payload size does not match dimensions");
        return false;
    }

    for (std::size_t index = 0; index < count; ++index) {
        generated::LidarReturn value;
        const auto offset = index * lidar_packed_return_size;
        if (!generated::detail::unpack(data.subspan(offset, lidar_packed_return_size), value)) {
            set_error(error, "LiDAR packed payload is truncated");
            return false;
        }
        if (!std::isfinite(value.range) || !std::isfinite(value.intensity)) {
            set_error(error, "LiDAR range and intensity values must be finite");
            return false;
        }
        if (value.hit > 1) {
            set_error(error, "LiDAR hit flag is invalid");
            return false;
        }
    }
    return true;
}

bool packed_type_matches(const PackedFrameView &frame, MessageType type,
                         PixelFormat format, std::string *error,
                         std::string_view name) {
    if (frame.type != type || frame.format != format) {
        set_error(error, std::string("packed frame is not a ") + std::string(name));
        return false;
    }
    return true;
}

std::size_t bytes_per_pixel(PixelFormat format) noexcept {
    switch (format) {
    case PixelFormat::rgba8:
    case PixelFormat::r32f_le:
    case PixelFormat::r32u_le:
        return 4;
    case PixelFormat::u64_le:
        return 8;
    }
    return 0;
}

bool valid_type_and_format(const PackedFrameView &frame, std::string *error) {
    const bool valid =
        (frame.type == MessageType::camera_frame && frame.format == PixelFormat::rgba8) ||
        (frame.type == MessageType::depth_frame && frame.format == PixelFormat::r32f_le) ||
        (frame.type == MessageType::segmentation_frame &&
         (frame.format == PixelFormat::r32u_le || frame.format == PixelFormat::u64_le));
    if (!valid) {
        set_error(error, "packed frame type and pixel format are incompatible");
        return false;
    }
    return true;
}

bool validate_packed_frame(const PackedFrameView &frame, std::size_t max_bytes,
                           std::string *error) {
    if (!valid_type_and_format(frame, error))
        return false;

    const auto pixel_bytes = bytes_per_pixel(frame.format);
    if (pixel_bytes == 0 || frame.width > std::numeric_limits<std::size_t>::max() / pixel_bytes) {
        set_error(error, "packed frame dimensions overflow its payload size");
        return false;
    }
    const auto row_bytes = static_cast<std::size_t>(frame.width) * pixel_bytes;
    if (frame.height != 0 && row_bytes > std::numeric_limits<std::size_t>::max() /
                                  static_cast<std::size_t>(frame.height)) {
        set_error(error, "packed frame dimensions overflow its payload size");
        return false;
    }
    const auto expected = row_bytes * frame.height;
    if (row_bytes > std::numeric_limits<std::uint32_t>::max() ||
        frame.stride != static_cast<std::uint32_t>(row_bytes)) {
        set_error(error, "packed frame stride is not tightly packed");
        return false;
    }
    if (expected > max_bytes || frame.data.size() != expected) {
        set_error(error, "packed frame payload size does not match dimensions");
        return false;
    }
    if (!std::isfinite(frame.header.capture_time) ||
        !std::isfinite(frame.header.delivery_time)) {
        set_error(error, "packed frame timestamps must be finite");
        return false;
    }
    return true;
}

std::optional<std::span<const std::uint8_t>> messagepack_payload(
    std::span<const std::uint8_t> encoded, std::size_t max_bytes, std::string *error) {
    if (encoded.size() < frame_header_size) {
        set_error(error, "MessagePack frame is truncated");
        return std::nullopt;
    }
    if (!std::equal(std::begin(hmpk_magic), std::end(hmpk_magic), encoded.begin())) {
        set_error(error, "invalid MessagePack frame magic");
        return std::nullopt;
    }
    if (encoded[4] != current_frame_version) {
        set_error(error, "unsupported MessagePack frame version");
        return std::nullopt;
    }
    if (encoded[5] != 0) {
        set_error(error, "unsupported MessagePack frame flags");
        return std::nullopt;
    }
    const auto payload_size = (static_cast<std::uint32_t>(encoded[6]) << 24) |
                              (static_cast<std::uint32_t>(encoded[7]) << 16) |
                              (static_cast<std::uint32_t>(encoded[8]) << 8) |
                              encoded[9];
    if (payload_size > max_bytes) {
        set_error(error, "MessagePack frame payload exceeds configured limit");
        return std::nullopt;
    }
    if (encoded.size() != frame_header_size + payload_size) {
        set_error(error, "MessagePack frame length mismatch");
        return std::nullopt;
    }
    return encoded.subspan(frame_header_size, payload_size);
}

std::optional<std::vector<std::uint8_t>> encode_hmpk_payload(
    const MessagePackWriter &payload, std::string *error, std::size_t max_messagepack_bytes) {
    if (payload.bytes().size() > max_messagepack_bytes ||
        payload.bytes().size() > std::numeric_limits<std::uint32_t>::max()) {
        set_error(error, "MessagePack payload exceeds configured limit");
        return std::nullopt;
    }

    std::vector<std::uint8_t> encoded(frame_header_size + payload.bytes().size());
    std::copy(std::begin(hmpk_magic), std::end(hmpk_magic), encoded.begin());
    encoded[4] = current_frame_version;
    encoded[5] = 0;
    const auto payload_size = static_cast<std::uint32_t>(payload.bytes().size());
    encoded[6] = static_cast<std::uint8_t>(payload_size >> 24);
    encoded[7] = static_cast<std::uint8_t>(payload_size >> 16);
    encoded[8] = static_cast<std::uint8_t>(payload_size >> 8);
    encoded[9] = static_cast<std::uint8_t>(payload_size);
    std::copy(payload.bytes().begin(), payload.bytes().end(),
              encoded.begin() + frame_header_size);
    return encoded;
}

} // namespace

std::optional<std::vector<std::uint8_t>> encode_packed_frame(
    const PackedFrameView &frame, std::string *error, std::size_t max_messagepack_bytes) {
    if (!validate_packed_frame(frame, max_messagepack_bytes, error))
        return std::nullopt;
    if (!validate_sensor_header(frame.header, error, "packed frame"))
        return std::nullopt;
    if (frame.data.size() > std::numeric_limits<std::uint32_t>::max()) {
        set_error(error, "packed frame payload is too large for MessagePack binary");
        return std::nullopt;
    }

    generated::PackedFrameMessage message;
    message.message_type = frame.type;
    message.sensor = frame.header.sensor;
    message.sequence = frame.header.sequence;
    message.capture_time = frame.header.capture_time;
    message.delivery_time = frame.header.delivery_time;
    message.frame = frame.header.frame;
    message.width = frame.width;
    message.height = frame.height;
    message.stride = frame.stride;
    message.pixel_format = frame.format;
    message.data = frame.data;
    MessagePackWriter payload;
    generated::detail::write(payload, message);

    return encode_hmpk_payload(payload, error, max_messagepack_bytes);
}

std::optional<std::vector<std::uint8_t>> encode_camera_frame(
    const CameraFrame &frame, std::string *error, std::size_t max_messagepack_bytes) {
    if (frame.width > std::numeric_limits<std::uint32_t>::max() / 4) {
        set_error(error, "camera width is too large for tightly packed RGBA8");
        return std::nullopt;
    }
    PackedFrameView packed;
    packed.header = frame.header;
    packed.type = MessageType::camera_frame;
    packed.format = PixelFormat::rgba8;
    packed.width = frame.width;
    packed.height = frame.height;
    packed.stride = frame.width * 4;
    packed.data = frame.rgba8;
    return encode_packed_frame(packed, error, max_messagepack_bytes);
}

std::optional<PackedFrameView> view_packed_frame(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto payload = messagepack_payload(encoded, max_messagepack_bytes, error);
    if (!payload.has_value())
        return std::nullopt;

    MessagePackReader reader(*payload);
    generated::PackedFrameMessage message;
    if (!generated::detail::read(reader, message, error))
        return std::nullopt;

    PackedFrameView frame;
    frame.type = message.message_type;
    frame.header.sensor = message.sensor;
    frame.header.sequence = message.sequence;
    frame.header.capture_time = message.capture_time;
    frame.header.delivery_time = message.delivery_time;
    frame.header.frame = message.frame;
    frame.width = message.width;
    frame.height = message.height;
    frame.stride = message.stride;
    frame.format = message.pixel_format;
    frame.data = message.data;
    if (!validate_packed_frame(frame, max_messagepack_bytes, error))
        return std::nullopt;
    return frame;
}

std::optional<PackedFrame> decode_packed_frame(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto view = view_packed_frame(encoded, error, max_messagepack_bytes);
    if (!view.has_value())
        return std::nullopt;

    PackedFrame frame;
    frame.header = view->header;
    frame.type = view->type;
    frame.width = view->width;
    frame.height = view->height;
    frame.stride = view->stride;
    frame.format = view->format;
    frame.data.assign(view->data.begin(), view->data.end());
    return frame;
}

std::optional<PackedFrameView> view_camera_frame(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto frame = view_packed_frame(encoded, error, max_messagepack_bytes);
    if (!frame.has_value())
        return std::nullopt;
    if (frame->type != MessageType::camera_frame || frame->format != PixelFormat::rgba8) {
        set_error(error, "packed frame is not a camera RGBA8 frame");
        return std::nullopt;
    }
    return frame;
}

std::optional<CameraFrame> decode_camera_frame(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto view = view_camera_frame(encoded, error, max_messagepack_bytes);
    if (!view.has_value())
        return std::nullopt;

    CameraFrame frame;
    frame.header = view->header;
    frame.width = view->width;
    frame.height = view->height;
    frame.rgba8.assign(view->data.begin(), view->data.end());
    return frame;
}

std::optional<std::vector<std::uint8_t>> encode_depth_frame(
    const DepthFrame &frame, std::string *error, std::size_t max_messagepack_bytes) {
    if (frame.width > std::numeric_limits<std::uint32_t>::max() / 4) {
        set_error(error, "depth width is too large for tightly packed R32F");
        return std::nullopt;
    }
    std::size_t count = 0;
    if (!element_count(frame.width, frame.height, count, error, "depth"))
        return std::nullopt;
    if (frame.meters.size() != count) {
        set_error(error, "depth value count does not match dimensions");
        return std::nullopt;
    }
    if (count > std::numeric_limits<std::size_t>::max() / 4) {
        set_error(error, "depth payload size overflows R32F storage");
        return std::nullopt;
    }
    if (count > max_messagepack_bytes / 4) {
        set_error(error, "depth payload exceeds configured limit");
        return std::nullopt;
    }

    std::vector<std::uint8_t> data;
    data.reserve(count * 4);
    for (const auto value : frame.meters) {
        std::uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(bits));
        append_le32(data, bits);
    }

    PackedFrameView packed;
    packed.header = frame.header;
    packed.type = MessageType::depth_frame;
    packed.format = PixelFormat::r32f_le;
    packed.width = frame.width;
    packed.height = frame.height;
    packed.stride = frame.width * 4;
    packed.data = data;
    return encode_packed_frame(packed, error, max_messagepack_bytes);
}

std::optional<DepthFrame> decode_depth_frame(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto view = view_packed_frame(encoded, error, max_messagepack_bytes);
    if (!view.has_value() ||
        !packed_type_matches(*view, MessageType::depth_frame, PixelFormat::r32f_le, error,
                             "R32F depth frame"))
        return std::nullopt;

    std::size_t count = 0;
    if (!element_count(view->width, view->height, count, error, "depth"))
        return std::nullopt;
    if (view->data.size() != count * 4) {
        set_error(error, "depth R32F payload size does not match dimensions");
        return std::nullopt;
    }

    DepthFrame frame;
    frame.header = view->header;
    frame.width = view->width;
    frame.height = view->height;
    frame.meters.resize(count);
    for (std::size_t index = 0; index < count; ++index) {
        std::uint32_t bits = 0;
        if (!read_le32(view->data, index * 4, bits)) {
            set_error(error, "depth R32F payload is truncated");
            return std::nullopt;
        }
        std::memcpy(&frame.meters[index], &bits, sizeof(bits));
    }
    return frame;
}

std::optional<std::vector<std::uint8_t>> encode_segmentation_frame(
    const SegmentationFrame &frame, std::string *error,
    std::size_t max_messagepack_bytes) {
    if (frame.width > std::numeric_limits<std::uint32_t>::max() / 8) {
        set_error(error, "segmentation width is too large for tightly packed U64");
        return std::nullopt;
    }
    std::size_t count = 0;
    if (!element_count(frame.width, frame.height, count, error, "segmentation"))
        return std::nullopt;
    if (frame.labels.size() != count) {
        set_error(error, "segmentation label count does not match dimensions");
        return std::nullopt;
    }
    if (count > std::numeric_limits<std::size_t>::max() / 8) {
        set_error(error, "segmentation payload size overflows U64 storage");
        return std::nullopt;
    }
    if (count > max_messagepack_bytes / 8) {
        set_error(error, "segmentation payload exceeds configured limit");
        return std::nullopt;
    }

    std::vector<std::uint8_t> data;
    data.reserve(count * 8);
    for (const auto value : frame.labels)
        append_le64(data, value);

    PackedFrameView packed;
    packed.header = frame.header;
    packed.type = MessageType::segmentation_frame;
    packed.format = PixelFormat::u64_le;
    packed.width = frame.width;
    packed.height = frame.height;
    packed.stride = frame.width * 8;
    packed.data = data;
    return encode_packed_frame(packed, error, max_messagepack_bytes);
}

std::optional<SegmentationFrame> decode_segmentation_frame(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto view = view_packed_frame(encoded, error, max_messagepack_bytes);
    if (!view.has_value() ||
        !packed_type_matches(*view, MessageType::segmentation_frame, PixelFormat::u64_le,
                             error, "U64 segmentation frame"))
        return std::nullopt;

    std::size_t count = 0;
    if (!element_count(view->width, view->height, count, error, "segmentation"))
        return std::nullopt;
    if (view->data.size() != count * 8) {
        set_error(error, "segmentation U64 payload size does not match dimensions");
        return std::nullopt;
    }

    SegmentationFrame frame;
    frame.header = view->header;
    frame.width = view->width;
    frame.height = view->height;
    frame.labels.resize(count);
    for (std::size_t index = 0; index < count; ++index) {
        if (!read_le64(view->data, index * 8, frame.labels[index])) {
            set_error(error, "segmentation U64 payload is truncated");
            return std::nullopt;
        }
    }
    return frame;
}

std::optional<std::vector<std::uint8_t>> encode_imu_sample(
    const ImuSample &sample, std::string *error, std::size_t max_messagepack_bytes) {
    if (!validate_sensor_header(sample.header, error, "IMU sample"))
        return std::nullopt;

    generated::ImuSampleData packed_data;
    packed_data.angular_velocity = {
        sample.angular_velocity.x,
        sample.angular_velocity.y,
        sample.angular_velocity.z};
    packed_data.linear_acceleration = {
        sample.linear_acceleration.x,
        sample.linear_acceleration.y,
        sample.linear_acceleration.z};
    packed_data.angular_velocity_covariance = sample.angular_velocity_covariance.values;
    packed_data.linear_acceleration_covariance = sample.linear_acceleration_covariance.values;
    const auto all_finite = [](const auto &items) {
        return std::all_of(items.begin(), items.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!all_finite(packed_data.angular_velocity) ||
        !all_finite(packed_data.linear_acceleration) ||
        !all_finite(packed_data.angular_velocity_covariance) ||
        !all_finite(packed_data.linear_acceleration_covariance)) {
        set_error(error, "IMU sample values must be finite");
        return std::nullopt;
    }
    const auto data = generated::detail::pack(packed_data);

    generated::ImuSampleMessage message;
    message.sensor = sample.header.sensor;
    message.sequence = sample.header.sequence;
    message.capture_time = sample.header.capture_time;
    message.delivery_time = sample.header.delivery_time;
    message.frame = sample.header.frame;
    message.data = data;
    MessagePackWriter payload;
    generated::detail::write(payload, message);

    return encode_hmpk_payload(payload, error, max_messagepack_bytes);
}

std::optional<ImuSampleView> view_imu_sample(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto payload = messagepack_payload(encoded, max_messagepack_bytes, error);
    if (!payload.has_value())
        return std::nullopt;

    MessagePackReader reader(*payload);
    generated::ImuSampleMessage message;
    if (!generated::detail::read(reader, message, error))
        return std::nullopt;

    ImuSampleView sample;
    sample.header.sensor = message.sensor;
    sample.header.sequence = message.sequence;
    sample.header.capture_time = message.capture_time;
    sample.header.delivery_time = message.delivery_time;
    sample.header.frame = message.frame;
    sample.data = message.data;
    if (!validate_sensor_header(sample.header, error, "IMU sample") ||
        !validate_imu_data(sample.data, error))
        return std::nullopt;
    return sample;
}

std::optional<ImuSample> decode_imu_sample(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto view = view_imu_sample(encoded, error, max_messagepack_bytes);
    if (!view.has_value())
        return std::nullopt;

    generated::ImuSampleData packed_data;
    if (!generated::detail::unpack(view->data, packed_data)) {
        set_error(error, "IMU packed payload is truncated");
        return std::nullopt;
    }

    ImuSample sample;
    sample.header = view->header;
    sample.angular_velocity = {packed_data.angular_velocity[0],
                               packed_data.angular_velocity[1],
                               packed_data.angular_velocity[2]};
    sample.linear_acceleration = {packed_data.linear_acceleration[0],
                                  packed_data.linear_acceleration[1],
                                  packed_data.linear_acceleration[2]};
    sample.angular_velocity_covariance.values = packed_data.angular_velocity_covariance;
    sample.linear_acceleration_covariance.values = packed_data.linear_acceleration_covariance;
    return sample;
}

std::optional<std::vector<std::uint8_t>> encode_lidar_scan(
    const LidarScan &scan, std::string *error, std::size_t max_messagepack_bytes) {
    if (!validate_sensor_header(scan.header, error, "LiDAR scan"))
        return std::nullopt;

    std::size_t count = 0;
    if (!element_count(scan.horizontal_count, scan.vertical_count, count, error, "LiDAR"))
        return std::nullopt;
    if (scan.returns.size() != count) {
        set_error(error, "LiDAR return count does not match scan dimensions");
        return std::nullopt;
    }
    if (count > std::numeric_limits<std::size_t>::max() / lidar_packed_return_size) {
        set_error(error, "LiDAR payload size overflows packed return storage");
        return std::nullopt;
    }
    if (count > max_messagepack_bytes / lidar_packed_return_size) {
        set_error(error, "LiDAR payload exceeds configured limit");
        return std::nullopt;
    }

    std::vector<std::uint8_t> data;
    data.reserve(count * lidar_packed_return_size);
    for (const auto &value : scan.returns) {
        generated::LidarReturn packed_return;
        packed_return.range = static_cast<float>(value.range);
        packed_return.intensity = static_cast<float>(value.intensity);
        packed_return.hit = value.hit ? 1 : 0;
        if (!std::isfinite(packed_return.range) || !std::isfinite(packed_return.intensity)) {
            set_error(error, "LiDAR range and intensity values must be finite");
            return std::nullopt;
        }
        const auto packed_bytes = generated::detail::pack(packed_return);
        data.insert(data.end(), packed_bytes.begin(), packed_bytes.end());
    }

    generated::LidarScanMessage message;
    message.sensor = scan.header.sensor;
    message.sequence = scan.header.sequence;
    message.capture_time = scan.header.capture_time;
    message.delivery_time = scan.header.delivery_time;
    message.frame = scan.header.frame;
    message.horizontal_count = scan.horizontal_count;
    message.vertical_count = scan.vertical_count;
    message.data = data;
    MessagePackWriter payload;
    generated::detail::write(payload, message);

    return encode_hmpk_payload(payload, error, max_messagepack_bytes);
}

std::optional<LidarScanView> view_lidar_scan(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto payload = messagepack_payload(encoded, max_messagepack_bytes, error);
    if (!payload.has_value())
        return std::nullopt;

    MessagePackReader reader(*payload);
    generated::LidarScanMessage message;
    if (!generated::detail::read(reader, message, error))
        return std::nullopt;

    LidarScanView scan;
    scan.header.sensor = message.sensor;
    scan.header.sequence = message.sequence;
    scan.header.capture_time = message.capture_time;
    scan.header.delivery_time = message.delivery_time;
    scan.header.frame = message.frame;
    scan.horizontal_count = message.horizontal_count;
    scan.vertical_count = message.vertical_count;
    scan.return_stride = message.return_stride;
    scan.data = message.data;
    if (!validate_sensor_header(scan.header, error, "LiDAR scan") ||
        !validate_lidar_data(scan.data, scan.horizontal_count, scan.vertical_count,
                             scan.return_stride, max_messagepack_bytes, error))
        return std::nullopt;
    return scan;
}

std::optional<LidarScan> decode_lidar_scan(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto view = view_lidar_scan(encoded, error, max_messagepack_bytes);
    if (!view.has_value())
        return std::nullopt;

    std::size_t count = 0;
    if (!element_count(view->horizontal_count, view->vertical_count, count, error, "LiDAR"))
        return std::nullopt;

    LidarScan scan;
    scan.header = view->header;
    scan.horizontal_count = view->horizontal_count;
    scan.vertical_count = view->vertical_count;
    scan.returns.resize(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto offset = index * lidar_packed_return_size;
        generated::LidarReturn packed_return;
        if (!generated::detail::unpack(
                view->data.subspan(offset, lidar_packed_return_size), packed_return)) {
            set_error(error, "LiDAR packed payload is truncated");
            return std::nullopt;
        }
        auto &value = scan.returns[index];
        value.hit = packed_return.hit != 0;
        value.range = static_cast<double>(packed_return.range);
        value.intensity = static_cast<double>(packed_return.intensity);
    }
    return scan;
}

std::optional<std::vector<std::uint8_t>> encode_sensor_measurement(
    const SensorMeasurement &measurement, std::string *error,
    std::size_t max_messagepack_bytes) {
    return std::visit(
        [&](const auto &value) -> std::optional<std::vector<std::uint8_t>> {
            using Value = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Value, ImuSample>)
                return encode_imu_sample(value, error, max_messagepack_bytes);
            else if constexpr (std::is_same_v<Value, CameraFrame>)
                return encode_camera_frame(value, error, max_messagepack_bytes);
            else if constexpr (std::is_same_v<Value, DepthFrame>)
                return encode_depth_frame(value, error, max_messagepack_bytes);
            else if constexpr (std::is_same_v<Value, SegmentationFrame>)
                return encode_segmentation_frame(value, error, max_messagepack_bytes);
            else if constexpr (std::is_same_v<Value, LidarScan>)
                return encode_lidar_scan(value, error, max_messagepack_bytes);
            else {
                set_error(error, "sensor measurement wire encoding is not implemented");
                return std::nullopt;
            }
        },
        measurement);
}

} // namespace nksensor::wire
