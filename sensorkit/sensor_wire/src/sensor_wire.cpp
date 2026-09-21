#include "nativekit_sensor_wire.hpp"
#include "msgpack.hpp"

#include <algorithm>
#include <array>
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
constexpr std::size_t packed_frame_field_count = 12;
constexpr std::size_t imu_sample_field_count = 8;
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

void append_le_double(std::vector<std::uint8_t> &bytes, double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    append_le64(bytes, bits);
}

bool read_le_double(std::span<const std::uint8_t> bytes, std::size_t offset,
                    double &value) {
    std::uint64_t bits = 0;
    if (!read_le64(bytes, offset, bits))
        return false;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&value, &bits, sizeof(value));
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
    if (data.size() != imu_packed_data_size) {
        set_error(error, "IMU packed payload must contain exactly 24 binary64 values");
        return false;
    }
    for (std::size_t index = 0; index < imu_packed_value_count; ++index) {
        double value = 0.0;
        if (!read_le_double(data, index * sizeof(double), value)) {
            set_error(error, "IMU packed payload is truncated");
            return false;
        }
        if (!std::isfinite(value)) {
            set_error(error, "IMU packed values must be finite");
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

bool read_field_integer(MessagePackReader &reader, std::uint64_t &value,
                        std::string *error, std::string_view field) {
    if (!reader.read_nonnegative(value)) {
        set_error(error, std::string(field) + " is not a non-negative integer");
        return false;
    }
    return true;
}

bool read_field_i64(MessagePackReader &reader, std::uint64_t &value, std::string *error,
                    std::string_view field) {
    if (!reader.read_nonnegative_i64(value)) {
        set_error(error, std::string(field) + " is not a supported integer");
        return false;
    }
    return true;
}

bool read_field_u32(MessagePackReader &reader, std::uint32_t &value, std::string *error,
                    std::string_view field) {
    std::uint64_t raw = 0;
    if (!read_field_integer(reader, raw, error, field))
        return false;
    if (raw > std::numeric_limits<std::uint32_t>::max()) {
        set_error(error, std::string(field) + " is out of range");
        return false;
    }
    value = static_cast<std::uint32_t>(raw);
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

    MessagePackWriter payload;
    payload.write_map_header(packed_frame_field_count);

    /* Field IDs are stable Haxeon @:wire identities. Never reuse an ID. */
    payload.write_integer(1);
    payload.write_integer(current_frame_version);
    payload.write_integer(2);
    payload.write_integer(static_cast<std::uint8_t>(frame.type));
    payload.write_integer(3);
    payload.write_integer(frame.header.sensor);
    payload.write_integer(4);
    payload.write_integer(frame.header.sequence);
    payload.write_integer(5);
    payload.write_float64(frame.header.capture_time);
    payload.write_integer(6);
    payload.write_float64(frame.header.delivery_time);
    payload.write_integer(7);
    payload.write_integer(frame.header.frame);
    payload.write_integer(8);
    payload.write_integer(frame.width);
    payload.write_integer(9);
    payload.write_integer(frame.height);
    payload.write_integer(10);
    payload.write_integer(frame.stride);
    payload.write_integer(11);
    payload.write_integer(static_cast<std::uint8_t>(frame.format));
    payload.write_integer(12);
    payload.write_binary(frame.data);

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
    std::uint32_t field_count = 0;
    if (!reader.read_map_size(field_count)) {
        set_error(error, "packed frame is not a MessagePack map");
        return std::nullopt;
    }

    PackedFrameView frame;
    bool has_version = false;
    bool has_type = false;
    bool has_sensor = false;
    bool has_sequence = false;
    bool has_capture = false;
    bool has_delivery = false;
    bool has_frame = false;
    bool has_width = false;
    bool has_height = false;
    bool has_stride = false;
    bool has_format = false;
    bool has_data = false;

    for (std::uint32_t index = 0; index < field_count; ++index) {
        std::uint64_t key = 0;
        if (!reader.read_nonnegative(key)) {
            set_error(error, "packed frame field key is not a non-negative integer");
            return std::nullopt;
        }
        switch (key) {
        case 1: {
            std::uint32_t version = 0;
            if (!read_field_u32(reader, version, error, "schema version") || version != 1) {
                if (version != 1)
                    set_error(error, "unsupported packed frame schema version");
                return std::nullopt;
            }
            has_version = true;
            break;
        }
        case 2: {
            std::uint32_t type = 0;
            if (!read_field_u32(reader, type, error, "message type") ||
                type > std::numeric_limits<std::uint8_t>::max()) {
                if (type > std::numeric_limits<std::uint8_t>::max())
                    set_error(error, "packed frame message type is out of range");
                return std::nullopt;
            }
            frame.type = static_cast<MessageType>(type);
            has_type = true;
            break;
        }
        case 3: {
            std::uint64_t value = 0;
            if (!read_field_i64(reader, value, error, "sensor"))
                return std::nullopt;
            frame.header.sensor = value;
            has_sensor = true;
            break;
        }
        case 4: {
            std::uint64_t value = 0;
            if (!read_field_i64(reader, value, error, "sequence"))
                return std::nullopt;
            frame.header.sequence = value;
            has_sequence = true;
            break;
        }
        case 5:
            if (!reader.read_float(frame.header.capture_time)) {
                set_error(error, "capture time is not a float");
                return std::nullopt;
            }
            has_capture = true;
            break;
        case 6:
            if (!reader.read_float(frame.header.delivery_time)) {
                set_error(error, "delivery time is not a float");
                return std::nullopt;
            }
            has_delivery = true;
            break;
        case 7: {
            std::uint64_t value = 0;
            if (!read_field_i64(reader, value, error, "frame"))
                return std::nullopt;
            frame.header.frame = value;
            has_frame = true;
            break;
        }
        case 8:
            if (!read_field_u32(reader, frame.width, error, "width"))
                return std::nullopt;
            has_width = true;
            break;
        case 9:
            if (!read_field_u32(reader, frame.height, error, "height"))
                return std::nullopt;
            has_height = true;
            break;
        case 10:
            if (!read_field_u32(reader, frame.stride, error, "stride"))
                return std::nullopt;
            has_stride = true;
            break;
        case 11: {
            std::uint32_t value = 0;
            if (!read_field_u32(reader, value, error, "pixel format") ||
                value > std::numeric_limits<std::uint8_t>::max()) {
                if (value > std::numeric_limits<std::uint8_t>::max())
                    set_error(error, "packed frame pixel format is out of range");
                return std::nullopt;
            }
            frame.format = static_cast<PixelFormat>(value);
            has_format = true;
            break;
        }
        case 12:
            if (!reader.read_binary_view(frame.data)) {
                set_error(error, "packed frame data is not MessagePack binary");
                return std::nullopt;
            }
            has_data = true;
            break;
        default:
            if (!reader.skip()) {
                set_error(error, "packed frame contains an invalid unknown field");
                return std::nullopt;
            }
            break;
        }
    }

    if (!reader.at_end()) {
        set_error(error, "packed frame MessagePack value has trailing bytes");
        return std::nullopt;
    }
    if (!has_version || !has_type || !has_sensor || !has_sequence || !has_capture ||
        !has_delivery || !has_frame || !has_width || !has_height || !has_stride ||
        !has_format || !has_data) {
        set_error(error, "packed frame is missing a required field");
        return std::nullopt;
    }
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

    /* Keep this layout explicit: it is the stable binary contract shared by
     * C++, Haxe, and any other transport consumer. Matrix3 is row-major. */
    const std::array<double, imu_packed_value_count> values{
        sample.angular_velocity.x,
        sample.angular_velocity.y,
        sample.angular_velocity.z,
        sample.linear_acceleration.x,
        sample.linear_acceleration.y,
        sample.linear_acceleration.z,
        sample.angular_velocity_covariance.values[0],
        sample.angular_velocity_covariance.values[1],
        sample.angular_velocity_covariance.values[2],
        sample.angular_velocity_covariance.values[3],
        sample.angular_velocity_covariance.values[4],
        sample.angular_velocity_covariance.values[5],
        sample.angular_velocity_covariance.values[6],
        sample.angular_velocity_covariance.values[7],
        sample.angular_velocity_covariance.values[8],
        sample.linear_acceleration_covariance.values[0],
        sample.linear_acceleration_covariance.values[1],
        sample.linear_acceleration_covariance.values[2],
        sample.linear_acceleration_covariance.values[3],
        sample.linear_acceleration_covariance.values[4],
        sample.linear_acceleration_covariance.values[5],
        sample.linear_acceleration_covariance.values[6],
        sample.linear_acceleration_covariance.values[7],
        sample.linear_acceleration_covariance.values[8]};

    std::vector<std::uint8_t> data;
    data.reserve(imu_packed_data_size);
    for (const auto value : values) {
        if (!std::isfinite(value)) {
            set_error(error, "IMU sample values must be finite");
            return std::nullopt;
        }
        append_le_double(data, value);
    }

    MessagePackWriter payload;
    payload.write_map_header(imu_sample_field_count);

    /* Fields 1-7 deliberately match PackedFrame's shared sample header. */
    payload.write_integer(1);
    payload.write_integer(current_frame_version);
    payload.write_integer(2);
    payload.write_integer(static_cast<std::uint8_t>(MessageType::imu_sample));
    payload.write_integer(3);
    payload.write_integer(sample.header.sensor);
    payload.write_integer(4);
    payload.write_integer(sample.header.sequence);
    payload.write_integer(5);
    payload.write_float64(sample.header.capture_time);
    payload.write_integer(6);
    payload.write_float64(sample.header.delivery_time);
    payload.write_integer(7);
    payload.write_integer(sample.header.frame);
    payload.write_integer(8);
    payload.write_binary(data);

    return encode_hmpk_payload(payload, error, max_messagepack_bytes);
}

std::optional<ImuSampleView> view_imu_sample(
    std::span<const std::uint8_t> encoded, std::string *error,
    std::size_t max_messagepack_bytes) {
    const auto payload = messagepack_payload(encoded, max_messagepack_bytes, error);
    if (!payload.has_value())
        return std::nullopt;

    MessagePackReader reader(*payload);
    std::uint32_t field_count = 0;
    if (!reader.read_map_size(field_count)) {
        set_error(error, "IMU sample is not a MessagePack map");
        return std::nullopt;
    }

    ImuSampleView sample;
    bool has_version = false;
    bool has_type = false;
    bool has_sensor = false;
    bool has_sequence = false;
    bool has_capture = false;
    bool has_delivery = false;
    bool has_frame = false;
    bool has_data = false;

    for (std::uint32_t index = 0; index < field_count; ++index) {
        std::uint64_t key = 0;
        if (!reader.read_nonnegative(key)) {
            set_error(error, "IMU sample field key is not a non-negative integer");
            return std::nullopt;
        }
        switch (key) {
        case 1: {
            std::uint32_t version = 0;
            if (!read_field_u32(reader, version, error, "schema version") || version != 1) {
                if (version != 1)
                    set_error(error, "unsupported IMU sample schema version");
                return std::nullopt;
            }
            has_version = true;
            break;
        }
        case 2: {
            std::uint32_t type = 0;
            if (!read_field_u32(reader, type, error, "message type"))
                return std::nullopt;
            if (type != static_cast<std::uint8_t>(MessageType::imu_sample)) {
                set_error(error, "MessagePack value is not an IMU sample");
                return std::nullopt;
            }
            has_type = true;
            break;
        }
        case 3:
            if (!read_field_i64(reader, sample.header.sensor, error, "sensor"))
                return std::nullopt;
            has_sensor = true;
            break;
        case 4:
            if (!read_field_i64(reader, sample.header.sequence, error, "sequence"))
                return std::nullopt;
            has_sequence = true;
            break;
        case 5:
            if (!reader.read_float(sample.header.capture_time)) {
                set_error(error, "capture time is not a float");
                return std::nullopt;
            }
            has_capture = true;
            break;
        case 6:
            if (!reader.read_float(sample.header.delivery_time)) {
                set_error(error, "delivery time is not a float");
                return std::nullopt;
            }
            has_delivery = true;
            break;
        case 7:
            if (!read_field_i64(reader, sample.header.frame, error, "frame"))
                return std::nullopt;
            has_frame = true;
            break;
        case 8:
            if (!reader.read_binary_view(sample.data)) {
                set_error(error, "IMU sample data is not MessagePack binary");
                return std::nullopt;
            }
            has_data = true;
            break;
        default:
            if (!reader.skip()) {
                set_error(error, "IMU sample contains an invalid unknown field");
                return std::nullopt;
            }
            break;
        }
    }

    if (!reader.at_end()) {
        set_error(error, "IMU sample MessagePack value has trailing bytes");
        return std::nullopt;
    }
    if (!has_version || !has_type || !has_sensor || !has_sequence || !has_capture ||
        !has_delivery || !has_frame || !has_data) {
        set_error(error, "IMU sample is missing a required field");
        return std::nullopt;
    }
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

    std::array<double, imu_packed_value_count> values{};
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!read_le_double(view->data, index * sizeof(double), values[index])) {
            set_error(error, "IMU packed payload is truncated");
            return std::nullopt;
        }
    }

    ImuSample sample;
    sample.header = view->header;
    sample.angular_velocity = {values[0], values[1], values[2]};
    sample.linear_acceleration = {values[3], values[4], values[5]};
    std::copy(values.begin() + 6, values.begin() + 15,
              sample.angular_velocity_covariance.values.begin());
    std::copy(values.begin() + 15, values.end(),
              sample.linear_acceleration_covariance.values.begin());
    return sample;
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
            else {
                set_error(error, "LiDAR wire encoding is not implemented yet");
                return std::nullopt;
            }
        },
        measurement);
}

} // namespace nksensor::wire
