#include "sensor_wire_codec.hpp"
#include "msgpack.hpp"

#include <array>
#include <bit>
#include <limits>
#include <string>

namespace nksensor::wire::generated::detail {
std::array<std::uint8_t, imusampledata_size> pack(const ImuSampleData &value) {
    std::array<std::uint8_t, imusampledata_size> bytes{};
    std::size_t offset = 0;
    for (std::size_t index = 0; index < 3; ++index) {
        const auto bits_angular_velocity = std::bit_cast<std::uint64_t>(value.angular_velocity[index]);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity >> 0);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity >> 8);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity >> 16);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity >> 24);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity >> 32);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity >> 40);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity >> 48);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity >> 56);
    }
    for (std::size_t index = 0; index < 3; ++index) {
        const auto bits_linear_acceleration = std::bit_cast<std::uint64_t>(value.linear_acceleration[index]);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration >> 0);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration >> 8);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration >> 16);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration >> 24);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration >> 32);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration >> 40);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration >> 48);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration >> 56);
    }
    for (std::size_t index = 0; index < 9; ++index) {
        const auto bits_angular_velocity_covariance = std::bit_cast<std::uint64_t>(value.angular_velocity_covariance[index]);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity_covariance >> 0);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity_covariance >> 8);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity_covariance >> 16);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity_covariance >> 24);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity_covariance >> 32);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity_covariance >> 40);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity_covariance >> 48);
        bytes[offset++] = static_cast<std::uint8_t>(bits_angular_velocity_covariance >> 56);
    }
    for (std::size_t index = 0; index < 9; ++index) {
        const auto bits_linear_acceleration_covariance = std::bit_cast<std::uint64_t>(value.linear_acceleration_covariance[index]);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration_covariance >> 0);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration_covariance >> 8);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration_covariance >> 16);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration_covariance >> 24);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration_covariance >> 32);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration_covariance >> 40);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration_covariance >> 48);
        bytes[offset++] = static_cast<std::uint8_t>(bits_linear_acceleration_covariance >> 56);
    }
    return bytes;
}

bool unpack(std::span<const std::uint8_t> bytes, ImuSampleData &value) {
    if (bytes.size() != imusampledata_size) return false;
    std::size_t offset = 0;
    for (std::size_t index = 0; index < 3; ++index) {
        std::uint64_t bits_angular_velocity = 0;
        bits_angular_velocity |= static_cast<std::uint64_t>(bytes[offset++]) << 0;
        bits_angular_velocity |= static_cast<std::uint64_t>(bytes[offset++]) << 8;
        bits_angular_velocity |= static_cast<std::uint64_t>(bytes[offset++]) << 16;
        bits_angular_velocity |= static_cast<std::uint64_t>(bytes[offset++]) << 24;
        bits_angular_velocity |= static_cast<std::uint64_t>(bytes[offset++]) << 32;
        bits_angular_velocity |= static_cast<std::uint64_t>(bytes[offset++]) << 40;
        bits_angular_velocity |= static_cast<std::uint64_t>(bytes[offset++]) << 48;
        bits_angular_velocity |= static_cast<std::uint64_t>(bytes[offset++]) << 56;
        value.angular_velocity[index] = std::bit_cast<double>(bits_angular_velocity);
    }
    for (std::size_t index = 0; index < 3; ++index) {
        std::uint64_t bits_linear_acceleration = 0;
        bits_linear_acceleration |= static_cast<std::uint64_t>(bytes[offset++]) << 0;
        bits_linear_acceleration |= static_cast<std::uint64_t>(bytes[offset++]) << 8;
        bits_linear_acceleration |= static_cast<std::uint64_t>(bytes[offset++]) << 16;
        bits_linear_acceleration |= static_cast<std::uint64_t>(bytes[offset++]) << 24;
        bits_linear_acceleration |= static_cast<std::uint64_t>(bytes[offset++]) << 32;
        bits_linear_acceleration |= static_cast<std::uint64_t>(bytes[offset++]) << 40;
        bits_linear_acceleration |= static_cast<std::uint64_t>(bytes[offset++]) << 48;
        bits_linear_acceleration |= static_cast<std::uint64_t>(bytes[offset++]) << 56;
        value.linear_acceleration[index] = std::bit_cast<double>(bits_linear_acceleration);
    }
    for (std::size_t index = 0; index < 9; ++index) {
        std::uint64_t bits_angular_velocity_covariance = 0;
        bits_angular_velocity_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 0;
        bits_angular_velocity_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 8;
        bits_angular_velocity_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 16;
        bits_angular_velocity_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 24;
        bits_angular_velocity_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 32;
        bits_angular_velocity_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 40;
        bits_angular_velocity_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 48;
        bits_angular_velocity_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 56;
        value.angular_velocity_covariance[index] = std::bit_cast<double>(bits_angular_velocity_covariance);
    }
    for (std::size_t index = 0; index < 9; ++index) {
        std::uint64_t bits_linear_acceleration_covariance = 0;
        bits_linear_acceleration_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 0;
        bits_linear_acceleration_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 8;
        bits_linear_acceleration_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 16;
        bits_linear_acceleration_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 24;
        bits_linear_acceleration_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 32;
        bits_linear_acceleration_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 40;
        bits_linear_acceleration_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 48;
        bits_linear_acceleration_covariance |= static_cast<std::uint64_t>(bytes[offset++]) << 56;
        value.linear_acceleration_covariance[index] = std::bit_cast<double>(bits_linear_acceleration_covariance);
    }
    return true;
}

std::array<std::uint8_t, lidarreturn_size> pack(const LidarReturn &value) {
    std::array<std::uint8_t, lidarreturn_size> bytes{};
    std::size_t offset = 0;
    const auto bits_range = std::bit_cast<std::uint32_t>(value.range);
    bytes[offset++] = static_cast<std::uint8_t>(bits_range >> 0);
    bytes[offset++] = static_cast<std::uint8_t>(bits_range >> 8);
    bytes[offset++] = static_cast<std::uint8_t>(bits_range >> 16);
    bytes[offset++] = static_cast<std::uint8_t>(bits_range >> 24);
    const auto bits_intensity = std::bit_cast<std::uint32_t>(value.intensity);
    bytes[offset++] = static_cast<std::uint8_t>(bits_intensity >> 0);
    bytes[offset++] = static_cast<std::uint8_t>(bits_intensity >> 8);
    bytes[offset++] = static_cast<std::uint8_t>(bits_intensity >> 16);
    bytes[offset++] = static_cast<std::uint8_t>(bits_intensity >> 24);
    const auto bits_hit = static_cast<std::uint8_t>(value.hit);
    bytes[offset++] = static_cast<std::uint8_t>(bits_hit >> 0);
    for (std::size_t index = 0; index < 3; ++index) {
        const auto bits_reserved = static_cast<std::uint8_t>(value.reserved[index]);
        bytes[offset++] = static_cast<std::uint8_t>(bits_reserved >> 0);
    }
    return bytes;
}

bool unpack(std::span<const std::uint8_t> bytes, LidarReturn &value) {
    if (bytes.size() != lidarreturn_size) return false;
    std::size_t offset = 0;
    std::uint32_t bits_range = 0;
    bits_range |= static_cast<std::uint32_t>(bytes[offset++]) << 0;
    bits_range |= static_cast<std::uint32_t>(bytes[offset++]) << 8;
    bits_range |= static_cast<std::uint32_t>(bytes[offset++]) << 16;
    bits_range |= static_cast<std::uint32_t>(bytes[offset++]) << 24;
    value.range = std::bit_cast<float>(bits_range);
    std::uint32_t bits_intensity = 0;
    bits_intensity |= static_cast<std::uint32_t>(bytes[offset++]) << 0;
    bits_intensity |= static_cast<std::uint32_t>(bytes[offset++]) << 8;
    bits_intensity |= static_cast<std::uint32_t>(bytes[offset++]) << 16;
    bits_intensity |= static_cast<std::uint32_t>(bytes[offset++]) << 24;
    value.intensity = std::bit_cast<float>(bits_intensity);
    const std::uint8_t bits_hit = bytes[offset++];
    value.hit = static_cast<std::uint8_t>(bits_hit);
    for (std::size_t index = 0; index < 3; ++index) {
        const std::uint8_t bits_reserved = bytes[offset++];
        value.reserved[index] = static_cast<std::uint8_t>(bits_reserved);
    }
    return true;
}

namespace {
void set_error(std::string *error, const char *message) { if (error) *error = message; }
}

void write(::nksensor::wire::detail::MessagePackWriter &writer, const PackedFrameMessage &value) {
    writer.write_map_header(12);
    writer.write_integer(1);
    writer.write_integer(1);
    writer.write_integer(2);
    writer.write_integer(static_cast<std::uint64_t>(value.message_type));
    writer.write_integer(3);
    writer.write_integer(value.sensor);
    writer.write_integer(4);
    writer.write_integer(value.sequence);
    writer.write_integer(5);
    writer.write_float64(static_cast<double>(value.capture_time));
    writer.write_integer(6);
    writer.write_float64(static_cast<double>(value.delivery_time));
    writer.write_integer(7);
    writer.write_integer(value.frame);
    writer.write_integer(8);
    writer.write_integer(value.width);
    writer.write_integer(9);
    writer.write_integer(value.height);
    writer.write_integer(10);
    writer.write_integer(value.stride);
    writer.write_integer(11);
    writer.write_integer(static_cast<std::uint64_t>(value.pixel_format));
    writer.write_integer(12);
    writer.write_binary(value.data);
}

bool read(::nksensor::wire::detail::MessagePackReader &reader, PackedFrameMessage &value, std::string *error) {
    std::uint32_t field_count = 0;
    if (!reader.read_map_size(field_count)) {
        set_error(error, "PackedFrameMessage is not a MessagePack map");
        return false;
    }
    bool has_schema_version = false;
    bool has_message_type = false;
    bool has_sensor = false;
    bool has_sequence = false;
    bool has_capture_time = false;
    bool has_delivery_time = false;
    bool has_frame = false;
    bool has_width = false;
    bool has_height = false;
    bool has_stride = false;
    bool has_pixel_format = false;
    bool has_data = false;
    for (std::uint32_t index = 0; index < field_count; ++index) {
        std::uint64_t key = 0;
        if (!reader.read_nonnegative(key)) {
            set_error(error, "wire field key is not a non-negative integer");
            return false;
        }
        switch (key) {
        case 1: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint8_t>::max()) {
                set_error(error, "invalid constant wire field");
                return false;
            }
            if (raw != 1) {
                set_error(error, "unsupported PackedFrameMessage schema version");
                return false;
            }
            value.schema_version = 1;
            has_schema_version = true;
            break;
        }
        case 2: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint8_t>::max()) {
                set_error(error, "message type is out of range or not an integer");
                return false;
            }
            value.message_type = static_cast<MessageType>(raw);
            has_message_type = true;
            break;
        }
        case 3: {
            if (!reader.read_nonnegative_i64(value.sensor)) {
                set_error(error, "sensor is not a supported integer");
                return false;
            }
            has_sensor = true;
            break;
        }
        case 4: {
            if (!reader.read_nonnegative_i64(value.sequence)) {
                set_error(error, "sequence is not a supported integer");
                return false;
            }
            has_sequence = true;
            break;
        }
        case 5: {
            if (!reader.read_float(value.capture_time)) {
                set_error(error, "capture time is not a float");
                return false;
            }
            has_capture_time = true;
            break;
        }
        case 6: {
            if (!reader.read_float(value.delivery_time)) {
                set_error(error, "delivery time is not a float");
                return false;
            }
            has_delivery_time = true;
            break;
        }
        case 7: {
            if (!reader.read_nonnegative_i64(value.frame)) {
                set_error(error, "frame is not a supported integer");
                return false;
            }
            has_frame = true;
            break;
        }
        case 8: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint32_t>::max()) {
                set_error(error, "width is out of range or not an integer");
                return false;
            }
            value.width = static_cast<std::uint32_t>(raw);
            has_width = true;
            break;
        }
        case 9: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint32_t>::max()) {
                set_error(error, "height is out of range or not an integer");
                return false;
            }
            value.height = static_cast<std::uint32_t>(raw);
            has_height = true;
            break;
        }
        case 10: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint32_t>::max()) {
                set_error(error, "stride is out of range or not an integer");
                return false;
            }
            value.stride = static_cast<std::uint32_t>(raw);
            has_stride = true;
            break;
        }
        case 11: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint8_t>::max()) {
                set_error(error, "pixel format is out of range or not an integer");
                return false;
            }
            value.pixel_format = static_cast<PixelFormat>(raw);
            has_pixel_format = true;
            break;
        }
        case 12: {
            if (!reader.read_binary_view(value.data)) {
                set_error(error, "data is not MessagePack binary");
                return false;
            }
            has_data = true;
            break;
        }
        default:
            if (!reader.skip()) {
                set_error(error, "wire message contains an invalid unknown field");
                return false;
            }
            break;
        }
    }
    if (!reader.at_end()) {
        set_error(error, "PackedFrameMessage MessagePack value has trailing bytes");
        return false;
    }
    if (!has_schema_version || !has_message_type || !has_sensor || !has_sequence || !has_capture_time || !has_delivery_time || !has_frame || !has_width || !has_height || !has_stride || !has_pixel_format || !has_data) {
        set_error(error, "PackedFrameMessage is missing a required field");
        return false;
    }
    return true;
}

void write(::nksensor::wire::detail::MessagePackWriter &writer, const ImuSampleMessage &value) {
    writer.write_map_header(8);
    writer.write_integer(1);
    writer.write_integer(1);
    writer.write_integer(2);
    writer.write_integer(static_cast<std::uint64_t>(static_cast<MessageType>(4)));
    writer.write_integer(3);
    writer.write_integer(value.sensor);
    writer.write_integer(4);
    writer.write_integer(value.sequence);
    writer.write_integer(5);
    writer.write_float64(static_cast<double>(value.capture_time));
    writer.write_integer(6);
    writer.write_float64(static_cast<double>(value.delivery_time));
    writer.write_integer(7);
    writer.write_integer(value.frame);
    writer.write_integer(8);
    writer.write_binary(value.data);
}

bool read(::nksensor::wire::detail::MessagePackReader &reader, ImuSampleMessage &value, std::string *error) {
    std::uint32_t field_count = 0;
    if (!reader.read_map_size(field_count)) {
        set_error(error, "ImuSampleMessage is not a MessagePack map");
        return false;
    }
    bool has_schema_version = false;
    bool has_message_type = false;
    bool has_sensor = false;
    bool has_sequence = false;
    bool has_capture_time = false;
    bool has_delivery_time = false;
    bool has_frame = false;
    bool has_data = false;
    for (std::uint32_t index = 0; index < field_count; ++index) {
        std::uint64_t key = 0;
        if (!reader.read_nonnegative(key)) {
            set_error(error, "wire field key is not a non-negative integer");
            return false;
        }
        switch (key) {
        case 1: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint8_t>::max()) {
                set_error(error, "invalid constant wire field");
                return false;
            }
            if (raw != 1) {
                set_error(error, "unsupported ImuSampleMessage schema version");
                return false;
            }
            value.schema_version = 1;
            has_schema_version = true;
            break;
        }
        case 2: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint8_t>::max()) {
                set_error(error, "invalid constant wire field");
                return false;
            }
            if (raw != 4) {
                set_error(error, "MessagePack value is not a ImuSampleMessage");
                return false;
            }
            value.message_type = static_cast<MessageType>(4);
            has_message_type = true;
            break;
        }
        case 3: {
            if (!reader.read_nonnegative_i64(value.sensor)) {
                set_error(error, "sensor is not a supported integer");
                return false;
            }
            has_sensor = true;
            break;
        }
        case 4: {
            if (!reader.read_nonnegative_i64(value.sequence)) {
                set_error(error, "sequence is not a supported integer");
                return false;
            }
            has_sequence = true;
            break;
        }
        case 5: {
            if (!reader.read_float(value.capture_time)) {
                set_error(error, "capture time is not a float");
                return false;
            }
            has_capture_time = true;
            break;
        }
        case 6: {
            if (!reader.read_float(value.delivery_time)) {
                set_error(error, "delivery time is not a float");
                return false;
            }
            has_delivery_time = true;
            break;
        }
        case 7: {
            if (!reader.read_nonnegative_i64(value.frame)) {
                set_error(error, "frame is not a supported integer");
                return false;
            }
            has_frame = true;
            break;
        }
        case 8: {
            if (!reader.read_binary_view(value.data)) {
                set_error(error, "data is not MessagePack binary");
                return false;
            }
            has_data = true;
            break;
        }
        default:
            if (!reader.skip()) {
                set_error(error, "wire message contains an invalid unknown field");
                return false;
            }
            break;
        }
    }
    if (!reader.at_end()) {
        set_error(error, "ImuSampleMessage MessagePack value has trailing bytes");
        return false;
    }
    if (!has_schema_version || !has_message_type || !has_sensor || !has_sequence || !has_capture_time || !has_delivery_time || !has_frame || !has_data) {
        set_error(error, "ImuSampleMessage is missing a required field");
        return false;
    }
    return true;
}

void write(::nksensor::wire::detail::MessagePackWriter &writer, const LidarScanMessage &value) {
    writer.write_map_header(11);
    writer.write_integer(1);
    writer.write_integer(1);
    writer.write_integer(2);
    writer.write_integer(static_cast<std::uint64_t>(static_cast<MessageType>(5)));
    writer.write_integer(3);
    writer.write_integer(value.sensor);
    writer.write_integer(4);
    writer.write_integer(value.sequence);
    writer.write_integer(5);
    writer.write_float64(static_cast<double>(value.capture_time));
    writer.write_integer(6);
    writer.write_float64(static_cast<double>(value.delivery_time));
    writer.write_integer(7);
    writer.write_integer(value.frame);
    writer.write_integer(8);
    writer.write_integer(value.horizontal_count);
    writer.write_integer(9);
    writer.write_integer(value.vertical_count);
    writer.write_integer(10);
    writer.write_integer(12);
    writer.write_integer(11);
    writer.write_binary(value.data);
}

bool read(::nksensor::wire::detail::MessagePackReader &reader, LidarScanMessage &value, std::string *error) {
    std::uint32_t field_count = 0;
    if (!reader.read_map_size(field_count)) {
        set_error(error, "LidarScanMessage is not a MessagePack map");
        return false;
    }
    bool has_schema_version = false;
    bool has_message_type = false;
    bool has_sensor = false;
    bool has_sequence = false;
    bool has_capture_time = false;
    bool has_delivery_time = false;
    bool has_frame = false;
    bool has_horizontal_count = false;
    bool has_vertical_count = false;
    bool has_return_stride = false;
    bool has_data = false;
    for (std::uint32_t index = 0; index < field_count; ++index) {
        std::uint64_t key = 0;
        if (!reader.read_nonnegative(key)) {
            set_error(error, "wire field key is not a non-negative integer");
            return false;
        }
        switch (key) {
        case 1: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint8_t>::max()) {
                set_error(error, "invalid constant wire field");
                return false;
            }
            if (raw != 1) {
                set_error(error, "unsupported LidarScanMessage schema version");
                return false;
            }
            value.schema_version = 1;
            has_schema_version = true;
            break;
        }
        case 2: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint8_t>::max()) {
                set_error(error, "invalid constant wire field");
                return false;
            }
            if (raw != 5) {
                set_error(error, "MessagePack value is not a LidarScanMessage");
                return false;
            }
            value.message_type = static_cast<MessageType>(5);
            has_message_type = true;
            break;
        }
        case 3: {
            if (!reader.read_nonnegative_i64(value.sensor)) {
                set_error(error, "sensor is not a supported integer");
                return false;
            }
            has_sensor = true;
            break;
        }
        case 4: {
            if (!reader.read_nonnegative_i64(value.sequence)) {
                set_error(error, "sequence is not a supported integer");
                return false;
            }
            has_sequence = true;
            break;
        }
        case 5: {
            if (!reader.read_float(value.capture_time)) {
                set_error(error, "capture time is not a float");
                return false;
            }
            has_capture_time = true;
            break;
        }
        case 6: {
            if (!reader.read_float(value.delivery_time)) {
                set_error(error, "delivery time is not a float");
                return false;
            }
            has_delivery_time = true;
            break;
        }
        case 7: {
            if (!reader.read_nonnegative_i64(value.frame)) {
                set_error(error, "frame is not a supported integer");
                return false;
            }
            has_frame = true;
            break;
        }
        case 8: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint32_t>::max()) {
                set_error(error, "horizontal count is out of range or not an integer");
                return false;
            }
            value.horizontal_count = static_cast<std::uint32_t>(raw);
            has_horizontal_count = true;
            break;
        }
        case 9: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint32_t>::max()) {
                set_error(error, "vertical count is out of range or not an integer");
                return false;
            }
            value.vertical_count = static_cast<std::uint32_t>(raw);
            has_vertical_count = true;
            break;
        }
        case 10: {
            std::uint64_t raw = 0;
            if (!reader.read_nonnegative(raw) || raw > std::numeric_limits<std::uint32_t>::max()) {
                set_error(error, "invalid constant wire field");
                return false;
            }
            if (raw != 12) {
                set_error(error, "LiDAR return stride is not the supported packed format");
                return false;
            }
            value.return_stride = 12;
            has_return_stride = true;
            break;
        }
        case 11: {
            if (!reader.read_binary_view(value.data)) {
                set_error(error, "data is not MessagePack binary");
                return false;
            }
            has_data = true;
            break;
        }
        default:
            if (!reader.skip()) {
                set_error(error, "wire message contains an invalid unknown field");
                return false;
            }
            break;
        }
    }
    if (!reader.at_end()) {
        set_error(error, "LidarScanMessage MessagePack value has trailing bytes");
        return false;
    }
    if (!has_schema_version || !has_message_type || !has_sensor || !has_sequence || !has_capture_time || !has_delivery_time || !has_frame || !has_horizontal_count || !has_vertical_count || !has_return_stride || !has_data) {
        set_error(error, "LidarScanMessage is missing a required field");
        return false;
    }
    return true;
}

} // namespace nksensor::wire::generated::detail
