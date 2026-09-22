#include "msgpack.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace nksensor::wire::detail {
namespace {

constexpr std::uint64_t signed_int64_max =
    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());

} // namespace

void MessagePackWriter::write_byte(std::uint8_t value) {
    bytes_.push_back(value);
}

void MessagePackWriter::write_be16(std::uint16_t value) {
    write_byte(static_cast<std::uint8_t>(value >> 8));
    write_byte(static_cast<std::uint8_t>(value));
}

void MessagePackWriter::write_be32(std::uint32_t value) {
    write_byte(static_cast<std::uint8_t>(value >> 24));
    write_byte(static_cast<std::uint8_t>(value >> 16));
    write_byte(static_cast<std::uint8_t>(value >> 8));
    write_byte(static_cast<std::uint8_t>(value));
}

void MessagePackWriter::write_be64(std::uint64_t value) {
    write_be32(static_cast<std::uint32_t>(value >> 32));
    write_be32(static_cast<std::uint32_t>(value));
}

void MessagePackWriter::write_integer(std::uint64_t value) {
    /* Match Haxeon's shortest Int64 representation. */
    if (value <= 0x7f) {
        write_byte(static_cast<std::uint8_t>(value));
    } else if (value <= 0xff) {
        write_byte(0xcc);
        write_byte(static_cast<std::uint8_t>(value));
    } else if (value <= 0xffff) {
        write_byte(0xcd);
        write_be16(static_cast<std::uint16_t>(value));
    } else if (value <= 0xffffffffULL) {
        write_byte(0xce);
        write_be32(static_cast<std::uint32_t>(value));
    } else {
        write_byte(0xd3);
        write_be64(value);
    }
}

void MessagePackWriter::write_signed_integer(std::int64_t value) {
    if (value >= 0) {
        write_integer(static_cast<std::uint64_t>(value));
    } else if (value >= -32) {
        write_byte(static_cast<std::uint8_t>(value));
    } else if (value >= std::numeric_limits<std::int8_t>::min()) {
        write_byte(0xd0);
        write_byte(static_cast<std::uint8_t>(value));
    } else if (value >= std::numeric_limits<std::int16_t>::min()) {
        write_byte(0xd1);
        write_be16(static_cast<std::uint16_t>(value));
    } else if (value >= std::numeric_limits<std::int32_t>::min()) {
        write_byte(0xd2);
        write_be32(static_cast<std::uint32_t>(value));
    } else {
        write_byte(0xd3);
        write_be64(static_cast<std::uint64_t>(value));
    }
}

void MessagePackWriter::write_float64(double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    write_byte(0xcb);
    write_be64(bits);
}

void MessagePackWriter::write_map_header(std::size_t count) {
    if (count <= 15) {
        write_byte(static_cast<std::uint8_t>(0x80 | count));
    } else if (count <= 0xffff) {
        write_byte(0xde);
        write_be16(static_cast<std::uint16_t>(count));
    } else {
        write_byte(0xdf);
        write_be32(static_cast<std::uint32_t>(count));
    }
}

void MessagePackWriter::write_binary(std::span<const std::uint8_t> value) {
    if (value.size() <= 0xff) {
        write_byte(0xc4);
        write_byte(static_cast<std::uint8_t>(value.size()));
    } else if (value.size() <= 0xffff) {
        write_byte(0xc5);
        write_be16(static_cast<std::uint16_t>(value.size()));
    } else {
        write_byte(0xc6);
        write_be32(static_cast<std::uint32_t>(value.size()));
    }
    bytes_.insert(bytes_.end(), value.begin(), value.end());
}

struct MessagePackReader::Integer {
    bool negative = false;
    std::uint64_t magnitude = 0;
};

bool MessagePackReader::read_byte(std::uint8_t &value) {
    if (position_ >= bytes_.size())
        return false;
    value = bytes_[position_++];
    return true;
}

bool MessagePackReader::read_be16(std::uint16_t &value) {
    std::uint8_t first = 0;
    std::uint8_t second = 0;
    if (!read_byte(first) || !read_byte(second))
        return false;
    value = static_cast<std::uint16_t>(first << 8 | second);
    return true;
}

bool MessagePackReader::read_be32(std::uint32_t &value) {
    std::uint8_t bytes[4]{};
    for (auto &part : bytes)
        if (!read_byte(part))
            return false;
    value = (static_cast<std::uint32_t>(bytes[0]) << 24) |
            (static_cast<std::uint32_t>(bytes[1]) << 16) |
            (static_cast<std::uint32_t>(bytes[2]) << 8) | bytes[3];
    return true;
}

bool MessagePackReader::read_be64(std::uint64_t &value) {
    std::uint32_t high = 0;
    std::uint32_t low = 0;
    if (!read_be32(high) || !read_be32(low))
        return false;
    value = (static_cast<std::uint64_t>(high) << 32) | low;
    return true;
}

bool MessagePackReader::read_integer(Integer &value) {
    std::uint8_t marker = 0;
    if (!read_byte(marker))
        return false;

    if (marker <= 0x7f) {
        value = {false, marker};
        return true;
    }
    if (marker >= 0xe0) {
        value = {true, static_cast<std::uint64_t>(0x100 - marker)};
        return true;
    }

    switch (marker) {
    case 0xcc: {
        std::uint8_t raw = 0;
        if (!read_byte(raw))
            return false;
        value = {false, raw};
        return true;
    }
    case 0xcd: {
        std::uint16_t raw = 0;
        if (!read_be16(raw))
            return false;
        value = {false, raw};
        return true;
    }
    case 0xce: {
        std::uint32_t raw = 0;
        if (!read_be32(raw))
            return false;
        value = {false, raw};
        return true;
    }
    case 0xcf: {
        std::uint64_t raw = 0;
        if (!read_be64(raw))
            return false;
        value = {false, raw};
        return true;
    }
    case 0xd0: {
        std::uint8_t raw = 0;
        if (!read_byte(raw))
            return false;
        if ((raw & 0x80) != 0)
            value = {true, static_cast<std::uint64_t>(0x100 - raw)};
        else
            value = {false, raw};
        return true;
    }
    case 0xd1: {
        std::uint16_t raw = 0;
        if (!read_be16(raw))
            return false;
        if ((raw & 0x8000) != 0)
            value = {true, static_cast<std::uint64_t>(0x10000 - raw)};
        else
            value = {false, raw};
        return true;
    }
    case 0xd2: {
        std::uint32_t raw = 0;
        if (!read_be32(raw))
            return false;
        if ((raw & 0x80000000U) != 0)
            value = {true, static_cast<std::uint64_t>(0x100000000ULL - raw)};
        else
            value = {false, raw};
        return true;
    }
    case 0xd3: {
        std::uint64_t raw = 0;
        if (!read_be64(raw))
            return false;
        if ((raw & (1ULL << 63)) != 0)
            value = {true, ~raw + 1};
        else
            value = {false, raw};
        return true;
    }
    default:
        return false;
    }
}

bool MessagePackReader::read_nonnegative(std::uint64_t &value) {
    Integer integer;
    if (!read_integer(integer) || integer.negative)
        return false;
    value = integer.magnitude;
    return true;
}

bool MessagePackReader::read_nonnegative_i64(std::uint64_t &value) {
    if (!read_nonnegative(value) || value > signed_int64_max)
        return false;
    return true;
}

bool MessagePackReader::read_signed_integer(std::int64_t &value) {
    Integer integer;
    if (!read_integer(integer))
        return false;
    if (!integer.negative) {
        if (integer.magnitude > signed_int64_max)
            return false;
        value = static_cast<std::int64_t>(integer.magnitude);
        return true;
    }

    constexpr auto signed_int64_min_magnitude = std::uint64_t{1} << 63;
    if (integer.magnitude > signed_int64_min_magnitude)
        return false;
    if (integer.magnitude == signed_int64_min_magnitude) {
        value = std::numeric_limits<std::int64_t>::min();
        return true;
    }
    value = -static_cast<std::int64_t>(integer.magnitude);
    return true;
}

bool MessagePackReader::read_float(double &value) {
    std::uint8_t marker = 0;
    if (!read_byte(marker))
        return false;
    if (marker == 0xca) {
        std::uint32_t bits = 0;
        if (!read_be32(bits))
            return false;
        float single = 0.0f;
        std::memcpy(&single, &bits, sizeof(single));
        value = single;
        return true;
    }
    if (marker != 0xcb)
        return false;
    std::uint64_t bits = 0;
    if (!read_be64(bits))
        return false;
    std::memcpy(&value, &bits, sizeof(value));
    return true;
}

bool MessagePackReader::read_map_size(std::uint32_t &count) {
    std::uint8_t marker = 0;
    if (!read_byte(marker))
        return false;
    if ((marker & 0xf0) == 0x80) {
        count = marker & 0x0f;
        return true;
    }
    if (marker == 0xde) {
        std::uint16_t value = 0;
        if (!read_be16(value))
            return false;
        count = value;
        return true;
    }
    if (marker == 0xdf)
        return read_be32(count);
    return false;
}

bool MessagePackReader::read_binary_view(std::span<const std::uint8_t> &value) {
    std::uint8_t marker = 0;
    if (!read_byte(marker))
        return false;
    std::uint32_t size = 0;
    if (marker == 0xc4) {
        std::uint8_t length = 0;
        if (!read_byte(length))
            return false;
        size = length;
    } else if (marker == 0xc5) {
        std::uint16_t length = 0;
        if (!read_be16(length))
            return false;
        size = length;
    } else if (marker == 0xc6) {
        if (!read_be32(size))
            return false;
    } else {
        return false;
    }
    if (size > bytes_.size() - position_)
        return false;
    value = bytes_.subspan(position_, size);
    position_ += size;
    return true;
}

bool MessagePackReader::skip() {
    return skip_value(0);
}

bool MessagePackReader::read_length_for_marker(std::uint8_t marker,
                                               std::uint32_t &length) {
    if ((marker & 0xe0) == 0xa0) {
        length = marker & 0x1f;
        return true;
    }
    if ((marker & 0xf0) == 0x90 || (marker & 0xf0) == 0x80)
        return false;
    switch (marker) {
    case 0xd9:
    case 0xc4: {
        std::uint8_t value = 0;
        if (!read_byte(value))
            return false;
        length = value;
        return true;
    }
    case 0xda:
    case 0xc5: {
        std::uint16_t value = 0;
        if (!read_be16(value))
            return false;
        length = value;
        return true;
    }
    case 0xdb:
    case 0xc6:
        return read_be32(length);
    default:
        return false;
    }
}

bool MessagePackReader::skip_bytes(std::size_t count) {
    if (count > bytes_.size() - position_)
        return false;
    position_ += count;
    return true;
}

bool MessagePackReader::skip_value(std::size_t depth) {
    if (depth > 64)
        return false;
    std::uint8_t marker = 0;
    if (!read_byte(marker))
        return false;
    if (marker <= 0x7f || marker >= 0xe0 || marker == 0xc0 ||
        marker == 0xc2 || marker == 0xc3)
        return true;
    if ((marker & 0xe0) == 0xa0)
        return skip_bytes(marker & 0x1f);
    if ((marker & 0xf0) == 0x90) {
        const auto count = static_cast<std::uint32_t>(marker & 0x0f);
        for (std::uint32_t index = 0; index < count; ++index)
            if (!skip_value(depth + 1))
                return false;
        return true;
    }
    if ((marker & 0xf0) == 0x80) {
        const auto count = static_cast<std::uint64_t>(marker & 0x0f) * 2;
        for (std::uint64_t index = 0; index < count; ++index)
            if (!skip_value(depth + 1))
                return false;
        return true;
    }

    switch (marker) {
    case 0xcc:
    case 0xd0:
        return skip_bytes(1);
    case 0xcd:
    case 0xd1:
        return skip_bytes(2);
    case 0xce:
    case 0xd2:
    case 0xca:
        return skip_bytes(4);
    case 0xcf:
    case 0xd3:
    case 0xcb:
        return skip_bytes(8);
    case 0xde: {
        std::uint16_t count = 0;
        if (!read_be16(count))
            return false;
        for (std::uint64_t index = 0; index < static_cast<std::uint64_t>(count) * 2;
             ++index)
            if (!skip_value(depth + 1))
                return false;
        return true;
    }
    case 0xdf: {
        std::uint32_t count = 0;
        if (!read_be32(count) || count > (bytes_.size() - position_) / 2 + 1)
            return false;
        for (std::uint64_t index = 0; index < static_cast<std::uint64_t>(count) * 2;
             ++index)
            if (!skip_value(depth + 1))
                return false;
        return true;
    }
    case 0xdc: {
        std::uint16_t count = 0;
        if (!read_be16(count))
            return false;
        for (std::uint32_t index = 0; index < count; ++index)
            if (!skip_value(depth + 1))
                return false;
        return true;
    }
    case 0xdd: {
        std::uint32_t count = 0;
        if (!read_be32(count))
            return false;
        for (std::uint32_t index = 0; index < count; ++index)
            if (!skip_value(depth + 1))
                return false;
        return true;
    }
    case 0xd9:
    case 0xda:
    case 0xdb:
    case 0xc4:
    case 0xc5:
    case 0xc6: {
        std::uint32_t length = 0;
        if (!read_length_for_marker(marker, length))
            return false;
        return skip_bytes(length);
    }
    case 0xd4:
        return skip_bytes(2);
    case 0xd5:
        return skip_bytes(3);
    case 0xd6:
        return skip_bytes(5);
    case 0xd7:
        return skip_bytes(9);
    case 0xd8:
        return skip_bytes(17);
    case 0xc7: {
        std::uint8_t length = 0;
        return read_byte(length) && skip_bytes(static_cast<std::size_t>(length) + 1);
    }
    case 0xc8: {
        std::uint16_t length = 0;
        return read_be16(length) && skip_bytes(static_cast<std::size_t>(length) + 1);
    }
    case 0xc9: {
        std::uint32_t length = 0;
        return read_be32(length) && skip_bytes(static_cast<std::size_t>(length) + 1);
    }
    default:
        return false;
    }
}

} // namespace nksensor::wire::detail
