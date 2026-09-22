#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace nksensor::wire::detail {

/**
 * Small typed MessagePack writer used by SensorWire codecs.
 *
 * This intentionally exposes primitives instead of a dynamic value tree. The
 * Haxeon wire profile uses the same approach, which keeps the C++ and Haxe
 * encoders deterministic and avoids allocations for individual pixels.
 */
class MessagePackWriter {
public:
    void write_integer(std::uint64_t value);
    void write_signed_integer(std::int64_t value);
    void write_float64(double value);
    void write_map_header(std::size_t count);
    void write_binary(std::span<const std::uint8_t> value);

    const std::vector<std::uint8_t> &bytes() const noexcept { return bytes_; }

private:
    void write_byte(std::uint8_t value);
    void write_be16(std::uint16_t value);
    void write_be32(std::uint32_t value);
    void write_be64(std::uint64_t value);

    std::vector<std::uint8_t> bytes_;
};

/**
 * Bounded typed MessagePack reader used by SensorWire codecs.
 *
 * Binary values are returned as spans into the input, so callers can validate
 * an external packet without copying its image payload. Unknown fields can be
 * skipped for forward-compatible schema evolution.
 */
class MessagePackReader {
public:
    explicit MessagePackReader(std::span<const std::uint8_t> bytes)
        : bytes_(bytes) {}

    bool at_end() const noexcept { return position_ == bytes_.size(); }
    bool read_nonnegative(std::uint64_t &value);
    bool read_nonnegative_i64(std::uint64_t &value);
    bool read_signed_integer(std::int64_t &value);
    bool read_float(double &value);
    bool read_map_size(std::uint32_t &count);
    bool read_binary_view(std::span<const std::uint8_t> &value);
    bool skip();

private:
    struct Integer;

    bool read_byte(std::uint8_t &value);
    bool read_be16(std::uint16_t &value);
    bool read_be32(std::uint32_t &value);
    bool read_be64(std::uint64_t &value);
    bool read_integer(Integer &value);
    bool read_length_for_marker(std::uint8_t marker, std::uint32_t &length);
    bool skip_bytes(std::size_t count);
    bool skip_value(std::size_t depth);

    std::span<const std::uint8_t> bytes_;
    std::size_t position_ = 0;
};

} // namespace nksensor::wire::detail
