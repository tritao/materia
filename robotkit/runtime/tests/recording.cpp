#include "robotkit_runtime.h"
#define MCAP_COMPRESSION_NO_ZSTD
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

static void roundTrip(rk_recording_compression compression, const char* suffix) {
  const std::string path = std::string("robotkit-recording-") + suffix + ".mcap";
  rk_recording_writer_handle writer{};
  assert(rk_recording_writer_create(path.c_str(), 1024 * 1024, compression, &writer) == RK_OK);
  const char schema[] = "{\"name\":\"TestMsg\",\"fields\":[{\"id\":1,\"name\":\"pixels\",\"type\":\"Bytes\"}]}";
  uint32_t channel = 0;
  assert(rk_recording_writer_register_channel(writer, "robotkit/test", "TestMsg",
      "robotkit-wire", reinterpret_cast<const uint8_t*>(schema), sizeof(schema) - 1,
      "msgpack", &channel) == RK_OK);
  assert(channel == 1);
  uint32_t duplicate = 0;
  assert(rk_recording_writer_register_channel(writer, "robotkit/test", "TestMsg",
      "robotkit-wire", reinterpret_cast<const uint8_t*>(schema), sizeof(schema) - 1,
      "msgpack", &duplicate) == RK_OK && duplicate == channel);
  assert(rk_recording_writer_register_channel(writer, "robotkit/test", "OtherMsg",
      "robotkit-wire", reinterpret_cast<const uint8_t*>(schema), sizeof(schema) - 1,
      "msgpack", &duplicate) == RK_ERROR_INVALID_ARGUMENT);
  const uint8_t payload[] = {0x81, 0x01, 0xc4, 0x02, 0x00, 0xff};
  assert(rk_recording_writer_enqueue(writer, channel, 9, 123456789, payload, sizeof(payload)) == RK_OK);
  assert(rk_recording_writer_enqueue(writer, channel,
      std::numeric_limits<uint64_t>::max() - 1, 123456790,
      payload, sizeof(payload)) == RK_OK);
  std::vector<uint8_t> oversized(1024 * 1024 + 1);
  assert(rk_recording_writer_enqueue(writer, channel, 10, 123456790,
      oversized.data(), oversized.size()) == RK_ERROR_QUEUE_FULL);
  rk_recording_writer_status status{}; status.struct_size = sizeof(status);
  assert(rk_recording_writer_get_status(writer, &status) == RK_OK && status.dropped == 1);
  assert(rk_recording_writer_finish(writer) == RK_OK);
  rk_recording_writer_destroy(writer);

  rk_recording_reader_handle reader{};
  assert(rk_recording_reader_open(path.c_str(), &reader) == RK_OK);
  rk_recording_message message{}; message.struct_size = sizeof(message);
  uint8_t output[256]{}; uint32_t size = sizeof(output);
  assert(rk_recording_reader_next(reader, &message, output, &size) == RK_OK);
  assert(message.schema_version == 7 && message.ordinal == 9 &&
      message.recording_timestamp_ns == 123456789 &&
      std::strcmp(message.topic, "robotkit/test") == 0 &&
      size == sizeof(payload) && std::memcmp(output, payload, size) == 0);
  uint32_t schemaSize = 0;
  assert(rk_recording_reader_schema(reader, nullptr, &schemaSize) == RK_ERROR_LIMIT);
  std::vector<uint8_t> actualSchema(schemaSize);
  assert(rk_recording_reader_schema(reader, actualSchema.data(), &schemaSize) == RK_OK);
  const std::string expectedSchema = std::string("TestMsg\n") + schema;
  assert(std::string(actualSchema.begin(), actualSchema.end()) == expectedSchema);
  size = sizeof(output);
  assert(rk_recording_reader_next(reader, &message, output, &size) == RK_OK);
  assert(message.ordinal == std::numeric_limits<uint64_t>::max() - 1);
  assert(rk_recording_reader_next(reader, &message, output, &size) == RK_ERROR_STALE_STATE);
  rk_recording_reader_destroy(reader);
  const std::string marker = path + ".incomplete";
  { std::ofstream pending(marker); pending << "incomplete\n"; }
  assert(rk_recording_reader_open(path.c_str(), &reader) == RK_ERROR_INVALID_STATE);
  std::filesystem::remove(marker);
  const std::string truncated = path + ".truncated";
  std::filesystem::copy_file(path, truncated,
      std::filesystem::copy_options::overwrite_existing);
  std::filesystem::resize_file(truncated, 8);
  assert(rk_recording_reader_open(truncated.c_str(), &reader) != RK_OK);
  std::filesystem::remove(truncated);
  std::remove(path.c_str());
}
int main() {
  roundTrip(RK_RECORDING_COMPRESSION_NONE, "none");
  roundTrip(RK_RECORDING_COMPRESSION_LZ4, "lz4");
  const char* busyPath = "robotkit-recording-queue-full.mcap";
  rk_recording_writer_handle writer{};
  assert(rk_recording_writer_create(busyPath, 128, RK_RECORDING_COMPRESSION_NONE, &writer) == RK_OK);
  const char schema[] = "{}";
  uint32_t channel = 0;
  assert(rk_recording_writer_register_channel(writer, "robotkit/busy", "Busy",
      "robotkit-wire", reinterpret_cast<const uint8_t*>(schema), 2,
      "msgpack", &channel) == RK_OK);
  std::vector<uint8_t> payload(64, 0);
  unsigned rejected = 0;
  for (uint64_t index = 0; index < 10000; ++index)
    if (rk_recording_writer_enqueue(writer, channel, index, index,
          payload.data(), payload.size()) == RK_ERROR_QUEUE_FULL) ++rejected;
  assert(rejected > 0);
  assert(rk_recording_writer_finish(writer) == RK_OK);
  rk_recording_writer_destroy(writer);
  std::filesystem::remove(busyPath);
  const std::string mixedPath = "robotkit-recording-foreign.mcap";
  {
    mcap::McapWriter mixed;
    auto options = mcap::McapWriterOptions("robotkit");
    options.compression = mcap::Compression::None;
    assert(mixed.open(mixedPath, options).ok());
    mcap::Metadata metadata;
    metadata.name = "robotkit";
    metadata.metadata["robotkit.schema_version"] = "7";
    assert(mixed.write(metadata).ok());
    mcap::Schema schema("Foreign", "jsonschema", mcap::ByteArray{});
    mixed.addSchema(schema);
    mcap::Channel foreign("other/café", "json", schema.id);
    mixed.addChannel(foreign);
    const std::byte payload[] = {std::byte{'x'}};
    mcap::Message message;
    message.channelId = foreign.id;
    message.publishTime = 1;
    message.logTime = 1;
    message.data = payload;
    message.dataSize = sizeof(payload);
    assert(mixed.write(message).ok());
    mixed.close();
  }
  rk_recording_reader_handle foreignReader{};
  assert(rk_recording_reader_open(mixedPath.c_str(), &foreignReader) == RK_OK);
  rk_recording_message foreignMessage{};
  foreignMessage.struct_size = sizeof(foreignMessage);
  uint8_t foreignPayload[8]{};
  uint32_t foreignSize = sizeof(foreignPayload);
  assert(rk_recording_reader_next(foreignReader, &foreignMessage,
      foreignPayload, &foreignSize) == RK_OK);
  assert(std::strcmp(foreignMessage.topic, "other/café") == 0 && foreignSize == 1);
  rk_recording_reader_destroy(foreignReader);
  std::filesystem::remove(mixedPath);
}
