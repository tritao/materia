#define MCAP_COMPRESSION_NO_LZ4
#define MCAP_COMPRESSION_NO_ZSTD
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include "robotkit_runtime.h"

#include <algorithm>
#include <charconv>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {
constexpr const char* Names[] = {"", "command", "snapshot", "sensor", "fault", "world", "world_event"};
constexpr const char* SchemaV1 = R"({"$schema":"https://json-schema.org/draft/2020-12/schema","title":"RobotKit recording payload v1","type":"object","additionalProperties":true,"required":["version","ordinal","recordingTimestampNs","robotId","sourceSequence","sourceTimestampNs","sourceClockId","type","payload"],"properties":{"version":{"const":1},"ordinal":{"type":"string","pattern":"^[0-9]+$"},"recordingTimestampNs":{"type":"string","pattern":"^[0-9]+$"},"robotId":{"type":"string"},"sourceSequence":{"type":"string"},"sourceTimestampNs":{"type":"string"},"sourceClockId":{"type":"string"},"type":{"type":"string"},"payload":{"type":"object"}}})";
constexpr const char* SchemaV2 = R"({"$schema":"https://json-schema.org/draft/2020-12/schema","title":"RobotKit recording payload v2","type":"object","additionalProperties":true,"required":["version","ordinal","recordingTimestampNs","robotId","sourceSequence","sourceTimestampNs","sourceClockId","type","payload"],"properties":{"version":{"const":2},"ordinal":{"type":"string","pattern":"^[0-9]+$"},"recordingTimestampNs":{"type":"string","pattern":"^[0-9]+$"},"robotId":{"type":"string"},"sourceSequence":{"type":"string"},"sourceTimestampNs":{"type":"string"},"sourceClockId":{"type":"string"},"type":{"type":"string"},"payload":{"type":"object"}}})";
constexpr const char* SchemaV3 = R"({"$schema":"https://json-schema.org/draft/2020-12/schema","title":"RobotKit recording payload v3","type":"object","additionalProperties":true,"required":["version","ordinal","recordingTimestampNs","robotId","sourceSequence","sourceTimestampNs","sourceClockId","type","payload"],"properties":{"version":{"const":3},"ordinal":{"type":"string","pattern":"^[0-9]+$"},"recordingTimestampNs":{"type":"string","pattern":"^[0-9]+$"},"robotId":{"type":"string"},"sourceSequence":{"type":"string"},"sourceTimestampNs":{"type":"string"},"sourceClockId":{"type":"string"},"type":{"type":"string"},"payload":{"type":"object"}}})";

struct Item {
  uint32_t kind = 0;
  uint32_t version = 1;
  uint64_t ordinal = 0;
  uint64_t timestamp = 0;
  std::vector<std::byte> data;
};
struct Writer {
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<Item> queue;
  uint64_t capacityBytes;
  uint64_t queuedBytes = 0, accepted = 0, written = 0, dropped = 0;
  uint32_t state = RK_RECORDING_OPEN;
  std::string error, marker;
  bool stopping = false;
  std::thread thread;
  Writer(uint64_t capacity, std::string markerPath)
      : capacityBytes(capacity), marker(std::move(markerPath)) {}
};
struct Reader {
  mcap::McapReader mcap;
  bool damaged = false;
  std::optional<mcap::LinearMessageView> messages;
  std::optional<mcap::LinearMessageView::Iterator> iterator;
  std::optional<Item> current;
};

std::mutex RegistryMutex;
uint32_t NextHandle = 1;
std::unordered_map<uint32_t, std::shared_ptr<Writer>> Writers;
std::unordered_map<uint32_t, std::shared_ptr<Reader>> Readers;

template<class T> std::shared_ptr<T> lookup(std::unordered_map<uint32_t,std::shared_ptr<T>>& values, uint32_t id) {
  std::lock_guard lock(RegistryMutex);
  auto found = values.find(id);
  return found == values.end() ? nullptr : found->second;
}
bool validKind(uint32_t kind) { return kind >= RK_RECORDING_COMMAND && kind <= RK_RECORDING_WORLD_EVENT; }
bool validSchemaData(const mcap::ByteArray& data, const char* schema) {
  const auto size = std::strlen(schema);
  return data.size() == size && std::memcmp(data.data(), schema, size) == 0;
}
bool payloadOrdinal(const std::byte* data, size_t size, uint64_t& ordinal) {
  const std::string_view text(reinterpret_cast<const char*>(data), size);
  const auto key = text.find("\"ordinal\"");
  if (key == std::string_view::npos) return false;
  auto cursor = text.find(':', key + 9);
  if (cursor == std::string_view::npos) return false;
  ++cursor;
  while (cursor < text.size() && (text[cursor] == ' ' || text[cursor] == '\t' ||
      text[cursor] == '\r' || text[cursor] == '\n')) ++cursor;
  if (cursor == text.size() || text[cursor++] != '"') return false;
  const auto end = text.find('"', cursor);
  if (end == std::string_view::npos || end == cursor) return false;
  const auto result = std::from_chars(text.data() + cursor, text.data() + end, ordinal);
  return result.ec == std::errc() && result.ptr == text.data() + end;
}
void fail(const std::shared_ptr<Writer>& writer, const std::string& message) {
  std::lock_guard lock(writer->mutex);
  if (writer->error.empty()) writer->error = message;
  writer->state = RK_RECORDING_FAILED;
  writer->stopping = true;
  writer->cv.notify_all();
}
void writeStatus(const std::shared_ptr<Writer>& writer) {
  std::ofstream out(writer->marker + ".status", std::ios::binary | std::ios::trunc);
  if (!out) return;
  out << "state=" << writer->state << "\naccepted=" << writer->accepted
      << "\nwritten=" << writer->written << "\ndropped=" << writer->dropped
      << "\nqueued=" << writer->queue.size() << "\nqueued_bytes=" << writer->queuedBytes
      << "\nerror=" << writer->error << "\n";
}
void writerMain(const std::shared_ptr<Writer>& writer, const std::string& path) {
  mcap::McapWriter output;
  auto options = mcap::McapWriterOptions("robotkit");
  options.compression = mcap::Compression::None;
  auto result = output.open(path, options);
  if (!result.ok()) { fail(writer, result.message); writeStatus(writer); return; }
  std::array<mcap::ChannelId, 7> channels{};
  for (uint32_t kind = 1; kind <= 6; ++kind) {
    mcap::Schema schema(std::string("robotkit.") + Names[kind] + ".v3", "jsonschema", SchemaV3);
    output.addSchema(schema);
    mcap::Channel channel(std::string("robotkit/") + Names[kind], "json", schema.id);
    channel.metadata["robotkit.schema_version"] = "3";
    output.addChannel(channel);
    channels[kind] = channel.id;
  }
  for (;;) {
    Item item;
    {
      std::unique_lock lock(writer->mutex);
      writer->cv.wait(lock, [&] { return writer->stopping || !writer->queue.empty(); });
      if (writer->queue.empty()) { if (writer->stopping) break; continue; }
      item = std::move(writer->queue.front());
      writer->queuedBytes -= item.data.size();
      writer->queue.pop_front();
    }
    mcap::Message message;
    message.channelId = channels[item.kind];
    message.sequence = static_cast<uint32_t>(item.ordinal);
    message.logTime = item.timestamp;
    // MCAP time fields remain times. The full-width deterministic ordinal lives
    // in the versioned payload and is recovered by the reader.
    message.publishTime = item.timestamp;
    message.data = item.data.data();
    message.dataSize = item.data.size();
    result = output.write(message);
    if (!result.ok()) { fail(writer, result.message); break; }
    std::lock_guard lock(writer->mutex);
    ++writer->written;
  }
  output.close();
  std::lock_guard lock(writer->mutex);
  if (writer->state != RK_RECORDING_FAILED) writer->state = RK_RECORDING_CLOSED;
  writeStatus(writer);
}

rk_result validateAndCache(const std::shared_ptr<Reader>& reader) {
  if (reader->damaged) return RK_ERROR_BACKEND;
  if (!reader->iterator || *reader->iterator == reader->messages->end()) return RK_ERROR_STALE_STATE;
  const auto& view = **reader->iterator;
  const auto& topic = view.channel->topic;
  uint32_t kind = 0;
  for (uint32_t index = 1; index <= 6; ++index)
    if (topic == std::string("robotkit/") + Names[index]) kind = index;
  auto version = view.channel->metadata.find("robotkit.schema_version");
  if (!kind || !view.schema || view.schema->encoding != "jsonschema" ||
      version == view.channel->metadata.end())
    return RK_ERROR_UNSUPPORTED;
  const auto expected_v1 = std::string("robotkit.") + Names[kind] + ".v1";
  const auto expected_v2 = std::string("robotkit.") + Names[kind] + ".v2";
  const auto expected_v3 = std::string("robotkit.") + Names[kind] + ".v3";
  const bool schema_v1 = version->second == "1" && view.schema->name == expected_v1 &&
      validSchemaData(view.schema->data, SchemaV1);
  const bool schema_v2 = version->second == "2" && view.schema->name == expected_v2 &&
      validSchemaData(view.schema->data, SchemaV2);
  const bool schema_v3 = version->second == "3" && view.schema->name == expected_v3 &&
      validSchemaData(view.schema->data, SchemaV3);
  if (!schema_v1 && !schema_v2 && !schema_v3) return RK_ERROR_UNSUPPORTED;
  Item item;
  item.kind = kind;
  item.version = schema_v1 ? 1 : schema_v2 ? 2 : 3;
  if (!payloadOrdinal(view.message.data, view.message.dataSize, item.ordinal))
    return RK_ERROR_INVALID_ARGUMENT;
  item.timestamp = view.message.logTime;
  item.data.assign(view.message.data, view.message.data + view.message.dataSize);
  reader->current = std::move(item);
  ++(*reader->iterator);
  return reader->damaged ? RK_ERROR_BACKEND : RK_OK;
}
}

extern "C" {
rk_result rk_recording_writer_create(const char* path, uint64_t capacity, rk_recording_writer_handle* out) {
  if (!path || !*path || !capacity || !out) return RK_ERROR_INVALID_ARGUMENT;
  try {
    std::string marker = std::string(path) + ".incomplete";
    { std::ofstream pending(marker, std::ios::trunc); if (!pending) return RK_ERROR_BACKEND; pending << "incomplete\n"; }
    auto writer = std::make_shared<Writer>(capacity, marker);
    { std::lock_guard lock(RegistryMutex); out->id = NextHandle++; Writers[out->id] = writer; }
    writer->thread = std::thread(writerMain, writer, std::string(path));
    return RK_OK;
  } catch (...) { return RK_ERROR_OUT_OF_MEMORY; }
}
rk_result rk_recording_writer_enqueue(rk_recording_writer_handle handle, rk_recording_event_kind kind,
    uint32_t version, uint64_t ordinal, uint64_t timestamp, const uint8_t* payload, uint32_t size) {
  auto writer = lookup(Writers, handle.id);
  if (!writer) return RK_ERROR_INVALID_HANDLE;
  if (!validKind(kind) || version != 3 || (!payload && size)) return RK_ERROR_INVALID_ARGUMENT;
  std::lock_guard lock(writer->mutex);
  if (writer->state != RK_RECORDING_OPEN) return RK_ERROR_INVALID_STATE;
  if (size > writer->capacityBytes || writer->queuedBytes > writer->capacityBytes - size) {
    // The control-loop-facing enqueue path never performs file I/O. Drop state
    // is observable immediately and persisted by finish/destroy.
    ++writer->dropped; return RK_ERROR_QUEUE_FULL;
  }
  Item item{kind, version, ordinal, timestamp, {}};
  item.data.resize(size);
  if (size) std::memcpy(item.data.data(), payload, size);
  writer->queuedBytes += size;
  writer->queue.push_back(std::move(item));
  ++writer->accepted;
  writer->cv.notify_one();
  return RK_OK;
}
rk_result rk_recording_writer_get_status(rk_recording_writer_handle handle, rk_recording_writer_status* status) {
  auto writer = lookup(Writers, handle.id);
  if (!writer) return RK_ERROR_INVALID_HANDLE;
  if (!status || status->struct_size < sizeof(*status)) return RK_ERROR_INVALID_ARGUMENT;
  std::lock_guard lock(writer->mutex);
  status->state=writer->state; status->accepted=writer->accepted; status->written=writer->written;
  status->dropped=writer->dropped; status->queued=writer->queue.size(); status->queued_bytes=writer->queuedBytes;
  std::memset(status->error,0,sizeof(status->error));
  std::memcpy(status->error,writer->error.data(),std::min(writer->error.size(),sizeof(status->error)-1));
  return RK_OK;
}
rk_result rk_recording_writer_finish(rk_recording_writer_handle handle) {
  auto writer=lookup(Writers,handle.id); if(!writer)return RK_ERROR_INVALID_HANDLE;
  {std::lock_guard lock(writer->mutex);writer->stopping=true;if(writer->state==RK_RECORDING_OPEN)writer->state=RK_RECORDING_CLOSING;}
  writer->cv.notify_all(); if(writer->thread.joinable())writer->thread.join();
  std::lock_guard lock(writer->mutex); writeStatus(writer);
  if(writer->state!=RK_RECORDING_CLOSED)return RK_ERROR_BACKEND;
  std::error_code error;std::filesystem::remove(writer->marker,error);return error?RK_ERROR_BACKEND:RK_OK;
}
void rk_recording_writer_destroy(rk_recording_writer_handle handle) {
  std::shared_ptr<Writer> writer;
  {std::lock_guard lock(RegistryMutex);auto found=Writers.find(handle.id);if(found==Writers.end())return;writer=found->second;Writers.erase(found);}
  {std::lock_guard lock(writer->mutex);if(writer->state==RK_RECORDING_OPEN){writer->error="writer destroyed without finish";writer->state=RK_RECORDING_FAILED;}writer->stopping=true;writeStatus(writer);}
  writer->cv.notify_all();if(writer->thread.joinable())writer->thread.join();
}
rk_result rk_recording_reader_open(const char* path,rk_recording_reader_handle* out) {
  if(!path||!*path||!out)return RK_ERROR_INVALID_ARGUMENT;
  try {
    if(std::filesystem::exists(std::string(path)+".incomplete"))return RK_ERROR_INVALID_STATE;
    auto reader=std::make_shared<Reader>();auto status=reader->mcap.open(path);if(!status.ok())return RK_ERROR_BACKEND;
    status=reader->mcap.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);if(!status.ok())return RK_ERROR_BACKEND;
    reader->messages.emplace(reader->mcap.readMessages([raw=reader.get()](const mcap::Status&){raw->damaged=true;}));
    reader->iterator.emplace(reader->messages->begin());
    {std::lock_guard lock(RegistryMutex);out->id=NextHandle++;Readers[out->id]=reader;}return RK_OK;
  } catch(...) {return RK_ERROR_BACKEND;}
}
rk_result rk_recording_reader_next(rk_recording_reader_handle handle,rk_recording_message* message,uint8_t* payload,uint32_t* size) {
  auto reader=lookup(Readers,handle.id);if(!reader)return RK_ERROR_INVALID_HANDLE;
  if(!message||message->struct_size<sizeof(*message)||!size)return RK_ERROR_INVALID_ARGUMENT;
  if(!reader->current){auto status=validateAndCache(reader);if(status!=RK_OK)return status;}
  auto& item=*reader->current;message->kind=item.kind;message->schema_version=item.version;
  message->ordinal=item.ordinal;message->recording_timestamp_ns=item.timestamp;message->payload_size=item.data.size();
  if(!payload||*size<item.data.size()){*size=item.data.size();return RK_ERROR_LIMIT;}
  std::memcpy(payload,item.data.data(),item.data.size());*size=item.data.size();reader->current.reset();return RK_OK;
}
void rk_recording_reader_destroy(rk_recording_reader_handle handle){std::lock_guard lock(RegistryMutex);Readers.erase(handle.id);}
}
