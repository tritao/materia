#include "robotkit_inference.h"

#include <onnxruntime_cxx_api.h>
#include <openssl/evp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {
struct Tensor {
  std::string name;
  std::vector<int64_t> shape;
  uint32_t type = 0;
  uint64_t offset = 0, bytes = 0;
};
struct Work {
  uint64_t sequence = 0;
  uint32_t width = 0, height = 0, stride = 0;
  rk_inference_image_options image{};
  std::vector<uint8_t> data;
};
struct Result {
  rk_inference_result meta{};
  std::vector<uint8_t> data;
};
struct Session {
  Ort::Session ort{nullptr};
  std::vector<Tensor> inputs, outputs;
  uint64_t inputBytes = 0, outputBytes = 0;
  std::mutex runMutex, mutex;
  std::condition_variable cv;
  std::optional<Work> pending;
  std::optional<Result> result;
  uint64_t dropped = 0;
  bool stopping = false;
  std::thread worker;
};
Ort::Env& environment() {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "robotkit_inference");
  return env;
}
std::mutex registryMutex;
std::unordered_map<rk_inference_session, std::shared_ptr<Session>> sessions;
rk_inference_session nextSession = 1;
std::shared_ptr<Session> lookup(rk_inference_session id) {
  std::lock_guard lock(registryMutex);
  auto found = sessions.find(id);
  return found == sessions.end() ? nullptr : found->second;
}
uint32_t elementBytes(uint32_t type) { return type == RK_INFERENCE_FLOAT32 ? 4 : 1; }

rk_result describe(const Ort::TypeInfo& info, const std::string& name,
    const rk_inference_options& options, bool input, Tensor& tensor) {
  if (info.GetONNXType() != ONNX_TYPE_TENSOR || name.empty() || name.size() >= 128)
    return RK_ERROR_UNSUPPORTED;
  const auto shape = info.GetTensorTypeAndShapeInfo();
  const auto elementType = shape.GetElementType();
  if (elementType == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) tensor.type = RK_INFERENCE_FLOAT32;
  else if (input && elementType == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) tensor.type = RK_INFERENCE_UINT8;
  else return RK_ERROR_UNSUPPORTED;
  tensor.name = name;
  tensor.shape = shape.GetShape();
  if (tensor.shape.empty() || tensor.shape.size() > 8) return RK_ERROR_UNSUPPORTED;
  bool nchw = tensor.shape.size() == 4 && tensor.shape[1] == 3;
  bool nhwc = tensor.shape.size() == 4 && tensor.shape[3] == 3;
  uint64_t elements = 1;
  for (size_t i = 0; i < tensor.shape.size(); ++i) {
    if (tensor.shape[i] <= 0) {
      if (i == 0) tensor.shape[i] = 1;
      else if (input && nchw && i == 2) tensor.shape[i] = options.dynamic_height;
      else if (input && nchw && i == 3) tensor.shape[i] = options.dynamic_width;
      else if (input && nhwc && i == 1) tensor.shape[i] = options.dynamic_height;
      else if (input && nhwc && i == 2) tensor.shape[i] = options.dynamic_width;
      else return RK_ERROR_UNSUPPORTED;
    }
    if (tensor.shape[i] <= 0 || elements > UINT32_MAX / static_cast<uint64_t>(tensor.shape[i]))
      return RK_ERROR_UNSUPPORTED;
    elements *= static_cast<uint64_t>(tensor.shape[i]);
  }
  tensor.bytes = elements * elementBytes(tensor.type);
  if (tensor.bytes > UINT32_MAX) return RK_ERROR_UNSUPPORTED;
  return RK_OK;
}

rk_result run(const std::shared_ptr<Session>& session, const uint8_t* input,
    uint32_t inputSize, std::vector<uint8_t>& output) {
  if (!input || inputSize != session->inputBytes) return RK_ERROR_INVALID_ARGUMENT;
  try {
    std::lock_guard lock(session->runMutex);
    const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> inputs, outputs;
    std::vector<const char*> inputNames, outputNames;
    for (const auto& tensor : session->inputs) {
      auto* data = const_cast<uint8_t*>(input + tensor.offset);
      inputs.push_back(Ort::Value::CreateTensor(memory, data, tensor.bytes,
          tensor.shape.data(), tensor.shape.size(), tensor.type == RK_INFERENCE_FLOAT32
              ? ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT : ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8));
      inputNames.push_back(tensor.name.c_str());
    }
    output.resize(session->outputBytes);
    for (const auto& tensor : session->outputs) {
      outputs.push_back(Ort::Value::CreateTensor(memory, output.data() + tensor.offset,
          tensor.bytes, tensor.shape.data(), tensor.shape.size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT));
      outputNames.push_back(tensor.name.c_str());
    }
    session->ort.Run(Ort::RunOptions{nullptr}, inputNames.data(), inputs.data(), inputs.size(),
        outputNames.data(), outputs.data(), outputs.size());
    return RK_OK;
  } catch (const Ort::Exception&) { return RK_ERROR_BACKEND; }
    catch (const std::bad_alloc&) { return RK_ERROR_OUT_OF_MEMORY; }
}

rk_result preprocess(const Work& work, const Tensor& input, std::vector<uint8_t>& output,
    rk_inference_result& meta) {
  const auto& p = work.image;
  if (input.shape.size() != 4 || input.type != RK_INFERENCE_FLOAT32) return RK_ERROR_UNSUPPORTED;
  const bool nchw = p.layout == RK_INFERENCE_NCHW;
  if ((nchw && (input.shape[1] != 3 || input.shape[2] != p.target_height || input.shape[3] != p.target_width)) ||
      (!nchw && (input.shape[1] != p.target_height || input.shape[2] != p.target_width || input.shape[3] != 3)))
    return RK_ERROR_INVALID_ARGUMENT;
  const float scale = std::min(float(p.target_width) / work.width, float(p.target_height) / work.height);
  const uint32_t resizedW = std::max(1u, uint32_t(std::lround(work.width * scale)));
  const uint32_t resizedH = std::max(1u, uint32_t(std::lround(work.height * scale)));
  const uint32_t padX = (p.target_width - resizedW) / 2;
  const uint32_t padY = (p.target_height - resizedH) / 2;
  meta.source_width = work.width; meta.source_height = work.height;
  meta.image_scale = scale; meta.pad_x = float(padX); meta.pad_y = float(padY);
  std::vector<float> values(size_t(p.target_width) * p.target_height * 3);
  const auto pixel = [&](int x, int y, int channel) -> float {
    x = std::clamp(x, 0, int(work.width) - 1);
    y = std::clamp(y, 0, int(work.height) - 1);
    return work.data[size_t(y) * work.stride + size_t(x) * 3 + channel];
  };
  for (uint32_t y = 0; y < p.target_height; ++y) {
    for (uint32_t x = 0; x < p.target_width; ++x) {
      bool inside = x >= padX && x < padX + resizedW && y >= padY && y < padY + resizedH;
      float sx = inside ? (float(x - padX) + .5f) / scale - .5f : 0;
      float sy = inside ? (float(y - padY) + .5f) / scale - .5f : 0;
      int x0 = int(std::floor(sx)), y0 = int(std::floor(sy));
      float fx = sx - x0, fy = sy - y0;
      for (int c = 0; c < 3; ++c) {
        float raw = p.padding_value;
        if (inside) raw = (1 - fy) * ((1 - fx) * pixel(x0,y0,c) + fx * pixel(x0+1,y0,c))
            + fy * ((1 - fx) * pixel(x0,y0+1,c) + fx * pixel(x0+1,y0+1,c));
        float value = (raw * p.scale - p.mean[c]) / p.std[c];
        size_t index = nchw ? (size_t(c) * p.target_height + y) * p.target_width + x
                            : (size_t(y) * p.target_width + x) * 3 + c;
        values[index] = value;
      }
    }
  }
  output.resize(values.size() * sizeof(float));
  std::memcpy(output.data(), values.data(), output.size());
  return RK_OK;
}

void workerMain(const std::shared_ptr<Session>& session) {
  for (;;) {
    Work work;
    {
      std::unique_lock lock(session->mutex);
      session->cv.wait(lock, [&] { return session->stopping || session->pending.has_value(); });
      if (session->stopping) return;
      work = std::move(*session->pending);
      session->pending.reset();
    }
    Result result;
    result.meta.struct_size = sizeof(result.meta);
    result.meta.sequence = work.sequence;
    std::vector<uint8_t> prepared;
    rk_result status = RK_OK;
    if (work.width) status = preprocess(work, session->inputs.front(), prepared, result.meta);
    else prepared = std::move(work.data);
    if (status == RK_OK) status = run(session, prepared.data(), prepared.size(), result.data);
    result.meta.status = status;
    result.meta.output_bytes = result.data.size();
    result.meta.completed_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    {
      std::lock_guard lock(session->mutex);
      if (session->result) ++session->dropped;
      session->result = std::move(result);
    }
  }
}
}

extern "C" {
rk_result rk_inference_create(const char* path, const rk_inference_options* options,
    rk_inference_session* out) {
  if (!path || !*path || !options || options->struct_size < sizeof(*options) || !out)
    return RK_ERROR_INVALID_ARGUMENT;
  *out = 0;
  try {
    Ort::SessionOptions ortOptions;
    ortOptions.SetIntraOpNumThreads(options->intra_op_threads ? options->intra_op_threads : 1);
    ortOptions.SetInterOpNumThreads(1);
    ortOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    auto session = std::make_shared<Session>();
    session->ort = Ort::Session(environment(), path, ortOptions);
    Ort::AllocatorWithDefaultOptions allocator;
    const auto collect = [&](bool output) -> rk_result {
      auto& tensors = output ? session->outputs : session->inputs;
      auto& total = output ? session->outputBytes : session->inputBytes;
      size_t count = output ? session->ort.GetOutputCount() : session->ort.GetInputCount();
      if (!count || count > 16) return RK_ERROR_UNSUPPORTED;
      for (size_t i = 0; i < count; ++i) {
        const auto name = output ? session->ort.GetOutputNameAllocated(i, allocator)
                                 : session->ort.GetInputNameAllocated(i, allocator);
        const auto info = output ? session->ort.GetOutputTypeInfo(i) : session->ort.GetInputTypeInfo(i);
        Tensor tensor;
        auto status = describe(info, name.get(), *options, !output, tensor);
        if (status != RK_OK) return status;
        tensor.offset = total;
        total += tensor.bytes;
        if (total > UINT32_MAX) return RK_ERROR_UNSUPPORTED;
        tensors.push_back(std::move(tensor));
      }
      return RK_OK;
    };
    auto status = collect(false);
    if (status == RK_OK) status = collect(true);
    if (status != RK_OK) return status;
    rk_inference_session handle;
    {
      std::lock_guard lock(registryMutex);
      handle = nextSession++;
      sessions.emplace(handle, session);
    }
    try { session->worker = std::thread(workerMain, session); }
    catch (...) {
      std::lock_guard lock(registryMutex);
      sessions.erase(handle);
      throw;
    }
    *out = handle;
    return RK_OK;
  } catch (const Ort::Exception&) { return RK_ERROR_BACKEND; }
    catch (const std::bad_alloc&) { return RK_ERROR_OUT_OF_MEMORY; }
    catch (...) { return RK_ERROR_BACKEND; }
}
void rk_inference_destroy(rk_inference_session id) {
  std::shared_ptr<Session> session;
  {
    std::lock_guard lock(registryMutex);
    auto found = sessions.find(id);
    if (found == sessions.end()) return;
    session = std::move(found->second);
    sessions.erase(found);
  }
  { std::lock_guard lock(session->mutex); session->stopping = true; session->pending.reset(); }
  session->cv.notify_one();
  if (session->worker.joinable()) session->worker.join();
}
rk_result rk_inference_get_info(rk_inference_session id, rk_inference_info* info) {
  if (!info || info->struct_size < sizeof(*info)) return RK_ERROR_INVALID_ARGUMENT;
  auto session = lookup(id); if (!session) return RK_ERROR_INVALID_HANDLE;
  info->input_count = session->inputs.size(); info->output_count = session->outputs.size();
  info->input_bytes = session->inputBytes; info->output_bytes = session->outputBytes;
  return RK_OK;
}
rk_result rk_inference_get_tensor(rk_inference_session id, uint32_t direction,
    uint32_t index, rk_inference_tensor* out) {
  if (!out || out->struct_size < sizeof(*out) || direction > 1) return RK_ERROR_INVALID_ARGUMENT;
  auto session = lookup(id); if (!session) return RK_ERROR_INVALID_HANDLE;
  const auto& tensors = direction == RK_INFERENCE_INPUT ? session->inputs : session->outputs;
  if (index >= tensors.size()) return RK_ERROR_INVALID_ARGUMENT;
  const auto& tensor = tensors[index];
  out->element_type = tensor.type; out->rank = tensor.shape.size();
  out->byte_offset = tensor.offset; out->byte_count = tensor.bytes;
  std::fill(std::begin(out->shape), std::end(out->shape), 0);
  std::copy(tensor.shape.begin(), tensor.shape.end(), out->shape);
  std::memset(out->name, 0, sizeof(out->name));
  std::memcpy(out->name, tensor.name.data(), tensor.name.size());
  return RK_OK;
}
rk_result rk_inference_model_sha256(const char* path, char* digest, uint32_t* size) {
  if (!path || !size) return RK_ERROR_INVALID_ARGUMENT;
  if (!digest || *size < 65) { *size = 65; return RK_ERROR_LIMIT; }
  std::ifstream file(path, std::ios::binary); if (!file) return RK_ERROR_BACKEND;
  auto* context = EVP_MD_CTX_new(); if (!context) return RK_ERROR_OUT_OF_MEMORY;
  bool ok = EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1;
  char buffer[65536];
  while (ok && file) {
    file.read(buffer, sizeof(buffer));
    auto count = file.gcount();
    if (count > 0) ok = EVP_DigestUpdate(context, buffer, count) == 1;
  }
  unsigned char raw[32]{}; unsigned int length = 0;
  ok = ok && !file.bad() && EVP_DigestFinal_ex(context, raw, &length) == 1 && length == 32;
  EVP_MD_CTX_free(context);
  if (!ok) return RK_ERROR_BACKEND;
  constexpr char hex[] = "0123456789abcdef";
  for (size_t i = 0; i < 32; ++i) { digest[2*i] = hex[raw[i] >> 4]; digest[2*i+1] = hex[raw[i] & 15]; }
  digest[64] = 0; *size = 65; return RK_OK;
}
rk_result rk_inference_run(rk_inference_session id, const uint8_t* input,
    uint32_t inputSize, uint8_t* output, uint32_t* size) {
  if (!size) return RK_ERROR_INVALID_ARGUMENT;
  auto session = lookup(id); if (!session) return RK_ERROR_INVALID_HANDLE;
  if (!input || inputSize != session->inputBytes) return RK_ERROR_INVALID_ARGUMENT;
  if (!output || *size < session->outputBytes) { *size = session->outputBytes; return RK_ERROR_LIMIT; }
  std::vector<uint8_t> data;
  auto status = run(session, input, inputSize, data);
  if (status != RK_OK) return status;
  std::memcpy(output, data.data(), data.size()); *size = data.size(); return RK_OK;
}
rk_result rk_inference_submit(rk_inference_session id, uint64_t sequence,
    const uint8_t* input, uint32_t inputSize) {
  auto session = lookup(id); if (!session) return RK_ERROR_INVALID_HANDLE;
  if (!input || inputSize != session->inputBytes) return RK_ERROR_INVALID_ARGUMENT;
  Work work; work.sequence = sequence;
  try { work.data.assign(input, input + inputSize); }
  catch (const std::bad_alloc&) { return RK_ERROR_OUT_OF_MEMORY; }
  {
    std::lock_guard lock(session->mutex);
    if (session->stopping) return RK_ERROR_INVALID_STATE;
    if (session->pending) ++session->dropped;
    session->pending = std::move(work);
  }
  session->cv.notify_one(); return RK_OK;
}
rk_result rk_inference_submit_rgb8(rk_inference_session id, uint64_t sequence,
    uint32_t width, uint32_t height, uint32_t stride, const uint8_t* pixels,
    uint32_t pixelBytes, const rk_inference_image_options* options) {
  auto session = lookup(id); if (!session) return RK_ERROR_INVALID_HANDLE;
  if (!options || options->struct_size < sizeof(*options) || !pixels || !width || !height ||
      !options->target_width || !options->target_height ||
      (options->layout != RK_INFERENCE_NCHW && options->layout != RK_INFERENCE_NHWC) ||
      !std::isfinite(options->padding_value) || !std::isfinite(options->scale) ||
      width > UINT32_MAX / 3 || stride < width * 3 ||
      uint64_t(stride) * height > pixelBytes || session->inputs.size() != 1)
    return RK_ERROR_INVALID_ARGUMENT;
  for (int c = 0; c < 3; ++c)
    if (!std::isfinite(options->mean[c]) || !std::isfinite(options->std[c]) || options->std[c] == 0)
      return RK_ERROR_INVALID_ARGUMENT;
  Work work; work.sequence = sequence; work.width = width; work.height = height;
  work.stride = stride; work.image = *options;
  try { work.data.assign(pixels, pixels + uint64_t(stride) * height); }
  catch (const std::bad_alloc&) { return RK_ERROR_OUT_OF_MEMORY; }
  {
    std::lock_guard lock(session->mutex);
    if (session->stopping) return RK_ERROR_INVALID_STATE;
    if (session->pending) ++session->dropped;
    session->pending = std::move(work);
  }
  session->cv.notify_one(); return RK_OK;
}
rk_result rk_inference_poll_result(rk_inference_session id, rk_inference_result* result,
    uint8_t* output, uint32_t* size) {
  if (!result || result->struct_size < sizeof(*result) || !size) return RK_ERROR_INVALID_ARGUMENT;
  auto session = lookup(id); if (!session) return RK_ERROR_INVALID_HANDLE;
  std::lock_guard lock(session->mutex);
  // The HXI two-call wrapper may query while the worker is still running.
  // Report the fixed output capacity even on STALE_STATE so a result arriving
  // between query and read cannot exceed the allocated buffer.
  if (!session->result) { *size = session->outputBytes; return RK_ERROR_STALE_STATE; }
  const auto& ready = *session->result;
  if (ready.data.size() && (!output || *size < ready.data.size())) {
    *size = ready.data.size(); return RK_ERROR_LIMIT;
  }
  *result = ready.meta; result->dropped = session->dropped;
  if (!ready.data.empty()) std::memcpy(output, ready.data.data(), ready.data.size());
  *size = ready.data.size(); session->result.reset(); session->dropped = 0;
  return RK_OK;
}
}
