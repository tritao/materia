#include "robotkit_inference.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

static rk_inference_session open(const char* path) {
  rk_inference_options options{};
  options.struct_size = sizeof(options);
  options.intra_op_threads = 1;
  rk_inference_session session = 0;
  assert(rk_inference_create(path, &options, &session) == RK_OK);
  assert(session != 0);
  return session;
}
static rk_inference_result waitFor(rk_inference_session session, std::vector<uint8_t>& bytes) {
  rk_inference_result result{};
  result.struct_size = sizeof(result);
  uint32_t size = bytes.size();
  for (int attempt = 0; attempt < 500; ++attempt) {
    size = bytes.size();
    auto status = rk_inference_poll_result(session, &result, bytes.data(), &size);
    if (status == RK_OK) { bytes.resize(size); return result; }
    assert(status == RK_ERROR_STALE_STATE);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(false && "inference worker did not finish");
  return result;
}
static float value(const std::vector<uint8_t>& bytes, size_t index) {
  float result = 0;
  std::memcpy(&result, bytes.data() + index * 4, 4);
  return result;
}
static rk_inference_image_options imageOptions(uint32_t layout) {
  rk_inference_image_options result{};
  result.struct_size = sizeof(result);
  result.target_width = result.target_height = 4;
  result.layout = layout;
  result.scale = 1.0f / 255.0f;
  result.std[0] = result.std[1] = result.std[2] = 1;
  return result;
}
static void basic() {
  auto session = open(RK_INFERENCE_FIXTURE);
  rk_inference_result pending{}; pending.struct_size = sizeof(pending);
  uint32_t pendingSize = 0;
  assert(rk_inference_poll_result(session, &pending, nullptr, &pendingSize) == RK_ERROR_STALE_STATE);
  assert(pendingSize == 48);
  rk_inference_info info{}; info.struct_size = sizeof(info);
  assert(rk_inference_get_info(session, &info) == RK_OK);
  assert(info.input_count == 1 && info.output_count == 1);
  assert(info.input_bytes == 192 && info.output_bytes == 48);
  rk_inference_tensor tensor{}; tensor.struct_size = sizeof(tensor);
  assert(rk_inference_get_tensor(session, RK_INFERENCE_INPUT, 0, &tensor) == RK_OK);
  assert(std::string(tensor.name) == "image" && tensor.rank == 4 && tensor.shape[1] == 3 && tensor.shape[2] == 4);
  assert(tensor.element_type == RK_INFERENCE_FLOAT32);
  assert(rk_inference_get_tensor(session, RK_INFERENCE_OUTPUT, 0, &tensor) == RK_OK);
  assert(std::string(tensor.name) == "detections" && tensor.shape[1] == 2 && tensor.shape[2] == 6);
  float zeros[48]{};
  std::vector<uint8_t> output(48);
  uint32_t size = output.size();
  assert(rk_inference_run(session, reinterpret_cast<uint8_t*>(zeros), sizeof(zeros), output.data(), &size) == RK_OK);
  assert(size == 48 && std::fabs(value(output, 0) - 2) < 1e-6 &&
      std::fabs(value(output, 4) - .9f) < 1e-6);
  size = 1;
  assert(rk_inference_run(session, reinterpret_cast<uint8_t*>(zeros), sizeof(zeros), output.data(), &size) == RK_ERROR_LIMIT);
  assert(size == 48);
  assert(rk_inference_run(session, reinterpret_cast<uint8_t*>(zeros), 4, output.data(), &size) == RK_ERROR_INVALID_ARGUMENT);
  char digest[65]{}; size = sizeof(digest);
  assert(rk_inference_model_sha256(RK_INFERENCE_FIXTURE, digest, &size) == RK_OK);
  assert(size == 65 && std::strlen(digest) == 64);
  rk_inference_destroy(session);
  assert(rk_inference_get_info(session, &info) == RK_ERROR_INVALID_HANDLE);
  rk_inference_destroy(session);
  rk_inference_options options{}; options.struct_size = sizeof(options);
  assert(rk_inference_create("/missing.onnx", &options, &session) == RK_ERROR_BACKEND);
}
static void images(const char* model, uint32_t layout) {
  auto session = open(model);
  auto options = imageOptions(layout);
  std::vector<uint8_t> output(48);
  const uint32_t widths[] = {8, 4, 4};
  const uint32_t heights[] = {4, 8, 4};
  const float scales[] = {.5f, .5f, 1};
  const float padsX[] = {0, 1, 0};
  const float padsY[] = {1, 0, 0};
  for (int i = 0; i < 3; ++i) {
    uint32_t stride = widths[i] * 3 + 5;
    std::vector<uint8_t> pixels(stride * heights[i], 255);
    for (uint32_t y = 0; y < heights[i]; ++y)
      for (uint32_t x = 0; x < widths[i] * 3; ++x) pixels[y * stride + x] = 51;
    assert(rk_inference_submit_rgb8(session, i + 1, widths[i], heights[i], stride,
        pixels.data(), pixels.size(), &options) == RK_OK);
    auto result = waitFor(session, output);
    assert(result.status == RK_OK && result.sequence == uint64_t(i+1));
    assert(result.image_scale == scales[i] && result.pad_x == padsX[i] && result.pad_y == padsY[i]);
    assert(result.source_width == widths[i] && result.source_height == heights[i]);
    // 51/255 is 0.2; the padded area is zero. This checks normalization and stride.
    float mean = i == 2 ? .2f : .1f;
    assert(std::fabs(value(output, 4) - (.9f + .001f * mean)) < 2e-5);
    output.resize(48);
  }
  options.std[1] = 0;
  uint8_t pixels[48]{};
  assert(rk_inference_submit_rgb8(session, 4, 4, 4, 12, pixels, 48, &options) == RK_ERROR_INVALID_ARGUMENT);
  rk_inference_destroy(session);
}
static void asyncDrops() {
  auto session = open(RK_INFERENCE_FIXTURE_LARGE);
  rk_inference_image_options options = imageOptions(RK_INFERENCE_NCHW);
  options.target_width = options.target_height = 512;
  std::vector<uint8_t> pixels(512 * 512 * 3, 64);
  assert(rk_inference_submit_rgb8(session, 1, 512, 512, 1536,
      pixels.data(), pixels.size(), &options) == RK_OK);
  const auto start = std::chrono::steady_clock::now();
  for (uint64_t i = 2; i <= 30; ++i)
    assert(rk_inference_submit_rgb8(session, i, 512, 512, 1536,
        pixels.data(), pixels.size(), &options) == RK_OK);
  auto elapsed = std::chrono::steady_clock::now() - start;
  assert(elapsed < std::chrono::milliseconds(300));
  std::vector<uint8_t> output(48);
  auto result = waitFor(session, output);
  assert(result.status == RK_OK && result.dropped > 0);
  rk_inference_destroy(session);
}
static void uint8Input() {
  auto session = open(RK_INFERENCE_FIXTURE_UINT8);
  rk_inference_tensor tensor{}; tensor.struct_size = sizeof(tensor);
  assert(rk_inference_get_tensor(session, RK_INFERENCE_INPUT, 0, &tensor) == RK_OK);
  assert(tensor.element_type == RK_INFERENCE_UINT8 && tensor.byte_count == 48);
  uint8_t input[48]; std::memset(input, 51, sizeof(input));
  uint8_t output[48]{}; uint32_t size = sizeof(output);
  assert(rk_inference_run(session, input, sizeof(input), output, &size) == RK_OK);
  float score = 0; std::memcpy(&score, output + 16, 4);
  assert(std::fabs(score - .951f) < 1e-5);
  rk_inference_destroy(session);
}
static void dynamicShapes() {
  rk_inference_options options{}; options.struct_size = sizeof(options);
  rk_inference_session session = 0;
  assert(rk_inference_create(RK_INFERENCE_FIXTURE_DYNAMIC, &options, &session) == RK_ERROR_UNSUPPORTED);
  options.dynamic_width = options.dynamic_height = 4;
  assert(rk_inference_create(RK_INFERENCE_FIXTURE_DYNAMIC, &options, &session) == RK_OK);
  rk_inference_tensor tensor{}; tensor.struct_size = sizeof(tensor);
  assert(rk_inference_get_tensor(session, RK_INFERENCE_INPUT, 0, &tensor) == RK_OK);
  assert(tensor.shape[0] == 1 && tensor.shape[2] == 4 && tensor.shape[3] == 4);
  rk_inference_destroy(session);
}
int main() {
  basic();
  images(RK_INFERENCE_FIXTURE, RK_INFERENCE_NCHW);
  images(RK_INFERENCE_FIXTURE_NHWC, RK_INFERENCE_NHWC);
  asyncDrops();
  uint8Input();
  dynamicShapes();
}
