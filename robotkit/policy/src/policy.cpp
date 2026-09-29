#include "robotkit_policy.h"

#include <onnxruntime_cxx_api.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct TensorSpec {
    std::string name;
    std::vector<int64_t> shape;
    uint32_t elements = 0;
    uint32_t offset = 0;
};

struct Policy {
    Ort::Session session{nullptr};
    std::vector<TensorSpec> inputs, outputs;
    uint32_t input_values = 0, output_values = 0;
    std::mutex run_mutex;
};

// One process-wide environment: sessions may outlive any single policy.
Ort::Env &environment() {
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "robotkit_policy");
    return env;
}

std::mutex registry_mutex;
std::unordered_map<rk_policy, std::shared_ptr<Policy>> registry;
rk_policy next_handle = 1;

std::shared_ptr<Policy> resolve(rk_policy handle) {
    std::lock_guard lock(registry_mutex);
    const auto found = registry.find(handle);
    return found == registry.end() ? nullptr : found->second;
}

// Reads the static shape of a float tensor. A dynamic leading dimension is a batch of one.
rk_result describe(const Ort::TypeInfo &type, const std::string &name, TensorSpec &out) {
    if (type.GetONNXType() != ONNX_TYPE_TENSOR) return RK_ERROR_UNSUPPORTED;
    const auto info = type.GetTensorTypeAndShapeInfo();
    if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) return RK_ERROR_UNSUPPORTED;
    out.name = name;
    out.shape = info.GetShape();
    uint64_t elements = 1;
    for (size_t i = 0; i < out.shape.size(); ++i) {
        if (out.shape[i] < 0 && i == 0) out.shape[i] = 1;
        if (out.shape[i] <= 0) return RK_ERROR_UNSUPPORTED;
        elements *= static_cast<uint64_t>(out.shape[i]);
        if (elements > RK_POLICY_MAX_VALUES) return RK_ERROR_UNSUPPORTED;
    }
    out.elements = static_cast<uint32_t>(elements);
    return RK_OK;
}

} // namespace

extern "C" {

rk_result rk_policy_create(const char *path, rk_policy *out_policy) {
    if (!path || !out_policy) return RK_ERROR_INVALID_ARGUMENT;
    *out_policy = RK_INVALID_POLICY;
    try {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        auto policy = std::make_shared<Policy>();
        policy->session = Ort::Session(environment(), path, options);
        Ort::AllocatorWithDefaultOptions allocator;
        const auto build = [&](bool output, std::vector<TensorSpec> &specs, uint32_t &total) -> rk_result {
            const size_t count = output ? policy->session.GetOutputCount() : policy->session.GetInputCount();
            if (count == 0 || count > RK_POLICY_MAX_TENSORS) return RK_ERROR_UNSUPPORTED;
            for (size_t i = 0; i < count; ++i) {
                const auto name = output ? policy->session.GetOutputNameAllocated(i, allocator)
                                         : policy->session.GetInputNameAllocated(i, allocator);
                const auto type = output ? policy->session.GetOutputTypeInfo(i)
                                         : policy->session.GetInputTypeInfo(i);
                TensorSpec spec;
                const rk_result status = describe(type, name.get(), spec);
                if (status != RK_OK) return status;
                if (spec.name.size() >= RK_POLICY_NAME_BYTES) return RK_ERROR_UNSUPPORTED;
                spec.offset = total;
                total += spec.elements;
                if (total > RK_POLICY_MAX_VALUES) return RK_ERROR_UNSUPPORTED;
                specs.push_back(std::move(spec));
            }
            return RK_OK;
        };
        rk_result status = build(false, policy->inputs, policy->input_values);
        if (status == RK_OK) status = build(true, policy->outputs, policy->output_values);
        if (status != RK_OK) return status;
        std::lock_guard lock(registry_mutex);
        const rk_policy handle = next_handle++;
        registry.emplace(handle, std::move(policy));
        *out_policy = handle;
        return RK_OK;
    } catch (const Ort::Exception &error) {
        std::fprintf(stderr, "robotkit_policy: %s\n", error.what());
        return RK_ERROR_BACKEND;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

void rk_policy_destroy(rk_policy policy) {
    std::shared_ptr<Policy> released;
    std::lock_guard lock(registry_mutex);
    const auto found = registry.find(policy);
    if (found == registry.end()) return;
    released = std::move(found->second);
    registry.erase(found);
}

rk_result rk_policy_get_info(rk_policy policy, rk_policy_info *out_info) {
    if (!out_info || out_info->struct_size < sizeof(rk_policy_info)) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(policy);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    out_info->input_count = static_cast<uint32_t>(value->inputs.size());
    out_info->output_count = static_cast<uint32_t>(value->outputs.size());
    out_info->input_values = value->input_values;
    out_info->output_values = value->output_values;
    return RK_OK;
}

rk_result rk_policy_get_tensor(rk_policy policy, uint32_t direction, uint32_t index,
                               rk_policy_tensor *out_tensor) {
    if (!out_tensor || out_tensor->struct_size < sizeof(rk_policy_tensor) ||
        (direction != RK_POLICY_INPUT && direction != RK_POLICY_OUTPUT))
        return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(policy);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    const auto &specs = direction == RK_POLICY_INPUT ? value->inputs : value->outputs;
    if (index >= specs.size()) return RK_ERROR_INVALID_ARGUMENT;
    out_tensor->elements = specs[index].elements;
    out_tensor->offset = specs[index].offset;
    std::memset(out_tensor->name, 0, sizeof(out_tensor->name));
    std::memcpy(out_tensor->name, specs[index].name.c_str(), specs[index].name.size());
    return RK_OK;
}

rk_result rk_policy_run(rk_policy policy, rk_policy_io *io) {
    if (!io || io->struct_size < sizeof(rk_policy_io)) return RK_ERROR_INVALID_ARGUMENT;
    const auto value = resolve(policy);
    if (!value) return RK_ERROR_INVALID_HANDLE;
    try {
        std::lock_guard lock(value->run_mutex);
        std::vector<float> input_data(value->input_values), output_data(value->output_values);
        for (uint32_t i = 0; i < value->input_values; ++i) input_data[i] = static_cast<float>(io->inputs[i]);
        const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<Ort::Value> inputs, outputs;
        std::vector<const char *> input_names, output_names;
        for (const auto &spec : value->inputs) {
            inputs.push_back(Ort::Value::CreateTensor<float>(memory, input_data.data() + spec.offset,
                                                             spec.elements, spec.shape.data(), spec.shape.size()));
            input_names.push_back(spec.name.c_str());
        }
        for (const auto &spec : value->outputs) {
            outputs.push_back(Ort::Value::CreateTensor<float>(memory, output_data.data() + spec.offset,
                                                              spec.elements, spec.shape.data(), spec.shape.size()));
            output_names.push_back(spec.name.c_str());
        }
        // Outputs are bound to caller-owned memory, so the run writes them in place.
        value->session.Run(Ort::RunOptions{nullptr}, input_names.data(), inputs.data(), inputs.size(),
                           output_names.data(), outputs.data(), outputs.size());
        for (uint32_t i = 0; i < value->output_values; ++i) io->outputs[i] = output_data[i];
        return RK_OK;
    } catch (const Ort::Exception &error) {
        std::fprintf(stderr, "robotkit_policy: %s\n", error.what());
        return RK_ERROR_BACKEND;
    } catch (const std::bad_alloc &) {
        return RK_ERROR_OUT_OF_MEMORY;
    }
}

} // extern "C"
