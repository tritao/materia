#include "animkit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <new>
#include <unordered_map>

#include "asset.hpp"
#include "runtime.hpp"

namespace {

using animkit::Asset;
using animkit::Instance;

template <typename T> class Registry {
public:
    uint32_t add(std::shared_ptr<T> value) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t id = next_++;
        if (id == 0) id = next_++;
        entries_.emplace(id, std::move(value));
        return id;
    }
    std::shared_ptr<T> find(uint32_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = entries_.find(id);
        return found == entries_.end() ? nullptr : found->second;
    }
    void remove(uint32_t id) {
        std::shared_ptr<T> released;
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = entries_.find(id);
        if (found == entries_.end()) return;
        released = std::move(found->second);
        entries_.erase(found);
    }

private:
    std::mutex mutex_;
    std::unordered_map<uint32_t, std::shared_ptr<T>> entries_;
    uint32_t next_ = 1;
};

Registry<const Asset> &assets() {
    static Registry<const Asset> registry;
    return registry;
}

Registry<Instance> &instances() {
    static Registry<Instance> registry;
    return registry;
}

thread_local std::string last_error;

template <typename Info> bool sized(const Info *info) {
    return info != nullptr && info->struct_size >= sizeof(Info);
}

template <typename T> ak_result writeBuffer(const std::vector<T> &source, uint8_t *data, uint32_t *size) {
    if (size == nullptr) return AK_ERROR_INVALID_ARGUMENT;
    const size_t bytes = source.size() * sizeof(T);
    if (bytes > UINT32_MAX) return AK_ERROR_OUT_OF_MEMORY;
    if (data == nullptr) {
        *size = static_cast<uint32_t>(bytes);
        return AK_OK;
    }
    if (*size < bytes) {
        *size = static_cast<uint32_t>(bytes);
        return AK_ERROR_BUFFER_TOO_SMALL;
    }
    if (bytes > 0) std::memcpy(data, source.data(), bytes);
    *size = static_cast<uint32_t>(bytes);
    return AK_OK;
}

const char *borrowed(const std::string &text) { return text.c_str(); }

ak_result publish(std::unique_ptr<Asset> asset, const std::string &error, ak_asset_handle *out_asset) {
    if (!asset) {
        last_error = error;
        return AK_ERROR_IMPORT;
    }
    last_error.clear();
    out_asset->id = assets().add(std::shared_ptr<const Asset>(std::move(asset)));
    return AK_OK;
}

} // namespace

extern "C" {

const char *ak_last_error(void) { return last_error.c_str(); }

ak_result ak_asset_load_file(const char *path, ak_asset_handle *out_asset) {
    if (path == nullptr || out_asset == nullptr) return AK_ERROR_INVALID_ARGUMENT;
    out_asset->id = 0;
    try {
        std::string error;
        auto asset = animkit::loadGltfFile(path, error);
        return publish(std::move(asset), error, out_asset);
    } catch (const std::bad_alloc &) {
        return AK_ERROR_OUT_OF_MEMORY;
    }
}

ak_result ak_asset_load_memory(const uint8_t *data, uint32_t size, ak_asset_handle *out_asset) {
    if (data == nullptr || size == 0 || out_asset == nullptr) return AK_ERROR_INVALID_ARGUMENT;
    out_asset->id = 0;
    try {
        std::string error;
        auto asset = animkit::loadGltfMemory(data, size, error);
        return publish(std::move(asset), error, out_asset);
    } catch (const std::bad_alloc &) {
        return AK_ERROR_OUT_OF_MEMORY;
    }
}

void ak_asset_destroy(ak_asset_handle asset) { assets().remove(asset.id); }

ak_result ak_asset_get_info(ak_asset_handle handle, ak_asset_info *out_info) {
    if (!sized(out_info)) return AK_ERROR_INVALID_ARGUMENT;
    const auto asset = assets().find(handle.id);
    if (!asset) return AK_ERROR_INVALID_HANDLE;
    out_info->joint_count = static_cast<uint32_t>(asset->joints.size());
    out_info->primitive_count = static_cast<uint32_t>(asset->primitives.size());
    out_info->material_count = static_cast<uint32_t>(asset->materials.size());
    out_info->image_count = static_cast<uint32_t>(asset->images.size());
    out_info->clip_count = static_cast<uint32_t>(asset->clips.size());
    out_info->warning_count = static_cast<uint32_t>(asset->warnings.size());
    return AK_OK;
}

// Borrowed strings stay valid while the asset handle is live, which the
// registry guarantees for as long as the caller holds that handle.
const char *ak_asset_warning(ak_asset_handle handle, uint32_t index) {
    const auto asset = assets().find(handle.id);
    return asset && index < asset->warnings.size() ? borrowed(asset->warnings[index]) : "";
}

const char *ak_asset_joint_name(ak_asset_handle handle, uint32_t joint) {
    const auto asset = assets().find(handle.id);
    return asset && joint < asset->joints.size() ? borrowed(asset->joints[joint].name) : "";
}

int32_t ak_asset_joint_parent(ak_asset_handle handle, uint32_t joint) {
    const auto asset = assets().find(handle.id);
    return asset && joint < asset->joints.size() ? asset->joints[joint].parent : -1;
}

int32_t ak_asset_find_joint(ak_asset_handle handle, const char *name) {
    const auto asset = assets().find(handle.id);
    if (!asset || name == nullptr) return -1;
    for (size_t i = 0; i < asset->joints.size(); ++i)
        if (asset->joints[i].name == name) return static_cast<int32_t>(i);
    return -1;
}

ak_result ak_asset_get_primitive(ak_asset_handle handle, uint32_t index, ak_primitive_info *out_info) {
    if (!sized(out_info)) return AK_ERROR_INVALID_ARGUMENT;
    const auto asset = assets().find(handle.id);
    if (!asset) return AK_ERROR_INVALID_HANDLE;
    if (index >= asset->primitives.size()) return AK_ERROR_INVALID_ARGUMENT;
    const animkit::Primitive &primitive = asset->primitives[index];
    out_info->vertex_count = static_cast<uint32_t>(primitive.positions.size() / 3);
    out_info->index_count = static_cast<uint32_t>(primitive.indices.size());
    out_info->material = primitive.material;
    out_info->skinned = primitive.skin >= 0 ? 1u : 0u;
    out_info->has_texcoords = primitive.texcoords.empty() ? 0u : 1u;
    out_info->joint = primitive.skin >= 0 ? -1 : primitive.node_joint;
    return AK_OK;
}

const char *ak_asset_primitive_name(ak_asset_handle handle, uint32_t index) {
    const auto asset = assets().find(handle.id);
    return asset && index < asset->primitives.size() ? borrowed(asset->primitives[index].name) : "";
}

ak_result ak_asset_read_indices(ak_asset_handle handle, uint32_t index, uint8_t *data, uint32_t *size) {
    const auto asset = assets().find(handle.id);
    if (!asset) return AK_ERROR_INVALID_HANDLE;
    if (index >= asset->primitives.size()) return AK_ERROR_INVALID_ARGUMENT;
    return writeBuffer(asset->primitives[index].indices, data, size);
}

ak_result ak_asset_read_texcoords(ak_asset_handle handle, uint32_t index, uint8_t *data, uint32_t *size) {
    const auto asset = assets().find(handle.id);
    if (!asset) return AK_ERROR_INVALID_HANDLE;
    if (index >= asset->primitives.size()) return AK_ERROR_INVALID_ARGUMENT;
    return writeBuffer(asset->primitives[index].texcoords, data, size);
}

ak_result ak_asset_get_material(ak_asset_handle handle, uint32_t index, ak_material_info *out_info) {
    if (!sized(out_info)) return AK_ERROR_INVALID_ARGUMENT;
    const auto asset = assets().find(handle.id);
    if (!asset) return AK_ERROR_INVALID_HANDLE;
    if (index >= asset->materials.size()) return AK_ERROR_INVALID_ARGUMENT;
    const animkit::Material &material = asset->materials[index];
    std::memcpy(out_info->base_color, material.base_color, sizeof(out_info->base_color));
    out_info->metallic = material.metallic;
    out_info->roughness = material.roughness;
    std::memcpy(out_info->emissive, material.emissive, sizeof(out_info->emissive));
    out_info->alpha_mode = material.alpha_mode;
    out_info->alpha_cutoff = material.alpha_cutoff;
    out_info->double_sided = material.double_sided ? 1u : 0u;
    out_info->base_color_image = material.base_color_image;
    return AK_OK;
}

const char *ak_asset_material_name(ak_asset_handle handle, uint32_t index) {
    const auto asset = assets().find(handle.id);
    return asset && index < asset->materials.size() ? borrowed(asset->materials[index].name) : "";
}

ak_result ak_asset_get_image(ak_asset_handle handle, uint32_t index, ak_image_info *out_info) {
    if (!sized(out_info)) return AK_ERROR_INVALID_ARGUMENT;
    const auto asset = assets().find(handle.id);
    if (!asset) return AK_ERROR_INVALID_HANDLE;
    if (index >= asset->images.size()) return AK_ERROR_INVALID_ARGUMENT;
    out_info->width = asset->images[index].width;
    out_info->height = asset->images[index].height;
    return AK_OK;
}

ak_result ak_asset_read_image(ak_asset_handle handle, uint32_t index, uint8_t *data, uint32_t *size) {
    const auto asset = assets().find(handle.id);
    if (!asset) return AK_ERROR_INVALID_HANDLE;
    if (index >= asset->images.size()) return AK_ERROR_INVALID_ARGUMENT;
    return writeBuffer(asset->images[index].rgba, data, size);
}

const char *ak_asset_clip_name(ak_asset_handle handle, uint32_t clip) {
    const auto asset = assets().find(handle.id);
    return asset && clip < asset->clips.size() ? borrowed(asset->clips[clip].name) : "";
}

float ak_asset_clip_duration(ak_asset_handle handle, uint32_t clip) {
    const auto asset = assets().find(handle.id);
    return asset && clip < asset->clips.size() ? asset->clips[clip].animation->duration() : 0.0f;
}

int32_t ak_asset_find_clip(ak_asset_handle handle, const char *name) {
    const auto asset = assets().find(handle.id);
    if (!asset || name == nullptr) return -1;
    for (size_t i = 0; i < asset->clips.size(); ++i)
        if (asset->clips[i].name == name) return static_cast<int32_t>(i);
    return -1;
}

ak_result ak_instance_create(ak_asset_handle handle, ak_instance_handle *out_instance) {
    if (out_instance == nullptr) return AK_ERROR_INVALID_ARGUMENT;
    out_instance->id = 0;
    auto asset = assets().find(handle.id);
    if (!asset) return AK_ERROR_INVALID_HANDLE;
    try {
        out_instance->id = instances().add(std::make_shared<Instance>(std::move(asset)));
        return AK_OK;
    } catch (const std::bad_alloc &) {
        return AK_ERROR_OUT_OF_MEMORY;
    }
}

void ak_instance_destroy(ak_instance_handle instance) { instances().remove(instance.id); }

ak_result ak_instance_set_layer(ak_instance_handle handle, uint32_t layer, int32_t clip, float time,
    float weight, uint32_t loop) {
    const auto instance = instances().find(handle.id);
    if (!instance) return AK_ERROR_INVALID_HANDLE;
    if (layer >= AK_MAX_LAYERS || clip >= static_cast<int32_t>(instance->asset().clips.size())
        || !(time == time) || !(weight == weight))
        return AK_ERROR_INVALID_ARGUMENT;
    instance->layers[layer] = {clip, time, weight, loop != 0};
    return AK_OK;
}

ak_result ak_instance_set_ik(ak_instance_handle handle, uint32_t chain, const ak_two_bone_ik *ik) {
    const auto instance = instances().find(handle.id);
    if (!instance) return AK_ERROR_INVALID_HANDLE;
    if (chain >= AK_MAX_IK_CHAINS) return AK_ERROR_INVALID_ARGUMENT;
    if (!ik || !(ik->weight > 0.0f)) {
        instance->ik[chain] = {};
        return AK_OK;
    }
    if (ik->struct_size < sizeof(*ik) || !instance->validChain(ik->start_joint, ik->mid_joint, ik->end_joint))
        return AK_ERROR_INVALID_ARGUMENT;
    for (int axis = 0; axis < 3; ++axis)
        if (!std::isfinite(ik->target[axis]) || !std::isfinite(ik->pole[axis]))
            return AK_ERROR_INVALID_ARGUMENT;
    if (!std::isfinite(ik->weight) || !(ik->soften > 0.0f) || ik->soften > 1.0f)
        return AK_ERROR_INVALID_ARGUMENT;
    animkit::IkChain &value = instance->ik[chain];
    value.start = ik->start_joint;
    value.mid = ik->mid_joint;
    value.end = ik->end_joint;
    std::copy_n(ik->target, 3, value.target);
    std::copy_n(ik->pole, 3, value.pole);
    value.weight = std::min(ik->weight, 1.0f);
    value.soften = ik->soften;
    return AK_OK;
}

ak_result ak_instance_evaluate(ak_instance_handle handle) {
    const auto instance = instances().find(handle.id);
    if (!instance) return AK_ERROR_INVALID_HANDLE;
    return instance->evaluate() ? AK_OK : AK_ERROR_INVALID_ARGUMENT;
}

ak_result ak_instance_read_positions(ak_instance_handle handle, uint32_t primitive, uint8_t *data,
    uint32_t *size) {
    const auto instance = instances().find(handle.id);
    if (!instance) return AK_ERROR_INVALID_HANDLE;
    if (primitive >= instance->asset().primitives.size()) return AK_ERROR_INVALID_ARGUMENT;
    return writeBuffer(instance->positions(primitive), data, size);
}

ak_result ak_instance_read_normals(ak_instance_handle handle, uint32_t primitive, uint8_t *data,
    uint32_t *size) {
    const auto instance = instances().find(handle.id);
    if (!instance) return AK_ERROR_INVALID_HANDLE;
    if (primitive >= instance->asset().primitives.size()) return AK_ERROR_INVALID_ARGUMENT;
    return writeBuffer(instance->normals(primitive), data, size);
}

ak_result ak_instance_read_joint_matrices(ak_instance_handle handle, uint8_t *data, uint32_t *size) {
    const auto instance = instances().find(handle.id);
    if (!instance) return AK_ERROR_INVALID_HANDLE;
    const size_t joints = instance->asset().joints.size();
    std::vector<float> matrices(joints * 16);
    for (size_t joint = 0; joint < joints; ++joint)
        std::memcpy(matrices.data() + joint * 16, instance->jointMatrix(static_cast<int>(joint)), 16 * sizeof(float));
    return writeBuffer(matrices, data, size);
}

ak_result ak_instance_get_bounds(ak_instance_handle handle, ak_bounds *out_bounds) {
    if (!sized(out_bounds)) return AK_ERROR_INVALID_ARGUMENT;
    const auto instance = instances().find(handle.id);
    if (!instance) return AK_ERROR_INVALID_HANDLE;
    instance->bounds(out_bounds->minimum, out_bounds->maximum);
    return AK_OK;
}

} // extern "C"
