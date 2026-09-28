#include "runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "ozz/animation/runtime/blending_job.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/geometry/runtime/skinning_job.h"

namespace animkit {
namespace {

ozz::math::Float4x4 toFloat4x4(const Matrix &m) {
    ozz::math::Float4x4 result;
    for (int column = 0; column < 4; ++column)
        result.cols[column] = ozz::math::simd_float4::LoadPtrU(m.data() + column * 4);
    return result;
}

void normalizeNormals(std::vector<float> &normals) {
    for (size_t v = 0; v + 2 < normals.size(); v += 3) {
        const float length = std::sqrt(normals[v] * normals[v] + normals[v + 1] * normals[v + 1]
            + normals[v + 2] * normals[v + 2]);
        if (length > 0.0f)
            for (int i = 0; i < 3; ++i) normals[v + i] /= length;
    }
}

} // namespace

Matrix identityMatrix() {
    return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

Matrix multiply(const Matrix &a, const Matrix &b) {
    Matrix result{};
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) sum += a[k * 4 + row] * b[column * 4 + k];
            result[column * 4 + row] = sum;
        }
    return result;
}

Matrix toMatrix(const ozz::math::Float4x4 &m) {
    Matrix result;
    for (int column = 0; column < 4; ++column) ozz::math::StorePtrU(m.cols[column], result.data() + column * 4);
    return result;
}

const Matrix &gltfToScene() {
    static const Matrix matrix = {0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1};
    return matrix;
}

Instance::Instance(std::shared_ptr<const Asset> asset) : asset_(std::move(asset)) {
    import_ = toFloat4x4(gltfToScene());
    for (const Skin &skin : asset_->skins) {
        std::vector<ozz::math::Float4x4> matrices;
        for (const Matrix &m : skin.inverse_bind) matrices.push_back(toFloat4x4(m));
        inverse_bind_.push_back(std::move(matrices));
    }
    const auto &skeleton = *asset_->skeleton;
    int max_tracks = 0;
    for (const Clip &clip : asset_->clips) max_tracks = std::max(max_tracks, clip.animation->num_tracks());
    for (int layer = 0; layer < kMaxLayers; ++layer) {
        contexts_[layer] = std::make_unique<ozz::animation::SamplingJob::Context>(max_tracks);
        layer_locals_[layer].resize(skeleton.num_soa_joints());
    }
    locals_.resize(skeleton.num_soa_joints());
    models_.resize(skeleton.num_joints());
    scene_models_.resize(skeleton.num_joints());
    outputs_.resize(asset_->primitives.size());
    for (size_t i = 0; i < outputs_.size(); ++i) {
        outputs_[i].positions.resize(asset_->primitives[i].positions.size());
        outputs_[i].normals.resize(asset_->primitives[i].normals.size());
    }
    evaluate();
}

bool Instance::evaluate() {
    const auto &skeleton = *asset_->skeleton;
    ozz::animation::BlendingJob::Layer blend_layers[kMaxLayers];
    int active = 0;
    bool valid = true;
    for (int i = 0; i < kMaxLayers; ++i) {
        const Layer &layer = layers[i];
        if (!(layer.weight > 0.0f) || layer.clip < 0) continue;
        if (layer.clip >= static_cast<int32_t>(asset_->clips.size())) {
            valid = false;
            continue;
        }
        const ozz::animation::Animation &animation = *asset_->clips[layer.clip].animation;
        const float duration = animation.duration();
        float time = layer.time;
        if (layer.loop) {
            time = std::fmod(time, duration);
            if (time < 0.0f) time += duration;
        }
        ozz::animation::SamplingJob sampling;
        sampling.animation = &animation;
        sampling.context = contexts_[i].get();
        sampling.ratio = std::clamp(time / duration, 0.0f, 1.0f);
        sampling.output = ozz::make_span(layer_locals_[i]);
        if (!sampling.Run()) {
            valid = false;
            continue;
        }
        blend_layers[active].transform = ozz::make_span(layer_locals_[i]);
        blend_layers[active].weight = layer.weight;
        ++active;
    }

    if (active == 0) {
        const auto rest = skeleton.joint_rest_poses();
        std::copy(rest.begin(), rest.end(), locals_.begin());
    } else if (active == 1) {
        std::copy(blend_layers[0].transform.begin(), blend_layers[0].transform.end(), locals_.begin());
    } else {
        ozz::animation::BlendingJob blending;
        blending.layers = ozz::span<const ozz::animation::BlendingJob::Layer>(blend_layers, active);
        blending.rest_pose = skeleton.joint_rest_poses();
        blending.output = ozz::make_span(locals_);
        blending.threshold = 1e-3f;
        if (!blending.Run()) valid = false;
    }

    ozz::animation::LocalToModelJob local_to_model;
    local_to_model.skeleton = &skeleton;
    local_to_model.input = ozz::make_span(locals_);
    local_to_model.output = ozz::make_span(models_);
    if (!local_to_model.Run()) return false;
    for (size_t joint = 0; joint < models_.size(); ++joint) scene_models_[joint] = toMatrix(import_ * models_[joint]);
    for (size_t primitive = 0; primitive < outputs_.size(); ++primitive) skinPrimitive(primitive);
    return valid;
}

void Instance::skinPrimitive(size_t index) {
    const Primitive &primitive = asset_->primitives[index];
    Output &output = outputs_[index];
    const int vertex_count = static_cast<int>(primitive.positions.size() / 3);
    if (primitive.skin < 0) {
        const Matrix &m = scene_models_[primitive.node_joint];
        for (int v = 0; v < vertex_count; ++v) {
            const float *p = primitive.positions.data() + v * 3;
            const float *n = primitive.normals.data() + v * 3;
            float *out_p = output.positions.data() + v * 3;
            float *out_n = output.normals.data() + v * 3;
            for (int row = 0; row < 3; ++row) {
                out_p[row] = m[row] * p[0] + m[4 + row] * p[1] + m[8 + row] * p[2] + m[12 + row];
                out_n[row] = m[row] * n[0] + m[4 + row] * n[1] + m[8 + row] * n[2];
            }
        }
        normalizeNormals(output.normals);
        return;
    }

    const Skin &skin = asset_->skins[primitive.skin];
    const auto &inverse_bind = inverse_bind_[primitive.skin];
    palette_.resize(skin.joints.size());
    for (size_t i = 0; i < skin.joints.size(); ++i)
        palette_[i] = import_ * models_[skin.joints[i]] * inverse_bind[i];

    ozz::geometry::SkinningJob skinning;
    skinning.vertex_count = vertex_count;
    skinning.influences_count = 4;
    skinning.joint_matrices = ozz::make_span(palette_);
    skinning.joint_indices = ozz::make_span(primitive.joint_indices);
    skinning.joint_indices_stride = sizeof(uint16_t) * 4;
    skinning.joint_weights = ozz::make_span(primitive.joint_weights);
    skinning.joint_weights_stride = sizeof(float) * 3;
    skinning.in_positions = ozz::make_span(primitive.positions);
    skinning.in_positions_stride = sizeof(float) * 3;
    skinning.in_normals = ozz::make_span(primitive.normals);
    skinning.in_normals_stride = sizeof(float) * 3;
    skinning.out_positions = ozz::make_span(output.positions);
    skinning.out_positions_stride = sizeof(float) * 3;
    skinning.out_normals = ozz::make_span(output.normals);
    skinning.out_normals_stride = sizeof(float) * 3;
    skinning.Run();
    normalizeNormals(output.normals);
}

void Instance::bounds(float minimum[3], float maximum[3]) const {
    for (int i = 0; i < 3; ++i) {
        minimum[i] = std::numeric_limits<float>::max();
        maximum[i] = -std::numeric_limits<float>::max();
    }
    for (const Output &output : outputs_)
        for (size_t v = 0; v + 2 < output.positions.size(); v += 3)
            for (int i = 0; i < 3; ++i) {
                minimum[i] = std::min(minimum[i], output.positions[v + i]);
                maximum[i] = std::max(maximum[i], output.positions[v + i]);
            }
}

} // namespace animkit
