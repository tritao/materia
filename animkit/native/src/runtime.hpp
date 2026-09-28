#ifndef ANIMKIT_RUNTIME_HPP
#define ANIMKIT_RUNTIME_HPP

#include <memory>
#include <vector>

#include "asset.hpp"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_transform.h"

namespace animkit {

enum { kMaxLayers = 4, kMaxIkChains = 4 };

struct Layer {
    int32_t clip = -1;
    float time = 0.0f;
    float weight = 0.0f;
    bool loop = true;
};

/** A two-bone chain reaching for a scene-space target; weight 0 disables it. */
struct IkChain {
    int32_t start = -1;
    int32_t mid = -1;
    int32_t end = -1;
    float target[3] = {0.0f, 0.0f, 0.0f};
    float pole[3] = {0.0f, 0.0f, 1.0f};
    float weight = 0.0f;
    float soften = 1.0f;
};

/**
 * One posed copy of an asset. Evaluation samples and blends the active clip
 * layers, computes joint transforms, and skins every primitive on the CPU.
 * All outputs are in SceneKit space (+Z up, +X forward).
 */
class Instance {
public:
    explicit Instance(std::shared_ptr<const Asset> asset);

    const Asset &asset() const { return *asset_; }
    std::array<Layer, kMaxLayers> layers;
    std::array<IkChain, kMaxIkChains> ik;

    /** True when start, mid, and end name joints where each is an ancestor of the next. */
    bool validChain(int32_t start, int32_t mid, int32_t end) const;

    /** Returns false when a layer names a missing clip. */
    bool evaluate();

    const float *jointMatrix(int joint) const { return scene_models_[joint].data(); }
    const std::vector<float> &positions(size_t primitive) const { return outputs_[primitive].positions; }
    const std::vector<float> &normals(size_t primitive) const { return outputs_[primitive].normals; }
    void bounds(float minimum[3], float maximum[3]) const;

private:
    struct Output {
        std::vector<float> positions;
        std::vector<float> normals;
    };

    std::shared_ptr<const Asset> asset_;
    ozz::math::Float4x4 import_;
    std::vector<std::vector<ozz::math::Float4x4>> inverse_bind_; // per skin
    std::array<std::unique_ptr<ozz::animation::SamplingJob::Context>, kMaxLayers> contexts_;
    std::array<std::vector<ozz::math::SoaTransform>, kMaxLayers> layer_locals_;
    std::vector<ozz::math::SoaTransform> locals_;
    std::vector<ozz::math::Float4x4> models_;
    std::vector<Matrix> scene_models_;
    std::vector<ozz::math::Float4x4> palette_;
    std::vector<Output> outputs_;

    void skinPrimitive(size_t index);
    /** Applies every enabled IK chain to locals_ and models_. */
    bool solveIk();
    ozz::math::Float4x4 sceneInverse_;
};

} // namespace animkit

#endif
