#include <metal_stdlib>

using namespace metal;

struct SceneClipParams
{
    float4 planes[32];
    float4 count;
};

struct ScenePickFragmentInput
{
    float4 pick_color [[user(locn0)]];
    float3 world_position [[user(locn1)]];
    float4 position [[position]];
    uint primitive_id [[primitive_id]];
};

struct ScenePickFragmentOutput
{
    float4 color [[color(0)]];
    float4 subelement [[color(1)]];
    float depth [[color(2)]];
};

fragment ScenePickFragmentOutput main0(ScenePickFragmentInput input [[stage_in]],
                                      constant SceneClipParams &clip [[buffer(2)]])
{
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip.count.x))
            break;
        if (dot(clip.planes[index].xyz, input.world_position) + clip.planes[index].w < 0.0)
            discard_fragment();
    }
    ScenePickFragmentOutput output = {};
    uint id = input.primitive_id + 1u;
    output.color = input.pick_color;
    output.subelement = float4(float(id & 0xffu) / 255.0,
                               float((id >> 8u) & 0xffu) / 255.0,
                               float((id >> 16u) & 0xffu) / 255.0, 1.0);
    output.depth = input.position.z;
    return output;
}
