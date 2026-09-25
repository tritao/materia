#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct SceneViewParams
{
    float4x4 value;
};

struct ScenePickVertexOutput
{
    float4 position [[position]];
    float4 pick_color [[user(locn0)]];
    float3 world_position [[user(locn1)]];
};

struct ScenePickVertexInput
{
    float3 position [[attribute(0)]];
    float4 transform0 [[attribute(1)]];
    float4 transform1 [[attribute(2)]];
    float4 transform2 [[attribute(3)]];
    float4 transform3 [[attribute(4)]];
    float4 pick_color [[attribute(5)]];
};

vertex ScenePickVertexOutput main0(ScenePickVertexInput input [[stage_in]],
                                  constant SceneViewParams &view [[buffer(1)]])
{
    ScenePickVertexOutput output = {};
    output.world_position = (float4x4(input.transform0, input.transform1,
                                      input.transform2, input.transform3) *
                             float4(input.position, 1.0)).xyz;
    output.position = view.value * float4(output.world_position, 1.0);
    output.pick_color = input.pick_color;
    return output;
}
