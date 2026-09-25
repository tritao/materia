#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct SceneViewParams
{
    float4x4 value;
};

struct SceneVertexOutput
{
    float4 position [[position]];
    float3 world_position [[user(locn0)]];
    float3 normal [[user(locn1)]];
    float2 texcoord0 [[user(locn2)]];
    float4 color0 [[user(locn3)]];
};

struct SceneVertexInput
{
    float3 position [[attribute(0)]];
    float4 transform0 [[attribute(1)]];
    float4 transform1 [[attribute(2)]];
    float4 transform2 [[attribute(3)]];
    float4 transform3 [[attribute(4)]];
    float3 normal [[attribute(5)]];
    float2 texcoord0 [[attribute(6)]];
    float4 color0 [[attribute(7)]];
};

vertex SceneVertexOutput main0(SceneVertexInput input [[stage_in]],
                              constant SceneViewParams &view [[buffer(1)]])
{
    SceneVertexOutput output = {};
    output.world_position = (float4x4(input.transform0, input.transform1,
                                      input.transform2, input.transform3) *
                             float4(input.position, 1.0)).xyz;
    output.position = view.value * float4(output.world_position, 1.0);
    float3 c0 = input.transform0.xyz, c1 = input.transform1.xyz, c2 = input.transform2.xyz;
    float3 corrected = input.normal.x * cross(c1, c2) +
                       input.normal.y * cross(c2, c0) + input.normal.z * cross(c0, c1);
    output.normal = normalize(corrected * sign(dot(c0, cross(c1, c2))));
    output.texcoord0 = input.texcoord0;
    output.color0 = input.color0;
    return output;
}
