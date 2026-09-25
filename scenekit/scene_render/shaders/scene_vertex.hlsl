cbuffer view_projection : register(b1)
{
    float4x4 value : packoffset(c0);
};

struct SceneVertexInput
{
    float3 position : POSITION0;
    float4 transform0 : TEXCOORD1;
    float4 transform1 : TEXCOORD2;
    float4 transform2 : TEXCOORD3;
    float4 transform3 : TEXCOORD4;
    float3 normal : TEXCOORD5;
    float2 texcoord0 : TEXCOORD6;
    float4 color0 : TEXCOORD7;
};

struct SceneVertexOutput
{
    float4 position : SV_Position;
    float3 world_position : TEXCOORD0;
    float3 normal : TEXCOORD1;
    float2 texcoord0 : TEXCOORD2;
    float4 color0 : TEXCOORD3;
};

SceneVertexOutput main(SceneVertexInput input)
{
    SceneVertexOutput output;
    float3 world_position = mul(float4(input.position, 1.0f),
                                float4x4(input.transform0, input.transform1,
                                         input.transform2, input.transform3)).xyz;
    output.position = mul(float4(world_position, 1.0f), value);
    output.world_position = world_position;
    float3 c0 = input.transform0.xyz, c1 = input.transform1.xyz, c2 = input.transform2.xyz;
    float3 corrected = input.normal.x * cross(c1, c2) +
                       input.normal.y * cross(c2, c0) + input.normal.z * cross(c0, c1);
    output.normal = normalize(corrected * sign(dot(c0, cross(c1, c2))));
    output.texcoord0 = input.texcoord0;
    output.color0 = input.color0;
    return output;
}
