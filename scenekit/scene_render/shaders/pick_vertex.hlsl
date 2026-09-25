cbuffer view_projection : register(b1)
{
    float4x4 value : packoffset(c0);
};

struct ScenePickVertexInput
{
    float3 position : POSITION0;
    float4 transform0 : TEXCOORD1;
    float4 transform1 : TEXCOORD2;
    float4 transform2 : TEXCOORD3;
    float4 transform3 : TEXCOORD4;
    float4 pick_color : TEXCOORD5;
};

struct ScenePickVertexOutput
{
    float4 position : SV_Position;
    float4 pick_color : TEXCOORD0;
    float3 world_position : TEXCOORD1;
};

ScenePickVertexOutput main(ScenePickVertexInput input)
{
    ScenePickVertexOutput output;
    output.world_position = mul(float4(input.position, 1.0f),
                                float4x4(input.transform0, input.transform1,
                                         input.transform2, input.transform3)).xyz;
    output.position = mul(float4(output.world_position, 1.0f), value);
    output.pick_color = input.pick_color;
    return output;
}
