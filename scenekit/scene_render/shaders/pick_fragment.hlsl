cbuffer clip_params : register(b2)
{
    float4 clip_planes[32] : packoffset(c0);
    float4 clip_plane_count : packoffset(c32);
};

struct ScenePickFragmentInput
{
    float4 pick_color : TEXCOORD0;
    float3 world_position : TEXCOORD1;
    float4 position : SV_Position;
    uint primitive_id : SV_PrimitiveID;
};

struct ScenePickFragmentOutput
{
    float4 color : SV_Target0;
    float4 subelement : SV_Target1;
    float depth : SV_Target2;
};

ScenePickFragmentOutput main(ScenePickFragmentInput input)
{
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip_plane_count.x))
            break;
        if (dot(clip_planes[index].xyz, input.world_position) + clip_planes[index].w < 0.0f)
            discard;
    }
    ScenePickFragmentOutput output;
    uint id = input.primitive_id + 1u;
    output.color = input.pick_color;
    output.subelement = float4(float(id & 0xffu) / 255.0f,
                               float((id >> 8u) & 0xffu) / 255.0f,
                               float((id >> 16u) & 0xffu) / 255.0f, 1.0f);
    output.depth = input.position.z;
    return output;
}
