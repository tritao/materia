cbuffer stroke_color_params : register(b0) { float4 stroke_color : packoffset(c0); };
cbuffer clip_params : register(b2) {
    float4 clip_planes[32] : packoffset(c0);
    float4 clip_plane_count : packoffset(c32);
};
struct Input {
    float4 position : SV_Position;
    float2 line_coordinate : TEXCOORD0;
    float line_length : TEXCOORD1;
    float line_half_width : TEXCOORD2;
    float3 world_position : TEXCOORD3;
};
float4 main(Input input) : SV_Target0 {
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip_plane_count.x)) break;
        if (dot(clip_planes[index].xyz, input.world_position) + clip_planes[index].w < 0.0f) discard;
    }
    float2 nearest = float2(clamp(input.line_coordinate.x, 0.0f, input.line_length), 0.0f);
    float signed_distance = length(input.line_coordinate - nearest) - input.line_half_width;
    float coverage = saturate(0.5f - signed_distance / max(fwidth(signed_distance), 1e-4f));
    return float4(stroke_color.rgb * coverage, stroke_color.a * coverage);
}
