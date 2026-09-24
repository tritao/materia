cbuffer view_projection : register(b1) { float4x4 value : packoffset(c0); };
cbuffer stroke_view_params : register(b3) { float4 stroke_view : packoffset(c0); };
struct Input {
    float3 start_position : POSITION0;
    float3 end_position : TEXCOORD1;
    float2 stroke_coordinate : TEXCOORD2;
    float4 transform0 : TEXCOORD3;
    float4 transform1 : TEXCOORD4;
    float4 transform2 : TEXCOORD5;
    float4 transform3 : TEXCOORD6;
};
struct Output {
    float4 position : SV_Position;
    float2 line_coordinate : TEXCOORD0;
    float line_length : TEXCOORD1;
    float line_half_width : TEXCOORD2;
    float3 world_position : TEXCOORD3;
};
Output main(Input input) {
    Output output;
    float4x4 model = float4x4(input.transform0, input.transform1, input.transform2, input.transform3);
    float3 world_a = mul(float4(input.start_position, 1.0f), model).xyz;
    float3 world_b = mul(float4(input.end_position, 1.0f), model).xyz;
    float4 a = mul(float4(world_a, 1.0f), value);
    float4 b = mul(float4(world_b, 1.0f), value);
    if (a.w <= 1e-5f && b.w <= 1e-5f) {
        output.position = float4(2.0f, 2.0f, 2.0f, 1.0f);
        output.line_coordinate = 0.0f;
        output.line_length = 0.0f;
        output.line_half_width = stroke_view.z;
        output.world_position = world_a;
        return output;
    }
    if (a.w <= 1e-5f) {
        float t = (1e-5f - a.w) / (b.w - a.w);
        a = lerp(a, b, t);
        world_a = lerp(world_a, world_b, t);
    } else if (b.w <= 1e-5f) {
        float t = (1e-5f - b.w) / (a.w - b.w);
        b = lerp(b, a, t);
        world_b = lerp(world_b, world_a, t);
    }
    output.world_position = lerp(world_a, world_b, input.stroke_coordinate.x);
    float2 delta = (b.xy / b.w - a.xy / a.w) * 0.5f * stroke_view.xy;
    float length_px = max(length(delta), 1e-5f);
    float2 tangent = delta / length_px;
    float2 normal = float2(-tangent.y, tangent.x);
    float along = input.stroke_coordinate.x;
    float half_width = stroke_view.z;
    float cap_extension = along < 0.5f ? -half_width : half_width;
    float4 clip = lerp(a, b, along);
    clip.xy += (normal * (input.stroke_coordinate.y * half_width) + tangent * cap_extension) *
               (2.0f / stroke_view.xy) * clip.w;
    output.position = clip;
    output.line_coordinate = float2(along * length_px + cap_extension,
                                    input.stroke_coordinate.y * half_width);
    output.line_length = length_px;
    output.line_half_width = half_width;
    return output;
}
