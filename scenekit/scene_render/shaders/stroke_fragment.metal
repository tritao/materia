#include <metal_stdlib>
using namespace metal;
struct StrokeColorParams { float4 stroke_color; };
struct ClipParams { float4 planes[32]; float4 count; };
struct Input {
    float4 position [[position]];
    float2 line_coordinate [[user(locn0)]];
    float line_length [[user(locn1)]];
    float line_half_width [[user(locn2)]];
    float3 world_position [[user(locn3)]];
};
fragment float4 main0(Input input [[stage_in]],
                      constant StrokeColorParams &params [[buffer(0)]],
                      constant ClipParams &clip [[buffer(2)]]) {
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip.count.x)) break;
        if (dot(clip.planes[index].xyz, input.world_position) + clip.planes[index].w < 0.0)
            discard_fragment();
    }
    float outside_x = max(max(-input.line_coordinate.x,
                              input.line_coordinate.x - input.line_length), 0.0);
    float outside_y = max(abs(input.line_coordinate.y) - input.line_half_width, 0.0);
    float signed_distance = length(float2(outside_x, outside_y)) +
        min(max(outside_x, abs(input.line_coordinate.y) - input.line_half_width), 0.0) -
        input.line_half_width;
    float coverage = clamp(0.5 - signed_distance / max(fwidth(signed_distance), 1e-4), 0.0, 1.0);
    return float4(params.stroke_color.rgb * coverage, params.stroke_color.a * coverage);
}
