#include <metal_stdlib>
#include <simd/simd.h>
using namespace metal;
struct ViewParams { float4x4 value; };
struct StrokeViewParams { float4 stroke_view; };
struct Input {
    float3 start_position [[attribute(0)]];
    float3 end_position [[attribute(1)]];
    float2 stroke_coordinate [[attribute(2)]];
    float4 transform0 [[attribute(3)]];
    float4 transform1 [[attribute(4)]];
    float4 transform2 [[attribute(5)]];
    float4 transform3 [[attribute(6)]];
};
struct Output {
    float4 position [[position]];
    float2 line_coordinate [[user(locn0)]];
    float line_length [[user(locn1)]];
    float line_half_width [[user(locn2)]];
    float3 world_position [[user(locn3)]];
};
vertex Output main0(Input input [[stage_in]], constant ViewParams &view [[buffer(1)]],
                    constant StrokeViewParams &params [[buffer(3)]]) {
    Output output = {};
    float4x4 model = float4x4(input.transform0, input.transform1, input.transform2, input.transform3);
    float3 world_a = (model * float4(input.start_position, 1.0)).xyz;
    float3 world_b = (model * float4(input.end_position, 1.0)).xyz;
    float4 a = view.value * float4(world_a, 1.0);
    float4 b = view.value * float4(world_b, 1.0);
    if (a.w <= 1e-5 && b.w <= 1e-5) {
        output.position = float4(2.0, 2.0, 2.0, 1.0);
        output.line_coordinate = float2(0.0);
        output.line_length = 0.0;
        output.line_half_width = params.stroke_view.z;
        output.world_position = world_a;
        return output;
    }
    if (a.w <= 1e-5) {
        float t = (1e-5 - a.w) / (b.w - a.w);
        a = mix(a, b, t);
        world_a = mix(world_a, world_b, t);
    } else if (b.w <= 1e-5) {
        float t = (1e-5 - b.w) / (a.w - b.w);
        b = mix(b, a, t);
        world_b = mix(world_b, world_a, t);
    }
    output.world_position = mix(world_a, world_b, input.stroke_coordinate.x);
    float2 delta = (b.xy / b.w - a.xy / a.w) * 0.5 * params.stroke_view.xy;
    float length_px = max(length(delta), 1e-5);
    float2 tangent = delta / length_px;
    float2 normal = float2(-tangent.y, tangent.x);
    float along = input.stroke_coordinate.x;
    float half_width = params.stroke_view.z;
    float extent = half_width + 1.0;
    float cap_extension = along < 0.5 ? -extent : extent;
    float4 clip = mix(a, b, along);
    clip.xy += (normal * (input.stroke_coordinate.y * extent) + tangent * cap_extension) *
               (2.0 / params.stroke_view.xy) * clip.w;
    output.position = clip;
    output.line_coordinate = float2(along * length_px + cap_extension,
                                    input.stroke_coordinate.y * extent);
    output.line_length = length_px;
    output.line_half_width = half_width;
    return output;
}
