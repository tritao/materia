#include <metal_stdlib>
using namespace metal;
struct GridParams { float4 eye_spacing; float4 forward; float4 right; float4 up; };
struct Input { float4 position [[position]]; float2 grid_ndc [[user(locn0)]]; };
float grid_coverage(float2 world, float step_size) {
    float2 coordinate = world / step_size;
    float2 distance_to_line = abs(fract(coordinate + 0.5) - 0.5);
    float2 pixel_width = max(fwidth(coordinate), float2(1e-5));
    float2 lines = float2(1.0) - smoothstep(float2(0.0), pixel_width, distance_to_line);
    return max(lines.x, lines.y);
}
fragment float4 main0(Input input [[stage_in]], constant GridParams &grid [[buffer(0)]]) {
    float3 ray = grid.forward.xyz + grid.right.xyz * input.grid_ndc.x +
                 grid.up.xyz * input.grid_ndc.y;
    float travel = -grid.eye_spacing.z / ray.z;
    if (travel <= 0.0 || abs(ray.z) < 1e-5) discard_fragment();
    float2 world = grid.eye_spacing.xy + ray.xy * travel;
    float spacing = grid.eye_spacing.w;
    float footprint = max(length(dfdx(world)), length(dfdy(world)));
    float level = clamp(log2(max(footprint / spacing * 12.0, 1.0)), 0.0, 16.0);
    float coarse_level = floor(level);
    float blend = smoothstep(0.0, 1.0, fract(level));
    float coverage = mix(grid_coverage(world, spacing * exp2(coarse_level)),
                         grid_coverage(world, spacing * exp2(min(coarse_level + 1.0, 16.0))),
                         blend);
    float fade_distance = max(spacing * 15.0, grid.forward.w * 1.5);
    float fade = 1.0 - smoothstep(fade_distance, fade_distance * 3.0,
                                  travel * length(ray));
    float screen_fade = 1.0 - smoothstep(0.0, 2.0, level);
    float alpha = 0.12 * coverage * fade * screen_fade;
    return float4(float3(0.15, 0.22, 0.28) * alpha, alpha);
}
