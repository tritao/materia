#include <metal_stdlib>
using namespace metal;
struct GridParams { float4 eye_spacing; float4 forward; float4 right; float4 up; };
struct Input { float4 position [[position]]; float2 grid_ndc [[user(locn0)]]; };
float grid_line(float coordinate) {
    float distance_to_line = abs(fract(coordinate + 0.5) - 0.5);
    return 1.0 - smoothstep(0.0, max(fwidth(coordinate), 1e-5), distance_to_line);
}
fragment float4 main0(Input input [[stage_in]], constant GridParams &grid [[buffer(0)]]) {
    float3 ray = grid.forward.xyz + grid.right.xyz * input.grid_ndc.x +
                 grid.up.xyz * input.grid_ndc.y;
    float travel = -grid.eye_spacing.z / ray.z;
    if (travel <= 0.0 || abs(ray.z) < 1e-5) discard_fragment();
    float2 world = grid.eye_spacing.xy + ray.xy * travel;
    float spacing = grid.eye_spacing.w;
    float footprint = max(length(dfdx(world)), length(dfdy(world)));
    float level = clamp(floor(log2(max(footprint / spacing * 2.0, 1.0))), 0.0, 16.0);
    float step_size = spacing * exp2(level);
    float coverage = max(grid_line(world.x / step_size), grid_line(world.y / step_size));
    float fade_distance = max(spacing * 15.0, grid.forward.w * 1.5);
    float fade = 1.0 - smoothstep(fade_distance, fade_distance * 3.0,
                                  travel * length(ray));
    float alpha = 0.12 * coverage * fade;
    return float4(float3(0.15, 0.22, 0.28) * alpha, alpha);
}
