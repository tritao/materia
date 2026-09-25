#include <metal_stdlib>
using namespace metal;
// eye_spacing.w is cell size; forward.w is camera distance for fading.
struct GridParams { float4 eye_spacing; float4 forward; float4 right; float4 up; };
struct Input { float4 position [[position]]; float2 grid_ndc [[user(locn0)]]; };
// Keep these presentation values in sync with the GLSL and HLSL variants.
constant float GRID_COARSEN_AT_PIXELS = 12.0;
constant float GRID_MAX_LEVEL = 16.0;
constant float GRID_FADE_START_LEVEL = 1.5;
constant float GRID_FADE_END_LEVEL = 4.5;
constant float GRID_MIN_FADE_CELLS = 15.0;
constant float GRID_CAMERA_FADE_MULTIPLIER = 1.5;
constant float GRID_FADE_END_MULTIPLIER = 3.0;
constant float GRID_OPACITY = 0.12;
constant float3 GRID_COLOR = float3(0.15, 0.22, 0.28);
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
    float pixel_footprint = max(length(dfdx(world)), length(dfdy(world)));
    float level = clamp(log2(max(pixel_footprint * GRID_COARSEN_AT_PIXELS /
                                 spacing, 1.0)), 0.0, GRID_MAX_LEVEL);
    float base_level = floor(level);
    float level_blend = smoothstep(0.0, 1.0, fract(level));
    float base_coverage = grid_coverage(world, spacing * exp2(base_level));
    float coarse_coverage = grid_coverage(world,
        spacing * exp2(min(base_level + 1.0, GRID_MAX_LEVEL)));
    float coverage = mix(base_coverage, coarse_coverage, level_blend);

    float fade_start = max(spacing * GRID_MIN_FADE_CELLS,
                           grid.forward.w * GRID_CAMERA_FADE_MULTIPLIER);
    float distance_fade = 1.0 - smoothstep(fade_start,
        fade_start * GRID_FADE_END_MULTIPLIER, travel * length(ray));
    float density_fade = 1.0 - smoothstep(GRID_FADE_START_LEVEL,
                                          GRID_FADE_END_LEVEL, level);
    float alpha = GRID_OPACITY * coverage * distance_fade * density_fade;
    return float4(GRID_COLOR * alpha, alpha);
}
