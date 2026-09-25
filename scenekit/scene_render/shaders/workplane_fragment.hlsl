cbuffer grid_params : register(b0) {
    // xyz = eye, w = cell size; forward.w = camera distance for fading.
    float4 grid_eye_spacing : packoffset(c0);
    float4 grid_forward : packoffset(c1);
    float4 grid_right : packoffset(c2);
    float4 grid_up : packoffset(c3);
};
struct Input { float4 position : SV_Position; float2 grid_ndc : TEXCOORD0; };
// Keep these presentation values in sync with the GLSL and Metal variants.
static const float GRID_COARSEN_AT_PIXELS = 12.0f;
static const float GRID_MAX_LEVEL = 16.0f;
static const float GRID_FADE_START_LEVEL = 1.5f;
static const float GRID_FADE_END_LEVEL = 4.5f;
static const float GRID_MIN_FADE_CELLS = 15.0f;
static const float GRID_CAMERA_FADE_MULTIPLIER = 1.5f;
static const float GRID_FADE_END_MULTIPLIER = 3.0f;
static const float GRID_OPACITY = 0.12f;
static const float3 GRID_COLOR = float3(0.15f, 0.22f, 0.28f);
float grid_coverage(float2 world, float step_size) {
    float2 coordinate = world / step_size;
    float2 distance_to_line = abs(frac(coordinate + 0.5f) - 0.5f);
    float2 pixel_width = max(fwidth(coordinate), float2(1e-5f, 1e-5f));
    float2 lines = float2(1.0f, 1.0f) - smoothstep(float2(0.0f, 0.0f),
                                                   pixel_width, distance_to_line);
    return max(lines.x, lines.y);
}
float4 main(Input input) : SV_Target0 {
    float3 ray = grid_forward.xyz + grid_right.xyz * input.grid_ndc.x +
                 grid_up.xyz * input.grid_ndc.y;
    float travel = -grid_eye_spacing.z / ray.z;
    if (travel <= 0.0f || abs(ray.z) < 1e-5f) discard;
    float2 world = grid_eye_spacing.xy + ray.xy * travel;
    float spacing = grid_eye_spacing.w;
    float pixel_footprint = max(length(ddx(world)), length(ddy(world)));
    float level = clamp(log2(max(pixel_footprint * GRID_COARSEN_AT_PIXELS /
                                 spacing, 1.0f)), 0.0f, GRID_MAX_LEVEL);
    float base_level = floor(level);
    float level_blend = smoothstep(0.0f, 1.0f, frac(level));
    float base_coverage = grid_coverage(world, spacing * exp2(base_level));
    float coarse_coverage = grid_coverage(world,
        spacing * exp2(min(base_level + 1.0f, GRID_MAX_LEVEL)));
    float coverage = lerp(base_coverage, coarse_coverage, level_blend);

    float fade_start = max(spacing * GRID_MIN_FADE_CELLS,
                           grid_forward.w * GRID_CAMERA_FADE_MULTIPLIER);
    float distance_fade = 1.0f - smoothstep(fade_start,
        fade_start * GRID_FADE_END_MULTIPLIER, travel * length(ray));
    float density_fade = 1.0f - smoothstep(GRID_FADE_START_LEVEL,
                                           GRID_FADE_END_LEVEL, level);
    float alpha = GRID_OPACITY * coverage * distance_fade * density_fade;
    return float4(GRID_COLOR * alpha, alpha);
}
