{{VERSION}}
{{FRAGMENT_PRECISION}}
uniform vec4 grid_eye_spacing; // xyz = eye, w = world-space cell size
uniform vec4 grid_forward;     // xyz = center ray, w = camera distance for fading
uniform vec4 grid_right;
uniform vec4 grid_up;
in vec2 grid_ndc;
out vec4 fragment_color;

// Keep these presentation values in sync with the HLSL and Metal variants.
const float GRID_COARSEN_AT_PIXELS = 12.0;
const float GRID_MAX_LEVEL = 16.0;
const float GRID_FADE_START_LEVEL = 1.5;
const float GRID_FADE_END_LEVEL = 4.5;
const float GRID_MIN_FADE_CELLS = 15.0;
const float GRID_CAMERA_FADE_MULTIPLIER = 1.5;
const float GRID_FADE_END_MULTIPLIER = 3.0;
const float GRID_OPACITY = 0.12;
const vec3 GRID_COLOR = vec3(0.15, 0.22, 0.28);

float grid_coverage(vec2 world, float step_size) {
    vec2 coordinate = world / step_size;
    vec2 distance_to_line = abs(fract(coordinate + 0.5) - 0.5);
    vec2 pixel_width = max(fwidth(coordinate), vec2(1e-5));
    vec2 lines = vec2(1.0) - smoothstep(vec2(0.0), pixel_width, distance_to_line);
    return max(lines.x, lines.y);
}

void main() {
    vec3 ray = grid_forward.xyz + grid_right.xyz * grid_ndc.x +
               grid_up.xyz * grid_ndc.y;
    float travel = -grid_eye_spacing.z / ray.z;
    if (travel <= 0.0 || abs(ray.z) < 1e-5) discard;
    vec2 world = grid_eye_spacing.xy + ray.xy * travel;
    float spacing = grid_eye_spacing.w;
    float pixel_footprint = max(length(dFdx(world)), length(dFdy(world)));
    // Blend adjacent power-of-two spacings before fine cells crowd the screen.
    float level = clamp(log2(max(pixel_footprint * GRID_COARSEN_AT_PIXELS /
                                 spacing, 1.0)), 0.0, GRID_MAX_LEVEL);
    float base_level = floor(level);
    float level_blend = smoothstep(0.0, 1.0, fract(level));
    float base_coverage = grid_coverage(world, spacing * exp2(base_level));
    float coarse_coverage = grid_coverage(world,
        spacing * exp2(min(base_level + 1.0, GRID_MAX_LEVEL)));
    float coverage = mix(base_coverage, coarse_coverage, level_blend);

    float fade_start = max(spacing * GRID_MIN_FADE_CELLS,
                           grid_forward.w * GRID_CAMERA_FADE_MULTIPLIER);
    float distance_fade = 1.0 - smoothstep(fade_start,
        fade_start * GRID_FADE_END_MULTIPLIER, travel * length(ray));
    float density_fade = 1.0 - smoothstep(GRID_FADE_START_LEVEL,
                                          GRID_FADE_END_LEVEL, level);
    float alpha = GRID_OPACITY * coverage * distance_fade * density_fade;
    fragment_color = vec4(GRID_COLOR * alpha, alpha);
}
