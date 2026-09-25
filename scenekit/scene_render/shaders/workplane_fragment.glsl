{{VERSION}}
{{FRAGMENT_PRECISION}}
uniform vec4 grid_eye_spacing;
uniform vec4 grid_forward;
uniform vec4 grid_right;
uniform vec4 grid_up;
in vec2 grid_ndc;
out vec4 fragment_color;

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
    float footprint = max(length(dFdx(world)), length(dFdy(world)));
    // Begin coarsening at twelve pixels per cell, then crossfade to
    // the next spacing instead of switching abruptly at powers of two.
    float level = clamp(log2(max(footprint / spacing * 12.0, 1.0)), 0.0, 16.0);
    float coarse_level = floor(level);
    float blend = smoothstep(0.0, 1.0, fract(level));
    float coverage = mix(grid_coverage(world, spacing * exp2(coarse_level)),
                         grid_coverage(world, spacing * exp2(min(coarse_level + 1.0, 16.0))),
                         blend);
    float fade_distance = max(spacing * 15.0, grid_forward.w * 1.5);
    float fade = 1.0 - smoothstep(fade_distance, fade_distance * 3.0,
                                  travel * length(ray));
    float screen_fade = 1.0 - smoothstep(0.0, 2.0, level);
    float alpha = 0.12 * coverage * fade * screen_fade;
    fragment_color = vec4(vec3(0.15, 0.22, 0.28) * alpha, alpha);
}
