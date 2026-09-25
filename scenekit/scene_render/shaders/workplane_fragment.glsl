{{VERSION}}
{{FRAGMENT_PRECISION}}
uniform vec4 grid_eye_spacing;
uniform vec4 grid_forward;
uniform vec4 grid_right;
uniform vec4 grid_up;
in vec2 grid_ndc;
out vec4 fragment_color;

float grid_line(float coordinate) {
    float distance_to_line = abs(fract(coordinate + 0.5) - 0.5);
    return 1.0 - smoothstep(0.0, max(fwidth(coordinate), 1e-5), distance_to_line);
}

void main() {
    vec3 ray = grid_forward.xyz + grid_right.xyz * grid_ndc.x +
               grid_up.xyz * grid_ndc.y;
    float travel = -grid_eye_spacing.z / ray.z;
    if (travel <= 0.0 || abs(ray.z) < 1e-5) discard;
    vec2 world = grid_eye_spacing.xy + ray.xy * travel;
    float spacing = grid_eye_spacing.w;
    float footprint = max(length(dFdx(world)), length(dFdy(world)));
    float level = clamp(floor(log2(max(footprint / spacing * 2.0, 1.0))), 0.0, 16.0);
    float step_size = spacing * exp2(level);
    float coverage = max(grid_line(world.x / step_size), grid_line(world.y / step_size));
    float fade_distance = max(spacing * 15.0, grid_forward.w * 1.5);
    float fade = 1.0 - smoothstep(fade_distance, fade_distance * 3.0,
                                  travel * length(ray));
    float alpha = 0.12 * coverage * fade;
    fragment_color = vec4(vec3(0.15, 0.22, 0.28) * alpha, alpha);
}
