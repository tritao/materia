{{VERSION}}
{{FRAGMENT_PRECISION}}

uniform vec4 stroke_color;
in vec2 line_coordinate;
in float line_length;
in float line_half_width;
in vec3 world_position;
uniform vec4 clip_planes[32];
uniform vec4 clip_plane_count;
out vec4 fragment_color;

void main() {
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip_plane_count.x)) break;
        if (dot(clip_planes[index].xyz, world_position) + clip_planes[index].w < 0.0) discard;
    }
    float outside_x = max(max(-line_coordinate.x, line_coordinate.x - line_length), 0.0);
    float outside_y = max(abs(line_coordinate.y) - line_half_width, 0.0);
    float signed_distance = length(vec2(outside_x, outside_y)) +
        min(max(outside_x, abs(line_coordinate.y) - line_half_width), 0.0) - line_half_width;
    float coverage = clamp(0.5 - signed_distance / max(fwidth(signed_distance), 1e-4), 0.0, 1.0);
    fragment_color = vec4(stroke_color.rgb * coverage, stroke_color.a * coverage);
}
