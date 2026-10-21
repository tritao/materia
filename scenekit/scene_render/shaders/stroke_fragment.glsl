{{VERSION}}
{{FRAGMENT_PRECISION}}

uniform vec4 stroke_color;
in vec3 line_coordinate_homogeneous;
in float line_length;
in float line_half_width;
in vec3 world_position;
in vec3 world_depth_bias;
uniform vec4 clip_planes[32];
uniform vec4 clip_plane_count;
out vec4 fragment_color;

void main() {
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip_plane_count.x)) break;
        float distance_to_plane = dot(clip_planes[index].xyz, world_position) + clip_planes[index].w;
        // Expanded ribbons carry centerline depth/position. Near a cut their
        // lateral coverage can lie over a neighbour's cap even before depth bias.
        // Cut back by the ribbon + AA footprint in screen pixels, in plane units.
        // This removes clipped surface-edge remnants, not a synthetic cap outline.
        float footprint = (line_half_width + 1.0) * fwidth(distance_to_plane)
                        + abs(dot(clip_planes[index].xyz, world_depth_bias));
        if (clip_plane_count.y > 0.5) {
            // Contours omit their source plane and clip exactly to the others.
            if (distance_to_plane < 0.0) discard;
        } else if (distance_to_plane <= footprint) discard;
    }
    vec2 line_coordinate = line_coordinate_homogeneous.xy / line_coordinate_homogeneous.z;
    vec2 nearest = vec2(clamp(line_coordinate.x, 0.0, line_length), 0.0);
    float signed_distance = length(line_coordinate - nearest) - line_half_width;
    float coverage = clamp(0.5 - signed_distance / max(fwidth(signed_distance), 1e-4), 0.0, 1.0);
    fragment_color = vec4(stroke_color.rgb * coverage, stroke_color.a * coverage);
}
