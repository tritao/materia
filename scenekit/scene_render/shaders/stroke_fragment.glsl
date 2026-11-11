{{VERSION}}
{{FRAGMENT_PRECISION}}

uniform vec4 stroke_color;
in vec3 line_coordinate_homogeneous;
in float line_length;
in float line_half_width;
in vec3 world_position;
in vec3 world_depth_bias;
flat in float support_sidedness;
in float step_coverage;
flat in float stroke_instance;
uniform vec4 clip_planes[32];
uniform vec4 clip_plane_count;
out vec4 fragment_color;

// Visibility against other parts, from the studio AO prepass's visible-surface buffer (app/plans/STROKE_VISIBILITY.md):
// the part id and world position of the surface each pixel shows.
uniform sampler2D pick_ids_texture;
uniform sampler2D surface_position_texture;
uniform mat4 surface_view_projection;
// xy: buffer size in pixels; z: 1 when the buffer holds this frame's surfaces; w: tolerance in world units.
uniform vec4 surface_viewport;

vec4 surface_at(ivec2 p) {
    return texture(surface_position_texture, (vec2(p) + 0.5) / surface_viewport.xy);
}

float part_at(ivec2 p) {
    vec4 c = texture(pick_ids_texture, (vec2(p) + 0.5) / surface_viewport.xy);
    return floor(c.r * 255.0 + 0.5) + 256.0 * floor(c.g * 255.0 + 0.5) + 65536.0 * floor(c.b * 255.0 + 0.5);
}

// One step along `axis` on the surface shown at p: the nearer neighbour showing the same part, or zero.
vec3 surface_step(ivec2 p, ivec2 axis, vec3 here, float part) {
    vec4 ahead = surface_at(p + axis), behind = surface_at(p - axis);
    bool forward = ahead.w > 0.5 && part_at(p + axis) == part;
    bool backward = behind.w > 0.5 && part_at(p - axis) == part;
    vec3 a = forward ? ahead.xyz - here : vec3(0.0), b = backward ? here - behind.xyz : vec3(0.0);
    if (forward && backward) return dot(a, a) <= dot(b, b) ? a : b;
    return forward ? a : b;
}

// Whether another part's visible surface lies in front of this stroke point. Comparing depths at the pixel would need
// a tolerance of a pixel's worth of the surface's slope, more than a thin wall over a hidden edge; instead that
// surface's tangent plane is carried to the exact point, which only needs a small tolerance. The stroke's own part is
// left to the depth test, which hides its own back edges.
bool behind_other_part(vec3 point) {
    if (surface_viewport.z < 0.5) return false;
    vec4 clip = surface_view_projection * vec4(point, 1.0);
    if (clip.w <= 0.0) return false;
    ivec2 p = ivec2(floor((clip.xy / clip.w * 0.5 + 0.5) * surface_viewport.xy));
    if (p.x < 1 || p.y < 1 || p.x >= int(surface_viewport.x) - 1 || p.y >= int(surface_viewport.y) - 1) return false;
    vec4 shown = surface_at(p);
    if (shown.w < 0.5) return false;
    float part = part_at(p);
    if (part == stroke_instance) return false;
    vec3 n = cross(surface_step(p, ivec2(1, 0), shown.xyz, part), surface_step(p, ivec2(0, 1), shown.xyz, part));
    float area = length(n);
    if (area <= 0.0) return false;
    n /= area;
    float footprint = sqrt(area);
    // Orient the normal toward the camera: a step along it must come nearer in depth.
    vec4 c0 = surface_view_projection * vec4(shown.xyz, 1.0);
    vec4 c1 = surface_view_projection * vec4(shown.xyz + n * footprint, 1.0);
    if (c1.z * c0.w > c0.z * c1.w) n = -n;
    return dot(n, point - shown.xyz) < -(surface_viewport.w + 0.02 * footprint);
}

void main() {
    if (abs(support_sidedness) == 1.0 && gl_FrontFacing != (support_sidedness > 0.0)) discard;
    if (behind_other_part(world_position)) discard;
    vec2 line_coordinate = line_coordinate_homogeneous.xy / line_coordinate_homogeneous.z;
    vec2 nearest = vec2(clamp(line_coordinate.x, 0.0, line_length), 0.0);
    float signed_distance = length(line_coordinate - nearest) - line_half_width;
    float coverage = clamp(0.5 - signed_distance / max(fwidth(signed_distance), 1e-4), 0.0, 1.0);
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
    coverage *= step_coverage;
    if (coverage <= 0.0) discard;
    fragment_color = vec4(stroke_color.rgb * coverage, stroke_color.a * coverage);
}
