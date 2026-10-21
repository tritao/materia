{{VERSION}}
{{VERTEX_PRECISION}}

layout(location=0) in vec3 start_position;
layout(location=1) in vec3 end_position;
layout(location=2) in vec2 stroke_coordinate;
layout(location=3) in vec4 transform0;
layout(location=4) in vec4 transform1;
layout(location=5) in vec4 transform2;
layout(location=6) in vec4 transform3;
uniform mat4 view_projection;
uniform vec4 stroke_view[2];
out vec3 line_coordinate_homogeneous;
out float line_length;
out float line_half_width;
out vec3 world_position;
out vec3 world_depth_bias;

void main() {
    mat4 model = mat4(transform0, transform1, transform2, transform3);
    vec3 world_a = (model * vec4(start_position, 1.0)).xyz;
    vec3 world_b = (model * vec4(end_position, 1.0)).xyz;
    vec4 a = view_projection * vec4(world_a, 1.0);
    vec4 b = view_projection * vec4(world_b, 1.0);
    if (a.w <= 1e-5 && b.w <= 1e-5) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        line_coordinate_homogeneous = vec3(0.0, 0.0, 1.0);
        line_length = 0.0;
        line_half_width = stroke_view[0].z;
        world_position = world_a;
        world_depth_bias = vec3(0.0);
        return;
    }
    if (a.w <= 1e-5) {
        float t = (1e-5 - a.w) / (b.w - a.w);
        a = mix(a, b, t);
        world_a = mix(world_a, world_b, t);
    } else if (b.w <= 1e-5) {
        float t = (1e-5 - b.w) / (a.w - b.w);
        b = mix(b, a, t);
        world_b = mix(world_b, world_a, t);
    }
    float along = clamp(stroke_coordinate.x, 0.0, 1.0);
    world_position = mix(world_a, world_b, along);
    vec2 delta = (b.xy / b.w - a.xy / a.w) * 0.5 * stroke_view[0].xy;
    float length_px = max(length(delta), 1e-5);
    vec2 tangent = delta / length_px;
    vec2 normal = vec2(-tangent.y, tangent.x);
    float half_width = stroke_view[0].z;
    float extent = half_width + 1.0;
    // The body spans [0,1]; separate cap quads use [-1,0] and [1,2].
    // Keeping the body endpoints fixed preserves its original depth slope.
    float cap_extension = (stroke_coordinate.x - along) * extent;
    vec4 clip = mix(a, b, along);
    vec2 pixel_offset = normal * (stroke_coordinate.y * extent) + tangent * cap_extension;
    clip.xy += pixel_offset * (2.0 / stroke_view[0].xy) * clip.w;
    // Caps may continue the depth ramp away from the eye, but must never
    // extrapolate towards it and peek through a foreground surface.
    float cap_depth = (cap_extension / length_px) * (b.z / b.w - a.z / a.w);
    clip.z += max(cap_depth, 0.0) * clip.w;
    gl_Position = clip;
    // Convert the constant depth-unit tie-break to a world-space displacement.
    // Differentiate homogeneous unprojection rather than subtracting nearly
    // equal world positions. Two D24 units conservatively cover depth rounding.
    vec4 dz = stroke_view[1] * (clip.w * (4.0 / 16777216.0));
    world_depth_bias = dz.xyz - world_position * dz.w;
    // Recover screen-linear pixel coordinates after perspective interpolation.
    line_coordinate_homogeneous = vec3(
        vec2(along * length_px + cap_extension, stroke_coordinate.y * extent) * clip.w,
        clip.w);
    line_length = length_px;
    line_half_width = half_width;
}
