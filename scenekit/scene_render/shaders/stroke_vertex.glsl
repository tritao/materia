{{VERSION}}
{{VERTEX_PRECISION}}

layout(location=0) in vec3 start_position;
layout(location=1) in vec3 end_position;
layout(location=2) in vec2 stroke_coordinate;
layout(location=3) in vec4 transform0;
layout(location=4) in vec4 transform1;
layout(location=5) in vec4 transform2;
layout(location=6) in vec4 transform3;
layout(location=7) in float stroke_step;
// The drawn instance's pick colour: which part this stroke belongs to in the visible-surface buffer.
layout(location=8) in vec4 pick_color;
uniform mat4 view_projection;
// xy: target size in pixels; z: the line's half width in pixels.
uniform vec4 stroke_view;
out vec3 line_coordinate_homogeneous;
out float line_length;
out float line_half_width;
// Centroid: with multisampling a border pixel whose centre lies outside the ribbon would extrapolate past the edge's
// end, to a point the edge never reaches.
centroid out vec3 world_position;
out float step_coverage;
flat out float stroke_instance;

// A step edge (app/plans/EDGE_SIGNIFICANCE.md) draws in proportion to its height over one pixel at its depth `w`;
// corners and other strokes carry no step and draw fully. The pixel's world size is 2 w / (P00 * width), where P00 is
// the projection's horizontal scale: the length of the view-projection's first row, since the view is a rotation.
float step_coverage_at(float w, mat4 model) {
    if (stroke_step <= 0.0) return 1.0;
    float horizontal = length(vec3(view_projection[0][0], view_projection[1][0], view_projection[2][0]));
    float pixel = 2.0 * max(w, 1e-6) / max(horizontal * stroke_view.x, 1e-12);
    float scale = (length(model[0].xyz) + length(model[1].xyz) + length(model[2].xyz)) / 3.0;
    return clamp(stroke_step * scale / pixel, 0.0, 1.0);
}

void main() {
    stroke_instance = floor(pick_color.r * 255.0 + 0.5) + 256.0 * floor(pick_color.g * 255.0 + 0.5) +
                      65536.0 * floor(pick_color.b * 255.0 + 0.5);
    mat4 model = mat4(transform0, transform1, transform2, transform3);
    vec3 world_a = (model * vec4(start_position, 1.0)).xyz;
    vec3 world_b = (model * vec4(end_position, 1.0)).xyz;
    vec4 a = view_projection * vec4(world_a, 1.0);
    vec4 b = view_projection * vec4(world_b, 1.0);
    if (a.w <= 1e-5 && b.w <= 1e-5) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        step_coverage = 0.0;
        line_coordinate_homogeneous = vec3(0.0, 0.0, 1.0);
        line_length = 0.0;
        line_half_width = stroke_view.z;
        world_position = world_a;
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
    vec2 delta = (b.xy / b.w - a.xy / a.w) * 0.5 * stroke_view.xy;
    float length_px = max(length(delta), 1e-5);
    vec2 tangent = delta / length_px;
    vec2 normal = vec2(-tangent.y, tangent.x);
    float half_width = stroke_view.z;
    float extent = half_width + 1.0;
    // The body spans [0,1]; separate circumscribed triangles cover the caps
    // on either side. Their x coordinates extend past 0 or 1, respectively.
    // Every vertex carries its centre-line point, which decides visibility (stroke_fragment.glsl).
    float cap_extension = (stroke_coordinate.x - along) * extent;
    vec4 clip = mix(a, b, along);
    vec2 pixel_offset = normal * (stroke_coordinate.y * extent) + tangent * cap_extension;
    clip.xy += pixel_offset * (2.0 / stroke_view.xy) * clip.w;
    gl_Position = clip;
    step_coverage = step_coverage_at(clip.w, model);
    // Recover screen-linear pixel coordinates after perspective interpolation.
    line_coordinate_homogeneous = vec3(
        vec2(along * length_px + cap_extension, stroke_coordinate.y * extent) * clip.w,
        clip.w);
    line_length = length_px;
    line_half_width = half_width;
}
