{{VERSION}}
{{VERTEX_PRECISION}}

layout(location=0) in vec3 start_position;
layout(location=1) in vec3 end_position;
layout(location=2) in vec2 stroke_coordinate;
layout(location=3) in vec4 transform0;
layout(location=4) in vec4 transform1;
layout(location=5) in vec4 transform2;
layout(location=6) in vec4 transform3;
layout(location=7) in vec3 support_position;
layout(location=8) in float stroke_step;
// The drawn instance's pick colour: which part this stroke belongs to in the visible-surface buffer.
layout(location=9) in vec4 pick_color;
uniform mat4 view_projection;
uniform vec4 stroke_view[6];
out vec3 line_coordinate_homogeneous;
out float line_length;
out float line_half_width;
out vec3 world_position;
out vec3 world_depth_bias;
flat out float support_sidedness;
out float step_coverage;
flat out float stroke_instance;

// A step edge (app/plans/EDGE_SIGNIFICANCE.md) draws in proportion to its height over one pixel at its depth `w`;
// corners and other strokes carry no step and draw fully. The pixel's world size is 2 w / (P00 * width), where P00 is
// the projection's horizontal scale: the length of the view-projection's first row, since the view is a rotation.
float step_coverage_at(float w, mat4 model) {
    if (stroke_step <= 0.0) return 1.0;
    float horizontal = length(vec3(view_projection[0][0], view_projection[1][0], view_projection[2][0]));
    float pixel = 2.0 * max(w, 1e-6) / max(horizontal * stroke_view[0].x, 1e-12);
    float scale = (length(model[0].xyz) + length(model[1].xyz) + length(model[2].xyz)) / 3.0;
    return clamp(stroke_step * scale / pixel, 0.0, 1.0);
}

void main() {
    support_sidedness = 0.0;
    stroke_instance = floor(pick_color.r * 255.0 + 0.5) + 256.0 * floor(pick_color.g * 255.0 + 0.5) +
                      65536.0 * floor(pick_color.b * 255.0 + 0.5);
    mat4 model = mat4(transform0, transform1, transform2, transform3);
    vec3 world_a = (model * vec4(start_position, 1.0)).xyz;
    vec3 world_b = (model * vec4(end_position, 1.0)).xyz;
    vec4 a = view_projection * vec4(world_a, 1.0);
    vec4 b = view_projection * vec4(world_b, 1.0);
    if (stroke_coordinate.y > 1.5) {
        // Fill only the narrow strip inside an incident mesh triangle. Its
        // original vertices reproduce the shaded face's rasterized MSAA depth.
        vec3 world_c = (model * vec4(support_position, 1.0)).xyz;
        vec4 c = view_projection * vec4(world_c, 1.0);
        // A support triangle crossing the eye can no longer use finite
        // projected line coordinates. The ordinary clipped ribbon remains.
        if (a.w <= 1e-5 || b.w <= 1e-5 || c.w <= 1e-5) {
            gl_Position = vec4(2.0,2.0,2.0,1.0);
            step_coverage = 0.0;
            world_position = world_a;
            world_depth_bias = vec3(0.0);
            line_coordinate_homogeneous = vec3(0.0,0.0,1.0);
            line_length = 0.0;
            line_half_width = stroke_view[0].z;
            return;
        }
        float corner = stroke_coordinate.x;
        vec4 clip = corner < 0.5 ? a : (corner < 1.5 ? b : c);
        world_position = corner < 0.5 ? world_a : (corner < 1.5 ? world_b : world_c);
        vec2 delta = (b.xy / b.w - a.xy / a.w) * 0.5 * stroke_view[0].xy;
        float length_px = max(length(delta), 1e-5);
        vec2 tangent = delta / length_px;
        vec2 offset = (clip.xy / clip.w - a.xy / a.w) * 0.5 * stroke_view[0].xy;
        gl_Position = clip;
        step_coverage = step_coverage_at(clip.w, model);
        line_coordinate_homogeneous = vec3(vec2(dot(offset,tangent), dot(offset,vec2(-tangent.y,tangent.x))) * clip.w, clip.w);
        line_length = length_px;
        line_half_width = stroke_view[0].z;
        float parity = sign(dot(transform0.xyz, cross(transform1.xyz,transform2.xyz)));
        support_sidedness = parity * (stroke_view[0].w > 1.5 ? 2.0 : 1.0);
        vec4 dz = stroke_view[1] * (clip.w * stroke_view[5].x);
        world_depth_bias = dz.xyz - world_position * dz.w;
        return;
    }
    if (a.w <= 1e-5 && b.w <= 1e-5) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        step_coverage = 0.0;
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
    // The body spans [0,1]; separate circumscribed triangles cover the caps
    // on either side. Their x coordinates extend past 0 or 1, respectively.
    // Keeping the body endpoints fixed preserves its original depth slope.
    float cap_extension = (stroke_coordinate.x - along) * extent;
    vec4 clip = mix(a, b, along);
    vec2 pixel_offset = normal * (stroke_coordinate.y * extent) + tangent * cap_extension;
    clip.xy += pixel_offset * (2.0 / stroke_view[0].xy) * clip.w;
    // Caps may continue the depth ramp away from the eye, but must never
    // extrapolate towards it and peek through a foreground surface.
    float cap_depth = (cap_extension / length_px) * (b.z / b.w - a.z / a.w);
    if (stroke_view[0].w > 0.5 && stroke_view[0].w < 1.5) {
        // Expand contour ribbons in their cutting plane. Their depth must match
        // that plane at the expanded pixel, not at the unexpanded centerline;
        // otherwise one side sinks into the cap and the other floats above it.
        vec2 displacement = pixel_offset * (2.0 / stroke_view[0].xy) * clip.w;
        vec4 h = vec4(world_position, 1.0) + stroke_view[2] * displacement.x + stroke_view[3] * displacement.y;
        float denominator = dot(stroke_view[4], stroke_view[1]);
        if (abs(denominator) > 1.0e-20) {
            float dz_plane = -dot(stroke_view[4], h) / denominator;
            h += stroke_view[1] * dz_plane;
            clip.z += dz_plane;
            if (abs(h.w) > 1.0e-20) {
                world_position = h.xyz / h.w;
                // Preserve perspective-correct interpolation of the expanded
                // plane positions as well as their NDC depth.
                clip /= h.w;
            } else {
                clip = vec4(2.0, 2.0, 2.0, 1.0);
            }
        }
    } else {
        clip.z += max(cap_depth, 0.0) * clip.w;
    }
    gl_Position = clip;
    step_coverage = step_coverage_at(clip.w, model);
    // Convert the constant depth-unit tie-break to a world-space displacement.
    // Differentiate homogeneous unprojection rather than subtracting nearly
    // equal world positions. stroke_view[5].x is two depth-buffer steps of clip z.
    vec4 dz = stroke_view[1] * (clip.w * stroke_view[5].x);
    world_depth_bias = dz.xyz - world_position * dz.w;
    // Recover screen-linear pixel coordinates after perspective interpolation.
    line_coordinate_homogeneous = vec3(
        vec2(along * length_px + cap_extension, stroke_coordinate.y * extent) * clip.w,
        clip.w);
    line_length = length_px;
    line_half_width = half_width;
}
