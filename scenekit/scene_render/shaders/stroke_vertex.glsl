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
uniform vec4 stroke_view;
out vec2 line_coordinate;
out float line_length;
out float line_half_width;
out vec3 world_position;

void main() {
    mat4 model = mat4(transform0, transform1, transform2, transform3);
    vec3 world_a = (model * vec4(start_position, 1.0)).xyz;
    vec3 world_b = (model * vec4(end_position, 1.0)).xyz;
    vec4 a = view_projection * vec4(world_a, 1.0);
    vec4 b = view_projection * vec4(world_b, 1.0);
    if (a.w <= 1e-5 && b.w <= 1e-5) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        line_coordinate = vec2(0.0);
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
    world_position = mix(world_a, world_b, stroke_coordinate.x);
    vec2 delta = (b.xy / b.w - a.xy / a.w) * 0.5 * stroke_view.xy;
    float length_px = max(length(delta), 1e-5);
    vec2 tangent = delta / length_px;
    vec2 normal = vec2(-tangent.y, tangent.x);
    float along = stroke_coordinate.x;
    float half_width = stroke_view.z;
    float cap_extension = along < 0.5 ? -half_width : half_width;
    vec4 clip = mix(a, b, along);
    vec2 pixel_offset = normal * (stroke_coordinate.y * half_width) + tangent * cap_extension;
    clip.xy += pixel_offset * (2.0 / stroke_view.xy) * clip.w;
    gl_Position = clip;
    line_coordinate = vec2(along * length_px + cap_extension, stroke_coordinate.y * half_width);
    line_length = length_px;
    line_half_width = half_width;
}
