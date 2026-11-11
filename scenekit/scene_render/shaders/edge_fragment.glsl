{{VERSION}}
{{FRAGMENT_PRECISION}}

// Screen-space CAD edges (app/plans/SCREEN_EDGES.md): a line where neighbouring pixels show different parts, or
// where one part's surface creases or occludes itself. Only visible surfaces are in these
// buffers, so a hidden edge can never be drawn and nothing ties with a face.
uniform vec4 edge_color;
// x, y: one over the target size; z: cosine of the crease angle; w: occlusion gap, in pixel footprints.
uniform vec4 edge_view;
uniform sampler2D pick_ids_texture;
uniform sampler2D surface_position_texture;
in vec2 vertex_uv;
out vec4 fragment_color;

vec2 texel_uv(ivec2 p) {
    return (vec2(p) + 0.5) * edge_view.xy;
}

float decode(vec4 c) {
    return floor(c.r * 255.0 + 0.5) + 256.0 * floor(c.g * 255.0 + 0.5) + 65536.0 * floor(c.b * 255.0 + 0.5);
}

float instance_at(ivec2 p) {
    return decode(texture(pick_ids_texture, texel_uv(p)));
}

vec4 position_at(ivec2 p) {
    return texture(surface_position_texture, texel_uv(p));
}

bool same_instance(ivec2 p, ivec2 q) {
    return position_at(q).w > 0.5 && instance_at(q) == instance_at(p);
}

// One step along `axis` on p's own surface: of the two neighbours, the nearer in space, so the step does not
// straddle a crease or an occlusion. Zero when neither neighbour shows the same part.
vec3 surface_step(ivec2 p, ivec2 axis) {
    vec3 here = position_at(p).xyz;
    bool forward = same_instance(p, p + axis), backward = same_instance(p, p - axis);
    vec3 ahead = forward ? position_at(p + axis).xyz - here : vec3(0.0);
    vec3 behind = backward ? here - position_at(p - axis).xyz : vec3(0.0);
    if (forward && backward) return dot(ahead, ahead) <= dot(behind, behind) ? ahead : behind;
    return forward ? ahead : behind;
}

// The surface normal at p and its world size of one pixel; zero when p has no neighbour on its part.
vec4 surface_frame(ivec2 p) {
    vec3 n = cross(surface_step(p, ivec2(1, 0)), surface_step(p, ivec2(0, 1)));
    float area = length(n);
    if (area <= 0.0) return vec4(0.0);
    return vec4(n / area, sqrt(area));
}

bool edge_between(ivec2 p, ivec2 q) {
    vec4 a = position_at(p), b = position_at(q);
    bool covered_p = a.w > 0.5, covered_q = b.w > 0.5;
    if (covered_p != covered_q) return true;
    if (!covered_p) return false;
    // Different parts meet: an outline or a contact line.
    if (instance_at(p) != instance_at(q)) return true;
    // One part: face ids may be triangles, so judge the shape. A line where it creases or occludes itself.
    vec4 np = surface_frame(p), nq = surface_frame(q);
    if (np.w <= 0.0 || nq.w <= 0.0) return false;
    if (dot(np.xyz, nq.xyz) < edge_view.z) return true;
    return abs(dot(np.xyz, b.xyz - a.xyz)) > edge_view.w * np.w;
}

void main() {
    ivec2 p = ivec2(gl_FragCoord.xy);
    bool edge = edge_between(p, p + ivec2(1, 0)) || edge_between(p, p - ivec2(1, 0))
             || edge_between(p, p + ivec2(0, 1)) || edge_between(p, p - ivec2(0, 1));
    if (!edge) discard;
    fragment_color = vec4(edge_color.rgb * edge_color.a, edge_color.a);
}
