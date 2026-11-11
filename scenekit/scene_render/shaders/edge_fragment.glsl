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

// x: the line's half width in pixels (strokes use 0.85).
uniform vec4 edge_style;

vec2 texel_uv(ivec2 p) {
    return (vec2(p) + 0.5) * edge_view.xy;
}

float decode(vec4 c) {
    return floor(c.r * 255.0 + 0.5) + 256.0 * floor(c.g * 255.0 + 0.5) + 65536.0 * floor(c.b * 255.0 + 0.5);
}

// A 5 x 5 window around this pixel, fetched once: index (x + 2) + 5 (y + 2).
vec4 window_position[25];
float window_instance[25];

int slot(ivec2 o) {
    return (o.x + 2) + 5 * (o.y + 2);
}

bool same_instance(ivec2 a, ivec2 b) {
    return window_position[slot(b)].w > 0.5 && window_instance[slot(b)] == window_instance[slot(a)];
}

// One step along `axis` on a's own surface: the nearer of the two neighbours, so it does not straddle a crease or
// an occlusion. Zero when neither neighbour shows the same part.
vec3 surface_step(ivec2 a, ivec2 axis) {
    vec3 here = window_position[slot(a)].xyz;
    bool forward = same_instance(a, a + axis), backward = same_instance(a, a - axis);
    vec3 ahead = forward ? window_position[slot(a + axis)].xyz - here : vec3(0.0);
    vec3 behind = backward ? here - window_position[slot(a - axis)].xyz : vec3(0.0);
    if (forward && backward) return dot(ahead, ahead) <= dot(behind, behind) ? ahead : behind;
    return forward ? ahead : behind;
}

// Normal and world size of one pixel at each of the 3 x 3 pixels around this one: index (x + 1) + 3 (y + 1).
vec4 frame[9];

vec4 surface_frame(ivec2 a) {
    vec3 n = cross(surface_step(a, ivec2(1, 0)), surface_step(a, ivec2(0, 1)));
    float area = length(n);
    if (area <= 0.0) return vec4(0.0);
    return vec4(n / area, sqrt(area));
}

bool edge_between(ivec2 a, ivec2 b) {
    vec4 pa = window_position[slot(a)], pb = window_position[slot(b)];
    bool covered_a = pa.w > 0.5, covered_b = pb.w > 0.5;
    if (covered_a != covered_b) return true;
    if (!covered_a) return false;
    // Different parts meet: an outline or a contact line.
    if (window_instance[slot(a)] != window_instance[slot(b)]) return true;
    // One part: face ids may be triangles, so judge the shape. A line where it creases or occludes itself.
    vec4 na = frame[(a.x + 1) + 3 * (a.y + 1)], nb = frame[(b.x + 1) + 3 * (b.y + 1)];
    if (na.w <= 0.0 || nb.w <= 0.0) return false;
    if (dot(na.xyz, nb.xyz) < edge_view.z) return true;
    return abs(dot(na.xyz, pb.xyz - pa.xyz)) > edge_view.w * na.w;
}

void main() {
    ivec2 p = ivec2(gl_FragCoord.xy);
    for (int y = -2; y <= 2; ++y) for (int x = -2; x <= 2; ++x) {
        vec2 uv = texel_uv(p + ivec2(x, y));
        window_position[slot(ivec2(x, y))] = texture(surface_position_texture, uv);
        window_instance[slot(ivec2(x, y))] = decode(texture(pick_ids_texture, uv));
    }
    for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 1; ++x)
        frame[(x + 1) + 3 * (y + 1)] = window_position[slot(ivec2(x, y))].w > 0.5 ? surface_frame(ivec2(x, y)) : vec4(0.0);
    // A boundary lies midway between two pixels that differ. Coverage falls off with the distance from this pixel's
    // centre to the nearest one: a line of the strokes' width with a one-pixel soft edge.
    float nearest = 4.0;
    for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 0; ++x) {
        if (edge_between(ivec2(x, y), ivec2(x + 1, y))) nearest = min(nearest, length(vec2(float(x) + 0.5, float(y))));
    }
    for (int y = -1; y <= 0; ++y) for (int x = -1; x <= 1; ++x) {
        if (edge_between(ivec2(x, y), ivec2(x, y + 1))) nearest = min(nearest, length(vec2(float(x), float(y) + 0.5)));
    }
    float coverage = clamp(edge_style.x + 0.5 - nearest, 0.0, 1.0);
    if (coverage <= 0.0) discard;
    fragment_color = vec4(edge_color.rgb * edge_color.a * coverage, edge_color.a * coverage);
}
