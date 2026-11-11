{{VERSION}}
{{FRAGMENT_PRECISION}}

// Screen-space CAD edges (app/plans/SCREEN_EDGES.md): a line where neighbouring pixels show different parts, or
// where one part's surface creases or occludes itself. Only visible surfaces are in these
// buffers, so a hidden edge can never be drawn and nothing ties with a face.
// Line colours on light and on dark materials; the AO prepass marks each surface pixel's tone in its position's w
// (1 light, 2 dark), from the same rule as stroke colours.
uniform vec4 edge_color;
uniform vec4 edge_color_on_dark;
// x, y: one over the target size; z: cosine of the crease angle; w: occlusion gap, in pixel footprints.
uniform vec4 edge_view;
uniform sampler2D pick_ids_texture;
uniform sampler2D surface_position_texture;
in vec2 vertex_uv;
out vec4 fragment_color;

// x: the line's half width in pixels (strokes use 0.85).
uniform vec4 edge_style;
// Rows z and w of the view-projection, for which of two surfaces is nearer.
uniform vec4 edge_depth_z;
uniform vec4 edge_depth_w;

vec2 texel_uv(ivec2 p) {
    return (vec2(p) + 0.5) * edge_view.xy;
}

float decode(vec4 c) {
    return floor(c.r * 255.0 + 0.5) + 256.0 * floor(c.g * 255.0 + 0.5) + 65536.0 * floor(c.b * 255.0 + 0.5);
}

// A 7 x 7 window around this pixel, fetched once: index (x + 3) + 7 (y + 3). Boundaries are found in the inner
// 5 x 5, whose surface normals need one more pixel on each side.
vec4 window_position[49];
float window_instance[49];

int slot(ivec2 o) {
    return (o.x + 3) + 7 * (o.y + 3);
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

// Normal and world size of one pixel at each of the 5 x 5 pixels around this one: index (x + 2) + 5 (y + 2).
vec4 frame[25];

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
    vec4 na = frame[(a.x + 2) + 5 * (a.y + 2)], nb = frame[(b.x + 2) + 5 * (b.y + 2)];
    if (na.w <= 0.0 || nb.w <= 0.0) return false;
    if (dot(na.xyz, nb.xyz) < edge_view.z) return true;
    return abs(dot(na.xyz, pb.xyz - pa.xyz)) > edge_view.w * na.w;
}

// Which line a boundary belongs to: the two parts it separates (-1 for background), or one part's own crease.
vec2 boundary_key(ivec2 a, ivec2 b) {
    float ia = window_position[slot(a)].w > 0.5 ? window_instance[slot(a)] : -1.0;
    float ib = window_position[slot(b)].w > 0.5 ? window_instance[slot(b)] : -1.0;
    return ia != ib ? vec2(min(ia, ib), max(ia, ib)) : vec2(ia, -2.0);
}

// The tone of the line between a and b: the nearer surface owns it, as a part owns its outline.
float boundary_tone(ivec2 a, ivec2 b) {
    vec4 pa = window_position[slot(a)], pb = window_position[slot(b)];
    if (pa.w <= 0.5) return pb.w;
    if (pb.w <= 0.5) return pa.w;
    float za = dot(edge_depth_z.xyz, pa.xyz) + edge_depth_z.w, wa = dot(edge_depth_w.xyz, pa.xyz) + edge_depth_w.w;
    float zb = dot(edge_depth_z.xyz, pb.xyz) + edge_depth_z.w, wb = dot(edge_depth_w.xyz, pb.xyz) + edge_depth_w.w;
    return za * wb <= zb * wa ? pa.w : pb.w;
}

// Boundaries in the inner 5 x 5: each lies midway between two pixels that differ. 20 between horizontal neighbours,
// 20 between vertical ones.
const int BOUNDARIES = 40;
vec2 boundary_point[40];
vec2 boundary_line[40];
bool boundary_on[40];

ivec2 boundary_a(int i) {
    return i < 20 ? ivec2(i - 4 * (i / 4) - 2, i / 4 - 2) : ivec2((i - 20) - 5 * ((i - 20) / 5) - 2, (i - 20) / 5 - 2);
}

ivec2 boundary_b(int i) {
    return boundary_a(i) + (i < 20 ? ivec2(1, 0) : ivec2(0, 1));
}

// The pixel centre's distance to the line through boundaries of `key`. A pixel-snapped boundary is a staircase;
// a least-squares line through its midpoints recovers the true edge to a fraction of a pixel, so the line is
// antialiased like a stroke. Fit locally around `seed`, then refit with every point of that line near the first
// fit. Where the points are not collinear (a corner, a tight curve) the nearest midpoint's distance stands.
float line_distance(vec2 key, int seed) {
    vec2 origin = boundary_point[seed];
    vec2 direction = vec2(0.0), centre = origin;
    bool fitted = false;
    for (int pass = 0; pass < 2; ++pass) {
        float weight = 0.0;
        vec2 sum = vec2(0.0);
        vec3 moments = vec3(0.0);
        for (int i = 0; i < BOUNDARIES; ++i) {
            if (!boundary_on[i] || boundary_line[i] != key) continue;
            vec2 q = boundary_point[i];
            bool near_line = pass == 0 ? length(q - origin) <= 1.6
                : abs(dot(q - centre, vec2(-direction.y, direction.x))) <= 0.75 && abs(dot(q - origin, direction)) <= 2.6;
            if (!near_line) continue;
            weight += 1.0;
            sum += q;
            moments += vec3(q.x * q.x, q.x * q.y, q.y * q.y);
        }
        if (weight < 3.0) return length(origin);
        vec2 mean = sum / weight;
        vec3 spread = moments / weight - vec3(mean.x * mean.x, mean.x * mean.y, mean.y * mean.y);
        float half_trace = 0.5 * (spread.x + spread.z);
        float root = sqrt(max(0.25 * (spread.x - spread.z) * (spread.x - spread.z) + spread.y * spread.y, 0.0));
        float major = half_trace + root, minor = half_trace - root;
        // A staircase's midpoints lie within a quarter pixel or so of its line; a corner spreads much wider.
        if (minor > 0.12 || major < 0.2) return length(origin);
        float angle = 0.5 * atan(2.0 * spread.y, spread.x - spread.z);
        direction = vec2(cos(angle), sin(angle));
        centre = mean;
        fitted = true;
    }
    if (!fitted) return length(origin);
    // Past the end of a line the fitted line runs on; the nearest midpoint limits that to under a pixel.
    return max(abs(dot(-centre, vec2(-direction.y, direction.x))), length(origin) - 0.75);
}

void main() {
    ivec2 p = ivec2(gl_FragCoord.xy);
    for (int y = -3; y <= 3; ++y) for (int x = -3; x <= 3; ++x) {
        vec2 uv = texel_uv(p + ivec2(x, y));
        window_position[slot(ivec2(x, y))] = texture(surface_position_texture, uv);
        window_instance[slot(ivec2(x, y))] = decode(texture(pick_ids_texture, uv));
    }
    for (int y = -2; y <= 2; ++y) for (int x = -2; x <= 2; ++x)
        frame[(x + 2) + 5 * (y + 2)] = window_position[slot(ivec2(x, y))].w > 0.5 ? surface_frame(ivec2(x, y)) : vec4(0.0);
    int seed = -1;
    float nearest = 4.0;
    for (int i = 0; i < BOUNDARIES; ++i) {
        ivec2 a = boundary_a(i), b = boundary_b(i);
        boundary_point[i] = 0.5 * vec2(a + b);
        boundary_on[i] = edge_between(a, b);
        boundary_line[i] = boundary_on[i] ? boundary_key(a, b) : vec2(0.0);
        float d = length(boundary_point[i]);
        if (boundary_on[i] && d < nearest) {
            nearest = d;
            seed = i;
        }
    }
    if (seed < 0 || nearest > edge_style.x + 1.5) discard;
    // The nearest line, and the nearest other line that meets it here (a junction), each fitted on its own.
    int other = -1;
    float other_nearest = 4.0;
    for (int i = 0; i < BOUNDARIES; ++i) {
        float d = length(boundary_point[i]);
        if (boundary_on[i] && boundary_line[i] != boundary_line[seed] && d < other_nearest) {
            other_nearest = d;
            other = i;
        }
    }
    float distance_to_line = line_distance(boundary_line[seed], seed);
    int owner = seed;
    if (other >= 0) {
        float other_distance = line_distance(boundary_line[other], other);
        if (other_distance < distance_to_line) {
            distance_to_line = other_distance;
            owner = other;
        }
    }
    // A line of the strokes' half width with a one-pixel soft edge.
    float coverage = clamp(edge_style.x + 0.5 - distance_to_line, 0.0, 1.0);
    if (coverage <= 0.0) discard;
    float tone = boundary_tone(boundary_a(owner), boundary_b(owner));
    vec4 color = tone > 1.5 ? edge_color_on_dark : edge_color;
    fragment_color = vec4(color.rgb * color.a * coverage, color.a * coverage);
}
