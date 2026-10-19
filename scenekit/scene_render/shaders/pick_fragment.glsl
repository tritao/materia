{{VERSION}}
{{FRAGMENT_PRECISION}}

in vec4 vertex_pick_color;
in vec3 world_position;
flat in float vertex_face_id;
uniform vec4 clip_planes[32];
uniform vec4 clip_plane_count;
out vec4 fragment_color;
layout(location=1) out vec4 fragment_subelement;
layout(location=2) out vec4 fragment_depth;
uniform vec4 pick_output;

void main() {
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip_plane_count.x))
            break;
        if (dot(clip_planes[index].xyz, world_position) + clip_planes[index].w < 0.0)
            discard;
    }
    uint id = uint(vertex_face_id);
    fragment_color = vertex_pick_color;
    fragment_subelement = vec4(float(id & 0xffu) / 255.0,
                               float((id >> 8u) & 0xffu) / 255.0,
                               float((id >> 16u) & 0xffu) / 255.0, 1.0);
    // Picking retains device depth; AO consumes positions with an explicit validity bit.
    fragment_depth = pick_output.x > 0.0 ? vec4(world_position, 1.0)
                                       : vec4(gl_FragCoord.z, 0.0, 0.0, 0.0);
}
