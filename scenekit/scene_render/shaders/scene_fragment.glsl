{{VERSION}}
{{FRAGMENT_PRECISION}}

uniform vec4 base_color;
uniform vec4 material_params;
uniform vec4 emissive;
uniform vec4 light_position_type[32];
uniform vec4 light_direction_range[32];
uniform vec4 light_color_intensity[32];
uniform vec4 light_cones[32];
uniform vec4 ambient_sky;
uniform vec4 ambient_ground;
uniform vec4 lighting_mode;
uniform sampler2D base_color_texture;
in vec3 world_position;
in vec3 vertex_normal;
in vec2 vertex_texcoord;
in vec4 vertex_color;
uniform vec4 clip_planes[32];
uniform vec4 clip_plane_count;
out vec4 fragment_color;

void main() {
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip_plane_count.x))
            break;
        if (dot(clip_planes[index].xyz, world_position) + clip_planes[index].w < 0.0)
            discard;
    }
    vec4 texture_color = texture(base_color_texture, vertex_texcoord);
    vec3 N = normalize(vertex_normal);
    float hemisphere = 0.5 + 0.5 * dot(N, vec3(0.0, 0.0, 1.0));
    vec3 diffuse = mix(ambient_ground.rgb, ambient_sky.rgb, hemisphere);
    for (int index = 0; index < 32; ++index) {
        if (index >= int(lighting_mode.y)) break;
        float intensity = max(light_color_intensity[index].w, 0.0);
        if (intensity <= 0.0) continue;
        vec3 L;
        float attenuation = 1.0;
        if (light_position_type[index].w < 1.5) {
            L = normalize(light_position_type[index].xyz);
        } else {
            vec3 displacement = light_position_type[index].xyz - world_position;
            float distance_to_light = length(displacement);
            float range = max(light_direction_range[index].w, 0.0001);
            if (distance_to_light >= range) continue;
            L = displacement / max(distance_to_light, 0.0001);
            float fraction = distance_to_light / range;
            float fade = 1.0 - fraction * fraction;
            attenuation = fade * fade / (1.0 + 4.0 * fraction * fraction);
            if (light_position_type[index].w > 2.5) {
                float cone = dot(-L, normalize(light_direction_range[index].xyz));
                float width = max(light_cones[index].x - light_cones[index].y, 0.0001);
                attenuation *= clamp((cone - light_cones[index].y) / width, 0.0, 1.0);
            }
        }
        diffuse += light_color_intensity[index].rgb *
                   (intensity * attenuation * max(dot(N, L), 0.0));
    }
    float surface_response = mix(1.0, 0.65, clamp(material_params.x, 0.0, 1.0)) *
                             mix(0.5, 1.0, clamp(material_params.y, 0.0, 1.0));
    surface_response = mix(surface_response, 1.0, lighting_mode.x);
    vec3 color = base_color.rgb * texture_color.rgb * vertex_color.rgb * diffuse *
                 surface_response + emissive.rgb;
    float alpha = base_color.a * texture_color.a * vertex_color.a;
    if (material_params.w > 1.5 && alpha < material_params.z)
        discard;
    fragment_color = vec4(color, alpha);
}
