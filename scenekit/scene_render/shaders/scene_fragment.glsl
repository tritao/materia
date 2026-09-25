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
uniform vec4 camera_position;
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
    vec3 albedo = base_color.rgb * texture_color.rgb * vertex_color.rgb;
    float metallic = clamp(material_params.x, 0.0, 1.0);
    float roughness = clamp(material_params.y, 0.045, 1.0);
    float alpha_ggx = roughness * roughness;
    float alpha2 = alpha_ggx * alpha_ggx;
    vec3 V = normalize(camera_position.xyz - world_position);
    float NdotV = max(dot(N, V), 0.0001);
    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 ambient = mix(ambient_ground.rgb, ambient_sky.rgb, hemisphere) *
                   albedo * (1.0 - metallic);
    vec3 direct = vec3(0.0);
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
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL <= 0.0) continue;
        vec3 H = normalize(V + L);
        float NdotH = max(dot(N, H), 0.0);
        float VdotH = max(dot(V, H), 0.0);
        float d = NdotH * NdotH * (alpha2 - 1.0) + 1.0;
        float D = alpha2 / max(3.14159265 * d * d, 0.0001);
        float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
        float Gv = NdotV / (NdotV * (1.0 - k) + k);
        float Gl = NdotL / (NdotL * (1.0 - k) + k);
        vec3 F = F0 + (1.0 - F0) * pow(1.0 - VdotH, 5.0);
        vec3 specular = D * Gv * Gl * F / max(4.0 * NdotV * NdotL, 0.0001);
        vec3 diffuse = (1.0 - F) * (1.0 - metallic) * albedo / 3.14159265;
        direct += light_color_intensity[index].rgb *
                  (intensity * attenuation * NdotL * 3.14159265) *
                  (diffuse + specular);
    }
    vec3 color = ambient + direct + emissive.rgb;
    float alpha = base_color.a * texture_color.a * vertex_color.a;
    if (material_params.w > 1.5 && alpha < material_params.z)
        discard;
    fragment_color = vec4(color, alpha);
}
