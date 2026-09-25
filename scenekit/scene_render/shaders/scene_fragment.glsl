{{VERSION}}
{{FRAGMENT_PRECISION}}

uniform vec4 base_color;
uniform vec4 material_params;
uniform vec4 emissive;
uniform vec4 texture_flags;
uniform vec4 light_position_type[32];
uniform vec4 light_direction_range[32];
uniform vec4 light_color_intensity[32];
uniform vec4 light_cones[32];
uniform vec4 ambient_sky;
uniform vec4 ambient_ground;
uniform vec4 lighting_mode;
uniform vec4 camera_position;
uniform sampler2D base_color_texture;
uniform sampler2D metallic_roughness_texture;
uniform sampler2D normal_texture;
uniform sampler2D occlusion_texture;
uniform sampler2D emissive_texture;
in vec3 world_position;
in vec3 vertex_normal;
in vec2 vertex_texcoord;
in vec4 vertex_color;
uniform vec4 clip_planes[32];
uniform vec4 clip_plane_count;
out vec4 fragment_color;

vec3 srgb_to_linear(vec3 value) {
    value = max(value, vec3(0.0));
    return mix(value / 12.92, pow((value + 0.055) / 1.055, vec3(2.4)),
               step(vec3(0.04045), value));
}

vec3 linear_to_srgb(vec3 value) {
    value = max(value, vec3(0.0));
    return mix(value * 12.92, 1.055 * pow(value, vec3(1.0 / 2.4)) - 0.055,
               step(vec3(0.0031308), value));
}

void main() {
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip_plane_count.x))
            break;
        if (dot(clip_planes[index].xyz, world_position) + clip_planes[index].w < 0.0)
            discard;
    }
    vec4 texture_color = texture(base_color_texture, vertex_texcoord);
    vec3 N = normalize(vertex_normal);
    if (texture_flags.x > 0.5) {
        vec3 mapped = texture(normal_texture, vertex_texcoord).xyz * 2.0 - 1.0;
        vec3 dp1 = dFdx(world_position), dp2 = dFdy(world_position);
        vec2 duv1 = dFdx(vertex_texcoord), duv2 = dFdy(vertex_texcoord);
        vec3 T = cross(dp2, N) * duv1.x + cross(N, dp1) * duv2.x;
        vec3 B = cross(dp2, N) * duv1.y + cross(N, dp1) * duv2.y;
        float scale = inversesqrt(max(max(dot(T, T), dot(B, B)), 1.0e-8));
        N = normalize(T * (mapped.x * scale) + B * (mapped.y * scale) + N * mapped.z);
    }
    vec4 metal_rough = texture(metallic_roughness_texture, vertex_texcoord);
    float hemisphere = 0.5 + 0.5 * dot(N, vec3(0.0, 0.0, 1.0));
    vec3 base_texel = texture_flags.y > 0.5
                          ? srgb_to_linear(texture_color.rgb) : texture_color.rgb;
    vec3 albedo = srgb_to_linear(base_color.rgb) * base_texel *
                  srgb_to_linear(vertex_color.rgb);
    float metallic = clamp(material_params.x * metal_rough.b, 0.0, 1.0);
    float roughness = clamp(material_params.y * metal_rough.g, 0.045, 1.0);
    float alpha_ggx = roughness * roughness;
    float alpha2 = alpha_ggx * alpha_ggx;
    vec3 V = normalize(camera_position.xyz - world_position);
    float NdotV = max(dot(N, V), 0.0001);
    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 ambient = mix(ambient_ground.rgb, ambient_sky.rgb, hemisphere) *
                   albedo * (1.0 - metallic) *
                   texture(occlusion_texture, vertex_texcoord).r;
    vec3 direct = vec3(0.0);
    if (lighting_mode.z > 0.0) {
        vec3 R = reflect(-V, N);
        float reflection_height = clamp(0.5 + 0.5 * R.z, 0.0, 1.0);
        vec3 reflected = mix(ambient_ground.rgb, ambient_sky.rgb, reflection_height);
        vec3 average_env = 0.5 * (ambient_ground.rgb + ambient_sky.rgb);
        reflected = mix(average_env, reflected, 1.0 - roughness * roughness);
        vec3 grazing = max(F0, vec3(1.0 - roughness));
        vec3 env_fresnel = F0 + (grazing - F0) * pow(1.0 - NdotV, 5.0);
        ambient += reflected * env_fresnel * lighting_mode.z *
                   texture(occlusion_texture, vertex_texcoord).r;
    }
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
    vec3 emissive_texel = texture(emissive_texture, vertex_texcoord).rgb;
    if (texture_flags.z > 0.5)
        emissive_texel = srgb_to_linear(emissive_texel);
    vec3 color = ambient + direct + srgb_to_linear(emissive.rgb) * emissive_texel;
    float alpha = base_color.a * texture_color.a * vertex_color.a;
    if (material_params.w > 1.5 && alpha < material_params.z)
        discard;
    fragment_color = vec4(linear_to_srgb(color), alpha);
}
