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
uniform vec4 camera_view_direction;
uniform vec4 ao_params;
uniform vec4 ao_viewport;
uniform sampler2D base_color_texture;
uniform sampler2D metallic_roughness_texture;
uniform sampler2D normal_texture;
uniform sampler2D occlusion_texture;
uniform sampler2D emissive_texture;
uniform sampler2D ao_position_texture;
in vec3 world_position;
in vec3 vertex_normal;
in vec2 vertex_texcoord;
in vec4 vertex_color;
in vec4 surface_highlight;
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

// Studio light i as a soft panel: a Gaussian lobe of angular radius light_cones[i].z around its direction,
// widened by the surface's specular spread. Its radiance keeps the panel's energy equal to the directional
// light's irradiance (pi * intensity), so I / rho^2 at the centre. Zero radius is a point light (no panel).
vec3 studio_panels(vec3 R, float roughness) {
    vec3 radiance = vec3(0.0);
    float spread = 1.2 * roughness * roughness;
    for (int index = 0; index < 3; ++index) {
        float radius = light_cones[index].z;
        if (radius <= 0.0) continue;
        float rho2 = radius * radius + spread * spread;
        float angle = acos(clamp(dot(R, normalize(light_position_type[index].xyz)), -1.0, 1.0));
        radiance += light_color_intensity[index].rgb * light_color_intensity[index].w *
                    exp(-angle * angle / rho2) / rho2;
    }
    return radiance;
}

// Khronos PBR Neutral tone mapping (also model-viewer's "neutral"), applied after exposure.
vec3 neutral_tone_map(vec3 color) {
    const float start_compression = 0.8 - 0.04;
    const float desaturation = 0.15;
    float x = min(color.r, min(color.g, color.b));
    float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
    color -= offset;
    float peak = max(color.r, max(color.g, color.b));
    if (peak < start_compression) return color;
    const float d = 1.0 - start_compression;
    float new_peak = 1.0 - d * d / (peak + d - start_compression);
    color *= new_peak / peak;
    float g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0);
    return mix(color, new_peak * vec3(1.0), g);
}

// Short-range screen-space obscurance of indirect light. Mesh normals keep flat
// faces flat; world-space range rejection prevents distant silhouettes making halos.
float screen_occlusion(vec3 N) {
    if (ao_params.x <= 0.0 || ao_params.y <= 0.0 || material_params.w > 1.5)
        return 1.0;
    vec2 pixel = gl_FragCoord.xy;
    float world_per_pixel = max(min(length(dFdx(world_position)), length(dFdy(world_position))), 1.0e-6);
    float radius_pixels = clamp(ao_params.x / world_per_pixel, 1.0, 32.0);
    float occlusion = 0.0;
    for (int direction = 0; direction < 8; ++direction) {
        float angle = float(direction) * 0.7853981634;
        vec2 axis = vec2(cos(angle), sin(angle));
        for (int ring = 0; ring < 2; ++ring) {
            float radius = ring == 0 ? 0.35 : 0.8;
            vec2 uv = (floor(pixel + axis * radius_pixels * radius) + 0.5) * ao_viewport.xy;
            if (uv.x <= 0.0 || uv.x >= 1.0 || uv.y <= 0.0 || uv.y >= 1.0) continue;
            // Store positions directly: reconstructing from device depth becomes
            // ill-conditioned when a close camera needs a very small near plane.
            vec4 sample_position = texture(ao_position_texture, uv);
            if (sample_position.w == 0.0) continue;
            vec3 delta = sample_position.xyz - world_position;
            float distance_to_sample = length(delta);
            if (distance_to_sample < 1.0e-6 || distance_to_sample >= ao_params.x) continue;
            float horizon = max(dot(N, delta / distance_to_sample) - ao_params.z, 0.0);
            float falloff = 1.0 - distance_to_sample / ao_params.x;
            occlusion += horizon * falloff * falloff;
        }
    }
    return 1.0 - ao_params.y * clamp(3.0 * occlusion / 16.0, 0.0, 0.75);
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
        // Derivatives depend on world units and pixel footprint. Rescale before
        // squaring rather than clamping their length to a world-scale epsilon:
        // that clamp makes normal-map strength vary with distance and zoom.
        float frame_max = max(max(max(abs(T.x), abs(T.y)), abs(T.z)),
                              max(max(abs(B.x), abs(B.y)), abs(B.z)));
        if (frame_max > 0.0) {
            T /= frame_max;
            B /= frame_max;
            float scale = inversesqrt(max(dot(T, T), dot(B, B)));
            N = normalize(T * (mapped.x * scale) + B * (mapped.y * scale) + N * mapped.z);
        }
    }
    vec4 metal_rough = texture(metallic_roughness_texture, vertex_texcoord);
    float hemisphere = 0.5 + 0.5 * dot(N, vec3(0.0, 0.0, 1.0));
    vec3 base_texel = texture_flags.y > 0.5
                          ? srgb_to_linear(texture_color.rgb) : texture_color.rgb;
    vec3 albedo = srgb_to_linear(base_color.rgb) * base_texel *
                  srgb_to_linear(vertex_color.rgb);
    albedo = mix(albedo, srgb_to_linear(surface_highlight.rgb), surface_highlight.a);
    float metallic = clamp(material_params.x * metal_rough.b, 0.0, 1.0);
    float roughness = clamp(material_params.y * metal_rough.g, 0.045, 1.0);
    if (lighting_mode.x > 0.5) roughness = max(roughness, ao_params.w);
    float alpha_ggx = roughness * roughness;
    float alpha2 = alpha_ggx * alpha_ggx;
    vec3 V = camera_position.w > 0.5
                 ? normalize(camera_position.xyz - world_position)
                 : normalize(camera_view_direction.xyz);
    float NdotV = max(dot(N, V), 0.0001);
    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    vec3 ambient = mix(ambient_ground.rgb, ambient_sky.rgb, hemisphere) *
                   albedo * (1.0 - metallic) *
                   texture(occlusion_texture, vertex_texcoord).r;
    vec3 direct = vec3(0.0);
    bool studio = lighting_mode.x > 0.5;
    // light_cones[0].w marks a studio environment: soft panels, a sky/ground room and display mapping.
    if (lighting_mode.z > 0.0 || studio) {
        vec3 R = reflect(-V, N);
        vec3 reflected;
        if (studio && light_cones[0].w > 0.5) {
            // The studio room is sky above the horizon and ground below (whose irradiance is the
            // hemisphere ambient above); a reflection sees that edge blurred by the surface's roughness.
            float edge = 0.05 + roughness * roughness;
            reflected = mix(ambient_ground.rgb, ambient_sky.rgb, smoothstep(-edge, edge, R.z)) *
                            lighting_mode.z + studio_panels(R, roughness);
        } else {
            float reflection_height = clamp(0.5 + 0.5 * R.z, 0.0, 1.0);
            reflected = mix(ambient_ground.rgb, ambient_sky.rgb, reflection_height);
            vec3 average_env = 0.5 * (ambient_ground.rgb + ambient_sky.rgb);
            reflected = mix(average_env, reflected, 1.0 - roughness * roughness) * lighting_mode.z;
        }
        vec3 grazing = max(F0, vec3(1.0 - roughness));
        vec3 env_fresnel = F0 + (grazing - F0) * pow(1.0 - NdotV, 5.0);
        ambient += reflected * env_fresnel *
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
        // A studio panel's specular comes from its reflection above, not from a point highlight.
        vec3 specular = studio && light_cones[index].z > 0.0
                            ? vec3(0.0) : D * Gv * Gl * F / max(4.0 * NdotV * NdotL, 0.0001);
        vec3 diffuse = (1.0 - F) * (1.0 - metallic) * albedo / 3.14159265;
        direct += light_color_intensity[index].rgb *
                  (intensity * attenuation * NdotL * 3.14159265) *
                  (diffuse + specular);
    }
    vec3 emissive_texel = texture(emissive_texture, vertex_texcoord).rgb;
    if (texture_flags.z > 0.5)
        emissive_texel = srgb_to_linear(emissive_texel);
    ambient *= screen_occlusion(N);
    vec3 color = ambient + direct + srgb_to_linear(emissive.rgb) * emissive_texel;
    float alpha = base_color.a * texture_color.a * vertex_color.a;
    // Only alpha-mask materials cut out below their cutoff (mode 2). Blended
    // materials (mode 3) keep every fragment and let the blend state fade it.
    if (material_params.w > 1.5 && material_params.w < 2.5 && alpha < material_params.z)
        discard;
    if (lighting_mode.w > 0.0)
        color = neutral_tone_map(color * lighting_mode.w);
    fragment_color = vec4(linear_to_srgb(color), alpha);
}
