#include <metal_stdlib>

using namespace metal;

struct SceneMaterialParams
{
    float4 base_color;
    float4 surface_params;
    float4 emissive;
    float4 texture_flags;
};

struct SceneLightingParams
{
    float4 light_position_type[32];
    float4 light_direction_range[32];
    float4 light_color_intensity[32];
    float4 light_cones[32];
    float4 ambient_sky;
    float4 ambient_ground;
    float4 lighting_mode;
    float4 camera_position;
};

struct SceneClipParams
{
    float4 planes[32];
    float4 count;
};

struct SceneFragmentInput
{
    float3 world_position [[user(locn0)]];
    float3 normal [[user(locn1)]];
    float2 texcoord0 [[user(locn2)]];
    float4 color0 [[user(locn3)]];
};

float3 srgb_to_linear(float3 value)
{
    value = max(value, float3(0.0));
    return mix(value / 12.92, pow((value + 0.055) / 1.055, float3(2.4)),
               step(float3(0.04045), value));
}

float3 linear_to_srgb(float3 value)
{
    value = max(value, float3(0.0));
    return mix(value * 12.92, 1.055 * pow(value, float3(1.0 / 2.4)) - 0.055,
               step(float3(0.0031308), value));
}

fragment float4 main0(SceneFragmentInput input [[stage_in]],
                      constant SceneMaterialParams &params [[buffer(0)]],
                      constant SceneClipParams &clip [[buffer(2)]],
                      constant SceneLightingParams &lights [[buffer(3)]],
                      texture2d<float> base_color_texture [[texture(0)]],
                      texture2d<float> metallic_roughness_texture [[texture(1)]],
                      texture2d<float> normal_texture [[texture(2)]],
                      texture2d<float> occlusion_texture [[texture(3)]],
                      texture2d<float> emissive_texture [[texture(4)]],
                      sampler base_color_sampler [[sampler(0)]])
{
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip.count.x))
            break;
        if (dot(clip.planes[index].xyz, input.world_position) + clip.planes[index].w < 0.0)
            discard_fragment();
    }
    float4 texture_color = base_color_texture.sample(base_color_sampler, input.texcoord0);
    float3 N = normalize(input.normal);
    if (params.texture_flags.x > 0.5) {
        float3 mapped = normal_texture.sample(base_color_sampler, input.texcoord0).xyz * 2.0 - 1.0;
        float3 dp1 = dfdx(input.world_position), dp2 = dfdy(input.world_position);
        float2 duv1 = dfdx(input.texcoord0), duv2 = dfdy(input.texcoord0);
        float3 T = cross(dp2, N) * duv1.x + cross(N, dp1) * duv2.x;
        float3 B = cross(dp2, N) * duv1.y + cross(N, dp1) * duv2.y;
        float scale = rsqrt(max(max(dot(T, T), dot(B, B)), 1.0e-8));
        N = normalize(T * (mapped.x * scale) + B * (mapped.y * scale) + N * mapped.z);
    }
    float4 metal_rough = metallic_roughness_texture.sample(base_color_sampler, input.texcoord0);
    float hemisphere = 0.5 + 0.5 * dot(N, float3(0.0, 0.0, 1.0));
    float3 base_texel = params.texture_flags.y > 0.5
                            ? srgb_to_linear(texture_color.rgb) : texture_color.rgb;
    float3 albedo = srgb_to_linear(params.base_color.rgb) * base_texel *
                    srgb_to_linear(input.color0.rgb);
    float metallic = clamp(params.surface_params.x * metal_rough.b, 0.0, 1.0);
    float roughness = clamp(params.surface_params.y * metal_rough.g, 0.045, 1.0);
    float alpha_ggx = roughness * roughness;
    float alpha2 = alpha_ggx * alpha_ggx;
    float3 V = normalize(lights.camera_position.xyz - input.world_position);
    float NdotV = max(dot(N, V), 0.0001);
    float3 F0 = mix(float3(0.04), albedo, metallic);
    float3 ambient = mix(lights.ambient_ground.rgb, lights.ambient_sky.rgb, hemisphere) *
                     albedo * (1.0 - metallic) *
                     occlusion_texture.sample(base_color_sampler, input.texcoord0).r;
    float3 direct = float3(0.0);
    if (lights.lighting_mode.z > 0.0) {
        float3 R = reflect(-V, N);
        float reflection_height = clamp(0.5 + 0.5 * R.z, 0.0, 1.0);
        float3 reflected = mix(lights.ambient_ground.rgb, lights.ambient_sky.rgb, reflection_height);
        float3 average_env = 0.5 * (lights.ambient_ground.rgb + lights.ambient_sky.rgb);
        reflected = mix(average_env, reflected, 1.0 - roughness * roughness);
        float3 grazing = max(F0, float3(1.0 - roughness));
        float3 env_fresnel = F0 + (grazing - F0) * pow(1.0 - NdotV, 5.0);
        ambient += reflected * env_fresnel * lights.lighting_mode.z *
                   occlusion_texture.sample(base_color_sampler, input.texcoord0).r;
    }
    for (int index = 0; index < 32; ++index) {
        if (index >= int(lights.lighting_mode.y)) break;
        float intensity = max(lights.light_color_intensity[index].w, 0.0);
        if (intensity <= 0.0) continue;
        float3 L;
        float attenuation = 1.0;
        if (lights.light_position_type[index].w < 1.5) {
            L = normalize(lights.light_position_type[index].xyz);
        } else {
            float3 displacement = lights.light_position_type[index].xyz - input.world_position;
            float distance_to_light = length(displacement);
            float range = max(lights.light_direction_range[index].w, 0.0001);
            if (distance_to_light >= range) continue;
            L = displacement / max(distance_to_light, 0.0001);
            float fraction = distance_to_light / range;
            float fade = 1.0 - fraction * fraction;
            attenuation = fade * fade / (1.0 + 4.0 * fraction * fraction);
            if (lights.light_position_type[index].w > 2.5) {
                float cone = dot(-L, normalize(lights.light_direction_range[index].xyz));
                float width = max(lights.light_cones[index].x - lights.light_cones[index].y, 0.0001);
                attenuation *= clamp((cone - lights.light_cones[index].y) / width, 0.0, 1.0);
            }
        }
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL <= 0.0) continue;
        float3 H = normalize(V + L);
        float NdotH = max(dot(N, H), 0.0);
        float VdotH = max(dot(V, H), 0.0);
        float d = NdotH * NdotH * (alpha2 - 1.0) + 1.0;
        float D = alpha2 / max(3.14159265 * d * d, 0.0001);
        float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
        float Gv = NdotV / (NdotV * (1.0 - k) + k);
        float Gl = NdotL / (NdotL * (1.0 - k) + k);
        float3 F = F0 + (1.0 - F0) * pow(1.0 - VdotH, 5.0);
        float3 specular = D * Gv * Gl * F / max(4.0 * NdotV * NdotL, 0.0001);
        float3 diffuse = (1.0 - F) * (1.0 - metallic) * albedo / 3.14159265;
        direct += lights.light_color_intensity[index].rgb *
                  (intensity * attenuation * NdotL * 3.14159265) *
                  (diffuse + specular);
    }
    float3 emissive_texel = emissive_texture.sample(base_color_sampler, input.texcoord0).rgb;
    if (params.texture_flags.z > 0.5)
        emissive_texel = srgb_to_linear(emissive_texel);
    float3 color = ambient + direct + srgb_to_linear(params.emissive.rgb) * emissive_texel;
    float alpha = params.base_color.a * texture_color.a * input.color0.a;
    if (params.surface_params.w > 1.5 && alpha < params.surface_params.z)
        discard_fragment();
    return float4(linear_to_srgb(color), alpha);
}
