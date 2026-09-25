cbuffer material_params : register(b0)
{
    float4 base_color : packoffset(c0);
    float4 surface_params : packoffset(c1);
    float4 emissive : packoffset(c2);
    float4 texture_flags : packoffset(c3);
};

cbuffer lighting_params : register(b3)
{
    float4 light_position_type[32] : packoffset(c0);
    float4 light_direction_range[32] : packoffset(c32);
    float4 light_color_intensity[32] : packoffset(c64);
    float4 light_cones[32] : packoffset(c96);
    float4 ambient_sky : packoffset(c128);
    float4 ambient_ground : packoffset(c129);
    float4 lighting_mode : packoffset(c130);
    float4 camera_position : packoffset(c131);
};

Texture2D base_color_texture : register(t0);
Texture2D metallic_roughness_texture : register(t1);
Texture2D normal_texture : register(t2);
Texture2D occlusion_texture : register(t3);
Texture2D emissive_texture : register(t4);
SamplerState base_color_sampler : register(s0);

cbuffer clip_params : register(b2)
{
    float4 clip_planes[32] : packoffset(c0);
    float4 clip_plane_count : packoffset(c32);
};

struct SceneFragmentInput
{
    float3 world_position : TEXCOORD0;
    float3 normal : TEXCOORD1;
    float2 texcoord0 : TEXCOORD2;
    float4 color0 : TEXCOORD3;
};

float3 srgb_to_linear(float3 value)
{
    value = max(value, 0.0f);
    return lerp(value / 12.92f, pow((value + 0.055f) / 1.055f, 2.4f),
                step(0.04045f, value));
}

float3 linear_to_srgb(float3 value)
{
    value = max(value, 0.0f);
    return lerp(value * 12.92f, 1.055f * pow(value, 1.0f / 2.4f) - 0.055f,
                step(0.0031308f, value));
}

float4 main(SceneFragmentInput input) : SV_Target0
{
    for (int index = 0; index < 32; ++index) {
        if (index >= int(clip_plane_count.x))
            break;
        if (dot(clip_planes[index].xyz, input.world_position) + clip_planes[index].w < 0.0f)
            discard;
    }
    float4 texture_color = base_color_texture.Sample(base_color_sampler, input.texcoord0);
    float3 N = normalize(input.normal);
    if (texture_flags.x > 0.5f) {
        float3 mapped = normal_texture.Sample(base_color_sampler, input.texcoord0).xyz * 2.0f - 1.0f;
        float3 dp1 = ddx(input.world_position), dp2 = ddy(input.world_position);
        float2 duv1 = ddx(input.texcoord0), duv2 = ddy(input.texcoord0);
        float3 T = cross(dp2, N) * duv1.x + cross(N, dp1) * duv2.x;
        float3 B = cross(dp2, N) * duv1.y + cross(N, dp1) * duv2.y;
        float scale = rsqrt(max(max(dot(T, T), dot(B, B)), 1.0e-8f));
        N = normalize(T * (mapped.x * scale) + B * (mapped.y * scale) + N * mapped.z);
    }
    float4 metal_rough = metallic_roughness_texture.Sample(base_color_sampler, input.texcoord0);
    float hemisphere = 0.5f + 0.5f * dot(N, float3(0.0f, 0.0f, 1.0f));
    float3 base_texel = texture_flags.y > 0.5f
                            ? srgb_to_linear(texture_color.rgb) : texture_color.rgb;
    float3 albedo = srgb_to_linear(base_color.rgb) * base_texel *
                    srgb_to_linear(input.color0.rgb);
    float metallic = saturate(surface_params.x * metal_rough.b);
    float roughness = clamp(surface_params.y * metal_rough.g, 0.045f, 1.0f);
    float alpha_ggx = roughness * roughness;
    float alpha2 = alpha_ggx * alpha_ggx;
    float3 V = normalize(camera_position.xyz - input.world_position);
    float NdotV = max(dot(N, V), 0.0001f);
    float3 F0 = lerp(float3(0.04f, 0.04f, 0.04f), albedo, metallic);
    float3 ambient = lerp(ambient_ground.rgb, ambient_sky.rgb, hemisphere) *
                     albedo * (1.0f - metallic) *
                     occlusion_texture.Sample(base_color_sampler, input.texcoord0).r;
    float3 direct = float3(0.0f, 0.0f, 0.0f);
    if (lighting_mode.z > 0.0f) {
        float3 R = reflect(-V, N);
        float reflection_height = saturate(0.5f + 0.5f * R.z);
        float3 reflected = lerp(ambient_ground.rgb, ambient_sky.rgb, reflection_height);
        float3 average_env = 0.5f * (ambient_ground.rgb + ambient_sky.rgb);
        reflected = lerp(average_env, reflected, 1.0f - roughness * roughness);
        float3 grazing = max(F0, 1.0f - roughness);
        float3 env_fresnel = F0 + (grazing - F0) * pow(1.0f - NdotV, 5.0f);
        ambient += reflected * env_fresnel * lighting_mode.z *
                   occlusion_texture.Sample(base_color_sampler, input.texcoord0).r;
    }
    for (int index = 0; index < 32; ++index) {
        if (index >= (int)lighting_mode.y) break;
        float intensity = max(light_color_intensity[index].w, 0.0f);
        if (intensity <= 0.0f) continue;
        float3 L;
        float attenuation = 1.0f;
        if (light_position_type[index].w < 1.5f) {
            L = normalize(light_position_type[index].xyz);
        } else {
            float3 displacement = light_position_type[index].xyz - input.world_position;
            float distance_to_light = length(displacement);
            float range = max(light_direction_range[index].w, 0.0001f);
            if (distance_to_light >= range) continue;
            L = displacement / max(distance_to_light, 0.0001f);
            float fraction = distance_to_light / range;
            float fade = 1.0f - fraction * fraction;
            attenuation = fade * fade / (1.0f + 4.0f * fraction * fraction);
            if (light_position_type[index].w > 2.5f) {
                float cone = dot(-L, normalize(light_direction_range[index].xyz));
                float width = max(light_cones[index].x - light_cones[index].y, 0.0001f);
                attenuation *= saturate((cone - light_cones[index].y) / width);
            }
        }
        float NdotL = max(dot(N, L), 0.0f);
        if (NdotL <= 0.0f) continue;
        float3 H = normalize(V + L);
        float NdotH = max(dot(N, H), 0.0f);
        float VdotH = max(dot(V, H), 0.0f);
        float d = NdotH * NdotH * (alpha2 - 1.0f) + 1.0f;
        float D = alpha2 / max(3.14159265f * d * d, 0.0001f);
        float k = (roughness + 1.0f) * (roughness + 1.0f) / 8.0f;
        float Gv = NdotV / (NdotV * (1.0f - k) + k);
        float Gl = NdotL / (NdotL * (1.0f - k) + k);
        float3 F = F0 + (1.0f - F0) * pow(1.0f - VdotH, 5.0f);
        float3 specular = D * Gv * Gl * F / max(4.0f * NdotV * NdotL, 0.0001f);
        float3 diffuse = (1.0f - F) * (1.0f - metallic) * albedo / 3.14159265f;
        direct += light_color_intensity[index].rgb *
                  (intensity * attenuation * NdotL * 3.14159265f) *
                  (diffuse + specular);
    }
    float3 emissive_texel = emissive_texture.Sample(base_color_sampler, input.texcoord0).rgb;
    if (texture_flags.z > 0.5f)
        emissive_texel = srgb_to_linear(emissive_texel);
    float3 color = ambient + direct + srgb_to_linear(emissive.rgb) * emissive_texel;
    float alpha = base_color.a * texture_color.a * input.color0.a;
    if (surface_params.w > 1.5f && alpha < surface_params.z)
        discard;
    return float4(linear_to_srgb(color), alpha);
}
