cbuffer material_params : register(b0)
{
    float4 base_color : packoffset(c0);
    float4 surface_params : packoffset(c1);
    float4 emissive : packoffset(c2);
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
};

Texture2D base_color_texture : register(t0);
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
    float hemisphere = 0.5f + 0.5f * dot(N, float3(0.0f, 0.0f, 1.0f));
    float3 diffuse = lerp(ambient_ground.rgb, ambient_sky.rgb, hemisphere);
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
        diffuse += light_color_intensity[index].rgb *
                   (intensity * attenuation * max(dot(N, L), 0.0f));
    }
    float surface_response = lerp(1.0f, 0.65f, saturate(surface_params.x)) *
                             lerp(0.5f, 1.0f, saturate(surface_params.y));
    surface_response = lerp(surface_response, 1.0f, lighting_mode.x);
    float3 color = base_color.rgb * texture_color.rgb * input.color0.rgb * diffuse *
                   surface_response + emissive.rgb;
    float alpha = base_color.a * texture_color.a * input.color0.a;
    if (surface_params.w > 1.5f && alpha < surface_params.z)
        discard;
    return float4(color, alpha);
}
