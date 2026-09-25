#include <metal_stdlib>

using namespace metal;

struct SceneMaterialParams
{
    float4 base_color;
    float4 surface_params;
    float4 emissive;
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

fragment float4 main0(SceneFragmentInput input [[stage_in]],
                      constant SceneMaterialParams &params [[buffer(0)]],
                      constant SceneClipParams &clip [[buffer(2)]],
                      constant SceneLightingParams &lights [[buffer(3)]],
                      texture2d<float> base_color_texture [[texture(0)]],
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
    float hemisphere = 0.5 + 0.5 * dot(N, float3(0.0, 0.0, 1.0));
    float3 diffuse = mix(lights.ambient_ground.rgb, lights.ambient_sky.rgb, hemisphere);
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
        diffuse += lights.light_color_intensity[index].rgb *
                   (intensity * attenuation * max(dot(N, L), 0.0));
    }
    float surface_response = mix(1.0, 0.65, clamp(params.surface_params.x, 0.0, 1.0)) *
                             mix(0.5, 1.0, clamp(params.surface_params.y, 0.0, 1.0));
    surface_response = mix(surface_response, 1.0, lights.lighting_mode.x);
    float3 color = params.base_color.rgb * texture_color.rgb * input.color0.rgb * diffuse *
                   surface_response + params.emissive.rgb;
    float alpha = params.base_color.a * texture_color.a * input.color0.a;
    if (params.surface_params.w > 1.5 && alpha < params.surface_params.z)
        discard_fragment();
    return float4(color, alpha);
}
