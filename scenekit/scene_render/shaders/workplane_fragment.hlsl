cbuffer grid_params : register(b0) {
    float4 grid_eye_spacing : packoffset(c0);
    float4 grid_forward : packoffset(c1);
    float4 grid_right : packoffset(c2);
    float4 grid_up : packoffset(c3);
};
struct Input { float4 position : SV_Position; float2 grid_ndc : TEXCOORD0; };
float grid_line(float coordinate) {
    float distance_to_line = abs(frac(coordinate + 0.5f) - 0.5f);
    return 1.0f - smoothstep(0.0f, max(fwidth(coordinate), 1e-5f), distance_to_line);
}
float4 main(Input input) : SV_Target0 {
    float3 ray = grid_forward.xyz + grid_right.xyz * input.grid_ndc.x +
                 grid_up.xyz * input.grid_ndc.y;
    float travel = -grid_eye_spacing.z / ray.z;
    if (travel <= 0.0f || abs(ray.z) < 1e-5f) discard;
    float2 world = grid_eye_spacing.xy + ray.xy * travel;
    float spacing = grid_eye_spacing.w;
    float footprint = max(length(ddx(world)), length(ddy(world)));
    float level = clamp(floor(log2(max(footprint / spacing * 2.0f, 1.0f))), 0.0f, 16.0f);
    float step_size = spacing * exp2(level);
    float coverage = max(grid_line(world.x / step_size), grid_line(world.y / step_size));
    float fade_distance = max(spacing * 15.0f, grid_forward.w * 1.5f);
    float fade = 1.0f - smoothstep(fade_distance, fade_distance * 3.0f,
                                   travel * length(ray));
    float alpha = 0.12f * coverage * fade;
    return float4(float3(0.15f, 0.22f, 0.28f) * alpha, alpha);
}
