{{VERSION}}
{{FRAGMENT_PRECISION}}

uniform vec4 color_params;
uniform vec4 distortion_params;
uniform ivec4 random_params;
uniform sampler2D source_image;
in vec2 vertex_uv;
out vec4 fragment_color;

uint hash32(uint value) {
    // Stateless integer mixing makes the result reproducible for every pixel.
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    return value ^ (value >> 16u);
}

float pixel_random(uvec2 pixel, uint salt) {
    // Seed and sequence identify the frame; coordinates and salt identify the
    // independent random event within that frame.
    uint value = uint(random_params.x) ^ uint(random_params.y);
    value ^= uint(random_params.z) ^ uint(random_params.w);
    value ^= pixel.x * 0x9e3779b9u;
    value ^= pixel.y * 0x85ebca6bu;
    value ^= salt * 0xc2b2ae35u;
    return float(hash32(value) & 0x00ffffffu) / 16777216.0;
}

float pixel_gaussian(uvec2 pixel) {
    // Box-Muller turns two uniform values into a deterministic normal deviate.
    float first = max(1.0e-7, pixel_random(pixel, 0u));
    float second = pixel_random(pixel, 1u);
    return sqrt(-2.0 * log(first)) * cos(6.283185307179586 * second);
}

void main() {
    vec2 source_uv = vertex_uv;
    // Radial distortion is evaluated as an inverse lookup: each output pixel
    // asks which source pixel should be sampled. This avoids holes in output.
    if (distortion_params.x != 0.0 || distortion_params.y != 0.0) {
        vec2 centered = vertex_uv * 2.0 - 1.0;
        float radius_squared = dot(centered, centered);
        float factor = 1.0 + distortion_params.x * radius_squared +
                       distortion_params.y * radius_squared * radius_squared;
        source_uv = centered * factor * 0.5 + 0.5;
    }

    uvec2 pixel = uvec2(gl_FragCoord.xy);
    if (source_uv.x < 0.0 || source_uv.x >= 1.0 || source_uv.y < 0.0 || source_uv.y >= 1.0 ||
        pixel_random(pixel, 2u) < distortion_params.z) {
        fragment_color = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    // Keep the GPU operation order aligned with the CPU reference path:
    // exposure/gain, additive noise, quantization, and final clamping.
    vec4 source = texture(source_image, source_uv);
    vec3 color = source.rgb;
    float scale = exp2(color_params.x) * color_params.y;
    color = color * scale + pixel_gaussian(pixel) * color_params.z;
    if (color_params.w > 0.0)
        color = round(color / color_params.w) * color_params.w;
    fragment_color = vec4(clamp(color, 0.0, 1.0), source.a);
}
