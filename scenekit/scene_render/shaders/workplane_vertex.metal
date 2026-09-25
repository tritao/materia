#include <metal_stdlib>
using namespace metal;
struct Input { float2 position [[attribute(0)]]; };
struct Output { float4 position [[position]]; float2 grid_ndc [[user(locn0)]]; };
vertex Output main0(Input input [[stage_in]]) {
    Output output;
    output.position = float4(input.position, 0.0, 1.0);
    output.grid_ndc = input.position;
    return output;
}
