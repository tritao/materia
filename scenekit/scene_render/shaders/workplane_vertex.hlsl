struct Input { float2 position : POSITION0; };
struct Output { float4 position : SV_Position; float2 grid_ndc : TEXCOORD0; };
Output main(Input input) {
    Output output;
    output.position = float4(input.position, 0.0f, 1.0f);
    output.grid_ndc = input.position;
    return output;
}
