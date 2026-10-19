# Scene renderer shaders

`scene`, `stroke`, `workplane`, `pick`, and `postprocess` use their GLSL vertex and fragment files as
the canonical shader source. `tools/generate-shaders.py` feeds these stages to
`sokol-shdc` and writes the backend sources to the checked-in
`src/scene_shader_sources.h`. Regular builds consume that header without
requiring shader tools.

Regenerate after editing a canonical shader:

```sh
SOKOL_SHDC=/path/to/sokol-shdc ./tools/generate-shaders.sh
SOKOL_SHDC=/path/to/sokol-shdc ./tools/generate-shaders.sh --check
```

Run these commands from `scenekit/scene_render`. Generation also requires
`glslangValidator` on `PATH`. The current generated header was made with
`sokol-shdc` from `floooh/sokol-tools-bin` commit
`11d0cf678105d614d675e6d9bd2aaf3eeff12f8c`.

The tool emits GLSL 4.10, GLSL ES 3.00, HLSL 5, and macOS Metal directly.
The generator validates both OpenGL variants with `glslangValidator`. Desktop
OpenGL requires a 4.1 context. Picking stores face IDs in an expanded vertex
buffer so the same shader works on every backend.

Stroke geometry uses a body quad and two cap quads. The body endpoints keep
those of the original projected segment, preserving its depth slope. Caps
extend depth only away from the eye, so their round coverage cannot pull a
hidden edge through an occluding face. Keep depth interpolation in rasterization
for per-sample MSAA depth tests; a fragment-depth override evaluates at the
fragment center and can leak under closely spaced sloping surfaces. The stroke
pipeline uses only a constant depth-unit bias to resolve coplanar ties.

Ambient occlusion samples world-space surface positions from an on-demand RGBA32F
prepass target. Its alpha channel marks valid samples. This avoids reconstructing
positions from nonlinear device depth, which loses precision with very small near
planes and can stripe an otherwise flat surface. Picking continues to write R32F
device depth for its readback and unprojection.
