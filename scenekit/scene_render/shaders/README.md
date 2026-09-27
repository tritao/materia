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
