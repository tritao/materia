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

Surface sidedness follows `MaterialFlags::DoubleSided`. Other materials reject back
faces in the shaded, picking and AO position shaders; view-aware CPU face picking
uses the same rule. Winding is explicitly counterclockwise, and a flat per-instance
transform parity corrects reflected instances without splitting their batches.
Double-sided sheets retain both sides and orient their shading normals towards the
visible side. Selection and hover tint the source material and retain its sidedness.
Section planes clip these passes consistently. Declared closed, single-sided,
opaque meshes also receive filled cuts. Unknown/open boundaries, double-sided
sheets, blended/transparent materials and alpha masks receive no caps.

For each plane and cut occurrence, the executor bounds the work by the projected
world bounds. A scissored fullscreen draw zeros stencil without touching colour
or depth. The occurrence is then drawn with only that plane enabled, both faces,
no colour/depth writes and an always-pass depth comparison: **occluded crossings
must still invert stencil**. Finally a unit quad transformed onto the plane draws
where parity is odd, clipped by every other plane. Stencil is independent for each
occurrence, including instances sharing a batch. It is per sample in an MSAA
attachment. The normal shaded pipeline is restored before transparency and edges.

Cap quads use ordinary rasterized depth, never a fragment-depth override. Their
normal faces the eye, and their darkened material colour has a plane-space hatch
at 3 mm spacing in world metres, anchored independently of the camera. Source UV
textures and normal maps do not apply to the synthetic cut. AO runs the same cap
passes into its world-position target, so both depth and obscurance see the plane.
The unit quad is static; plane/bounds transforms and scissoring use the ordinary
renderer data. There is no CAD intersection, mesh retriangulation or stencil
readback while dragging a section.

Picking executes the same parity and quad passes. The subelement attachment's
alpha byte encodes `plane + 1` for caps (ordinary faces use 255), preserving all
24 bits of ordinary primitive IDs. Readback returns `NKS_RENDER_CAP_SUBELEMENT_TAG`
plus the zero-based plane index. A tagged cap has no B-rep face or CAD edge id.
Haxe callers use `PickResult.capPlane()`; its negative subelement tag is excluded
from face/edge hover and mate picking. View-aware CPU ray picking performs the
same kept-boundary parity query and returns the same tag.

Closure is an explicit `GeometryPayload.closed` / `GeometryData.setClosed`
declaration, default false. Native boxes declare it by construction. CadKit solid
producers declare it only when all tessellated faces belong to solids; open
supplier connector faces stay undeclared. Portable scene artifact version 43
carries this per-part flag. Non-CAD producers can use `MeshClosure.declare`, which
checks welded boundary edges, winding against normals and positive enclosed
volume. The socket fixture generator runs that proof on its actual output meshes.
The presentation schema is unchanged.

`scene_render_sections` includes named pixel regressions
`TwoPlaneCornerBothSurfaceHitsClipped` and `CavityInFrontOfBackSurface`, as well as
one-plane caps, hollow cores, overlapping instances, mirrored transforms,
transparent/double-sided/open exclusions, plane-depth picks, AO, and hidden edges
and silhouette coverage with MSAA. Exact B-rep drawing sections remain separate.
