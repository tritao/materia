# Stroke visibility: hidden edges through thin walls

Status: done 2026-11-11 for strokes and section contours (SV0-SV4 and the cleanup below); SV5 (web) statically checked,
not yet run in a browser.

## Problem

A CAD edge hidden under a thin wall is drawn through it when the wall is seen
obliquely. Found on the flexspline hobber's Z bellows: each loose skin's collar
edge lies 0.2 mm under the next skin's wall and shows as a line across every
skin (`app/plans/EDGE_SIGNIFICANCE.md`, Findings).

Mechanism, reproduced in a SceneKit GPU test at the editor's projection
(0.63 m, 1167 x 828): a stroke is a screen-space ribbon about two pixels wide
whose every fragment is depth-tested at the centre line's depth. Across those
pixels an oblique wall's depth changes by about 1 mm per pixel, so on one side
of the ribbon the wall lies behind the hidden edge and the ribbon passes.

| View | Wall | Pixels leaking (1x / 4x MSAA) |
|---|---|---|
| Oblique | 0.2 mm | 93 / 134 |
| Oblique | 0.5 mm | 8 / 45 |
| Oblique | 1 mm and more | 0 |
| Straight on | 0.05 mm | 0 |

Depth precision is not involved: 0.2 mm is about 14 depth steps there.

## Why not a plain depth test

Testing the centre line against a scene depth texture still needs a tolerance:
an edge lies exactly on its own faces and pixel centres are never exactly on
it. A tolerance from surface slope (about one pixel of depth) is larger than
the 0.2 mm wall, so the leak returns; classic slope-scaled polygon offset fails
the same way. What separates a visible edge from a hidden one is which surface
covers it, not how far away that surface is.

## Rule (as built)

Every frame that draws lines (strokes, or section contours when the view has
clip planes) renders the visible-surface buffer: each pixel's part (pick id)
and world position, opaque surfaces and section caps. Ambient occlusion reads
the same buffer. Lines have no depth test; the line shader decides each point
on the edge once, from the buffer, and every fragment of the ribbon carries
that point (centroid-interpolated, so a multisampled border pixel cannot
extrapolate past the edge's end). A line is drawn whole or not at all.

- background: visible;
- the line's own part, at the point's pixel: the point lies on that surface up
  to the pixel centre's offset, so it is hidden when it lies deeper (along its
  view ray) than the surface's depth change over one pixel, measured from the
  neighbouring pixels of that part (the larger difference on each axis, so it
  holds across a crease). This hides a part's back edges and the inner edges of
  its holes at any viewing angle; a tolerance from the pixel footprint does
  not, since at grazing angles a pixel spans a long stretch of surface;
- other parts, at the four pixel centres around the point: each surface's
  tangent plane (rebuilt from neighbouring positions, oriented to the camera)
  is carried to the exact point, and the point is hidden when any lies in front
  by more than float noise (0.5 % of a pixel footprint). Depth comparison would
  need a tolerance larger than a thin wall over a hidden edge. Four pixels,
  because two surfaces can cross inside one (a sheet just in front of the seam
  where two parts meet). A touching part passes through the point and leaves
  it visible.

Section contours lie on their own caps and follow the same rule; edges clipped
at a section plane are cut back by the ribbon footprint plus a hundredth of a
pixel's world size (for edges parallel to the cut).

Limits, kept on purpose: a part's own edge under its own wall thinner than
about a pixel can show (tighter own-part tests break silhouettes on grazing
and curved faces); near another part's silhouette a hidden line can end up to
a pixel early.

Removed with the depth test: support strips, the tie-break bias, face
sidedness, contour expansion within the cutting plane, the cap depth ramp and
a separate contour pipeline.

Found on the way: section caps were missing from the pick/visibility pass
beside touching parts. nkgpu passed NKGPU_COLORMASK_NONE to sokol as 0, which
sokol reads as "all channels", and its pipeline-wide mask covered only the
first target, so stencil-only cap steps overwrote the subelement and position
targets (also a picking bug). Fixed in NativeKit (color_mask_smoke) and pinned
by section_caps' touching-facets test.

## Steps

| Step | What | Check |
|---|---|---|
| SV0 | (done) `stroke_visibility_regression`: an edge 0.2 mm under another part's oblique wall at the editor's projection, 1x and 4x samples; a block's contact edges and outline stay drawn, its own back edge hidden. `stroke_outline_regression`: a part's own outline stays whole at slopes 1/7 to 15. | The depth test alone leaks 91 / 132 pixels; with the buffer 0. |
| SV1 | (done) No new pass: the studio AO prepass is the buffer (part id, world position). | Pick and AO tests unchanged. |
| SV2 | (done, smaller) Stroke vertices read the instance's pick colour; no face ids. | Stroke tests. |
| SV3 | (done) With the buffer, the stroke shader decides each edge point from it and the depth test is off for strokes (own part: depth along the view ray within one pixel's depth change; other parts: tangent plane at the exact point); support strips are skipped. | All SceneKit tests pass; a grazing slab's silhouette and fold carry the same line per row (2.64 / 2.59). |
| SV4 | (done) The hobber bellows replay shows no line across the skins; corners and band lines unchanged. Prepass cost: none added (it already ran for AO). | `frame.png` crops. |
| SV5 | Web build: the buffer is RGBA8 ids + RGBA32F positions (the web host enables EXT_color_buffer_float via enableExtensionsByDefault), nearest sampling of float textures and centroid varyings, all WebGL2; the GLSL ES 3.0 shaders compile in the generator. Still to run: `app/web/test.sh`. | Web tour. |
