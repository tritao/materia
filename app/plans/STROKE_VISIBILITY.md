# Stroke visibility: hidden edges through thin walls

Status: SV0, SV1 (reused), SV3 and SV4 done 2026-11-11 as the part rule below; SV5 (web) open. Screen-space edges
(`SCREEN_EDGES.md`) were tried in between and dropped: they cannot antialias near-axis lines and draw sub-pixel detail
at full weight.

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

The studio AO prepass already renders, every editor frame, each pixel's part
(pick id) and world position: the visible-surface buffer. The stroke fragment
shader reads it at the pixel of its point (the centre-line point for a ribbon,
the face point for a support strip):

- background, or the stroke's own part: no change; the depth test decides, as
  before (it hides a part's own back edges);
- another part: rebuild that surface's tangent plane from the neighbouring
  positions of the same part, orient it toward the camera, and carry it to the
  exact point; the stroke is hidden when the point lies behind the plane by
  more than 2 % of a pixel footprint.

Instance ids replace the per-face ids of the original design: hiding by another
part is what the depth test cannot do, and a part's own geometry stays with
the depth test. A touching part (a bolt head on a plate) lies on the plane and
stays visible; a wall of another part in front of the edge, however thin,
hides it. Left: a part's own edge under its own sub-pixel wall, as before.
Section contours keep the depth test alone. Without studio AO (no prepass)
strokes fall back to the depth test.

## Steps

| Step | What | Check |
|---|---|---|
| SV0 | (done) `stroke_visibility_regression`: an edge 0.2 mm under another part's oblique wall at the editor's projection, 1x and 4x samples; a block's contact edges and outline stay drawn, its own back edge hidden. `stroke_outline_regression`: a part's own outline stays whole at slopes 1/7 to 15. | The depth test alone leaks 91 / 132 pixels; with the buffer 0. |
| SV1 | (done) No new pass: the studio AO prepass is the buffer (part id, world position). | Pick and AO tests unchanged. |
| SV2 | (done, smaller) Stroke vertices read the instance's pick colour; no face ids. | Stroke tests. |
| SV3 | (done) The stroke fragment shader applies the part rule on top of the depth test. | All SceneKit tests pass. |
| SV4 | (done) The hobber bellows replay shows no line across the skins; corners and band lines unchanged. Prepass cost: none added (it already ran for AO). | `frame.png` crops. |
| SV5 | Web build: R32F/RGBA16F targets and texture sampling in the stroke pass on WebGL2. | Web smoke. |
