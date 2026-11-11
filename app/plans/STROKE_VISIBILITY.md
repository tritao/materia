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
(pick id) and world position: the visible-surface buffer. When it is present,
strokes skip the depth test and the stroke fragment shader decides visibility
from it, once per point on the edge: every fragment of the ribbon carries the
same point (the centre-line point), so a line is drawn whole or not at all.
At that point's pixel:

- background: visible;
- the stroke's own part: the point lies on that surface up to the pixel
  centre's offset, so it is hidden when it lies deeper (along its view ray)
  than the surface's depth change over one pixel, measured from the
  neighbouring pixels of that part (the larger difference on each axis, so it
  holds across a crease). This hides a part's back edges and the inner edges of
  its holes at any viewing angle; a tolerance from the pixel footprint does
  not, since at grazing angles a pixel spans a long stretch of surface;
- another part: comparing depths would need that same tolerance, larger than a
  thin wall over a hidden edge, so the part's tangent plane (rebuilt from
  neighbouring positions, oriented to the camera) is carried to the exact
  point; hidden beyond 2 % of a pixel footprint behind it.

The depth test decided each fragment at the centre line's depth separately,
so half of a silhouette ribbon lying over its own grazing face could fail it:
silhouettes drew thin and broken (the hobber's carriage plate). Support strips,
which only won depth ties, are skipped in this mode. Without studio AO (no
prepass) strokes keep the depth test. Section contours keep the depth test.
Left: a part's own edge under its own sub-pixel wall.

## Steps

| Step | What | Check |
|---|---|---|
| SV0 | (done) `stroke_visibility_regression`: an edge 0.2 mm under another part's oblique wall at the editor's projection, 1x and 4x samples; a block's contact edges and outline stay drawn, its own back edge hidden. `stroke_outline_regression`: a part's own outline stays whole at slopes 1/7 to 15. | The depth test alone leaks 91 / 132 pixels; with the buffer 0. |
| SV1 | (done) No new pass: the studio AO prepass is the buffer (part id, world position). | Pick and AO tests unchanged. |
| SV2 | (done, smaller) Stroke vertices read the instance's pick colour; no face ids. | Stroke tests. |
| SV3 | (done) With the buffer, the stroke shader decides each edge point from it and the depth test is off for strokes (own part: depth along the view ray within one pixel's depth change; other parts: tangent plane at the exact point); support strips are skipped. | All SceneKit tests pass; a grazing slab's silhouette and fold carry the same line per row (2.64 / 2.59). |
| SV4 | (done) The hobber bellows replay shows no line across the skins; corners and band lines unchanged. Prepass cost: none added (it already ran for AO). | `frame.png` crops. |
| SV5 | Web build: R32F/RGBA16F targets and texture sampling in the stroke pass on WebGL2. | Web smoke. |
