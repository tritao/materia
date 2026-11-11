# Stroke visibility: hidden edges through thin walls

Status: superseded 2026-11-11 by `SCREEN_EDGES.md` (edges drawn from the visible-surface buffer cannot leak). Kept for the mechanism and its measurements.

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

## Rule

A stroke fragment asks what covers the stroke's centre line at the nearest
centre-line point:

- one of the edge's own faces (same drawn instance, one of its incident face
  ids): visible;
- background: visible;
- any other surface: extend that surface's tangent plane (position and normal
  from the G-buffer pixel) to the exact centre-line point along its view ray;
  the edge is hidden when that plane is nearer by more than a few depth steps.

A touching part (a bolt head on a plate) ties at the contact line and stays
visible; a wall in front of the edge, however thin, hides it.

## Steps

| Step | What | Check |
|---|---|---|
| SV0 | The thin-wall reproduction becomes a SceneKit GPU test: oblique and straight views, walls 0.05-20 mm, 1x and 4x samples, plus a visible edge on a grazing face and a contact edge (block on plate) that must stay drawn. | Fails today on the thin oblique walls only. |
| SV1 | A single-sample G-buffer prepass before the main pass, from the pick pass: instance id, face id, world position, world normal. Only when the frame has strokes. | Pick and AO tests unchanged; G-buffer readback test. |
| SV2 | Stroke vertices carry their instance id and the edge's incident face ids (from the stroke supports); the ribbon passes the centre-line point and its window depth unperspective-interpolated. | Payload and compiler tests. |
| SV3 | The stroke fragment shader applies the rule; the hardware depth test for surface-edge strokes is replaced by it. Section contours keep their own path. | SV0 passes; existing stroke, section and contour tests pass. |
| SV4 | The hobber bellows replay shows no line across the skins; record the view in `app/tests/depth-precision/`. Measure the prepass cost on the hobber. | `frame.png` crops; frame time before and after. |
| SV5 | Web build: R32F/RGBA16F targets and texture sampling in the stroke pass on WebGL2. | Web smoke. |
