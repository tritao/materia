# Edge significance: sub-pixel steps

Status: E1-E3 done 2026-11-10; E4 showed the bellows line is not a step (see Findings).

## Problem

Feature-edge strokes are drawn at full weight whatever the size of the shape
they outline. A step much smaller than a pixel still draws a line, and that
line flickers with zoom: the two edges of the step land in one pixel and the
concave one wins or loses its depth test by the stroke tie budget, which is of
the same size as the step.

Found on the flexspline hobber's Z bellows (`BELLOWS-PU-LOW-8x10`): each skin's
outside slide lip stands `gap` = 0.05 mm proud of the skin. At 0.63 m a pixel
is 0.7 mm, so the step is 1/14 px, yet a line crosses every skin and comes and
goes with zoom. Depth precision is not the cause (worst step 0.055 px); the
line disappears with strokes off (`app/tests/haxeon.depth-capture.json`,
`frame.png` against `frame-shaded.png`).

## Rule

A feature edge between two faces that turn (a box edge, a chamfer's edges) is
a corner and always draws: it outlines the silhouette whatever its size.

A feature edge is a step when, walking from it straight across the face beside
it, the first boundary that runs along the edge belongs to a face parallel to
the edge's other face and facing the same way. Its significance is the step
height `h`: the offset between those parallel planes. The test is local on
purpose: a thin ring (a tube's end) reaches a parallel face on its far side,
but its first boundary across is its own inner face, so a tube end is a corner. Its stroke coverage is scaled by `clamp(h / p, 0, 1)`, where
`p` is the world size of one pixel at the stroke's depth. A step of a pixel or
more draws fully; a 1/14 px step draws at 1/14, which reads as nothing, and
does so steadily instead of flickering.

Coverage is continuous in `h / p`, so there is no threshold to tune and no
popping while zooming. Strokes that are not CAD feature edges (section
contours, analytic overlays) carry no step and are unaffected.

## Steps

| Step | What | Check |
|---|---|---|
| E1 | `MeshFeatureEdges` reports a step height per segment (0 = corner): for each B-rep face beside the edge that is planar and narrow across it, the face beyond it parallel to the edge's other face gives `h` as the offset between those planes. | Unit cases: plain box (all corners), chamfered box (chamfer edges are corners), plate with a 0.05 mm raised band (both band edges `h = 0.05`), stepped block with a 10 mm step (`h = 10`). |
| E2 | Stroke segments carry the step (`nkscene_stroke_segment.step`, 0 for none) from `GeometryData` through the scene payload to the stroke vertex buffer. | SceneKit C API and payload tests; existing stroke tests unchanged. |
| E3 | The stroke shader scales coverage by `h / p`, with `p` from the stroke's clip `w`, the viewport width and the view-projection's horizontal scale. | GPU test: a plate with a 0.05 mm band draws no band strokes at a pixel of 1 mm and full strokes at 0.01 mm; a box edge draws fully at both. |
| E4 | Verify on the bellows replay: no line across the skins at the reported view, lines at a close zoom; record the view in `app/tests/depth-precision/`. | `frame.png` crops before and after. |

## Findings

- E1 first used the whole narrow face's extent, which rejected ring-shaped
  step faces (a collar's end is one ring), then only plane offsets, which made
  a U-channel's end ring a 19 mm "step" between one leg's outer face and the
  other leg's inner face. The local walk fixes both (`EdgeStepSmoke`: box,
  chamfer, raised band, 10 mm block, collared tube, nested channels).
- The bellows line across each skin is not a step edge. Fading every edge the
  analysis calls a step leaves it; fading the corners removes it. The visible
  band boundaries are 0.25 mm steps between two separate parts (parent and
  child skin), which a per-part analysis cannot see, so they stay corners and
  draw fully, as a bellows should look.
