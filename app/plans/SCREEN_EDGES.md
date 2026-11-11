# Screen-space CAD edges

Status: dropped 2026-11-11 and removed. Lines found from a one-sample-per-pixel buffer only know an edge to the pixel:
a least-squares fit through the staircase recovers shallow and diagonal lines, but a near-vertical or near-horizontal
line steps once per many pixels, wider than any local window, and showed hooks at each step (0.39 px deviation at
slope 15). They also drew every visible crease at full weight, so sub-pixel detail (a rail's grooves) cluttered where
strokes fade it. Strokes now take their visibility from the same buffer (`STROKE_VISIBILITY.md`), which fixes the
bellows line with exact, antialiased geometry. Kept for the findings below.

## Why

CAD feature edges are drawn as depth-tested line ribbons. Every edge defect so
far comes from deciding a line's visibility by comparing depths at pixel
resolution: lines fighting their own faces at hole rims (tie bias, support
strips), sub-pixel steps flickering (`EDGE_SIGNIFICANCE.md`), and hidden edges
showing through thin oblique walls (`STROKE_VISIBILITY.md`; the hobber's Z
bellows).

Drawing edges from what is visible removes the class: a line goes where
neighbouring pixels show different faces that actually turn. Hidden edges
cannot be drawn, nothing fights a face, and a face thinner than a pixel never
makes a line.

## Rule

From a per-pixel buffer of the visible surface (instance id and world
position):

- different instance (or instance against background): silhouette or contact
  outline, a line;
- same instance: a line only where the surface creases (normals more than
  20 degrees apart) or occludes itself (a gap along the normal of more than two
  pixel footprints). Face ids are not used: for most meshes the pick pass's
  subelement is a triangle, so coplanar triangles would draw their diagonals.
  Normals are rebuilt from world positions, each axis stepping to the nearer
  neighbour so a step does not straddle a crease.

CAD feature strokes are then not drawn for opaque surfaces. Section contours,
hover/selection strokes and other overlays stay geometric.

### Transparent surfaces keep strokes (a rule, not a gap)

The visible-surface buffer holds one surface per pixel, but through a
transparent part every layer's edges should show: an enclosure's near panel,
its far panel seen through it, and the opaque machine inside. One buffer can
outline only the nearest transparent layer; peeling layers costs a pass per
layer. The stroke failures that motivate screen edges mostly do not arise
there: transparent faces write no depth, so their edges neither fight their
own faces nor leak through their own walls, and only opaque geometry should
hide them, which a depth-tested stroke does.

So the material decides, in one place: opaque surfaces draw edges from the
buffer; transparent surfaces draw strokes depth-tested against opaque depth.
Both use the same width, colour rule and antialiasing, so a line does not show
which system drew it. Once opaque parts no longer use strokes, the
opaque-only stroke machinery (support strips, tie bias, step fading) can go.

## Steps

| Step | What | Check |
|---|---|---|
| SE1 | Prototype (done): a full-screen edge pass inside the main pass, reading the AO prepass's ids and positions; opaque CAD feature strokes off when on. | Bellows replay; scene render tests pass. |
| SE2 | Compare on real views with the capture tool: hobber bellows, cutting close-up, hole rims; side by side with strokes. | User judges line quality. |
| SE3 | (done) Antialiasing and line weight matching strokes (half width 0.85 px, soft edge); colour rule per surface, shared with strokes (`material_takes_light_edges`). | Side-by-side crops of the bellows and detail views. |
| SE4 | (done) `nkscene_render_view.screen_edges` replaces the environment variable; editor setting and command; only in the Feature edge mode (All shows every mesh edge as strokes, Shaded none); `DepthCapture --screen-edges`. GPU test `screen_edges_regression`: 0.2 mm oblique wall over a stroked edge, 0.05 mm band (no lines), 10 mm chamfer (both creases), dark block on a light plate (outline colours), 1 and 4 samples. | Tests; live comparison in the editor. |
| SE5 | If kept: default on; remove support strips, tie bias and step fading for opaque edges; web build. Writing ids in the main pass only matters for views without AO. | Frame time vs strokes; web smoke. |

## Findings

- No extra geometry pass in the editor: every viewport look enables ambient
  occlusion, whose prepass already renders the pick ids and world positions of
  opaque surfaces each frame. The edge pass is one full-screen triangle inside
  the main pass. Transparent parts are not in that buffer and keep strokes.
- Hobber bellows view (`app/tests/haxeon.depth-capture.json`): the line across
  each skin is gone; outlines, holes, rails and fins are kept; no triangle
  diagonals or speckle. Lines are thinner, pixel-stepped and lighter than the
  antialiased strokes: SE3 needs antialiasing and a line weight to match.
- SE3: a pixel's coverage falls off with its distance to the nearest boundary
  in its 3 x 3 neighbourhood (each boundary midway between the two pixels that
  differ), with the strokes' 0.85 px half width and a one-pixel soft edge. On
  the bellows and detail views the weight now matches the strokes and the
  lines are smooth. Left: small dots at some band corners, and the colour rule
  (light lines on dark materials) is not yet applied.
- Correction to the SE3 note above: fading by the distance to the nearest
  pixel-snapped boundary softens but keeps the staircase (a shallow line's
  centre wandered 0.29 px; strokes 0.11 px). The pass now reads a 7 x 7 window,
  finds boundaries in the inner 5 x 5, and fits a least-squares line through
  the midpoints of the same line (the same two parts, or one part's crease):
  locally first, then with every point near that fit. Coverage falls off with
  the distance to the fitted line; where the points are not collinear (corners,
  tight curves) the nearest midpoint's distance stands, and the nearest midpoint
  limits overshoot past a line's end. At junctions the nearest two lines are
  fitted separately. Fitted distances match the true edge to 0.085 px RMS;
  a shallow line's centre now wanders 0.15 px (`screen_edge_antialiasing_regression`),
  and on the hobber slanted lines look like strokes.
- Colour: the AO prepass writes each surface pixel's edge tone in its
  position's w (1 for a light material, 2 for a dark one, from the rule strokes
  use), and a boundary takes the tone of its nearer surface, so a part owns its
  outline: on the hobber the dark column's outlines are light, and the orange
  bellows in front of it keep dark lines.
- The band-corner dots are the band lines meeting the corner line: the
  side-face band lines sit a little lower than the front ones in perspective,
  and the stroke render shows the same junctions. Real geometry.
- Surface normals step to the nearer neighbour. On an oblique face only one or
  two pixels wide, a step across the crease onto a face-on neighbour can be the
  nearer one, which moves that crease line by up to a pixel toward the oblique
  face; choosing the flatter continuation (second differences) fixes that but
  needs a wider window. Not worth it so far.
- Fixed, not about edges: captures after a scene rebuild (an edge-mode switch,
  for one) rendered metal parts black. The live paint applied the camera pose,
  grid and studio look to the view after prepareRenderView, so frame and depth
  captures only had them while a live frame's view was retained; any rebuild
  dropped the studio environment, which metals live on. prepareRenderView now
  configures the whole view for every path (`ViewportSelectionTests`).
