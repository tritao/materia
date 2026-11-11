# Screen-space CAD edges

Status: SE1 prototype and SE3 antialiasing/line weight done 2026-11-11 (`NK_SCENE_SCREEN_EDGES=1`, default off); SE3 colour rule, SE4 next.

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
| SE1 | Prototype (done): a full-screen edge pass inside the main pass, reading the AO prepass's ids and positions; `NK_SCENE_SCREEN_EDGES=1`; opaque CAD feature strokes off when on. | Bellows replay; scene render tests pass. Still to add: a GPU test (box against a box behind it, a 0.2 mm oblique wall over an edge: no line, a 0.05 mm raised band: no line, a chamfer: lines). |
| SE2 | Compare on real views with the capture tool: hobber bellows, cutting close-up, hole rims; side by side with strokes. | User judges line quality. |
| SE3 | Antialiasing and line weight matching strokes (half width 0.85 px, soft edge); colour rule per surface like `stroke_color_for`. | Side-by-side crops of the bellows and detail views. |
| SE4 | Editor toggle instead of the environment variable; a GPU test (oblique thin wall: no line; 0.05 mm band: no line; chamfer: lines; part in front of a part). | Tests; live comparison in the editor. |
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
