# SceneKit follow-ups

These are separate engineering tasks from the node vocabulary cleanup:

- Evaluate the sparse component stores against a consolidated node table using
  representative scene sizes and mutation patterns.
- Measure snapshot page copying, lazy materialization, and the derived lookup
  indexes before changing their ownership or publication model.
- Split the largest scene and render translation units where it improves build
  or review time.
- Decide whether SceneKit needs a public C++ `Scene`/`Transaction` façade. The
  current public C++ API exposes IDs and immutable `SceneSnapshot` data; scene
  mutation is currently available through the C ABI and Haxe binding.
- Add reversed-Z with floating-point depth to improve CAD depth precision across
  large distance ranges. Preserve stencil support for section caps, and update
  projection matrices, depth comparisons/clears, stroke bias, picking and depth
  reconstruction together. Define backend capability checks and a conventional
  depth fallback, including the OpenGL 4.1 baseline and GLES/WebGL targets.
  Measure depth/stencil memory and MSAA performance; validate visible/hidden
  edges, section caps/contours and picking at close zoom and large scene scales.
  Keep the frustum-aware camera clipping fit introduced for the bench-mill mold
  edge breakup; reversed-Z complements that correction.
