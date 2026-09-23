# SceneKit follow-ups

These are separate engineering tasks from the node vocabulary cleanup:

- Evaluate the sparse component stores against a consolidated node table using
  representative scene sizes and mutation patterns.
- Measure snapshot page copying, lazy materialization, and the derived lookup
  indexes before changing their ownership or publication model.
- Revisit resource-delta history retention and trimming under long-lived
  renderers.
- Split the largest scene and render translation units where it improves build
  or review time.
- Decide whether SceneKit needs a public C++ `Scene`/`Transaction` façade. The
  current public C++ API exposes IDs and immutable `SceneSnapshot` data; scene
  mutation is currently available through the C ABI and Haxe binding.
