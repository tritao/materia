# Open-time cache

**Goal.** Opening a generated project after a code change costs about the compile plus assembling the machine from cached
parts, not a full rebuild of every solid. Status: plan, 2026-11-07.

## Measured (flexspline hobber, cold open after a source change, 2026-11-07)

About 240 s, with the HashLink sampler (`hl --diagnostics PORT` + `hlprof-live`, report with `scripts/hlprof-report.py`):

| Phase | Time | Dominated by |
|---|---|---|
| Compile (haxeon) | ~36 s | |
| Machine build (`new FlexsplineHobber`) | ~29 s | foundation sizing building casting solids |
| Enclosure fit and assembly | ~38 s | casting solids built for bounding boxes |
| Scene export (`runtimeScene`) | ~134 s | the hob's detailed display mesh (~53 %, `HobMesh`/`HobSolid`: loft, fuse, gash cuts), castings again (~20 %), face descriptors (~14 %) |

`d5b770ed6` memoizes casting solids within one run (export 134 → 88 s). Everything else is rebuilt on every cold open,
because today's scene cache is all-or-nothing on the source fingerprint: one edit to `FlexsplineHobber.hx` rebuilds every
solid even though none changed.

## Design

Every build-time memo (`machinekit.cache.Memo`) can opt into a **persistent tier**, keyed on its inputs and on the identity
of the code that computes it.

- **Code identity from haxeon.** The build records, for each module, a fingerprint of its transitive dependency closure (its
  source, every module it imports, the compiler version), and exposes it at runtime (for example `CodeIdentity.of(Class)`).
  Editing a module or anything it depends on changes the fingerprint; editing an unrelated module does not. Module closures,
  not per-function hashing: simpler, and geometry code changes rarely.
- **Persistent tier in `Memo`.** Opt-in per memo. Key: memo name, input key, the owning module's code identity, the native
  build identity (CadKit/OCCT). Values in a content-addressed store under `$XDG_CACHE_HOME/materia/memo`, with a size cap and
  least-recently-used eviction. Shapes stored as OCCT BREP, plain data as JsonWire. A verify mode (environment variable)
  recomputes on every hit and fails on a difference, so a missing dependency shows up as an error, never as a stale shape.
- **Turn it on where it pays:** casting solids, the hob mesh, anchor windows, sizing layouts, and the tessellated scene
  part from `AssemblyPreview.part` (mesh, mass properties, face descriptors) keyed on the part's `geometryKey` and the
  tessellation settings.

## Steps

- **OC1. Code identity.** haxeon emits module closure fingerprints with the build and a runtime lookup; tests: editing a
  module changes its own and its dependants' fingerprints and nothing else; deterministic across clean and incremental
  builds.
- **OC2. Persistent memo tier.** Store, keys, size cap and eviction, BREP and JsonWire values, verify mode; tests for
  hit/miss, invalidation by code identity and by native build, eviction, verify-mode failure on a planted stale entry,
  concurrent writers.
- **OC3. Turn it on.** Castings, hob mesh, anchor windows, sizing layouts, scene parts. Measure the hobber cold open after
  (a) an edit to `FlexsplineHobber.hx`, (b) an edit to `MineralCasting.hx`, (c) no edit; record before/after in the Log.
  The scene output must be byte-identical to an uncached build (or the difference explained).
- **OC4. Docs.** How to clear and inspect the cache, the verify mode, and the rule for adding a persistent memo.

Target: (a) about the compile plus 20 s; (b) the casting rebuilt, everything else cached; (c) unchanged.

## Log
