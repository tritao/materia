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

A **content-addressed cache of pure derived values**: a value is a function of its inputs and of the code and native build that
computed it, and the key says exactly that, so a hit equals a recomputation.

- **Code identity from haxeon.** The build records, per module, a fingerprint of its transitive dependency closure (source,
  imports, compiler version) and exposes it at runtime (`CodeIdentity.of`). Module closures, not per-function hashing; the
  interface stays coarse enough to refine later. Clean and incremental builds must agree.
- **Native identity from the build.** haxeon already fingerprints native actions; the CadKit and OCCT build hash is exposed at
  runtime and is part of every key (replaces the `naming` version + `MATERIA_MEMO_SALT` stand-in).
- **Persistent tier in `Memo`** (`DiskStore`, done in OC2). Opt-in per memo; key = memo name, code identity, native identity,
  input key; files under `$XDG_CACHE_HOME/materia/memo`, size cap, LRU. Verify mode (`MATERIA_MEMO_VERIFY=1`) recomputes every
  hit and fails on a difference.
- **Shapes are serializable.** CadKit gains `cad_shape_write/read` (OCCT binary BREP plus the naming and surface attributes),
  so solids are cacheable as solids and downstream booleans too. No bounding-box-only workarounds.
- **The cache boundary is the component.** `ComponentType.key(values)` is already a canonical exact-input key. Geometry per
  detail, display mesh, physical properties, bounds and anchor windows are cached per `(type, key(values), detail)`; the
  hand-keyed memos that this subsumes are deleted. In-process `Memo` stays as the fast tier in front.
- **Typed keys** from `@:wire` codecs where they exist, so a forgotten input cannot happen.
- **Scene export per part**, keyed by `geometryKey` and tessellation settings, so opening is a diff against cached parts.

Until OC1 lands there is no trustworthy code identity: `CodeIdentity.of` returns null (tier off) unless
`MATERIA_MEMO_STANDIN` is set, which is for measuring and tests only.

## Steps

- **OC2. Persistent memo tier. DONE.** `DiskStore`, `Memo.persist`, `Codecs`, verify mode; `PersistentMemoTests`
  (54 checks); first user `HobMesh.build`.
- **OC1. Code identity.** haxeon emits module-closure fingerprints with the build and a runtime lookup; tests: editing a module
  changes its own and its dependants' fingerprints and nothing else; identical across clean and incremental builds.
- **OC1b. Native identity.** CadKit/OCCT build hash at runtime, into every key.
- **OC5. Shape serialization.** `cad_shape_write/read` with names and attributes; round-trip tests: names, volume, face and
  edge counts, tessellation byte-identical, a boolean on a restored shape equals one on the original.
- **OC3. Component-boundary cache.** Castings, hob mesh, anchor windows, sizing layouts, bounds per component; scene parts
  per `geometryKey`. Measure the hobber cold open after (a) an edit to `FlexsplineHobber.hx`, (b) an edit to
  `MineralCasting.hx`, (c) no edit; scene output byte-identical to an uncached build.
- **OC6. Verify gate.** Run the whole test suite with `MATERIA_MEMO_VERIFY=1` (and a cold store) once; fix every missing
  dependency it finds.
- **OC4. Docs.** Clearing and inspecting the cache, verify mode, the rule for adding a persistent memo.

Target: (a) about the compile plus 20 s; (b) the casting rebuilt, everything else cached; (c) unchanged.

## Log

- 2026-11-07 OC2 done. Hob display mesh (default hob, debug build): 153 s computed, 0.09 s from the persistent tier in a new
  process, digest identical; a different parameter set is a separate entry. The mesh entry is 38 MB.
