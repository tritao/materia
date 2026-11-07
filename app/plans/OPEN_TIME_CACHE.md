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
- **OC1. Code identity. DONE** (haxeon d463bd509). haxeon emits module-closure fingerprints with the build and a runtime lookup; tests: editing a module
  changes its own and its dependants' fingerprints and nothing else; identical across clean and incremental builds.
- **OC1b. Native identity. DONE.** `CodeIdentity.nativeBuild(package)` hashes the libraries published beside the program; OCCT is
  covered by `CADKIT_OCCT_DIR` and `MATERIA_MEMO_SALT` (its libraries live outside the program directory).
- **OC5. Shape serialization.** `cad_shape_write/read` with names and attributes; round-trip tests: names, volume, face and
  edge counts, tessellation byte-identical, a boolean on a restored shape equals one on the original.
- **OC3. Component-boundary cache.** Castings, hob mesh, anchor windows, sizing layouts, bounds per component; scene parts
  per `geometryKey`. Measure the hobber cold open after (a) an edit to `FlexsplineHobber.hx`, (b) an edit to
  `MineralCasting.hx`, (c) no edit; scene output byte-identical to an uncached build.
- **OC6. Verify gate.** Run the whole test suite with `MATERIA_MEMO_VERIFY=1` (and a cold store) once; fix every missing
  dependency it finds.
- **OC4. Docs.** Clearing and inspecting the cache, verify mode, the rule for adding a persistent memo.

Target: (a) about the compile plus 20 s; (b) the casting rebuilt, everything else cached; (c) unchanged.

## Using the cache

Two layers share one store (`cadkit.cache.DiskStore`, files named by the SHA-256 of the full key):

- **Kernel cache** (`cadkit.cache.OpCache`): pure CadKit work keyed by construction history (`Shape.key`): fuse, cut and common,
  loft, and the queries mass properties, face descriptors and tessellation. Correct by construction: a key names the inputs
  of a pure operation, the CadKit/OCCT build and the cadkit code. Haxe source edits never enter these keys.
- **Component memos** (`machinekit.cache.Memo.persist`): masses, scene parts, anchor derivations, casting solids, the hob mesh.
  Keyed on the input key plus the code identity of the code that computes them (`CodeIdentity`) and the native build.

**Environment** (all optional)

| Variable | Effect |
|---|---|
| `MATERIA_MEMO_DIR` | Cache directory. Default `$XDG_CACHE_HOME/materia/memo`, else `~/.cache/materia/memo`. |
| `MATERIA_MEMO=off` | No persistent tier. Results are identical, only slower. |
| `MATERIA_MEMO_MAX_MB` | Size cap, default 2048. Least recently used entries are evicted past it. |
| `MATERIA_MEMO_VERIFY=1` | Recompute every hit and fail on any difference (see below). |
| `MATERIA_MEMO_SALT` | Extra text in the native identity. Set it after rebuilding OCCT or CadKit in a way the library hashes and OCCT version cannot see (same sizes, same version text). |

**Clear and inspect.** Clear: delete the cache directory (it is only a cache). Inspect: `HOBBER_PHASES=1` on the hobber scene
(`machinekit/examples/flexspline-hobber/haxeon.enclosure.json`) prints the build/fit/export times, one `MEMO` line per memo
(hits, misses, how many came from disk, time spent computing) and `OPS` lines for the kernel cache (keyed and unkeyed
booleans, history booleans that cannot be cached and why, queries).

**Verify mode.** `MATERIA_MEMO_VERIFY=1` makes every disk hit recompute and compare bytes; a difference throws
`persistent memo "<name>" is stale ...`. It means the key or the identity missed an input. To find missing dependencies across
commits: fill the cache on one commit, switch to a commit that changed code, run with verify on. It is slow by design: use it as
a gate, not in everyday runs.

**Adding a persistent memo**
1. The key must name every input of the value: a component's `geometryKey` (type, exact inputs, material, detail) is the model.
   Never build keys from object identity or anything that varies between runs; a null key means "not cached".
2. If the computing code is virtual (a component's `geometry`, `massProperties`), add `CodeIdentity.ofObject(instance)` to the
   key: the memo's own identity covers only the module that owns it.
3. Give `persist` the module whose code computes the value (`CodeIdentity.of("pkg.Module")`) and exact codecs
   (`Codecs`): decode(encode(x)) must equal x to the bit.
4. Solids use `Memos.persistentParts`: the value is always the canonical form (`Shape.canonical`), on a miss, a hit and with the
   tier off, so results never depend on the cache state.
5. Test it the way `PersistentMemoTests` does: miss, hit, another identity, damaged entry, verify mode refusing a planted
   stale value.

**Adding a keyed operation** (kernel cache). Give the result a key (`ShapeKey`) built from the operation name, exact-bit
parameters and the input keys. An operation that cannot name its inputs returns a null key, which makes everything built on it
uncached (never wrong). Shapes carrying face attributes, and operations that keep their history (`Part.combine`, `subtract`), are
not cached today: see the follow-ups.

**Known limits.** Cold runs are not faster (a shared, read-only cache would be the answer). Code identity is the import closure of
a module, which in this codebase is nearly the whole program for generic modules (`AssemblyPreview`, `MachineComponent`), so an
edit to a widely imported module invalidates the component memos; the kernel cache is what makes recomputing them cheap.
Tessellation meshes an independent copy, so it never changes its input or its `bounds()`.

## Log

- 2026-11-07 OC2 done. Hob display mesh (default hob, debug build): 153 s computed, 0.09 s from the persistent tier in a new
  process, digest identical; a different parameter set is a separate entry. The mesh entry is 38 MB.
- 2026-11-07 OC1/OC1b done. Identity per module = source (64-bit) + imports' identities (cycles condensed) + compiler + defines,
  written as one generated literal per compile (<= 8 ms for 1500 modules). Persistent memos are on by default; a module's
  identity changes with its own or a dependency's edit (HobSolid edit changed HobMesh/HobSolid/Hob, which form an import cycle,
  and not Memo). Hob mesh with real identities: 142 s miss, 0.08 s hit in a new process.
- 2026-11-07 OC5 follow-up. Root cause of the non-idempotent shape bytes: OCCT's `BinTools_LocationSet::Read` rebuilds each
  location with `gp_Trsf::SetValues` (determinant normalisation, then Gram-Schmidt `Orthogonalize`) and surface placements
  through `gp_Ax3` (Y recomputed from N x X), so a read moves last bits and flips signed zeros; round-off makes it converge
  over a few passes (a fully chamfered box takes five), not one. Fix: `cad_shape_write_bytes` writes the fixed point (repeat
  write-read until the B-rep bytes stop changing), so `fromBytes(b).toBytes() == b` exactly and `Shape.canonical()` is
  `fromBytes(toBytes())`. A shape built from scratch can still differ from its canonical form in signed zeros, in the last
  bits of rotated placements, and in where a rotated cylinder's triangulation starts, so **a persistent shape memo must return
  the canonical form on a miss as well as on a hit, and also when the tier is off.** The OC5 tessellation change (-0.0 written
  as +0.0) is removed: it changed every mesh (4569 of 1.3 M values in the height-12 hob mesh) to hide the reader's signed
  zeros; the default hob digest is back to `27fd345a...`.
- 2026-11-07 Landed on local main (materia 69ccdb1db, haxeon 691ac35f; unpushed). Stages: OC2 persistent `Memo`, OC1/OC1b code and
  native identity, OC1c `CodeIdentity.ofObject` + OCCT version in the native identity, OC5 shape bytes (fixed point,
  `Shape.canonical`), component-level caches (masses, scene parts, anchor derivations, castings, hob mesh), then the kernel
  cache: stage 1 `Shape.key` + persistent fuse/cut/common, 2 spline/loft keys + `Part.combineKept`, 3 mass properties and face
  descriptors, 4 tessellation made pure (meshes an independent copy) and cached, 5 lofts cached (`Part.loftKept`).
  Hobber scene, including compile: uncached 234 s; cold 199 s; warm 27 s; edit to FlexsplineHobber.hx ~56 s; edit to
  MineralCasting.hx 103 s (was 194 s before the kernel cache; machine loaded by other sessions, timings +-20 %). Artifact digest
  `0fab9107c808d6b2` in every run, cache on or off. Verify gate (MATERIA_MEMO_VERIFY=1 on the populated cache, same code and after a
  MineralCasting edit): 4259 booleans, 333 mass, 145 face-descriptor and 198 tessellation queries recomputed and equal, no stale.
  Profile of the edit run: loft 26 % (now cached), history-keeping fuse on shapes with face attributes 15 %, `describeFaces` 11 %,
  JSON key building in InsertAnchorGeometry.derive ~13 %, `HobMesh.fromShape` array conversion < 1 %.
- Follow-ups (none scheduled): (1) key face attributes (threads, details) so attributed shapes and `Part.combine`/`subtract` history
  booleans (~17 s in the edit run) are cached; (2) a cheaper derivation key than `Json.stringify` of the rule and pose;
  (3) one mechanism: let the kernel cache carry geometry and shrink the identity-keyed memos to non-geometric data; the import closure
  of generic modules is nearly the whole program (`MachineAssemblyCodec` falls back to the full catalogue, `PrecisionErrors` imports
  `MachineAssembly`, `import machinekit.component.*` pulls the registry): cutting it was tried and abandoned (stash "catalog package
  move"); (4) a test that fails when keyed boolean coverage drops on a reference model; (5) a shared read-only cache for cold runs;
  (6) lazy shape restoration for deep chains of cached operations; (7) the display-mesh array conversion in `HobMesh`.
- 2026-11-08 Stage 6: face-attributed shapes get construction keys (key = old key + hash of the exact `FaceAttributeCodec` bytes; `ShapeKey.op`
  no longer refuses attributed inputs; `Part.joined` no longer falls back to the uncached history path), so the history-fuse path and the
  describe-faces/mass/tessellate queries on threaded fasteners and feet are cached. Landed on local main 0bd86d874 (unpushed). Hobber scene,
  including compile (load avg ~6): cache off 185 s, cold 185 s, warm 28 s, edit to MineralCasting.hx 83 s (103 s at stage 5, 194 s before the
  kernel cache). The edit run now has 5608 keyed boolean hits, 0 misses and 0 unkeyed booleans; still uncached: `Part.combine`/`subtract`
  history booleans (557, 6.9 s). Remaining edit-run memo time: anchor derivations 13.8 s, scene component parts 9.9 s, masses 7.2 s,
  hob mesh 2.8 s. The artifact digest is now `ef53215727e1edf8` in every run, cache on or off: the stage-5 baseline `0fab9107...` was stale
  because it predates merging main; with the stage-5 cadkit on the merged tree the cache-off digest is `987472b3...`, and the 10 parts that
  differ from it differ only in normals (threaded fasteners and feet, whose booleans now return the canonical form). The earlier "JSON key
  building ~13 %" was a double count: it is 7.6 % over four callers (`AssemblyDefinitionFlattener.flatten`, `BeltPoseContext`,
  `InsertHoleCodec.snapshot`, `solvedPoses`), no single fix.
- 2026-11-08 Stage 7 (landed, materia 79ccb6672 + haxeon 6c910e58): JSON was the largest non-cache cost after stage 6. Two causes: (1)
  `haxe.format.JsonPrinter.enter` scanned every active ancestor with `Dynamic ==` on each array/object (slow in HashLink: `$enum-dynamic-equal`,
  66 % of `JsonWireWriter.getString`; it affects every `JsonWire.encode`), now an identity-keyed `ObjectMap` with the same cycle semantics;
  (2) `BeltPoseContext.new` made a full JSON copy of the machine's assembly definition on every `DriveSystem.derive`/`solvedPoses`, now it
  copies only what it changes (couplings, actuators, encoders, elasticNetworks, joint limits/stop; `BeltPoseTests` keeps the old constructor as
  the oracle, 45 checks). Matched sampler profile of the MineralCasting-edit run: phases build 31/fit 10/export 26 (67 s) -> 19/6/17 (42 s),
  samples 22185 -> 14608. Artifact digest unchanged (`ef53215727e1edf8`) in all four runs; wall-clock matrix at load 10-16 is not comparable
  (cache off 208 s, cold 229 s, warm 33 s, edit 119 s). Left: `InsertHoleCodec.snapshot` 5.8 %, the flattener's JsonWire decode 3.3 %.
  NEGATIVE RESULT, do not retry: keying `anchor-derivations` by the body solid's `Shape.key` cannot make it hit after a casting code edit, because
  the memo's own identity is `InsertAnchorGeometry`'s import closure (1184 modules, whole program) and MineralCasting is reached by five
  routes (InsertAnchorGeometry -> MineralFoundation typedef; CastInsert -> registry; HobberChipCover -> HobberFoundationSizing; RadialPinionAssembly ->
  CastingVolumeAgreement; RackMesh -> RadialPinionAssembly). Only narrowing that identity (module lists / annotations, rejected) or the import
  refactor (abandoned) would change it; the same holds for the sibling anchor memos.

