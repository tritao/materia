# Cache cleanup

What is left after the open-time cache work (`OPEN_TIME_CACHE.md`, stages 1–9). The structure is settled. There is one
`DiskStore`, one dependency record written by the compile, and the kernel cache keyed by content (`Shape.key`). These steps
fix how each piece knows what it depends on, so that no key relies on a hand-kept list, a salt or a file timestamp.

Rules for every step:
- One mechanism. When a step replaces something, delete the old path: no fallback, no compat.
- Gates are small fixtures. The hobber is used only for the timing and digest check at the end of a step.
- Commit on local main (haxeon first). Do not push.

## CC1. Each output records the files the run read

**Problem.** The record covers what the compile read, not what the generator reads while it runs. Supplier STEP files and other
data reach the key only through `cache.inputs` in `materia.project.json`. The hobber declares `"cache": {}`, so a changed
supplier file is served from a stale `.mtrg`.

**Change.**
- Route every file a generator reads through one function. It reads the bytes and notes the path with its digest.
- The generator writes that list next to its output, and the runner stores it with the `.mtrg` entry. The output then has its own
  dependency file.
- On lookup, the runner rehashes the listed files. If one changed, it regenerates without compiling.
- Delete `cache.inputs` and the code that reads it.

**Gates.**
- Fixture: editing a data file the generator reads makes the next open regenerate. An unchanged reopen still hits.
- A grep check finds no direct `sys.io.File` reads left in generator code paths, or the step lists each remaining one and says
  why it is excluded.
- Hobber: an unchanged reopen still takes well under a second. Touching one supplier STEP file regenerates the scene without a
  compile.

## CC2. One native identity: the content of what is loaded. DONE

**Problem.** Native code was identified in four ways: the CadKit library's content hash, the OCCT version text plus the names and
sizes of its libraries, `MATERIA_CACHE_SALT`, and the record's size and mtime stamps for toolchain binaries. The salt existed
because none of them was complete.

**What was built.** Content, everywhere, from one place.
- `NativeIdentity` is the content hash of the CadKit library the loader mapped (`CodeIdentity.mappedLibraries`, from
  `/proc/self/maps`), the `occt.identity` stamp of the OCCT install the mapped `libTK*` came from (next to its `lib` directory), and
  the naming-scheme version. A missing CadKit library, OCCT stamp or OCCT library prints `kernel cache off: no native identity
  (<what is missing>)` and turns the persistent tiers off.
- `cadkit/scripts/build-occt-prefix.sh` writes `occt.identity` (a hash of the installed `libTK*` content) after installing, and
  stamps an existing install that has none. The stamp of the prebuilt install was written once, without rebuilding.
- The record (version 3) holds content digests, not size and mtime, for the native libraries and the toolchain binaries (Haxe,
  HashLink, the runtime library), and the tool's identity hashes the Haxe compiler by content. Touching a file or rebuilding it
  to the same bytes keeps the fingerprint; a change of one byte of the same size changes it.
- Deleted: `MATERIA_CACHE_SALT`, `CodeIdentity.nativeBuild`, `CodeIdentity.loadedLibraries` (names and sizes), the stamp columns.

**Why not the build-written identity this section first described.** Three facts in the code. (1) Haxeon's `native-cmake-publish`
action sees only its own inputs; the fingerprint is computed by the executor over the action graph, so writing it next to the
library means exposing the dependency chain's fingerprint to the publish script, a change to the executor. (2) The OCCT prebuilt
install is outside the build graph; nothing in a build can say what it contains, so it needs a stamp from the script that makes it
either way. (3) The Haxe compiler, `hl` and `haxeon_runtime.hdll` come from the bootstrap scripts, not from the build graph. Content
hashing the loaded files is complete by construction (it cannot miss a rebuild whatever tool made it), needs no coordination
between haxeon and Materia, and costs one read of the files.

**Gates (passed).**
- Fixture (`nativeIdentityTracksBuild` in the project-loading test): an identical copy of the CadKit library elsewhere keeps the
  identity; the same size with one byte changed changes it; a copy of the OCCT install with the same stamp keeps it, another stamp
  changes it, no stamp turns the kernel cache off with the message. `test-fingerprint.sh` (17 checks) covers the record: other
  content, same size other content, restored content, new mtime with the same content.
- The NativeIdentity regression test (the generator fails without an identity, the store must hold kernel entries) still passes.

## CC3. A module resolves to exactly one file. DONE

**Check first.** Nothing shadows a module on purpose. A build of the hobber program (3358 modules, every source root) with a
report of every module found in a second root printed nothing. Haxeon's stdlib overrides Haxe's std through the compiler's
own lookup, not through a second source root, so no exception was needed.

**What was built.**
- `ModuleSourceLoader.load` looks in every root and throws `Module "X" is in two source roots: A and B` when two different
  files declare it. A file whose package declaration names another module still does not count (the existing rule).
- The record (version 5) no longer lists source directories; `sourceListingDigest`, `listSourceDirectories`, the runner's check of
  them, `Compiler.hasShadowedModule` and the `CompilerSession` reset it caused are deleted. A new file matters once an import names
  it, and the importing file is in the record.

**Known limit.** A persistent compiler session that already loaded module `X` does not look at the roots again, so a new
`X` in another root is noticed by the next clean compile (which rejects it), not by an incremental one. The old reset hid
this by recompiling; the project is invalid in that state either way, and a cached output of the valid project is not served
for it by a clean build.

**Also found.** `InputFiles` shared its lists between threads (the hobber generator reads files from `Parallel` workers); one
cold open in about six died with `assert` inside `InputFiles.wrote`. The lists are now guarded by a mutex (only when recording).

**Gates (passed).** `test-fingerprint.sh` (18 checks, now with a clean compile that rejects a module in two roots and accepts
the project once the copy is removed); `CompilerInputsMain` (fresh session rejects the second copy); the project-loading test
(a new module nothing imports does not recompile); hobber digest `89fab112da0b8210`, warm 0.2-0.3 s, verify clean.

## CC4. Verify as a standing gate. DONE

**Problem.** Verify mode only catches a missing dependency when someone runs it.

**What was built.** `machinekit/tests/hobbing/src/CacheGateTests.hx` (manifest `haxeon.cache-gate.json`, workspace project
`cache-gate`, tag `cadkit`) builds a small hob (`HobSolid.build`: ten keyed booleans, gashes fused into a bored cylinder) three
times into a fresh store: cold, warm, warm with verify on. It checks:
- zero unkeyed booleans and zero unkeyed history booleans on the cold run (a boolean that loses its key fails the gate);
- the warm run has no misses and at least `BASELINE_HITS` (10) hits, and the same bytes as the cold run;
- the verified run recomputes every hit (`verified >= hits`) with the same bytes;
- two controls: an operation made impure on purpose is caught by verify, and a boolean on an unkeyed shape is counted.
When the model's operations change on purpose, set `BASELINE_HITS` to the warm hit count the test prints.

**Cost.** About 2 s of operations (cold 0.9 s, warm 0.3 s, verify 1.0 s) plus the compile of the test project. The first model
(module 1, 40 teeth, two starts, default tolerances) took 27 s; coarser surface and edge tolerances kept the same ten booleans.

**Gate (passed).** Marking the input of the model's final cut unkeyed in `HobSolid` made the test fail on the history-boolean
check; restored, it passes. The stale `machinekit/tests/haxeon.hob-probe.json` (its entry was deleted with the persistent tier) is
removed.

## CC5. Edit-loop Haxe work (performance, after CC1–CC4)

After an edit, most of the hobber's roughly 32 s scene run is plain Haxe work, not geometry:
- anchor derivations (10 s), where about a third of the parallel samples wait on the GC lock;
- `InsertHoleCodec.snapshot` copying through JSON;
- per-candidate allocation in the pattern search.

Do one profile and fix what it shows, with a limit of 1.5 h. The digest (`89fab112da0b8210`) must not change.

## Not now

- Typed keys for the in-memory memos, which are hand-built and often `Json.stringify`.
- Narrower code identity for kernel queries. Lists and annotations were rejected.
- Upstream work on OCCT's unstable serialization (the canonical-form passes).
- A shared read-only cache.
- Native loaders that open a path themselves, which `haxeon.build.InputFiles` cannot see (it notes reads made through `sys.io.File` and
  `sys.FileSystem`; CadKit's STEP import reads the file in Haxe first, so it is covered): animkit `ak_asset_load_file`, robotkit
  `rk_policy_create`, `rk_inference_create`/`_checked`/`_model_sha256` and `rk_recording_reader_open`, and the nativekit UI font, image
  and audio loaders (`nkui_font_collection_add`, `nkui_image_load_file`, `nk_audio_clip_create_from_file`). None is used by a CAD
  generator today. A generator that uses one must read the file in Haxe first (for its digest) or the output can be stale.
