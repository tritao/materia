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

## CC2. One native identity, written by the build

**Problem.** Native code is identified in four ways: the CadKit library's content hash, the OCCT version text plus the names and
sizes of its libraries, `MATERIA_*_SALT`, and the record's size and mtime stamps for toolchain binaries. The salt exists because
none of them is complete.

**Change.**
- haxeon already fingerprints native build actions (`build/execution/ActionFingerprint.hx`). When it publishes a library, it
  writes that fingerprint next to the library. `.haxeon-published` currently holds only the library name.
- `NativeIdentity` reads the identity file next to each library the process actually loaded, found through `/proc/self/maps` as
  now.
- OCCT's identity is the hash in its prebuilt install, from a stamp file in the install. The directory name is not used.
- The record uses the same identities for the native libraries and the toolchain binaries. Content is hashed only for a binary
  that no build of ours produces.
- Delete the salt, the size and mtime stamps, the `nativeBuild` fallback, and the OCCT names-and-sizes list.

**Gates.**
- Fixture: rebuilding CadKit with a source change changes the identity. A rebuild with no change keeps it.
- A missing identity file prints the "kernel cache off" line, and the regression test from the NativeIdentity fix still passes.

## CC3. A module resolves to exactly one file

**Problem.** A new file in an earlier source root can shadow a module. The record lists directory contents for that reason, and
the incremental-compile shadowing bug (`hasShadowedModule`) came from the same cause.

**Change.**
- First, check whether anything shadows a module on purpose, apart from haxeon's stdlib overriding Haxe's std. If something does,
  stop and report it.
- Otherwise the compiler rejects a module name that exists in two roots. The stdlib override set is fixed and covered by the
  compiler's own identity, so it stays allowed.
- A new file then matters only once an import names it, and the importing file is already in the record. The record keeps file
  contents only.
- Delete the directory listings from the record and `hasShadowedModule`.

**Gates.**
- Compiler test: a duplicate module across roots is an error that names both files.
- `test-fingerprint.sh` still passes, with its shadowing case changed to expect that error.

## CC4. Verify as a standing gate

**Problem.** Verify mode only catches a missing dependency when someone runs it.

**Change.** Add one small reference model to the normal suite. It runs cold, then warm with verify on. It also checks keyed
boolean coverage: zero unkeyed booleans, and a hit count no lower than the recorded baseline.

**Gate.** The test fails when a keyed operation is made impure on purpose, and when a boolean goes unkeyed.

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
