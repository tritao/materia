# Editor compile time regression (2026-11-06)

**Symptom.** Launching the editor after any change takes minutes. A cold `app/haxeon.json` compile log shows
`compiler: 104152 ms, 29683 functions retyped` (2026-11-05). Under load, 100–230 s.

**Baseline.** On 2026-10-04 a cold app build with a warm ccache took **33 s**: the import re-parse fix, AliasTable, LambdaName,
the inliner memo and the compile/native overlap. See the commits on haxeon main around c8cf3e6c and materia 85aa9a874. Warm edit
was about 6–10 s, no-op 0.9 s.

**How to measure** (October method):
- Time `./haxeon/scripts/haxeon build --project app/haxeon.json` with a fresh `HAXEON_COMPILER_SESSION_DIR`, which gives a cold
  compile. Read the `compiler phases (ms)` line and the driver allocation phases in the log.
- Then make a one-line edit (a comment in a uikit or app file) and rebuild, which gives a warm edit.
- Then a no-op rebuild.
- Compare against an October haxeon and materia pair if needed. The machine is shared and loaded, so measure in the same window
  and report CPU as well as wall time.

**Steps.**
1. Measure cold, warm-edit and no-op on today's main. Split by phase: graph/parse, typing, canonicalization, IR, inliner, verify,
   encode, GC.
2. Find the regression. Bisect haxeon (and the app's source growth) between the October baseline and today. Separate code growth
   from compiler slowdowns: for example the recent equality and enum work (`Compare class instances by identity`, `give values
   equality`, `plan inline enum payload storage`, `Carry enum payload references through typed lanes`, typed lanes), retyping
   counts, GC pressure (`compile-gc-ms`), or the session/cache invalidation that made "29683 functions retyped" happen.
3. Fix the root causes in haxeon (or the build graph) cleanly, keeping output deterministic: a cold build must still be
   byte-identical to a warm one (see the bytecode determinism notes and `hldump`).
4. Also check whether haxeon pin bumps needlessly invalidate project and compile caches. A compiler-version change must invalidate,
   but say what is cached by content.

**Acceptance.** Cold app compile back near the October 33 s (or the best achievable, with the remaining cost explained). Warm edit
and no-op measured. The haxeon test suite and the app tests pass. Numbers before and after are in the commits.

## Round 1 results (2026-11-06)

**Validated.** Ten fixes are committed locally in haxeon, ending at `57f7c9bc`.
The latest editor cycle measures **92.013 s cold / 35.315 s comment edit / 1.829 s no-op**, with
**107.41 / 40.00 / 0 s resident-worker CPU**. Cold and warm artifacts are byte-identical for both original
and edited sources, including every `hldump` section. The complete latest compiler suite passed **788/788**; the latest app suite passed, exit 0.
**The October 33 s target is not reached.** Remaining costs and measurement limits are recorded below.
No push, merge or Materia submodule pin change has been made.

Work is confined to Materia `compile-speed` (starting `1de8ec1f6`) and its separate haxeon clone
(starting `e7d8716e`). The four pre-existing machinekit edits and pcbkit checkout are outside this task.
History was fetched read-only from the shared clone. Historical worktrees at `haxeon/out/compile-speed-history`
and `haxeon/out/compile-speed-array-bisect` use their exact vendored HashLink/NativeKit pins and rebuilt
release runtimes. The current matching release runtime was rebuilt. OCCT was not rebuilt.

### Editor measurements

Raw logs, artifact copies, source digests, phase traces, probes and the measurement harness are in
`/media/joao/DATA1/claude-hobber-tools/tmp/compile-speed/`. Samples use heavy-run, a fresh private session for
cold builds, disabled shared artifact restoration, and deletion of the local output to defeat action fingerprints.
Queue time is excluded. Worker user+system CPU is read separately from the CLI/child process CPU.

| Compiler/workload | Cold command / compiler / worker CPU | Comment command / compiler / worker CPU | No-op command |
| --- | --- | --- | --- |
| October pair: haxeon `c8cf3e6c`, Materia `85aa9a874` | 31.896 / 26.185 / 34.59 s | not remeasured | not remeasured |
| Original current, initial valid cycle (`before.*`) | 143.452 / 131.287 / 177.84 s | 127.957 / 125.052 / 155.04 s | 4.614 s |
| Original current, comparison batch (`paired-before.*`) | 120.865 / 111.225 / 152.95 s | 77.569 / 75.470 / 99.63 s | 2.318 s |
| Original cold repeat (`paired-before-repeat.*`) | 113.966 / 105.840 / 142.73 s | not repeated | not repeated |
| Nine fixes (`paired-nine.*`) | 106.179 / 95.861 / 125.33 s | 43.802 / 41.582 / 50.03 s | 2.032 s |
| Ten fixes (`paired-ten.*`) | 92.013 / 83.468 / 107.41 s | 35.315 / 33.543 / 40.00 s | 1.829 s |

The nine-fix cycle still failed edited-source determinism; only the ten-fix cycle passes the complete gate.
The ten-fix restored warm command is **40.249 s / 46.47 s CPU**; its independent edited cold is
**98.325 s / 116.52 s CPU**, also with identical bytes. The original comment build incorrectly retyped zero
functions; the final compiler refreshes **33** callers whose embedded defaults refer to the edited source.

Shared load varies: comparison original cold load **5.00–6.75**, nine-fix cold **4.61–13.38**, ten-fix cold
**3.01–2.94**; original warm **6.75–5.88**, ten-fix warm **2.94–3.24**. These are successive samples in the
comparison batch, not isolated-machine latency claims. CPU and causal allocation probes support the improvement
independently of wall time. The final original cold repeat brackets the fixed cycle: **113.966 command / 105.840 compiler / 142.73 CPU s**,
load **4.74–3.82**, compared with fixed **92.013 / 83.468 / 107.41 s**. This is **19.3% less command wall time**,
**21.1% less compiler wall time**, **24.7% less worker CPU**. Warm command/worker CPU falls **54.5% / 59.9%**,
with the load qualification above.
A final cold rebuild from the actual committed source (`final-stock.*`, no source override) measured
**95.159 s command / 112.67 s worker CPU**, load 14.03–6.96, and restored the editor output with exactly the
validated original-source artifact hash. This restoration is not substituted for the comparison batch.
Latest CLI/child CPU adds **9.095 s cold**, **2.821 s warm**, **2.766 s no-op** to worker CPU. Original comparison
CLI/child CPU is **9.421 / 3.192 / 3.065 s**.

Phase wall times in seconds (original cold uses the final repeat; original comment uses the comparison triplet):

| Phase | Original cold | Fixed cold | Original comment | Fixed comment |
| --- | ---: | ---: | ---: | ---: |
| Frontend | 27.777 | 28.164 | 2.194 | 1.862 |
| Typing/lowering | 23.603 | 23.441 | 1.808 | 1.741 |
| IR assembly | 35.444 | 18.147 | 47.252 | 14.238 |
| ABI | 0.130 | 0.204 | 0.106 | 0.189 |
| Incremental backend | 15.645 | 11.072 | 20.050 | 12.414 |
| Output backend | 2.342 | 1.536 | 2.991 | 2.226 |

Backend cold subphases: verify **1.991 s**, function lowering **5.528 s**, debug **2.756 s**; warm:
verify **1.474 s**, functions **6.665 s**, debug **2.877 s**. Fixed graph/restart work is **25.047 s**
of frontend time. Driver compile allocation falls **64.654 -> 41.274 GB**; final warm allocation is **14.335 GB**.
Cold compile GC wall time falls **13.044 s -> 8.660 s**. Intermediate array-only IR allocation
falls **25.101 -> 13.728 GB**, then **9.739 GB** after dense non-null facts; latest IR allocation is about **9.27 GB**.
Backend function allocation falls **11.037 -> 3.439 GB**.

Earlier unmatched-load measurements are retained as evidence, not used as latency comparisons: array-only cold
**210.836 command / 192.528 compiler / 242.37 CPU s** under load 17.84–22.11; dense-facts cold
**128.309 / 117.331 / 145.45 s** under load 20.78–16.71. Five-fix cold was **165.712 / 154.373 / 192.11 s**.
Seven fixes completed **107.937 / 98.774 / 129.46 s** cold and **39.262 / 37.344 / 47.65 s** warm but still
had caller-dependent primitive-helper metadata. These results motivated the final correctness fixes.

### Regression localization and workload growth

Two algorithmic regressions were localized to their introducing commits, with historical reproduction:

- **`7a2a53e8` (Nov 2), struct-array lowering.** Every element shape revisits/copies unrelated functions.
  The file is absent in parent `a44605bb`. A matching-runtime build at the introduction reproduces **451.34 MB**
  for 16 shapes / 250 unrelated functions. Current original: **468.42 MB / 0.29 s CPU**; indexed fix:
  **8.63 MB / 0.01 s CPU**. This regression predates the latest enum-payload changes.
- **`c72c2da9` (Nov 5), non-null must-analysis.** Each CFG block copies hash-map facts for every SSA value,
  and phi edge queries copy sets again. The 256-block/32-scalars probe falls **217.53 -> 3.49 MB**,
  **0.19 -> <0.01 s CPU**, preserving every queried fact and emitted HL bytes.

Representation passes also recreate immutable IR bodies, losing the inliner's identity-based memo even when
no function is retyped. Exact input-state comparison restores that identity without serialized buffer retention.
Optional field storage appeared in `c7704d86`/`b41b017f` (Oct 31); nullable narrow-field storage in `c389bd27`
(Nov 2). Both copied unaffected functions. The combined editor phase allocated **1.018 GB / 4.10 s CPU** before
the selective field rewrite fix. Equality and enum semantics were kept intact.

The October workload was reconstructed at `build/compile-speed-october`, with NativeKit `88cb8f888`.
Explicit Haxe inputs grew **1,703 files / 7,051,066 bytes -> 3,328 / 16,840,894 bytes** (**2.39x bytes**).
Cold retyping grew **15,705 -> 29,993 functions**. `hldump` shows artifact growth **34,822,286 -> 98,671,899 bytes**,
functions **16,201 -> 33,662**, types **15,624 -> 35,652**, globals **14,536 -> 40,503**; source-span metadata
**15.807 -> 54.941 MB**, snapshots **8.061 -> 17.552 MB**.

The current-compiler/unchanged-October-workload plan resolves all inputs, but compilation fails at
`app/CncProgramPlayer.hx:412:37`, **E1025: enum `motionkit.robot.SessionState` does not conform to Equatable**.
The failed 12.760 s command is not a performance sample. Migrating that workload would change the comparison,
so no exact numerical partition between language evolution and source growth is claimed. The two historical
algorithmic probes above isolate compiler regressions without such a migration.

### Local haxeon fix commits

| Commit | Change and validation |
| --- | --- |
| `ac360445` | Index immutable function types for array lowering; skip irrelevant shape passes and helper copies. Six scaling cases and mixed nested/nullable/iterator/dynamic cases preserve canonical IR on HL, Wasm32 and Wasm GC. Registered allocation bound fails before and passes after. |
| `b2e050d2` | Dense non-null bitsets, sparse SSA-ID compaction, allocation-free phi edge queries. Six cases preserve facts and HL bytes. Registered 128-block probe: 50.58 -> 1.12 MB, below 8 MiB. |
| `c2e5db8c` | Refresh static initializer/default-argument syntax while retaining validated types, including static-only classes. Original fails comment edits; all initial 30 source-edit cases pass. |
| `d57fb5dd` | Preserve pristine IR input identity across representation rewrites. Original memo probe: 299.17 -> 13.99 MB, 0.28 -> 0.02 CPU s. Tests retain callee/object/settings invalidation, transaction isolation, placement and pruning. |
| `1362b563` | Anchor nominal enum/value equality helpers at current declarations while retaining caller origins for dependencies. Retire prior cached owners when fresh generation moves helpers. Initial 16 cross-module edit cases pass; original fails the first. |
| `75a03949` | Preserve all validated function attributes during body refresh. Partial reconstruction dropped `inline`, changing emitted code on comment edits. Expanded 36 source-edit cases pass. |
| `33f56fb4` | Compare complete immutable IR state without serialized copies. Memo probe: 13.99 -> 5.88 MB, 0.02 -> 0.01 CPU s; strengthened 8 MiB bound fails the serialized version. 253 checks compare with full encoding across every instruction, attributes, provenance, signed zero and NaN payloads. Unknown classes conservatively compare unequal. |
| `57a2b4da` | Preflight optional/nullable field storage and share unchanged instructions. Four 400-function/100-instruction cases preserve canonical IR and verifier results: 18.45 -> 0.117 MB, below 1 MiB. Invalid optional copy and nullable width diagnostics remain checked. |
| `5fc568ec` | Explicit generated provenance for built-in equality helpers, which have no source declaration. Eliminates three caller-dependent helper identity rows exposed by the seven-fix editor cycle. Expanded 24 enum/struct/nullable-field edit cases pass; real user snapshot hashing remains unchanged. |
| `57f7c9bc` | Track exact immutable source snapshots consumed by omitted defaults. Refresh foreign callers, including lambdas/generic origins, when source text changes; preserve transaction isolation and prune retired entries. Fixes the last two stale span hashes in the editor. Expanded 54 source-edit cases pass; original fails the first cross-module case. |

Each fix has its own commit, measured before/after evidence and the required Codex trailer. All 27 changed Haxe
files pass the pinned formatter. The only Materia changes from this task are this plan and its documentation commits;
the haxeon/pcbkit gitlinks are not staged or committed.

### Determinism and memory gates

Final source input digests:

- Original/restored: `1972fc377ff72a18bad11a994e88351a6b601d885e3a1a9400997f964b22a0f1`.
- Comment-edited, cold and warm: `00452bdecbb44b08b308fef821e43164659156ef2053462da5e8a76883ee6e76`.

Final artifact SHA-256:

- Original cold and resident warm restore: `c07cd7d52dc33195df1e3d91a5a88e0cc2d0d3976a851f809c9d7e2993af53c1`.
- Edited resident warm and independent cold: `1031d39a07cc9587b97a4575e670c1fe263ca937b5ee98c2f6c177b59191d73e`.

`paired-ten-determinism.json` passes all five checks. `paired-ten-hldump.json` passes all eight comparisons:
sections, executable hashes, identities and spans for each pair. These are complete bytes, not just code hashes. Original versus fixed cold executable function hashes also all match.
The 54 source revision cases plus 24 equality cases exercise metadata and emitted code on targeted source edits.

Several earlier cycles were invalidated by machine-wide earlyoom. Examples: worker `1870602` at 13:47:09
(12,961 MiB RSS), `2128313` at 15:23:52 (8,458 MiB), `2172130` at 15:41:46 (6,918 MiB), original comparison
restore worker `2260170` at 16:13:37 (14,819 MiB). The cgroup recorded zero OOM kills. Fallback/restarted workers
are rejected as warm measurements. Final samples waited inside heavy-run for **18 GiB MemAvailable on two samples**;
that wait is excluded. The harness restores Main.hx and shuts down only its own private workers through authenticated
RPC, verifying the exact session path against `/proc`. No other session was signalled.

### Validation, cache audit and remaining work

- Initial two fixes: complete **783/783 passed** (`full-driver-tests.log`). First five: complete **786/786 passed**
  (`full-current-tests.log`) and focused **36/36** (`focused-current-tests.log`). Seven-fix focused **31/31**
  (`focused-structural-tests.log`). Latest focused incremental/provenance suite **12/12 passed**
  (`default-source-final-tests.log`). Complete latest ten-fix suite: **788/788 passed**, exit 0 (`full-final-tests.log`, driver `--jobs 8`).
- App tests passed with six fixes, exit 0 (`app-tests.log`), including project entrypoint compilation, viewport/presentation/settings
  and CAD workflows. Latest ten-fix app run: **passed, exit 0** (`app-final-tests.log`), including the CAD part workflow.
- Compiler action keys hash the host Haxe binary, compiler source, stdlib, explicit Haxe/FFI inputs and launch options.
  Compiler/stdlib content changes correctly choose new workers/artifacts. A gitlink-only bump with identical source and
  stdlib does not independently enter that worker key (those directories have no `.git` marker). Whole-repository native
  actions can additionally include Git identity. No evidence justifies weakening invalidation.
- Cold frontend still discards typing when exact HashMap/DynamicOps runtime edges are discovered. Long-term direction:
  extend the validated declaration graph and resume from a safe checkpoint when an exact edge is added. Preserve owner
  ordering and transaction rollback; do not pre-import speculative runtimes or cache answers across changed semantics.
  Early abort and signature preflight prototypes preserved probe bytes but did not improve CPU (1,500-function probe
  **2.27 -> 2.39 s**; metadata probe **1.30 -> 1.40 s**), so neither is landed.
- The remaining final cold cost is measured: frontend **28.164 s**, typing/publication **23.441 s**, IR **18.147 s**,
  backend **11.072 s**, output **1.536 s**. Typing metadata alone is **12.809 s / 5.80 GB**. The larger intended language
  and application workload is material; reaching 33 s requires further frontend/publication/output architecture work.
  Warm still assembles and lowers the program after a small edit; input identity now enables the inliner memo correctly.
- Out-of-branch probes are retained, not advertised as editor gains: scalar preflights **22.77 -> 3.27 MB** on a
  251-function arithmetic probe; initialization subclass index **206.12 -> 10.73 MB / 0.80 -> <0.01 CPU s** at 4,000 classes.
  Actual editor initializer summaries cost only **61.71 MB / 0.30 CPU s**, transitive reads **2.14 MB / 0.01 CPU s**, so the
  latter is not a priority. Neither speculative optimization was landed.

## Round 2 (2026-11-06, lead)

The starting ten haxeon fixes (up to `57f7c9bc`) cut compiler cold time from 106 to 83 s and a one-line edit from 75 to 34 s.
They initially conflicted with main `36bc1497`, while another session had uncommitted typing/enum changes. Those changes are
now on main as `861c37aa`; both integrations have been rebased and validated below.

Round 2:

1. **Rebase** the branch onto the current haxeon main, resolve the conflicts, and re-validate (haxeon suite, app tests,
   byte-identical cold/warm). Keep the branch landable; the lead lands it once the other session's edits are committed.
2. **Keep cutting**, by phase share:
   - cold compile toward the October per-file rate;
   - warm edit (34 s), which is now the main pain for interactive work.

   For warm edits, find out why a one-line edit retypes and relowers so much (invalidation granularity, the fingerprint and
   session reuse), and what fraction is backend assembly/encode by design. Determinism stays a hard requirement.
3. **Pin bumps and cache invalidation:** measure what a haxeon pin bump costs every worktree (CLI rebuild, project recompile),
   and cut what isn't needed.

### Round 2 result

**Rebased and validated on current main `861c37aa`; haxeon tip `3bfe783e`.** The initial ten fixes now run from
`b13523f9` through `a37d67a7`. Conflict resolution preserves main's concrete enum storage arguments and enum representation
restore/drain, alongside the provenance/default-source fixes. `git range-diff` was reviewed. The matching release runtime
was configured/rebuilt (Ninja: no work); OCCT was not rebuilt. Both branches remain `compile-speed`.

**The October 33 s cold target is not reached.** Round 2 removes more cold IR work and improves warm edits; cold command
latency remains close to the existing result within shared-machine noise. Further substantial cold/warm reductions cross
the frontend/backend design boundaries described below. Determinism was preserved; no architecture change was made.

Final same-window editor runs, using heavy-run, disabled shared artifact restoration, fresh private cold sessions and an
explicit comment edit followed by a no-op. CPU is resident-worker user+system; CLI/native child CPU is reported separately.
The before compiler is `a37d67a7` (new main plus the ten Round 1 fixes), and after is `3bfe783e`.

| Build | Before command / compiler / worker CPU | After command / compiler / worker CPU |
| --- | --- | --- |
| Cold | 117.260 / 106.588 / 137.32 s | 114.470 / 105.222 / 134.93 s |
| Comment edit | 50.356 / 47.062 / 57.83 s | 40.633 / 38.640 / 46.45 s |
| No-op | 2.663 / 0.000 / 0.00 s | 2.173 / 0.000 / 0.00 s |

Warm command/worker CPU falls **19.3% / 19.7%**.
Cold command/worker CPU falls **2.4% / 1.7%** in the first pair, but the
independent edited cold measures **109.629 / 130.68 CPU s before** versus
**111.296 / 130.92 CPU s after**. There is no convincing whole-command cold speedup beyond noise.
Before/after cold load is **13.27–12.17 / 11.38–12.25**; comment load **12.17–11.51 / 12.25–10.16**. These are serial
samples in the same window, not isolated-machine latency claims. No-op variance is not credited to the scalar fixes.
CLI/native child CPU before is **11.465 / 3.845 / 3.454 s**;
after **9.961 / 3.170 / 3.204 s**.
Restored-source warm command/CPU is **50.973 / 56.09 -> 41.886 / 46.23 s**.

Phase wall seconds (IR includes representation passes and the inliner; nested figures below are included, not additive):

| Phase | Cold before | Cold after | Comment before | Comment after |
| --- | ---: | ---: | ---: | ---: |
| Frontend | 34.394 | 37.944 | 2.074 | 2.320 |
| Typing/publication | 27.855 | 28.915 | 3.264 | 2.141 |
| IR assembly/inliner | 26.166 | 21.578 | 18.773 | 14.616 |
| ABI | 0.132 | 0.178 | 0.172 | 0.196 |
| Backend assembly | 14.479 | 13.452 | 17.249 | 15.518 |
| Encode/output | 2.422 | 2.209 | 4.445 | 2.773 |

Final cold frontend graph/restarts: **34.120 s**; canonicalization **3.043 s**. Typing metadata: **15.343 s / 5.842 GB**.
Final backend verify/function lowering/debug assembly: cold **2.330 / 6.734 / 3.500 s**, warm **2.821 / 8.523 / 3.190 s**.
Compile GC wall: cold **11.411 -> 11.079 s**, warm **3.257 -> 3.089 s**. Driver compile allocation falls
**41.581 -> 40.580 GB cold**, **14.351 -> 12.591 GB warm**; IR allocation **9.266 -> 8.265 GB cold**, **7.180 -> 5.420 GB warm**.
Backend function allocation stays about **3.439 GB**. This establishes less IR work even where cold wall time is flat.
Cold retypes **29,993** functions; the comment edit retypes **33** callers with embedded source-dependent defaults.
Warm frontend+typing is **4.461 s**; IR/backend/output consumes **32.907 s**, about **81%** of command wall time.

### Round 2 fixes and origins

| Commit | Fix and measured evidence |
| --- | --- |
| `12d7861f` | Hash Unix CLI input content, including host Haxe/stdlib, additions and removals. Timestamp-only source touch: **1.49 -> 0.12 s wall**, **1.42 -> 0.12 CPU s** in the launcher probe; no-op **0.10 -> 0.12 s**. The integration fixture checks content edits with preserved mtime, touches, additions/removals, toolchain changes and missing artifacts. |
| `a2e5dcac` | Skip value replacement, value-phi graph construction and nullable flow setup when no candidate exists. The 251-function/64-block probe falls **22.773 -> 3.267 MB**, **0.03 -> 0.01 CPU s**; exact canonical IR, identity and verifier checks pass. Registered 5 MiB guard fails before and passes after. |
| `2da3ef28` | Retain irrelevant functions during scalar closure lowering. Inspect every rewritten type position, including phi operands, literals and debug bindings. The 253-function probe falls **16.386 -> 8.828 MB**, **0.02 -> 0.01 CPU s**, with identical canonical IR and all 250 unrelated identities retained. Registered 12 MiB guard, positive redirection and exact body-state checks pass; focused suite **17/17**. |

Each performance fix is separate, with before/after evidence and the requested trailer. `7e79b85d` and `3bfe783e` only
format merged code. All **33 changed Haxe files pass** the pinned formatter.
The scalar work originated in `ecd34e1e` (standalone replacement), `6871c5be` (value phis), `7a2a53e8` (nullable flow),
and `ac5c2f79` (closure lowering). History/blame localizes those unscoped walks/copies; the paired probes on the rebased
baseline demonstrate their cost. The timestamp cache test originated in `9967cffc`. These are not new full historical
editor bisection claims; Round 1's reproduced array/non-null regressions remain documented above.

### Cache and determinism validation

Final timestamp-only checkout simulation touches `HaxeonCli.hx` and `Compiler.hx` without changing content.
Project no-op is **2.071 s / 3.028 CLI/native CPU s**; after the touches,
**2.001 s / 3.004 CPU s**. CLI inode/mtime are retained,
the project action stays clean, no compiler is called, and bytes match fixed cold. All five cache checks pass.
Compiler/stdlib/host-tool content and launch options still invalidate compiler/project results. Git identity can still enter
whole-repository native inputs; that conservative behavior was not weakened. A real implementation-changing pin still pays
the measured cold compile. This optimization removes timestamp-only CLI rebuilds, not required compiler invalidation.
The final fixed-source editor restoration/cache prime measured **105.564 s** and left the original-source output in place.

Original/restored source digest: `1972fc377ff72a18bad11a994e88351a6b601d885e3a1a9400997f964b22a0f1`.
Edited source digest: `00452bdecbb44b08b308fef821e43164659156ef2053462da5e8a76883ee6e76`.
Original/restored artifact SHA-256: `5c5ef5ea1808de7945f6a9ebf1cf5c04458a1b180d0d82e71303ad174839795e`.
Edited cold/warm artifact SHA-256: `5d9de4921ff33a7cec7d5a70356a54dc6b79b9723ec956f23cf6a124ca146577`.

- Complete new-base compiler suite: **795/795 passed**, exit 0, jobs 8. Explicit matching HashLink paths, a private TMPDIR
  outside the checkout, and the current prebuilt Main follow the supported driver workflow.
- App tests: **passed, exit 0**, including viewport/presentation/settings, compiler-input invalidation, entrypoint geometry,
  and the CAD part workflow.
- Both before and after pass all five original/restored/edited cold/warm checks. No warm worker restart or fallback occurred.
- **30/30 checks** pass across eight artifacts: **24 complete hldump output comparisons** (sections, executable hashes,
  identities and source spans) plus six raw-artifact digest comparisons. **Six literal cmp checks** also pass. Before/after
  outputs are identical, as are independent cold/warm pairs.
- Matching release runtime rebuilt successfully; OCCT was not rebuilt. Samples use heavy-run and a **16 GiB** pre-run
  headroom gate on two samples. The prior 18 GiB waiter was replaced before any new-base editor measurement began. The worker
  monitor records high-water marks and zero worker swap. No other session was signalled; private workers were shut down by RPC.

Evidence is under `/media/joao/DATA1/claude-hobber-tools/tmp/compile-speed/`, prefix `round2-main861-`:
`validation.json`, `summary.json`, `full-suite.log`, `app-tests-run.log`, `pin.json`, `hldump.json`, `byte-compare.json`,
`memory-peaks.json`, and all raw per-build logs/artifacts. Earlier failed environment attempts are not validation claims.
The fully validated earlier `36bc1497` integration remains on `compile-speed-round2-validated-36bc1497`; its lower-load
repeat measured **91.316 / 35.502 / 1.741 -> 90.731 / 32.548 / 1.759 s**, with worker CPU
**106.90 / 39.86 / 0 -> 105.95 / 36.56 / 0 s**. Those numbers are retained separately, not substituted for the new base.

### Remaining boundary

The larger workload and intended language features remain material (Round 1 measured **2.39x explicit source bytes**).
Cold graph/restart work alone now costs **34.120 s**; metadata costs another **15.343 s**. A major further cold reduction
needs a reviewed safe checkpoint/resume design for exact runtime edges. Preserve declaration/owner ordering and transaction
rollback; do not pre-import speculative runtimes or cache across changed semantics. Warm artifact builds still assemble and
lower the entire canonical module; substantial further savings need a reviewed design for reuse under deterministic symbol
layouts. Neither architecture change nor any relaxation of determinism was implemented, following the lead's stop rule.

All compiler fixes are local commits in the designated clone. Materia commits change this plan only. The haxeon/pcbkit
pointers and four pre-existing machinekit edits were never committed. No push or merge was made; the branch is ready for
lead review/landing. The cold target remains unmet, with its measured costs and next design decisions explicit above.
