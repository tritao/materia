# Editor performance gate

The structural edit regression uses 1,000 objects and checks counters for a
colour edit and a selection change. Run it with:

```sh
./haxeon/scripts/haxeon run --project app/tests/performance/edit-regression.json
```

It checks that these edits avoid whole-scene reconciliation and spatial-index
rebuilds, and that the colour edit reaches the renderer as one SceneKit change
set. It has no timing threshold.

From the repository root, run:

```sh
python3 app/tools/check-editor-performance.py
```

The command builds the current app and headless benchmark, runs three 500-cycle
Hierarchy/Sensors interaction captures without a display server, and measures
30 seconds of desktop idle time. It needs `DISPLAY` or `WAYLAND_DISPLAY` for
the idle phase. In headless CI, use `--headless-only` to run the interaction
and retention checks. Use `--skip-build` only when the app and benchmark
binaries already contain the source being tested.

The scenarios start the editor with the demo scene (the plain editor starts empty) and the tab matrix covers the
Hierarchy/Sensors and Console/Telemetry groups. The `architecture` scenario currently fails building its 500-link model
(`simulation.addRobot failed with RobotKit status -8`, a physics backend error) and is not part of the gate.

Each interaction run verifies the inspector rename, frame p95, RSS growth
after the first 100 cycles, and the retained listener, widget resource, style,
state, and key path counts. The RSS check allows up to 32 MiB total growth
after warmup and rejects two consecutive 100-cycle windows above 8 MiB each.
The frame p95 limit is 30 ms. Desktop idle checks ignore the first five
seconds and reject consecutive five-second windows with over one CPU second
or 16 MiB RSS growth. These limits were set against three 500-cycle captures
and several desktop idle captures on 2026-09-23; the interaction captures
showed 11.7–12.2 ms p95 and 8.1–8.9 MiB RSS growth after warmup.

The gate writes all raw captures under `app/build/profiles/gate-*`. To inspect
existing captures without launching the app, pass one or more `--capture`
paths and optionally `--idle-capture` with the matching `--idle-seconds`.
The JSON summaries show the measured values and any failed checks.

For a heap snapshot, run `python3 app/tools/profile-editor.py --scenario
tab-inspector --cycles 100 --heap-dump`. The capture contains `heap.dump`, the
exact `headless-profile.hl` bytecode used for that run, and a `capture.json`
manifest. Inspect it with `haxeon/scripts/haxeon heap inspect --capture
<capture>`; the shared command writes `heap-report.txt` beside the dump. It
also accepts explicit bytecode and dump paths when no manifest is available.
The headless workload releases its saved frame and action JSON buffers before
the heap snapshot so they do not appear as retained editor memory.

To investigate tab-switch latency without a display server, run
`python3 app/tools/profile-editor.py --scenario tab-matrix --cycles 50`.
The workload exercises every ordered pair within the Hierarchy/Sensors group
and within the Viewport/Perspective/Console/Telemetry group. The report shows
the median, p95, and maximum input-plus-frame latency for each transition,
along with allocated bytes and GC collections. Cycle 0 is excluded from the
steady latency summary; `tab-spikes.json` keeps the 20 slowest transitions,
including cold switches. Each row records target lookup, bounds calculation,
pointer down and up, click capture/target/bubble dispatch, frame and tree/style
time, GC activity during input and submission, and style cache misses. Headless
scenario timeouts scale with the requested cycle count. Run with `--no-profile`
for lower-overhead timing, or with the default profiler to inspect the matching
`editor.perfetto.json` trace and `span-spikes.json` report. The benchmark measures headless input handling
and frame submission; it does not include desktop presentation latency.

To capture the architecture workloads, run `python3 app/tools/profile-editor.py
--scenario architecture --cycles 1`. This records CAD parameter recomputes, BIM
opening edits, construction of a 10,000 object scene, a single object nudge,
500 moving object updates, 1,000 ordered undo operations, and presentation
captures for a 500 link robot. The capture writes phase timings to
`architecture-summary.json` and the usual frame, action, memory, and profiler
artifacts beside it. Scene construction breaks the load into geometry-data
generation, batched geometry and material create/publication, transaction
preparation and commit, application bookkeeping, and final snapshot and
spatial-index creation. Timers are enabled only for this profile run. The
summary also separates SceneKit snapshot capture from spatial-index construction
using five warmed samples after the workload.
Increase `--cycles` to extend repeated movement, CAD, BIM, and simulation
measurements. The scenario is intentionally not part of the default regression
gate because it exercises large workloads.

The tab matrix uses 50 Hz CPU sampling with allocation sampling disabled. The
500 Hz setting saturated the profiler on this workload. Use `--sample-rate` and
`--allocation-interval` to tune a capture; compare latency with `--no-profile`.
The capture command builds the Release HashLink runtime before each run, even
with `--skip-build`, so timing comparisons use the same optimized VM. Haxeon's
Debug CMake preset writes its VM to its own build tree.

## Frame-boundary collection

`DesktopUiHost` turns automatic garbage collection off while a frame renders and
collects afterwards instead: when the host goes idle, or right after a frame
once allocation since the last collection is three times the collector's own
trigger. The tab-matrix workload models this by wrapping each submit and
collecting after each interaction. Set `MATERIA_FRAME_GC=0` to switch it off and
compare; `frame-gc.json` in a capture records the idle and forced collections.

On 100 tab-matrix cycles without the profiler (2026-09-29), interaction latency
(input plus frame) went from 3.32 ms median, 4.60 ms p95 and 7.61 ms max to
2.68 ms, 3.72 ms and 5.57 ms, and no interaction contained a collection (280 of
396 did before). Total mark time was unchanged at about 290 ms; it moved into
200 idle collections averaging 0.7 ms.

## Where a frame's allocation goes

`UiFrameMetrics` records the bytes allocated by each submit phase (`viewAllocatedBytes`, `treeBuildAllocatedBytes`,
`treeCompareAllocatedBytes`, `nativeLayoutAllocatedBytes`, `reconcileAllocatedBytes`, and the render phases), and the
headless profile writes them into `frame-timeline.jsonl`. They read `hl.Gc.totalAllocated`, so they are 0 on Wasm.
The sampling profiler only attributes allocations to the native allocator, so these counters are the way to find which
phase to look at first.

On the tab-matrix workload, median allocation per interaction went from 1264 KiB to 587 KiB and median latency from
2.96 ms to 1.69 ms: `StyleDiff` compared every property of every node through copying accessors (464 KiB, now 23 KiB),
`RenderNode` no longer allocates handler, paint and decoration containers it never uses, and string building moved to
the compiler's one-allocation concatenation.

## Per-node allocation costs

`python3 app/tools/profile-editor.py --scenario primitives` prints the bytes and time each building block of a rendered
node costs in isolation (`new RenderNode`, `new StyleTarget`, a style-cache hit, `toLayoutStyle`, a scope and widget id,
and whole `Text` and `Button` builds). It is single-threaded, so unlike the phase counters it is not mixed with
allocation from the workspace save worker. On 2026-09-30 a `Text` build cost 3.3 KB and a `Button` 5.5 KB; a
`RenderNode` is 0.9 KB, a scope and id 0.5 KB. A tab switch rebuilds about 320 nodes, so the fixed per-node cost, not
any one widget, sets the tree-build total (about 530 KiB); shrinking it much further means reusing unchanged subtrees
instead of rebuilding them.

## Reading a profile

Sampling used to hold the program stopped for about 650 us per sample (`selection-stress` measured 21-25 ms per frame profiled
against 2.8 ms without), because the stack capture scanned the whole stack and asked a quadratic question about every
candidate. It now follows the frame-pointer chain and stops a thread for about 8 us, so at 2000 samples a second a frame costs
about what it does unprofiled. Every profiled headless scenario still reruns itself without the profiler (skip with
`--no-baseline`) and prints `profiler overhead: median frame X ms profiled vs Y ms unprofiled`, also saved as
`profiler-overhead.json`, so a regression shows up. `HL_PROFILE_STATS=1` makes the VM print, when profiling stops, how long
threads were stopped per sample and how long a sample took. The runtime needs the current fork (`cmake --build --preset release`
in `haxeon`); an older `hl` still shows the old overhead.

`profile-report.txt` (printed in short after the run) is `haxeon/scripts/hlprof-report.py` over the Perfetto export. It drops
samples whose leaf is a blocking wait (about half of a short capture is a parked worker) and keeps only samples taken while
`UiContext.submit*` is on the stack, so it describes frames and not setup or idle. Run it by hand to change that:
`python3 haxeon/scripts/hlprof-report.py <capture>/editor.perfetto.json --within <function> --top 30`. It lists self and
inclusive time with readable generic and lambda names, and flags `<Dynamic>` generic instances on hot paths (boxed values).
Raise `--sample-rate` (default 500 a second; 2000 is fine now) for a short scenario: a 40-cycle `selection-stress` yields only a
few hundred samples.

## Allocation census

`python3 app/tools/profile-editor.py --scenario tab-matrix --cycles 30 --no-profile --census` counts every allocation by
type (exact) and samples the allocating call stack once per 4 KiB (jittered), then writes `census.json` to the capture
directory; `python3 app/tools/allocation-census.py <capture-dir> --cycles 29` prints allocation per cycle by type and by
allocating function. The first tab-matrix cycle is excluded so cold caches do not count. It runs in the runtime
(`hl.Gc.censusStart/Reset/Stop/Dump`), so it sees runtime allocations (strings, arrays, boxed values) as well as compiled `new`.
Allocations made entirely inside native code show up as `(native)`; the harness's own JSON and frame recording appear too.

On 2026-09-30 the first census found `BuildContext.claimRetainedTree` building an unused description string for every
retained node (26% of all bytes); dropping it took a tab-matrix cycle from 5.56 to 4.08 MiB. `Transform2D`, `Point`, `Rect`
and `LayoutAxis` together were about 16%, which is why converting one of them to a value class could not move the total.

Fixes the census led to, in order (tab-matrix cycle, KiB): unused ID descriptions in `claimRetainedTree` 5,556 -> 4,075; allocation-free
point mapping and hit testing 3,596; `Map.clear` keeping its storage 3,115; reusing last frame's `ResolvedLayoutItem` when its
record is byte-identical 2,761; `LayoutTransaction` clearing its maps, an allocation-free `RenderNode.find` 2,570; a flat handler
list and an allocation-free cycle check in `RenderNode` 2,440. About 12% of what remains is the profiling harness's own JSON and
frame recording.

## Interaction frames

`python3 app/tools/profile-editor.py --scenario interaction --cycles 60 --no-profile` drives input that leaves the tree structure alone and writes
`interaction.json`: bytes allocated across the input events and the frame (`hl.Gc.totalAllocated`), and how many frames were dirty, per action. The `idle` and
`hover-inside` actions show the measurement floor: about 52 KiB of that is the harness recording its own frame JSON, not the app, so subtract it.

On 2026-09-30 (KiB per frame, harness floor of about 52 included):

| action | total | dirty | what it rebuilds |
|---|---|---|---|
| hover onto another tab button | 340 | always | dock chrome, `console` and `perspective` (no cache key); `inspector` and `hierarchy` hit their caches |
| hover inside the same node | 54 | never | nothing |
| scroll the inspector | 472 | always | the whole inspector panel (about 110 nodes) |
| type a character in the inspector | 480 | always | the whole inspector panel, plus the field |
| idle | 52 | never | nothing |

A frame with no state change is free, but any hover, scroll or keystroke rebuilds a whole panel or the whole dock chrome: a panel's cache is invalidated by any state change inside it, and a
hover changes an interaction state that every widget on the path reads at build time. Getting these frames near zero means updating in place (a hover restyles the node it landed on; a scroll offset is
applied in layout, not by rebuilding the content) and invalidating at the widget that owns the state, not the panel.

### Retaining dock panes and caching the console and perspective panels (2026-09-30)

`DockWorkspace` now retains each pane (its tab strip and active panel) as one subtree, keyed by the pane's tabs, width, style and viewport, the drag revision and the active panel's cache key,
and revalidated against the interaction and state inside it. A hit replays the pane's drop-target registrations, because the interaction object forgets them every frame. A pane whose active panel has
no cache key is never retained. `console` gained a cache key (log revision, stale edits, theme) and `perspective` reuses the chrome revision key plus the two inputs it lacked.
`profile-editor.py --scenario dock-drag` drags a tab onto another pane and fails if it does not dock: it fails when the replay is removed, so it guards this.

KiB per frame (harness floor of about 52 included), before to after: hover onto a tab 340 to 207, scroll the inspector 472 to 340, type a character 480 to 352.
What a hover still rebuilds is the hovered pane's own tab strip (about 110 KiB of tree build); scroll and typing still rebuild the whole inspector panel.

### Scrolling without a rebuild (2026-09-30)

A `ScrollView` already applied its offset to the content in place after layout, but its controller also called `State.update`, bumping the state revision and so invalidating every cached
subtree around it (a scroll rebuilt the whole inspector, about 110 nodes). It now only requests a frame (`commands.refresh()`); the layout feedback that was already there moves the content.
Scroll frames in the interaction scenario: 340 to 245 KiB (harness floor of about 52 included). The scenario asserts the content really moves.
`MATERIA_TRACE_STATE=1` prints which states each action changed (`StateStore.idsChangedSince`); a keystroke changes exactly one, the name field's, yet still rebuilds the inspector panel.

### In-place widget patches and cache validation that ignores structural nodes (2026-09-30)

- **Cache validation.** `RetainedView` and the dock caches compared every node's recorded hover, press and focus bits with the current ones. Structural nodes (`tab-strip`, `scroll-content`, `editor-content`) sit in the
  hovered chain but never record interaction state, so while the pointer rested inside a panel or a tab strip its cache could never validate and it rebuilt every frame. `RenderNode.recordsInteraction` (set when a widget assigns `states`)
  now limits the comparison to nodes that actually read interaction state.
- **Self-updating widgets.** `BuildContext.selfUpdating(id, build)` records the scope, inherited style and text style a widget was built in. When the widget's own state changes it updates that state with
  `State.updateQuietly` (no revision bump, so no ancestor cache is invalidated) and calls `requestPatch(id)`. `applyPatches()` runs after `beginFrame()` and before the root builds: it re-runs the builder, swaps the new node into the retained
  tree (`RenderNode.replaceWith`), claims no IDs (the retained walk claims them), and hands the frame comparison the replaced nodes as its priors. `TextField` is the first adopter.
- **Verification.** The interaction scenario compares the tree after a patched keystroke with the tree after a forced full rebuild (hover and press flags excluded: the event dispatcher writes them onto live nodes).

KiB per frame (harness floor of about 52 included), from the start of the interaction work to now: hover onto a tab 340 to 177, scroll the inspector 472 to 215, type a character 480 to 196.

### What did not pay (2026-09-30)

Two attempts to widen in-place rebuilds were measured and removed:
- Making each tab header (button, indicator, drag handlers, key handling, drop target) a self-updating unit triggered by its button's hover. A patch cost about 13 KiB and the strip was not being rebuilt anyway once the
  cache validation ignored structural nodes, so per-frame bytes did not fall. It also needed drop targets to follow replaced nodes.
- Having `TextField` also patch itself on hover, press and focus changes: a hover onto the field went from 199 to 208 KiB.

The interaction scenario now also has `hover-field` and `hover-away`, and `MATERIA_INTERACTION_ONLY=hover-enter,hover-other` (comma list) skips every other action, so the census can be run on one kind of frame.
The remaining per-frame cost is flat in the census (nothing above about 3%): pane key strings, boxed booleans returned by `RenderNode.walk`, style diffs, event objects. Note that `hl.Gc.totalAllocated` is global, so the 52 KiB idle floor
includes another thread; the census attributes about 95 KiB to a whole hover frame including the harness.

### String literals are created once (2026-09-30, haxeon `23104c59`)

Every string literal evaluation used to allocate a 24-byte `String` object, even a plain comparison, so `RenderNode.invoke` alone allocated about 8 KiB per frame passing `"capture"`, `"target"` and `"bubble"`.
In a build nothing will patch (the default; `--live` is off) a literal is now a global holding one `String` object, created by the VM from the HLB constants section when the module loads; a live module keeps the old path because
a patch cannot add a global. Same app source, old compiler against new, tab-matrix cycle: allocations 42,466 to 32,828 (-22.7%), bytes 1,714 to 1,488 KiB (-13.2%), `String` objects 12,034 to 2,396.
An IR pass that hoisted each literal to the entry block was tried first and made things worse (+14% allocations: error-message literals in loops were allocated on every call, loop or not) and was reverted; only a constant is free.
Interaction frames with the new compiler (KiB, harness floor of about 48 included): hover onto a tab 153, scroll 196, type a character 174.

### Allocation-free ID sets (2026-09-30, haxeon `aa5dc31b`)

`Map<Int, Bool>.set` boxes its value on every insert, and the per-frame ID claim set and the state-usage set are filled for every node. `IdSet` (open addressing over two arrays, `clear` is a generation bump)
replaces them, and `StateStore.usedIdsSince` reuses a scratch set instead of making a map per call. Tab-matrix cycle with the same compiler: allocations 32,828 to 28,522 (-13.1%), bytes 1,488 to 1,416 KiB (-4.8%), boxed
booleans 4,251 to 16 per cycle. Interaction frames (KiB, harness floor of about 48 included): hover onto a tab 145, scroll 187, type a character 165.
What is left in the census is many sites of 1-2% each: pane cache key strings, `FocusManager.collect`, `LayoutAxis.fit` per node, style diffs, geometry decode for changed nodes.
