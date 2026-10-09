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

## Checks to run before merging

Three scripts catch what the tests of one kit do not:

- `tools/check-compile-all.py` compiles every tracked `haxeon.json` project and the script-built targets in
  `tools/compile-sweep.json` (simulation bindings; the UIKit showcase with `--fast` omitted), so a compiler change cannot
  break a kit nobody builds that day. Library packages (no `entry`) are built through the projects that use them. About
  10 minutes at `--jobs 3`. Its first run found two binding checks that had been failing for days: they used a type
  without importing it.
- `tools/check-determinism.py` builds the performance project cold, then again incrementally after an edit, then cold from the
  edited source, and requires the last two to be byte-identical (two edits: a line inserted at the top of
  `BuildContext.hx`, and a longer string literal). On a difference it decodes both with `hldump`. About 4 minutes per edit.
- `app/tools/check-editor-performance.py --budgets` runs the headless scenarios and holds each action's median frame time
  and allocation to the ceilings in `budgets.json`. Time ceilings are loose (machines differ), allocation ceilings tight.
  `--budgets --measure` prints the measured values, to set a budget after a deliberate change. The older default mode
  checks for leaks and unbounded growth.

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

## Comparing two runs

`python3 app/tools/profile-compare.py BEFORE AFTER` (capture directories) compares per action the median frame, tree/style,
native layout and allocation, and, when both captures have a profile, each function's share of frame-submission samples.
`profile-editor.py ... --compare BEFORE` runs it after a fresh capture. A change is called real only if it exceeds the noise
of both runs and `--min-change` (10%): two runs of one build differed by up to 8% in a frame's median, so smaller changes are
reported as noise. Profile shares are relative (one function getting cheaper raises the others), and they count as different
only beyond two standard errors of the sample counts. To measure a change, capture with `--no-profile` before and after
(same scenario and `--cycles`), then use `--sample-rate 2000` captures to see where the time moved.

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

## Belt presentation

Belt animation writes tooth geometry into reusable packed buffers, caches each
belt's driver phase and attachment positions in the belt's own frame, and
publishes all changed belts with one application scene refresh. The display
cache ignores movement below 0.01 mm; simulation and collision geometry retain
full precision. Triangle indices are reused while the seam topology is unchanged.

Run the mesh equivalence and MuJoCo presentation checks from the repository root:

```sh
./haxeon/scripts/haxeon run --project machinekit/tests/belt-mesh/haxeon.json
./haxeon/scripts/haxeon run --project app/tests/belt-display/haxeon.json
```

The mesh check compares positions, normals, indices and bounds against the
original tooth builder at positive, negative and seam phases, including a
mixed-side CoreXY route. The presentation check covers unchanged frames,
independent belt invalidation during gantry homing, pause and reset. The default
app runtime-geometry tests also check batched publication and restoration.

In a gantry picker desktop capture at 1600 by 1000 with 4x MSAA, median frame
work fell from about 121 ms to 23 ms and allocation from 127 MiB to 1.6 MiB per
frame. Observed playback rose from about 7 to 19 FPS. These are measurements,
not timing gates; presentation scheduling and GPU work remain in the total.

## Desktop frame scheduling

GTK frame delivery wakes the event waiter so the owning loop can advance the
simulation and request another frame. The desktop host also skips a second wait
when polling already delivered a frame. Capture metrics include
`scheduling.hostLoop`: poll and wait durations include nested frame callbacks;
`pollFrameSeconds` and `waitFrameSeconds` report those callback portions.

The native `gtk_surface_request_frame` integration test checks that a requested
frame interrupts a long wait and that on-demand rendering returns to idle. The
`time` unit test covers normal timeout and cross-thread wake behavior.

Viewport clipping and framing index simulation poses by object ID and reuse
pose transforms within each supplied presentation. New simulation input always
invalidates the cache, including input with the same revision. Centred box
bounds use affine half extents (`abs(M) * halfSize`) rather than eight temporary
corner arrays. Scene editing tests cover transform invalidation, simulation stop,
and rotated/scaled bounds.

## Native waits and geometry reconciliation

GTK waits iterate GLib with a bounded timeout source instead of sleeping on a
condition variable while native frame sources become ready. Cross-thread wakes
also wake GLib. The `gtk_event_wait`, `gtk_surface_request_frame` and `time`
native tests cover source delivery, explicit wakes, cleanup and on-demand idle.
A 2 ms GLib timer benchmark measured 10.07 ms median wake latency before and
2.06 ms after; idle waits continue to sleep.

The GPU executor checks immutable geometry revisions before validating indices
or packing surface, picking and stroke buffers during reconciliation. Changed
revisions retain validation and upload behavior. GPU tests cover cached
rendering/picking across a fresh plan and subsequent geometry changes.

A gantry comparison with identical bytecode, camera and 1600 × 1000 window
measured median frame work 21.45 → 16.81 ms and render work 13.55 → 8.00 ms.
End-to-end throughput stayed near 20 FPS (19.93 → 19.79); this pass demonstrates
less render work, not an FPS gain. Desktop measurements remain observational.

GTK successor frames are submitted before returning from the frame callback.
The GTK pump returns when work wakes the owner, so a slow self-requesting
frame cannot monopolize event draining. Early submission is confined to
Linux/GTK; other desktop backends keep their existing timing.
The request-frame integration test includes a 40 ms self-requesting frame and
checks that polling yields between frames.

A capture pair with input and resizing locked, matching camera, 1600 × 1000
and 4× MSAA measured 18.91 → 20.79 FPS (+9.9%). The previous native resource
cache and GLib wait changes were present in both builds. This is a desktop
observation under current load, not a timing gate.


## On-demand GPU picking data and capture timings

Animated geometry updates now upload surface data without expanding and uploading
per-triangle pick vertices. Pick and depth requests refresh their buffers from
the current immutable geometry revision, including position streams. Tests move
an already-picked mesh, check the old location misses, and restore it with
non-indexed triangles before checking the next pick.

A controlled 1600 × 1000 gantry pair (4× MSAA, fixed camera, final eight seconds)
measured CPU scene preparation 5.59 → 3.87 ms median, frame work
19.90 → 18.44 ms, and throughput 18.29 → 21.03 FPS. These are observational
captures under desktop load, not a reproducible performance gate.

Set `NK_SCENE_GPU_TIMINGS=/absolute/path/timings.jsonl` to opt into native scene
capture diagnostics. JSONL records include epoch start time, CPU preparation,
CPU submission and GPU draw milliseconds. GPU queries are polled on later
frames, never waited on, and capped at eight pending queries. Unsupported or
unfinished queries report null. GPU draw timing covers the main scene pass
between pass begin/end; it excludes target clear, MSAA resolve, optional
postprocessing, UI composition and desktop presentation. CPU submission includes
pass completion and optional postprocessing. Logging and query flushes can affect
timing; keep this environment variable unset for normal use.

A separate capture with valid GPU queries measured main-pass GPU draw time
0.486 ms median (1.642 ms p95, 118 samples), CPU preparation 2.405 ms and
submission 0.621 ms median. Its throughput was 14.67 FPS with 22.58 ms
median frame work; query flushes and desktop load make this unsuitable for
comparison with the preceding pair. These GPU figures do not measure GTK/UI
composition or presentation waits and do not establish their cost.

Build and GPU rendering/picking suite passed, with diagnostics both disabled
and enabled; the enabled suite returned 52 valid GPU timing samples.


## Shared belt rings and retained index uploads

A fresh hlprof-live capture, restricted to the final eight seconds and
`UiHostRuntime.render`, attributed approximately 20% of 358 frame samples to
`PackedTimingBeltMesh.update`. Its face emission accounted for 12% directly.
The builder now transforms and converts each shared ring once, copies its
float32 positions into adjacent faces, and converts each flat face normal once.
Reference comparisons pass for positions, normals, topology, bounds, seam phases,
and both tooth directions. A 162,496-vertex CPU benchmark measured median
3.52 → 2.90 ms; p95 varied from 3.85 → 4.32 ms in that pair.

The GPU executor reuses its vertex-packing allocation, resetting all default
attributes on each use. It compares index contents against retained immutable
source data and uploads indices only when they change, including changes with
identical index counts. This avoids an additional retained copy of the index
array. Geometry validation runs once on the GPU preparation path. Renderer tests
cover equal-sized index changes, indexed/non-indexed transitions, rendering and
picking after mesh changes.

Three subsequent alternating CPU benchmark pairs measured belt update medians
3.55/3.52/3.52 ms before and 2.92/2.92/2.90 ms after (about 17% less time).
A separate software-GL benchmark updating an otherwise unused 300,000-vertex
resource, with no raster draws, measured 9.98/10.02/10.20 ms before and
3.93/5.41/5.28 ms after. These isolate different operations and are not whole
application frame timings.

The controlled gantry comparison matched camera, 1600 × 1000 size and 4× MSAA,
with GPU query diagnostics disabled. Throughput was 22.09 → 22.42 FPS and
median frame work 15.38 → 16.48 ms. This pair establishes no meaningful
end-to-end gain despite the focused benchmark improvements. Build, reference
belt mesh comparisons and the GPU rendering/picking suite passed.
Capture artifacts: `/tmp/materia-topology-belt-measure`; steady-state profiler
report: `/tmp/materia-next-profile/profile/simulation-report.txt`.


## Allocation sources during gantry simulation

`MATERIA_CAPTURE_ALLOCATIONS=1` with a timed diagnostic capture writes
`allocations.json`: exact per-type allocation counts/bytes plus caller stacks
sampled every 64 KiB. Counting starts four seconds after capture readiness to
exclude project loading and simulation startup. The census disables allocation
fast paths and affects throughput; use a separate capture without this option
for timing. Live allocation samples alone identify native allocator entry points,
whereas the census resolves the Haxe callers needed for this investigation.

The initial eight-second census recorded 341 MB and 7.50 million allocations.
Temporary `BeltPoint` objects accounted for 50.3 MB (14.7%, exact count).
Stacks passing through `KeyScope.prune` accounted for an estimated 36.3 MB
(10.6%). Its map iterators copied arrays/strings while walking an unchanged tree.

`TimingBelt.pointAt` now accepts an optional caller-owned output. Ordinary calls
still return independent snapshots; the packed display mesh owns and reuses one
sample. Pruning keeps dense lists alongside lookup maps and compacts the lists
in place. It preserves cache eviction, retained scope identity and deterministic
widget IDs on remount. The shared-ring geometry and normal calculations remain
the same.

The belt benchmark dropped from 408,885 to 2,606 allocated bytes/update
(99.36% less). Pruning a retained 2,500-entry tree dropped from 288,112 bytes
per traversal to zero; measured traversal time was 0.179 → 0.028 ms.
A controlled gantry pair with matching camera, 1600 × 1000 size and 4× MSAA
measured median allocation 1,582,472 → 1,122,112 bytes/frame (29.1% less),
and p95 2,222,968 → 1,131,456. A subsequent census contains no `BeltPoint`
allocations or stacks attributed to `KeyScope.prune`. Total census bytes are not
directly comparable because the faster run rendered more frames.

FPS is not an acceptance result for this pair: the baseline shows repeated
100 ms frame-delivery gaps, so its presentation cadence differs from the
updated run. The allocation result and isolated operation benchmarks are the
useful evidence. Incremental GC and collector settings were left unchanged.

Build, belt mesh reference comparisons (including output reuse and independent
samples), and UI key-scope eviction/remount tests passed. The latter can be built
with `--compiler-only --project haxeon/packages/ui/tests/key-scope/haxeon.json`.
Raw captures are in `/tmp/materia-census-measure` and
`/tmp/materia-allocation-opt-measure`.

Capture timelines now freeze frame work before creating diagnostic records and
separately report preparation, application submission, painting, bookkeeping and
GC counters. This prevents a GC triggered by diagnostic record allocation from
being misattributed to application rendering. These phase probes run only with
capture diagnostics enabled.

### Remaining allocation sources: runtime publication and pose transforms

Runtime display geometry now publishes a visual revision without advancing the
saved document or physics environment revisions. Runtime geometry/bounds,
restoration, and running belt tests pass. This did not materially change measured
gantry allocation (1,124,000 → 1,121,824 median bytes/frame), so it is not counted
as an allocation optimization.

The next census attributed about 5.1 MB of sampled allocation to constructing
presentation pose transforms across 193 captured frames. ScenePresentation now
retains scratch transforms by pose slot and updates their matrix in place;
SceneView copies them into its own pose overrides. A picking regression checks
that a subsequent frame moves the object while an earlier view retains its pose.
The scene editing suite, running belt integration, and application build pass.

A subsequent 1600 × 1000 gantry capture measured 1,081,584 median bytes/frame,
about 40 KB/frame (3.6%) below the preceding capture. The fresh baseline replay
stalled during project loading and was terminated; this comparison uses the
preceding successful baseline, so it is not a clean timing comparison. FPS is
not claimed as a result. Collector settings remain unchanged. Artifacts:
`/tmp/materia-runtime-ui-measure`, `/tmp/materia-transform-current`, and
`/tmp/materia-transform-census`.

The follow-up census also stalled during project loading and was stopped before
simulation, so removal of the sampled stack is not yet independently confirmed.
The successful normal capture above and the view-independence regression are
the validation for this iteration.

### Viewport clipping scratch storage

The viewport now retains pose transforms across simulation presentations and
reuses bounding-box arrays while fitting camera clip planes. Its per-presentation
lookup is still cleared for each input, so moved poses refresh even with an
unchanged revision, reordered poses cannot use another object's cached matrix,
and stopping simulation restores authored transforms. The retained storage is
bounded by the largest number of displayed poses/bounds for that viewport.

A matched gantry pair at 1600 × 1000 measured median allocation
1,079,736 → 1,017,964 bytes/frame (5.7% less), with p95
1,095,792 → 1,033,328. Median frame work was essentially unchanged
(13.38 → 13.29 ms); presentation FPS is not claimed as an optimization result.
The application build and scene editing tests pass, including bounds output
reuse and transform reuse across successive presentations. GC settings remain
unchanged. Captures: `/tmp/materia-clip-measure`; census:
`/tmp/materia-clip-census`.

The follow-up census completed successfully and contains no sampled allocation
stacks through either pose-transform helper or `transformedBounds`. Remaining
large sampled sources include UI path elements/styles, robot snapshots, native
pose-override records, and camera projection. Final scene editing tests pass.

### Shared sensor projection

Sensor overlay painting now prepares one projection matrix for all projected
links, mounts, and lidar hits. PerspectiveCamera exposes `projectWithMatrix`
so callers can explicitly share the projection without a mutable global cache.
Point projection uses scalar homogeneous coordinates instead of allocating
source and clip arrays. Existing `project` callers retain the same interface.

Matched 1600 × 1000 gantry captures measured median allocation
1,018,592 → 951,696 bytes/frame (6.6% less), p95 1,035,432 → 968,856.
FPS was 36.1 → 34.9 and median frame work 10.72 → 13.05 ms, so this pair
does not demonstrate a timing improvement. Application build and scene editing
tests pass, including a homogeneous matrix reference and behind-camera rejection.
GC settings are unchanged. Captures: `/tmp/materia-projection-measure`.
The standard HL launcher encountered a JIT error in Reflect.fields before
compilation; the supported Haxe --run tools.HaxeonCli fallback built successfully.

Arena storage was evaluated for native pose overrides: the renderer copies its
input, but the current imported-struct bindings lack a direct raw arena-backed
record publication path. A focused binding/lifetime change is needed before
replacing the working SceneView ownership with an arena-backed batch.

### Status reads without snapshot construction

RuntimeRobotAdapter.status previously constructed a complete RobotSnapshot,
including joint arrays and sensor frames, to inspect only endpoint and safety
flags. RobotRuntime.status now observes the supplied endpoint into the existing
locked native scratch buffer and reads those flags directly. Device sensor
polling remains in snapshot(), where consumers actually request observations.
No status cache, arenas, collector changes, or new native entrypoints are needed.

Runtime endpoint tests cover Ready, endpoint faults, safety faults, fault clearing,
absence of sensor polling on status reads, adapter delegation, and Disconnected
after adapter close. The targeted endpoint suite and application build pass.

Matched 1600 × 1000 gantry captures measured median allocation
950,744 → 909,628 bytes/frame (4.3% less), p95 967,232 → 926,784.
FPS was 37.0 → 32.1 and median frame work 11.29 → 12.26 ms, so this pair
supports an allocation reduction, not a timing improvement. Captures:
`/tmp/materia-status-measure`. Arena-backed pose publication remains a separate
binding/lifetime task rather than a prerequisite for removing this snapshot work.

### Reusable rectangle path encoding

Canvas now owns one scratch PathBuilder for solid and gradient rectangles.
PathBuilder.clear reuses its element storage, and append reuses native element
wrappers by slot. Path operations pass scalar coordinates instead of allocating
coordinate arrays; only coordinates consumed by each verb are written. Native
nkui_path_create copies the input into its own NanoVGPath, so later builder
reuse preserves previously created paths and retained display lists. Canvas
continues to retain each distinct path/paint through its existing update lifetime.

Matched gantry captures measured median allocation 911,792 → 861,992 bytes/frame
(5.5% less), p95 928,472 → 876,512. FPS was 26.0 → 28.4, but median custom
paint time was effectively unchanged (5.65 → 5.70 ms), so only the allocation
reduction is treated as established. Application build and the full Haxe UI
FrameworkSmoke suite pass, including the 4,000-node pressure and input routing
checks. The suite used its documented bundled font/image fixtures under Xvfb.
Collector settings remain unchanged. Captures: `/tmp/materia-path-measure`.

### Narrow joint reads for switch synthesis

RobotRuntime.physicalPositions now copies joint positions directly from the
locked endpoint scratch buffer instead of constructing a complete RobotSnapshot.
A new jointObservation returns owned counter coordinates and their source time;
the simulated switch adapter uses it instead of a full snapshot when synthesizing
switch edges. These reads do not trigger device sensor polling. Counter reads
still use observe(), while physical coordinates use observeEndpoint(), preserving
the distinction across coordinate zeros.

Endpoint tests cover values, source timestamps, independent returned arrays,
and absence of device sensor polling. The gantry homing/belt integration and
application build pass. This iteration did not produce a measurable gantry
allocation reduction: physical-read pair 861,720 → 861,984 median bytes/frame;
combined joint-read pair 861,104 → 861,000. No performance gain is claimed.
A fresh census shows large switch allocations in sensor publication, rather than
the narrowed joint reads; text override copy-on-write remains another large
source. GC settings remain unchanged. Captures: `/tmp/materia-positions-measure`,
`/tmp/materia-joints-measure`, `/tmp/materia-joints-census`.

### Cache explicit text colors during style resolution

Text now supplies its explicit color to style resolution instead of mutating a
cached computed style afterward. The resolver applies this local text override
before publishing its cache entry, after transitions to retain the previous
precedence, and includes RGBA in both the hash and collision-checked cache match.
Color is immutable. The existing bounded style cache and copy-on-write isolation
remain in place; provenance stays `local / text / -1 / local`.

Regression checks cover equal-valued colors sharing cached storage, different
colors and absent overrides resolving independently, local provenance, and
mutating one returned style without poisoning the cache. Application build and
full Haxe UI FrameworkSmoke pass.

Validated 127-component gantry captures at 1600 × 1000 measured median allocation
861,360 → 834,384 bytes/frame (3.1% less), p95 877,752 → 850,328.
Median frame work was flat (14.17 → 14.26 ms); no sustained FPS gain is claimed.
GC settings remain unchanged. Captures: `/tmp/materia-text-valid-measure`.
Earlier probes in `/tmp/materia-text-measure` are invalid: project fingerprinting
failed due to a missing native __sha256_raw symbol, leaving an empty scene.
Rebuilding the native runtime restored the symbol. The measurement helper now
rejects captures with fewer than 100 scene objects so that loading failures
cannot masquerade as gantry performance improvements.

### Share immutable sensor mounts and stamp native frames once

RobotRuntimeSensorBlueprint now owns a SensorMount whose immutable position and
rotation arrays are shared with native/external runtime SensorFrames. The legacy
SensorFrame constructor still copies raw mount arrays; an optional shared mount
provides the runtime's immutable path. Observation values and timestamps remain
per-frame. Tests verify shared identity, isolation from mutable authored/exported
arrays, and independent values and observation identities.

Native sensor frames now receive the endpoint source clock during fromNative(),
after the endpoint observation updates its epoch. RuntimeRobotAdapter therefore
no longer needs to recreate those frames solely to correct an unspecified clock.
External publishers retain their supplied clock domains. Clock observations
(36 assertions), endpoint/immutability tests, gantry homing/belt integration,
and application build pass. Collector settings are unchanged.

Mount sharing alone measured 836,240 → 832,600 median bytes/frame (0.4% less).
The final combined matched gantry pair measured 834,664 → 822,296 bytes/frame
(1.5% less), p95 850,720 → 839,056. Median frame work was 10.57 → 10.89 ms
and FPS 42.1 → 38.3, so no timing improvement is claimed. Captures and mount
census: `/tmp/materia-mount-measure`; final pair:
`/tmp/materia-sensor-final-measure`. Both helpers require the gantry to load.
The census confirms that remaining sensor allocations primarily construct the
per-observation values, while larger remaining sites include layout styles,
RenderNodes, pose overrides, and AssemblyRobot pose composition.

### Scalar assembly pose composition and value-type experiment

AssemblyRobot.compose now rotates offset coordinates and multiplies quaternions
using scalar locals, removing the temporary offset vector, rotated vector, and
quaternion array. Returned position/rotation arrays remain independently owned.
Scene editing tests compare 32 rotations/translations against sequential point
rotation, verify quaternion composition order, and check output independence.
Build and scene editing tests pass.

The matched 127-component gantry pair measured median allocation
823,576 → 793,456 bytes/frame (3.7% less), p95 840,600 → 807,064.
Custom paint time was essentially unchanged (6.80 → 6.85 ms); no sustained timing
gain is claimed. Captures: `/tmp/materia-compose-measure`.

An Insets @:value experiment built and passed the full UI FrameworkSmoke suite,
but the live gantry rejected a layout with "Layout alignment or positioning is
invalid". The annotation was reverted, rather than retaining a change that only
passed smoke coverage. The restored application built and completed a validated
gantry capture at 793,016 median bytes/frame. Artifacts:
`/tmp/materia-insets-measure` (failed experiment) and
`/tmp/materia-compose-restored-measure` (successful restoration).
The compiler contains scalar replacement and HashLink construct-in-place passes;
the older INLINING_AND_VALUE_TYPES overview does not reflect that implementation.
Value types remain candidates for focused measured work, but this UI layout
conversion needs a separate compiler/representation investigation. GC settings
remain unchanged.

### Insets escape investigation

Enriched layout validation reproduced the failure with valid alignment,
positioning, gaps, and z-index, but all four padding doubles contained invalid
memory contents. Capture: `/tmp/materia-insets-investigate-measure/after`.

The focused `app/tests/insets-escape` regression retains padding in an
Array<Dynamic>, releases its owning LayoutStyle, churns allocations, and forces
eight major collections. With Insets marked @:value it reads correctly before
collection, then SIGSEGVs after collection (reproduced twice). With ordinary
Insets the same test passes all eight rounds. This isolates an escaping inline
value lifetime failure, independently of the gantry and layout engine.

HashLink OToDyn stores the struct pointer in its dynamic box without copying
its data; packed field reads can return interior pointers into their owners.
Heap marking in vendor/hashlink/src/gc.c gc_flush_mark uses allocation-start
lookup, unlike stack marking's interior-pointer lookup. Thus a boxed interior
pointer cannot be relied on to retain its parent. A representation fix must
materialize owned storage when inline values escape (including immutable
values), with coverage beyond this one UI class. No GC implementation or
settings were changed in this investigation. Insets remains an ordinary class;
its default zero padding is already shared by LayoutStyle, so packing every
style also has a memory cost that would need measurement after correctness.

Run the regression with:
`./haxeon/scripts/haxeon build --compiler-only --project app/tests/insets-escape/haxeon.json --output /tmp/materia-insets-escape.hl`
and the normal application HashLink/native library environment.

### Value binding ownership fix

ValueCopy now gives immutable value-class bindings owned copies as well as
mutable ones, including nested value fields. ConversionResolver applies the
same binding rule before Dynamic boxing: HashLink boxes a pointer, so the
payload cannot borrow a parent object's inline slot. Fresh results keep the
existing no-extra-copy rule; scalar replacement, construction in place and
read-only argument copy elision remain responsible for removing safe copies.
No GC code or configuration changed.

The new compiler fixture `value-immutable-escape.hx` retains values through
Dynamic boxes, typed arrays, function arguments/returns, captures, maps,
nested values and nullable boxes. It replaces the source inline slot and
forces eight major collections under allocation churn. It returns 42 on
HashLink, Wasm32 and Wasm GC. All nine value-* HashLink fixtures pass, as do
TestMain (51 PASS groups), InlinerMain, the focused Insets escape regression
with @:value enabled, and the full UI FrameworkSmoke suite. The general test
driver encountered an unrelated unmanifested Sha256Benchmark.hx fixture;
the nine value fixtures were compiled and executed directly instead.

The gantry now completes captures with @:value Insets. A controlled pair with
the fixed compiler measured ordinary Insets at median 801,180 bytes/frame
(p95 816,808), and packed Insets at 814,592 (p95 830,776): 1.7% more allocation.
Both captures loaded 127 components and used 1600x1000 windows. Timing remains
variable, so no FPS improvement is claimed. Insets therefore remains an
ordinary class; the compiler correctness fix is retained independently.
The normal application was rebuilt after removing the experimental annotation.
Captures: `/tmp/materia-insets-owned-controlled-measure`; initial restored
value capture: `/tmp/materia-insets-owned-measure`.

### Prepared assembly geometry-centre frames

The baseline allocation census sampled 30 allocations in
ApplicationSimulation.rotateOffset during presentation. Each assembly part
previously composed its authored pose, rotated the geometry centre, and then
allocated a second position array for the centred pose, plus a temporary input
pose wrapper. The centre and the offset on the link are fixed for the simulation
configuration, so ApplicationSimulation now composes that geometry-centred
local frame once while preparing the candidate rebuild. The active frame data
is replaced only after a successful rebuild and follows the existing lifecycle.
AssemblyRobot.composeComponents accepts the position and rotation arrays
without an input pose wrapper; compose retains its existing API and delegates
to the same scalar implementation. Published pose arrays remain freshly owned.

Scene editing tests compare 32 parent/offset rotations and translations against
the original world-space centre calculation. The live gantry test retains a
snapshot across 100 simulation ticks and verifies distinct arrays in the next
snapshot and unchanged old positions/quaternions, alongside homing, belt cache
and reset checks. App build and both suites pass.

The matched 127-component gantry pair at 1600x1000 measured median allocation
801,348 -> 781,112 bytes/frame (2.5% less), p95 816,312 -> 796,624.
Median frame work was 13.19 -> 13.32 ms and custom paint 5.30 -> 5.26 ms;
no timing or sustained FPS improvement is claimed. The after census sampled
zero ApplicationSimulation.rotateOffset allocations during presentation.
Baseline census: `/tmp/materia-sim-pose-census`; paired captures and after
census: `/tmp/materia-sim-pose-measure`. GC code and settings were unchanged.

### Pointer-free HXI struct storage

The remaining allocation census identified SceneView.replacePoses marshalling:
numeric node/transform/pose-override records constructed per-record root arrays
and wrapper views, and packing records allocated another array of empty root
slots. HxiHaxeEmitter now reuses its recursive pointer-free layout proof to
omit that bookkeeping for constructors, attachments, packed struct arrays,
and nested/fixed-array field access. These records still own managed bytes;
getters/setters and packed arrays still copy their data. Pointer-bearing
records retain the existing root propagation and borrowed-buffer ownership.
No native runtime, GC implementation, or GC configuration was changed.

HxiParserMain and the existing native HxiCallMain/HxiRetainedMain tests pass.
The new registered runtime case HxiNumericStorageMain verifies zeroing,
aliases, nested numeric arrays, packed arrays, attachment, independent copies,
bounds errors, and a numeric child inside a pointer-bearing parent retaining a
borrowed byte buffer through four forced major collections. The test driver
passes this case; it also runs successfully as a Wasm32 fixture. A Wasm GC
attempt fails validation with "memory index 0 exceeds number of declared
memories"; recompiling the same fixture with the original generator reproduces
the failure, so this is a pre-existing backend limitation, not a passing GC
backend check. App build, SceneEditingTests and full UI FrameworkSmoke pass.
The transient duplicate-type error in the concurrent project-loader refactor
cleared without changes to that refactor by this optimization.

An isolated 10,000-record constructor benchmark, compiled with the original
emitter overlaid from git and with the new emitter, measured 1,844,856 ->
562,040 allocated bytes (69.5% less). Both variants pass the ownership fixture.
Artifacts: `/tmp/materia-pointer-free-bench` and
`/tmp/materia-pointer-free-baseline-src`.

The paired 127-component 1600x1000 gantry captures measured median allocation
780,768 -> 647,656 bytes/frame (17.0% less), p95 796,544 -> 661,864.
Median frame work was 13.79 -> 13.55 ms, render 8.19 -> 8.10 ms, and custom
paint 6.16 -> 6.37 ms. FPS varied 32.3 -> 36.4; no sustained FPS gain is
claimed. The after census no longer sampled root arrays in the pose-override
constructor. Captures: `/tmp/materia-pointer-free-measure`; fixture/backend
artifacts: `/tmp/materia-pointer-free-storage.hl` and
`/tmp/materia-pointer-free-wasm`.


### Allocation-free plain struct copies

The native `structCopy` primitive previously allocated a managed snapshot and
C byte buffer for every copy, including numeric records. When both buffers
have no owned UTF-8 fields, it now uses `memmove` directly after the existing
bounds checks. Overlapping buffers and borrowed views retain copy semantics;
records with owned strings keep the original snapshot/ownership path. This
changes byte copying only, without changing GC implementation or settings.

HxiNumericStorageMain now checks overlap in both directions (including a
borrowed view), empty buffers, source/destination bounds, and a 10,000-copy
allocation budget of 1 KiB. It passes alongside the existing HxiCallMain and
HxiRetainedMain ownership tests, SceneEditingTests, live gantry BeltDisplayTests,
and full UI FrameworkSmoke. The native runtime rebuild passes.

Using the same compiled app module before and after the native runtime rebuild,
the 127-component 1600x1000 gantry measured 647,656 -> 615,736 median allocated
bytes/frame (4.9% less), p95 661,304 -> 629,392. Median frame work measured
11.65 -> 11.15 ms and custom paint 4.95 -> 4.46 ms. FPS varied 34.7 -> 41.0;
this short pair does not establish a sustained FPS gain. A separate after census
measured 617,656 bytes/frame. Captures: `/tmp/materia-struct-copy-measure`;
updated correctness/allocation fixture: `/tmp/materia-struct-copy-storage.hl`.


### Direct typed pose-buffer publication

HXI now generates an owned `TBuffer` for pointer-free records, with checked
indices, immutable element count, scalar/nested-record setters, whole-record
copy/set, zero initialization, and `@struct_size` initialization. Counted borrowed
struct fields have a typed `_packed` setter which retains the storage and writes
the element count without repacking. This setter explicitly borrows mutable
storage; its lifetime/reuse contract is documented in C_HEADER_FFI.md.

SceneView.replacePoses writes nodes and world transforms directly into one such
buffer, removing per-pose managed records, the temporary record array, and the
final packing pass. Each publication owns a separate buffer so retained views
and picking stay independent of later publications and transform scratch reuse.
The generic buffers can be reused when readers have finished; the scene path
deliberately keeps one buffer per publication. Individual setPose edits lazily
materialize records, preserving the existing incremental-edit API. The upstream
node/transform input arrays remain; this change targets FFI packing.

App build and HxiParserMain pass. Updated HxiNumericStorageMain checks buffer
bounds/size limits, empty buffers, struct-size initialization, independent copies,
and a borrowed packed buffer retained through four major collections. Regenerated
HxiCallMain and HxiRetainedMain pass. Updated SceneEditingTests verifies bulk to
incremental edits, replacement, rejection of mismatched counts, clearing, and
retained-view picking after later pose publication. No GC code/settings changed.

An isolated alternating-order benchmark constructs 2,000 views with 128 poses
each, using the same compiler/runtime for the original record-array path and the
new packed path. The original allocates about 21.75 MB versus 0.53 MB packed
(97.6% less packing-path allocation). Median runtime across four rounds is
49.87 ms original versus 5.94 ms packed (about 8.4x faster). This is a packing
microbenchmark, not an application FPS measurement. Artifacts and exact results:
`/tmp/materia-packed-bench`.

The confirmation pair of 127-component 1600x1000 gantry captures measured
616,368 -> 605,824 median allocated bytes/frame (1.7% less), p95
629,280 -> 619,096. An earlier pair measured 617,084 -> 605,308 (1.9% less),
and the after census measured 607,144. Timing remains variable: confirmation
median frame work was 10.70 -> 14.07 ms and FPS 35.6 -> 30.3; the earlier pair
also overlapped compiler work. These captures establish an allocation reduction,
not an application frame-time/FPS improvement. Captures:
`/tmp/materia-packed-pose-measure` and `/tmp/materia-packed-pose-confirm`.


### HXI buffer code-size cleanup

The initial packed-buffer API emitted a full class and all its method bodies for
every pointer-free HXI struct. Buffer companions are now erased nominal abstracts
over a single haxe.io.StructBufferStorage class. Allocation, stride/count bounds,
and native-endian struct-size initialization live in that shared implementation.
Read-only named class-field forwarding preserves the existing buffer.length API
without granting writes or an implicit conversion to the storage class.

The compiler already lowers abstract instance methods on demand. Using that
existing path omits unused generated buffer bodies and per-buffer runtime class
metadata; no global DCE pass, new retention policy, GC changes, or runtime-native
changes were needed. The source declarations/signatures are still projected and
parsed, so this does not eliminate every frontend cost of a large interface.

A controlled app build used the saved original emitter via HAXEON_COMPILER_SOURCE
in a temporary symlinked compiler source tree, then rebuilt with the new emitter.
Both used current app sources and the same compiler core. The first attempted
classpath-only CLI overlay was not a valid baseline because the CLI launches a
separate compiler worker; that artifact is excluded from the comparison.
HashLink module size: 40,971,366 -> 39,512,512 bytes, a reduction of 1,458,854
bytes (3.56%). Function-map entries: 22,341 -> 19,243 (3,098 fewer). Entries whose
names contain Buffer. fell 3,142 -> 46; that broad count includes pre-existing
non-HXI buffers, so it is not a count solely of generated companions. The used
pose-buffer methods remain, while its unused whole-record set method disappears.
Artifacts: /tmp/materia-buffer-controlled-before.hl and app/build/host/main.hl;
baseline compiler source: /tmp/materia-buffer-cleanup-baseline-src.

Updated HxiNumericStorageMain asserts that unused BatchBuffer methods and
per-record buffer object types are absent from IR. A second compile adds the
previously unused buffer to the existing program; both resulting modules execute
with the expected exit 42. Bounds, struct-size initialization, independent copies,
and four-major-GC ownership checks still pass. HxiParserMain, IR assembly/readonly
forwarding tests, IncrementalSnapshotMain, regenerated HxiCallMain/HxiRetainedMain,
updated SceneEditingTests, and the app build pass.

An exploratory second compile which replaced the entire original test program
with a much smaller one crashed HashLink during module-index initialization.
The original emitter reproduces that crash; fresh/minimal incremental probes and
adding a buffer within the retained program pass. This pre-existing broader
incremental-program-replacement issue is left outside the buffer cleanup.
Reproducers: /tmp/materia-full-baseline-probe.hl.incremental.hl and
/tmp/materia-buffer-incremental-crash.txt.

Follow-up: the program-replacement crash is fixed in HlModuleAssembler. An
already-requested domain reload rediscovered a shorter native-import list but
kept interned object prototypes containing the previous absolute function slots
(StructBufferStorage.offset retained slot 222 instead of 190). Domain reloads
now discard published symbols and lowered bodies before rebuilding the module;
stable function identities remain in the function cache. The new
hxi-numeric-storage-reload runtime case compiles both revisions and executes the
replacement artifact with exit 42. Its method-slot assertion fails against the
previous assembler and passes with the fix. No GC changes were needed.

The isolated packing benchmark retains the same ~0.53 MB allocation per 2,000
packed views with both representations (versus ~21.75 MB for the original record
array path). Packing timings vary between runs; no new speedup is claimed here.
Benchmark artifacts: /tmp/materia-packed-bench/cleanup-{before,after}*.
The valid paired gantry captures measured roughly unchanged median allocation:
605,808 -> 607,380 bytes/frame; median frame work 11.62 -> 12.18 ms and FPS
38.0 -> 36.5. These short captures do not establish a frame-time improvement.
Captures: /tmp/materia-buffer-cleanup-measure.

### Current gantry allocation and CPU profile after the reload fix

Three separate captures used the current app module, gantry-picker's 127
components, 1600x1000, default 4x MSAA, and GPU query diagnostics disabled.
Each capture ran for 12 seconds after project readiness; timing summaries use
the final eight seconds. Artifacts: `/tmp/materia-current-bottlenecks`.
The reproduction driver is `/tmp/materia-current-bottlenecks.py`; the separate
profile driver is `/tmp/materia-current-profile.py` (attach the profiler before
waiting for its diagnostic window, because diagnostics-wait blocks startup).

The unprofiled run measured 36.82 FPS, median frame work 12.38 ms (p95 29.15),
and median allocation 606,280 bytes/frame (p95 618,184). Median custom paint
was 5.13 ms, native UI rendering 1.48 ms, and UI submission 1.64 ms.
Median inter-frame interval was 31.87 ms, with 12.20 ms outside the measured
render callback. That interval includes event-loop/presentation waiting and
simulation work; it is not all CPU work. No GC collection time was recorded
inside these measured callbacks. Other development processes were active,
so these short captures are diagnostic rather than a controlled speedup claim.

The separate allocation census recorded 4,905,734 allocations and about 201 MiB
over its final-eight-second window. It covers all threads and work outside the
render callback, so its total is not interchangeable with frame allocatedBytes.
Arrays account for 36.9% of exact bytes, raw bytes 10.6%, LayoutStyle 7.3%, and
RenderNode 6.8%. Grouping sampled allocating frames by function, UI-related
callers account for roughly 55%; individual hotspots are distributed, with
Button.buildScoped 3.8%, LayoutNode.new 3.3%, AssemblyRobot.composeComponents
3.2%, and SimulatedSwitchSensorAdapter.afterSimulationStep 3.0%.

The hlprof-live run used 100 Hz CPU samples and no allocation sampling. It
measured 25.91 FPS versus 36.82 unprofiled, so sampled shares must not be treated
as absolute costs. The steady report selects eight seconds ending at the last
sample inside UiHostRuntime.render, excluding final diagnostic serialization.
Among 371 render-callback samples, EditorPerspectiveViewport.paint accounts
for 41.8% inclusive, PackedTimingBeltMesh.update 18.6%, and
EditorScene.updateRuntimeGeometry 9.7%; inclusive shares can overlap.
Native driver calls may be attributed to their Haxe callers. Reports are in
`profile/steady-render-report.txt` and `profile/main-thread-summary.json`.

Next allocation target: Main.chromeRevisionKey includes simulation.appliedRevision
and the presentation revision. The top/status RetainedViews therefore invalidate
on simulation advancement even when their displayed controls and labels are
unchanged. Split their dependency keys around actual displayed state, including
mission status, command availability and error/pending state, and verify updates
remain correct. This targets avoidable UI construction without pooling retained
objects or weakening snapshot ownership. The separate CPU target is belt mesh
generation/publication; event-loop waiting also needs separate investigation
before attributing the remaining gap to 60 FPS to GC or allocation alone.

### Retain editor chrome across pose-only simulation updates

The top and status bars now have separate retention keys based on their displayed
labels and state. Shared label helpers drive both the key and the widgets, so
mission progress/failures, pending rebuilds, document labels, selection, backend,
world status, grid settings and mate diagnostics still update. The toolbar also
checks its commands' enabled/checked predicates, since those can change without
a command-registry refresh. RetainedView continues to handle viewport, style and
interaction changes. The perspective pane keeps its presentation-dependent key.
This reuses owned render subtrees; it introduces no pooling or GC changes.

SceneEditingTests checks actual subtree identity after simulation stepping and
checks start/pause transport availability, pending configuration, errors and
document-confirmation blocking/unblocking. The scene suite and app build pass.
A separate one-line explicit Array<Dynamic> annotation in exportArtifact fixes
an unrelated empty-array typing error encountered while building current sources.

The first attempt to reuse the older saved app as a baseline failed to load the
project under the current runtime (binary data rejected as a String); that capture
is excluded. The valid baseline was rebuilt from current sources in the isolated
`/tmp/materia-chrome-baseline-6kmuu5o6` source overlay with only the two RetainedView
callbacks restored to the old shared key. Both valid captures loaded 127 components
at 1600x1000 with 4x MSAA. Final-eight-second median allocation fell
618,792 -> 470,476 bytes/frame (23.97%); p95 fell 631,824 -> 484,368.
Median frame work was 11.89 -> 11.29 ms, but FPS was 36.20 -> 36.53: this pair
establishes an allocation reduction, not a meaningful throughput improvement.
Artifacts and reproduction driver: `/tmp/materia-chrome-measure` and
`/tmp/materia-measure-chrome.py`. Project loading is excluded from these summaries;
the baseline regenerated its artifact and the after run reused that artifact.

The follow-up census measured about 124 RenderNode/LayoutNode and 191 LayoutStyle
allocations per captured frame, versus about 188 and 295 in the earlier census.
These are separate all-thread diagnostic captures, not a matched timing pair;
they support the expected reduction in UI construction. Its median rendered-frame
allocation was 469,056 bytes. Raw census and report are under
`/tmp/materia-chrome-measure/census`.

### Simplify packed belt face emission

PackedTimingBeltMesh now uses the belt's planar extrusion structure when computing
face normals. The two XY faces need only the cross product's Z component and copy
a prepacked four-vertex unit normal. The two extrusion faces compute only XY normal
components and duplicate their packed normal with two copies. For a nondegenerate
segment, normal-buffer operations fall from 24 to 12 and square roots from four to
two. Four explicit position copies preserve the original winding without a
per-corner branch loop. The mesh retains a 96-byte normal template; no per-update
storage, native API, publication ownership or GC changes were introduced.

The reference mesh suite passes exact topology/index comparisons and tolerant
position/normal/bounds checks for both tooth sides, three routes, seam-adjacent
phases and both ordinary and near-degenerate thin extrusions. The full belt-display
suite passes paused reuse, selective moving-belt updates, reset and retained pose
ownership checks. The app build passes. Current project-export code also needed
an explicit Array<Dynamic> local for dynamicParts to resolve an unrelated empty
array typing error after its decodeProject call changed.

An isolated benchmark compiles both old and new mesh implementations into one
module, warms both, then alternates order across twelve rounds of 200 updates
each. On the 476-tooth, 38,096-vertex belt, median update time was
0.6456 -> 0.4227 ms (34.5% less); median within-round after/before ratio was
0.6590 (34.1% less). Two rounds had visible contention outliers. Sources, bytecode
and full results: `/tmp/materia-belt-face-bench`, especially
`BeltFacePairBench.hx` and `paired-results.json`. The original source snapshot is
`/tmp/materia-belt-mesh-before.hx`.

The 127-component, 1600x1000, 4x MSAA gantry pair measured 32.45 -> 39.70 FPS
and median frame work 13.45 -> 10.38 ms. Reversing capture order measured
38.13 FPS before versus 36.95 after, with median frame work 10.68 -> 9.96 ms.
These variable runs do not establish a repeatable throughput gain. Median frame
allocation stayed about 470 KB. The isolated mesh speedup is the supported CPU
result; the remaining frame pacing/rendering costs still prevent a 60 FPS claim.
Captures and drivers: `/tmp/materia-belt-face-measure`,
`/tmp/materia-belt-face-confirm`, `/tmp/materia-measure-belt-face.py`, and
`/tmp/materia-confirm-belt-face.py`. Loading is excluded from all timing summaries.

### Coalesce native instance-buffer uploads (2026-10-07)

SceneKit now retains packed instance records and uploads the enclosing dirty range
once per affected batch. Previously every changed instance called
`nkgpu_buffer_update`; NativeKit's persistent-buffer implementation copies its
shadow and recreates a buffer on subsequent updates in the same renderer frame.
The retained records preserve unchanged instances inside the uploaded range.
Transform revisions advance only after a successful upload. Pending update storage
is reused. The extra retained CPU storage is 80 bytes per rendered instance.

The native GPU regression covers two changed instances sharing a batch, a subsequent
unchanged execution, and two changed instances after delta-history overflow. Both
incremental and full reconciliation perform one upload for the shared batch.
The 50,000-node GPU benchmark reports 100 changed records across four batches as
four uploads (previously 100). All six `scene_render` CTest checks passed after
rebuilding their executables; the benchmark also asserts the upload count.

Controlled gantry captures used identical `main.hl`, 1600x1000, default 4x MSAA,
12 seconds per run, final eight seconds analyzed, order before/after/after/before.
Saved previous library: `/tmp/materia-instance-before`; capture artifacts:
`/tmp/materia-instance-measure`. FPS was 40.67/40.08/40.88/42.88, respectively.
Median frame work was 11.74/12.97/11.92/11.52 ms. Allocation remained about
470 KB/frame. This establishes the upload-count reduction, but does not demonstrate
a gantry FPS improvement; no 60-FPS claim is justified by these captures.

### Native vertex packing (2026-10-07)

The saved `hlprof-live` capture was recoverable after its shutdown timeout:
`/tmp/materia-viewport-next/profile/editor.hlpc`. Exporting it showed viewport
samples concentrated in native `renderImage`, including driver waits, rather than
camera clipping. The opt-in `NK_SCENE_GPU_TIMINGS` diagnostics now also report
`synchronizeMilliseconds`, `geometryMilliseconds` and `packingMilliseconds`.
These are nested timings, not additive; normal rendering does not read the clock
for these phases when diagnostics are disabled.

A diagnostic gantry capture (`/tmp/materia-packing-split`) measured 3.71 ms median
geometry preparation and 1.33 ms vertex validation/packing on frames with geometry
updates. Unchanged-geometry frames were much cheaper. Native vertex packing now
initializes defaults and positions in one pass instead of clearing the entire
output and revisiting it for positions. Matching float position, normal, UV and
color streams validate their byte range once and scatter fixed-size attributes
directly. Other formats retain the existing decoder. Strides, duplicate-stream
order and default attributes retain their existing semantics.

An isolated benchmark extracted the previous and current packers into the same
C++ executable, checked identical output, and alternated their order for 12 rounds
of 1,000 packs of 38,096 vertices with a float3 normal stream. Median packing time
was 0.23161 -> 0.08974 ms (61.3% less); median paired ratio was 0.3873. A separate
comparison with only the initialization pass combined measured another 12.2%
reduction from the float-stream path. Sources/results are in
`/tmp/materia-pack-bench-build.py`, `/tmp/materia-pack-bench.cpp`,
`/tmp/materia-pack-bench-v2.jsonl` and `/tmp/materia-pack-bench-init.jsonl`.

All six `scene_render` CTest checks passed. GPU regression checks additionally
compare tightly packed and padded normal-stream images, reject a truncated last
attribute, and successfully retry a corrected geometry revision. The application
native library was rebuilt; the Haxe bytecode was unchanged.

Final application comparison: `/tmp/materia-packing-v2-measure`, identical bytecode,
1600x1000, default 4x MSAA, 12-second captures, final eight seconds analyzed,
before/after/after/before order. FPS: 28.29/32.38/32.42/25.29. Median frame work:
10.92/11.15/10.95/11.58 ms. Allocation remained about 470 KB/frame. The earlier
float-stream-only comparison varied between 35 and 41 FPS, so the final pair's
higher throughput is observational and does not establish a stable FPS gain.
The repeatable isolated packing improvement is the supported CPU result; driver
synchronization and presentation cadence remain unresolved.

### Incremental GC gantry trial (2026-10-07)

Compared the current collector with `HL_GC_INCREMENTAL=1` using identical app
bytecode and native libraries, 1600x1000, default 4x MSAA, simulation enabled.
Four 40-second captures ran in full/incremental/incremental/full order; each
summary analyzes the final 30 seconds. No validation, latency tracing or GPU
queries ran during comparative captures. The current runtime reports
`incrementalSupported=1`. The desktop scheduler uses its existing shared 1 ms
frame allowance. The application default was not changed.

Artifacts and driver: `/tmp/materia-incremental-gc`,
`/tmp/materia-incremental-gc.py`. Frame summaries include p95, p99 and maximum
frame work, frame intervals, GC mark time, completed collections and allocation.
A separate 0.5-second `/proc/<owned-capture-pid>/status` sampler records process
RSS; timed-window peaks exclude capture serialization and shutdown.

| Run | FPS | Frame work p99 (ms) | Total GC time (ms / 30 s) | Max frame GC mark time (ms) | Timed peak process RSS (MiB) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Full | 32.20 | 39.01 | 20.93 | 4.57 | 626.4 |
| Incremental | 26.36 | 46.76 | 903.08 | 14.33 | 795.9 |
| Incremental repeat | 26.97 | 47.18 | 968.26 | 16.99 | 779.8 |
| Full repeat | 29.85 | 41.34 | 29.26 | 5.04 | 627.6 |

Full completed six/seven collections during the measured windows; incremental
completed three in each. Allocation stayed around 474–475 KB/frame. Incremental
slices appeared in most frames and increased total collector work substantially.
The ordinary collector's recorded GC time was under 0.1% of elapsed time in these
windows, so spreading its pauses is not currently a promising route to 60 FPS.
Desktop absolute FPS remains observational, but both orders favor full GC here.

A separate 12-second `HL_GC_LATENCY_TRACE=1` diagnostic run is stored in
`incremental-trace`; it is excluded from the comparison. Its last 200 phase records
spent substantial time capturing and rearming dirty-page tracking, with individual
slices exceeding the soft 1 ms allowance. Native incremental correctness guards
passed with independent reachability validation enabled, one and four GC workers
(`/tmp/materia-gc-guards-1.log`, `/tmp/materia-gc-guards-4.log`). The trial changes
no runtime code or default configuration. Keep incremental GC opt-in while
investigating tracking cost and convergence separately.

### Full geometry replacement without cloning prior snapshots (2026-10-07)

`nkscene_geometry_set_data` receives a complete replacement payload. It used to
call the copy-on-write edit helpers, deep-copying the previous vertex streams,
indices and subelement table immediately before replacing them. `GeometryResource`
now creates immutable snapshots from the new payload directly and advances its
resource revision once per replacement. Readers holding earlier scene snapshots
continue to own the old payload safely.

An isolated C-ABI benchmark used a 38,096-vertex geometry with 57,144 indices and
float3 position and normal streams. Across 48 batches in before/after/after/before
order, median `nkscene_geometry_set_data` time fell from 0.263 to 0.080 ms
(69.4%); p95 fell from 0.268 to 0.104 ms. This measures one native geometry
replacement, not whole-frame or FPS performance. Source and samples:
`/tmp/materia-scene-replace-bench.cpp` and
`/tmp/materia-scene-replace-measure/native-results.json`.

The stream ingestion path also stopped retaining a second copy of positions in
the auxiliary stream list. It validates the supplied position stream directly
into the canonical vertex array; render packing already starts from that array,
and keeps support for position streams built by other SceneKit paths. Against
the direct-replacement version, the same benchmark fell from 0.081 to 0.058 ms
median (27.9%); against the original copy-on-write path, that is about 78% less
time per update. Follow-up samples are in
`/tmp/materia-scene-position-measure/after-results.json`.

The application build passes. A four-run gantry comparison was discarded because
GTK delivered only four or five frames over each measured window (2-second frame
intervals); it cannot establish an end-to-end speedup. The next capture needs a
stable foreground frame clock before we can quantify the change in application
frame time.

### GTK frame-clock follow-up and full-buffer staging (2026-10-07)

Foreground captures confirmed that the two-second gaps above occurred while the
GL area was mapped, visible and realized. A standalone GTK drawing-area control
ticked at about 60 Hz; a GL-area control with a tick callback on the GL area
fell to about 0.5 Hz. Moving the callback to its top-level window restored about
30 Hz. The Linux backend now uses the window callback to queue GL-area renders,
removes that callback safely on destruction, and preserves requests across hiding
and showing a surface. Finishing a GTK frame also stops queuing another render:
GTK presents the framebuffer after its render callback returns.

The latest unprofiled gantry capture at 1600x1000 with 4x MSAA completed normally
at 31.88 FPS, with 33.1 ms median frame intervals and 6.56 ms median Haxe frame
work (14.35 ms p95). Artifact: `/tmp/materia-after-upload-clean`. These numbers
still do not establish 60 FPS or an FPS gain from the upload change below.
Other captures varied widely while desktop applications shared the GPU; the
MSAA comparisons did not establish a repeatable quality/performance result.

Full `nkgpu_buffer_update` replacements now upload directly from the caller's
complete data and copy it into the retained CPU mirror after success. This
removes an allocation and the redundant copy of the previous contents. Partial
updates retain their complete copy-and-patch upload, and failed replacements
leave the CPU mirror unchanged. An isolated CPU staging benchmark alternated
the old and new paths over 24 rounds of 500 updates of 1,828,608 bytes. Median
staging time fell from 0.1644 to 0.0728 ms (55.7%). This excludes GPU upload and
does not measure whole-frame throughput. Source and samples:
`/tmp/materia-buffer-staging-bench.cpp`,
`/tmp/materia-buffer-staging-results.txt`.

`NK_SCENE_GPU_TIMINGS=<jsonl-path>` now samples one render in 16 by default,
including its CPU preparation stages, and polls query results on later sampled
frames. `NK_SCENE_GPU_TIMING_INTERVAL=1` restores dense sampling; larger values
reduce driver calls. Each record includes its render ordinal as `frame`.
Sparse captures measured about 0.4 ms for the main scene GPU pass, but this
excludes GTK presentation. Compare neighboring unprofiled captures before
attributing frame-rate changes to rendering work. The application build and
post-change gantry captures passed; GTK presentation cadence remains the next
investigation.

### Flutter GTK/X11 comparison (2026-10-07)

Inspected Flutter revision `7c548a2a9ae5e086000de7bbf4d4cdb2fe6ca6d7`,
particularly `fl_view_renderer_opengl.cc`, `fl_opengl_frame.cc`,
`fl_view_renderer_subsurface.cc` and `fl_gl_fence.cc`. Sources were downloaded
under `/tmp/materia-flutter-reference`. The current session is X11 with GTK
3.24.41. Flutter's GTK OpenGL renderer uses a drawing-area widget, composites
frames separately, then queues GTK redraws through the main loop. Its X11 frame
handoff explicitly reads pixels into CPU memory and uploads a texture for
`gdk_cairo_draw_from_gl`. The Wayland subsurface implementation uses a different
presentation mechanism with shared textures and explicit synchronization;
that is not a direct replacement for the current X11 surface.

Six sequential four-second controls used the same 800x500 window and window
tick callback. These measure callback/render delivery, not physical display
presentation, and include desktop contention:

| Control | Renders/s | Median tick interval (ms) |
| --- | ---: | ---: |
| Cairo drawing area | 47.99 | 17.20 |
| GtkGLArea | 36.25 | 31.17 |
| Drawing area with GL texture handoff | 36.49 | 25.54 |
| Drawing area with CPU readback and texture upload | 32.22 | 32.95 |
| Cairo repeat | 51.99 | 16.85 |
| GtkGLArea repeat | 38.00 | 21.31 |

GTK reported a 16,668 us refresh interval, but all retained frame histories
had zero presentation timestamps. The drawing-area texture control uses
`gdk_cairo_draw_from_gl`; its internal fast path was not independently verified.
These controls establish no repeatable advantage from replacing GtkGLArea.
Source/results: `/tmp/materia-gtk-presentation-probe.c` and
`/tmp/materia-gtk-presentation-probe.jsonl`.

A separate application trace wrapped GTK tick/render callbacks, context changes,
GL composition and draw completion without changing application source. It
completed normally at 34.77 frames/s over its final 180 frames. Phase medians:
tick-to-render 0.302 ms, render callback 9.291 ms, GL composition 0.260 ms,
draw completion 0.092 ms, render-exit-to-after-paint 0.476 ms, and after-paint
to the next tick 13.883 ms. Medians of separate phases must not be summed as a
single representative frame. GL composition p95 was 7.121 ms; tick intervals
had 29.946 ms median and 48.104 ms p95. Repeated context activation inside a
render callback totaled 0.114 ms median across 2,736 calls. Instrumentation
adds overhead, so these are diagnostic phase timings, not an FPS comparison.

Artifacts: `/tmp/materia-flutter-comparison-app2`,
`/tmp/materia-gtk-phase-trace2.csv.2720897`,
`/tmp/materia-gtk-phase-trace.c`, `/tmp/materia-flutter-trace-run.py`.
The first trace was discarded because child processes inherited the logger
and opened the same output file; the retained trace uses separate PID files
and aggregates frequent context calls.

Conclusion: the observed cadence is variable, not a demonstrated hard 30 FPS
limit. Keep the current GL-area backend while investigating X11 compositor
throttling and frame-clock scheduling. A faster producer timer alone would
not prove smoother physical presentation. The earlier symbol-level GLX/EGL
swap interposer was also inconclusive: libepoxy's function-pointer dispatch
can bypass such wrappers, so absent intercepted swaps do not rule out a
presentation wait.

### X11 compositor acknowledgement experiment (2026-10-07)

GTK 3.24.41's `gdk_x11_window_end_frame` freezes the frame clock while
waiting for the compositor's `_NET_WM_FRAME_DRAWN` acknowledgement. Cinnamon
advertises both this protocol and `_NET_WM_FRAME_TIMINGS` in this session.
Source: https://github.com/GNOME/gtk/blob/3.24.41/gdk/x11/gdkwindow-x11.c

A temporary preload experiment calls `gdk_x11_window_set_frame_sync_enabled`
with FALSE on the application's top-level window at its first tick. This
bypasses GTK's acknowledgement wait, not the compositor's own presentation
synchronization. It changes neither desktop settings nor production code.
The following gantry captures ran sequentially at 1600x1000 with 4x MSAA,
with the same callback/phase instrumentation as the preceding experiment:

| Capture | Callback frames/s | Median frame work (ms) | Median after-paint to next tick (ms) |
| --- | ---: | ---: | ---: |
| Acknowledgement bypass | 34.38 | 9.03 | 3.35 |
| Normal synchronization | 17.42 | 11.38 | 27.78 |
| Acknowledgement bypass repeat | 43.61 | 9.87 | 4.53 |
| Normal synchronization repeat | 17.76 | 11.65 | see phase trace |

The bypass captures use the final 180 frames; the normal captures contain
fewer than 180 frames and use the complete capture, including startup.
The initial app capture overlapped standalone controls and is excluded.
Standalone results in `/tmp/materia-gtk-pacing-results.jsonl` also overlap
that initial capture and must not be used to quantify a speedup.
Desktop/GPU load varied; these measurements establish a scheduling lead,
not a controlled whole-application speedup or physical presentation rate.
All four retained application captures exited normally.

This narrows a substantial delay to GTK/compositor acknowledgement pacing.
A global default that bypasses acknowledgement is not justified: it would
remove backpressure without demonstrating that extra frames are displayed.
Keep normal synchronization while investigating compositor acknowledgement
latency and actual presentation feedback. The X11 event filter saw many
`_NET_WM_FRAME_TIMINGS` messages but few `_NET_WM_FRAME_DRAWN` messages;
filter placement relative to GTK's internal handling was not verified, so
this is not evidence that acknowledgements were absent.

Artifacts: `/tmp/materia-gtk-pacing-probe.c`,
`/tmp/materia-gtk-pacing-trace.c`, `/tmp/materia-gtk-pacing-trace.so`,
`/tmp/materia-gtk-pacing-run.py`, `/tmp/materia-gtk-pacing-summary.json`.
Application captures: `/tmp/materia-gtk-pacing-app-nosync`,
`/tmp/materia-gtk-pacing-app-sync`,
`/tmp/materia-gtk-pacing-app-nosync-repeat`,
`/tmp/materia-gtk-pacing-app-sync-repeat`.
Corresponding PID-suffixed phase traces:
`/tmp/materia-gtk-pacing-trace.csv.{2731002,2732408,2733200,2733850}`.

### Raw compositor feedback trace (2026-10-07)

Moved observation ahead of GTK's event filters by interposing `XNextEvent`,
and recorded all five 32-bit words of frame ClientMessages plus GTK's
`XSyncSetCounter` submissions. Matched complete 64-bit frame serials. This
corrects the previous event-filter ambiguity: acknowledgements are arriving;
GTK's internal filter consumes `_NET_WM_FRAME_DRAWN` before our old filter.
The installed compositor packages are Cinnamon 6.2.9 and Muffin 6.2.0.

Sequential normal/bypass/normal gantry captures exited normally:

| Capture | Callback FPS | Drawn/timing messages | Submission to drawn receipt median (ms) | Compositor stamp to receipt median (ms) |
| --- | ---: | ---: | ---: | ---: |
| Normal | 18.94 | 122 / 122 | 22.819 | 0.074 |
| Bypass | 35.62 | 225 / 225 | 26.645 | 4.789 |
| Normal repeat | 19.03 | 121 / 121 | 23.693 | 0.081 |

The corresponding median submission-to-compositor-stamp delays were 22.164,
18.765, and 21.961 ms. Normal after-paint-to-next-tick medians were 22.835
and 23.546 ms, versus 4.251 ms for bypass. Separate medians must not be
added or subtracted as a representative frame. The submission timestamp is
recorded immediately before `XSyncSetCounter`; it includes downstream X11
processing, compositor scheduling, and any rendering dependencies, rather
than isolating compositor CPU time. Server/compositor timestamps closely
match this host's monotonic clock in these normal captures.

All 468 `_NET_WM_FRAME_TIMINGS` messages had a zero presentation-time offset
and a 16,668 us refresh interval. Therefore they provide no physical
presentation timestamps. Muffin's 6.2.0 sender leaves this offset zero when
presentation time is unavailable (or cannot fit the protocol offset), and
GTK only populates presentation time when a nonzero offset is provided.
A drawn acknowledgement is not a scanout timestamp. GPU utilization was
62% in a separate one-shot NVIDIA query; it was not sampled throughout the
captures and cannot establish which process caused the contention.

Sources:
https://github.com/GNOME/gtk/blob/3.24.41/gdk/x11/gdkdisplay-x11.c
https://github.com/linuxmint/muffin/blob/6.2.0/src/compositor/meta-window-actor-x11.c

Conclusion: under these conditions, normal pacing spends substantial time
waiting for the compositor to generate its drawn acknowledgement; ordinary
client event delivery after that generation is fast at the median. Bypassing
the wait lets rendering overlap outstanding acknowledgements, but increases
client-side acknowledgement delivery latency and supplies no proof of
better physical presentation. Retain synchronization in production. The
next measurement should profile compositor scheduling/render dependencies,
or use a presentation path with trustworthy presentation feedback.

Artifacts: `/tmp/materia-gtk-feedback-trace.c`,
`/tmp/materia-gtk-feedback-trace.so`, `/tmp/materia-gtk-feedback-run.py`,
`/tmp/materia-gtk-feedback-analyze.py`,
`/tmp/materia-gtk-feedback-summary.jsonl` (three pretty-printed JSON objects).
Captures: `/tmp/materia-gtk-feedback-app-normal`,
`/tmp/materia-gtk-feedback-app-bypass`,
`/tmp/materia-gtk-feedback-app-normal-repeat`.
Phase/message traces: `/tmp/materia-gtk-feedback-trace.csv.{2738937,2739527,2740322}`.
No production runtime changes were made for this experiment.

### Compositor CPU/GPU profile and desktop load (2026-10-07)

The user enabled `kernel.perf_event_paranoid=-1` through pkexec and requested
that it remain enabled. Direct attachment to root-owned Xorg still failed;
system-wide `cpu-clock:u` sampling succeeded without further settings changes.
The valid system profile is `/tmp/materia-compositor-profile4/perf.data`.
It used 99 Hz samples with DWARF call stacks over 20 seconds; restrict reports
to capture bounds 1352973.048498--1352979.232173 monotonic seconds. Xorg maps
remained unresolved, so its samples cannot identify an internal hotspot.
Use `--no-inline --no-call-graph --percentage relative` for inexpensive reports;
inline resolution encountered an unreadable local debug-cache entry.

During the 6.18-second application frame interval, Cinnamon's main-thread
scheduler accounting accumulated 878.5 ms CPU runtime and 245.5 ms runqueue
wait. At 5 ms polling intervals, 720 of 1,121 observations found it in poll,
137 in nanosleep, 52 in NVIDIA read/write lock waits, and 208 running or with
no resolved wait channel. These observations do not measure exact wait
durations or identify the code that requested each sleep. They suggest that
main-thread CPU saturation alone does not explain the acknowledgement gap.
The gantry capture reached 13.57 callback FPS; submission-to-compositor-drawn
stamp was 34.104 ms median, and message receipt followed that stamp by
0.111 ms median. Profiling overhead and other workloads prevent comparison
with earlier unprofiled FPS.

Concurrent NVIDIA process samples showed substantial activity from Discord,
regular Chrome, Xorg, and Cinnamon. This is shared GPU activity, not proof of
saturation or a specific driver bottleneck. Per-process percentages are
sampled utilization estimates and must not be summed as additive GPU shares.

A separate headless Chrome GPU process, PID 2677980, belongs to an Exosuit
web run (`--headless=new --use-angle=swiftshader-webgl`, profile directory
`/tmp/exosuit-current-web-chrome-new`). In the system profile its 6K CPU
samples were 76.80% SwiftShader JIT and 16.44% `libvk_swiftshader.so`.
A subsequent one-second process CPU measurement found it using about
1,013% CPU on this 20-logical-CPU host. This is software rendering CPU work;
it must not be confused with the regular Chrome process's NVIDIA GPU activity.
No unrelated workload was stopped or changed during these measurements.

A subsequent 800x500 / 1600x1000 / 1600x1000 / 800x500 sequence reached
6.08 / 5.53 / 4.40 / 4.52 callback FPS, respectively. All captures exited
normally, but load changed throughout the sequence, so it does not establish
a pixel-area optimization. Keep production frame synchronization unchanged.
A paired comparison with the identified external CPU workload temporarily
paused would help isolate its contribution; this requires the user's approval
because the process belongs to a separate workload.

Artifacts: `/tmp/materia-compositor-profile2` (Cinnamon-only task-clock trace),
`/tmp/materia-compositor-profile4` (system-wide CPU, scheduler, NVIDIA process
and total GPU logs), `/tmp/materia-compositor-profile4.py`,
`/tmp/materia-compositor-scheduler.py`, `/tmp/materia-gtk-area-pairs`,
`/tmp/materia-gtk-area-pairs.py`, `/tmp/materia-gtk-area-run.py`.
Application profiles: `/tmp/materia-gtk-feedback-app-normal-compositor-profile2`,
`/tmp/materia-gtk-feedback-app-normal-compositor-profile4`,
`/tmp/materia-gtk-feedback-app-normal-area-{0,1,2,3}`.
No production runtime changes were made during this profiling work.

### Exosuit workload termination follow-up (2026-10-07)

With explicit user authorization, terminated Exosuit's headless Chrome GPU
process 2677980. Chrome immediately respawned it as 2780598, so terminated
its verified dedicated headless browser instance 2677935 to stop the workload.
Verified that no Chrome processes retained that instance's profile directory.
Regular Chrome and other applications were left running.

The first transitional capture is not a clean before/after comparison. A fresh
post-termination capture exited normally at 17.26 callback FPS, with 15.85 ms
median Haxe frame work, 9.18 ms custom paint, and 1.98 ms native scene render.
Submission-to-compositor-drawn stamp remained 26.444 ms median; compositor
stamp-to-receipt was 0.051 ms, and after-paint-to-next-tick was 28.552 ms.
All 108 timing messages still lacked presentation timestamps. Removing the
identified software-rendering workload did not establish a solution to the
GTK/compositor pacing delay. Other desktop activity and changed frame costs
prevent a causal FPS comparison with earlier captures.

Artifacts: `/tmp/materia-gtk-feedback-app-normal-after-exosuit-browser-stop`,
`/tmp/materia-gtk-feedback-trace.csv.2782064`.

### X11 flush, epoxy dispatch, and viewport paint follow-up (2026-10-07)

A temporary Xlib trace measured a median 1.443 ms from the GTK end-frame
counter update to the next flush-capable Xlib call (p95 2.731 ms). Calling
`XFlush` immediately after every counter update reduced that interval to
0.002 ms, but sequential gantry captures reached 17.12 and 18.16 callback FPS,
respectively. This does not establish a speedup or justify a production flush;
XPending is flush-capable only under its normal queue conditions, and the
experiment included both start- and end-frame counter updates.

The corrected GL trace wraps libepoxy's exported dispatch pointers and preserves
their lazy resolver updates. It observed 138 `glXSwapBuffers` calls in a gantry
capture: 0.043 ms median, 0.325 ms p95, 11.900 ms maximum. This establishes that
swap calls occur, correcting the earlier native-symbol interposer blind spot,
and shows no sustained swap-call CPU stall in this capture. It does not time
asynchronous GPU completion or physical presentation. GTK texture composition
still had a 20.336 ms p95 (1.999 ms median).

A separate 100 Hz, allocation-disabled, eight-second hlprof-live capture exited
normally. Of 252 samples within `UiHostRuntime.render`, 27.0% were in
`__sched_yield` beneath `EditorPerspectiveViewport.paint`. That points to a
native wait in the paint path but does not identify its precise API or prove
CPU arithmetic is the cause. The capture's median Haxe frame work was 19.10 ms
and custom paint 9.73 ms; instrumentation and desktop load affect these numbers.
The follow-up NativeKit-only sample below isolates the presentation baseline
before further changes to the application architecture.

Artifacts: `/tmp/materia-gtk-flush-{trace.c,trace.so,run.py}`,
`/tmp/materia-gtk-epoxy-{trace.c,trace.so,run.py}`,
`/tmp/materia-gtk-feedback-app-normal-{flush-baseline,flush-immediate,epoxy-dispatch,paint-hlprof}`.
The last directory contains `editor.hlpc` and `editor.perfetto.json`.

### NativeKit-only frame-pacing baseline (2026-10-07)

Added `modules/gpu/examples/frame_pacing.cpp` to NativeKit as the
`nativekit_gpu_frame_pacing` CMake example target. It uses only C++ and NativeKit's
public window, surface callback, event, and GPU APIs, with no Haxe, UI framework,
simulation, producer timer, or readback. Modes: clear a window frame; draw a static
triangle; draw that triangle into a retained offscreen texture and composite it
with a fullscreen quad. The composite mode is a minimal two-pass handoff, not
the complete application's SceneKit/UI integration. It uses 1x sampling.

Built Release from current NativeKit sources under
`/tmp/nativekit-frame-pacing-build`, with GPU/examples enabled and tests disabled.
The build passed, as did a warnings-enabled syntax check. All three modes ran
normally on the real X11 desktop; a separate screenshot confirmed the offscreen
texture/triangle rendered correctly. The sample documents its CLI in NativeKit's
GPU README, writes per-frame CSV only after rendering stops, and prints JSON
callback/CPU timing statistics. It excludes renderer initialization and a
configurable warmup (one second for these measurements).

Six foreground runs used 1600x1000, six measured seconds, normal synchronization,
and clear/triangle/composite/composite/triangle/clear order:

| Mode | Callback FPS, first / repeat | Median CPU render duration (ms), first / repeat |
| --- | ---: | ---: |
| Clear | 31.726 / 33.332 | 0.065 / 0.066 |
| Triangle | 30.321 / 32.509 | 0.076 / 0.076 |
| Offscreen composite | 31.310 / 32.327 | 0.111 / 0.108 |

Separate feedback traces measured 35.385 / 36.353 / 32.441 callback FPS for
clear / triangle / composite. Median submission-to-drawn-acknowledgement receipt
was 23.351 / 22.267 / 26.892 ms; median GTK texture composition was only
0.167 / 0.170 / 0.161 ms, with p95 around 8--10 ms. Trace summaries include
warmup and final callbacks; the sample's JSON/CSV statistics exclude warmup.
A diagnostic clear-only run with GTK acknowledgement waiting bypassed reached
55.440 callback FPS with 0.072 ms median CPU render duration. Its GTK composition
p95 rose to 16.838 ms and end-draw p95 to 13.046 ms, illustrating that disabling
backpressure can move stalls elsewhere. No production synchronization changed.

All 1,097 retained feedback timing messages lacked a nonzero presentation-time
offset, so none of these measurements establish physical presentation FPS.
Desktop load varied between runs, and this sample is independently built from
NativeKit sources rather than a replacement for the existing application binary.
The result nevertheless reproduces the pacing limitation without Haxe, layout,
simulation, GC, or substantial GPU submission work. Investigate the NativeKit /
GTK / X11 compositor path using this baseline before attributing that limitation
to application allocations or replacing the application rendering architecture.

Source: `haxeon/vendor/nativekit/modules/gpu/examples/frame_pacing.cpp`.
Usage: `haxeon/vendor/nativekit/modules/gpu/README.md`, minimal frame-pacing section.
Artifacts: `/tmp/nativekit-frame-pacing-results` (six plain captures, four traced
captures, phase/feedback summaries, and `composite.png`),
`/tmp/nativekit-frame-pacing-measure.py`, `/tmp/nativekit-frame-pacing-trace.py`.
Raw feedback traces: `/tmp/nativekit-pacing-feedback.csv.{2818211,2818540,2818781,2819033}`.

### Separate application swap vsync from GTK acknowledgement pacing (2026-10-07)

Used the native clear-only sample to test application GLX swap synchronization
independently from GTK's compositor acknowledgement wait. The display is one
3840x2160 output at 60 Hz; OpenGL uses NVIDIA GTX 1060 / driver 550.144.03.
Read-only NVIDIA settings report global SyncToVBlank=1, but that is not the
actual per-drawable interval. The presentation drawable queried through
`glXQueryDrawable(GLX_SWAP_INTERVAL_EXT)` reported **0** in every normal run.
GTK's composited-screen flag was true. GTK 3.24.41 explicitly selects interval
zero for its attached GL context when a compositor is active, leaving final
vblank synchronization to the compositor:
https://github.com/GNOME/gtk/blob/3.24.41/gdk/x11/gdkglcontext-x11.c

Eight sequential foreground clear-only runs used 1600x1000, a one-second
warmup, and five measured seconds. A temporary dispatch hook timed actual
`glXSwapBuffers` calls and queried the interval at the first swap (during
warmup). Forced intervals use `glXSwapIntervalEXT` on that exact drawable;
queries verified the requested 0 or 1. A separate NVIDIA environment control
used process-local `__GL_SYNC_TO_VBLANK=0`, documented for this driver:
https://download.nvidia.com/XFree86/Linux-x86_64/550.144.03/README/openglenvvariables.html

| Condition | Actual swap interval | Callback FPS | Swap median / p95 (ms) | After-paint to next tick median (ms) |
| --- | ---: | ---: | ---: | ---: |
| Normal | 0 | 35.454 | 0.039 / 0.067 | 22.669 |
| Force interval 0 | 0 | 34.034 | 0.038 / 0.079 | 24.972 |
| Bypass GTK acknowledgement wait | 0 | 55.215 | 0.042 / 7.186 | 9.134 |
| Bypass acknowledgement + force interval 0 | 0 | 55.973 | 0.042 / 10.730 | 10.887 |
| Force interval 1 | 1 | 31.820 | 0.046 / 0.238 | 25.657 |
| Bypass acknowledgement + force interval 1 | 1 | 48.984 | 6.200 / 26.231 | 0.180 |
| NVIDIA process-local vblank synchronization off | 0 | 33.932 | 0.038 / 0.063 | 23.737 |
| Normal repeat | 0 | 34.500 | 0.037 / 0.099 | 23.662 |

CPU rendering remained about 0.07 ms median. Phase statistics above are
restricted to the measured sample CSV timestamp window; separate medians must
not be added as a representative frame. Normal submission-to-acknowledgement
receipt was 22.641 ms median (23.642 ms on repeat). Normal swaps had occasional
11--12 ms outliers despite very small median/p95 values. Forcing interval 1
while bypassing acknowledgement waiting produced a sustained swap wait,
serving as a positive control for both the override and the timing hook.

Conclusion: these measurements do not support a normal-path double-vsync
wait (application swap plus compositor). Application swap synchronization is
already off, and explicitly turning it off again did not improve cadence.
GTK's acknowledgement wait remains the dominant scheduling lead. This does
not rule out the compositor's own vsync scheduling or driver dependencies;
those were not disabled. All 1,640 retained timing messages lacked presentation
timestamps, so 55--56 callback FPS still does not prove physical displayed FPS.
Desktop load varied, and runs are short. Retain production synchronization.
No NVIDIA desktop configuration, compositor setting, or runtime source changed.

All eight retained captures exited normally. Earlier discarded probe attempts
crashed from recursion through unresolved libepoxy dispatch pointers in the
temporary observer. Debugging isolated that observer failure; the retained
observer invokes driver entry points resolved through `glXGetProcAddressARB`
and guards hook installation. These were instrumented sample failures, not
crashes in the unmodified NativeKit sample or Materia.

Artifacts: `/tmp/nativekit-vsync-results/results.jsonl`,
`/tmp/nativekit-vsync-results/analysis.json`, per-condition CSV/log files in
that directory, `/tmp/nativekit-vsync-measure.py`,
`/tmp/nativekit-vsync-analyze.py`, `/tmp/nativekit-vsync-trace.c`,
`/tmp/nativekit-vsync-trace.so`.
Retained raw trace PIDs: 2832134, 2832515, 2832737, 2833047, 2833244,
2833454, 2833909, and 2834102, under `/tmp/nativekit-vsync-trace.csv.<PID>`.

### GLFW X11 control against the GTK baseline

A temporary GLFW 3.4 control was built from the pinned release under
`/tmp/nativekit-glfw-control`, without installing a system dependency or changing
NativeKit's backend. It creates an X11 OpenGL 3.3 core window at 1600×1000,
clears it continuously, polls events, and swaps buffers. Each foreground run
uses one second of warmup and about six seconds of measurement. Runs alternate
GTK, GLFW interval 1, GLFW interval 0, then repeat in the same order.

| Path | First run FPS | Repeat FPS |
| --- | ---: | ---: |
| NativeKit GTK clear | 37.670 | 33.724 |
| GLFW, requested swap interval 1 | 55.824 | 55.766 |
| GLFW, requested swap interval 0 | 698.111 | 711.979 |

All six processes exited normally. GLFW interval-1 swap time was 16.738/16.396 ms
median; interval-0 swap time was 0.056/0.058 ms median. The interval-0 numbers
measure submission-loop throughput, not visible frames. None of these measurements
prove physical presentation FPS, and the GLFW control does not query the effective
swap interval. The observed blocking is consistent with the requested interval.

This strengthens the GTK/compositor frame-acknowledgement scheduling lead: the
same desktop, driver, window dimensions, and trivial clear workload permit much
higher loop throughput through GLFW. It does not isolate GTK alone: GLFW uses a
direct GLX swap path, while NativeKit uses Sokol and GtkGLArea's offscreen/composite
path. It also does not measure the full Materia renderer. A permanent backend
change needs a same-renderer comparison and integration assessment first.

Reproduction source, CMake build, measurement script, and raw results are under
`/tmp/nativekit-glfw-control` (`control.cpp`, `CMakeLists.txt`, `measure.py`, and
`results.jsonl`). These temporary artifacts are not persistent repository assets.

### Isolated GTK source build and internal frame-clock tracing

Built upstream GTK 3.24.41 from
`https://download.gnome.org/sources/gtk+/3.24/gtk+-3.24.41.tar.xz`
(SHA256 `47da61487af3087a94bc49296fd025ca0bc02f96ef06c556e7c8988bd651b6fa`)
with Meson `debugoptimized` (optimization plus DWARF debug symbols).
The installed package is `3.24.41-4ubuntu1.3`, so this matches the upstream
version, without claiming to reproduce Ubuntu's patches or build configuration.
System GTK was not replaced and no system packages were installed.

Build and source artifacts live under `/tmp/gtk-source-debug`. Configuration:

```sh
meson setup /tmp/gtk-source-debug/build /tmp/gtk-source-debug/gtk+-3.24.41 \
  --prefix=/tmp/gtk-source-debug/install --buildtype=debugoptimized \
  -Dwayland_backend=true -Dintrospection=false -Dtests=false \
  -Ddemos=false -Dexamples=false -Dprint_backends=file -Dgtk_doc=false -Dman=false
ninja -C /tmp/gtk-source-debug/build -j6 \
  gdk/libgdk-3.so.0.2409.32 gtk/libgtk-3.so.0.2409.32
```

Both X11 and Wayland symbols are needed because NativeKit links WebKit, even
when testing X11. The first X11-only build could not start the sample due to a
missing Wayland symbol. These failed startup probes are excluded. Initial
successful source runs also lacked the optional XApp desktop module; the retained
comparison supplies `GTK_PATH=/usr/lib/x86_64-linux-gnu/gtk-3.0` to all conditions.
Process maps confirm the source GTK/GDK libraries and the system XApp module loaded.

The unmodified libraries were copied to `baseline/` before applying
[the diagnostic patch](gtk-source-frame-pacing.patch) to the isolated source.
This patch adds buffered CSV events for frame-clock freeze/thaw, scheduling,
paint entry, X11 frame completion, and acknowledgement processing. It changes
no scheduling decisions. `NK_GTK_TRACE=/tmp/gtk-source-debug/trace-N` enables
logging, producing separate `-clock.csv`, `-window.csv`, and `-display.csv` files.
The patch applies cleanly to the release archive and both builds compile.

Retained clear-only foreground runs use the same NativeKit binary at 1600×1000,
with one-second warmup and six seconds of measurement, alternating all three
conditions twice:

| GTK condition | First callback FPS | Repeat callback FPS |
| --- | ---: | ---: |
| Installed GTK | 36.495 | 35.380 |
| Unmodified source GTK | 35.579 | 37.607 |
| Instrumented source GTK | 28.334 | 38.953 |

All six retained runs exited normally. Desktop variability and one slower traced
run prevent a precise claim about logging overhead or small throughput differences.
The unmodified source build reproduces the low baseline.

| Internal trace metric | First median / p95 (ms) | Repeat median / p95 (ms) |
| --- | ---: | ---: |
| Frame end → matching acknowledgement | 21.904 / 49.333 | 21.019 / 38.912 |
| Compositor timestamp → acknowledgement handling | 0.134 / 4.329 | 0.114 / 2.274 |
| Clock thaw → next paint entry | 0.0175 / 0.033 | 0.018 / 0.038 |
| Requested next-paint timer delay | 0 / 0 | 0 / 0 |

Trace analysis excludes the first second of trace events, which starts slightly
before the sample's measurement warmup. The two traces contain 180 and 244 matched
frame acknowledgements after this cutoff. Clock events are paired by clock pointer;
frame completion and acknowledgement are paired by protocol serial.

GTK is resuming promptly after the acknowledgement; there is no measured extra
refresh-sized delay in its post-ack scheduling. Most waiting happens before the
acknowledgement, and usually before the compositor's reported drawn timestamp.
The next investigation should follow Muffin's extended-sync-counter handling and
frame-drawn message production, including driver/GPU waits. This does not establish
that Muffin alone is defective, nor measure physical display presentation FPS.

For another traced run (or use these environment settings inside GDB):

```sh
env GDK_BACKEND=x11 GTK_PATH=/usr/lib/x86_64-linux-gnu/gtk-3.0 \
  LD_LIBRARY_PATH=/tmp/gtk-source-debug/build/gdk:/tmp/gtk-source-debug/build/gtk \
  NK_GTK_TRACE=/tmp/gtk-source-debug/manual \
  /tmp/nativekit-frame-pacing-build/modules/gpu/nativekit_gpu_frame_pacing \
  --mode clear --warmup 1 --seconds 6
```

Raw results, process maps, measurement script, analyzer, build logs, and JSON
analysis are in `/tmp/gtk-source-debug` (`results.json`, `maps-N.txt`, `measure.py`,
`analyze.py`, and `analysis.json`). Temporary build artifacts are not permanent;
the diagnostic source patch and this report are preserved in the repository.

### Live Muffin frame-counter and swap tracing

Profiled the existing Cinnamon process (PID 2066) without restarting it or
changing its presentation settings. Installed Muffin is `6.2.0+wilma`.
Upstream 6.2.0 source was downloaded to `/tmp/muffin-frame-debug/muffin-6.2.0`
for inspection. Installed debug symbols and disassembly, rather than assumed
upstream offsets, were used to locate probes in the running binaries:

| Library | Build ID |
| --- | --- |
| libmuffin | df79da3755c8c3e9b82564b5194e65f317f9ac18 |
| libmuffin-clutter | dfaadcdc17cbc11c5da632913fa795a53fd659f7 |
| libmuffin-cogl | b1f4a49eda751991a6be8bfbb496a3a707bb4b9e |

Temporary uprobes measured compositor frame requests, actor frame queuing,
acknowledgement send entry/return, Clutter clock dispatch, redraw entry/return,
and the actual indirect `glXSwapBuffers` call/return inside Cogl's GLX backend.
The latter probes are at file offsets `0x5b217` and `0x5b21a` in this exact Cogl
binary. DWARF confirms the called GLX renderer member at offset `0x60` is
`glXSwapBuffers`; disassembly confirms the instruction boundaries. These offsets
must not be reused against different binaries. MetaWindow serial and XID offsets
were likewise checked against installed DWARF (`0x1f0` and `0x50`).

The first `perf probe` attempt could not resolve an optimized-away counter-update
function and added no events. Raw uprobes at verified file offsets were then
enabled through pkexec. Captures use `perf --clockid mono` to align with GTK's
monotonic CSV trace. Frame requests are filtered by the sample's XID; actor
identity is established by the immediately following queue call; requests, sends,
and GTK acknowledgements are paired by protocol serial. All events are recorded
only from Cinnamon PID 2066. No other application was stopped.

An initial capture with concurrent 199 Hz DWARF CPU sampling reached 19.612 FPS,
so its timings are not treated as a representative performance baseline. It did
show NVIDIA driver and `sched_yield` stacks beneath Cogl's `glXSwapBuffers` call.
Two subsequent captures omit CPU sampling and retain only stage probes plus
buffered GTK tracing. Both use the foreground NativeKit clear-only sample at
1600×1000, one-second warmup, six-second measurement. All three samples exited
normally, and both lightweight recording helpers exited successfully.

| Lightweight capture | First | Repeat |
| --- | ---: | ---: |
| Callback FPS | 32.650 | 32.752 |
| Matched frames after trace warmup | 208 | 210 |
| GTK end → Muffin frame request, median / p95 ms | 1.875 / 23.114 | 1.356 / 21.246 |
| Muffin frame request → acknowledgement send, median / p95 ms | 18.097 / 29.557 | 18.170 / 31.798 |
| Acknowledgement send → GTK receipt, median / p95 ms | 0.124 / 3.362 | 0.142 / 3.972 |

The entire request-to-send interval can be split into five consecutive stages.
All matched frames have these stages in order:

| Stage, median / p95 ms | First | Repeat |
| --- | ---: | ---: |
| Frame request → next Clutter dispatch | 0.056 / 9.950 | 0.048 / 9.884 |
| Clutter dispatch → redraw entry | 0.434 / 16.484 | 0.431 / 11.304 |
| Redraw entry → GLX swap call | 2.482 / 11.744 | 1.803 / 11.500 |
| Actual glXSwapBuffers call | 8.447 / 23.827 | 10.120 / 24.998 |
| Swap return → acknowledgement send | 0.060 / 0.112 | 0.055 / 0.105 |

Do not add stage medians: they describe different frames. Stage means do add for
these complete pairs: 18.107 ms first and 18.371 ms repeat. Actual GLX swap time
accounts for 9.202 ms and 10.688 ms of those means (about 51% and 58%). This is
elapsed driver-call time, not measured GPU execution time or proof that every
wait is specifically vblank. Other compositor work, scheduler delays, and shared
GPU activity can contribute.

Every observed sample frame request already carries `skip_sync_delay=1`
(235/235 first and 229/229 repeat, including warmup). Muffin's source computes
this flag from the extended-sync-counter progression. The matching send occurs
just after the compositor's swap returns. This explains how a fast application
swap can coexist with long GTK acknowledgement waits: GTK depends on the
compositor's later frame cycle, whose own swap can block. The earlier finding
that the app does not wait for vsync twice remains valid.

These captures establish a compositor/driver contribution to the pacing limit;
they do not establish a generic GTK FPS cap, a Muffin defect, physical presentation
FPS, or a safe synchronization change. Moving acknowledgements earlier or disabling
compositor vsync requires separate correctness work. A same-renderer GLFW control
remains useful to evaluate a window path without this serial GTK acknowledgement
dependency. No production synchronization code was changed.

Artifacts under `/tmp/muffin-frame-debug`: initial `probes.txt`, `cpu.data`,
`cpu.txt`, `analysis.json`; lightweight `light/` and `light2/` contain `app.json`,
`probes.data`, `probes.txt`, GTK CSVs, `analysis.json`, `pairs.json`,
`stage-analysis.json`, and `stage-pairs.json`. Capture/analyzer scripts and the
exact probe definitions are at the directory root. These are temporary machine
artifacts. The final recording helper removed all nine `nkframe` probe definitions;
no recording or sample process remains. The previously authorized
`kernel.perf_event_paranoid=-1` setting remains enabled as requested.

### Live compositor swap configuration and immediate-update scheduling

Continued profiling Cinnamon PID 2066 without replacing GTK/Muffin, restarting
the desktop, changing swap settings, or calling functions inside the compositor.
Two captures in `/tmp/muffin-frame-debug/config` and `config2` extend the previous
verified uprobes with read-only renderer pointers, stage deadline/pending-swap
fields, manual-vblank path entry, and NVIDIA's exported `glXWaitVideoSyncSGI`
entry/return. The second adds Clutter's schedule-update entry arguments and
existing deadline. Both root recording helpers remove their probe definitions on
exit. No recording or sample process remains; the previously authorized perf
sysctl is left unchanged.

The NVIDIA GLX library build ID is
`b9608b23b0d015f37d7921edd33e7fd9d698cd3a` (driver 550.144.03). Stage field offsets
were verified from installed DWARF and disassembly: update time `0x38`, pending
swaps `0x2c`, next presentation `0x48`, last sync delay `0x54`. The scheduling
entry at `0xc6f80` checks the update timestamp against -1 and immediately returns
if an update is already scheduled. These offsets are specific to the recorded
binaries and cannot be reused without checking their build IDs.

A separate read-only GLX client queried Cinnamon's actual swap drawable,
`0x1c0000c`, observed at the swap-call probe. `glXQueryDrawable` reported:

- Width/height: 3840×2160.
- Swap interval: 1; maximum interval: 8.
- No X protocol errors.

This query was repeated with the same results. It creates no GL context, changes
no interval, and does not execute code inside Cinnamon. Live renderer pointers
show Cogl has a non-null swap-interval function and dispatches swaps through
libGLX. There were zero hits on Cogl's manual-vblank helper or NVIDIA's exported
video-sync wait in either capture. The stage's pending-swap field was zero in all
1,722 / 2,006 getter observations. Thus the earlier pending-swap gate exists in
source but was not observed to limit these captures.

| Metric | Configuration capture | Scheduling capture |
| --- | ---: | ---: |
| Clear-only callback FPS | 37.528 | 41.319 |
| Matched sample frames after trace warmup | 234 | 259 |
| Frame request → acknowledgement send, median / p95 ms | 17.309 / 30.224 | 17.430 / 30.365 |
| Actual compositor GLX swap, median / p95 ms | 11.237 / 26.514 | 11.589 / 25.258 |
| Draw before swap, median / p95 ms | 1.317 / 3.258 | 1.287 / 7.802 |
| Swap return → acknowledgement send, median ms | 0.057 | 0.056 |

Both samples and helpers exited normally. These are callback measurements on a
shared desktop with changing load, not physical presentation rates or a
controlled before/after optimization comparison. Interval 1 establishes that the
compositor requests synchronized swaps; elapsed swap-call time still cannot
separate vblank waiting from GPU backpressure or other driver dependencies.

The scheduling capture provides a concrete source-level lead:

1. Every matched sample frame requests `skip_sync_delay=1`.
2. Muffin calls `clutter_stage_skip_sync_delay()`, which requests a stage update
   with sync delay -1 (immediate).
3. `clutter_stage_cogl_schedule_update()` returns immediately when `update_time`
   is already set, preserving the old deadline instead of advancing it.
4. Of 259 matched immediate-update calls, 148 found an existing deadline;
   83 found a deadline still in the future. The remaining wait at entry was
   5.440 ms median, 8.236 ms p95, and 9.159 ms maximum.

Those 83 requests reached Clutter dispatch after 6.534 ms median. The 111 calls
that established a new immediate deadline reached it after 0.040 ms median.
This confirms that the immediate-update request can retain a future deadline.
It is a candidate scheduling defect, not yet a demonstrated FPS fix: median
request-to-acknowledgement-send was 17.537 ms for the future-deadline group and
18.371 ms for the new-immediate group. Their different refresh phases and driver
waits prevent causal comparison; advancing a deadline may simply move waiting
into the synchronized swap. Do not add the 5.440 ms scheduling median to the
11.589 ms swap median as a per-frame cost.

The relevant source is
`clutter/clutter/clutter-stage.c:clutter_stage_skip_sync_delay` and
`clutter/clutter/cogl/clutter-stage-cogl.c:clutter_stage_cogl_schedule_update`.
The immediate-return behaviour is also present in the unchanged master-clock/
scheduling path inspected in newer Muffin source. A future controlled experiment
should test advancing only an existing future deadline on an urgent request and
measure acknowledgement latency, swap blocking, CPU/GPU load, and presentation
correctness. No such mutation or production fix was made in this investigation.

Artifacts: `/tmp/muffin-frame-debug/config{,2}/probes.txt`, `probes.data`,
`app.json`, GTK CSVs, `analysis.json`, `stage-analysis.json`; `config2` also has
`scheduling-analysis.json` and `scheduling-pairs.json`. The read-only drawable
query is preserved as `/tmp/muffin-frame-debug/query-drawable.py`, with results
in `config/drawable-query.json`. Capture helpers and analyzers are at the parent
directory root. Temporary machine artifacts are not permanent repository assets.

### Experimental urgent-deadline patch and nested validation

Prepared [muffin-urgent-deadline.patch](muffin-urgent-deadline.patch) against
upstream Muffin 6.2.0. When an urgent schedule request finds an existing deadline
still in the future, it moves the deadline to the current monotonic time,
clears the obsolete predicted-presentation timestamp, and records the urgent sync
delay for subsequent rescheduling. Already-due deadlines and normal requests keep
the previous behaviour. This is an experimental compositor patch, not an installed
fix or NativeKit workaround.

Built Muffin with Meson `debugoptimized`, X11/GLX enabled, Wayland/native/remote
backends disabled, and introspection enabled. Missing development packages were
downloaded and extracted under `/tmp/muffin-frame-debug/deps`; no packages were
installed. Extracted pkg-config paths and library symlinks were adjusted to use
the private headers with installed runtime libraries. Meson dependency discovery
was refreshed after repairing the symlinks to avoid linking non-PIC static archives.
No distro patch directory exists in the downloaded upstream archive; this build
is not claimed to reproduce all installed distribution build settings.

The unmodified Clutter shared library was copied to `control-clutter/` before
applying the patch and rebuilding. The patched library is in `patched-clutter/`.
Both conditions use the same locally built Muffin executable, Cogl libraries,
and default plugin, selecting only the Clutter library through `LD_LIBRARY_PATH`.
Process maps confirm the correct control/patched library loaded for every run.
The patch applies cleanly to the original source and compiles successfully.

[muffin-deadline-check.py](muffin-deadline-check.py) exercises the actual compiled
private scheduler using a real ClutterStageCogl GObject. It derives the private
function address from the local library's symbols and uses field offsets verified
for this Muffin 6.2.0 layout; it must not be used blindly with other versions.
Five checks pass in both control and patched builds, with the expected difference
for urgent future deadlines: future urgent (-1), already-due urgent, future normal,
empty urgent, and another negative urgent delay (-2). This confirms advancement,
preservation, and presentation-timestamp/sync-delay bookkeeping.

Example for the patched checks:

```sh
LD_LIBRARY_PATH=/tmp/muffin-frame-debug/build/cogl/cogl:/tmp/muffin-frame-debug/build/cogl/cogl-pango:/tmp/muffin-frame-debug/build/cogl/cogl-path \
  python3 app/tests/performance/muffin-deadline-check.py \
  /tmp/muffin-frame-debug/patched-clutter/libmuffin-clutter-0.so.0 patched
```

Runtime validation uses an owned Xephyr X server on a free display (:80 during
these captures), with a private Xauthority file, no TCP listener or host input
grab, and a private D-Bus session per compositor. This runs standalone Muffin's
default plugin, not the full Cinnamon shell. The host Cinnamon PID 2066 was not
replaced, restarted, or modified. Every owned process is stopped by the harness.
One setup probe exited before running the app because the WM support window lacks
a PID property; the retained harness identifies its own Muffin child directly.

Important limitation: despite Xephyr's glamor option, nested GLX reports
**llvmpipe (LLVM 20.1.2), Accelerated: no**. The nested results do not reproduce
the real desktop's NVIDIA synchronized-swap path and cannot establish physical
presentation FPS, GPU usage, flicker/tearing correctness, or hardware performance.

Eight foreground 1600×1000 clear-only NativeKit runs use one-second warmup and
six-second measurement, with system-compatible source GTK tracing. Each scenario
runs control, patched, patched, control. The second scenario adds a visible
160×200 Xlib window producing spontaneous damage every approximately 17 ms.

| Scenario / build | Callback FPS, first / repeat | End→ack median ms, first / repeat | Muffin CPU %, first / repeat |
| --- | ---: | ---: | ---: |
| Single client / control | 15.928 / 18.312 | 27.526 / 23.834 | 61.6 / 72.2 |
| Single client / patched | 22.169 / 20.574 | 18.369 / 19.549 | 86.8 / 84.7 |
| Background damage / control | 20.171 / 17.358 | 24.019 / 26.965 | 79.5 / 71.5 |
| Background damage / patched | 19.414 / 18.361 | 21.768 / 22.243 | 78.0 / 74.8 |

CPU is compositor process user+system time divided by wall time across the sample
process lifetime, including warmup; 100% means one CPU core. End-to-ack statistics
exclude the first second of GTK trace events, which starts slightly before the
sample's measurement warmup. All eight samples exited normally, with no compositor
assertion or crash observed. Nested GLX/GTK/compositor costs outside the sample's
render callback also contribute to its frame interval.

The single-client patched runs are faster and consume more compositor CPU; with
background damage, throughput ranges overlap. These small shared-desktop captures
are insufficient to establish causality or a production FPS gain. They validate
the implementation and basic nested operation. A controlled separate login/session
on the real display is still needed to determine whether advancing the deadline
reduces acknowledgement latency or merely shifts waiting into the NVIDIA swap.
No system package, login configuration, or desktop session has been changed.

Artifacts: `/tmp/muffin-frame-debug/build`, `control-clutter`, `patched-clutter`,
`control-check.json`, `patched-check.json`, `compile.log`, `patch-build.log`, and
`nested/` (GLX renderer info, eight result rows, derived analysis, GTK CSVs,
compositor logs, and process maps). Reproduction harnesses are `nested.py`,
`damage.c`, `apply-deadline-patch.py`, and `check-deadline.py` at the parent root.
The reviewed patch, focused checks, and this report are preserved in the repository;
large build/capture artifacts remain temporary.
