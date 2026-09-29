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
