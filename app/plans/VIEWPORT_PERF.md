# Viewport performance: orbiting the flexspline hobber (2026-11-06)

**Symptom (user).** Orbiting the camera around the flexspline hobber in the editor is slow: low frame rate and lag.

**Initial evidence.**
- The hobber scene has about 255 assembly members, plus tessellated purchased parts: HGH blocks and rails, the eRob STEP, BK12
  STEP mounts, sealed scales with long cables, a 59.6 MB generated artifact, fine features such as threads and hob teeth, and
  derived holes. Editor resident memory is about 1.3 GB once loaded.
- Earlier profiling (`app/tools/profile-editor.py`, sampling) covered idle and loading, not orbiting. A UI submit costs
  `RenderNode.walk` about 14 %, `UiStyleInvalidationMetrics.compareNode` about 16 % (this is the real invalidation pass, not
  debug), `syncRevisions` and value equality. Whether orbiting triggers a full UI submit per frame is unknown.
- GPU stats: `nkgpu_renderer_get_stats` exists. The editor capture mode currently fails writing `layout.json` ("Cannot encode
  function as JSON" in `DesktopUiHost.writeDiagnostics`), a separate bug that may need a small fix to get frame captures.
- `hlprof-live report` over a full capture is too slow. Report 60 s windows with `--start-ms`/`--end-ms`.

**Steps.**
1. **Reproducible benchmark.** A scripted orbit (fixed camera path, N frames) over the hobber, warm-opened from the scene cache,
   recording per-frame CPU time split by phase (UI submit, scene update, render submit), GPU time (timer queries or renderer
   stats), draw calls, triangles and buffer uploads per frame. Use the scripted-editor infrastructure (`app/scripted/`) or a CLI
   flag. Print a summary and keep it as a perf check.
2. **Diagnose.** Find out whether it is CPU bound (UI rebuilds while orbiting, per-object scene work, culling, matrix updates,
   re-tessellation, per-frame allocation and GC) or GPU bound (draw call count, triangles, overdraw, shadows, MSAA, edge and line
   rendering). Profile the orbit window.
3. **Fix the dominant costs cleanly, in order of gain.** Candidates:
   - no UI submit or invalidation for camera-only frames;
   - static batching or instancing of repeated parts (screws, blocks, wipers);
   - level of detail or decimation for tiny high-poly features (threads, gear teeth, cable spirals);
   - frustum culling;
   - feature-edge caching;
   - avoiding per-frame buffer re-uploads.

   Measure every change on the benchmark.

**Acceptance.** Orbit frame time before and after on the same machine, a target of at least 60 fps (or the best achievable with
the reason stated). The app tests pass, the visuals are unchanged (or the LOD threshold is documented), and the benchmark is
committed.

**Status: measured CPU fixes complete; 60 fps remains unmet.** The best observed fixed run is 38.60 fps; the final awake
paired comparison is 25.54 -> 30.04 fps. All application tests pass. Quality remains 4x MSAA with no LOD, tessellation,
culling, batching or renderer architecture changes. Stop here at the native frame scheduling/presentation boundary;
a reliable 60 fps result needs further native rendering work, rather than more camera-only UI caching.

**Final paired comparison.** `orbit-baseline-awake-03` uses application sources from `3441586c3` (benchmark before performance
fixes); `orbit-final-awake-03` uses `5b5a4d1d5`. Only Main/viewport were temporarily restored for the baseline, then restored
byte-for-byte before rebuilding the final app. `paired-variant.json` records their source commit and SHA-256 hashes; native
code, project, runtime and settings are shared. Both captures record DPMS On throughout. Run artifacts are below
`/media/joao/DATA1/claude-hobber-tools/tmp/`. The final compiled app contains the final sources.

Benchmark: full desktop host, routed captured middle-button drag, one degree yaw per rendered frame at fixed pitch 0.35;
60 warmup + 360 measured + 8 GPU drain frames. Warm cached 403-object hobber, 1320x900 window, 766x498 scene target,
4x MSAA, GTX 1060 3GB / NVIDIA 550.144.03 on the 60 Hz GTK desktop. Per-frame CPU includes input; GPU is scene passes only,
not UI/compositor. Phase timings nest. Draws and triangles include AO and edge passes. Throughput uses actual start cadence.

| Metric | Awake baseline 03 | Final awake 03 |
| --- | ---: | ---: |
| Throughput | 25.54 fps | 30.04 fps |
| Mean cadence | 39.15 ms | 33.29 ms |
| CPU median / p95 | 15.91 / 37.27 ms | 4.44 / 7.58 ms |
| Scene GPU median / p95 | 7.98 / 21.50 ms | 6.33 / 7.07 ms |
| Drag input median | 4.134 ms | 0.043 ms |
| View setup median | 4.250 ms | 0.092 ms |
| UI submission median | 2.837 ms | 0.072 ms |
| Native scene preparation median | 0.610 ms | 0.638 ms |
| Native scene submission median / p95 | 1.255 / 6.550 ms | 1.341 / 2.359 ms |
| Draw calls median | 1,541 | 1,540 |
| Submitted triangles median | 2,275,358 | 2,275,354 |
| Resource upload median / p95 | 0 / 864 bytes | 0 / 288 bytes |
| Allocations median | 1,854,172 bytes | 116,664 bytes |
| Reused UI submissions | 0 / 360 | 360 / 360 |
| Automatic collections / GC time | 17 / 248.67 ms | 0 / 0 ms |

CPU falls 72.1%, allocations 93.7%, and throughput rises 17.6% in the paired run. GPU load varies on the shared workstation;
do not attribute its change to the CPU fixes. The one-draw/four-triangle median difference is the hover overlay suppressed
while navigating. Hover resumes on release. After that change, all 360 draw/triangle pairs match through the final run;
the final viewport matches the previous bounds capture pixel-for-pixel across 379,440 interior pixels.

**Root causes and commits.**

- `89a641fab`: drag-time hover picking performed geometry ray tests despite camera capture; suspend it during navigation.
- `1dcfd4327`: camera frames rebuilt presentation overrides and revalidated cosmetic threads; retain a revision-keyed view.
- `7fbd0d1c0`: collision summary cleared/restored its observed value each frame, and camera invalidation requested a widget
  patch; publish the final summary once and use the existing paint invalidation API. Keep full rebuild fallback on unmount.
- `5b5a4d1d5`: every clip fit rebuilt world bounds and used repeated linear visibility lookups; retain displayed bounds by
  revision and still fit the current camera's near/far range each frame.
- `3441586c3`: committed routed benchmark, asynchronous GPU stats, per-frame CPU/input phases and report regressions.
- `542973f60`: reject display-power-saving captures, after observing a focused but DPMS-Off window fall below 1 fps.
- `5bb9c3f98`: prerequisite project UI preparation before tracked panel construction; fixes benchmark project opening.

Capture prerequisite: haxeon branch `viewport-perf-capture`, commit `84113e4904a483fcbad00612ca7ef20ce761b905`, serializes
computed diagnostic styles without function values. The parent gitlink stays at `e7d8716e59dc387409df4de1a15029883c2aca14`.
No nativekit branch or changes. No push/merge. Other sessions' four source files are untouched by these commits.

**Validation and remaining limit.** Full app suite passes (exit 0, `viewport-retained-bounds-app-tests-03.log`), including
268 routed coordinate assertions, retained selection/pose/section behavior, collision notification suppression, 88 clipping
assertions and the complete CAD workflow. All seven standalone benchmark-report tests pass. All final source files are
restored and the compiled app was rebuilt from them.

Final CPU p95 7.58 ms and scene GPU p95 7.07 ms are below 16.67 ms individually, yet 351/359 intervals round to 33 or 34 ms.
Median request-to-callback delay is 28.86 ms. Thus final cadence is limited by the native frame scheduling/presentation
path, not the removed managed work or a proven geometry throughput ceiling. Native GTK runs rendering in GtkGLArea's
platform-owned callback/context; its backend submit/finish defer actual presentation until callback return. Earlier fixed
captures also show NVIDIA submission stalls (~16 ms p95) and GPU tails (~18 ms p95). The exact driver/GTK scheduling cause
is not isolated; do not claim a universal 30 fps cap or guaranteed 60 fps after a particular native change.

Controls: no-query/no-sampling orbit remains below 60 fps, 1x MSAA does not remove the cadence limit, and a clear-only native
surface achieves 58.51 fps (42.75 fps with a 10 ms callback delay). Fixed-app profiling shows native-driver yielding and P0
GPU clocks, rather than a fixed low-power clock. Further work should trace GTK frame-clock deadlines and native
submission/presentation scheduling before choosing a physical render-executor split or other renderer architecture change.
Such a change is a new design boundary; no visible LOD or native scheduling workaround is applied here.

**Investigation record (chronological; pending notes below describe their historical point in the work).** Inspected the editor host, scripted entry point, retained viewport paint binding and SceneKit executor.
The design-mode editor already uses `submitCached` and a camera paint patch. Measure their actual cost before changing them.
The native capture profiler already supplies asynchronous per-pass GPU timestamps (`NK_SCENE_GPU_TIMINGS`, interval 1 for
the benchmark); extend this single instrumentation path with draw/triangle/upload counters instead of adding another timer.
The scripted benchmark routes captured middle-button drags through `UiContext`, advances by one degree per rendered frame
and runs the complete desktop host. Camera-only setters would omit orbit's current per-move hover ray test; input time and
camera advance/capture checks are therefore part of the benchmark. Suppressing drag-time hover is a candidate, not yet a
measured fix. The cache-priming run still uses the earlier camera-only draft; it is not the interaction baseline.
No LOD or visible feature changes are authorized. Initial runtime build exposed uninitialized nested dependencies; the pinned
Haxeon submodules are now initialized and the release runtime builds. After waiting for a shared `heavy-run` slot, the cold
app build found the app's MuJoCo/animation dependencies uninitialized; restoring those exact pins before retrying native
configuration. The first Haxe compile passed (3,328 files), and native configuration now passes with the pinned dependencies.
The native build passed. The initial priming run rejected an empty scene: Haxeon launches the app with `app/` as its cwd,
while supplier libraries need `MATERIA_ROOT` for this checkout's `build/suppliers` link. The runner now sets that root and
keeps temporary generation files in its output directory. The same compiled generator succeeded from the worktree root
(serial diagnostic run): 59,569,269 artifact bytes, SHA-256
`f20aa39b53c57519074fa94d4a5fa62a30d62b98e90c0d7791550ef46b212d7d`. Its own-worktree fingerprint cache is primed.
The routed-input app build passed (3,328 files; native targets cached). Its idle compiler worker retained the completed build's
queue lock and 7.7 GB heap; that exact own-worktree idle PID was stopped after verifying the build had exited successfully.
Bounded benchmark runs now use the supported `HAXEON_COMPILER_IDLE_SECONDS=1` setting. The short routed-input pilot is queued.
Native executor syntax and the six standalone report regressions pass (`python3 app/tools/test-benchmark-orbit.py`):
time-based GPU/input matching, capture/camera advance, warmup/drain exclusion, throughput from nonuniform cadence, and
failed-load/missing-result rejection. Runtime validation and the routed-input baseline are pending.
The routed pilot loaded the cached artifact but aborted before its first scene frame: `projectUiPanel()` lazily assigned
`projectUiExtension` inside a tracked Inspector build. Extension opening/colour publication is now prepared in
`updatePresentation()`; panel builds only read that state. A replacement/error/close regression was added to the app suite
and the routed pilot then rendered the project successfully. The full app suite, including the new regression, passed:
`./haxeon/scripts/haxeon run --project app/tests/haxeon.json` (exit 0; `viewport-app-tests-prereq.log`).

**Initial full-turn measurement (before performance fixes).** `orbit-baseline-01`: 60 warmup + 360 measured + 8 drain frames,
403 objects, 766×498 scene image, 4× MSAA, GTX 1060 3GB. 20.16 fps (49.62 ms cadence); CPU median 27.69 ms / p95 58.03 ms;
scene GPU median 14.14 ms / p95 30.13 ms. Median input 6.41 ms, UI submit 2.89 ms, custom paint 14.22 ms, native scene
prepare 0.94 ms and submit 4.75 ms, native UI render 2.18 ms. Median 1,541 draws / 2,275,358 submitted triangles (including
AO/edge passes), median resource upload 0 bytes. Median frame allocation 1,854,656 bytes; 17 collections / 378 ms GC over
the measured window. No physical pointer moves were delivered during this run; pitch remained 0.35.

Physical mouse input contaminated the earlier short pilot's pitch. The harness now uses a separate pointer ID and records
and validates pitch as well as yaw. Recheck the full baseline with that hardened harness before comparative fix commits.
These phase/allocation measurements point to transient view setup and drag input, rather than geometry re-upload; confirm
the hot functions with the CPU profiler before optimizing.

**Step 1 validated: hardened baseline.** `orbit-baseline-02` used the same full-turn path and matched the first run's camera,
draw/triangle/upload statistics exactly. Its isolated pointer ID tolerated 94 physical pointer moves without changing pitch.
Six report regressions pass. The committed runner command is documented in `app/README.md`.

| Metric | Baseline 02 median | p95 |
| --- | ---: | ---: |
| CPU frame | 23.88 ms | 60.25 ms |
| Scene GPU | 12.00 ms | 22.60 ms |
| Routed input | 5.62 ms | 9.17 ms |
| View setup | 6.82 ms | 8.27 ms |
| Scene plan preparation | 0.49 ms | 0.68 ms |
| UI submit | 3.37 ms | 4.53 ms |
| Draw calls | 1,541 | 1,581 |
| Submitted triangles | 2,275,358 | 2,290,826 |
| Resource uploads | 0 bytes | 864 bytes |

Cadence: 24.56 fps / 40.71 ms mean. Background shared-machine load varies (compare baseline 01); use matching paths and
repeated runs before interpreting small gains. The first CPU sample capture is qualitative because one physical pointer
event changed its pitch; it found cosmetic-thread checks and clip fitting in routed input. A hardened, warmed orbit profile
is the next diagnostic, followed by independent measured fixes. Native plan refresh is already small; view setup and input
are the principal CPU phase targets. No performance fix, LOD, quality reduction or renderer architecture change is applied yet.
The capture serialization repair is committed as haxeon `84113e490` on branch `viewport-perf-capture`; the parent pin remains
unchanged. Validation: parseable layout for 209 nodes/80 styles, 9,696 loading/empty-scene host timeline records and screenshot.
Those records are capture validation only and are not hobber orbit measurements.


**Step 2: warmed diagnosis.** `orbit-profile-before-02` captured 428 ready frames, 403 objects, constant pitch 0.35
and one-degree yaw steps. Analysis uses Frame 229 through Frame 588 (11.874–30.481 s), excluding loading and warmup.
Within routed input, `ScenePresentation.configureRenderView` accounts for 70.8% of samples and cosmetic-thread `ensure`
for 49.5%; within paint, `ensure` accounts for 41.0%. This repeatedly scans CAD metadata and serializes appearance merely
to discover unchanged overlay nodes. Uploads are already mostly zero; tessellation and native plan refresh are not the
steady-orbit bottleneck. View construction also repeats pose, selection, visibility and section overrides each camera frame.

**Step 3a: suspend hover during camera navigation.** Orbit/pan were omitted from the existing drag-time hover policy.
Clear hover on navigation press, skip picks on captured moves, and restore hover on release through the existing handler.
This intentionally removes transient hover highlighting during navigation; geometry, materials, camera path and render
quality are unchanged. Routed coordinate regressions cover hover before/during/after orbit and preserve the surface pivot.

`orbit-no-drag-hover-01`, same full-turn benchmark:

| Metric | Baseline 02 | Drag fix 01 |
| --- | ---: | ---: |
| Throughput | 24.56 fps | 28.64 fps |
| Mean cadence | 40.71 ms | 34.92 ms |
| Median CPU / p95 | 23.88 / 60.25 ms | 15.87 / 21.83 ms |
| Median input / p95 | 5.62 / 9.17 ms | 0.060 / 0.089 ms |
| Median view setup | 6.82 ms | 6.59 ms |
| Median scene GPU / p95 | 12.00 / 22.60 ms | 9.13 / 12.00 ms |
| Median draws | 1,541 | 1,540 |
| Median submitted triangles | 2,275,358 | 2,275,354 |
| Median uploads / p95 | 0 / 864 bytes | 0 / 288 bytes |

Tiny draw/triangle differences are the removed hover overlay. GPU improvements also reflect shared-machine load and must
not be attributed solely to this CPU fix. The measured input reduction is direct evidence. Next: retain camera-independent
view overrides while updating camera, grid and studio lighting every render. No LOD or renderer architecture change.

Validation: routed viewport-coordinate suite passes 238 assertions plus hover/docked-navigation regressions.


**Step 3b: retain scene-view overrides.** The viewport now retains its render view across camera-only frames; scene/selection,
runtime pose, presentation/section, lighting, hover, collision, cosmetic visibility, read-only and grid revisions rebuild it.
Projection, exact eye/direction, grid ray basis and studio lighting still update every render. Picking uses its existing fresh
views. Dispose drops the retained reference. No native renderer policy, batching, LOD, resolution or MSAA change.

`orbit-retained-view-01` (same 360-frame full-turn, 4x MSAA): 29.75 fps / 33.62 ms mean cadence; median CPU 10.50 ms
(p95 13.13), input 0.051 ms, view setup 1.003 ms, scene GPU 6.24 ms (p95 7.14). Median draws 1,540, triangles 2,275,354,
uploads 0 bytes (p95 288). All 360 per-frame draw/triangle pairs exactly match the drag-fix run. Final viewport screenshot
is pixel-identical over 379,440 pixels. The reduction from 6.59 to 1.00 ms setup directly measures the removed work;
shared-machine load contributes to the GPU difference. Added retained-view selection, pose and section-close regressions;
full app suite pending. A compiler worker EOF triggered the compiler's successful one-shot fallback, without source errors.

The median callback is now 10.50 ms and scene GPU 6.24 ms, but cadence clusters at 33.34 ms on the 60 Hz GTK desktop.
Request-to-callback delay is 23.01 ms. Investigate presentation timing (including a diagnostic 1x MSAA run) before attributing
the remaining 30 fps ceiling to scene complexity. Quality remains at 4x MSAA for acceptance comparisons.

Retained-view allocation check: median host-frame allocations fall from 1,851,404 to 491,320 bytes (73.5%).
Automatic GC collections over 360 frames fall from 10 to 3, with total measured GC time 164.33 -> 24.90 ms.

Presentation diagnostic: `orbit-retained-msaa1-diagnostic` completes the identical 360-frame path at 1x MSAA: 29.79 fps,
33.57 ms mean cadence, median CPU 10.17 ms, scene GPU 7.77 ms, identical draw/triangle counts. It does not remove
the ceiling. GPU load differs, so this is a cadence diagnostic, not a quality-adjusted acceptance comparison.
The acceptance setting remains 4x MSAA. A minimal native surface callback is the next control.


The retained-view full app suite passed (exit 0; `viewport-retained-app-tests.log`), including 238 coordinate assertions,
selection/pose/section invalidation, project UI frame preparation and the complete CAD workflow.
A minimal native surface control (1320x900, clear only, self-requested on-demand frames, 30 warmup + 120 samples) reaches
58.51 fps with no work and 42.75 fps with a 10 ms callback delay. Thus this is not an intrinsic fixed 30 fps backend cap;
frame-clock deadlines are sensitive to callback work. The fixed-app warmed profile (`orbit-profile-retained`, frames
150–509, 8.324–20.425 s) also measures native UI GPU time at 0.503 ms median / 0.537 ms p95. CAD input and overlay
validation are no longer dominant, but UI submission still walks the tree for a camera-only widget patch.

**Step 3c implementation: paint-only invalidation.** Use the existing `RenderNode.invalidatePaint()` operation for the
viewport's native-camera revision instead of `BuildContext.requestPatch()`. Resolve reconciled/replaced nodes through
`RenderNode.latest` and validate their current self-updating registration before invalidating; an unmounted binding still
requests the ordinary owner rebuild. Keep the custom display-list cache for unchanged frames. Both perspective and
inspection views use `updateRetainedPaint`, removing the separate camera patch path. No new framework API or rendering
architecture is introduced. Routed camera regressions now assert reuse of the submitted UI tree during each orbit move.


**Benchmark hardening: DPMS.** The display entered DPMS Off during a later run, reducing cadence below 1 fps while the
window remained focused and viewable. `orbit-stable-ui-01` is discarded. Wake the display without changing its power policy
(`xset dpms force on`). The runner now samples X11 DPMS state once per second, records `display-timeline.jsonl`, rejects
an initially sleeping display and rejects measured windows overlapping Off/Standby/Suspend. Unsupported queries are
recorded as unavailable. Seven report tests pass, including rejection of sleeping measurement windows and exclusion of
pre-warmup sleeping samples. `orbit-stable-ui-02` records On throughout: 38.60 fps, median CPU 7.74 ms, scene GPU 6.41 ms,
1,540 draws and 2,275,354 triangles with the pending UI-frame fix. This is validation of an awake capture; GPU time varies
with shared-machine load. Earlier healthy baselines predate DPMS recording; their cadence is tens of milliseconds rather
than the multi-second power-saving cadence. Use a guarded paired comparison for the final report.


**Step 3c validated: keep camera frames on the retained UI path.** Direct source invalidation counting found
`ReferenceEditorApp.collisionSummary` being cleared to null and restored on every query (two invalidations per frame).
The perspective controls observe it, so unchanged queries rebuilt their tab content. Publish the final summary once;
clear it only when collision display is disabled. Together with `RenderNode.invalidatePaint` this removes both reasons
for a camera-only UI rebuild. Temporary diagnostic source and core instrumentation have been removed; haxeon is clean.

`orbit-stable-ui-02`, display On throughout, same 360-frame 4x-MSAA path:

| Metric | Retained view 01 | Stable UI 02 |
| --- | ---: | ---: |
| Throughput | 29.75 fps | 38.60 fps |
| Mean cadence | 33.62 ms | 25.91 ms |
| Median CPU / p95 | 10.50 / 13.13 ms | 7.74 / 20.76 ms |
| Median UI submit | 3.20 ms | 0.084 ms |
| Reused UI submissions | 0 / 360 | 360 / 360 |
| Median scene GPU / p95 | 6.24 / 7.14 ms | 6.41 / 17.75 ms |
| Median allocations | 491,320 bytes | 155,824 bytes |
| Automatic GC collections | 3 | 1 |

All 360 draw/triangle pairs match (medians 1,540 / 2,275,354); the final viewport remains pixel-identical over 379,440 pixels.
The full app suite passes (`viewport-stable-ui-app-tests-02.log`, exit 0), including camera-only UI reuse, unchanged collision
summary notification suppression, disabled-summary publication, and existing geometry/CAD workflows. The fly-paint assertion
now accepts either changed content key or paint revision, matching the public display-list invalidation contract.

The native-submit p95 is 15.50 ms in this capture, while native plan preparation is 0.52 ms median. The warmed fixed-app CPU
profile shows NVIDIA driver work and scheduler yielding inside scene rendering after the managed rebuild costs are removed.
GPU state during the profile is P0 (1,569–1,771 MHz graphics, 4,006 MHz memory); no fixed low-power clock limit was observed.
Those controls and the awake paired comparison are recorded in the final status above. No LOD or renderer architecture change is applied.


**Remaining-cost controls.** `orbit-no-query-control-01` runs the same 428 ready frames without CPU sampling or GPU timing
queries: 29.15 fps, median CPU 8.20 ms, 360/360 UI submissions reused. Removing query instrumentation does not remove the
cadence limit. Shared-machine timing varies; this control is not a quality-adjusted performance result.

**Step 3d: retain displayed clip bounds.** World bounds are independent of camera motion but were
rebuilt for every clip fit; visibility checks also looked up each already-known item through a linear scene search.
Retain the bounds until scene visual, runtime-pose/active, or presentation revisions change, and use the existing visibility
policy directly on the current item when rebuilding. Recompute near/far clipping against the current camera, aspect and
inspection FOV on every call. Add regressions for edited geometry, hide/show, runtime-pose revisions, leaving simulation,
and camera motion against a freshly computed reference. No clipping margin or culling policy changes.


`orbit-retained-bounds-01` (display On, same 360 frames, 4x MSAA) reduces median view setup from 1.007 to 0.127 ms
(p95 0.151 ms) and allocations from 155,824 to 116,332 bytes per frame. All 360 draw/triangle pairs and all 379,440 viewport
pixels match `orbit-stable-ui-02`. Median CPU is 8.07 ms, scene GPU 6.61 ms, and throughput 31.91 fps; native UI rendering
rose from 1.67 to 2.85 ms under shared-machine load. This run establishes the setup/allocation saving, not an end-to-end
FPS gain. Scene submission remains 15.85 ms at p95 and GPU 18.28 ms at p95. Median GPU pass times are 3.64 ms surfaces,
2.38 ms surface edges, 0.44 ms AO surfaces, and 0.03 ms resolve. Full app suite passed (exit 0; `viewport-retained-bounds-app-tests-03.log`),
including 88 clip assertions, 268 routed coordinate assertions and the complete CAD workflow. The new bounds fixture uses
an empty scene so default editor objects cannot mask a moved object's clip range.

## Round 2 (2026-11-06, lead)

Round 1 is merged (9ae2bb27f): camera frames take the retained UI path, hover picking is suspended while navigating, bounds are
retained. CPU went from 15.9 to 4.4 ms median, GPU from 8.0 to 6.3 ms, and allocation from 1.85 MB to 0.12 MB per frame.

**Out of scope: native/GTK frame pacing.** The 30 fps (33 ms) cap is the compositor. The user has a separate patch for it. Don't
investigate it, and don't judge success by fps: measure per-frame CPU and GPU milliseconds and the work counts.

Round 2, everything else, measured on the committed orbit benchmark (`app/tools/test-benchmark-orbit.py`):
1. **Draw calls (1,540 for 403 objects).**
   - Find why there are so many: per-face or per-feature draws, edge and section passes, picking, overlays, and duplicated draws
     for repeated parts.
   - Cut them: batch static geometry by material, instance repeated parts (screws, dowels, wipers, blocks, rails), and merge
     per-part sub-draws.
2. **Triangles (2.28 M).**
   - Find which parts dominate: threads, gear and hob teeth, cable spirals, fillets, STEP-imported parts (eRob, BK12 mounts,
     scales).
   - Add view-dependent level of detail or tessellation tolerance by screen size, and frustum culling. Keep visible quality the
     same at normal zoom, and document any threshold.
3. **GPU passes.** The cost of edges and silhouettes, section caps and contours, MSAA and shadows per frame. Cache what doesn't
   change with the camera.
4. **CPU per frame.** Profile the remaining 4.4 ms (scene update, culling, uniform uploads, render-graph build) and cut what is
   per-frame but camera-independent.
5. **Scale.** Re-run on the bench mill and the tending cell as well as the hobber, so the gains are general.

### Round 2 status: material instancing and edge caps measured; remaining work in progress

Read the retained viewport, SceneRenderer, render compiler, GPU executor and nkgpu draw path.
Frustum culling and geometry/material instancing already exist. Surface, stroke and AO passes
each submit the visible batches; these are not per-face draws. Each loaded occurrence receives
its own material resource, potentially splitting repeated geometry into separate batches.
The census below confirms this split; GPU equivalence now coalesces it without changing ownership.

The orbit runner now accepts `--project` for the scale checks, summarizes GPU passes and
per-pass draw/triangle work, and exports a node/resource inventory beside the native timeline.
No rendering behavior, quality, tessellation or scheduling change yet. Native instrumentation
compiles; eight report tests and the native compiler/GPU/AO/section-cap/contour suites pass.
The first current-source hobber capture (`r2-baseline-hobber-01`) completed: 35.73 fps,
8.74 ms median CPU (22.68 p95), 6.86 ms scene GPU (19.64 p95), 1,540 median draws and
2,299,637 median submitted triangles. This run primed changed source geometry; it is not the
warm acceptance baseline. Publishing the instrumentation library also invalidates the
generator fingerprint, so `r2-baseline-hobber-02` primes that dependency before the unchanged
warm run and resource census. No gain is claimed from the timing variation. Artifacts use the `r2-` prefix
under `/media/joao/DATA1/claude-hobber-tools/tmp/`.

**Warm current-source baseline:** `r2-baseline-hobber-03`, 360 measured frames after 60 warmup,
cached source open in 2.46 s, 4x MSAA, 766x498 target, DPMS On. Median/p95 CPU
7.44/21.57 ms; scene GPU 8.16/18.70 ms; native scene preparation 0.933 ms,
submission 2.341 ms, plan preparation 0.521 ms. Throughput 30.25 fps (reported only).
Median draws 1,540; submitted triangles 2,299,637; all work counts match the priming run.
GPU medians: surfaces 4.515 ms, edges 2.978 ms, AO surfaces 0.545 ms, resolve 0.037 ms.
Median pass work: surfaces 521 draws/485,402 triangles, edges 501/1,328,858,
AO surfaces 518/485,377. The original 403 editor objects expand to 542 native nodes
including cosmetic overlays (535 shown); 318 shown geometry IDs have 535 separate
materials. Repeated geometry includes 71 M6 bolts, 21 M8 bolts, eight HGH15 blocks,
and repeated mounts, wipers, extrusions and door rollers. The hob alone has 151,696
base triangles; the casting and three BK12 mounts dominate edge segments.

**First fix measured:** coalesce GPU instance groups with exactly equal material
values, keeping source material IDs and source batches unchanged. Equivalence includes
all colour/opacity/finish/alpha/sidedness, texture and sampler fields. Resource edits
re-evaluate equivalence; camera frames retain it. Sparse visibility/pose changes use
the geometry's indexed source batches. Native callers updating resource values against
a retained render plan are covered. The new GPU regression verifies equal-resource
pixel identity, independent edits, splitting/rejoining, and occurrence picking. All
11 renderer suites pass, including lighting images and the 50,000-node sparse-update
benchmark (whose material groups now use explicitly distinct roughness values).
`r2-material-batches-hobber-01`, same full source-project orbit, 4x MSAA and camera:

| Median metric | Warm baseline 03 | Material batches 01 |
| --- | ---: | ---: |
| CPU / p95 | 7.44 / 21.57 ms | 6.18 / 20.20 ms |
| Scene GPU / p95 | 8.16 / 18.70 ms | 6.11 / 16.66 ms |
| Native preparation | 0.933 ms | 0.669 ms |
| Native submission | 2.341 ms | 1.260 ms |
| Draw calls | 1,540 | 910.5 |
| Submitted triangles | 2,299,637 | 2,299,637 |
| Surface GPU | 4.515 ms | 2.861 ms |
| Edge GPU | 2.978 ms | 2.465 ms |
| AO surface GPU | 0.545 ms | 0.457 ms |
| Throughput (context only) | 30.25 fps | 35.48 fps |

Median draws fall 40.9%; all 360 per-frame triangle counts match. Median per-pass
draws become 311.5 surfaces, 291 edges and 308.5 AO surfaces. Timings vary under
shared-machine load; the work reduction is deterministic. At the identical final
camera, 3 of 376,922 interior viewport pixels differ by at most one 8-bit channel
level (`comparison.json`); no visible quality, resolution, MSAA or LOD change.
The full app suite passed (exit 0, `r2-material-batches-app-tests.log`); scale checks,
triangle reduction and remaining CPU/GPU work are pending. Source hashes are in
`r2-material-batches-variant.json`. No haxeon/nativekit changes for this fix.

**Triangle-cap fix measured:** the stroke buffer used six triangles
per segment for its body and two round caps, plus up to two incident support triangles.
Each cap's circular antialias footprint is now covered by one circumscribed triangle
instead of a quad, retaining the shader's coverage/depth rule. Body endpoints,
support triangles, opacity, clipping, stroke width and MSAA stay unchanged. Reserve
and overflow limits derive from the actual corner count, also for section contours.
All 11 renderer suites pass, including new short/long stroke endpoint coverage and
reversal checks at 24 angles with 1x/4x MSAA. Canonical shaders remain up to date
with the pinned shader compiler (only a comment changed). No LOD.

`r2-triangle-caps-hobber-01`, identical full source-project orbit and 4x MSAA:

| Median metric | Material batches 01 | Triangle caps 01 |
| --- | ---: | ---: |
| CPU / p95 | 6.18 / 20.20 ms | 3.90 / 19.18 ms |
| Scene GPU / p95 | 6.11 / 16.66 ms | 4.70 / 15.09 ms |
| Native preparation | 0.669 ms | 0.463 ms |
| Native submission | 1.260 ms | 0.782 ms |
| Draw calls | 910.5 | 910.5 |
| Submitted triangles | 2,299,637 | 1,965,069 |
| Edge triangles | 1,328,858 | 994,290 |
| Edge GPU | 2.465 ms | 1.637 ms |
| Surface GPU | 2.861 ms | 2.582 ms |
| Throughput (context only) | 35.48 fps | 52.28 fps |

Edge triangles fall 25.2%; total submitted triangles fall 14.5%. All 360 per-frame
draw counts and surface triangle counts match. At the same final camera, 34 of
376,922 interior pixels differ, maximum two 8-bit channel levels (only two pixels
exceed one level). This is raster interpolation rounding, with no visible quality
change. Comparisons are in `comparison.json`; source hashes in
`r2-triangle-caps-variant.json`. Timing also varies under shared load; attribute the
deterministic work reduction to this fix, not every CPU/surface timing change.
The full app suite passed for material batching; final app validation after remaining
render changes and the bench-mill/tending-cell comparisons are still pending.

**Remaining CPU diagnosis:** `r2-orbit-profile-01` reuses the built cap version with
500 Hz CPU sampling and allocations disabled. The source open is cached (2.19 s).
Inspect only host frames 133–492, 5.668–12.486 s (60 warmup/360 measured/8 drain),
not loading. Bounded native report: 1,059 on-CPU samples; UI `prepareCommands` has
13.79% inclusive, native UI execution 17.75%, NVIDIA 14.45%, scene plan update 4.82%,
and the scene surface draw lambda 3.12%. `Std.isOfType`/safe casts also remain.
Artifact `orbit-cpu-report.txt` records this window; full capture wall/GC summaries
include loading and are not steady-orbit results. Compositor waiting is excluded.

**Controlled renderer variants for scale checks:** the runner's `--skip-build` and
`--native-library-dir` options run the same built app with a separately built scene
renderer, recording the selected binary's path/SHA-256. The default build/run remains
unchanged. The built launcher prepends the explicit directory rather than replacing
the production renderer or source files. `r2-baseline-sources` is a `git archive` of
`dfddf3806`'s SceneKit, built with the current NativeKit and matching Release flags plus
frame pointers; configuration is in `r2-baseline-render-build.json`.
The baseline pilot (`r2-baseline-variant-pilot-01`, 5 warmup/16 measured) opens the
previous hobber artifact only to validate library selection: 1,581 draws/2,315,109
triangles match the opening-angle baseline census. `/proc` maps verify only the scene
renderer comes from the baseline build; GPU and scene libraries remain the app's.
This pilot is not a full-orbit performance comparison. Eight report tests and the
built-launcher shell check pass. Use full source-project orbits for the scale checks.

**Pass-uniform fix measured:** keep the bound surface pipeline between compatible
draws, uploading camera/lighting/clip blocks once per actual pipeline transition. Reset
tracking after section caps, which can change GPU state. AO/picking and edge passes
bind their fixed pipelines and camera/clip blocks once; per-material/sidedness data
still updates per draw. Nothing is retained across frames. All 11 renderer suites pass,
including a new pixel comparison of mixed indexed/nonindexed, opaque/blended draws
against isolated references with camera shifts, sections and 1x/4x MSAA.
`r2-pass-uniforms-hobber-01` has identical 360-frame work counts, but timings increased
across both changed and unchanged phases (for example input 0.043 -> 0.162 ms and native
UI 1.669 -> 3.920 ms), so it does not establish a performance gain. A back-to-back
source-project pair (`r2-pass-uniforms-before-02` / `after-02`) compares the separately
built cap renderer (`5de7ad27d`) with the current uniform version using the same built
app; build recipe and sources are in `r2-native-variant-cmake` / `r2-cap-variant-sources`.

| Median metric | Cap renderer, before 02 | Pass uniforms, after 02 |
| --- | ---: | ---: |
| CPU | 6.963 ms | 6.616 ms |
| Scene GPU | 5.302 ms | 4.893 ms |
| Native preparation | 0.876 ms | 0.817 ms |
| Native submission | 1.625 ms | 1.343 ms |
| Native UI render (control) | 2.710 ms | 2.683 ms |
| Draw calls | 910.5 | 910.5 |
| Submitted triangles | 1,965,069 | 1,965,069 |
| Throughput (context only) | 30.99 fps | 32.39 fps |

All 360 work-count pairs match and all 376,922 final interior pixels are identical.
The reverse-order pair (`after-03` then `before-03`) reports CPU 5.917 vs 6.308 ms,
GPU 5.154 vs 5.996 ms, native submission 1.219 vs 1.254 ms, and UI 2.242 vs 2.746 ms.
Its UI control varies, so do not attribute that entire CPU difference to the fix.
Both pairs support keeping the smaller scene-submit change; the gain is modest.
Reverse-order work and pixels also match exactly. Binary hashes are recorded by the
runner; `r2-pass-uniforms-variant.json` maps the production sources to this variant.

Further candidates (not implemented): surface shaders already discard single-sided back
faces, while all ordinary pipelines disable hardware culling. A per-batch winding
classification (positive, reflected, mixed/singular), retained with instance transforms,
could safely reject the same faces earlier; reflected batches need front culling with
the original CCW convention to preserve the shader's `gl_FrontFacing` parity test.
Screen-size LOD/tessellation and the bench-mill/tending-cell comparisons remain pending;
frustum culling is already active and covered by the native camera/culling tests.

**Scale validation running:** `r2-scale-current.sh` runs source-project full orbits
serially: `r2-scale-bench-before-01` / `after-01` use `bench-mill/materia.bench.project.json`;
`r2-scale-cell-before-01` / `after-01` use `machine-tending/materia.project.json`.
Before is the separate `dfddf3806` renderer; after is the built three-fix renderer.
Both use the same built app and the standard 60/360/8 frame, 4x-MSAA benchmark.
The script then runs the full app suite into `r2-three-fixes-app-tests.log`.
Do not rebuild/replace the app's native libraries while these captures are live.
An own-process-scoped helper forces DPMS On without changing its policy while the
batch lives (`r2-scale-display-wake.jsonl`). Native and app run handles must be checked
before interpreting unfinished logs or starting another main-app renderer build.

LOD investigation remains real work, not a completed gate: consider cached index-only
LODs for high-poly, untextured geometry without feature strokes (the hob is the main
candidate). Reuse original packed vertices/attributes and full-resolution picking;
select a level conservatively for the largest projected instance in a batch so there
is no per-camera instance regrouping/upload. Section/inspection correctness and a
documented screen-error threshold need validation before any such fix is accepted.
