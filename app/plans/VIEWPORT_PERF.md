# Viewport performance: orbiting the flexspline hobber (2026-11-06)

**Symptom (user).** Orbiting the camera around the flexspline hobber in the editor is slow: low frame rate and lag.

**Known so far.**
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

**Status (in progress).** Inspected the editor host, scripted entry point, retained viewport paint binding and SceneKit executor.
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
