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
