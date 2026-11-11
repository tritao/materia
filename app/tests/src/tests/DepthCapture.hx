package tests;

import app.editor.ViewportDepth;
import app.editor.ViewportState;
import haxe.io.Bytes;
import nativekit.scene.DepthPrecision;

/**
 * Replays a viewport state copied in the editor ("Copy view state") and writes
 * what its depth buffer holds there:
 *
 *   depth.png    view depth of every covered pixel, log scale near white to far dark; background blue
 *   steps.png    depth-buffer step per pixel in that pixel's footprints: green well under
 *                the limit, yellow at it, red ten times over; background black
 *   summary.json near/far, focus, step percentiles, pixels over the limit
 *   window-depth.f32  raw window depth, float32 per pixel, rows from the top down
 *   faces.png    which subelement won each pixel: one flat colour per (object, subelement), a
 *                dark line where neighbours differ. Subelements are B-rep faces when the mesh
 *                carries face ids, otherwise triangles, so plain meshes show their tessellation.
 *   frame.png    the rendered frame as the viewport draws it (lighting, edges, samples)
 *   frame-shaded.png  the same frame without edges (strokes or screen-space)
 *
 * --presentation-camera keeps the presentation view's own framing instead of the
 * recorded camera, and state.json then records that framing: a way to start a
 * state from a named view without the editor open.
 *
 * --probe=x,y (repeatable) lists every CAD stroke segment passing within 2.5 px of that pixel: the part, its
 * B-rep edge, feature and step flags, world end points, and its window depth there against the captured surface's,
 * with the view-depth gap in mm (negative: the segment lies in front of the surface). The gap compares the surface at
 * the probe pixel with the segment at its nearest point, up to 2.5 px away: on an oblique surface that offset alone
 * moves the gap by about a pixel's worth of slope, so read it as identification, not as a visibility verdict.
 *
 * --screen-edges draws opaque parts' edges from the visible surfaces (the editor's Screen-Space Edges setting).
 *
 * --bounds=text (repeatable) prints the world bounding box of every part whose id contains `text`, as displayed.
 *
 * Exits non-zero when the focus step exceeds --max-focus-step-px (default the
 * camera's own target) or, if given, the largest step exceeds --max-step-px.
 *
 *   xvfb-run -a haxeon/scripts/haxeon run --project app/tests/haxeon.depth-capture.json -- \
 *     --state=view.json --out=DIR [--max-focus-step-px=F] [--max-step-px=F] [editor launch args]
 */
@:access(app.Main) @:access(app.Main.ReferenceEditorApp) @:access(app.EditorPerspectiveViewport) class DepthCapture {
  static inline var TIMEOUT_SECONDS = 900.0;

  public static function main():Int {
    var statePath:Null<String> = null, out:Null<String> = null, launch:Array<String> = [];
    var focusLimit = DepthPrecision.FOCUS_STEP_PIXELS * 1.01, maxLimit = Math.POSITIVE_INFINITY;
    var presentationCamera = false, screenEdges = false;
    var probes:Array<Array<Int>> = [];
    var boundsFilters:Array<String> = [];
    for (arg in Sys.args()) {
      if (StringTools.startsWith(arg, "--state=")) statePath = arg.substr(8);
      else if (StringTools.startsWith(arg, "--out=")) out = arg.substr(6);
      else if (StringTools.startsWith(arg, "--max-focus-step-px=")) focusLimit = Std.parseFloat(arg.substr(20));
      else if (StringTools.startsWith(arg, "--max-step-px=")) maxLimit = Std.parseFloat(arg.substr(14));
      else if (arg == "--presentation-camera") presentationCamera = true;
      else if (arg == "--screen-edges") screenEdges = true;
      else if (StringTools.startsWith(arg, "--bounds=")) boundsFilters.push(arg.substr(9));
      else if (StringTools.startsWith(arg, "--probe=")) {
        var xy = arg.substr(8).split(",");
        probes.push([Std.parseInt(xy[0]), Std.parseInt(xy[1])]);
      }
      else launch.push(arg);
    }
    if (statePath == null || out == null) throw "DepthCapture requires --state=FILE and --out=DIR";
    var state = ViewportStates.decode(sys.io.File.getContent(statePath));
    if (state.project != null) launch.push("--project=" + state.project);
    var outDir:String = out;
    sys.FileSystem.createDirectory(outDir);

    var live = app.Main.open(launch), started = Sys.time();
    var phase = 0, settleFrames = 0, summary:Null<ViewportDepthSummary> = null;
    while (live.tick()) {
      if (Sys.time() - started > TIMEOUT_SECONDS) throw 'DepthCapture timed out in phase $phase';
      var editor = app.Main.liveEditor;
      if (editor == null || editor.openingProject() || editor.projectVariants.isLoading() || editor.documents.blocked()) continue;
      var viewport = editor.perspectiveViewport;
      if (viewport == null) continue;
      switch phase {
        case 0:
          // Open what the state names.
          var variant = state.variant;
          if (variant != null && editor.session.projectVariant != variant) {
            editor.projectVariants.select(variant);
            continue;
          }
          var document = state.document;
          if (document != null) {
            editor.documents.requestOpenPath(document);
            if (editor.documents.error != null) throw editor.documents.error;
          }
          phase = 1;
        case 1:
          if (state.presentation != null && viewport.presentation == null) continue;
          if (presentationCamera && state.presentation == null) throw "--presentation-camera needs a presentation in the state";
          viewport.applyViewState(state);
          viewport.setScreenEdges(screenEdges);
          if (presentationCamera) viewport.pendingViewState = null;
          phase = 2;
        case 2:
          if (viewport.viewStatePending() || (viewport.presentation != null && viewport.presentationCameraDirty)) continue;
          // Let the frame that follows the camera publish its presentation poses.
          if (++settleFrames < 3) continue;
          var width = Std.int(Math.min(2048, Math.ceil(state.width)));
          var height = Std.int(Math.min(2048, Math.ceil(state.height)));
          var depth = viewport.captureDepth(width, height);
          var limit = DepthPrecision.FOCUS_STEP_PIXELS;
          for (probe in probes) probeStrokes(editor, viewport, depth, probe[0], probe[1]);
          for (filter in boundsFilters) for (item in editor.scene.items()) if (item.id.indexOf(filter) >= 0) {
            var t = viewport.displayTransform(item);
            var hx = Math.abs(item.width) / 2, hy = Math.abs(item.height) / 2, hz = Math.abs(item.depth) / 2;
            var c = [t.element(12), t.element(13), t.element(14)];
            var e = [for (r in 0...3) Math.abs(t.element(r)) * hx + Math.abs(t.element(4 + r)) * hy + Math.abs(t.element(8 + r)) * hz];
            Sys.println('bounds ${item.id}: x ${c[0] - e[0]}..${c[0] + e[0]} y ${c[1] - e[1]}..${c[1] + e[1]} z ${c[2] - e[2]}..${c[2] + e[2]}');
          }
          sys.io.File.saveBytes(outDir + "/depth.png", depthImage(depth));
          sys.io.File.saveBytes(outDir + "/steps.png", stepImage(depth, limit));
          sys.io.File.saveBytes(outDir + "/window-depth.f32", depth.windowBytes());
          if (depth.hasSurfaceIds()) sys.io.File.saveBytes(outDir + "/faces.png", faceImage(depth));
          sys.io.File.saveBytes(outDir + "/frame.png", materia.image.Png.rgba(width, height, viewport.captureFrame(width, height)));
          // The same frame without edge strokes separates stroke artefacts from surface ones.
          var edges = editor.scene.cadEdgeMode;
          editor.scene.setCadEdgeMode(Shaded);
          sys.io.File.saveBytes(outDir + "/frame-shaded.png", materia.image.Png.rgba(width, height, viewport.captureFrame(width, height)));
          editor.scene.setCadEdgeMode(edges);
          var result = depth.summary(limit);
          sys.io.File.saveContent(outDir + "/summary.json", haxe.Json.stringify(result, null, "  "));
          var replayed = viewport.viewState();
          replayed.width = state.width;
          replayed.height = state.height;
          if (state.project != null) replayed.project = state.project;
          if (state.variant != null) replayed.variant = state.variant;
          if (state.document != null) replayed.document = state.document;
          sys.io.File.saveContent(outDir + "/state.json", ViewportStates.encode(replayed));
          Sys.println("depth-capture " + haxe.Json.stringify(result));
          summary = result;
          break;
      }
    }
    live.close();
    var result = summary;
    if (result == null) throw "The editor closed before the depth was captured";
    var failed = false;
    if (result.focusStepPixels > focusLimit) {
      Sys.println('depth-capture: focus step ${result.focusStepPixels} px exceeds $focusLimit px');
      failed = true;
    }
    if (result.maxStepPixels > maxLimit) {
      Sys.println('depth-capture: largest step ${result.maxStepPixels} px exceeds $maxLimit px');
      failed = true;
    }
    return failed ? 1 : 0;
  }

  /** Projects every CAD stroke with the capture's camera and reports those near pixel (px, py). */
  static function probeStrokes(editor:app.Main.ReferenceEditorApp, viewport:app.EditorPerspectiveViewport,
      depth:ViewportDepth, px:Int, py:Int):Void {
    var width = depth.width, height = depth.height;
    var shownWidth = viewport.viewportWidth, shownHeight = viewport.viewportHeight;
    viewport.updateViewportSize(width, height);
    viewport.fitCameraClipRange();
    var vp = viewport.camera.viewProjection(width / height);
    viewport.updateViewportSize(shownWidth, shownHeight);
    viewport.fitCameraClipRange();
    var m = [for (i in 0...16) vp.element(i)];
    function clip(p:Array<Float>):Array<Float> return [for (r in 0...4) m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r]];
    var surface = depth.covered(px, py) ? depth.viewDepth(px, py) : Math.POSITIVE_INFINITY;
    Sys.println('probe ($px,$py): surface window ${depth.windowDepth(px, py)} view ${surface} m, object ${depth.objectAt(px, py)} face ${depth.faceAt(px, py)}');
    var scene = editor.scene;
    var scanned = 0, withRuntime = 0, withCad = 0, strokes = 0, nearest = Math.POSITIVE_INFINITY, nearestId = "";
    for (item in scene.items()) {
      scanned++;
      var runtime = try scene.runtimeFor(item.id) catch (_:Dynamic) null;
      if (runtime == null) continue;
      withRuntime++;
      var entry = scene.cadDisplayGeometry.get(haxe.Int64.toStr(runtime.geometry.id().get_value()));
      if (entry == null) continue;
      withCad++;
      var t = viewport.displayTransform(item);
      var tm = [for (i in 0...16) t.element(i)];
      function world(x:Float, y:Float, z:Float):Array<Float>
        return [tm[0] * x + tm[4] * y + tm[8] * z + tm[12], tm[1] * x + tm[5] * y + tm[9] * z + tm[13], tm[2] * x + tm[6] * y + tm[10] * z + tm[14]];
      for (stroke in @:privateAccess entry.data.cadStrokes) {
        var q = stroke.points, a = world(q[0], q[1], q[2]), b = world(q[3], q[4], q[5]);
        var ca = clip(a), cb = clip(b);
        if (ca[3] <= 1e-6 || cb[3] <= 1e-6) continue;
        var ax = (ca[0] / ca[3] + 1) * 0.5 * width, ay = (1 - ca[1] / ca[3]) * 0.5 * height;
        var bx = (cb[0] / cb[3] + 1) * 0.5 * width, by = (1 - cb[1] / cb[3]) * 0.5 * height;
        var dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
        var u = len2 > 0 ? Math.max(0, Math.min(1, ((px + 0.5 - ax) * dx + (py + 0.5 - ay) * dy) / len2)) : 0.0;
        var ex = ax + u * dx - (px + 0.5), ey = ay + u * dy - (py + 0.5);
        strokes++;
        if (Math.sqrt(ex * ex + ey * ey) < nearest) {
          nearest = Math.sqrt(ex * ex + ey * ey);
          nearestId = item.id;
        }
        if (ex * ex + ey * ey > 2.5 * 2.5) continue;
        // Screen-linear u to the segment's own parameter, then its window depth there.
        var s = (u / cb[3]) / ((1 - u) / ca[3] + u / cb[3]);
        var p = [for (k in 0...3) a[k] + s * (b[k] - a[k])], cp = clip(p);
        var window = (cp[2] / cp[3] + 1) * 0.5;
        var ndc = window * 2 - 1, view = 2 * depth.far * depth.near / ((depth.far + depth.near) - ndc * (depth.far - depth.near));
        Sys.println('  ${item.id} edge ${stroke.id} feature ${stroke.feature} step ${stroke.step} '
          + 'from [${a.map(v -> Math.round(v * 1e5) / 1e5)}] to [${b.map(v -> Math.round(v * 1e5) / 1e5)}] '
          + 'off ${Math.round(Math.sqrt(ex * ex + ey * ey) * 100) / 100} px, window ${window}, view gap ${Math.round((view - surface) * 1e6) / 1e3} mm');
      }
    }
    Sys.println('  scanned $scanned items, $withRuntime with nodes, $withCad with CAD strokes, $strokes segments; nearest $nearest px on $nearestId');
  }

  static function depthImage(depth:ViewportDepth):Bytes {
    var summary = depth.summary(DepthPrecision.FOCUS_STEP_PIXELS);
    var nearest = Math.max(1e-12, summary.nearestSurface), ratio = Math.max(1.0 + 1e-9, summary.farthestSurface / nearest);
    var pixels = Bytes.alloc(depth.width * depth.height * 4);
    for (y in 0...depth.height) for (x in 0...depth.width) {
      var index = (y * depth.width + x) * 4;
      if (!depth.covered(x, y)) {
        put(pixels, index, 40, 60, 110);
        continue;
      }
      // Log depth: CAD scenes span decades, and equal ratios read as equal steps.
      var shade = Std.int(255 * (1.0 - 0.85 * Math.log(depth.viewDepth(x, y) / nearest) / Math.log(ratio)));
      put(pixels, index, shade, shade, shade);
    }
    return materia.image.Png.rgba(depth.width, depth.height, pixels);
  }

  /** Log scale around the limit: 0.1x and below green, 1x yellow, 10x and above red. */
  static function stepImage(depth:ViewportDepth, limit:Float):Bytes {
    var pixels = Bytes.alloc(depth.width * depth.height * 4);
    for (y in 0...depth.height) for (x in 0...depth.width) {
      var index = (y * depth.width + x) * 4;
      if (!depth.covered(x, y)) {
        put(pixels, index, 0, 0, 0);
        continue;
      }
      var t = Math.max(0.0, Math.min(1.0, (Math.log(depth.stepPixels(x, y) / limit) / Math.log(10) + 1.0) / 2.0));
      // Green (0) -> yellow (0.5) -> red (1).
      var red = t < 0.5 ? Std.int(510 * t) : 255, green = t < 0.5 ? 200 : Std.int(200 * (1.0 - t) * 2.0);
      put(pixels, index, red, green, 40);
    }
    return materia.image.Png.rgba(depth.width, depth.height, pixels);
  }

  static function faceImage(depth:ViewportDepth):Bytes {
    var pixels = Bytes.alloc(depth.width * depth.height * 4);
    function key(x:Int, y:Int):Int return depth.objectAt(x, y) * 7919 + depth.faceAt(x, y);
    for (y in 0...depth.height) for (x in 0...depth.width) {
      var index = (y * depth.width + x) * 4;
      if (depth.objectAt(x, y) == 0) {
        put(pixels, index, 0, 0, 0);
        continue;
      }
      var k = key(x, y);
      var boundary = (x + 1 < depth.width && key(x + 1, y) != k) || (y + 1 < depth.height && key(x, y + 1) != k);
      if (boundary) {
        put(pixels, index, 20, 20, 20);
        continue;
      }
      // A stable, well-spread colour per face.
      var h = (k * 0x9E3779B1) >>> 8;
      put(pixels, index, 90 + (h & 0x7f), 90 + ((h >> 7) & 0x7f), 90 + ((h >> 14) & 0x7f));
    }
    return materia.image.Png.rgba(depth.width, depth.height, pixels);
  }

  static function put(pixels:Bytes, index:Int, red:Int, green:Int, blue:Int):Void {
    pixels.set(index, red);
    pixels.set(index + 1, green);
    pixels.set(index + 2, blue);
    pixels.set(index + 3, 255);
  }
}
