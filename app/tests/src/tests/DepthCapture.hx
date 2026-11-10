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
 *
 * --presentation-camera keeps the presentation view's own framing instead of the
 * recorded camera, and state.json then records that framing: a way to start a
 * state from a named view without the editor open.
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
    var presentationCamera = false;
    for (arg in Sys.args()) {
      if (StringTools.startsWith(arg, "--state=")) statePath = arg.substr(8);
      else if (StringTools.startsWith(arg, "--out=")) out = arg.substr(6);
      else if (StringTools.startsWith(arg, "--max-focus-step-px=")) focusLimit = Std.parseFloat(arg.substr(20));
      else if (StringTools.startsWith(arg, "--max-step-px=")) maxLimit = Std.parseFloat(arg.substr(14));
      else if (arg == "--presentation-camera") presentationCamera = true;
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
          sys.io.File.saveBytes(outDir + "/depth.png", depthImage(depth));
          sys.io.File.saveBytes(outDir + "/steps.png", stepImage(depth, limit));
          sys.io.File.saveBytes(outDir + "/window-depth.f32", depth.windowBytes());
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

  static function put(pixels:Bytes, index:Int, red:Int, green:Int, blue:Int):Void {
    pixels.set(index, red);
    pixels.set(index + 1, green);
    pixels.set(index + 2, blue);
    pixels.set(index + 3, 255);
  }
}
