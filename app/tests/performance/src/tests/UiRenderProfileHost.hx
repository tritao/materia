package tests;

import haxe.Json;
import haxeon.platform.Runtime;
import haxeon.platform.Surface as PlatformSurface;
import haxeon.ui.Surface;
import haxeon.platform.SurfaceOptions;
import haxeon.platform.WindowOptions;
import haxeon.ui.FrameInfo;
import haxeon.ui.LayoutFrame;
import haxeon.ui.Renderer;
import haxeon.ui.core.UiContext;
import haxeon.ui.debug.UiFrameMetrics;
import nativekit.ffi.NativeKitTypes.WindowFlags;
import nativekit.ffi.NativeKitTypes.GraphicsApi;
import nativekit.ffi.NativeKitTypes.SurfaceFlags;
import nativekit.ffi.NativeKitTypes.Result;
import nativekit.ffi.NativeKitTypes.SurfaceFrameMode;
import nativekit.ffi.NativeKit;
import sys.io.File;

/** Optional real GPU submission for the otherwise headless editor scenarios. */
class UiRenderProfileHost {
  final runtime:Runtime;
  final surface:Surface;
  final nativeSurface:PlatformSurface;
  final renderer:Renderer;
  final frameInfo:FrameInfo;
  final frames:Array<String> = [];
  var rendered = false;
  var started = 0.0;
  var ended = 0.0;
  var waitSeconds = 0.0;
  var pending:Null<UiContext> = null;
  var completed = false;
  var failure:Dynamic = null;

  function new(runtime:Runtime, nativeSurface:PlatformSurface, surface:Surface, renderer:Renderer, width:Int, height:Int,
      framebufferWidth:Int, framebufferHeight:Int, pixelScale:Float) {
    this.runtime = runtime;
    this.surface = surface;
    this.nativeSurface = nativeSurface;
    this.renderer = renderer;
    frameInfo = new FrameInfo(width, height, framebufferWidth, framebufferHeight, pixelScale);
    NativeKit.nk_surface_set_frame_mode_checked(nativeSurface.nativeHandle(), SurfaceFrameMode.OnDemand);
    nativeSurface.onFrame(onFrame);
  }

  public static function create(width:Int, height:Int):UiRenderProfileHost {
    var runtime = Runtime.start();
    var renderer:Null<Renderer> = null;
    try {
      var options = new WindowOptions(width, height);
      options.title = "UI render profile (temporary)";
      options.flags = WindowFlags.Borderless;
      var window = runtime.createWindow(options);
      var surfaceOptions = new SurfaceOptions(width, height);
      surfaceOptions.api = GraphicsApi.Opengl;
      surfaceOptions.flags = SurfaceFlags.ForwardCompatible | SurfaceFlags.Stencil;
      surfaceOptions.majorVersion = 3;
      surfaceOptions.minorVersion = 3;
      var nativeSurface = window.createSurface(surfaceOptions);
      var surface = Surface.fromNativeSurface(nativeSurface);
      var scale = NativeKit.nk_window_get_scale(window.nativeHandle());
      if (scale.status != Result.Ok || scale.out_scale <= 0.0) throw "GPU profiling requires a window scale";
      var size = NativeKit.nk_surface_get_framebuffer_size(nativeSurface.nativeHandle());
      var deadline = Sys.time() + 5.0;
      while (size.status == Result.Ok &&
          (size.out_width != Std.int(width * scale.out_scale) || size.out_height != Std.int(height * scale.out_scale)) &&
          Sys.time() < deadline) {
        while (runtime.events.poll()) {}
        runtime.events.wait(0.001);
        size = NativeKit.nk_surface_get_framebuffer_size(nativeSurface.nativeHandle());
      }
      if (size.status != Result.Ok || size.out_width != Std.int(width * scale.out_scale) ||
          size.out_height != Std.int(height * scale.out_scale)) throw "GPU profiling requires the full-sized framebuffer";
      renderer = Renderer.create();
      return new UiRenderProfileHost(runtime, nativeSurface, surface, renderer, width, height,
        size.out_width, size.out_height, scale.out_scale);
    } catch (error:Dynamic) {
      if (renderer != null) renderer.dispose();
      runtime.dispose();
      throw error;
    }
  }

  public function render(ui:UiContext, frame:LayoutFrame):Void {
    var metrics = ui.frameMetrics;
    rendered = metrics != null && (!metrics.reusedSubmission || metrics.hoverPaintUpdatedNodes > 0);
    waitSeconds = 0.0;
    if (!rendered) return;
    frameInfo.set(frame.width, frame.height, frameInfo.framebufferWidth, frameInfo.framebufferHeight, frameInfo.pixelScale);
    pending = ui;
    completed = false;
    failure = null;
    var requested = Sys.time();
    NativeKit.nk_surface_request_frame_checked(nativeSurface.nativeHandle());
    while (!completed && Sys.time() - requested < 30.0) {
      while (runtime.events.poll()) {}
      if (!completed) runtime.events.wait(0.001);
    }
    if (!completed) {pending = null; throw "GPU profiling did not receive a surface frame";}
    waitSeconds = Sys.time() - requested - (ended - started);
    if (failure != null) throw failure;
  }

  function onFrame(width:Int, height:Int):Void {
    var ui = pending;
    if (ui == null) return;
    pending = null;
    frameInfo.set(frameInfo.logicalWidth, frameInfo.logicalHeight, width, height, frameInfo.pixelScale);
    started = Sys.time();
    try ui.render(renderer, surface, frameInfo)
    catch (error:Dynamic) failure = error;
    ended = Sys.time();
    completed = true;
  }

  /** Serialize after the action's allocation and time counters have been sampled. */
  public function record(action:String, cycle:Null<Int>, frameStarted:Float, frameSeconds:Float,
      metrics:UiFrameMetrics):Void {
    frames.push(Json.stringify({
      action: action, cycle: cycle, frameStartedAtSeconds: frameStarted, frameSeconds: frameSeconds,
      rendered: rendered, renderStartedAtSeconds: rendered ? started : null,
      renderSeconds: rendered ? ended - started : 0.0,
      surfaceWaitSeconds: waitSeconds, activeFrameSeconds: frameSeconds - waitSeconds,
      customPaintSeconds: rendered ? metrics.customPaintSeconds : 0.0,
      nativeRenderSeconds: rendered ? metrics.nativeRenderSeconds : 0.0,
      customPaintAllocatedBytes: rendered ? metrics.customPaintAllocatedBytes : 0.0,
      nativeRenderAllocatedBytes: rendered ? metrics.nativeRenderAllocatedBytes : 0.0,
      paintedNodes: rendered ? metrics.paintedNodes : 0,
      paintSkippedNodes: rendered ? metrics.paintSkippedNodes : 0
    }));
  }

  public function save(path:String):Void {
    File.saveContent(path, frames.join("\n") + "\n");
    File.saveContent(path + ".config.json", Json.stringify({
      logicalWidth: frameInfo.logicalWidth, logicalHeight: frameInfo.logicalHeight,
      framebufferWidth: frameInfo.framebufferWidth, framebufferHeight: frameInfo.framebufferHeight,
      pixelScale: frameInfo.pixelScale, planReuse: renderer.planReuse(),
      api: "OpenGL", hidden: false, boundary: "GPU submission in the surface frame callback, without 3D viewport rendering"
    }));
  }

  public function dispose():Void {
    pending = null;
    renderer.dispose();
    runtime.dispose();
  }
}
