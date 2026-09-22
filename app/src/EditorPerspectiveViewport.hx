package app;

import Canvas;
import Color;
import Image;
import ImageFormat;
import LayoutAxis;
import LayoutStyle;
import LayoutVisualKind;
import Rect;
import ResolvedLayoutItem;
import nativekit.scene.SceneRenderer;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;
import nativekit.ffi.NativeKitGpu;
import nativekit.ffi.NativeKitTypes;
import nativekit.ui.core.BuildContext;
import nativekit.ui.core.Key;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.View;
import nativekit.ui.semantics.AccessibilityRole;
import nativekit.ui.semantics.Semantics;

/** Fixed-camera SceneKit GPU render composited into the editor UI. */
class EditorPerspectiveViewport implements View {
  public final key:String;
  final scene:EditorScene;
  final surface:SurfaceHandle;
  var renderer:Null<SceneRenderer> = null;
  var gpu:Null<nkgpu_renderer> = null;
  final style:LayoutStyle;
  var image:Null<Image> = null;
  var renderedRevision:Int = -1;
  var renderedWidth:Int = 0;
  var renderedHeight:Int = 0;

  public function new(key:String, scene:EditorScene, surface:SurfaceHandle, ?style:LayoutStyle) {
    this.key = key;
    this.scene = scene;
    this.surface = surface;
    this.style = style == null ? defaultStyle() : style.copy();
  }

  public function build(context:BuildContext):RenderNode {
    return context.withScope(new Key(key), function() {
      var node = new RenderNode(context.id("perspective"), LayoutVisualKind.Custom, style);
      node.hitTestSelf = true;
      node.semantics = new Semantics(AccessibilityRole.Image,
        "Scene perspective GPU view");
      node.onPaint(paint, "perspective:" + scene.revision + ":" + renderedWidth + "x" + renderedHeight);
      return node;
    });
  }

  function paint(canvas:Canvas, geometry:ResolvedLayoutItem):Void {
    ensureRenderer();
    var width = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.width))));
    var height = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.height))));
    if (image == null || renderedRevision != scene.revision ||
        width != renderedWidth || height != renderedHeight) {
      var view = scene.configureRenderView(new SceneView(), fixedViewProjection(width / height));
      var pixels = renderer.captureRgba8(scene.renderSnapshot(), view, width, height);
      var next = Image.create(width, height, ImageFormat.RGBA8, pixels);
      if (image != null) image.dispose();
      image = next;
      renderedRevision = scene.revision;
      renderedWidth = width;
      renderedHeight = height;
    }
    canvas.fillRect(new Rect(0, 0, geometry.width, geometry.height),
      Color.rgba(0.025, 0.035, 0.055, 1.0));
    if (image != null) canvas.drawImage(image, new Rect(0, 0, geometry.width, geometry.height));
  }

  public function dispose():Void {
    if (image != null) image.dispose();
    image = null;
    if (renderer != null) renderer.dispose();
    renderer = null;
    if (gpu != null) checkGpu(NativeKitGpu.nkgpu_renderer_destroy(gpu),
      "perspective.renderer.destroy");
    gpu = null;
  }

  function ensureRenderer():Void {
    if (renderer != null) return;
    var made = NativeKitGpu.nkgpu_renderer_create(surface);
    checkGpu(made.status, "perspective.renderer.create");
    gpu = made.out_renderer;
    renderer = SceneRenderer.createBorrowed(made.out_renderer);
  }

  static function checkGpu(status:Int, operation:String):Void {
    if (status != 0)
      throw '$operation failed with NativeKit GPU status $status: ${NativeKitGpu.nkgpu_last_error()}';
  }

  static function fixedViewProjection(aspect:Float):Transform {
    var eye = [4.5, -6.5, 5.0];
    var forward = normalize([-eye[0], -eye[1], -eye[2]]);
    var side = normalize(cross(forward, [0.0, 0.0, 1.0]));
    var up = cross(side, forward);
    var view = [
      side[0], up[0], -forward[0], 0.0,
      side[1], up[1], -forward[1], 0.0,
      side[2], up[2], -forward[2], 0.0,
      -dot(side, eye), -dot(up, eye), dot(forward, eye), 1.0
    ];
    var near = 0.05;
    var far = 100.0;
    var scale = 1.0 / Math.tan(50.0 * Math.PI / 360.0);
    var projection = [
      scale / aspect, 0.0, 0.0, 0.0,
      0.0, scale, 0.0, 0.0,
      0.0, 0.0, (far + near) / (near - far), -1.0,
      0.0, 0.0, 2.0 * far * near / (near - far), 0.0
    ];
    var combined:Array<Float> = [];
    for (column in 0...4) for (row in 0...4) {
      var value = 0.0;
      for (index in 0...4) value += projection[index * 4 + row] * view[column * 4 + index];
      combined.push(value);
    }
    var result = Transform.identity();
    for (index in 0...16) result.set(index, combined[index]);
    return result;
  }

  static function normalize(value:Array<Float>):Array<Float> {
    var length = Math.pow(dot(value, value), 0.5);
    return [value[0] / length, value[1] / length, value[2] / length];
  }
  static function dot(left:Array<Float>, right:Array<Float>):Float
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
  static function cross(left:Array<Float>, right:Array<Float>):Array<Float>
    return [left[1] * right[2] - left[2] * right[1],
      left[2] * right[0] - left[0] * right[2],
      left[0] * right[1] - left[1] * right[0]];

  static function defaultStyle():LayoutStyle {
    var result = new LayoutStyle();
    result.width = LayoutAxis.stretch();
    result.height = LayoutAxis.stretch();
    result.clipToParent = true;
    return result;
  }
}
