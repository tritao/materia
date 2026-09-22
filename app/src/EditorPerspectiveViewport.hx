package app;

import Canvas;
import Color;
import GraphicsSurface;
import LayoutAxis;
import LayoutStyle;
import LayoutVisualKind;
import Rect;
import ResolvedLayoutItem;
import nativekit.scene.SceneRenderer;
import nativekit.scene.SceneView;
import nativekit.ui.core.BuildContext;
import nativekit.ui.core.Key;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.UiEvent;
import nativekit.ui.core.UiEventKind;
import nativekit.ui.core.View;
import nativekit.ui.host.DesktopUiHostContext;
import nativekit.ui.semantics.AccessibilityRole;
import nativekit.ui.semantics.Semantics;

/** Fixed-camera SceneKit GPU render composited into the editor UI. */
class EditorPerspectiveViewport implements View {
  public final key:String;
  final scene:EditorScene;
  final host:DesktopUiHostContext;
  var renderer:Null<SceneRenderer> = null;
  final style:LayoutStyle;
  var surface:Null<GraphicsSurface> = null;
  var renderedRevision:Int = -1;
  var renderedWidth:Int = 0;
  var renderedHeight:Int = 0;
  var renderedCameraRevision:Int = -1;
  var renderCount:Int = 0;
  var lastRenderSeconds:Float = 0.0;
  var totalRenderSeconds:Float = 0.0;
  final camera:PerspectiveCamera = new PerspectiveCamera();
  var navigationPointer:Null<Int> = null;
  var navigationMode:Int = 0;
  var pointerX:Float = 0.0;
  var pointerY:Float = 0.0;

  public function new(key:String, scene:EditorScene, host:DesktopUiHostContext, ?style:LayoutStyle) {
    this.key = key;
    this.scene = scene;
    this.host = host;
    this.style = style == null ? defaultStyle() : style.copy();
  }

  public function build(context:BuildContext):RenderNode {
    return context.withScope(new Key(key), function() {
      var node = new RenderNode(context.id("perspective"), LayoutVisualKind.Custom, style);
      node.hitTestSelf = true;
      node.semantics = new Semantics(AccessibilityRole.Image,
        "Scene perspective GPU view");
      node.onPaint(paint, "perspective:" + scene.revision + ":" + camera.revision + ":" +
        renderedWidth + "x" + renderedHeight);
      installNavigation(node);
      return node;
    });
  }

  function paint(canvas:Canvas, geometry:ResolvedLayoutItem):Void {
    ensureRenderer();
    var width = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.width))));
    var height = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.height))));
    if (renderer != null && (surface == null || renderedRevision != scene.revision ||
        renderedCameraRevision != camera.revision ||
        width != renderedWidth || height != renderedHeight)) {
      var started = Sys.time();
      var view = scene.configureRenderView(new SceneView(), camera.viewProjection(width / height));
      var rendered = renderer.renderImage(scene.renderSnapshot(), view, width, height);
      var next = GraphicsSurface.fromImage(rendered);
      rendered.dispose();
      if (surface != null) surface.dispose();
      surface = next;
      renderedRevision = scene.revision;
      renderedCameraRevision = camera.revision;
      renderedWidth = width;
      renderedHeight = height;
      lastRenderSeconds = Sys.time() - started;
      totalRenderSeconds += lastRenderSeconds;
      renderCount++;
    }
    canvas.fillRect(new Rect(0, 0, geometry.width, geometry.height),
      Color.rgba(0.025, 0.035, 0.055, 1.0));
    if (surface != null) canvas.drawSurface(surface, new Rect(0, 0, geometry.width, geometry.height));
  }

  public function diagnosticState():Dynamic return {
    width: renderedWidth,
    height: renderedHeight,
    composition: "gpu-surface",
    renders: renderCount,
    cpuTransferBytes: 0,
    lastRenderMilliseconds: lastRenderSeconds * 1000.0,
    averageRenderMilliseconds: renderCount == 0 ? 0.0 : totalRenderSeconds * 1000.0 / renderCount,
    camera: {targetX: camera.targetX, targetY: camera.targetY, targetZ: camera.targetZ,
      yaw: camera.yaw, pitch: camera.pitch, distance: camera.distance}
  };

  public function frameSelected():Void {
    var selected = scene.object(scene.selectedId);
    if (selected != null) {
      var transform = scene.info(selected.id).worldTransform();
      camera.frame(transform.element(12), transform.element(13), transform.element(14),
        selected.width, selected.height, 0.05, aspect());
      return;
    }
    var items = scene.items();
    if (items.length == 0) { camera.reset(); return; }
    var minX = 1000000000.0, minY = 1000000000.0;
    var maxX = -1000000000.0, maxY = -1000000000.0;
    for (item in items) {
      if (!scene.info(item.id).visible()) continue;
      var transform = scene.info(item.id).worldTransform();
      minX = Math.min(minX, transform.element(12) - item.width / 2);
      maxX = Math.max(maxX, transform.element(12) + item.width / 2);
      minY = Math.min(minY, transform.element(13) - item.height / 2);
      maxY = Math.max(maxY, transform.element(13) + item.height / 2);
    }
    if (minX == 1000000000.0) { camera.reset(); return; }
    camera.frame((minX + maxX) / 2, (minY + maxY) / 2, 0.0,
      maxX - minX, maxY - minY, 0.05, aspect());
  }

  public function resetView():Void camera.reset();

  public function dispose():Void {
    if (surface != null) surface.dispose();
    surface = null;
    if (renderer != null) renderer.dispose();
    renderer = null;
  }

  function ensureRenderer():Void {
    if (renderer != null || host.gpuRendererId == 0) return;
    renderer = SceneRenderer.createBorrowedId(host.gpuRendererId);
  }

  function aspect():Float return renderedWidth <= 0 || renderedHeight <= 0
    ? 1.0 : renderedWidth / renderedHeight;

  function installNavigation(node:RenderNode):Void {
    node.on(UiEventKind.PointerDown, function(event:UiEvent) {
      if (event.button != 0 && event.button != 2) return;
      navigationPointer = event.pointerId;
      navigationMode = event.button == 0 ? 1 : 2;
      pointerX = event.x; pointerY = event.y;
      event.capturePointer(); event.preventDefault(); event.stopPropagation();
    });
    node.on(UiEventKind.PointerMove, function(event:UiEvent) {
      if (navigationPointer == null || event.pointerId != navigationPointer) return;
      var deltaX = event.x - pointerX, deltaY = event.y - pointerY;
      pointerX = event.x; pointerY = event.y;
      if (navigationMode == 1) camera.orbit(deltaX, deltaY);
      else camera.pan(deltaX, deltaY, Math.max(1, renderedHeight));
      event.preventDefault(); event.stopPropagation();
    });
    var finish = function(event:UiEvent) {
      if (navigationPointer == null || event.pointerId != navigationPointer) return;
      navigationPointer = null; navigationMode = 0;
      event.releasePointer(); event.preventDefault(); event.stopPropagation();
    };
    node.on(UiEventKind.PointerUp, finish);
    node.on(UiEventKind.PointerCancel, finish);
    node.on(UiEventKind.Scroll, function(event:UiEvent) {
      camera.zoom(event.deltaY);
      event.preventDefault(); event.stopPropagation();
    });
  }

  static function defaultStyle():LayoutStyle {
    var result = new LayoutStyle();
    result.width = LayoutAxis.stretch();
    result.height = LayoutAxis.stretch();
    result.clipToParent = true;
    return result;
  }
}
