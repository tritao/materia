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
import nativekit.ui.core.UiKey;
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
  var pointerStartX:Float = 0.0;
  var pointerStartY:Float = 0.0;
  var pointerMoved:Bool = false;
  var objectDrag:Null<PerspectiveSceneDrag> = null;
  var gridSnapEnabled:Bool = false;
  var gridStep:Float = EditorSceneViewport.GRID_STEP;

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
      node.focusable = true;
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
        selected.width, selected.height, selected.depth, aspect());
      return;
    }
    var items = scene.items();
    if (items.length == 0) { camera.reset(); return; }
    var minX = 1000000000.0, minY = 1000000000.0, minZ = 1000000000.0;
    var maxX = -1000000000.0, maxY = -1000000000.0, maxZ = -1000000000.0;
    for (item in items) {
      if (!scene.info(item.id).visible()) continue;
      var transform = scene.info(item.id).worldTransform();
      minX = Math.min(minX, transform.element(12) - item.width / 2);
      maxX = Math.max(maxX, transform.element(12) + item.width / 2);
      minY = Math.min(minY, transform.element(13) - item.height / 2);
      maxY = Math.max(maxY, transform.element(13) + item.height / 2);
      minZ = Math.min(minZ, transform.element(14) - item.depth / 2);
      maxZ = Math.max(maxZ, transform.element(14) + item.depth / 2);
    }
    if (minX == 1000000000.0) { camera.reset(); return; }
    camera.frame((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2,
      maxX - minX, maxY - minY, maxZ - minZ, aspect());
  }

  public function resetView():Void camera.reset();

  public function setPlacementOptions(snap:Bool, step:Float):Void {
    gridSnapEnabled = snap;
    gridStep = step;
  }

  public function dragging():Bool return objectDrag != null;

  public function commitDrag():Null<PerspectivePointer> {
    if (objectDrag == null) return null;
    objectDrag.commit(); objectDrag = null;
    return releaseNavigation();
  }

  public function cancelDrag():Null<PerspectivePointer> {
    if (objectDrag == null) return null;
    objectDrag.cancel(); objectDrag = null;
    return releaseNavigation();
  }

  function releaseNavigation():Null<PerspectivePointer> {
    var pointer = navigationPointer;
    navigationPointer = null; navigationMode = 0;
    return pointer == null ? null : new PerspectivePointer(pointer, pointerX, pointerY);
  }

  public function pick(localX:Float, localY:Float):String {
    return pickScene(scene, camera, Math.max(1, renderedWidth), Math.max(1, renderedHeight),
      localX, localY);
  }

  public static function pickScene(scene:EditorScene, camera:PerspectiveCamera,
      width:Float, height:Float, localX:Float, localY:Float):String {
    var ray = camera.screenRay(localX, localY, width, height);
    var closest = 1000000000.0;
    var result = "scene";
    for (item in scene.items()) {
      var state = scene.info(item.id);
      if (!state.visible()) continue;
      var transform = state.worldTransform();
      var centerX = transform.element(12), centerY = transform.element(13);
      var centerZ = transform.element(14);
      var distance = rayBoxDistance(ray,
        centerX - item.width / 2, centerY - item.height / 2, centerZ - item.depth / 2,
        centerX + item.width / 2, centerY + item.height / 2, centerZ + item.depth / 2);
      if (distance != null && distance < closest) {
        closest = distance;
        result = item.id;
      }
    }
    return result;
  }

  static function rayBoxDistance(ray:PerspectiveRay, minX:Float, minY:Float, minZ:Float,
      maxX:Float, maxY:Float, maxZ:Float):Null<Float> {
    var origins = [ray.originX, ray.originY, ray.originZ];
    var directions = [ray.directionX, ray.directionY, ray.directionZ];
    var minimums = [minX, minY, minZ], maximums = [maxX, maxY, maxZ];
    var near = 0.0, far = 1000000000.0;
    for (axis in 0...3) {
      var direction = directions[axis], origin = origins[axis];
      if (Math.abs(direction) < 0.000001) {
        if (origin < minimums[axis] || origin > maximums[axis]) return null;
      } else {
        var first = (minimums[axis] - origin) / direction;
        var second = (maximums[axis] - origin) / direction;
        if (first > second) { var swap = first; first = second; second = swap; }
        near = Math.max(near, first); far = Math.min(far, second);
        if (near > far) return null;
      }
    }
    return far < 0.0 ? null : near;
  }

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
      if (event.button == 0) {
        var hit = pick(event.localX, event.localY);
        if (hit != "scene") {
          scene.select(hit);
          objectDrag = PerspectiveSceneDrag.begin(scene, camera, hit, event.localX, event.localY,
            Math.max(1, renderedWidth), Math.max(1, renderedHeight), gridSnapEnabled, gridStep);
          if (objectDrag != null) navigationMode = 3;
        }
      }
      pointerX = event.x; pointerY = event.y;
      pointerStartX = event.x; pointerStartY = event.y; pointerMoved = false;
      event.capturePointer(); event.preventDefault(); event.stopPropagation();
    });
    node.on(UiEventKind.PointerMove, function(event:UiEvent) {
      if (navigationPointer == null || event.pointerId != navigationPointer) return;
      var deltaX = event.x - pointerX, deltaY = event.y - pointerY;
      pointerX = event.x; pointerY = event.y;
      if (Math.abs(event.x - pointerStartX) >= 3.0 || Math.abs(event.y - pointerStartY) >= 3.0)
        pointerMoved = true;
      if (navigationMode == 1) camera.orbit(deltaX, deltaY);
      else if (navigationMode == 2) camera.pan(deltaX, deltaY, Math.max(1, renderedHeight));
      else if (objectDrag != null) objectDrag.update(camera, event.localX, event.localY,
        Math.max(1, renderedWidth), Math.max(1, renderedHeight));
      event.preventDefault(); event.stopPropagation();
    });
    var finish = function(event:UiEvent) {
      if (navigationPointer == null || event.pointerId != navigationPointer) return;
      if (navigationMode == 3 && objectDrag != null) {
        if (event.kind == UiEventKind.PointerUp) objectDrag.commit(); else objectDrag.cancel();
        objectDrag = null;
      } else if (event.kind == UiEventKind.PointerUp && navigationMode == 1 && !pointerMoved)
        scene.select(pick(event.localX, event.localY));
      navigationPointer = null; navigationMode = 0;
      event.releasePointer(); event.preventDefault(); event.stopPropagation();
    };
    node.on(UiEventKind.PointerUp, finish);
    node.on(UiEventKind.PointerCancel, finish);
    node.on(UiEventKind.Scroll, function(event:UiEvent) {
      camera.zoom(event.deltaY);
      event.preventDefault(); event.stopPropagation();
    });
    node.on(UiEventKind.KeyDown, function(event:UiEvent) {
      if (event.key != UiKey.Escape || objectDrag == null) return;
      objectDrag.cancel(); objectDrag = null;
      navigationPointer = null; navigationMode = 0;
      event.releasePointer(); event.preventDefault(); event.stopPropagation();
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

class PerspectivePointer {
  public final id:Int;
  public final x:Float;
  public final y:Float;
  public function new(id:Int, x:Float, y:Float) {
    this.id = id; this.x = x; this.y = y;
  }
}
