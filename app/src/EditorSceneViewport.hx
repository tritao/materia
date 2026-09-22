package app;

import Canvas;
import Color;
import Rect;
import PathBuilder;
import nativekit.ui.core.ViewportContent;
import nativekit.ui.core.ViewportCamera;

/** XY orthographic presentation of the editor's planar SceneKit meshes. */
class EditorSceneViewport implements ViewportContent {
  public static inline var SCALE:Float = 100.0;
  public static inline var ORIGIN_X:Float = 320.0;
  public static inline var ORIGIN_Y:Float = 260.0;
  public static inline var GRID_STEP:Float = 0.2;
  final scene:EditorScene;
  var drag:Null<EditorSceneDrag> = null;
  public var gridStep(default, null):Float = GRID_STEP;
  public var viewportWidth:Float = 640.0;
  public var viewportHeight:Float = 520.0;

  public function new(scene:EditorScene) this.scene = scene;
  public function width():Float return 960.0;
  public function height():Float return 640.0;
  public function revision():Int return scene.revision;

  public function pick(camera:ViewportCamera, x:Float, y:Float):String {
    var point = scenePoint(camera, x, y);
    return scene.pick(point.x, point.y);
  }

  public function dragging():Bool return drag != null;

  public function setGridStep(value:Float):Bool {
    if (value != value || value - value != 0.0 || value <= 0.0 || value > 1000000.0)
      throw "Grid spacing must be finite and positive";
    if (value == gridStep) return false;
    gridStep = value;
    return true;
  }

  /** Begins a primary-button move while retaining the pointer-to-origin offset. */
  public function beginDrag(camera:ViewportCamera, x:Float, y:Float, snap:Bool):Bool {
    if (drag != null) cancelDrag();
    var point = scenePoint(camera, x, y);
    var id = scene.pick(point.x, point.y);
    if (scene.object(id) == null) return false;
    scene.select(id);
    var transform = scene.info(id).localTransform();
    drag = new EditorSceneDrag(id, transform.element(12), transform.element(13),
      transform.element(12) - point.x, transform.element(13) - point.y, snap);
    return true;
  }

  public function updateDrag(camera:ViewportCamera, x:Float, y:Float, constrain:Bool = false):Bool {
    var active = drag;
    if (active == null) return false;
    var point = scenePoint(camera, x, y);
    var nextX = point.x + active.offsetX;
    var nextY = point.y + active.offsetY;
    if (constrain) {
      if (active.constraintAxis < 0)
        active.constraintAxis = Math.abs(nextX - active.startX) >= Math.abs(nextY - active.startY) ? 0 : 1;
      if (active.constraintAxis == 0) nextY = active.startY;
      else nextX = active.startX;
    }
    if (active.snap) {
      nextX = Math.round(nextX / gridStep) * gridStep;
      nextY = Math.round(nextY / gridStep) * gridStep;
    }
    nextX = Math.max(-1000000.0, Math.min(1000000.0, nextX));
    nextY = Math.max(-1000000.0, Math.min(1000000.0, nextY));
    if (nextX == active.currentX && nextY == active.currentY) return false;
    scene.setPositionXY(active.id, nextX, nextY);
    active.currentX = nextX;
    active.currentY = nextY;
    return true;
  }

  public function commitDrag():Bool {
    var active = drag;
    if (active == null) return false;
    drag = null;
    return scene.recordMove(active.id, active.startX, active.startY,
      active.currentX, active.currentY);
  }

  public function cancelDrag():Bool {
    var active = drag;
    if (active == null) return false;
    drag = null;
    if (active.currentX != active.startX || active.currentY != active.startY)
      scene.setPositionXY(active.id, active.startX, active.startY);
    return true;
  }

  function scenePoint(camera:ViewportCamera, x:Float, y:Float):Point {
    var point = camera.viewportToWorld(x, y);
    return new Point((point.x - ORIGIN_X) / SCALE, (ORIGIN_Y - point.y) / SCALE);
  }

  public function frameSelected(camera:ViewportCamera):Void {
    var item = scene.object(scene.selectedId);
    if (item == null) return;
    var state = scene.info(item.id).worldTransform();
    camera.fit(item.width * SCALE, item.height * SCALE, viewportWidth, viewportHeight, 48.0);
    camera.setPan(ORIGIN_X + state.element(12) * SCALE - viewportWidth / camera.zoom / 2,
      ORIGIN_Y - state.element(13) * SCALE - viewportHeight / camera.zoom / 2);
  }

  public function paint(canvas:Canvas, destination:Rect):Void {
    var ordered = scene.items();
    ordered.sort(function(a, b) {
      var first = scene.info(a.id).worldTransform().element(14);
      var second = scene.info(b.id).worldTransform().element(14);
      return first < second ? -1 : first > second ? 1 : 0;
    });
    for (item in ordered) {
      var state = scene.info(item.id);
      if (!state.visible()) continue;
      var transform = state.worldTransform();
      var rect = new Rect(ORIGIN_X + (transform.element(12) - item.width / 2) * SCALE,
        ORIGIN_Y - (transform.element(13) + item.height / 2) * SCALE,
        item.width * SCALE, item.height * SCALE);
      canvas.fillRect(rect, Color.rgba(item.red, item.green, item.blue, 1.0));
      if (scene.selectedId == item.id) {
        var outline = new PathBuilder().moveTo(rect.x, rect.y)
          .lineTo(rect.x + rect.width, rect.y).lineTo(rect.x + rect.width, rect.y + rect.height)
          .lineTo(rect.x, rect.y + rect.height).close().build();
        canvas.strokeTransient(outline, Color.rgba(1.0, 0.88, 0.35, 1.0), 3.0);
      }
    }
  }
}

private class EditorSceneDrag {
  public final id:String;
  public final startX:Float;
  public final startY:Float;
  public final offsetX:Float;
  public final offsetY:Float;
  public final snap:Bool;
  public var currentX:Float;
  public var currentY:Float;
  public var constraintAxis:Int = -1;
  public function new(id:String, startX:Float, startY:Float, offsetX:Float, offsetY:Float, snap:Bool) {
    this.id = id; this.startX = startX; this.startY = startY;
    this.offsetX = offsetX; this.offsetY = offsetY; this.snap = snap;
    currentX = startX; currentY = startY;
  }
}
