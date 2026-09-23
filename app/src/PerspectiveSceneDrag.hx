package app;

/** One previewed perspective move constrained to the object's world-Z plane. */
class PerspectiveSceneDrag {
  final scene:EditorScene;
  final id:String;
  final planeZ:Float;
  final offsetX:Float;
  final offsetY:Float;
  final startX:Float;
  final startY:Float;
  final snap:Bool;
  final gridStep:Float;
  var currentX:Float;
  var currentY:Float;
  var finished:Bool = false;

  public static function begin(scene:EditorScene, camera:PerspectiveCamera, id:String,
      x:Float, y:Float, width:Float, height:Float, snap:Bool, gridStep:Float):Null<PerspectiveSceneDrag> {
    var item = scene.object(id);
    if (item == null) return null;
    var point = camera.intersectPlaneZ(x, y, width, height, item.z);
    if (point == null) return null;
    return new PerspectiveSceneDrag(scene, id, item.z, item.x, item.y,
      item.x - point.x, item.y - point.y, snap, gridStep);
  }

  function new(scene:EditorScene, id:String, planeZ:Float, startX:Float, startY:Float,
      offsetX:Float, offsetY:Float, snap:Bool, gridStep:Float) {
    this.scene = scene; this.id = id; this.planeZ = planeZ;
    this.startX = startX; this.startY = startY;
    this.offsetX = offsetX; this.offsetY = offsetY;
    this.snap = snap; this.gridStep = gridStep;
    currentX = startX; currentY = startY;
  }

  public function update(camera:PerspectiveCamera, x:Float, y:Float,
      width:Float, height:Float):Bool {
    if (finished) return false;
    var point = camera.intersectPlaneZ(x, y, width, height, planeZ);
    if (point == null) return false;
    var nextX = point.x + offsetX, nextY = point.y + offsetY;
    if (snap) {
      nextX = Math.round(nextX / gridStep) * gridStep;
      nextY = Math.round(nextY / gridStep) * gridStep;
    }
    nextX = Math.max(-1000000.0, Math.min(1000000.0, nextX));
    nextY = Math.max(-1000000.0, Math.min(1000000.0, nextY));
    if (nextX == currentX && nextY == currentY) return false;
    scene.setPositionXY(id, nextX, nextY);
    currentX = nextX; currentY = nextY;
    return true;
  }

  public function commit():Bool {
    if (finished) return false;
    finished = true;
    return scene.recordMove(id, startX, startY, currentX, currentY);
  }

  public function cancel():Bool {
    if (finished) return false;
    finished = true;
    if (currentX != startX || currentY != startY)
      scene.setPositionXY(id, startX, startY);
    return true;
  }
}
