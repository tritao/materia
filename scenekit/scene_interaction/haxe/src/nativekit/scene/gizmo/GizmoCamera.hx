package nativekit.scene.gizmo;

/** Frozen perspective pose, suitable for capture from PerspectiveCamera's public eye/target/FOV. */
class GizmoCamera {
  public final eye:GizmoVec3;
  public final forward:GizmoVec3;
  public final right:GizmoVec3;
  public final up:GizmoVec3;
  public final width:Float;
  public final height:Float;
  public final near:Float;
  public final far:Float;
  final tangent:Float;
  public function new(eye:GizmoVec3, target:GizmoVec3, worldUp:GizmoVec3, width:Float, height:Float,
      fovDegrees:Float = 50, near:Float = 0.000001, far:Float = 1e300) {
    if (eye == null || target == null || worldUp == null || !Math.isFinite(width) || !Math.isFinite(height) || width <= 0 || height <= 0
      || !Math.isFinite(fovDegrees) || fovDegrees <= 0 || fovDegrees >= 179 || !Math.isFinite(near) || !Math.isFinite(far) || near <= 0 || far <= near)
      throw "Invalid gizmo camera snapshot";
    this.eye = eye; this.width = width; this.height = height; this.near = near; this.far = far;
    forward = target.subtract(eye).normalized(); right = forward.cross(worldUp.normalized()).normalized(); up = right.cross(forward).normalized();
    tangent = Math.tan(fovDegrees*Math.PI/360);
  }
  public function depth(point:GizmoVec3):Float return point.subtract(eye).dot(forward);
  public function worldLength(point:GizmoVec3, pixels:Float):Float return 2*depth(point)*tangent*pixels/height;
  public function project(point:GizmoVec3):Null<GizmoPoint> {
    var relative = point.subtract(eye), d = relative.dot(forward);
    if (!Math.isFinite(d) || d < near || d > far) return null;
    var scale = height/(2*tangent*d), x = width*0.5+relative.dot(right)*scale, y = height*0.5-relative.dot(up)*scale;
    if (!Math.isFinite(x) || !Math.isFinite(y)) return null;
    return new GizmoPoint(x, y, d);
  }
  public function contains(x:Float, y:Float):Bool return Math.isFinite(x) && Math.isFinite(y) && x >= 0 && y >= 0 && x <= width && y <= height;
  public function rayDirection(x:Float, y:Float):GizmoVec3 {
    if (!Math.isFinite(x) || !Math.isFinite(y)) throw "Invalid gizmo pointer";
    var dx = (2*x/width-1)*tangent*width/height, dy = (1-2*y/height)*tangent;
    return forward.add(right.scaled(dx)).add(up.scaled(dy)).normalized();
  }
}
