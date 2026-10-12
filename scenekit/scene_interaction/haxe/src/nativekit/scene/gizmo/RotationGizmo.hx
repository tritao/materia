package nativekit.scene.gizmo;
import nativekit.scene.gizmo.TranslationHandle;

/** Frozen axis rings. Perspective logical pixels; no scene, source or UI ownership. */
class RotationGizmo {
  public final camera:GizmoCamera;
  public final pivot:GizmoVec3;
  public final basis:GizmoBasis;
  final geometry:Array<RotationShape> = [];
  public function new(camera:GizmoCamera,pivot:GizmoVec3,pixels:Float=75,basis:Null<GizmoBasis> = null) {
    if (camera == null || pivot == null || !Math.isFinite(pixels) || pixels <= 0 || pixels > 512) throw "Invalid rotation layout";
    this.camera = camera; this.pivot = pivot; this.basis = basis == null ? GizmoBasis.world() : basis;
    var center = camera.project(pivot); if (center == null || !camera.contains(center.x,center.y)) return;
    var radius = camera.worldLength(pivot,pixels), eye = camera.eye.subtract(pivot).normalized();
    for (handle in [AxisX,AxisY,AxisZ]) {
      var normal = this.basis.axis(handle); if (Math.abs(normal.dot(eye)) < 0.15) continue;
      var u = handle == AxisX ? this.basis.y : this.basis.x, v = normal.cross(u);
      var points:Array<GizmoPoint> = [], valid = true;
      for (i in 0...65) {
        var angle = i*Math.PI/32;
        var point = camera.project(pivot.add(u.scaled(radius*Math.cos(angle))).add(v.scaled(radius*Math.sin(angle))));
        if (point == null) { valid = false; break; } points.push(point);
      }
      if (valid) geometry.push(new RotationShape(handle,points));
    }
  }
  public function shapes():Array<RotationShape> return geometry.copy();
  public function pick(x:Float,y:Float):Null<TranslationHandle> {
    if (!camera.contains(x,y)) return null;
    var best:Null<TranslationHandle> = null, distance = 8.0;
    for (shape in geometry) { var d = shape.distance(x,y); if (d < distance) { distance = d; best = shape.handle; } }
    return best;
  }
  public function begin(handle:TranslationHandle,x:Float,y:Float,snapDegrees:Float=0):RotationDrag {
    if (handle == null || pick(x,y) != handle) throw "Rotation must start on a visible picked ring";
    return new RotationDrag(camera,pivot,basis.axis(handle),handle,x,y,snapDegrees);
  }
}
