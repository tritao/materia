package nativekit.scene.gizmo;

/** Frozen constraint producing absolute World displacement from pointer-down. */
class TranslationDrag {
  public final handle:TranslationHandle;
  public var lastDelta(default, null):GizmoVec3 = new GizmoVec3(0,0,0);
  public var closed(default, null):Bool = false;
  final camera:GizmoCamera;
  final pivot:GizmoVec3;
  final normal:GizmoVec3;
  final axes:Array<GizmoVec3>;
  final initial:GizmoVec3;
  final snap:Float;
  @:allow(nativekit.scene.gizmo.TranslationGizmo)
  private function new(camera:GizmoCamera, pivot:GizmoVec3, handle:TranslationHandle, x:Float, y:Float, snap:Float, basis:GizmoBasis) {
    if (!Math.isFinite(snap) || snap < 0) throw "Translation snap must be a finite nonnegative increment";
    this.camera = camera; this.pivot = pivot; this.handle = handle; this.snap = snap;
    switch handle {
      case AxisX, AxisY, AxisZ:
        axes = [basis.axis(handle)];
        var eye = camera.eye.subtract(pivot).normalized();
        normal = eye.subtract(axes[0].scaled(axes[0].dot(eye))).normalized();
      default:
        axes = basis.planeAxes(handle); normal = axes[0].cross(axes[1]).normalized();
    }
    var start = intersect(x, y); if (start == null) throw "Unstable translation drag start"; initial = start;
  }
  function intersect(x:Float, y:Float):Null<GizmoVec3> {
    if (!Math.isFinite(x) || !Math.isFinite(y)) return null;
    try {
      var ray = camera.rayDirection(x,y), denominator = normal.dot(ray);
      if (!Math.isFinite(denominator) || Math.abs(denominator) < 0.000001) return null;
      var t = normal.dot(pivot.subtract(camera.eye))/denominator;
      if (!Math.isFinite(t) || t <= 0) return null;
      return camera.eye.add(ray.scaled(t));
    } catch (_:Dynamic) return null;
  }
  public function update(x:Float, y:Float):Null<GizmoVec3> {
    if (closed) throw "Translation drag is closed";
    var hit = intersect(x,y); if (hit == null) return null;
    try {
      var difference = hit.subtract(initial), delta = new GizmoVec3(0,0,0);
      for (axis in axes) {
        var coefficient = difference.dot(axis);
        if (snap > 0) coefficient = Math.fround(coefficient/snap)*snap;
        delta = delta.add(axis.scaled(coefficient));
      }
      lastDelta = delta; return delta;
    } catch (_:Dynamic) return null;
  }
  public function finish():GizmoVec3 { if (closed) throw "Translation drag is closed"; closed = true; return lastDelta; }
  public function cancel():Void { if (closed) throw "Translation drag is closed"; closed = true; }
}
