package nativekit.scene.gizmo;

/** Signed angle about a frozen World axis, unwrapped across the atan2 boundary. */
class RotationDrag {
  public final handle:TranslationHandle;
  public final axis:GizmoVec3;
  public var lastDegrees(default,null):Float = 0;
  public var closed(default,null):Bool = false;
  final camera:GizmoCamera;
  final pivot:GizmoVec3;
  final initial:GizmoVec3;
  final snap:Float;
  var previous:Float = 0;
  var accumulated:Float = 0;
  @:allow(nativekit.scene.gizmo.RotationGizmo)
  private function new(camera:GizmoCamera,pivot:GizmoVec3,axis:GizmoVec3,handle:TranslationHandle,x:Float,y:Float,snap:Float) {
    if (!Math.isFinite(snap) || snap < 0 || snap > 360) throw "Invalid rotation snap";
    this.camera = camera; this.pivot = pivot; this.axis = axis; this.handle = handle; this.snap = snap;
    var start = radial(x,y); if (start == null) throw "Unstable rotation start"; initial = start;
  }
  function radial(x:Float,y:Float):Null<GizmoVec3> {
    if (!Math.isFinite(x) || !Math.isFinite(y)) return null;
    try {
      var ray = camera.rayDirection(x,y), denominator = axis.dot(ray);
      if (Math.abs(denominator)<0.000001) return null;
      var t = axis.dot(pivot.subtract(camera.eye))/denominator; if (!Math.isFinite(t) || t<=0) return null;
      var vector = camera.eye.add(ray.scaled(t)).subtract(pivot);
      if (vector.dot(vector)<0.000000000001) return null;
      return vector.normalized();
    } catch (_:Dynamic) return null;
  }
  public function update(x:Float,y:Float):Null<Float> {
    if (closed) throw "Rotation drag is closed";
    var current = radial(x,y); if (current == null) return null;
    var raw = Math.atan2(axis.dot(initial.cross(current)),initial.dot(current))*180/Math.PI;
    var step = raw-previous; if (step>180) step-=360; if (step< -180) step+=360;
    var next = accumulated+step; if (!Math.isFinite(next) || Math.abs(next)>1000000) return null;
    previous = raw; accumulated = next; lastDegrees = snap == 0 ? next : Math.fround(next/snap)*snap;
    return lastDegrees;
  }
  public function finish():Float { if (closed) throw "Rotation drag is closed"; closed = true; return lastDegrees; }
  public function cancel():Void { if (closed) throw "Rotation drag is closed"; closed = true; }
}
