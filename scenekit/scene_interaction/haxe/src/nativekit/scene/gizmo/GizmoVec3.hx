package nativekit.scene.gizmo;

/** Immutable Float64 geometry, independent of scene entities and rendering. */
class GizmoVec3 {
  public final x:Float; public final y:Float; public final z:Float;
  public function new(x:Float, y:Float, z:Float) {
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z)) throw "Gizmo vector must be finite";
    this.x = x; this.y = y; this.z = z;
  }
  public function add(b:GizmoVec3):GizmoVec3 return new GizmoVec3(x+b.x, y+b.y, z+b.z);
  public function subtract(b:GizmoVec3):GizmoVec3 return new GizmoVec3(x-b.x, y-b.y, z-b.z);
  public function scaled(s:Float):GizmoVec3 return new GizmoVec3(x*s, y*s, z*s);
  public function dot(b:GizmoVec3):Float return x*b.x+y*b.y+z*b.z;
  public function cross(b:GizmoVec3):GizmoVec3 return new GizmoVec3(y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x);
  public function normalized():GizmoVec3 {
    var maximum = Math.max(Math.abs(x), Math.max(Math.abs(y), Math.abs(z)));
    if (maximum == 0) throw "Degenerate gizmo direction";
    var a = x/maximum, b = y/maximum, c = z/maximum, length = Math.sqrt(a*a+b*b+c*c);
    return new GizmoVec3(a/length, b/length, c/length);
  }
}
