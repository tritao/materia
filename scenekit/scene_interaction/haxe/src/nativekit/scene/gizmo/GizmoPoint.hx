package nativekit.scene.gizmo;

/** Logical viewport pixels and positive eye-space depth. */
class GizmoPoint {
  public final x:Float; public final y:Float; public final depth:Float;
  public function new(x:Float, y:Float, depth:Float) {
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(depth) || depth <= 0) throw "Invalid gizmo screen point";
    this.x = x; this.y = y; this.depth = depth;
  }
}
