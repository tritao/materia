package nativekit.scene.gizmo;

/** Two points draw an axis; four draw a plane pad. Picking uses these same primitives. */
class TranslationShape {
  public final handle:TranslationHandle;
  public final label:String;
  final vertices:Array<GizmoPoint>;
  public function new(handle:TranslationHandle, label:String, vertices:Array<GizmoPoint>) {
    if (vertices == null || (vertices.length != 2 && vertices.length != 4)) throw "Invalid translation primitive";
    for (point in vertices) if (point == null) throw "Invalid translation primitive point";
    this.handle = handle; this.label = label; this.vertices = vertices.copy();
  }
  public function points():Array<GizmoPoint> return vertices.copy();
  public function depth():Float { var value = 0.0; for (point in vertices) value += point.depth/vertices.length; return value; }
  public function distance(x:Float, y:Float):Null<Float> return GizmoShapeHit.distance(vertices,x,y);
}
