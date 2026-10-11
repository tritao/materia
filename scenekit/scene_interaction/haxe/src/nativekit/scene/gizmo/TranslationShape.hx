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
  public function distance(x:Float, y:Float):Null<Float> {
    if (vertices.length == 2) {
      var a = vertices[0], b = vertices[1], dx = b.x-a.x, dy = b.y-a.y;
      var squared = dx*dx+dy*dy;
      if (squared <= 0) return null;
      var t = Math.max(0, Math.min(1, ((x-a.x)*dx+(y-a.y)*dy)/squared));
      dx = x-a.x-t*dx; dy = y-a.y-t*dy; return Math.sqrt(dx*dx+dy*dy);
    }
    var sign = 0;
    for (index in 0...4) {
      var a = vertices[index], b = vertices[(index+1)%4], cross = (b.x-a.x)*(y-a.y)-(b.y-a.y)*(x-a.x);
      if (Math.abs(cross) < 0.000001) continue;
      var next = cross < 0 ? -1 : 1; if (sign != 0 && sign != next) return null; sign = next;
    }
    return 0.0;
  }
}
