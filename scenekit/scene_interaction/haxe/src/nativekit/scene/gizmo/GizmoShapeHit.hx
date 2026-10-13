package nativekit.scene.gizmo;

/** Shared capsule/convex-pad hit geometry in logical pixels. */
class GizmoShapeHit {
  public static function distance(vertices:Array<GizmoPoint>, x:Float, y:Float):Null<Float> {
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
