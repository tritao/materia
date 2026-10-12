package nativekit.scene.gizmo;

/** Owned projected ring polyline used identically by drawing and picking. */
class RotationShape {
  public final handle:TranslationHandle;
  public final label:String;
  final vertices:Array<GizmoPoint>;
  public function new(handle:TranslationHandle, points:Array<GizmoPoint>) {
    this.handle = handle; label = TranslationGizmo.label(handle); vertices = points.copy();
  }
  public function points():Array<GizmoPoint> return vertices.copy();
  public function distance(x:Float,y:Float):Float {
    var best = 1e300;
    for (i in 1...vertices.length) {
      var a = vertices[i-1], b = vertices[i], dx = b.x-a.x, dy = b.y-a.y;
      var length = dx*dx+dy*dy;
      var t = length == 0 ? 0 : Math.max(0,Math.min(1,((x-a.x)*dx+(y-a.y)*dy)/length));
      var px = x-a.x-t*dx, py = y-a.y-t*dy; best = Math.min(best,Math.sqrt(px*px+py*py));
    }
    return best;
  }
}
