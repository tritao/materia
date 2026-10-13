package nativekit.scene.gizmo;

/** Owned axis shaft or center pad; drawing and picking use the same points. */
class ScaleShape {
  public final handle:ScaleHandle;
  public final label:String;
  final vertices:Array<GizmoPoint>;
  public function new(handle:ScaleHandle,points:Array<GizmoPoint>) {
    if(handle==null || points==null || (points.length!=2 && points.length!=4)) throw "Invalid scale shape";
    for(point in points) if(point==null) throw "Missing scale point";
    this.handle=handle; label=ScaleGizmo.label(handle); vertices=points.copy();
  }
  public function points():Array<GizmoPoint> return vertices.copy();
  public function distance(x:Float,y:Float):Null<Float> return GizmoShapeHit.distance(vertices,x,y);
}
