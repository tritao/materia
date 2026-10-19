package app;

import nativekit.scene.GeometryData;
import materia.project.ThreadRenderData;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

/** Local display interval; nominal thread dimensions stay in the model's millimetres. */
class CadThreadGeometry {
  public final source:ThreadRenderData;
  public final frame:Plane;
  public final length:Float;
  public final pitch:Float;
  public final diameter:Float;
  public function new(source:ThreadRenderData, center:Array<Float>, scale:Float) {
    this.source = source;
    frame = new Plane(new Vector((source.origin[0] - center[0]) * scale,
      (source.origin[1] - center[1]) * scale, (source.origin[2] - center[2]) * scale),
      new Vector(source.xDirection[0], source.xDirection[1], source.xDirection[2]),
      new Vector(source.direction[0], source.direction[1], source.direction[2]));
    length = source.length * scale;
    pitch = source.spec.pitch * CadSceneGeometry.METRES_PER_MILLIMETRE;
    diameter = source.spec.majorDiameter * CadSceneGeometry.METRES_PER_MILLIMETRE;
  }
}

/** Base triangles and their derived semantic display data share one geometry lifetime. */
class CadGeometryData extends GeometryData {
  public final threads:Array<CadThreadGeometry>;
  final cadStrokes:Array<{points:Array<Float>,id:Int,feature:Bool}> = [];
  var edgeMode:CadEdgeMode=Feature;
  public function addCadStroke(points:Array<Float>,id:Int,feature:Bool):Void {
    cadStrokes.push({points:points,id:id,feature:feature});
    if(feature)super.addStrokeSegment(points[0],points[1],points[2],points[3],points[4],points[5],id);
  }
  public function setEdgeMode(mode:CadEdgeMode):Void {
    if(edgeMode==mode)return;
    edgeMode=mode;clearStrokeSegments();
    for(stroke in cadStrokes)if(mode==All || mode==Feature && stroke.feature) {
      var p=stroke.points;super.addStrokeSegment(p[0],p[1],p[2],p[3],p[4],p[5],stroke.id);
    }
  }

  public function new(threads:Array<ThreadRenderData>, center:Array<Float>, scale:Float) {
    super();
    this.threads = threads == null ? [] : [for (thread in threads) new CadThreadGeometry(thread, center, scale)];
  }
}
