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
  public function new(threads:Array<ThreadRenderData>, center:Array<Float>, scale:Float) {
    super();
    this.threads = threads == null ? [] : [for (thread in threads) new CadThreadGeometry(thread, center, scale)];
  }
}
