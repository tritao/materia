package robotkit.tool;

import robotkit.model.CollisionShape;
import robotkit.spatial.Vec3;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionBounds;
import collisionkit.CollisionPose;

/** Construction and union bounds for flange-posed tool shapes. */
class ToolGeometry {
  public static function box(half:Vec3, ?centre:Vec3):Array<CollisionShape> {
    return [new CollisionShape(Box(half.x,half.y,half.z),centre==null?null:[centre.x,centre.y,centre.z])];
  }
  public static function convex(pieces:Array<Array<Float>>, inflation:Float):Array<CollisionShape> {
    return [for (points in pieces) {var shape=new CollisionShape(Convex(points));shape.inflation=inflation;shape;}];
  }
  public static function bounds(shapes:Array<CollisionShape>):{centre:Vec3, halfExtents:Vec3} {
    if (shapes==null||shapes.length==0) throw "Tool has no collision geometry";
    var lo=[Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY];
    var hi=[Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY];
    for (shape in shapes) {
      var invalid=shape.validate(); if (invalid!=null) throw invalid;
      var p=shape.position,q=shape.rotation;
      var b=CollisionBounds.of(shape.geometry,new CollisionPose(p[0],p[1],p[2],q[0],q[1],q[2],q[3]),shape.inflation);
      var c=[b.centre.x,b.centre.y,b.centre.z],h=[b.halfExtents.x,b.halfExtents.y,b.halfExtents.z];
      for (a in 0...3) {lo[a]=Math.min(lo[a],c[a]-h[a]);hi[a]=Math.max(hi[a],c[a]+h[a]);}
    }
    return {centre:new Vec3((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2),
      halfExtents:new Vec3((hi[0]-lo[0])/2,(hi[1]-lo[1])/2,(hi[2]-lo[2])/2)};
  }
}
