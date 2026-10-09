package collisionkit;

/**
 * A shape of a `CollisionDescription` on a body (-1: the world), at an
 * offset in the body's frame, with its inflation and what it approximates
 * (`source`, e.g. "primitive", "hull", "enclosing hull", "surface mesh",
 * "decomposed piece"), for the report's assumptions (CL-D8).
 */
class DescribedObject {
  public final name:String;
  public final body:Int;
  public final offset:CollisionPose;
  public final geometry:CollisionGeometry;
  public final inflation:Float;
  public final source:String;

  public function new(name:String, body:Int, offset:CollisionPose, geometry:CollisionGeometry, inflation:Float,
      source:String) {
    this.name = name;
    this.body = body;
    this.offset = offset;
    this.geometry = geometry;
    this.inflation = inflation;
    this.source = source;
  }
}
