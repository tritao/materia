package collisionkit;

/**
 * One articulation of a `CollisionDescription` (a robot, a CAD mechanism):
 * its bodies, and the reference configuration they are posed at when the
 * description is built (CL-D3: overlap at reference applies among them).
 */
class DescribedArticulation {
  public final name:String;
  public final bodies:Array<Int>;
  /** What the reference configuration is ("home", "zero", or the caller's name). */
  public final reference:String;

  public function new(name:String, bodies:Array<Int>, reference:String) {
    this.name = name;
    this.bodies = bodies;
    this.reference = reference;
  }
}
