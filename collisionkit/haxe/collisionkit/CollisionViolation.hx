package collisionkit;

/**
 * A checked pair closer than its query required (or, from `closest`, the
 * closest pair): the objects, their bodies (-1: the world), the signed
 * distance, and the clearance required (the group margin plus both bodies'
 * inflation). `set` is the index of the pose set it was found in, for
 * batched queries (0 otherwise).
 */
class CollisionViolation {
  public final a:Int;
  public final b:Int;
  public final bodyA:Int;
  public final bodyB:Int;
  public final distance:Float;
  public final required:Float;
  public final set:Int;

  public function new(a:Int, b:Int, bodyA:Int, bodyB:Int, distance:Float, required:Float, set:Int) {
    this.a = a;
    this.b = b;
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.distance = distance;
    this.required = required;
    this.set = set;
  }
}
