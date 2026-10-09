package collisionkit;

/** Two collision objects, the lower id first, with the bodies they are on (-1: the world). */
class CollisionPair {
  public final a:Int;
  public final b:Int;
  public final bodyA:Int;
  public final bodyB:Int;

  public function new(a:Int, b:Int, bodyA:Int, bodyB:Int) {
    this.a = a;
    this.b = b;
    this.bodyA = bodyA;
    this.bodyB = bodyB;
  }
}
