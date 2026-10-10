package collisionkit;

/**
 * A pair's closest approach in world coordinates: the objects and their
 * bodies (-1: the world), the signed distance (negative when penetrating),
 * the closest point on each object (x, y, z), and the unit normal from `a`
 * toward `b`.
 */
class CollisionDistance {
  public final a:Int;
  public final b:Int;
  public final bodyA:Int;
  public final bodyB:Int;
  public final distance:Float;
  public final pointA:Array<Float>;
  public final pointB:Array<Float>;
  public final normal:Array<Float>;

  public function new(a:Int, b:Int, bodyA:Int, bodyB:Int, distance:Float, pointA:Array<Float>, pointB:Array<Float>,
      normal:Array<Float>) {
    this.a = a;
    this.b = b;
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.distance = distance;
    this.pointA = pointA;
    this.pointB = pointB;
    this.normal = normal;
  }
}
