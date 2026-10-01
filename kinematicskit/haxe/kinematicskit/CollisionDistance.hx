package kinematicskit;

/**
 * A pair's closest approach in world coordinates: the signed distance
 * (negative when penetrating), the closest point on each object, and the
 * unit normal from `a` toward `b`.
 */
class CollisionDistance {
  public final a:Int;
  public final b:Int;
  public final distance:Float;
  public final pointA:Vector3;
  public final pointB:Vector3;
  public final normal:Vector3;

  public function new(a:Int, b:Int, distance:Float, pointA:Vector3, pointB:Vector3, normal:Vector3) {
    this.a = a;
    this.b = b;
    this.distance = distance;
    this.pointA = pointA;
    this.pointB = pointB;
    this.normal = normal;
  }
}
