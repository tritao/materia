package collisionkit;

/**
 * A rigid transform: a translation and a unit quaternion (x, y, z, w), in
 * the world's frame for a body, in its body's frame for an object's offset.
 * Callers convert their own transforms (a kit snapshot's body poses,
 * RobotKit's `Transform3`) to it.
 */
class CollisionPose {
  public final x:Float;
  public final y:Float;
  public final z:Float;
  public final qx:Float;
  public final qy:Float;
  public final qz:Float;
  public final qw:Float;

  public function new(x:Float, y:Float, z:Float, qx:Float, qy:Float, qz:Float, qw:Float) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.qx = qx;
    this.qy = qy;
    this.qz = qz;
    this.qw = qw;
  }

  public static function identity():CollisionPose return new CollisionPose(0, 0, 0, 0, 0, 0, 1);

  public static function translation(x:Float, y:Float, z:Float):CollisionPose return new CollisionPose(x, y, z, 0, 0, 0, 1);

  /** Appends the seven numbers to `out`. */
  public function writeTo(out:Array<Float>):Void {
    out.push(x);
    out.push(y);
    out.push(z);
    out.push(qx);
    out.push(qy);
    out.push(qz);
    out.push(qw);
  }
}
