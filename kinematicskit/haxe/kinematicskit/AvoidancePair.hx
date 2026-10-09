package kinematicskit;

/**
 * Two bodies within the influence distance of each other (COLLISION.md
 * CL-D4, CL5), as a collision world reports them: the closest points on each
 * (world frame), the unit normal from `a` toward `b`, the signed distance,
 * and the pair's safety margin `ds` and influence distance `di`. A side that
 * is not a body of the solved model (the environment, another robot) is -1:
 * it does not move with the solve.
 */
class AvoidancePair {
  public final bodyA:Int;
  public final bodyB:Int;
  public final pointA:Vector3;
  public final pointB:Vector3;
  public final normal:Vector3;
  public final distance:Float;
  public final safety:Float;
  public final influence:Float;

  public function new(bodyA:Int, bodyB:Int, pointA:Vector3, pointB:Vector3, normal:Vector3, distance:Float, safety:Float,
      influence:Float) {
    if (!(influence > safety) || safety < 0) throw "An avoidance pair needs 0 <= ds < di";
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.pointA = pointA;
    this.pointB = pointB;
    this.normal = normal;
    this.distance = distance;
    this.safety = safety;
    this.influence = influence;
  }
}
