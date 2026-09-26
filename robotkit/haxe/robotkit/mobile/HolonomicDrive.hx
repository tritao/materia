package robotkit.mobile;

import robotkit.world.JointTarget;

/**
 * Omnidirectional ("kiwi") drive: three wheels mounted at 120-degree
 * intervals around the base center, each rolling tangentially. `targets`
 * accepts the full `Twist2` body velocity. Navigator/GoTo continue to produce
 * lateral = 0, while direct callers and work planners can command a strafe.
 */
class HolonomicDrive implements DriveModel {
  public final wheelJoints:Array<Int>;
  public final wheelRadius:Float;
  public final baseRadius:Float;

  /** Mount angle of each wheel about +Z from the base's x axis, in radians. */
  public final wheelAngles:Array<Float>;

  public function new(wheelJoints:Array<Int>, wheelRadius:Float, baseRadius:Float) {
    if (wheelJoints == null || wheelJoints.length != 3)
      throw "Holonomic drive requires exactly three wheel joint indices";
    var seen = new Map<Int, Bool>();
    for (joint in wheelJoints) {
      if (joint < 0) throw "Holonomic drive wheel joint indices must be non-negative";
      if (seen.exists(joint)) throw "Holonomic drive requires three distinct wheel joint indices";
      seen.set(joint, true);
    }
    if (!Math.isFinite(wheelRadius) || wheelRadius <= 0.0 ||
        !Math.isFinite(baseRadius) || baseRadius <= 0.0)
      throw "Holonomic drive dimensions must be finite and positive";
    this.wheelJoints = wheelJoints.copy();
    this.wheelRadius = wheelRadius;
    this.baseRadius = baseRadius;
    wheelAngles = [for (i in 0...3) Math.PI * 0.5 + i * (Math.PI * 2.0 / 3.0)];
  }

  public function constrain(twist:Twist2):Twist2 return twist;

  /** Every heading is reachable while driving straight, like in-place rotation. */
  public function maxCurvature():Float return 1.0e300;

  /** No two-wheel differential relationship to decode; HolonomicDrivePlant uses wheelAngles. */
  public function createOdometry():Null<DifferentialOdometry> return null;

  public function supportsInPlaceRotation():Bool return true;

  public function targets(twist:Twist2):Array<JointTarget> {
    if (twist == null) throw "Holonomic drive requires a twist";
    var result:Array<JointTarget> = [];
    for (i in 0...3) {
      // Tangential wheel speed for a body moving at (vx, vy) and yawing at
      // omega: s_i = -sin(theta_i)*vx + cos(theta_i)*vy + baseRadius*omega.
      var wheelSpeed = -Math.sin(wheelAngles[i]) * twist.linear +
        Math.cos(wheelAngles[i]) * twist.lateral + baseRadius * twist.angular;
      result.push(JointTarget.velocity(wheelJoints[i], wheelSpeed / wheelRadius));
    }
    return result;
  }
}
