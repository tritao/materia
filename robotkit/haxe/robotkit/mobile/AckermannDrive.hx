package robotkit.mobile;

import robotkit.world.JointTarget;

/** Bicycle-model Ackermann steering plus one driven wheel joint. */
class AckermannDrive implements DriveModel {
  public final steeringJoint:Int;
  public final driveWheelJoint:Int;
  public final wheelBase:Float;
  public final wheelRadius:Float;
  public final maxSteeringAngle:Float;

  public function new(steeringJoint:Int, driveWheelJoint:Int, wheelBase:Float,
      wheelRadius:Float, maxSteeringAngle:Float) {
    if (steeringJoint < 0 || driveWheelJoint < 0 || steeringJoint == driveWheelJoint)
      throw "Ackermann drive requires distinct non-negative steering and drive joints";
    if (!Math.isFinite(wheelBase) || wheelBase <= 0.0 ||
        !Math.isFinite(wheelRadius) || wheelRadius <= 0.0 ||
        !Math.isFinite(maxSteeringAngle) || maxSteeringAngle <= 0.0 ||
        maxSteeringAngle >= Math.PI * 0.5)
      throw "Ackermann dimensions and steering angle must be finite and positive";
    this.steeringJoint = steeringJoint;
    this.driveWheelJoint = driveWheelJoint;
    this.wheelBase = wheelBase;
    this.wheelRadius = wheelRadius;
    this.maxSteeringAngle = maxSteeringAngle;
  }

  public function constrain(twist:Twist2):Twist2 {
    if (twist == null) throw "Ackermann drive requires a twist";
    if (Math.abs(twist.linear) < 1e-9) {
      if (Math.abs(twist.angular) > 1e-9)
        throw "Ackermann drive cannot turn in place";
      return new Twist2();
    }
    var maxYawRate = Math.abs(twist.linear) * Math.tan(maxSteeringAngle) / wheelBase;
    return new Twist2(twist.linear,
      twist.angular > maxYawRate ? maxYawRate :
      (twist.angular < -maxYawRate ? -maxYawRate : twist.angular));
  }

  public function maxCurvature():Float return Math.tan(maxSteeringAngle) / wheelBase;

  public function createOdometry():Null<DifferentialOdometry> return null;

  public function supportsInPlaceRotation():Bool return false;

  public function targets(twist:Twist2):Array<JointTarget> {
    if (twist == null) throw "Ackermann drive requires a twist";
    var limited = constrain(twist);
    if (Math.abs(limited.linear) < 1e-9 && Math.abs(limited.angular) < 1e-9)
      return [JointTarget.position(steeringJoint, 0.0),
        JointTarget.velocity(driveWheelJoint, 0.0)];
    var curvatureRatio = wheelBase * limited.angular / limited.linear;
    var steering = PlanarMath.atan(curvatureRatio);
    if (steering > maxSteeringAngle) steering = maxSteeringAngle;
    if (steering < -maxSteeringAngle) steering = -maxSteeringAngle;
    var wheelLinearSpeed = Math.pow(limited.linear * limited.linear +
      (limited.angular * wheelBase) * (limited.angular * wheelBase), 0.5);
    var direction = limited.linear < 0.0 ? -1.0 : 1.0;
    return [
      JointTarget.position(steeringJoint, steering),
      JointTarget.velocity(driveWheelJoint, direction * wheelLinearSpeed / wheelRadius)
    ];
  }

}
