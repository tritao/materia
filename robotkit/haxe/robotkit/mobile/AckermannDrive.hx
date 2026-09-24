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

  public function targets(twist:Twist2):Array<JointTarget> {
    if (twist == null) throw "Ackermann drive requires a twist";
    var limited = constrain(twist);
    if (Math.abs(limited.linear) < 1e-9 && Math.abs(limited.angular) < 1e-9)
      return [JointTarget.position(steeringJoint, 0.0),
        JointTarget.velocity(driveWheelJoint, 0.0)];
    var curvatureRatio = wheelBase * limited.angular / limited.linear;
    var steering = inverseTangent(curvatureRatio);
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

  function inverseTangent(value:Float):Float {
    var sign = value < 0.0 ? -1.0 : 1.0;
    var x = Math.abs(value);
    var outerOffset = 0.0;
    var outerSign = 1.0;
    var localOffset = 0.0;
    if (x > 1.0) {
      x = 1.0 / x;
      outerOffset = Math.PI * 0.5;
      outerSign = -1.0;
    }
    if (x > 0.5) {
      x = (x - 1.0) / (x + 1.0);
      localOffset = Math.PI * 0.25;
    }
    // Range reduction keeps the alternating series within |x| <= 0.5.
    var squared = x * x;
    var power = x;
    var result = x;
    for (index in 1...13) {
      power *= -squared;
      result += power / (index * 2.0 + 1.0);
    }
    return sign * (outerOffset + outerSign * (localOffset + result));
  }
}
