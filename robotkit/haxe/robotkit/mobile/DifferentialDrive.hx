package robotkit.mobile;

import robotkit.world.JointTarget;

/** Differential-drive wheel velocity mapping. */
class DifferentialDrive implements DriveModel {
  public final leftWheelJoint:Int;
  public final rightWheelJoint:Int;
  public final wheelRadius:Float;
  public final trackWidth:Float;

  public function new(leftWheelJoint:Int, rightWheelJoint:Int,
      wheelRadius:Float, trackWidth:Float) {
    if (leftWheelJoint < 0 || rightWheelJoint < 0 || leftWheelJoint == rightWheelJoint)
      throw "Differential drive requires two distinct non-negative wheel joint indices";
    if (!Math.isFinite(wheelRadius) || wheelRadius <= 0.0 ||
        !Math.isFinite(trackWidth) || trackWidth <= 0.0)
      throw "Differential drive dimensions must be finite and positive";
    this.leftWheelJoint = leftWheelJoint;
    this.rightWheelJoint = rightWheelJoint;
    this.wheelRadius = wheelRadius;
    this.trackWidth = trackWidth;
  }

  public function constrain(twist:Twist2):Twist2 return twist;

  public function createOdometry():Null<DifferentialOdometry>
    return new DifferentialOdometry(leftWheelJoint, rightWheelJoint, wheelRadius, trackWidth);

  public function targets(twist:Twist2):Array<JointTarget> {
    if (twist == null) throw "Differential drive requires a twist";
    var halfTurnSpeed = twist.angular * trackWidth * 0.5;
    return [
      JointTarget.velocity(leftWheelJoint, (twist.linear - halfTurnSpeed) / wheelRadius),
      JointTarget.velocity(rightWheelJoint, (twist.linear + halfTurnSpeed) / wheelRadius)
    ];
  }
}
