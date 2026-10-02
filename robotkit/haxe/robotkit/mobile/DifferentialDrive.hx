package robotkit.mobile;

import robotkit.world.JointTarget;

/**
 * Differential-drive wheel velocity mapping. A wheel's direction is +1 when a
 * positive joint rate rolls it forward and -1 when its joint turns the other
 * way, such as a right wheel whose axis follows its outward-facing motor shaft;
 * the compiler derives both from the wheel joints' axes.
 */
class DifferentialDrive implements DriveModel {
  public final leftWheelJoint:Int;
  public final rightWheelJoint:Int;
  public final wheelRadius:Float;
  public final trackWidth:Float;
  public final leftDirection:Int;
  public final rightDirection:Int;

  public function new(leftWheelJoint:Int, rightWheelJoint:Int,
      wheelRadius:Float, trackWidth:Float, leftDirection:Int = 1, rightDirection:Int = 1) {
    if (leftWheelJoint < 0 || rightWheelJoint < 0 || leftWheelJoint == rightWheelJoint)
      throw "Differential drive requires two distinct non-negative wheel joint indices";
    if (!Math.isFinite(wheelRadius) || wheelRadius <= 0.0 ||
        !Math.isFinite(trackWidth) || trackWidth <= 0.0)
      throw "Differential drive dimensions must be finite and positive";
    if (Math.abs(leftDirection) != 1 || Math.abs(rightDirection) != 1)
      throw "Differential drive wheel directions must be 1 or -1";
    this.leftDirection = leftDirection;
    this.rightDirection = rightDirection;
    this.leftWheelJoint = leftWheelJoint;
    this.rightWheelJoint = rightWheelJoint;
    this.wheelRadius = wheelRadius;
    this.trackWidth = trackWidth;
  }

  public function constrain(twist:Twist2):Twist2 {
    rejectLateral(twist);
    return twist;
  }

  public function maxCurvature():Float return 1.0e300;

  public function createOdometry():Null<DifferentialOdometry>
    return new DifferentialOdometry(leftWheelJoint, rightWheelJoint, wheelRadius, trackWidth, null,
      leftDirection, rightDirection);

  public function supportsInPlaceRotation():Bool return true;

  public function targets(twist:Twist2):Array<JointTarget> {
    if (twist == null) throw "Differential drive requires a twist";
    rejectLateral(twist);
    var halfTurnSpeed = twist.angular * trackWidth * 0.5;
    return [
      JointTarget.velocity(leftWheelJoint, leftDirection * (twist.linear - halfTurnSpeed) / wheelRadius),
      JointTarget.velocity(rightWheelJoint, rightDirection * (twist.linear + halfTurnSpeed) / wheelRadius)
    ];
  }

  static function rejectLateral(twist:Twist2):Void {
    if (twist == null) throw "Differential drive requires a twist";
    if (Math.abs(twist.lateral) > 1e-9)
      throw "Differential drive cannot execute a lateral command";
  }
}
