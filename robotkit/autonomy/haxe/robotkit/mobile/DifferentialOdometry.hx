package robotkit.mobile;

import haxe.Int64;
import robotkit.core.RobotSnapshot;

/** Wheel-position odometry that resets its integration baseline on source-clock changes. */
class DifferentialOdometry {
  public final leftWheelJoint:Int;
  public final rightWheelJoint:Int;
  public final wheelRadius:Float;
  public final trackWidth:Float;
  /** +1 when a positive joint rate rolls the wheel forward, -1 when it rolls it back. */
  public final leftDirection:Int;
  public final rightDirection:Int;
  public var lastDistance(default, null):Float = 0.0;
  public var lastHeadingChange(default, null):Float = 0.0;
  var pose:Pose2;
  var previousLeft:Null<Float> = null;
  var previousRight:Null<Float> = null;
  var previousTimestamp:Null<Int64> = null;
  var previousClock:Null<String> = null;

  public function new(leftWheelJoint:Int, rightWheelJoint:Int,
      wheelRadius:Float, trackWidth:Float, ?initialPose:Pose2, leftDirection:Int = 1, rightDirection:Int = 1) {
    if (leftWheelJoint < 0 || rightWheelJoint < 0 || leftWheelJoint == rightWheelJoint)
      throw "Odometry requires two distinct non-negative wheel joint indices";
    if (!Math.isFinite(wheelRadius) || wheelRadius <= 0.0 ||
        !Math.isFinite(trackWidth) || trackWidth <= 0.0)
      throw "Odometry dimensions must be finite and positive";
    if (Math.abs(leftDirection) != 1 || Math.abs(rightDirection) != 1)
      throw "Odometry wheel directions must be 1 or -1";
    this.leftDirection = leftDirection;
    this.rightDirection = rightDirection;
    this.leftWheelJoint = leftWheelJoint;
    this.rightWheelJoint = rightWheelJoint;
    this.wheelRadius = wheelRadius;
    this.trackWidth = trackWidth;
    pose = initialPose == null ? new Pose2() : initialPose;
  }

  /** Updates and returns odometry from an immutable RobotSnapshot. */
  public function update(snapshot:RobotSnapshot):Pose2 {
    if (snapshot == null) throw "Wheel odometry requires a robot snapshot";
    var positions = snapshot.positions;
    if (positions.length <= leftWheelJoint || positions.length <= rightWheelJoint)
      throw "Wheel odometry joint index is outside the robot snapshot";
    var left = positions.get(leftWheelJoint);
    var right = positions.get(rightWheelJoint);
    lastDistance = 0.0;
    lastHeadingChange = 0.0;
    var clockChanged = previousClock != null && previousClock != snapshot.sourceClockId;
    var timestampRegressed = previousTimestamp != null &&
      Int64.compare(snapshot.sourceTimestampNs, previousTimestamp) <= 0;
    if (timestampRegressed && !clockChanged)
      return pose;
    if (previousLeft == null || previousRight == null || clockChanged) {
      setBaseline(snapshot, left, right);
      return pose;
    }
    var leftDistance = leftDirection * (left - previousLeft) * wheelRadius;
    var rightDistance = rightDirection * (right - previousRight) * wheelRadius;
    var distance = (leftDistance + rightDistance) * 0.5;
    var headingChange = (rightDistance - leftDistance) / trackWidth;
    lastDistance = distance;
    lastHeadingChange = headingChange;
    pose = pose.integrateDisplacement(distance, headingChange);
    setBaseline(snapshot, left, right);
    return pose;
  }

  public function current():Pose2 return pose;

  public function reset(?newPose:Pose2):Void {
    pose = newPose == null ? new Pose2() : newPose;
    previousLeft = null;
    previousRight = null;
    previousTimestamp = null;
    previousClock = null;
    lastDistance = 0.0;
    lastHeadingChange = 0.0;
  }

  function setBaseline(snapshot:RobotSnapshot, left:Float, right:Float):Void {
    previousLeft = left;
    previousRight = right;
    previousTimestamp = snapshot.sourceTimestampNs;
    previousClock = snapshot.sourceClockId;
  }
}
