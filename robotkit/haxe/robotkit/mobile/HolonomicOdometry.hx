package robotkit.mobile;

import haxe.Int64;
import robotkit.world.RobotSnapshot;

/**
 * Wheel-position odometry for a three-wheel omni/kiwi base (see
 * `HolonomicDrive`). The three wheel angles are 120 degrees apart, so the
 * inverse of the same tangential speed relationship used by
 * `HolonomicDrive.targets()` decouples exactly:
 * `s_i = -sin(theta_i) * distance + cos(theta_i) * lateralDistance +
 * baseRadius * headingChange`. Forward and lateral displacement are weighted
 * sums of the wheel distances, while heading change is their average divided
 * by the base radius.
 */
class HolonomicOdometry {
  public final wheelJoints:Array<Int>;
  public final wheelRadius:Float;
  public final baseRadius:Float;
  public var lastDistance(default, null):Float = 0.0;
  public var lastLateralDistance(default, null):Float = 0.0;
  public var lastHeadingChange(default, null):Float = 0.0;

  final wheelAngles:Array<Float>;
  var pose:Pose2;
  var previousPositions:Null<Array<Float>> = null;
  var previousTimestamp:Null<Int64> = null;
  var previousClock:Null<String> = null;

  public function new(wheelJoints:Array<Int>, wheelRadius:Float, baseRadius:Float, ?initialPose:Pose2) {
    if (wheelJoints == null || wheelJoints.length != 3)
      throw "Holonomic odometry requires exactly three wheel joint indices";
    var seen = new Map<Int, Bool>();
    for (joint in wheelJoints) {
      if (joint < 0) throw "Holonomic odometry wheel joint indices must be non-negative";
      if (seen.exists(joint)) throw "Holonomic odometry requires three distinct wheel joint indices";
      seen.set(joint, true);
    }
    if (!Math.isFinite(wheelRadius) || wheelRadius <= 0.0 ||
        !Math.isFinite(baseRadius) || baseRadius <= 0.0)
      throw "Holonomic odometry dimensions must be finite and positive";
    this.wheelJoints = wheelJoints.copy();
    this.wheelRadius = wheelRadius;
    this.baseRadius = baseRadius;
    wheelAngles = [for (i in 0...3) Math.PI * 0.5 + i * (Math.PI * 2.0 / 3.0)];
    pose = initialPose == null ? new Pose2() : initialPose;
  }

  /** Updates and returns odometry from an immutable RobotSnapshot. */
  public function update(snapshot:RobotSnapshot):Pose2 {
    if (snapshot == null) throw "Holonomic odometry requires a robot snapshot";
    var positions = snapshot.positions;
    var current:Array<Float> = [];
    for (joint in wheelJoints) {
      if (positions.length <= joint) throw "Holonomic odometry joint index is outside the robot snapshot";
      current.push(positions.get(joint));
    }
    lastDistance = 0.0;
    lastLateralDistance = 0.0;
    lastHeadingChange = 0.0;
    var clockChanged = previousClock != null && previousClock != snapshot.sourceClockId;
    var timestampRegressed = previousTimestamp != null &&
      Int64.compare(snapshot.sourceTimestampNs, previousTimestamp) <= 0;
    if (timestampRegressed && !clockChanged) return pose;
    if (previousPositions == null || clockChanged) {
      setBaseline(snapshot, current);
      return pose;
    }
    var forwardSum = 0.0, lateralSum = 0.0, yawSum = 0.0;
    for (i in 0...3) {
      var wheelDistance = (current[i] - previousPositions[i]) * wheelRadius;
      forwardSum += -Math.sin(wheelAngles[i]) * wheelDistance;
      lateralSum += Math.cos(wheelAngles[i]) * wheelDistance;
      yawSum += wheelDistance;
    }
    var distance = forwardSum / 1.5;
    var lateralDistance = lateralSum / 1.5;
    var headingChange = yawSum / (3.0 * baseRadius);
    lastDistance = distance;
    lastLateralDistance = lateralDistance;
    lastHeadingChange = headingChange;
    pose = pose.integrateDisplacement(distance, headingChange, lateralDistance);
    setBaseline(snapshot, current);
    return pose;
  }

  public function current():Pose2 return pose;

  public function reset(?newPose:Pose2):Void {
    pose = newPose == null ? new Pose2() : newPose;
    previousPositions = null;
    previousTimestamp = null;
    previousClock = null;
    lastDistance = 0.0;
    lastLateralDistance = 0.0;
    lastHeadingChange = 0.0;
  }

  function setBaseline(snapshot:RobotSnapshot, values:Array<Float>):Void {
    previousPositions = values.copy();
    previousTimestamp = snapshot.sourceTimestampNs;
    previousClock = snapshot.sourceClockId;
  }
}
