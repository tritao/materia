package robotkit.runtime;

import haxe.Int64;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.world.RobotSnapshot;

/**
 * Ideal rolling-kinematics plant for a differential-drive robot in SimKit.
 *
 * Couples the robot's wheel joints to its kinematic base through the native
 * simulation: every tick, after the robot applies its commands and before
 * physics advances, the base rolls by the wheel velocity targets the robot
 * actually applied for that tick. Those are the runtime's rate-clamped targets
 * whether they came from MobileBase or were submitted straight to the robot,
 * and they are zero after a normal or emergency stop or a safety reset, so the
 * chassis follows every stop on the tick it takes effect with no added
 * latency. Articulated joints and sensors still advance through the shared
 * simulation, and the IMU measures the chassis motion.
 */
class DifferentialDrivePlant {
  public final simulation:Simulation;
  public final robotIndex:Int;
  public final base:MobileBase;
  /**
   * Planar base pose after the latest step or teleport: position on the floor
   * and heading (unwrapped). The base keeps its authored roll and pitch.
   */
  public var pose(get, never):Pose2;
  /** Base height, preserved while the plant drives the planar pose. */
  public var baseHeight(get, never):Float;

  public function new(simulation:Simulation, robotIndex:Int, base:MobileBase,
      ?initialPose:Pose2) {
    if (simulation == null || robotIndex < 0 || base == null)
      throw "Differential-drive plant requires a simulation, robot index, and mobile base";
    var odometry = base.driveModel.createOdometry();
    if (odometry == null)
      throw "Differential-drive plant requires a differential drive model";
    this.simulation = simulation;
    this.robotIndex = robotIndex;
    this.base = base;
    simulation.setDifferentialDrive(robotIndex, odometry.leftWheelJoint,
      odometry.rightWheelJoint, odometry.wheelRadius, odometry.trackWidth);
    if (initialPose != null) teleport(initialPose);
  }

  /**
   * Jumps the chassis for the next tick while keeping wheel targets, sensor
   * history, and the chassis velocity; the jump itself does not read as motion.
   * The chassis keeps its height and its roll and pitch relative to its
   * heading, as the native plant does while it drives.
   */
  public function teleport(pose:Pose2):Void {
    if (pose == null) throw "Differential-drive plant pose cannot be null";
    var q = simulation.robotPose(robotIndex).rotation;
    // Tilt = Rz(-heading) * rotation: the current rotation with its heading
    // (the body x axis projected on the floor) removed.
    var heading = Math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
    var tilt = multiply(yawRotation(-heading), q);
    simulation.placeRobotBase(robotIndex, [pose.x, pose.y, baseHeight],
      multiply(yawRotation(pose.yaw), tilt));
  }

  static function yawRotation(yaw:Float):Array<Float> {
    var half = yaw * 0.5;
    return [0.0, 0.0, Math.sin(half), Math.cos(half)];
  }

  // Hamilton product of xyzw quaternions, normalized.
  static function multiply(a:Array<Float>, b:Array<Float>):Array<Float> {
    var x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    var y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    var z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    var w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    var norm = Math.sqrt(x * x + y * y + z * z + w * w);
    return [x / norm, y / norm, z / norm, w / norm];
  }

  /** Advances one fixed simulation step and returns the robot's snapshot. */
  public function step(timestampNs:Int64):RobotSnapshot {
    simulation.step(timestampNs);
    return base.robot.snapshot();
  }

  /** Wheel rates, in rad/s, the robot applied during the latest step. */
  public function appliedWheelRates():{left:Float, right:Float} {
    var state = simulation.differentialDriveState(robotIndex);
    return {left: state.leftWheelRate, right: state.rightWheelRate};
  }

  function get_pose():Pose2 {
    var state = simulation.differentialDriveState(robotIndex);
    return new Pose2(state.x, state.y, state.yaw);
  }

  function get_baseHeight():Float
    return simulation.differentialDriveState(robotIndex).height;
}
