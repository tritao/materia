package robotkit.runtime;

import haxe.Int64;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.core.RobotSnapshot;

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
  final harness:SimulationHarness;
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

  public function new(harness:SimulationHarness, robotIndex:Int, base:MobileBase,
      ?initialPose:Pose2) {
    if (harness == null || robotIndex < 0 || base == null)
      throw "Differential-drive plant requires a simulation, robot index, and mobile base";
    var odometry = base.driveModel.createOdometry();
    if (odometry == null)
      throw "Differential-drive plant requires a differential drive model";
    this.harness = harness;
    this.simulation = harness.simulation;
    this.robotIndex = robotIndex;
    this.base = base;
    simulation.setDifferentialDrive(robotIndex, odometry.leftWheelJoint,
      odometry.rightWheelJoint, odometry.wheelRadius, odometry.trackWidth,
      odometry.leftDirection, odometry.rightDirection);
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
    BasePlacement.place(simulation, robotIndex, pose, baseHeight);
  }

  /** Advances one fixed simulation step and returns the robot's snapshot. */
  public function step(timestampNs:Int64):RobotSnapshot {
    harness.step(timestampNs);
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
