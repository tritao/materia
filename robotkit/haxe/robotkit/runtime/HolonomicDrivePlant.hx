package robotkit.runtime;

import haxe.Int64;
import robotkit.mobile.HolonomicDrive;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.world.RobotSnapshot;

/**
 * Ideal rolling-kinematics plant for an omnidirectional-base robot in
 * SimKit, the three-omni-wheel counterpart of `DifferentialDrivePlant`.
 *
 * Couples the robot's wheel joints to its kinematic base through the native
 * simulation: every tick, after the robot applies its commands and before
 * physics advances, the base moves by the full planar body twist (forward,
 * lateral, and yaw rate) decoded from the wheel velocity targets the robot
 * actually applied for that tick. Those are the runtime's rate-clamped
 * targets whether they came from MobileBase or were submitted straight to the
 * robot, and they are zero after a normal or emergency stop or a safety
 * reset, so the chassis follows every stop with no added latency, and wheels
 * driven directly can strafe through the full `Twist2` command.
 */
class HolonomicDrivePlant {
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
      throw "Holonomic-drive plant requires a simulation, robot index, and mobile base";
    var drive:HolonomicDrive = cast(base.driveModel, HolonomicDrive);
    if (drive == null)
      throw "Holonomic-drive plant requires a holonomic drive model";
    this.simulation = simulation;
    this.robotIndex = robotIndex;
    this.base = base;
    simulation.setOmniDrive(robotIndex, drive.wheelJoints, drive.wheelAngles, drive.wheelRadius,
      drive.baseRadius);
    if (initialPose != null) teleport(initialPose);
  }

  /**
   * Jumps the chassis for the next tick while keeping wheel targets, sensor
   * history, and the chassis velocity; the jump itself does not read as motion.
   * The chassis keeps its height and its roll and pitch relative to its
   * heading, as the native plant does while it drives.
   */
  public function teleport(pose:Pose2):Void {
    if (pose == null) throw "Holonomic-drive plant pose cannot be null";
    BasePlacement.place(simulation, robotIndex, pose, baseHeight);
  }

  /** Advances one fixed simulation step and returns the robot's snapshot. */
  public function step(timestampNs:Int64):RobotSnapshot {
    simulation.step(timestampNs);
    return base.robot.snapshot();
  }

  /** Wheel rates, in rad/s, the robot applied during the latest step. */
  public function appliedWheelRates():Array<Float>
    return simulation.omniDriveState(robotIndex).wheelRates;

  function get_pose():Pose2 {
    var state = simulation.omniDriveState(robotIndex);
    return new Pose2(state.x, state.y, state.yaw);
  }

  function get_baseHeight():Float
    return simulation.omniDriveState(robotIndex).height;
}
