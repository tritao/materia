package robotkit.runtime;

import haxe.Int64;
import robotkit.mobile.HolonomicDrive;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.world.JointTargetMode;
import robotkit.world.RobotSnapshot;

/**
 * Ideal rolling-kinematics plant for an omnidirectional-base robot in
 * SimKit, in the style of `DifferentialDrivePlant`: articulated joints and
 * sensors still advance through the shared native simulation, while the
 * chassis pose is integrated from the commanded body twist and teleported.
 *
 * `Twist2` (shared with `Navigator`/`GoTo`) carries only forward speed and
 * yaw rate, never a lateral term, so `HolonomicDrive.targets()` always
 * encodes a body motion with zero lateral component; decoding the three
 * wheel rates back through the general omni-kinematics inverse would
 * therefore reproduce exactly `twist.linear`/`twist.angular` (the encode is
 * invertible and lossless for that domain), the same relationship
 * `DifferentialDrivePlant`'s own wheel-rate reconstruction reduces to for a
 * differential pair. This plant integrates directly from the commanded
 * twist and only re-derives the wheel targets to validate that the drive
 * model's three wheel joints are the ones actually configured.
 */
class HolonomicDrivePlant {
  public final simulation:Simulation;
  public final robotIndex:Int;
  public final base:MobileBase;
  public var pose(default, null):Pose2;

  final drive:HolonomicDrive;

  public function new(simulation:Simulation, robotIndex:Int, base:MobileBase,
      ?initialPose:Pose2) {
    if (simulation == null || robotIndex < 0 || base == null)
      throw "Holonomic-drive plant requires a simulation, robot index, and mobile base";
    var configuredDrive:HolonomicDrive = cast(base.driveModel, HolonomicDrive);
    if (configuredDrive == null)
      throw "Holonomic-drive plant requires a holonomic drive model";
    this.simulation = simulation;
    this.robotIndex = robotIndex;
    this.base = base;
    drive = configuredDrive;
    pose = initialPose == null
      ? new Pose2()
      : new Pose2(initialPose.x, initialPose.y, initialPose.yaw);
  }

  /** Sets the chassis pose while preserving the current wheel targets. */
  public function teleport(pose:Pose2):Void {
    if (pose == null) throw "Holonomic-drive plant pose cannot be null";
    var value = new Pose2(pose.x, pose.y, pose.yaw);
    applyPose(value);
    this.pose = value;
  }

  /** Advances one fixed simulation step from the mobile base's commanded twist. */
  public function step(timestampNs:Int64):RobotSnapshot {
    var twist = base.currentCommand();
    var targets = drive.targets(twist);
    var seenWheels = new Map<Int, Bool>();
    for (target in targets) {
      if (target.mode != JointTargetMode.Velocity)
        throw "Holonomic-drive plant requires wheel velocity targets";
      seenWheels.set(target.joint, true);
    }
    for (wheelJoint in drive.wheelJoints)
      if (!seenWheels.exists(wheelJoint))
        throw "Holonomic-drive plant did not receive every wheel target";

    var distance = twist.linear * simulation.fixedTimestepSeconds;
    var headingChange = twist.angular * simulation.fixedTimestepSeconds;
    var nextPose = pose.integrateDisplacement(distance, headingChange);
    applyPose(nextPose);
    pose = nextPose;
    simulation.step(timestampNs);
    return base.robot.snapshot();
  }

  function applyPose(value:Pose2):Void {
    var halfYaw = value.yaw * 0.5;
    simulation.teleportRobot(robotIndex, [value.x, value.y, 0.0],
      [0.0, 0.0, Math.sin(halfYaw), Math.cos(halfYaw)]);
  }
}
