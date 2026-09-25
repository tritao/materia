package robotkit.runtime;

import haxe.Int64;
import robotkit.mobile.DifferentialOdometry;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.world.JointTargetMode;
import robotkit.world.RobotSnapshot;

/**
 * Ideal rolling-kinematics plant for a differential-drive robot in SimKit.
 * Wheel target rates drive a planar base pose; articulated joints and sensors
 * still advance through the shared native simulation.
 */
class DifferentialDrivePlant {
  public final simulation:Simulation;
  public final robotIndex:Int;
  public final base:MobileBase;
  public var pose(default, null):Pose2;

  final odometry:DifferentialOdometry;

  public function new(simulation:Simulation, robotIndex:Int, base:MobileBase,
      ?initialPose:Pose2) {
    if (simulation == null || robotIndex < 0 || base == null)
      throw "Differential-drive plant requires a simulation, robot index, and mobile base";
    var configuredOdometry = base.driveModel.createOdometry();
    if (configuredOdometry == null)
      throw "Differential-drive plant requires a differential drive model";
    this.simulation = simulation;
    this.robotIndex = robotIndex;
    this.base = base;
    odometry = configuredOdometry;
    pose = initialPose == null
      ? new Pose2()
      : new Pose2(initialPose.x, initialPose.y, initialPose.yaw);
  }

  /** Sets the chassis pose while preserving the current wheel targets. */
  public function teleport(pose:Pose2):Void {
    if (pose == null) throw "Differential-drive plant pose cannot be null";
    var value = new Pose2(pose.x, pose.y, pose.yaw);
    applyPose(value);
    this.pose = value;
  }

  /** Advances one fixed simulation step from the mobile base's wheel targets. */
  public function step(timestampNs:Int64):RobotSnapshot {
    var targetRates = base.driveModel.targets(base.currentCommand());
    var leftRate = 0.0;
    var rightRate = 0.0;
    var hasLeft = false;
    var hasRight = false;
    for (target in targetRates) {
      if (target.joint == odometry.leftWheelJoint) {
        if (target.mode != JointTargetMode.Velocity)
          throw "Differential-drive plant requires wheel velocity targets";
        leftRate = target.target;
        hasLeft = true;
      } else if (target.joint == odometry.rightWheelJoint) {
        if (target.mode != JointTargetMode.Velocity)
          throw "Differential-drive plant requires wheel velocity targets";
        rightRate = target.target;
        hasRight = true;
      }
    }
    if (!hasLeft || !hasRight)
      throw "Differential-drive plant did not receive both wheel targets";

    var leftDistance = leftRate * odometry.wheelRadius * simulation.fixedTimestepSeconds;
    var rightDistance = rightRate * odometry.wheelRadius * simulation.fixedTimestepSeconds;
    var distance = (leftDistance + rightDistance) * 0.5;
    var headingChange = (rightDistance - leftDistance) / odometry.trackWidth;
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
