package robotkit.mobile;

import robotkit.world.JointTarget;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.StopMode;

/** Configured mobile-drive view over an ordinary Robot instance. */
class MobileBase {
  public final robot:Robot;
  public final driveModel:DriveModel;
  public final motionLimits:MotionLimits;
  public final footprint:Null<Footprint>;
  var previousCommand:Twist2 = new Twist2();

  public function new(robot:Robot, driveModel:DriveModel, motionLimits:MotionLimits,
      ?footprint:Footprint) {
    if (robot == null || driveModel == null || motionLimits == null)
      throw "MobileBase requires a robot, drive model, and motion limits";
    this.robot = robot;
    this.driveModel = driveModel;
    this.motionLimits = motionLimits;
    this.footprint = footprint;
  }

  /** Limits a body command and submits all drive joints as one RobotCommand. */
  public function command(twist:Twist2, ?durationSeconds:Float):Twist2 {
    var bounded = motionLimits.constrain(twist, previousCommand, durationSeconds);
    bounded = driveModel.constrain(bounded);
    var targets = driveModel.targets(bounded);
    if (targets == null || targets.length == 0)
      throw "Drive model produced no joint targets";
    var capabilities = robot.capabilities();
    if (capabilities.jointCount > 0) {
      for (target in targets) {
        if (target.joint >= capabilities.jointCount)
          throw 'Drive model targets joint ${target.joint}, but robot has ${capabilities.jointCount} joints';
        var supported = switch target.mode {
          case robotkit.world.JointTargetMode.Position: capabilities.supportsPosition;
          case robotkit.world.JointTargetMode.Velocity: capabilities.supportsVelocity;
          case robotkit.world.JointTargetMode.Effort: capabilities.supportsEffort;
        };
        if (!supported)
          throw 'Robot does not support ${Std.string(target.mode)} joint targets';
      }
    }
    // copyBatch validates indices, values, and duplicate joint targets before submission.
    robot.submit(RobotCommand.JointTargets(JointTarget.copyBatch(targets), null));
    previousCommand = bounded;
    return bounded;
  }

  public function stop(?mode:StopMode = StopMode.Normal):Void {
    robot.stop(mode);
    previousCommand = new Twist2();
  }
}
