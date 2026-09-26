package robotkit.mobile;

import robotkit.world.JointTarget;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.world.StopMode;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntimeConfiguration;
import robotkit.runtime.RobotRuntimeDriveConfiguration;
import robotkit.runtime.RobotRuntimeMobileConfiguration;

/** Configured mobile-drive view over an ordinary Robot instance. */
class MobileBase {
  public final robot:Robot;
  public final driveModel:DriveModel;
  public final nominalMotionLimits:MotionLimits;
  public var motionLimits(default, null):MotionLimits;
  public final footprint:Null<Footprint>;
  public var safetyStopRequired(default, null):Bool = false;
  var previousCommand:Twist2 = new Twist2();

  /** Builds the mobile view from roles and dimensions authored on a RobotModel. */
  public static function fromRobot(robot:Robot, model:RobotModel):MobileBase
    return fromBlueprint(robot, RobotRuntimeCompiler.compile(model));

  /** Builds the mobile view from a previously compiled robot blueprint. */
  public static function fromBlueprint(robot:Robot,
      blueprint:RobotRuntimeBlueprint):MobileBase {
    if (robot == null || blueprint == null)
      throw "Robot blueprint has no mobile-base configuration";
    var configuration:Null<RobotRuntimeConfiguration> = blueprint.configuration;
    if (configuration == null)
      throw "Robot blueprint has no mobile-base configuration";
    var maybeConfig:Null<RobotRuntimeMobileConfiguration> = configuration.mobileBase;
    if (maybeConfig == null) throw "Robot blueprint has no mobile-base configuration";
    var config:RobotRuntimeMobileConfiguration = cast maybeConfig;
    var drive:DriveModel = switch config.drive {
      case RobotRuntimeDriveConfiguration.Differential(leftIndex, leftName,
          rightIndex, rightName, wheelRadius, trackWidth):
        requireJoint(robot, leftIndex, leftName);
        requireJoint(robot, rightIndex, rightName);
        new DifferentialDrive(leftIndex, rightIndex, wheelRadius, trackWidth);
      case RobotRuntimeDriveConfiguration.Ackermann(steeringIndex, steeringName,
          driveIndex, driveName, wheelBase, wheelRadius, maxSteeringAngle):
        requireJoint(robot, steeringIndex, steeringName);
        requireJoint(robot, driveIndex, driveName);
        new AckermannDrive(steeringIndex, driveIndex, wheelBase,
          wheelRadius, maxSteeringAngle);
      case RobotRuntimeDriveConfiguration.Holonomic(wheelIndices, wheelNames, wheelRadius, baseRadius):
        for (index in 0...wheelIndices.length) requireJoint(robot, wheelIndices[index], wheelNames[index]);
        new HolonomicDrive(wheelIndices, wheelRadius, baseRadius);
    };
    var footprint = config.footprintLength == null ? null : Footprint.rectangle(
      config.footprintLength, cast config.footprintWidth);
    return new MobileBase(robot, drive, new MotionLimits(config.maxLinearSpeed,
      config.maxAngularSpeed, config.maxLinearAcceleration,
      config.maxAngularAcceleration), footprint);
  }

  static function requireJoint(robot:Robot, index:Int, expectedName:String):Void {
    var joints = robot.description().joints;
    if (index < 0 || index >= joints.length || joints[index] != expectedName ||
        joints.indexOf(expectedName) != index)
      throw 'Robot description does not match authored joint "$expectedName" at index $index';
  }

  public function new(robot:Robot, driveModel:DriveModel, motionLimits:MotionLimits,
      ?footprint:Footprint) {
    if (robot == null || driveModel == null || motionLimits == null)
      throw "MobileBase requires a robot, drive model, and motion limits";
    this.robot = robot;
    this.driveModel = driveModel;
    this.nominalMotionLimits = motionLimits;
    this.motionLimits = motionLimits;
    this.footprint = footprint;
  }

  /** Applies user-level safety setpoints that can only lower authored limits. */
  public function applySafetyLimits(limits:MotionLimits):Void {
    if (limits == null) throw "Safety motion limits cannot be null";
    if (limits.maxLinearSpeed > nominalMotionLimits.maxLinearSpeed ||
        limits.maxAngularSpeed > nominalMotionLimits.maxAngularSpeed ||
        limits.maxLinearAcceleration > nominalMotionLimits.maxLinearAcceleration ||
        limits.maxAngularAcceleration > nominalMotionLimits.maxAngularAcceleration)
      throw "Safety motion limits cannot exceed the authored mobile-base limits";
    motionLimits = limits;
  }

  public function clearSafetyLimits():Void motionLimits = nominalMotionLimits;

  /** Blocks MobileBase commands while a user-level policy requires a stop. */
  public function applySafetyStop(required:Bool):Void safetyStopRequired = required;

  public function currentCommand():Twist2
    return new Twist2(previousCommand.linear, previousCommand.angular);

  /** Limits a body command and submits all drive joints as one RobotCommand. */
  public function command(twist:Twist2, ?durationSeconds:Float):Twist2 {
    if (safetyStopRequired) throw "MobileBase command rejected by an active safety stop";
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
