package robotkit.mobile;

import kinematicskit.UnicycleEnvelope;

import robotkit.core.JointTarget;
import robotkit.core.Robot;
import robotkit.core.RobotCommand;
import robotkit.core.StopMode;
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
  public var velocityEnvelope(default, null):Null<UnicycleEnvelope> = null;
  public var safetyStopRequired(default, null):Bool = false;
  var previousCommand:Twist2 = new Twist2();

  /** Builds the mobile view from roles and dimensions authored on a RobotProfile, resolved against a RobotModel. */
  public static function fromRobot(robot:Robot, model:RobotModel, profile:robotkit.profile.RobotProfile):MobileBase
    return fromBlueprint(robot, RobotRuntimeCompiler.compile(model, profile));

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
          rightIndex, rightName, wheelRadius, trackWidth, leftDirection, rightDirection):
        requireJoint(robot, leftIndex, leftName);
        requireJoint(robot, rightIndex, rightName);
        new DifferentialDrive(leftIndex, rightIndex, wheelRadius, trackWidth, leftDirection, rightDirection);
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
    var result = new MobileBase(robot, drive, new MotionLimits(config.maxLinearSpeed,
      config.maxAngularSpeed, config.maxLinearAcceleration,
      config.maxAngularAcceleration), footprint);
    if (Std.isOfType(drive, DifferentialDrive)) {
      var differential:DifferentialDrive = cast drive;
      var left = blueprint.joints[differential.leftWheelJoint].maxRate;
      var right = blueprint.joints[differential.rightWheelJoint].maxRate;
      if (left != null || right != null) result.velocityEnvelope = new UnicycleEnvelope(
        Math.min(left == null ? Math.POSITIVE_INFINITY : left, right == null ? Math.POSITIVE_INFINITY : right) * differential.wheelRadius,
        differential.trackWidth);
    }
    return result;
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
        limits.maxAngularAcceleration > nominalMotionLimits.maxAngularAcceleration ||
        limits.maxLateralSpeed > nominalMotionLimits.maxLateralSpeed ||
        limits.maxLateralAcceleration > nominalMotionLimits.maxLateralAcceleration)
      throw "Safety motion limits cannot exceed the authored mobile-base limits";
    motionLimits = limits;
  }

  public function clearSafetyLimits():Void motionLimits = nominalMotionLimits;

  /** Blocks MobileBase commands while a user-level policy requires a stop. */
  public function applySafetyStop(required:Bool):Void safetyStopRequired = required;

  /** Maximum translation speed for a path curvature, including simultaneous wheel motion. */
  public function pathSpeed(curvature:Float):Float
    return velocityEnvelope == null ? motionLimits.maxLinearSpeed : Math.min(motionLimits.maxLinearSpeed, velocityEnvelope.pathSpeed(curvature));

  public function currentCommand():Twist2
    return new Twist2(previousCommand.linear, previousCommand.angular, previousCommand.lateral);

  /** Limits a body command and submits all drive joints as one RobotCommand. */
  public function command(twist:Twist2, ?durationSeconds:Float):Twist2 {
    if (safetyStopRequired) throw "MobileBase command rejected by an active safety stop";
    var bounded = motionLimits.constrain(twist, previousCommand,
      velocityEnvelope == null ? durationSeconds : null);
    bounded = driveModel.constrain(bounded);
    if (velocityEnvelope != null) {
      var allowed = velocityEnvelope.constrain(bounded.linear, bounded.angular);
      bounded = new Twist2(allowed.linear, allowed.angular, bounded.lateral);
      if (durationSeconds != null) {
        bounded = motionLimits.constrain(bounded, previousCommand, durationSeconds);
        var envelope:UnicycleEnvelope = cast velocityEnvelope;
        if (Math.abs(bounded.linear) + Math.abs(bounded.angular) * envelope.trackWidth / 2 >
            envelope.groundSpeed + 1e-12) {
          // Independent acceleration clamps can leave the wheel envelope.
          // The segment from the previous feasible command stays inside both
          // acceleration bounds; stop it at the envelope boundary.
          var low = 0.0, high = 1.0;
          for (_ in 0...30) {
            var middle = (low + high) / 2;
            var linear = previousCommand.linear + middle * (bounded.linear - previousCommand.linear);
            var angular = previousCommand.angular + middle * (bounded.angular - previousCommand.angular);
            if (Math.abs(linear) + Math.abs(angular) * envelope.trackWidth / 2 <= envelope.groundSpeed)
              low = middle;
            else high = middle;
          }
          bounded = new Twist2(previousCommand.linear + low * (bounded.linear - previousCommand.linear),
            previousCommand.angular + low * (bounded.angular - previousCommand.angular),
            previousCommand.lateral + low * (bounded.lateral - previousCommand.lateral));
        }
      }
    }
    var targets = driveModel.targets(bounded);
    if (targets == null || targets.length == 0)
      throw "Drive model produced no joint targets";
    var capabilities = robot.capabilities();
    if (capabilities.jointCount > 0) {
      for (target in targets) {
        if (target.joint >= capabilities.jointCount)
          throw 'Drive model targets joint ${target.joint}, but robot has ${capabilities.jointCount} joints';
        var supported = capabilities.accepts(target.mode);
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
