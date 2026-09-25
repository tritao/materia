package robotkit.material;

import robotkit.world.JointTarget;
import robotkit.world.JointTargetMode;
import robotkit.world.Robot;
import robotkit.world.RobotCommand;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntimeConfiguration;
import robotkit.runtime.RobotRuntimeForkAxisConfiguration;
import robotkit.runtime.RobotRuntimeForkConfiguration;

/** Named fork mechanism view over a normal Robot instance. */
class Forks {
  public final robot:Robot;
  public final config:ForkConfig;
  public var loadState(default, null):LoadState = LoadState.unknown();

  final liftIndex:Int;
  final tiltIndex:Null<Int>;
  final spreadIndex:Null<Int>;

  /** Builds the fork view from roles and limits authored on a RobotModel. */
  public static function fromRobot(robot:Robot, model:RobotModel):Forks
    return fromBlueprint(robot, RobotRuntimeCompiler.compile(model));

  /** Builds the fork view from a previously compiled robot blueprint. */
  public static function fromBlueprint(robot:Robot,
      blueprint:RobotRuntimeBlueprint):Forks {
    if (robot == null || blueprint == null)
      throw "Robot blueprint has no fork-mechanism configuration";
    var configuration:Null<RobotRuntimeConfiguration> = blueprint.configuration;
    if (configuration == null)
      throw "Robot blueprint has no fork-mechanism configuration";
    var maybeConfig:Null<RobotRuntimeForkConfiguration> = configuration.forks;
    if (maybeConfig == null) throw "Robot blueprint has no fork-mechanism configuration";
    var config:RobotRuntimeForkConfiguration = cast maybeConfig;
    requireJoint(robot, config.lift);
    if (config.tilt != null) requireJoint(robot, cast config.tilt);
    if (config.spread != null) requireJoint(robot, cast config.spread);
    return new Forks(robot, new ForkConfig(axisConfig(config.lift),
      new LoadLimits(config.maxMassKg, config.maxLoadMomentKgMeters,
        config.maxLiftHeightMeters),
      config.tilt == null ? null : axisConfig(cast config.tilt),
      config.spread == null ? null : axisConfig(cast config.spread)));
  }

  static function axisConfig(value:RobotRuntimeForkAxisConfiguration):ForkAxisConfig
    return new ForkAxisConfig(value.jointName, value.minimum, value.maximum);

  static function requireJoint(robot:Robot,
      expected:RobotRuntimeForkAxisConfiguration):Void {
    var joints = robot.description().joints;
    if (expected.jointIndex < 0 || expected.jointIndex >= joints.length ||
        joints[expected.jointIndex] != expected.jointName ||
        joints.indexOf(expected.jointName) != expected.jointIndex)
      throw 'Robot description does not match authored joint "${expected.jointName}" at index ${expected.jointIndex}';
  }

  public function new(robot:Robot, config:ForkConfig) {
    if (robot == null || config == null)
      throw "Forks requires a robot and fork configuration";
    var joints = robot.description().joints;
    liftIndex = resolve(joints, config.lift);
    tiltIndex = config.tilt == null ? null : resolve(joints, config.tilt);
    spreadIndex = config.spread == null ? null : resolve(joints, config.spread);
    this.robot = robot;
    this.config = config;
  }

  public function state():ForkState {
    var snapshot = robot.snapshot();
    return new ForkState(robot.id(), readAxis(snapshot, config.lift, liftIndex),
      config.tilt == null ? null : readAxis(snapshot, config.tilt, cast tiltIndex),
      config.spread == null ? null : readAxis(snapshot, config.spread, cast spreadIndex),
      snapshot.sourceTimestampNs, snapshot.receivedTimestampNs,
      snapshot.sourceClockId, snapshot.receivedClockId);
  }

  public function setLoadState(value:LoadState):Void {
    if (value == null) throw "Fork load state cannot be null";
    loadState = value;
  }

  /** Submits all requested axis positions in one heterogeneous-safe joint batch. */
  public function command(?lift:Float, ?tilt:Float, ?spread:Float):Void {
    if (lift == null && tilt == null && spread == null)
      throw "Fork command requires at least one target";
    if (tilt != null && config.tilt == null)
      throw "Fork tilt axis is not configured";
    if (spread != null && config.spread == null)
      throw "Fork spread axis is not configured";
    if (lift != null) config.lift.validate(lift);
    if (tilt != null) config.tilt.validate(tilt);
    if (spread != null) config.spread.validate(spread);

    var current = state();
    var resultingLift = lift == null ? current.lift.position : lift;
    var violation = config.loadLimits.violation(loadState.payload, resultingLift);
    if (violation != null) throw violation;

    var targets:Array<JointTarget> = [];
    if (lift != null) targets.push(JointTarget.position(liftIndex, lift));
    if (tilt != null) targets.push(JointTarget.position(cast tiltIndex, tilt));
    if (spread != null) targets.push(JointTarget.position(cast spreadIndex, spread));
    var capabilities = robot.capabilities();
    if (capabilities.jointCount > 0) {
      if (!capabilities.supportsPosition)
        throw "Robot does not support fork position targets";
      for (target in targets) if (target.joint >= capabilities.jointCount)
        throw 'Fork target joint ${target.joint} exceeds robot joint count ${capabilities.jointCount}';
    }
    robot.submit(RobotCommand.JointTargets(JointTarget.copyBatch(targets), null));
  }

  public function raise(heightMeters:Float):Void command(heightMeters, null, null);
  public function tiltTo(angleRadians:Float):Void command(null, angleRadians, null);
  public function spreadTo(widthMeters:Float):Void command(null, null, widthMeters);

  static function resolve(joints:Array<String>, axis:ForkAxisConfig):Int {
    var index = joints.indexOf(axis.jointName);
    if (index < 0) throw 'Fork joint "${axis.jointName}" is missing from the robot description';
    return index;
  }

  static function readAxis(snapshot:robotkit.world.RobotSnapshot,
      config:ForkAxisConfig, index:Int):ForkAxisState {
    if (snapshot.positions.length <= index || snapshot.velocities.length <= index ||
        snapshot.efforts.length <= index)
      throw 'Robot snapshot does not contain fork joint "${config.jointName}"';
    return new ForkAxisState(snapshot.positions.get(index), snapshot.velocities.get(index),
      snapshot.efforts.get(index));
  }
}
