package robotkit.runtime;

import RobotKitRuntime;

/**
 * Per-robot realtime execution boundary.
 *
 * A `RobotRuntime` owns one command mailbox and one immutable state stream.
 * Standalone runtimes own a worker lifecycle; runtimes created by a
 * `Simulation` are externally driven and advance only through that shared
 * simulation so all robots observe one physics tick.
 */
class RobotRuntime {
  final owner:Ownedrk_robot_runtime;
  final defaultMaxRates:Array<Float>;
  final defaultMaxEfforts:Array<Float>;
  final sensorLayout:Array<RobotRuntimeSensorBlueprint>;
  var disposed:Bool = false;

  @:allow(robotkit.runtime.Simulation)
  private function new(owner:Ownedrk_robot_runtime, blueprint:RobotRuntimeBlueprint) {
    this.owner = owner;
    defaultMaxRates = [for (joint in blueprint.joints) joint.maxRate];
    defaultMaxEfforts = [for (joint in blueprint.joints) joint.maxEffort];
    sensorLayout = blueprint.sensorLayout();
  }

  /** Creates a standalone in-memory runtime with its own worker lifecycle. */
  public static function create(blueprint:RobotRuntimeBlueprint):RobotRuntime {
    var result = RobotKitRuntime.rk_robot_runtime_create(blueprint.nativeValue());
    check(result.status, "runtime.create");
    return new RobotRuntime(result.out_runtime, blueprint);
  }

  /** Creates a standalone runtime over a POSIX serial robot endpoint. */
  public static function createSerial(blueprint:RobotRuntimeBlueprint,
      devicePath:String, ?baud:Int = 115200):RobotRuntime {
    if (blueprint == null) throw "Serial runtime requires a compiled blueprint";
    if (devicePath == null || StringTools.trim(devicePath).length == 0)
      throw "Serial runtime requires a device path";
    var result = RobotKitRuntime.rk_robot_runtime_create_serial(
      blueprint.nativeValue(), devicePath, baud);
    check(result.status, "runtime.createSerial");
    return new RobotRuntime(result.out_runtime, blueprint);
  }

  /** Creates a v5 serial runtime using a deployed layout fingerprint and SI-unit error budget. */
  public static function createSerialV5(blueprint:RobotRuntimeBlueprint,
      devicePath:String, fingerprintHex:String, maxTargetError:Float,
      ?baud:Int = 115200):RobotRuntime {
    if (blueprint == null) throw "Serial v5 runtime requires a compiled blueprint";
    if (devicePath == null || StringTools.trim(devicePath).length == 0)
      throw "Serial v5 runtime requires a device path";
    if (fingerprintHex == null || !~/^[0-9a-fA-F]{32}$/.match(fingerprintHex) ||
        fingerprintHex.toLowerCase() == "00000000000000000000000000000000")
      throw "Serial v5 runtime requires a nonzero 32-digit fingerprint";
    if (!Math.isFinite(maxTargetError) || maxTargetError < 0.0)
      throw "Serial v5 runtime requires a finite nonnegative target error budget";
    var result = RobotKitRuntime.rk_robot_runtime_create_serial_v5(
      blueprint.nativeValue(), devicePath, baud, fingerprintHex, maxTargetError);
    check(result.status, "runtime.createSerialV5");
    return new RobotRuntime(result.out_runtime, blueprint);
  }

  /** Starts a standalone runtime worker; Simulation-owned runtimes reject this. */
  public function start():Void {
    ensureLive();
    check(RobotKitRuntime.rk_robot_runtime_start(owner.borrow()), "runtime.start");
  }

  /** Stops a standalone worker; it never advances a shared Simulation. */
  public function stop():Void {
    if (disposed)
      return;
    check(RobotKitRuntime.rk_robot_runtime_stop(owner.borrow()), "runtime.stop");
  }

  /** Submits a complete heterogeneous joint-target batch in one native call. */
  public function submitTargets(targets:Array<robotkit.world.JointTarget>, sequence:Int,
      ?timestampNs:haxe.Int64):Void {
    ensureLive();
    var batch = robotkit.world.JointTarget.copyBatch(targets);
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(haxe.Int64.ofInt(sequence));
    command.set_timestamp_ns(timestampNs == null ? haxe.Int64.ofInt(0) : timestampNs);
    command.set_kind(RobotKitRuntimeConstants.RK_COMMAND_JOINT_TARGETS);
    command.set_target_count(batch.length);
    for (index in 0...batch.length) {
      var targetValue = batch[index];
      var target = new rk_joint_target();
      target.set_joint(targetValue.joint);
      target.set_mode(switch targetValue.mode {
        case robotkit.world.JointTargetMode.Position: RobotKitRuntimeConstants.RK_TARGET_POSITION;
        case robotkit.world.JointTargetMode.Velocity: RobotKitRuntimeConstants.RK_TARGET_VELOCITY;
        case robotkit.world.JointTargetMode.Effort: RobotKitRuntimeConstants.RK_TARGET_EFFORT;
      });
      target.set_target(targetValue.target);
      target.set_max_rate(targetValue.joint < defaultMaxRates.length
        ? defaultMaxRates[targetValue.joint] : 0.0);
      target.set_max_effort(targetValue.joint < defaultMaxEfforts.length
        ? defaultMaxEfforts[targetValue.joint] : 0.0);
      command.set_targets(index, target);
    }
    check(RobotKitRuntime.rk_robot_runtime_submit(owner.borrow(), command),
      "runtime.submitTargets");
  }

  /** Submits all position targets in one native call. */
  public function submitPositions(positions:Array<Float>, sequence:Int,
      ?timestampNs:haxe.Int64):Void {
    var targets:Array<robotkit.world.JointTarget> = [];
    for (index in 0...positions.length)
      targets.push(robotkit.world.JointTarget.position(index, positions[index]));
    submitTargets(targets, sequence, timestampNs);
  }

  /** Submits one position target without implying ownership of a simulation tick. */
  public function submitPosition(joint:Int, targetValue:Float, sequence:Int,
      ?timestampNs:haxe.Int64):Void {
    submitTargets([robotkit.world.JointTarget.position(joint, targetValue)], sequence,
      timestampNs);
  }

  /** Submits a stop to the owner thread; it never steps synchronously. */
  public function submitStop(sequence:Int, emergency:Bool):Void {
    ensureLive();
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(haxe.Int64.ofInt(sequence));
    command.set_timestamp_ns(haxe.Int64.ofInt(0));
    command.set_kind(emergency
      ? RobotKitRuntimeConstants.RK_COMMAND_EMERGENCY_STOP
      : RobotKitRuntimeConstants.RK_COMMAND_STOP);
    command.set_target_count(0);
    check(RobotKitRuntime.rk_robot_runtime_submit(owner.borrow(), command),
      "runtime.submitStop");
  }

  /** Clears a latched safety stop only after the application has acknowledged it. */
  public function resetSafety(sequence:Int):Void {
    ensureLive();
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(haxe.Int64.ofInt(sequence));
    command.set_timestamp_ns(haxe.Int64.ofInt(0));
    command.set_kind(RobotKitRuntimeConstants.RK_COMMAND_RESET_SAFETY);
    command.set_target_count(0);
    check(RobotKitRuntime.rk_robot_runtime_submit(owner.borrow(), command),
      "runtime.resetSafety");
  }

  /** Reads the latest published native state without advancing time. */
  public function snapshot():RobotSnapshot {
    ensureLive();
    var value = new rk_robot_snapshot();
    value.set_struct_size(rk_robot_snapshot.size());
    check(RobotKitRuntime.rk_robot_runtime_snapshot_full(owner.borrow(), value).status,
      "runtime.snapshot");
    return RobotSnapshot.fromNative(value, sensorLayout);
  }

  public function dispose():Void {
    if (disposed)
      return;
    stop();
    owner.close();
    disposed = true;
  }

  function ensureLive():Void {
    if (disposed)
      throw "RobotKit runtime has been disposed";
  }

  static function check(status:Int, operation:String):Void {
    if (status != RobotKitRuntimeConstants.RK_OK)
      throw '$operation failed with RobotKit status $status';
  }
}
