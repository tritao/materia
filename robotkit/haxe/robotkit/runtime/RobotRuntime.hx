package robotkit.runtime;

import RobotKitRuntime;
import haxe.Int64;
import nativekit.ffi.NativeKit;
import robotkit.world.CameraImage;
import robotkit.world.SensorFrame;
import robotkit.world.TrajectoryChunk;
import sys.thread.Mutex;

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
  final externalSensorLayout:Array<RobotRuntimeSensorBlueprint>;
  final externalMutex = new Mutex();
  final externalFrames:Map<String, SensorFrame> = new Map();
  var disposed:Bool = false;

  @:allow(robotkit.runtime.Simulation)
  private function new(owner:Ownedrk_robot_runtime, blueprint:RobotRuntimeBlueprint) {
    this.owner = owner;
    defaultMaxRates = [for (joint in blueprint.joints) joint.maxRate];
    defaultMaxEfforts = [for (joint in blueprint.joints) joint.maxEffort];
    sensorLayout = blueprint.nativeSensorLayout();
    externalSensorLayout = blueprint.externalSensorLayout();
  }

  /** Creates a standalone in-memory runtime with its own worker lifecycle. */
  public static function create(blueprint:RobotRuntimeBlueprint):RobotRuntime {
    var result = RobotKitRuntime.rk_robot_runtime_create(blueprint.nativeValue());
    check(result.status, "runtime.create");
    return new RobotRuntime(result.out_runtime, blueprint);
  }

  /** Creates a serial runtime using a deployed layout fingerprint and SI-unit error budget. */
  public static function createSerial(blueprint:RobotRuntimeBlueprint,
      devicePath:String, fingerprintHex:String, maxTargetError:Float,
      ?baud:Int = 115200):RobotRuntime {
    if (blueprint == null) throw "Serial runtime requires a compiled blueprint";
    if (devicePath == null || StringTools.trim(devicePath).length == 0)
      throw "Serial runtime requires a device path";
    if (fingerprintHex == null || !~/^[0-9a-fA-F]{32}$/.match(fingerprintHex) ||
        fingerprintHex.toLowerCase() == "00000000000000000000000000000000")
      throw "Serial runtime requires a nonzero 32-digit fingerprint";
    if (!Math.isFinite(maxTargetError) || maxTargetError < 0.0)
      throw "Serial runtime requires a finite nonnegative target error budget";
    var result = RobotKitRuntime.rk_robot_runtime_create_serial(
      blueprint.nativeValue(), devicePath, baud, fingerprintHex, maxTargetError);
    check(result.status, "runtime.createSerial");
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

  /** Reports the endpoint's actual buffered-trajectory capability. */
  public function supportsTrajectoryQueue():Bool {
    ensureLive();
    var value = new rk_robot_capabilities();
    value.set_struct_size(rk_robot_capabilities.size());
    check(RobotKitRuntime.rk_robot_runtime_capabilities(owner.borrow(), value).status,
      "runtime.capabilities");
    return value.get_supports_trajectory_queue() != 0;
  }

  /** Submits a complete heterogeneous joint-target batch in one native call. */
  public function submitTargets(targets:Array<robotkit.world.JointTarget>, sequence:Int,
      ?timestampNs:haxe.Int64):Void {
    submitTargets64(targets, haxe.Int64.ofInt(sequence), timestampNs);
  }

  public function submitTargets64(targets:Array<robotkit.world.JointTarget>, sequence:haxe.Int64,
      ?timestampNs:haxe.Int64):Void {
    ensureLive();
    var batch = robotkit.world.JointTarget.copyBatch(targets);
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(sequence);
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

  /** Appends a timestamped position chunk to the native runtime queue. */
  public function submitTrajectory(chunk:TrajectoryChunk, sequence:Int,
      ?timestampNs:haxe.Int64):Void {
    submitTrajectory64(chunk, haxe.Int64.ofInt(sequence), timestampNs);
  }

  public function submitTrajectory64(chunk:TrajectoryChunk, sequence:haxe.Int64,
      ?timestampNs:haxe.Int64):Void {
    ensureLive();
    if (chunk == null) throw "Trajectory chunk is required";
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(sequence);
    command.set_timestamp_ns(timestampNs == null ? haxe.Int64.ofInt(0) : timestampNs);
    command.set_kind(RobotKitRuntimeConstants.RK_COMMAND_TRAJECTORY_CHUNK);
    command.set_target_count(0);
    var payload = new rk_trajectory_chunk();
    payload.set_struct_size(rk_trajectory_chunk.size());
    payload.set_point_count(chunk.points.length);
    payload.set_tag(chunk.tag);
    payload.set_splice_tag(chunk.spliceTag);
    payload.set_splice_time_ns(chunk.spliceTimeNs);
    for (index in 0...chunk.points.length) {
      var source = chunk.points[index];
      var point = new rk_trajectory_point();
      point.set_time_from_start_ns(source.timeFromStartNs);
      point.set_joint_count(source.positions.length);
      point.set_reserved0(0);
      for (joint in 0...source.positions.length)
        point.set_positions(joint, source.positions[joint]);
      payload.set_points(index, point);
    }
    check(RobotKitRuntime.rk_robot_runtime_submit_trajectory(owner.borrow(), command, payload),
      "runtime.submitTrajectory");
  }

  /** Submits all position targets in one native call. */
  public function submitPositions(positions:Array<Float>, sequence:Int,
      ?timestampNs:haxe.Int64):Void {
    submitPositions64(positions, haxe.Int64.ofInt(sequence), timestampNs);
  }

  public function submitPositions64(positions:Array<Float>, sequence:haxe.Int64,
      ?timestampNs:haxe.Int64):Void {
    var targets:Array<robotkit.world.JointTarget> = [];
    for (index in 0...positions.length)
      targets.push(robotkit.world.JointTarget.position(index, positions[index]));
    submitTargets64(targets, sequence, timestampNs);
  }

  /** Submits one position target without implying ownership of a simulation tick. */
  public function submitPosition(joint:Int, targetValue:Float, sequence:Int,
      ?timestampNs:haxe.Int64):Void {
    submitTargets([robotkit.world.JointTarget.position(joint, targetValue)], sequence,
      timestampNs);
  }

  /** Submits a stop to the owner thread; it never steps synchronously. */
  public function submitStop(sequence:Int, emergency:Bool):Void {
    submitStop64(haxe.Int64.ofInt(sequence), emergency);
  }

  public function submitStop64(sequence:haxe.Int64, emergency:Bool):Void {
    ensureLive();
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(sequence);
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
    resetSafety64(haxe.Int64.ofInt(sequence));
  }

  public function resetSafety64(sequence:haxe.Int64):Void {
    ensureLive();
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(sequence);
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
    var native = RobotSnapshot.fromNative(value, sensorLayout);
    if (externalSensorLayout.length == 0) return native;
    var frames = native.sensors.toArray();
    externalMutex.acquire();
    for (config in externalSensorLayout) {
      var frame = externalFrames.get(config.id);
      if (frame != null) frames.push(frame);
    }
    externalMutex.release();
    return new RobotSnapshot(native.robotId, native.sequence, native.sourceTimestampNs,
      native.mode, native.safety, native.endpoint, native.faultCode, native.q.toArray(),
      native.dq.toArray(), native.effort.toArray(), native.receivedTimestampNs, frames);
  }

  /**
   * Publishes one observation of an externally sourced sensor (see
   * `RobotRuntimeSensorBlueprint.isExternalKind`) against its authored mount.
   * The runtime stamps the authored frame, link, and mount plus the receive
   * time; later snapshots carry the latest frame for each such sensor.
   * A `gnss_pose` frame carries latitude and longitude in degrees and ENU yaw
   * in radians; a `camera` frame carries an image and no values.
   */
  public function publishSensorFrame(sensorId:String, values:Array<Float>, sequence:Int64,
      sourceTimestampNs:Int64, sourceClockId:String, ?image:CameraImage):Void {
    ensureLive();
    if (sensorId == null || sensorId.length == 0 || values == null || sequence == null ||
        Int64.compare(sequence, Int64.ofInt(0)) <= 0 || sourceTimestampNs == null ||
        Int64.compare(sourceTimestampNs, Int64.ofInt(0)) < 0 || sourceClockId == null ||
        sourceClockId.length == 0)
      throw "Sensor publication requires a sensor ID, values, positive sequence, and source clock";
    for (value in values) if (!Math.isFinite(value))
      throw 'Sensor "$sensorId" publication has a non-finite value';
    var config:Null<RobotRuntimeSensorBlueprint> = null;
    for (candidate in externalSensorLayout)
      if (candidate.id == sensorId) { config = candidate; break; }
    if (config == null)
      throw 'Robot model has no externally sourced sensor "$sensorId"';
    var mounted:RobotRuntimeSensorBlueprint = cast config;
    switch mounted.kind {
      case "camera":
        if (image == null || values.length != 0)
          throw 'Camera "$sensorId" publication requires an image and no values';
      case "gnss_pose":
        if (image != null || values.length != 3)
          throw 'GNSS "$sensorId" publication requires latitude, longitude, and yaw';
      case _:
    }
    var frame = new SensorFrame(mounted.id, mounted.kind, mounted.frameId, sequence,
      sourceTimestampNs, values, NativeKit.nk_time_now_ns(), mounted.linkId,
      mounted.position.toArray(), mounted.rotation.toArray(), sourceClockId,
      "robotkit.monotonic", image);
    externalMutex.acquire();
    var previous = externalFrames.get(sensorId);
    if (previous != null && Int64.compare(sequence, previous.sequence) <= 0) {
      externalMutex.release();
      throw 'Sensor "$sensorId" received a stale sequence';
    }
    externalFrames.set(sensorId, frame);
    externalMutex.release();
  }

  /** Publishes one camera image; see `publishSensorFrame`. */
  public function publishCameraFrame(sensorId:String, image:CameraImage, sequence:Int64,
      sourceTimestampNs:Int64, ?sourceClockId:String = "unspecified"):Void
    publishSensorFrame(sensorId, [], sequence, sourceTimestampNs, sourceClockId, image);

  public function dispose():Void {
    if (disposed)
      return;
    stop();
    externalMutex.acquire();
    externalFrames.clear();
    externalMutex.release();
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
