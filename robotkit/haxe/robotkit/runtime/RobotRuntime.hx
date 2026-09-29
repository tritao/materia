package robotkit.runtime;

import RobotKitRuntime;
import haxe.Int64;
import nativekit.ffi.NativeKit;
import robotkit.world.CameraImage;
import robotkit.world.SensorFrame;
import robotkit.world.TrajectoryChunk;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventCodec;
import robotkit.world.ProcessHoldPolicy;
import robotkit.world.ProcessChannelDeclaration;
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
  @:allow(robotkit.runtime.Simulation)
  final owner:Ownedrk_robot_runtime;
  final defaultMaxRates:Array<Float>;
  final defaultMaxEfforts:Array<Float>;
  final sensorLayout:Array<RobotRuntimeSensorBlueprint>;
  public final channels:Array<ProcessChannelDeclaration>;
  final externalSensorLayout:Array<RobotRuntimeSensorBlueprint>;
  final externalMutex = new Mutex();
  final externalFrames:Map<String, SensorFrame> = new Map();
  var disposed:Bool = false;
  @:allow(robotkit.runtime.Simulation)
  var simulation:Null<Simulation>;

  @:allow(robotkit.runtime.Simulation)
  private function new(owner:Ownedrk_robot_runtime, blueprint:RobotRuntimeBlueprint) {
    this.owner = owner;
    defaultMaxRates = [for (joint in blueprint.joints) joint.maxRate];
    defaultMaxEfforts = [for (joint in blueprint.joints) joint.maxEffort];
    sensorLayout = blueprint.nativeSensorLayout();
    channels = blueprint.channels.copy();
    externalSensorLayout = blueprint.externalSensorLayout();
  }

  /** Contacts from the latest simulation tick. Standalone runtimes have none. */
  public function contacts():Array<RobotContact> {
    if (simulation == null) return [];
    return simulation.robotContacts(this);
  }

  /** Inactive proximity contacts on attached tool pieces. */
  public function toolProximity():Array<RobotContact>
    return contacts().filter(contact -> contact.toolPieceIndex >= 0 && !contact.active);

  public function hasExternalSensor(id:String, kind:String):Bool {
    for (sensor in externalSensorLayout)
      if (sensor.id == id && sensor.kind == kind) return true;
    return false;
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
      ?baud:Int = 115200, ?stepTickHz:Int = 40000,
      ?linkLossTimeoutNs:haxe.Int64, ?clockSyncBoundNs:haxe.Int64):RobotRuntime {
    if (blueprint == null) throw "Serial runtime requires a compiled blueprint";
    if (devicePath == null || StringTools.trim(devicePath).length == 0)
      throw "Serial runtime requires a device path";
    if (fingerprintHex == null || !~/^[0-9a-fA-F]{32}$/.match(fingerprintHex) ||
        fingerprintHex.toLowerCase() == "00000000000000000000000000000000")
      throw "Serial runtime requires a nonzero 32-digit fingerprint";
    if (!Math.isFinite(maxTargetError) || maxTargetError < 0.0)
      throw "Serial runtime requires a finite nonnegative target error budget";
    if (linkLossTimeoutNs == null) linkLossTimeoutNs = haxe.Int64.ofInt(500000000);
    if (clockSyncBoundNs == null) clockSyncBoundNs = haxe.Int64.ofInt(30000000);
    var result = RobotKitRuntime.rk_robot_runtime_create_serial6(
      blueprint.nativeValue(), devicePath, baud, fingerprintHex, maxTargetError,
      stepTickHz, linkLossTimeoutNs, clockSyncBoundNs, haxe.Int64.ofInt(100000));
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

  public function supportsExecutionPlans():Bool {
    ensureLive();
    var value = new rk_robot_capabilities();
    value.set_struct_size(rk_robot_capabilities.size());
    check(RobotKitRuntime.rk_robot_runtime_capabilities(owner.borrow(), value).status,
      "runtime.capabilities");
    return value.get_supports_execution_plans() != 0;
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
        case robotkit.world.JointTargetMode.Servo: RobotKitRuntimeConstants.RK_TARGET_SERVO;
      });
      if (targetValue.mode == robotkit.world.JointTargetMode.Servo) {
        var servo = new rk_joint_servo();
        servo.set_velocity(targetValue.servoVelocity);
        servo.set_stiffness(targetValue.stiffness);
        servo.set_damping(targetValue.damping);
        servo.set_feedforward(targetValue.feedforward);
        command.set_servos(index, servo);
      }
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

  /** Appends a bounded polynomial segment chunk to the native runtime queue. */
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
    command.set_kind(RobotKitRuntimeConstants.RK_COMMAND_TRAJECTORY_SEGMENTS);
    command.set_target_count(0);
    var payload = new rk_trajectory_segment_chunk();
    payload.set_struct_size(rk_trajectory_segment_chunk.size());
    payload.set_segment_count(chunk.segments.length);
    payload.set_tag(chunk.tag);
    for (index in 0...chunk.segments.length) {
      var source = chunk.segments[index];
      var segment = new rk_trajectory_segment();
      segment.set_time_from_start_ns(source.timeFromStartNs);
      segment.set_duration_ns(source.durationNs);
      segment.set_degree(source.degree);
      segment.set_joint_count(source.jointCount);
      for (joint in 0...source.jointCount) {
        var coefficients = new rk_trajectory_coefficients();
        for (degree in 0...source.degree + 1)
          coefficients.set_value(degree, source.coefficients[joint][degree]);
        segment.set_coefficients(joint, coefficients);
      }
      payload.set_segments(index, segment);
    }
    check(RobotKitRuntime.rk_robot_runtime_submit_segments(owner.borrow(), command, payload),
      "runtime.submitTrajectorySegments");
  }

  /** Accepts a plan atomically, including its revision and horizon checks. */
  public function submitPlan(plan:ExecutionPlanSubmission, sequence:Int):Void {
    ensureLive();
    if (plan == null) throw "Execution plan is required";
    var native = new rk_plan_submission();
    native.set_struct_size(rk_plan_submission.size());
    native.set_sequence(Int64.ofInt(sequence));
    native.set_plan_id(plan.planId);
    native.set_model_revision(plan.modelRevision);
    native.set_calibration_revision(plan.calibrationRevision);
    native.set_required_capabilities(plan.requiredCapabilities);
    native.set_ends_at_rest(plan.endsAtRest ? 1 : 0);
    native.set_reserved0(plan.jerkUnchecked ? 1 : 0);
    native.set_event_count(plan.events.length);
    for (index in 0...plan.events.length) {
      var authored = plan.events[index];
      var event = new rk_timed_event();
      event.set_time_ns(authored.timeNs);
      for (i in 0...authored.channel.length)
        event.set_channel(i, authored.channel.charCodeAt(i));
      event.set_value(ProcessEventCodec.encode(authored.value));
      event.set_hold_policy(switch authored.holdPolicy {
        case Keep: RobotKitRuntimeConstants.RK_EVENT_KEEP;
        case SafeWhileHeld: RobotKitRuntimeConstants.RK_EVENT_SAFE_WHILE_HELD;
        case RestoreOnResume: RobotKitRuntimeConstants.RK_EVENT_RESTORE_ON_RESUME;
      });
      native.set_events(index, event);
    }
    native.set_replace_after_plan_id(plan.replaceAfterPlanId);
    native.set_replace_after_time_ns(plan.replaceAfterTimeNs);
    var positions = plan.startPosition.toArray();
    var velocities = plan.startVelocity.toArray();
    var accelerations = plan.startAcceleration.toArray();
    var positionTolerances = plan.positionTolerances.toArray();
    var velocityTolerances = plan.velocityTolerances.toArray();
    var accelerationTolerances = plan.accelerationTolerances.toArray();
    for (joint in 0...positions.length) {
      native.set_start_position(joint, positions[joint]);
      native.set_start_velocity(joint, velocities[joint]);
      native.set_start_acceleration(joint, accelerations[joint]);
      native.set_position_tolerance(joint, positionTolerances[joint]);
      native.set_velocity_tolerance(joint, velocityTolerances[joint]);
      native.set_acceleration_tolerance(joint, accelerationTolerances[joint]);
    }
    var payload = new rk_trajectory_segment_chunk();
    payload.set_struct_size(rk_trajectory_segment_chunk.size());
    payload.set_segment_count(plan.segments.length);
    payload.set_tag(plan.planId);
    for (index in 0...plan.segments.length) {
      var source = plan.segments[index];
      var segment = new rk_trajectory_segment();
      segment.set_time_from_start_ns(source.timeFromStartNs);
      segment.set_duration_ns(source.durationNs);
      segment.set_degree(source.degree);
      segment.set_joint_count(source.jointCount);
      for (joint in 0...source.jointCount) {
        var coefficients = new rk_trajectory_coefficients();
        for (degree in 0...source.degree + 1)
          coefficients.set_value(degree, source.coefficients[joint][degree]);
        segment.set_coefficients(joint, coefficients);
      }
      payload.set_segments(index, segment);
    }
    native.set_segments(payload);
    check(RobotKitRuntime.rk_robot_runtime_submit_plan(owner.borrow(), native),
      "runtime.submitPlan");
  }

  /** Drains output changes produced by the runtime owner clock. */
  public function pollEvents():{events:Array<FiredProcessEvent>, overflow:Bool} {
    ensureLive();
    var batch = new rk_event_record_batch();
    batch.set_struct_size(rk_event_record_batch.size());
    check(RobotKitRuntime.rk_robot_runtime_poll_events(owner.borrow(), batch),
      "runtime.pollEvents");
    var result:Array<FiredProcessEvent> = [];
    for (index in 0...batch.get_count()) {
      var record = batch.get_records(index);
      result.push(new FiredProcessEvent(record.get_plan_id(),
        ProcessEventCodec.readChannel(record), ProcessEventCodec.decode(record.get_value()),
        record.get_scheduled_time_ns(), record.get_applied_owner_time_ns(),
        record.get_cause()));
    }
    return {events:result, overflow:batch.get_overflow() != 0};
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

  /** Pauses, resumes, or aborts a native execution path. */
  public function submitHold(sequence:Int):Void
    submitLifecycle(sequence, RobotKitRuntimeConstants.RK_COMMAND_HOLD);

  public function submitResume(sequence:Int):Void
    submitLifecycle(sequence, RobotKitRuntimeConstants.RK_COMMAND_RESUME);

  public function submitAbort(sequence:Int):Void
    submitLifecycle(sequence, RobotKitRuntimeConstants.RK_COMMAND_ABORT);

  function submitLifecycle(sequence:Int, kind:Int):Void {
    ensureLive();
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(haxe.Int64.ofInt(sequence));
    command.set_kind(kind);
    check(RobotKitRuntime.rk_robot_runtime_submit(owner.borrow(), command),
      "runtime.submitLifecycle");
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
      native.dq.toArray(), native.effort.toArray(), native.receivedTimestampNs, frames,
      native.trajectoryQueueDepth, native.trajectoryActive,
      native.trajectoryTimeNs, native.trajectoryDurationNs,
      native.trajectoryTag, native.trajectoryTagTimeNs,
      native.modelRevision, native.calibrationRevision,
      native.sessionState, native.activePlanId,
      native.committedUntilNs, native.queueEndTimeNs);
  }

  /**
   * Publishes one observation of an externally sourced sensor (see
   * `RobotRuntimeSensorBlueprint.isExternalKind`) against its authored mount.
   * The runtime stamps the authored frame, link, and mount plus the receive
   * time; later snapshots carry the latest frame for each such sensor.
   * A `gnss_pose` frame carries latitude and longitude in degrees and ENU yaw
   * in radians; a `camera` frame carries an image and no values. Tool contact
   * carries one digital value; tool vacuum carries one non-negative kPa value.
   */
  public function publishSensorFrame(sensorId:String, values:Array<Float>, sequence:Int64,
      sourceTimestampNs:Int64, sourceClockId:String, ?image:CameraImage):Void {
    publishSensorFrameAndGet(sensorId, values, sequence, sourceTimestampNs,
      sourceClockId, image);
  }

  /** Publish and return the exact frame retained by the runtime snapshot. */
  public function publishSensorFrameAndGet(sensorId:String, values:Array<Float>, sequence:Int64,
      sourceTimestampNs:Int64, sourceClockId:String, ?image:CameraImage):SensorFrame {
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
      case "tool_contact":
        if (image != null || values.length != 1 || (values[0] != 0.0 && values[0] != 1.0))
          throw 'Contact "$sensorId" publication requires one digital value';
      case "tool_vacuum_kpa":
        if (image != null || values.length != 1 || values[0] < 0.0)
          throw 'Vacuum "$sensorId" publication requires one non-negative kPa value';
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
    return frame;
  }

  /** Publishes one camera image; see `publishSensorFrame`. */
  public function publishCameraFrame(sensorId:String, image:CameraImage, sequence:Int64,
      sourceTimestampNs:Int64, ?sourceClockId:String = "unspecified"):Void {
    publishSensorFrame(sensorId, [], sequence, sourceTimestampNs, sourceClockId, image);
  }

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
      throw new RobotRuntimeError(status, operation);
  }
}
