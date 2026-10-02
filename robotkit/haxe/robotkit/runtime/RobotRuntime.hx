package robotkit.runtime;

import RobotKitRuntime;
import haxe.Int64;
import nativekit.ffi.NativeKit;
import robotkit.device.DeviceBinding;
import robotkit.world.CameraImage;
import robotkit.world.SensorFrame;
import robotkit.world.TrajectoryChunk;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventCodec;
import robotkit.world.ProcessEventValue;
import robotkit.world.ProcessHoldPolicy;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessTimedEvent;
import robotkit.world.TrajectorySegment;
import runtime.memory.Arena;
import runtime.memory.NativeSpan;
import runtime.memory.RawPtr;
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
  /** The blueprint's joint couplings: a joint a plan leaves out follows its leader. */
  public final couplings:Array<robotkit.world.CoupledJoint>;
  final externalSensorLayout:Array<RobotRuntimeSensorBlueprint>;
  final externalMutex = new Mutex();
  final externalFrames:Map<String, SensorFrame> = new Map();

  /**
   * Native output buffers reused across calls. `rk_robot_snapshot` is 17 KB and `rk_event_record_batch` 10 KB
   * because their arrays are sized for the largest robot, and allocating and zeroing them on every simulation
   * tick was a large share of the tick. The native calls overwrite everything the readers use (`snapshot_full`
   * the whole struct, `poll_events` resets its count first), and readers copy values out before the lock is
   * released, so nothing retains a reference to the shared storage.
   */
  static final scratchMutex = new Mutex();
  static final snapshotScratch = new rk_robot_snapshot();
  static final eventBatchScratch = new rk_event_record_batch();
  static final planHeaderScratch = new rk_plan_header();
  static final jointMapScratch = new Arena(256);
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
    couplings = [for (coupling in blueprint.couplings)
      new robotkit.world.CoupledJoint(coupling.follower, coupling.leader, coupling.ratio, coupling.offset)];
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

  /**
   * Creates a serial runtime for the board `controllerHex` (its 32-digit unique id) wired as
   * `binding` says, within an SI-unit error budget. The blueprint should be compiled from the
   * binding's model so its limits are the device's. The board must be that controller and agree
   * with the configuration, or this throws with the device's reason on stderr.
   */
  public static function createSerial(blueprint:RobotRuntimeBlueprint,
      devicePath:String, controllerHex:String, binding:DeviceBinding, maxTargetError:Float,
      ?baud:Int = 115200, ?linkLossTimeoutNs:haxe.Int64, ?clockSyncBoundNs:haxe.Int64):RobotRuntime {
    if (blueprint == null) throw "Serial runtime requires a compiled blueprint";
    if (devicePath == null || StringTools.trim(devicePath).length == 0)
      throw "Serial runtime requires a device path";
    if (binding == null) throw "Serial runtime requires a device binding";
    if (controllerHex == null || !~/^[0-9a-fA-F]{32}$/.match(controllerHex) ||
        controllerHex.toLowerCase() == "00000000000000000000000000000000")
      throw "Serial runtime requires a nonzero 32-digit controller id";
    if (!Math.isFinite(maxTargetError) || maxTargetError < 0.0)
      throw "Serial runtime requires a finite nonnegative target error budget";
    if (linkLossTimeoutNs == null) linkLossTimeoutNs = haxe.Int64.ofInt(500000000);
    if (clockSyncBoundNs == null) clockSyncBoundNs = haxe.Int64.ofInt(30000000);
    var device = new rk_serial_device_desc();
    device.set_struct_size(rk_serial_device_desc.size());
    device.set_actuator_count(binding.channels.length);
    for (i in 0...16)
      device.set_controller(i, Std.parseInt("0x" + controllerHex.substr(i * 2, 2)));
    for (i in 0...binding.channels.length) {
      var channel = binding.channels[i];
      device.set_actuator_joint(i, channel.jointIndex);
      device.set_actuator_ratio(i, channel.ratio);
      device.set_actuator_offset(i, channel.offset);
      device.set_actuator_steps_per_unit(i, channel.stepsPerUnit);
      device.set_actuator_max_rate(i, channel.maxRate);
      device.set_actuator_direction_setup_ticks(i, channel.directionSetupTicks);
      device.set_actuator_skew_bound(i, channel.skewBound);
      if (channel.actuatorId.length > 63) throw "Serial actuator ID is longer than 63 characters";
      for (byte in 0...channel.actuatorId.length)
        device.set_actuator_ids(i * 64 + byte, channel.actuatorId.charCodeAt(byte));
    }
    var result = RobotKitRuntime.rk_robot_runtime_create_serial6(
      blueprint.nativeValue(), devicePath, baud, device, maxTargetError,
      binding.stepTickHz, linkLossTimeoutNs, clockSyncBoundNs, haxe.Int64.ofInt(100000));
    check(result.status, "runtime.createSerial");
    return new RobotRuntime(result.out_runtime, blueprint);
  }

  /** Reads the unique id (32 lowercase hex digits) of the board on a serial port. */
  public static function identifySerial(devicePath:String, baud:Int):String {
    if (devicePath == null || StringTools.trim(devicePath).length == 0)
      throw "Identifying a serial device requires a device path";
    var result = RobotKitRuntime.rk_serial_device_identify(devicePath, baud);
    check(result.status, "runtime.identifySerial");
    var text = "";
    for (i in 0...16) text += StringTools.hex(result.out_controller.get_bytes(i), 2).toLowerCase();
    return text;
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

  /**
   * Submits a complete heterogeneous joint-target batch in one native call.
   * `expiresAtNs` (robot source clock, as in snapshots) makes the batch's
   * velocity targets lapse then: the runtime brakes those joints to zero.
   */
  public function submitTargets(targets:Array<robotkit.world.JointTarget>, sequence:Int,
      ?timestampNs:haxe.Int64, ?expiresAtNs:haxe.Int64):Void {
    submitTargets64(targets, haxe.Int64.ofInt(sequence), timestampNs, expiresAtNs);
  }

  public function submitTargets64(targets:Array<robotkit.world.JointTarget>, sequence:haxe.Int64,
      ?timestampNs:haxe.Int64, ?expiresAtNs:haxe.Int64):Void {
    ensureLive();
    var batch = robotkit.world.JointTarget.copyBatch(targets);
    var command = new rk_robot_command();
    command.set_struct_size(rk_robot_command.size());
    command.set_sequence(sequence);
    command.set_timestamp_ns(timestampNs == null ? haxe.Int64.ofInt(0) : timestampNs);
    command.set_kind(RobotKitRuntimeConstants.RK_COMMAND_JOINT_TARGETS);
    command.set_expires_at_ns(expiresAtNs == null ? haxe.Int64.ofInt(0) : expiresAtNs);
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
    var arrays = SegmentValues.of(chunk.segments);
    check(RobotKitRuntime.rk_robot_runtime_submit_segments(owner.borrow(), command, chunk.tag,
      arrays.starts, arrays.durations, arrays.degrees, arrays.coefficients),
      "runtime.submitTrajectorySegments");
  }

  /**
    Accepts a plan atomically, including its revision and horizon checks. A
    plan carrying native segment arrays reaches the runtime in place and is
    copied once there; managed segments are flattened into arrays first. Only
    the small header is marshalled field by field, into storage reused across
    plans.
  **/
  public function submitPlan(plan:ExecutionPlanSubmission, sequence:Int):Void {
    ensureLive();
    if (plan == null) throw "Execution plan is required";
    var events = [for (event in plan.events) nativeEvent(event)];
    scratchMutex.acquire();
    try {
      var header = planHeader(plan, sequence);
      var arrays = plan.arrays;
      var status = if (arrays != null) {
        jointMapScratch.reset();
        var map:RawPtr<Int> = jointMapScratch.alloc(arrays.jointMap.length);
        for (joint in 0...arrays.jointMap.length)
          map.offset(joint).store(arrays.jointMap[joint]);
        RobotKitRuntime.rk_robot_runtime_submit_plan_span(owner.borrow(), header, arrays.starts,
          arrays.durations, arrays.degrees, arrays.coefficients,
          new NativeSpan<Int>(map, arrays.jointMap.length), events);
      } else {
        var values = SegmentValues.of(plan.segments);
        RobotKitRuntime.rk_robot_runtime_submit_plan(owner.borrow(), header, values.starts,
          values.durations, values.degrees, values.coefficients,
          [for (joint in 0...plan.startPosition.length) joint], events);
      }
      check(status, "runtime.submitPlan");
    } catch (error:Dynamic) {
      scratchMutex.release();
      throw error;
    }
    scratchMutex.release();
  }

  /** The plan's identity and start state, written into the shared header; call under scratchMutex. */
  static function planHeader(plan:ExecutionPlanSubmission, sequence:Int):rk_plan_header {
    var header = planHeaderScratch;
    header.set_struct_size(rk_plan_header.size());
    header.set_sequence(Int64.ofInt(sequence));
    header.set_plan_id(plan.planId);
    header.set_tag(plan.planId);
    header.set_model_revision(plan.modelRevision);
    header.set_calibration_revision(plan.calibrationRevision);
    header.set_required_capabilities(plan.requiredCapabilities);
    header.set_ends_at_rest(plan.endsAtRest ? 1 : 0);
    header.set_flags(plan.jerkUnchecked ? RobotKitRuntimeConstants.RK_PLAN_JERK_UNCHECKED : 0);
    header.set_replace_after_plan_id(plan.replaceAfterPlanId);
    header.set_replace_after_time_ns(plan.replaceAfterTimeNs);
    var positions = plan.startPosition.toArray(), velocities = plan.startVelocity.toArray(),
      accelerations = plan.startAcceleration.toArray(),
      positionTolerances = plan.positionTolerances.toArray(),
      velocityTolerances = plan.velocityTolerances.toArray(),
      accelerationTolerances = plan.accelerationTolerances.toArray();
    for (joint in 0...RobotKitRuntimeConstants.RK_MAX_TRAJECTORY_JOINTS) {
      var used = joint < positions.length;
      header.set_start_position(joint, used ? positions[joint] : 0.0);
      header.set_start_velocity(joint, used ? velocities[joint] : 0.0);
      header.set_start_acceleration(joint, used ? accelerations[joint] : 0.0);
      header.set_position_tolerance(joint, used ? positionTolerances[joint] : 0.0);
      header.set_velocity_tolerance(joint, used ? velocityTolerances[joint] : 0.0);
      header.set_acceleration_tolerance(joint, used ? accelerationTolerances[joint] : 0.0);
    }
    return header;
  }

  static function nativeEvent(authored:ProcessTimedEvent):rk_timed_event {
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
    return event;
  }

  /** Drains output changes produced by the runtime owner clock. */
  public function pollEvents():{events:Array<FiredProcessEvent>, overflow:Bool} {
    ensureLive();
    var result:Array<FiredProcessEvent> = [];
    var overflow = false;
    scratchMutex.acquire();
    try {
      eventBatchScratch.set_struct_size(rk_event_record_batch.size());
      check(RobotKitRuntime.rk_robot_runtime_poll_events(owner.borrow(), eventBatchScratch),
        "runtime.pollEvents");
      for (index in 0...eventBatchScratch.get_count()) {
        var record = eventBatchScratch.get_records(index);
        result.push(new FiredProcessEvent(record.get_plan_id(),
          ProcessEventCodec.readChannel(record), ProcessEventCodec.decode(record.get_value()),
          record.get_scheduled_time_ns(), record.get_applied_owner_time_ns(),
          record.get_cause()));
      }
      overflow = eventBatchScratch.get_overflow() != 0;
    } catch (error:Dynamic) {
      scratchMutex.release();
      throw error;
    }
    scratchMutex.release();
    return {events:result, overflow:overflow};
  }

  /**
   * A process channel's current output value, as the device on it sees it, without draining the
   * events `pollEvents` reports.
   */
  public function channelValue(channel:String):ProcessEventValue {
    ensureLive();
    var value = new rk_event_value();
    check(RobotKitRuntime.rk_robot_runtime_get_channel_value(owner.borrow(), channel, value),
      'runtime.channelValue("$channel")');
    return ProcessEventCodec.decode(value);
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
    var native:RobotSnapshot;
    scratchMutex.acquire();
    try {
      snapshotScratch.set_struct_size(rk_robot_snapshot.size());
      check(RobotKitRuntime.rk_robot_runtime_snapshot_full(owner.borrow(), snapshotScratch).status,
        "runtime.snapshot");
      native = RobotSnapshot.fromNative(snapshotScratch, sensorLayout);
    } catch (error:Dynamic) {
      scratchMutex.release();
      throw error;
    }
    scratchMutex.release();
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
      native.committedUntilNs, native.queueEndTimeNs, native.setpoint.toArray());
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

/** Managed segments flattened into the runtime's segment arrays, over every joint they carry. */
private class SegmentValues {
  public final starts:Array<Int64> = [];
  public final durations:Array<Int64> = [];
  public final degrees:Array<Int> = [];
  public final coefficients:Array<Float> = [];

  function new() {}

  public static function of(segments:Array<TrajectorySegment>):SegmentValues {
    var values = new SegmentValues();
    for (segment in segments) {
      values.starts.push(segment.timeFromStartNs);
      values.durations.push(segment.durationNs);
      values.degrees.push(segment.degree);
      for (joint in 0...segment.jointCount)
        for (power in 0...RobotKitRuntimeConstants.RK_TRAJECTORY_COEFFICIENT_STRIDE)
          values.coefficients.push(power <= segment.degree ? segment.coefficients[joint][power] : 0.0);
    }
    return values;
  }
}
