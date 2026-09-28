package motionkit.trajectory;

import MotionKitNative;
import haxe.Int64;
import motionkit.event.TimedEvent;
import motionkit.event.EventValue;
import motionkit.event.HoldPolicy;

/** Validated, immutable native trajectory with explicit start-state assumptions. */
class ExecutionPlan {
  final owner:Ownedmk_plan_handle;
  var disposed:Bool = false;
  public final report:ValidationReport;
  /** Validation claims carried with this plan's telemetry. */
  public function guarantees():ValidationGuarantees return report.guarantees();
  public final planId:Int64;
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  public final trajectoryRevision:Int64;
  public final requiredCapabilities:Int64;
  public final planningAuthority:Int;
  public final durationSeconds:Float;
  final storedEvents:Array<TimedEvent>;
  public var events(get, never):Array<TimedEvent>;
  final storedSegments:Array<{timeFromStartNs:Int64, durationNs:Int64,
    coefficients:Array<Array<Float>>}>;
  final storedStartPositions:Array<Float>;
  final storedStartVelocities:Array<Float>;
  final storedStartAccelerations:Array<Float>;
  final storedPositionTolerances:Array<Float>;
  final storedVelocityTolerances:Array<Float>;
  final storedAccelerationTolerances:Array<Float>;

  private function new(owner:Ownedmk_plan_handle, report:ValidationReport,
      trajectory:Trajectory, positions:Array<Float>, velocities:Array<Float>,
      accelerations:Array<Float>, pTol:Array<Float>, vTol:Array<Float>,
      aTol:Array<Float>) {
    this.owner = owner;
    this.report = report;
    storedSegments = trajectory.segments();
    storedStartPositions = positions.copy(); storedStartVelocities = velocities.copy();
    storedStartAccelerations = accelerations.copy();
    storedPositionTolerances = pTol.copy(); storedVelocityTolerances = vTol.copy();
    storedAccelerationTolerances = aTol.copy();
    var info = new mk_plan_info();
    info.set_struct_size(mk_plan_info.size());
    check(MotionKitNative.mk_plan_get_info(owner.borrow(), info), "plan.info");
    planId = info.get_plan_id();
    modelRevision = info.get_model_revision();
    calibrationRevision = info.get_calibration_revision();
    trajectoryRevision = info.get_trajectory_revision();
    requiredCapabilities = info.get_required_capabilities();
    planningAuthority = info.get_planning_authority();
    durationSeconds = Int64.toFloat(info.get_duration_ns()) * 1e-9;
    storedEvents = [];
    for (index in 0...info.get_event_count()) {
      var nativeEvent = new mk_timed_event();
      check(MotionKitNative.mk_plan_get_event(owner.borrow(), index, nativeEvent),
        "plan.event");
      var channel = new StringBuf();
      for (i in 0...MotionKitNativeConstants.MK_EVENT_CHANNEL_BYTES) {
        var code = nativeEvent.get_channel(i);
        if (code == 0) break;
        channel.addChar(code);
      }
      var nativeValue = nativeEvent.get_value();
      var value = switch nativeValue.get_kind() {
        case MotionKitNativeConstants.MK_EVENT_DIGITAL:
          EventValue.Digital(nativeValue.get_digital() != 0);
        case MotionKitNativeConstants.MK_EVENT_ANALOG:
          EventValue.Analog(nativeValue.get_analog());
        case MotionKitNativeConstants.MK_EVENT_PROCESS:
          var command = new StringBuf();
          for (i in 0...MotionKitNativeConstants.MK_EVENT_COMMAND_BYTES) {
            var code = nativeValue.get_command(i);
            if (code == 0) break;
            command.addChar(code);
          }
          EventValue.Process(command.toString(), nativeValue.get_argument());
        default: throw "Unknown native event value";
      };
      var hold = switch nativeEvent.get_hold_policy() {
        case MotionKitNativeConstants.MK_EVENT_SAFE_WHILE_HELD: HoldPolicy.SafeWhileHeld;
        case MotionKitNativeConstants.MK_EVENT_RESTORE_ON_RESUME: HoldPolicy.RestoreOnResume;
        default: HoldPolicy.Keep;
      };
      storedEvents.push(new TimedEvent(nativeEvent.get_time_ns(), channel.toString(), value, hold));
    }
  }

  function get_events():Array<TimedEvent> return storedEvents.copy();

  /** Copies the native-validated payload so callers cannot change this plan. */
  public function segments():Array<{timeFromStartNs:Int64, durationNs:Int64,
      coefficients:Array<Array<Float>>}> {
    return [for (segment in storedSegments) {
      timeFromStartNs: segment.timeFromStartNs,
      durationNs: segment.durationNs,
      coefficients: [for (joint in segment.coefficients) joint.copy()]
    }];
  }

  public function copyStartPositions():Array<Float> return storedStartPositions.copy();
  public function copyStartVelocities():Array<Float> return storedStartVelocities.copy();
  public function copyStartAccelerations():Array<Float> return storedStartAccelerations.copy();
  public function copyPositionTolerances():Array<Float> return storedPositionTolerances.copy();
  public function copyVelocityTolerances():Array<Float> return storedVelocityTolerances.copy();
  public function copyAccelerationTolerances():Array<Float> return storedAccelerationTolerances.copy();

  /** Does not infer derivatives from degree-1 chords; callers supply the authored state. */
  public static function create(trajectory:Trajectory, limits:ValidationLimits, planId:Int64,
      positions:Array<Float>, velocities:Array<Float>, accelerations:Array<Float>,
      positionTolerances:Array<Float>, velocityTolerances:Array<Float>,
      accelerationTolerances:Array<Float>, ?events:Array<TimedEvent>):ExecutionPlan {
    var count = trajectory.jointCount();
    if (count != limits.jointCount || positions.length != count || velocities.length != count ||
        accelerations.length != count || positionTolerances.length != count ||
        velocityTolerances.length != count || accelerationTolerances.length != count)
      throw "Plan start-state joint count mismatch";
    var start = new mk_start_state();
    start.set_struct_size(mk_start_state.size());
    start.set_joint_count(count);
    for (joint in 0...count) {
      start.set_position(joint, positions[joint]);
      start.set_velocity(joint, velocities[joint]);
      start.set_acceleration(joint, accelerations[joint]);
      start.set_position_tolerance(joint, positionTolerances[joint]);
      start.set_velocity_tolerance(joint, velocityTolerances[joint]);
      start.set_acceleration_tolerance(joint, accelerationTolerances[joint]);
    }
    var spec = new mk_plan_spec();
    spec.set_struct_size(mk_plan_spec.size());
    spec.set_plan_id(planId);
    spec.set_model_revision(limits.modelRevision);
    spec.set_calibration_revision(limits.calibrationRevision);
    spec.set_required_capabilities(Int64.ofInt(MotionKitNativeConstants.MK_CAP_TIMED_TRAJECTORY));
    var authored = events == null ? [] : events;
    if (authored.length > MotionKitNativeConstants.MK_MAX_PLAN_EVENTS)
      throw "Too many timed events in plan";
    if (authored.length > 0)
      spec.set_required_capabilities(Int64.ofInt(
        MotionKitNativeConstants.MK_CAP_TIMED_TRAJECTORY | MotionKitNativeConstants.MK_CAP_EVENTS));
    spec.set_event_count(authored.length);
    var previous = Int64.ofInt(0);
    for (index in 0...authored.length) {
      var event = authored[index];
      if (event == null || Int64.compare(event.timeNs, previous) < 0)
        throw "Timed events must be sorted";
      previous = event.timeNs;
      var nativeEvent = new mk_timed_event();
      writeAscii(event.channel, MotionKitNativeConstants.MK_EVENT_CHANNEL_BYTES,
        function(i, code) nativeEvent.set_channel(i, code));
      nativeEvent.set_time_ns(event.timeNs);
      nativeEvent.set_hold_policy(switch event.holdPolicy {
        case Keep: MotionKitNativeConstants.MK_EVENT_KEEP;
        case SafeWhileHeld: MotionKitNativeConstants.MK_EVENT_SAFE_WHILE_HELD;
        case RestoreOnResume: MotionKitNativeConstants.MK_EVENT_RESTORE_ON_RESUME;
      });
      var nativeValue = new mk_event_value();
      switch event.value {
        case Digital(enabled):
          nativeValue.set_kind(MotionKitNativeConstants.MK_EVENT_DIGITAL);
          nativeValue.set_digital(enabled ? 1 : 0);
        case Analog(number):
          nativeValue.set_kind(MotionKitNativeConstants.MK_EVENT_ANALOG);
          nativeValue.set_analog(number);
        case Process(command, argument):
          nativeValue.set_kind(MotionKitNativeConstants.MK_EVENT_PROCESS);
          writeAscii(command, MotionKitNativeConstants.MK_EVENT_COMMAND_BYTES,
            function(i, code) nativeValue.set_command(i, code));
          nativeValue.set_argument(argument);
      }
      nativeEvent.set_value(nativeValue);
      spec.set_events(index, nativeEvent);
    }
    spec.set_planning_authority(MotionKitNativeConstants.MK_AUTHORITY_MATERIA);
    spec.set_start_state(start);
    var nativeReport = new mk_validation_report();
    nativeReport.set_struct_size(mk_validation_report.size());
    var created = MotionKitNative.mk_plan_create(trajectory.owner.borrow(), spec,
      limits.nativeValue(), nativeReport);
    if (created.status == MotionKitNativeConstants.MK_ERROR_LIMIT)
      throw new PlanLimitError(new ValidationReport(nativeReport));
    check(created.status, "plan.create");
    return new ExecutionPlan(created.out_plan, new ValidationReport(nativeReport),
      trajectory, positions, velocities, accelerations, positionTolerances,
      velocityTolerances, accelerationTolerances);
  }

  public function evaluate(timeSeconds:Float):TrajectoryState {
    if (disposed) throw "Native plan has been disposed";
    var state = new mk_trajectory_state();
    state.set_struct_size(mk_trajectory_state.size());
    check(MotionKitNative.mk_plan_evaluate(owner.borrow(),
      Trajectory.nanoseconds(timeSeconds), state), "plan.evaluate");
    var positions:Array<Float> = [];
    var velocities:Array<Float> = [];
    var accelerations:Array<Float> = [];
    var jerks:Array<Float> = [];
    for (joint in 0...state.get_joint_count()) {
      positions.push(state.get_position(joint));
      velocities.push(state.get_velocity(joint));
      accelerations.push(state.get_acceleration(joint));
      jerks.push(state.get_jerk(joint));
    }
    return new TrajectoryState(positions, velocities, accelerations, jerks);
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  static function check(status:Int, operation:String):Void {
    if (status != MotionKitNativeConstants.MK_OK)
      throw '$operation failed with MotionKit error $status';
  }

  static function writeAscii(value:String, capacity:Int, write:Int -> Int -> Void):Void {
    if (value == null || value.length == 0 || value.length >= capacity)
      throw "Event text does not fit native ABI";
    for (i in 0...value.length) {
      var code = value.charCodeAt(i);
      if (code <= ' '.code || code > '~'.code) throw "Native events need printable ASCII text";
      write(i, code);
    }
  }
}
