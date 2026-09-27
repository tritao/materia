package motionkit.trajectory;

import MotionKitNative;
import haxe.Int64;

/** Owns a native piecewise-polynomial joint trajectory. */
@:allow(motionkit.trajectory.ExecutionPlan)
class Trajectory {
  final owner:Ownedmk_trajectory_handle;
  var disposed:Bool = false;

  private function new(owner:Ownedmk_trajectory_handle) {
    this.owner = owner;
  }

  /** Preserves sample times and positions; native derivatives come from chords. */
  public static function fromJointTrajectory(source:JointTrajectory):Trajectory {
    if (source == null || source.samples.length < 2)
      throw "Native trajectory needs at least two samples";
    return fromPositionSamples([for (sample in source.samples) sample.timeSeconds],
      [for (sample in source.samples) sample.positions]);
  }

  /** Builds a degree-1 trajectory directly from authored positions. */
  public static function fromPositionSamples(times:Array<Float>, positions:Array<Array<Float>>):Trajectory {
    if (times == null || positions == null || times.length < 2 ||
        times.length != positions.length || positions[0] == null)
      throw "Native trajectory needs at least two position samples";
    var count = positions[0].length;
    if (count < 1 || count > MotionKitNativeConstants.MK_MAX_JOINTS)
      throw "Native trajectory exceeds the joint limit";
    var samples:Array<mk_sample> = [];
    var previous = Int64.ofInt(-1);
    for (index in 0...times.length) {
      if (positions[index] == null || positions[index].length != count)
        throw "Native trajectory position count changed";
      var time = nanoseconds(times[index]);
      if (Int64.compare(time, previous) <= 0) {
        var duplicate = Int64.compare(time, previous) == 0;
        if (duplicate)
          for (joint in 0...count)
            if (positions[index][joint] != positions[index - 1][joint]) duplicate = false;
        if (duplicate) continue;
        throw "Native trajectory has conflicting or decreasing sample times after nanosecond rounding";
      }
      var value = new mk_sample();
      value.set_struct_size(mk_sample.size());
      value.set_time_ns(time);
      value.set_joint_count(count);
      for (joint in 0...count) {
        if (!Math.isFinite(positions[index][joint])) throw "Non-finite trajectory position";
        value.set_position(joint, positions[index][joint]);
      }
      samples.push(value);
      previous = time;
    }
    if (samples.length < 2) throw "Native trajectory needs two distinct sample times";
    var created = MotionKitNative.mk_trajectory_from_samples(count, samples);
    check(created.status, "trajectory.fromSamples");
    return new Trajectory(created.out_trajectory);
  }

  /** Offline, jerk-limited synchronized state-to-state motion. */
  public static function generateStateToState(currentPosition:Array<Float>,
      currentVelocity:Array<Float>, currentAcceleration:Array<Float>, targetPosition:Array<Float>,
      maximumVelocity:Array<Float>, maximumAcceleration:Array<Float>,
      maximumJerk:Array<Float>):Trajectory {
    if (currentPosition == null || currentPosition.length < 1 ||
        currentPosition.length > MotionKitNativeConstants.MK_MAX_JOINTS)
      throw "Invalid generated trajectory joint count";
    var count = currentPosition.length;
    for (values in [currentVelocity, currentAcceleration, targetPosition,
        maximumVelocity, maximumAcceleration, maximumJerk])
      if (values == null || values.length != count)
        throw "Generated trajectory vector count mismatch";
    var request = new mk_state_to_state_request();
    request.set_struct_size(mk_state_to_state_request.size());
    request.set_joint_count(count);
    request.set_synchronization(MotionKitNativeConstants.MK_SYNCHRONIZATION_TIME);
    request.set_control_mode(MotionKitNativeConstants.MK_CONTROL_POSITION);
    for (joint in 0...count) {
      request.set_current_position(joint, currentPosition[joint]);
      request.set_current_velocity(joint, currentVelocity[joint]);
      request.set_current_acceleration(joint, currentAcceleration[joint]);
      request.set_target_position(joint, targetPosition[joint]);
      request.set_target_velocity(joint, 0.0);
      request.set_target_acceleration(joint, 0.0);
      request.set_max_velocity(joint, maximumVelocity[joint]);
      request.set_max_acceleration(joint, maximumAcceleration[joint]);
      request.set_max_jerk(joint, maximumJerk[joint]);
    }
    var created = MotionKitNative.mk_generate_state_to_state(request);
    check(created.status, 'trajectory.generateStateToState (Ruckig ${created.out_ruckig_result})');
    return new Trajectory(created.out_trajectory);
  }

  /** Copies polynomial coefficients for submission to an executor. */
  public function segments():Array<{timeFromStartNs:Int64, durationNs:Int64,
      coefficients:Array<Array<Float>>}> {
    ensureLive();
    var count = MotionKitNative.mk_trajectory_segment_count(owner.borrow());
    check(count.status, "trajectory.segmentCount");
    var result = [];
    for (index in 0...count.out_segment_count) {
      var native = new mk_segment();
      native.set_struct_size(mk_segment.size());
      check(MotionKitNative.mk_trajectory_get_segment(owner.borrow(), index, native),
        "trajectory.segment");
      var coefficients:Array<Array<Float>> = [];
      for (joint in 0...native.get_joint_count()) {
        var source = native.get_coefficients(joint);
        coefficients.push([for (degree in 0...(native.get_degree() + 1))
          source.get_value(degree)]);
      }
      result.push({timeFromStartNs: native.get_t0_ns(),
        durationNs: native.get_duration_ns(), coefficients: coefficients});
    }
    return result;
  }

  public function evaluate(timeSeconds:Float):TrajectoryState {
    ensureLive();
    var state = new mk_trajectory_state();
    state.set_struct_size(mk_trajectory_state.size());
    check(MotionKitNative.mk_trajectory_evaluate(owner.borrow(), nanoseconds(timeSeconds), state),
      "trajectory.evaluate");
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

  /** Degree-1 uses chord differences; degree 0 and degree >= 2 use analytic derivatives. */
  public function estimatePathDerivatives(timeSeconds:Float,
      windowSeconds:Float):PathDerivativeEstimate {
    ensureLive();
    var estimate = new mk_path_derivative_estimate();
    estimate.set_struct_size(mk_path_derivative_estimate.size());
    check(MotionKitNative.mk_trajectory_estimate_path_derivatives(owner.borrow(),
      nanoseconds(timeSeconds), nanoseconds(windowSeconds), estimate),
      "trajectory.estimatePathDerivatives");
    return new PathDerivativeEstimate(estimate);
  }

  public function durationSeconds():Float {
    ensureLive();
    var result = MotionKitNative.mk_trajectory_duration_ns(owner.borrow());
    check(result.status, "trajectory.duration");
    return Int64.toFloat(result.out_duration_ns) * 1e-9;
  }

  public function jointCount():Int {
    ensureLive();
    var result = MotionKitNative.mk_trajectory_joint_count(owner.borrow());
    check(result.status, "trajectory.jointCount");
    return result.out_joint_count;
  }

  public function validate(limits:ValidationLimits):ValidationReport {
    ensureLive();
    if (limits.jointCount != jointCount()) throw "Validation joint count mismatch";
    var report = new mk_validation_report();
    report.set_struct_size(mk_validation_report.size());
    check(MotionKitNative.mk_validate(owner.borrow(), limits.nativeValue(), report),
      "trajectory.validate");
    return new ValidationReport(report);
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  public static function nanoseconds(seconds:Float):Int64 {
    if (!Math.isFinite(seconds) || seconds < 0.0 || seconds > 2e9)
      throw "Trajectory time must be finite and non-negative";
    // Round only the fractional second as a 32-bit value. Rounding the full
    // nanosecond Float can overflow Std.int at 2.147 s and can misround a
    // decimal time such as 2.21 s by one nanosecond.
    var whole = Std.int(seconds);
    var fraction = Std.int((seconds - whole) * 1e9 + 0.5);
    if (fraction == 1000000000) {
      whole++;
      fraction = 0;
    }
    var digits = Std.string(fraction);
    while (digits.length < 9) digits = "0" + digits;
    return Int64.parseString(Std.string(whole) + digits);
  }

  static function check(status:Int, operation:String):Void {
    if (status != MotionKitNativeConstants.MK_OK)
      throw '$operation failed with MotionKit error $status';
  }

  function ensureLive():Void {
    if (disposed) throw "Native trajectory has been disposed";
  }
}
