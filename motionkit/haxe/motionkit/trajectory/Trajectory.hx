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
    if (source.jointCount > MotionKitNativeConstants.MK_MAX_JOINTS)
      throw "Native trajectory exceeds the joint limit";
    var samples:Array<mk_sample> = [];
    var previous = Int64.ofInt(-1);
    for (sample in source.samples) {
      var time = nanoseconds(sample.timeSeconds);
      if (Int64.compare(time, previous) <= 0)
        throw "Native trajectory sample times must increase after nanosecond rounding";
      var value = new mk_sample();
      value.set_struct_size(mk_sample.size());
      value.set_time_ns(time);
      value.set_joint_count(source.jointCount);
      for (joint in 0...source.jointCount)
        value.set_position(joint, sample.positions[joint]);
      samples.push(value);
      previous = time;
    }
    var created = MotionKitNative.mk_trajectory_from_samples(source.jointCount, samples);
    check(created.status, "trajectory.fromSamples");
    return new Trajectory(created.out_trajectory);
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
    return Std.parseFloat(Int64.toStr(result.out_duration_ns)) * 1e-9;
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

  static function nanoseconds(seconds:Float):Int64 {
    if (!Math.isFinite(seconds) || seconds < 0.0 || seconds > 9e9)
      throw "Trajectory time must be finite and non-negative";
    return Int64.fromFloat(Math.floor(seconds * 1e9 + 0.5));
  }

  static function check(status:Int, operation:String):Void {
    if (status != MotionKitNativeConstants.MK_OK)
      throw '$operation failed with MotionKit error $status';
  }

  function ensureLive():Void {
    if (disposed) throw "Native trajectory has been disposed";
  }
}
