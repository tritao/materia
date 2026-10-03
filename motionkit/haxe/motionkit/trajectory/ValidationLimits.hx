package motionkit.trajectory;

import MotionKitNative;
import haxe.Int64;

/** Explicit claims about one native trajectory. Each derivative claim carries its presence separately from its value. */
class ValidationLimits {
  public final jointCount:Int;
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  final native:mk_limits;

  public function new(jointCount:Int, modelRevision:Int64, calibrationRevision:Int64) {
    if (jointCount < 1 || jointCount > MotionKitNativeConstants.MK_MAX_JOINTS)
      throw "Invalid validation joint count";
    this.jointCount = jointCount;
    this.modelRevision = modelRevision;
    this.calibrationRevision = calibrationRevision;
    native = new mk_limits();
    native.set_struct_size(mk_limits.size());
    native.set_joint_count(jointCount);
    native.set_model_revision(modelRevision);
    native.set_calibration_revision(calibrationRevision);
  }

  public function position(joint:Int, lower:Float, upper:Float):Void {
    validJoint(joint);
    if (!Math.isFinite(lower) || !Math.isFinite(upper) || lower > upper)
      throw "Invalid position bounds";
    native.set_position_claimed(joint, 1);
    native.set_position_lower(joint, lower);
    native.set_position_upper(joint, upper);
  }

  public function velocity(joint:Int, maximum:Float):Void {
    validJoint(joint);
    validMaximum(maximum);
    native.set_max_velocity(joint, maximum);
    native.set_derivative_claimed(joint, native.get_derivative_claimed(joint) | 1);
  }

  public function acceleration(joint:Int, maximum:Float):Void {
    validJoint(joint);
    validMaximum(maximum);
    native.set_max_acceleration(joint, maximum);
    native.set_derivative_claimed(joint, native.get_derivative_claimed(joint) | 2);
  }

  public function jerk(joint:Int, maximum:Float):Void {
    validJoint(joint);
    validMaximum(maximum);
    native.set_max_jerk(joint, maximum);
    native.set_derivative_claimed(joint, native.get_derivative_claimed(joint) | 4);
  }

  public function continuity(order:Int, maximumJump:Float):Void {
    if (order < 0 || order > 2) throw "Continuity order must be 0, 1, or 2";
    validMaximum(maximumJump);
    native.set_max_continuity_jump(order, maximumJump);
  }

  /** Executor clock resolution in nanoseconds; zero uses 1 ns. Round fractional device ticks up. */
  public function timeResolutionNs(resolution:Int64):Void {
    if (Int64.compare(resolution, Int64.ofInt(0)) < 0)
      throw "Validation time resolution must be nonnegative";
    native.set_executor_time_resolution_ns(resolution);
  }

  public function nativeValue():mk_limits return native;

  /** Preserve machine claims while leaving path-timed jerk unclaimed. */
  public function withoutJerk():ValidationLimits {
    var result = new ValidationLimits(jointCount, modelRevision, calibrationRevision);
    for (joint in 0...jointCount) {
      if (native.get_position_claimed(joint) != 0)
        result.position(joint, native.get_position_lower(joint),
          native.get_position_upper(joint));
      if ((native.get_derivative_claimed(joint) & 1) != 0)
        result.velocity(joint, native.get_max_velocity(joint));
      if ((native.get_derivative_claimed(joint) & 2) != 0)
        result.acceleration(joint, native.get_max_acceleration(joint));
    }
    for (order in 0...3)
      result.continuity(order, native.get_max_continuity_jump(order));
    result.timeResolutionNs(native.get_executor_time_resolution_ns());
    return result;
  }

  function validJoint(joint:Int):Void {
    if (joint < 0 || joint >= jointCount) throw "Validation joint out of range";
  }

  static function validMaximum(value:Float):Void {
    if (!Math.isFinite(value) || value < 0.0) throw "Invalid nonnegative limit";
  }
}
