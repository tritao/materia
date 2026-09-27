package motionkit.trajectory;

import MotionKitNative;
import haxe.Int64;

/** Snapshot of native extrema and their claimed limits. */
class ValidationReport {
  final native:mk_validation_report;
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  public final trajectoryRevision:Int64;
  public final executorTimeResolutionNs:Int64;
  public final checks:Array<ValidationCheck>;
  public final unresolvedAssumptions:Array<String>;
  public final unresolvedAssumptionCount:Int;

  public function new(native:mk_validation_report) {
    this.native = native;
    modelRevision = native.get_model_revision();
    calibrationRevision = native.get_calibration_revision();
    trajectoryRevision = native.get_trajectory_revision();
    executorTimeResolutionNs = native.get_executor_time_resolution_ns();
    unresolvedAssumptionCount = native.get_assumption_count();
    unresolvedAssumptions = [];
    for (index in 0...unresolvedAssumptionCount) {
      var assumption = native.get_assumptions(index);
      var text = new StringBuf();
      for (letter in 0...MotionKitNativeConstants.MK_ASSUMPTION_LENGTH) {
        var code = assumption.get_text(letter);
        if (code == 0) break;
        text.addChar(code);
      }
      unresolvedAssumptions.push(text.toString());
    }
    checks = [];
    for (index in 0...MotionKitNativeConstants.MK_CHECK_COUNT) {
      var check = native.get_checks(index);
      checks.push(ValidationCheck.fromNative(check));
    }
  }

  /** Records a task-space check sampled by the planner's kinematics layer. */
  public function setTaskSpace(status:Int, worst:Float, timeSeconds:Float,
      tolerance:Float, resolutionNs:Int64):Void {
    var result = MotionKitNative.mk_report_set_task_space(native, status, worst,
      timeSeconds, tolerance, resolutionNs);
    if (result != MotionKitNativeConstants.MK_OK)
      throw 'validationReport.setTaskSpace failed with MotionKit error $result';
    checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE] = ValidationCheck.fromNative(
      native.get_checks(MotionKitNativeConstants.MK_CHECK_TASK_SPACE));
  }

  public function hasFailure():Bool {
    for (check in checks)
      if (check.status == MotionKitNativeConstants.MK_CHECK_FAILED) return true;
    return false;
  }
}

class ValidationCheck {
  public final status:Int;
  public final joint:Int;
  public final derivativeOrder:Int;
  public final method:Int;
  public final value:Float;
  public final timeSeconds:Float;
  public final limit:Float;
  public final margin:Float;
  public final tolerance:Float;
  public final resolutionNs:Int64;

  public function new(status:Int, joint:Int, derivativeOrder:Int, method:Int,
      value:Float, timeSeconds:Float, limit:Float, margin:Float, tolerance:Float,
      resolutionNs:Int64) {
    this.status = status;
    this.joint = joint;
    this.derivativeOrder = derivativeOrder;
    this.method = method;
    this.value = value;
    this.timeSeconds = timeSeconds;
    this.limit = limit;
    this.margin = margin;
    this.tolerance = tolerance;
    this.resolutionNs = resolutionNs;
  }

  public static function fromNative(native:mk_validation_check):ValidationCheck
    return new ValidationCheck(native.get_status(), native.get_joint(),
      native.get_derivative_order(), native.get_method(), native.get_value(),
      native.get_time_seconds(), native.get_limit(), native.get_margin(),
      native.get_tolerance(), native.get_resolution_ns());
}
