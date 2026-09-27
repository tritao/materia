package motionkit.trajectory;

import MotionKitNative;
import haxe.Int64;

/** Snapshot of native extrema and their claimed limits. */
class ValidationReport {
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  public final trajectoryRevision:Int64;
  public final executorTimeResolutionNs:Int64;
  public final checks:Array<ValidationCheck>;
  public final unresolvedAssumptions:Array<String>;
  public final unresolvedAssumptionCount:Int;

  public function new(native:mk_validation_report) {
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
      checks.push(new ValidationCheck(check.get_status(), check.get_joint(),
        check.get_derivative_order(), check.get_value(), check.get_time_seconds(),
        check.get_limit(), check.get_margin(), check.get_tolerance()));
    }
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
  public final value:Float;
  public final timeSeconds:Float;
  public final limit:Float;
  public final margin:Float;
  public final tolerance:Float;

  public function new(status:Int, joint:Int, derivativeOrder:Int, value:Float,
      timeSeconds:Float, limit:Float, margin:Float, tolerance:Float) {
    this.status = status;
    this.joint = joint;
    this.derivativeOrder = derivativeOrder;
    this.value = value;
    this.timeSeconds = timeSeconds;
    this.limit = limit;
    this.margin = margin;
    this.tolerance = tolerance;
  }
}
