package motionkit.trajectory;

/** A rejected plan still carries the validation report. */
class PlanLimitError {
  public final report:ValidationReport;

  public function new(report:ValidationReport) {
    this.report = report;
  }

  /** Names each failed check with the joint, time, value and limit where it failed. */
  public function toString():String {
    // Check order and the failed status (2) follow MotionKit's native MK_CHECK_* constants.
    var names = ["position", "velocity", "acceleration", "jerk", "continuity", "task-space"];
    var failures:Array<String> = [];
    for (index in 0...report.checks.length) {
      var check = report.checks[index];
      if (check.status == 2)
        failures.push('${index < names.length ? names[index] : "check " + index} on joint ${check.joint} at '
          + '${check.timeSeconds} s: ${check.value} against ${check.limit}');
    }
    return failures.length == 0 ? "Execution plan exceeds claimed limits"
      : "Execution plan exceeds claimed limits: " + failures.join("; ");
  }
}
