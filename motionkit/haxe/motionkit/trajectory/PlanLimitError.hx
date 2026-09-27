package motionkit.trajectory;

/** A rejected plan still carries the validation report. */
class PlanLimitError {
  public final report:ValidationReport;

  public function new(report:ValidationReport) {
    this.report = report;
  }

  public function toString():String return "Execution plan exceeds claimed limits";
}
