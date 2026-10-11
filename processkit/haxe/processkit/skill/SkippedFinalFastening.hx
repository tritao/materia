package processkit.skill;

import robotkit.skill.*;
import robotkit.core.RobotSnapshot;
import processkit.screwdriving.FasteningRunReport;
import processkit.screwdriving.FasteningRunReport.FasteningPass;

/** Failed snug cannot supply an installed screw; skipping final invents no tightening measurement. */
class SkippedFinalFastening implements FasteningExecution {
  final report:FasteningRunReport;
  final lifecycle = new SkillLifecycle();
  public function new(snug:FasteningRunReport) {
    if (snug == null || snug.pass != SnugPass || !snug.finished || !snug.failed || snug.fault != null || snug.installed() != null)
      throw "Skipping final needs a completed failed snug report without a controller fault";
    report = new FasteningRunReport(snug.fastening,FinalPass);
  }
  public function start():Void {
    lifecycle.begin(); report.reset(); report.skip("Snug failed; no screw is installed for final tightening");
    lifecycle.succeed("final tightening skipped after failed snug");
  }
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus return lifecycle.status();
  public function cancel():Void {if (lifecycle.isRunning()) {report.stop("cancelled"); lifecycle.cancel();}}
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
  public function runReport():FasteningRunReport return report;
}
