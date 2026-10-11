package processkit.skill;

import robotkit.core.RobotSnapshot;
import robotkit.skill.*;
import robotkit.tool.ScrewdriverContract;
import processkit.screwdriving.ScrewdrivingRunner;
import processkit.screwdriving.FasteningRunReport;

/** The cell binds an attempt's body, program and motion before its skill starts. */
typedef ScrewAttempt = {var screw:String; var skill:Skill; var runner:ScrewdrivingRunner;}

/** Try an identified screw, recover a NOK, then use one identified replacement before reporting and moving on. */
class DriveFastening implements Skill {
  public final report:FasteningRunReport;
  final attempt:Int -> ScrewAttempt;
  final recover:ScrewdrivingRunner -> Skill;
  final lifecycle = new SkillLifecycle();
  var index = 0;
  var recovering = false;
  var active:Null<Skill> = null;
  var runner:Null<ScrewdrivingRunner> = null;
  var screw:String = "";
  public function new(fastening:String, attempt:Int -> ScrewAttempt, recover:ScrewdrivingRunner -> Skill) {
    if (attempt == null || recover == null) throw "Fastening execution needs identified attempts and a recovery skill";
    report = new FasteningRunReport(fastening); this.attempt = attempt; this.recover = recover;
  }
  public function start():Void {
    lifecycle.begin(); report.reset(); index = 0; recovering = false; active = null; runner = null;
    beginAttempt();
  }
  function beginAttempt():Void {
    try {
      var selected = attempt(index);
      if (selected == null || selected.skill == null || selected.runner == null || selected.screw == null || selected.screw.length == 0) throw "Fastening attempt has no skill or runner";
      for (previous in report.attempts) if (previous.screw == selected.screw) throw "Replacement reuses an attempted screw";
      screw = selected.screw; active = selected.skill; runner = selected.runner; recovering = false; selected.skill.start();
    } catch (error:Dynamic) fail(Std.string(error));
  }
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try {
      var skill = active, drive = runner;
      if (skill == null || drive == null) throw "Fastening execution lost its active attempt";
      switch skill.update(snapshot,durationSeconds) {
        case Succeeded:
          if (recovering) {
            report.recovered();
            if (index == 0) {index++; beginAttempt();}
            else {report.finish(); lifecycle.succeed("fastening failed after one replacement");}
          } else {
            var result = drive.tighteningResult;
            if (result == null || result.outcome != ScrewdriverContract.OK) throw "Successful screw skill has no measured OK result";
            report.record(index,screw,result); report.finish(); lifecycle.succeed("fastening tightened");
          }
        case Failed(message):
          if (recovering) fail('recovery: $message');
          else {
            var result = drive.tighteningResult;
            if (result == null) fail(message);
            else {
              report.record(index,screw,result);
              if (result.outcome == ScrewdriverContract.OK) {fail(message); return lifecycle.status();}
              recovering = true;
              var recovery = recover(drive);
              if (recovery == null) throw "Fastening has no recovery skill";
              active = recovery; recovery.start();
            }
          }
        case Cancelled: report.stop("cancelled"); lifecycle.cancel();
        case _: {}
      }
    } catch (error:Dynamic) fail(Std.string(error));
    return lifecycle.status();
  }
  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    var skill = active;
    if (skill != null) skill.cancel();
    report.stop("cancelled"); lifecycle.cancel();
  }
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
  function fail(message:String):Void {
    var skill = active;
    if (skill != null) try skill.cancel() catch (_:Dynamic) {}
    report.stop(message); lifecycle.fail(message);
  }
}
