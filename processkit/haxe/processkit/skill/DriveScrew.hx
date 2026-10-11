package processkit.skill;

import robotkit.skill.Skill;
import robotkit.skill.SkillLifecycle;
import robotkit.skill.SkillStatus;
import robotkit.skill.SkillResult;
import robotkit.core.RobotSnapshot;
import robotkit.localization.Localization;
import robotkit.spatial.Transform3;
import processkit.screwdriving.ScrewPlan;
import processkit.screwdriving.ScrewdrivingRunner;
import processkit.screwdriving.ScrewdriverFrames;

/** Drives one CAD fastening, reading the same tool sensor in simulation and on a controller. */
class DriveScrew implements Skill {
  public final runner:ScrewdrivingRunner;
  final localization:Localization;
  final plan:Void -> ScrewPlan;
  final lifecycle = new SkillLifecycle();
  final frames:ScrewdriverFrames;
  final prepare:Null<robotkit.skill.Skill>;
  var prepared = false;

  public function new(runner:ScrewdrivingRunner, localization:Localization, plan:Void -> ScrewPlan, sensor:String, ?prepare:robotkit.skill.Skill) {
    if (runner == null || localization == null || plan == null || sensor == null || sensor.length == 0)
      throw "DriveScrew needs a runner, localization, fastening plan and screwdriver sensor";
    this.runner = runner; this.localization = localization; this.plan = plan; this.prepare = prepare; frames = new ScrewdriverFrames(sensor);
  }
  public function start():Void {
    lifecycle.begin(); frames.reset(); prepared = prepare == null;
    try {
      if (prepare != null) {prepare.start(); return;}
      startRunner();
    } catch (error:Dynamic) fail(Std.string(error));
  }
  function startRunner():Void {
    try {
      var state = localization.state();
      if (state == null) throw "DriveScrew has no robot pose";
      runner.run(plan().transformed(Transform3.fromPose2(state.pose).inverse()));
    } catch (error:Dynamic) fail(Std.string(error));
  }
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try {
      if (!prepared && prepare != null) {
        switch prepare.update(snapshot,durationSeconds) {
          case Succeeded: prepared = true; frames.reset(); startRunner();
          case Failed(message): fail(message);
          case Cancelled: lifecycle.cancel();
          case _: {}
        }
        return lifecycle.status();
      }
      var reading = frames.read(snapshot,durationSeconds);
      if (reading != null) {
        runner.update(durationSeconds,reading);
        var failure = runner.failure();
        if (failure != null) fail(failure);
        else if (runner.completed()) lifecycle.succeed("screw tightened and tool retracted");
      }
    } catch (error:Dynamic) fail(Std.string(error));
    return lifecycle.status();
  }
  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    if (!prepared && prepare != null) prepare.cancel();
    if (runner.running()) runner.abort(); lifecycle.cancel();
  }
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
  function fail(message:String):Void {
    try { if (!prepared && prepare != null) prepare.cancel(); } catch (_:Dynamic) {}
    try { if (runner.running()) runner.abort(); } catch (_:Dynamic) {}
    lifecycle.fail(message);
  }
}
