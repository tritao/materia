package processkit.skill;

import robotkit.core.RobotSnapshot;
import robotkit.skill.*;
import robotkit.localization.Localization;
import robotkit.spatial.Transform3;
import processkit.screwdriving.ScrewdrivingRunner;
import processkit.screwdriving.ScrewdriverFrames;

/** Unwinds the measured failed cycle and releases its actual held body at the declared reject bin. */
class RecoverScrew implements Skill {
  final runner:ScrewdrivingRunner;
  final localization:Localization;
  final reject:Void -> Transform3;
  final frames:ScrewdriverFrames;
  final lifecycle = new SkillLifecycle();
  public function new(runner:ScrewdrivingRunner, localization:Localization, reject:Void -> Transform3, sensor:String) {
    if (runner == null || localization == null || reject == null) throw "Screw recovery needs its failed runner, localization and reject pose";
    this.runner = runner; this.localization = localization; this.reject = reject; frames = new ScrewdriverFrames(sensor);
  }
  public function start():Void {
    lifecycle.begin(); frames.reset();
    try {
      var state = localization.state();
      if (state == null) throw "Screw recovery has no robot pose";
      runner.recover(Transform3.fromPose2(state.pose).inverse().compose(reject()));
    } catch (error:Dynamic) fail(Std.string(error));
  }
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try {
      var reading = frames.read(snapshot,durationSeconds);
      if (reading != null) {
        runner.update(durationSeconds,reading);
        var failure = runner.failure();
        if (failure != null) fail(failure);
        else if (runner.recoveryCompleted()) lifecycle.succeed("failed screw recovered and tool retracted");
      }
    } catch (error:Dynamic) fail(Std.string(error));
    return lifecycle.status();
  }
  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    runner.abort(); lifecycle.cancel();
  }
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
  function fail(message:String):Void {
    try {if (runner.running()) runner.abort();} catch (_:Dynamic) {}
    lifecycle.fail(message);
  }
}
