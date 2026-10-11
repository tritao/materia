package processkit.skill;

import haxe.Int64;
import robotkit.tool.ScrewdriverContract;
import robotkit.skill.Skill;
import robotkit.skill.SkillLifecycle;
import robotkit.skill.SkillStatus;
import robotkit.skill.SkillResult;
import robotkit.core.RobotSnapshot;
import robotkit.localization.Localization;
import robotkit.spatial.Transform3;
import processkit.screwdriving.ScrewPlan;
import processkit.screwdriving.ScrewdrivingRunner;
import processkit.screwdriving.ScrewdriverSensor;

/** Drives one CAD fastening, reading the same tool sensor in simulation and on a controller. */
class DriveScrew implements Skill {
  public final runner:ScrewdrivingRunner;
  final localization:Localization;
  final plan:Void -> ScrewPlan;
  final sensor:String;
  final lifecycle = new SkillLifecycle();
  var silent = 0.0;
  var lastSequence:Null<Int64> = null;

  public function new(runner:ScrewdrivingRunner, localization:Localization, plan:Void -> ScrewPlan, sensor:String) {
    if (runner == null || localization == null || plan == null || sensor == null || sensor.length == 0)
      throw "DriveScrew needs a runner, localization, fastening plan and screwdriver sensor";
    this.runner = runner; this.localization = localization; this.plan = plan; this.sensor = sensor;
  }
  public function start():Void {
    lifecycle.begin(); silent = 0; lastSequence = null;
    try {
      var state = localization.state();
      if (state == null) throw "DriveScrew has no robot pose";
      runner.run(plan().transformed(Transform3.fromPose2(state.pose).inverse()));
    } catch (error:Dynamic) fail(Std.string(error));
  }
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try {
      var values:Null<Array<Float>> = null;
      for (index in 0...snapshot.sensors.length) {
        var frame = snapshot.sensors.get(index);
        if (frame.sensorId == sensor) {
          if (frame.kind != ScrewdriverContract.SENSOR_KIND) throw 'Sensor "$sensor" is not a screwdriver';
          var previous = lastSequence;
          if (previous != null && Int64.compare(frame.sequence,previous) < 0)
            throw 'Screwdriver sensor "$sensor" sequence regressed';
          if (previous == null || Int64.compare(frame.sequence,previous) > 0) {
            values = frame.values.toArray(); lastSequence = frame.sequence;
          }
        }
      }
      if (values == null) {
        silent += durationSeconds;
        if (silent > 2) throw 'Screwdriver sensor "$sensor" has not reported a fresh frame';
      } else {
        silent = 0;
        runner.update(durationSeconds,ScrewdriverSensor.reading(values));
        var failure = runner.failure();
        if (failure != null) fail(failure);
        else if (runner.completed()) lifecycle.succeed("screw tightened and tool retracted");
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
    try { if (runner.running()) runner.abort(); } catch (_:Dynamic) {}
    lifecycle.fail(message);
  }
}
