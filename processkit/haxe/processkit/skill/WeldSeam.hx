package processkit.skill;

import robotkit.skill.*;

import robotkit.localization.Localization;
import robotkit.spatial.Transform3;
import processkit.tool.WeldFault;
import processkit.tool.WeldSensor;
import robotkit.world.RobotSnapshot;

/**
 * Welds one seam with the robot's torch. `seam` gives the plan in the map frame, read when the skill starts so it
 * follows the workpiece wherever it stands; the robot's place comes from `localization`, and its base must stand
 * still meanwhile. The runner brings the torch in, strikes and holds the arc, travels the seam, fills the crater, ends
 * the arc and retracts, restarting with an overlap if the arc is lost on the way; this skill only starts it and
 * reads the outcome on the torch's `tool_weld` sensor, as `HandlePart` does the vacuum's. The weld has succeeded
 * when the runner is done and the welder reports its arc out and no fault.
 */
class WeldSeam implements Skill {
  public final runner:WeldRunner;
  public final sensor:String;
  /** How long the sensor may stay silent before the weld is given up, in seconds. */
  public static inline var SENSOR_TIMEOUT:Float = 2.0;

  final localization:Localization;
  final seam:Void -> WeldPlan;
  final lifecycle = new SkillLifecycle();
  var silent:Float = 0.0;

  public function new(runner:WeldRunner, localization:Localization, seam:Void -> WeldPlan, sensor:String) {
    if (runner == null || localization == null || seam == null || sensor == null || sensor.length == 0)
      throw "WeldSeam needs a runner, localization, a seam and the torch's weld sensor";
    this.runner = runner;
    this.localization = localization;
    this.seam = seam;
    this.sensor = sensor;
  }

  public function start():Void {
    lifecycle.begin();
    silent = 0.0;
    try {
      var state = localization.state();
      if (state == null) throw "WeldSeam has no robot pose";
      var base_T_map = Transform3.fromPose2(state.pose).inverse();
      runner.run(seam().transformed(base_T_map));
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try {
      var values = reported(snapshot);
      if (values == null) {
        silent += durationSeconds;
        if (silent > SENSOR_TIMEOUT) throw 'Weld sensor "$sensor" has not reported';
        return lifecycle.status();
      }
      silent = 0.0;
      var reading = WeldSensor.reading(values);
      runner.update(durationSeconds, reading);
      var failure = runner.failure();
      if (failure != null) fail(failure);
      else if (runner.completed()) {
        if (reading.fault != WeldFault.None) lifecycle.fail(WeldSensor.faultMessage(reading.fault));
        else if (reading.arc) lifecycle.fail("the arc is still burning after the weld");
        else lifecycle.succeed(runner.restarts() == 0 ? "seam welded" : 'seam welded after ${runner.restarts()} restarts');
      }
    } catch (error:Dynamic) {
      fail(Std.string(error));
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    runner.abort();
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  /** The weld sensor's latest values, or null before it has reported. */
  function reported(snapshot:RobotSnapshot):Null<Array<Float>> {
    for (index in 0...snapshot.sensors.length) {
      var frame = snapshot.sensors.get(index);
      if (frame.sensorId == sensor) return [for (i in 0...frame.values.length) frame.values.get(i)];
    }
    return null;
  }

  function fail(message:String):Void {
    try runner.abort() catch (_:Dynamic) {}
    lifecycle.fail(message);
  }
}
