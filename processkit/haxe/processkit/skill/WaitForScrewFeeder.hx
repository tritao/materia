package processkit.skill;

import processkit.feeding.ScrewFeederDevice;
import robotkit.skill.*;
import robotkit.core.RobotSnapshot;
import haxe.Int64;

/** Wait for actual receipt before the robot starts its unchanged pickup skill. */
class WaitForScrewFeeder implements Skill {
  final feeder:ScrewFeederDevice;
  final timeout:Float;
  final lifecycle = new SkillLifecycle();
  var elapsed = 0.0;
  var silent = 0.0;
  var previous:Null<Int64> = null;
  var requested = false;
  public function new(feeder:ScrewFeederDevice, timeout:Float) {
    if (feeder == null || !Math.isFinite(timeout) || timeout <= 0) throw "Feeder wait needs a device and explicit timeout";
    this.feeder = feeder; this.timeout = timeout;
  }
  public function start():Void {
    lifecycle.begin(); elapsed = 0; silent = 0; previous = null; requested = false;
    try feeder.enable(true) catch (error:Dynamic) fail(Std.string(error));
  }
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try {
      if (!Math.isFinite(durationSeconds) || durationSeconds <= 0) throw "Feeder wait needs positive time";
      elapsed += durationSeconds;
      var fault = feeder.faultDetail(); if (fault != null) throw fault;
      var value = feeder.reading();
      if (value == null) silent += durationSeconds;
      else {
        var prior = previous;
        if (prior != null && Int64.compare(value.sequence,prior) < 0) throw "Feeder source sequence regressed";
        if (prior == null || Int64.compare(value.sequence,prior) > 0) {silent = 0; previous = value.sequence;}
        else silent += durationSeconds;
        if (value.jammed == true) throw "Feeder jam prevents delivery";
        if (value.present) lifecycle.succeed("screw physically present at pickup nest");
        else if (!requested && value.empty == true) throw "Feeder supply is empty";
        else if (!requested && value.ready) {feeder.request(); requested = true;}
      }
      if (silent > timeout) throw "Feeder wait has no fresh acquisition";
      if (elapsed > timeout && lifecycle.isRunning()) throw "Feeder receipt timed out";
    } catch (error:Dynamic) fail(Std.string(error));
    return lifecycle.status();
  }
  public function cancel():Void {if (lifecycle.isRunning()) {feeder.safe(); lifecycle.cancel();}}
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
  function fail(message:String):Void {try feeder.safe() catch (_:Dynamic) {} lifecycle.fail(message);}
}
