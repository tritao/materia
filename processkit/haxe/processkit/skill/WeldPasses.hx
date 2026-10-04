package processkit.skill;

import robotkit.core.RobotSnapshot;
import robotkit.skill.*;

/** Run each pass to completion, with cooling between passes and the arc off during that cooling. */
class WeldPasses implements Skill {
  final passes:Array<Skill>;
  final dwells:Array<Float>;
  final lifecycle = new SkillLifecycle();
  var index:Int = 0;
  var waiting:Bool = false;
  var remaining:Float = 0;

  public function new(passes:Array<Skill>, dwells:Array<Float>) {
    if (passes == null || passes.length == 0 || dwells == null || dwells.length != passes.length)
      throw "WeldPasses needs passes and their interpass dwells";
    for (pass in passes) if (pass == null) throw "WeldPasses has an empty pass";
    for (dwell in dwells) if (!(dwell >= 0 && dwell <= 10)) throw "Invalid interpass dwell";
    this.passes = passes.copy();
    this.dwells = dwells.copy();
  }

  public function start():Void {
    lifecycle.begin();
    index = 0;
    waiting = false;
    remaining = 0;
    beginPass();
  }

  function beginPass():Void {
    try passes[index].start() catch (error:Dynamic) lifecycle.fail(Std.string(error));
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (waiting) {
      remaining -= durationSeconds;
      if (remaining > 0) return lifecycle.status();
      waiting = false;
      beginPass();
      return lifecycle.status();
    }
    try {
      switch passes[index].update(snapshot, durationSeconds) {
        case Succeeded:
          index++;
          if (index == passes.length) lifecycle.succeed('welded ${passes.length} passes');
          else {
            remaining = dwells[index];
            waiting = true;
          }
        case Failed(message): lifecycle.fail('pass ${index + 1}: $message');
        case Cancelled: lifecycle.cancel();
        case _: {}
      }
    } catch (error:Dynamic) {
      try passes[index].cancel() catch (_:Dynamic) {}
      lifecycle.fail(Std.string(error));
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    if (!waiting) passes[index].cancel();
    lifecycle.cancel();
  }
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}
