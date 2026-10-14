package processkit.skill;

import processkit.ContactRegistrationRunner;
import robotkit.skill.Skill;
import robotkit.skill.SkillLifecycle;
import robotkit.skill.SkillStatus;
import robotkit.skill.SkillResult;
import robotkit.core.RobotSnapshot;

/** Acquire an accepted work frame through executed, source-identified contact probes. */
class FindContactWork implements Skill {
  final make:Void->ContactRegistrationRunner;
  final accept:ContactRegistrationRunner->Void;
  final lifecycle = new SkillLifecycle();
  var runner:Null<ContactRegistrationRunner> = null;
  public function new(make:Void->ContactRegistrationRunner, accept:ContactRegistrationRunner->Void) {
    if (make == null || accept == null) throw "Finding work needs an executed registration factory and evidence consumer";
    this.make = make; this.accept = accept;
  }
  public function start():Void {
    lifecycle.begin();
    try { runner = make(); cast(runner, ContactRegistrationRunner).start(); }
    catch (error:Dynamic) lifecycle.fail(Std.string(error));
  }
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try {
      var active:ContactRegistrationRunner = cast runner;
      active.update(durationSeconds);
      if (active.failure != null) lifecycle.fail(cast active.failure);
      else if (active.completed()) {
        accept(active);
        lifecycle.succeed("work frame measured from contacts");
      }
    } catch (error:Dynamic) {
      var active = runner; if (active != null) active.cancel();
      lifecycle.fail(Std.string(error));
    }
    return lifecycle.status();
  }
  public function cancel():Void {
    var active = runner; if (active != null) active.cancel();
    lifecycle.cancel();
  }
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}
