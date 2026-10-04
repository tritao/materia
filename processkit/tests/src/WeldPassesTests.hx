import robotkit.core.RobotSnapshot;
import robotkit.skill.*;
import processkit.skill.WeldPasses;

private class PassFixture implements Skill {
  public var starts:Int = 0;
  public var cancels:Int = 0;
  public var updates:Int = 0;
  public var outcome:SkillStatus = Succeeded;
  final lifecycle = new SkillLifecycle();
  public function new() {}
  public function start():Void { starts++; lifecycle.begin(); }
  public function update(snapshot:RobotSnapshot, seconds:Float):SkillStatus { updates++; return outcome; }
  public function cancel():Void { cancels++; lifecycle.cancel(); }
  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}

class WeldPassesTests {
  static var checks:Int = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  static function running(value:SkillStatus):Bool return switch value { case Running: true; case _: false; };
  public static function run():Void {
    var snapshot = new RobotSnapshot("weld", haxe.Int64.ofInt(0), haxe.Int64.ofInt(0), [], [], [], 0, 0);
    var root = new PassFixture(), fill = new PassFixture(), cap = new PassFixture();
    var weld = new WeldPasses([() -> root, () -> { check(root.updates > 0, "fill factory reads the completed root"); return fill; }, () -> cap], [0.0, 0.5, 0.2]);
    weld.start();
    check(root.starts == 1 && fill.starts == 0, "only root starts immediately");
    check(running(weld.update(snapshot, 0.1)), "root completion keeps sequence running");
    weld.update(snapshot, 0.3);
    check(fill.starts == 0 && root.updates == 1, "cooling does not update or start a pass");
    weld.update(snapshot, 0.2);
    check(fill.starts == 1 && fill.updates == 0, "fill starts after cooling");
    weld.update(snapshot, 0.1);
    weld.update(snapshot, 0.2);
    check(cap.starts == 1, "cap follows fill and its dwell");
    check(switch weld.update(snapshot, 0.1) { case Succeeded: true; case _: false; }, "all passes complete");
    check(root.starts == 1 && fill.starts == 1 && cap.starts == 1, "each pass starts once");
    weld.start();
    root.outcome = Running;
    weld.cancel();
    check(root.cancels == 1 && fill.cancels == 0, "cancel stops the active pass");
    root.outcome = Succeeded;
    weld.start();
    weld.update(snapshot, 0.1);
    weld.cancel();
    check(root.cancels == 1, "cancel during cooling has no active pass");
    var failing = new PassFixture(), later = new PassFixture();
    failing.outcome = Failed("arc fault");
    var failed = new WeldPasses([() -> failing, () -> later], [0.0, 0.0]);
    failed.start();
    check(switch failed.update(snapshot, 0.1) { case Failed(message): message == "pass 1: arc fault"; case _: false; }, "failure identifies pass");
    check(later.starts == 0, "failed pass prevents later strikes");
    Sys.println('Weld pass tests passed: $checks assertions');
  }
}
