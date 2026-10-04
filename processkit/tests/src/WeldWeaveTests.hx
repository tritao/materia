import motionkit.path.PosePath;
import motionkit.path.WeavePattern;
import motionkit.path.WeaveProfile;
import processkit.WeldingPlanRunner;
import processkit.skill.WeldPlan;
import processkit.skill.WeldPlan.WeldParameters;
import processkit.skill.WeldPlan.WeldSegment;
import processkit.tool.WeldBead;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

class WeldWeaveTests {
  static var assertions:Int = 0;
  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
    assertions++;
  }
  static function parameters(speed:Float):WeldParameters return {
    wireSpeed: 8.0, voltage: 24.0, travelSpeed: speed, approach: 0.04,
    startDwell: 0.15, craterDwell: 0.15, burnback: 0.1,
    weave: new WeaveProfile(WeavePattern.Sine, 0.002, 50.0, speed)
  };
  static function path(rotation:Quat, speed:Float):PosePath {
    var segment = new WeldSegment(new Transform3(new Vec3(0.0, 0.0, 0.0), rotation),
      new Transform3(new Vec3(0.18, 0.0, 0.0), rotation), "fillet", new Vec3(0.0, 0.0, 1.0));
    return new PosePath(WeldingPlanRunner.FRAME, WeldingPlanRunner.pathOf(new WeldPlan([segment], parameters(speed)),
      speed, {angularSpeed: 3.0, angularAcceleration: 2.0}, [0]));
  }
  public static function run():Void {
    var a = path(Quat.identity(), 0.01);
    var b = path(Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI / 2), 0.01);
    check(Math.abs(a.length() - 0.18) < 1e-12, "woven travel remains seam progress");
    var maximum = 0.0;
    for (i in 0...101) {
      var s = 0.18 * i / 100;
      var p = a.poseAt(s), q = b.poseAt(s);
      maximum = Math.max(maximum, Math.abs(p.y));
      check(Math.abs(p.x - q.x) < 1e-12 && Math.abs(p.y - q.y) < 1e-12 && Math.abs(p.z - q.z) < 1e-12,
        "weave direction is independent of the torch roll");
    }
    check(maximum > 0.0019, "the native preflight path contains the authored weave");
    var area = 0.007 * 0.007 / 2.0;
    var speed = (8.0 / 60.0) * Math.PI * 0.0012 * 0.0012 / 4.0 * 0.95 / area;
    var woven = path(Quat.identity(), speed);
    var bead = new WeldBead([0.0, 0.0, 0.0], [0.18, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], 1.2, 0.95);
    var steps = Std.int(Math.ceil(woven.length() / (speed * 0.002)));
    var dt = woven.length() / speed / steps;
    for (i in 0...steps) {
      var p = woven.poseAt(woven.length() * (i + 0.5) / steps);
      bead.step(dt, true, 8.0, [p.x, p.y, p.z]);
    }
    check(Math.abs(bead.meanLeg(0.15, 0.85) - 0.007) < 0.0005,
      "a woven pass deposits a measured 7 mm leg at the seam-progress feed");
    check(bead.gaps() == 0 && bead.stray == 0.0, "woven positions deposit into the seam stations without gaps or stray metal");
    check(Math.abs(bead.deposited - area * 0.18) < 1e-12, "lateral motion does not increase metal per seam metre");
    Sys.println('ProcessKit weld weave tests passed ($assertions assertions)');
  }
}
