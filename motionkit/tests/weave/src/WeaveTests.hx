import motionkit.event.EventValue;
import motionkit.event.PathEvent;
import motionkit.kinematics.Pose3;
import motionkit.path.FixedWeaveFrame;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PoseWaypoint;
import motionkit.path.WeaveDirection;
import motionkit.path.WeaveFrame;
import motionkit.path.WeavePath;
import motionkit.path.WeavePattern;
import motionkit.path.WeaveProfile;

class TurningFrame implements WeaveFrame {
  public function new() {}
  public function at(s:Float):WeaveDirection {
    var a = 3.0 * s;
    return new WeaveDirection([0.0, Math.cos(a), Math.sin(a)],
      [0.0, -3.0 * Math.sin(a), 3.0 * Math.cos(a)],
      [0.0, -9.0 * Math.cos(a), -9.0 * Math.sin(a)]);
  }
}

class WeaveTests {
  static var assertions:Int = 0;
  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
    assertions++;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void
    check(Math.abs(a - b) <= tolerance, '$message: $a versus $b');

  static function main():Void {
    var a = new PoseWaypoint(new Pose3(), 0.0005, 0.02);
    var b = new PoseWaypoint(new Pose3(0.1), 0.0005, 0.02);
    var c = new PoseWaypoint(new Pose3(0.2), 0.0005, 0.02);
    var event = new PathEvent(0.123, "wire", EventValue.Analog(8.0));
    var base = new PosePath("work", [new PoseLine(a, b, OrientationPolicy.Fixed, 0.1, 0.01),
      new PoseLine(b, c, OrientationPolicy.Fixed, 0.1, 0.01)], [event]);
    var fixed = new FixedWeaveFrame([0.0, 1.0, 0.0]);
    var zero = WeavePath.apply(base, new WeaveProfile(WeavePattern.Sine, 0.0, 50.0, 0.01), fixed);
    check(zero == base, "zero amplitude preserves the exact base path");
    for (pattern in [WeavePattern.Sine, WeavePattern.Triangle, WeavePattern.Zigzag]) {
      var profile = new WeaveProfile(pattern, 0.002, 50.0, 0.01, 0.05);
      var path = WeavePath.apply(base, profile, new TurningFrame());
      near(path.length(), base.length(), 1e-12, "length means seam progress");
      check(path.events.length == 1 && path.events[0] == event && path.events[0].distance == 0.123,
        "authored events retain their seam distance");
      near(path.poseAt(0.0).y, 0.0, 1e-12, "start offset tapers to zero");
      near(path.poseAt(path.length()).y, 0.0, 1e-12, "end offset tapers to zero");
      for (primitive in path.primitives) {
        var s = primitive.length() * 0.37, h = Math.min(1e-6, primitive.length() / 100.0);
        var left = primitive.waypointAt(s - h).pose.positionArray();
        var middle = primitive.waypointAt(s).pose.positionArray();
        var right = primitive.waypointAt(s + h).pose.positionArray();
        var d = primitive.derivativesAt(s);
        for (axis in 0...3) {
          near(d.linear[axis], (right[axis] - left[axis]) / (2.0 * h), 1e-5,
            "exact weave/frame first derivative agrees with positions");
          near(d.linearSecond[axis], (right[axis] - 2.0 * middle[axis] + left[axis]) / (h * h), 0.02,
            "exact weave/frame second derivative agrees with positions");
        }
      }
    }
    var triangle = new WeaveProfile(WeavePattern.Triangle, 0.002, 50.0, 0.01);
    check(triangle.at(0.01, true).first > 0.0 && triangle.at(0.01).first < 0.0,
      "triangle joins expose their arriving/leaving derivatives");
    var sine = new WeaveProfile(WeavePattern.Sine, 0.002, 50.0, 0.01, 0.1);
    var hold = sine.at(0.0095);
    near(hold.offset, 0.002, 1e-12, "edge hold keeps lateral offset while forward motion continues");
    near(hold.first, 0.0, 1e-12, "edge hold has no lateral velocity");
    var hz = WeaveProfile.perSecond(WeavePattern.Sine, 0.002, 0.5, 0.01, 0.1);
    near(hz.period, sine.period, 1e-12, "per-second frequency converts using nominal seam speed");
    var mm = WeaveProfile.perMillimetre(WeavePattern.Sine, 0.002, 0.05, 0.01, 0.1);
    near(mm.period, sine.period, 1e-12, "per-millimetre frequency converts to SI progress");
    Sys.println('MotionKit weave tests passed ($assertions assertions)');
  }
}
