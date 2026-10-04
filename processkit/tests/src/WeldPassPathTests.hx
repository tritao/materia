import processkit.WeldPassPath;
import processkit.skill.WeldPlan.WeldSegment;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

class WeldPassPathTests {
  static var checks:Int = 0;
  static function check(value:Bool, message:String):Void { checks++; if (!value) throw message; }
  static function point(x:Float, y:Float):Transform3 return new Transform3(new Vec3(x, y, 0), Quat.identity());
  public static function run():Void {
    var corners = [point(0, 0), point(0.04, 0), point(0.04, 0.04), point(0, 0.04)];
    var segments = [for (index in 0...4) new WeldSegment(corners[index], corners[(index + 1) % 4], 'side $index', new Vec3(0, 0, 1))];
    var faces = [for (outward in [new Vec3(0, -1, 0), new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(-1, 0, 0)])
      [new Vec3(0, 0, 1), outward]];
    check(WeldPassPath.offset(segments, faces, 0, 0) == segments, "zero offset preserves exact path");
    var shifted = WeldPassPath.offset(segments, faces, 0.002, 0.003);
    for (index in 0...4) {
      var segment = shifted[index], next = shifted[(index + 1) % 4];
      check(segment.stop.translation.sub(next.start.translation).norm() < 1e-10, "offset loop remains connected");
      check(Math.abs(segment.length() - 0.046) < 1e-10, "outward face offset expands perimeter by derived mitres");
      check(Math.abs(segment.start.translation.z - 0.002) < 1e-10, "first face offset lifts every corner");
      check(segment.name == segments[index].name && segment.open == segments[index].open, "CAD seam metadata is retained");
    }
    var rejected = false;
    try WeldPassPath.offset(segments, faces, 0.002, 0.025) catch (_:Dynamic) rejected = true;
    check(rejected, "offset that consumes a corner is rejected");
    var open = WeldPassPath.offset([segments[0]], [faces[0]], 0.002, 0.003);
    check(open[0].length() == segments[0].length(), "one straight pass retains authored progress length");
    Sys.println('Weld pass path tests passed: $checks assertions');
  }
}
