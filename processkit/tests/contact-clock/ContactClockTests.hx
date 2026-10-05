import haxe.Int64;
import processkit.perception.ContactSearch;
import robotkit.core.SensorFrame;
import robotkit.spatial.Vec3;
import robotkit.time.ClockMapping;
import robotkit.time.ClockMappings;

class ContactClockTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function search():ContactSearch return new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.0005);
  static function frame(sequence:Int, time:Int, clock:String, touch:Bool = false):SensorFrame
    return new SensorFrame("torch", "tool_weld", "tcp", Int64.ofInt(sequence), Int64.ofInt(time),
      [0.0, 0.0, 24.0, touch ? 1.0 : 0.0, 0.0, 0.0], null, "", null, null, clock);
  static function mappings(bound:Int, until:Int = 100000000):ClockMappings {
    var result = new ClockMappings();
    result.add(new ClockMapping("board.0", "joint.0", Int64.ofInt(-50000000), 100000.0,
      Int64.ofInt(bound), Int64.ofInt(0), "test calibration", 10.0, Int64.ofInt(until)));
    return result;
  }
  public static function main():Void {
    ContactSearchTests.run();
    var s = search();
    s.observe(Int64.ofInt(1000000), "joint.0", new Vec3(), frame(1, 50000000, "board.0"), mappings(10000));
    check(s.running() && s.velocity().z < 0, "Bounded offset/drift mapping authorizes fresh arc-off feedback");
    s.observe(Int64.ofInt(11000000), "joint.0", new Vec3(0, 0, -0.000005), frame(2, 60000000, "board.0", true), mappings(10000));
    check(s.contact != null && s.failure == null, "Measured contact uses a mapped timestamp without changing the raw frame");
    s = search(); s.observe(Int64.ofInt(1000000), "joint.0", new Vec3(), frame(1, 50000000, "board.0"));
    check(s.failure != null, "Different clocks without an explicit relationship fail");
    s = search(); s.observe(Int64.ofInt(21000000), "joint.0", new Vec3(), frame(1, 60000000, "board.0"), mappings(11000000));
    check(s.failure != null, "Mapping uncertainty that exceeds the age/position budget fails");
    s = search(); s.observe(Int64.ofInt(1000000), "joint.0", new Vec3(), frame(1, 50000000, "board.0"), mappings(2000000));
    check(s.failure != null, "A timestamp interval extending beyond measured FK fails");
    s = search(); s.observe(Int64.ofInt(11000000), "joint.0", new Vec3(), frame(1, 60000000, "board.0"), mappings(0, 5000000));
    check(s.failure != null, "Expired mappings cannot authorize a contact");
    s = search(); s.observe(Int64.ofInt(1000000), "joint.0", new Vec3(), frame(1, 1000000, "joint.0"));
    s.observe(Int64.ofInt(2000000), "joint.1", new Vec3(), frame(2, 2000000, "joint.1", true));
    check(s.failure != null && s.contact == null, "Joint epoch changes abort an active probe");
    s = search(); s.observe(Int64.ofInt(1000000), "joint.0", new Vec3(), frame(1, 50000000, "board.0"), mappings(10000));
    s.observe(Int64.ofInt(11000000), "joint.0", new Vec3(), frame(2, 60000000, "board.1", true), mappings(10000));
    check(s.failure != null && s.contact == null, "An earlier epoch's calibration cannot authorize a reset device");
    s = search(); s.observe(Int64.ofInt(1000000), "unspecified", new Vec3(), frame(1, 1000000, "unspecified"));
    check(s.failure != null, "Matching unknown labels do not establish a shared clock");
    s = search(); s.observe(Int64.ofInt(1), "joint.0", new Vec3(), frame(1, -1, "joint.0"));
    check(s.failure != null, "Negative sensor time is rejected even on a matching clock");
    Sys.println('Contact clocks: $checks assertions passed');
  }
}
