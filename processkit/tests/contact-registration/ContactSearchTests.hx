import haxe.Int64;
import processkit.perception.ContactSearch;
import processkit.ContactSearchRunner;
import processkit.tool.WeldSensor;
import robotkit.core.SensorFrame;
import robotkit.spatial.Vec3;

class ContactSearchTests {
  static var checks:Int = 0;
  static function check(ok:Bool, reason:String):Void { checks++; if (!ok) throw reason; }
  static function frame(seq:Int, time:Int, touch:Bool = false, arc:Bool = false, fault:Int = 0, clock:String = "robot"):SensorFrame
    return new SensorFrame("torch", WeldSensor.KIND, "tcp", Int64.ofInt(seq), Int64.ofInt(time),
      [arc ? 1.0 : 0.0, 0.0, touch ? 0.0 : 24.0, touch ? 1.0 : 0.0, fault, 0.0], null, "", null, null, clock);
  static function contactFrame(seq:Int, time:Int, touch:Bool, clock:String = "robot"):SensorFrame
    return new SensorFrame("screwdriver/tip-contact", "tool_contact", "arm/tool/tcp", Int64.ofInt(seq), Int64.ofInt(time),
      [touch ? 1.0 : 0.0], null, "", null, null, clock);
  static function search():ContactSearch return new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.01, 0.02, 0.002, 0.0005);
  public static function run():Int {
    var s = search();
    check(s.velocity().norm() == 0, "No motion before fresh feedback");
    s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1));
    check(s.running() && s.velocity().z == -0.01, "Fresh arc-off feedback permits inward search");
    s.observe(Int64.ofInt(10000001), "robot", new Vec3(0, 0, -0.001), frame(1, 1));
    check(s.contact == null && s.running(), "A repeated fresh noncontact observation does not invent contact");
    s.observe(Int64.ofInt(20000001), "robot", new Vec3(0, 0, -0.002), frame(2, 20000001, true));
    var point:Vec3 = cast s.contact;
    check(point != null && Math.abs(point.z + 0.0025) < 1e-12, "Measured contact applies explicit calibrated threshold offset");
    check(s.measurement != null && s.measurement.sensorId == "torch" && s.measurement.sensorKind == WeldSensor.KIND &&
      s.measurement.unit == "m" && s.measurement.coordinateFrame == "root" && s.measurement.sequence == Int64.ofInt(2),
      "Touch registration retains sensor identity, units, coordinate frame and acquisition sequence");
    check(!s.running() && s.velocity().norm() == 0, "Contact stops the commanded search");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1, true));
    check(s.failure != null && s.contact == null, "Existing touch cannot be mistaken for a new surface");
    s = search(); s.observe(Int64.ofInt(30000000), "robot", new Vec3(), frame(1, 1, true));
    check(s.failure != null, "Stale touch cannot authorize registration");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), null);
    check(s.failure != null, "Missing feedback stops probing");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1, false, true));
    check(s.failure != null, "Established arc stops probing");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1, false, false, 2));
    check(s.failure != null, "Welder faults stop probing");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1, false, false, 0, "other"));
    check(s.failure != null, "Unmapped sensor clocks are rejected");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 2));
    check(s.failure != null, "Future feedback is rejected");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1));
    s.observe(Int64.ofInt(10000001), "robot", new Vec3(0, 0, -0.1), frame(2, 10000001));
    check(s.failure != null && s.velocity().norm() == 0, "No touch at the distance limit stops motion");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1));
    s.observe(Int64.ofInt(10000001), "robot", new Vec3(0.003, 0, -0.001), frame(2, 10000001, true));
    check(s.failure != null && s.contact == null, "Off-corridor contact is rejected");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(2, 1));
    s.observe(Int64.ofInt(10000001), "robot", new Vec3(), frame(1, 10000001, true));
    check(s.failure != null, "Regressing sequence cannot create a contact");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1));
    s.observe(Int64.ofInt(10000001), "robot", new Vec3(), frame(2, 1, true));
    check(s.failure != null, "Sequence advance needs timestamp advance");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1));
    s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(2, 1, true));
    check(s.failure != null, "Repeated joint-clock ticks cannot generate observations");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1)); s.cancel();
    check(s.failure != null && s.velocity().norm() == 0, "Cancellation removes the search command");
    s = search(); s.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1));
    var late = Int64.fromFloat(13e9);
    var lateFrame = new SensorFrame("torch", WeldSensor.KIND, "tcp", Int64.ofInt(2), late,
      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], null, "", null, null, "robot");
    s.observe(late, "robot", new Vec3(0, 0, -0.01), lateFrame);
    check(s.failure != null && s.contact == null, "Touch after the bounded search timeout is rejected");
    var digital = new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.01, 0.02, 0.002,
      0.0005, "tool_contact", "arm/base", 0.0001);
    digital.observe(Int64.ofInt(1), "robot", new Vec3(), contactFrame(1, 0, false));
    digital.observe(Int64.ofInt(10000001), "robot", new Vec3(0, 0, -0.001), contactFrame(2, 10000001, true));
    check(digital.failure == null && digital.contact != null && digital.measurement != null &&
      digital.measurement.sensorId == "screwdriver/tip-contact" && digital.measurement.sensorFrame == "arm/tool/tcp" &&
      digital.measurement.coordinateFrame == "arm/base" && digital.measurement.uncertaintyMeters >= 0.0001,
      "Digital tool contact yields a typed metric observation with source time and calibrated uncertainty");
    digital = new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.01, 0.02, 0.002,
      0.0005, "tool_contact", "arm/base", 0.00005);
    digital.observe(Int64.ofInt(1), "robot", new Vec3(), contactFrame(1, 0, false));
    digital.observe(Int64.ofInt(10000001), "robot", new Vec3(0, 0, -0.001), contactFrame(2, 1, true));
    check(digital.measurement != null && Math.abs(digital.measurement.uncertaintyMeters - 0.00015) < 1e-12,
      "Contact uncertainty combines calibration and speed-scaled sensor-to-joint acquisition lag");
    var overshoot = new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.01, 0.02, 0.002,
      0.0, "tool_contact", "arm/base", 0.0, 0.0002);
    overshoot.observe(Int64.ofInt(1), "robot", new Vec3(), contactFrame(1, 0, false));
    overshoot.observe(Int64.ofInt(10000001), "robot", new Vec3(0, 0, -0.1), contactFrame(2, 10000001, false));
    check(overshoot.running() && overshoot.velocity().norm() == 0,
      "Nominal endpoint stops commanded motion while allowing one bounded feedback update");
    overshoot.observe(Int64.ofInt(20000001), "robot", new Vec3(0, 0, -0.10015), contactFrame(3, 20000001, true));
    check(overshoot.failure == null && overshoot.contact != null,
      "Fresh contact is accepted within the separately bounded sensor-age stopping travel");
    overshoot = new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.01, 0.02, 0.002,
      0.0, "tool_contact", "arm/base", 0.0, 0.0002);
    overshoot.observe(Int64.ofInt(1), "robot", new Vec3(), contactFrame(1, 0, false));
    overshoot.observe(Int64.ofInt(10000001), "robot", new Vec3(0, 0, -0.1), contactFrame(2, 10000001, false));
    overshoot.observe(Int64.ofInt(30000001), "robot", new Vec3(0, 0, -0.10015), contactFrame(3, 30000001, false));
    check(overshoot.failure != null && overshoot.contact == null,
      "No-contact feedback fails after the bounded endpoint wait");
    overshoot = new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.01, 0.02, 0.002,
      0.0, "tool_contact", "arm/base", 0.0, 0.0002);
    overshoot.observe(Int64.ofInt(1), "robot", new Vec3(), contactFrame(1, 0, false));
    overshoot.observe(Int64.ofInt(10000001), "robot", new Vec3(0, 0, -0.10021), contactFrame(2, 10000001, true));
    check(overshoot.failure != null && overshoot.contact == null,
      "Contact beyond the sensor-age stopping allowance is rejected");
    var excessiveOvershoot = false;
    try new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.01, 0.02, 0.002,
      0.0, "tool_contact", "arm/base", 0.0, 0.0002001) catch (_:Dynamic) excessiveOvershoot = true;
    check(excessiveOvershoot, "Stopping travel cannot exceed the configured sensor-age bound");
    digital = new ContactSearch(new Vec3(), new Vec3(0, 0, -1), 0.1, 0.01, 0.02, 0.002, 0,
      "tool_contact", "arm/base");
    digital.observe(Int64.ofInt(1), "robot", new Vec3(), frame(1, 1));
    check(digital.failure != null && digital.contact == null, "A different sensor kind cannot authorize tool contact");
    var invalidRunner = false;
    try new ContactSearchRunner(null, "torch", new Vec3(0, 0, -1), 0.1, 0.01) catch (_:Dynamic) invalidRunner = true;
    check(invalidRunner, "Executed probing requires an explicit servo owner");
    Sys.println('Contact search: $checks assertions passed');
    return checks;
  }
}
