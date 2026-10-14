import motionkit.robot.CartesianAdmittance;
import processkit.screwdriving.ForceGuidedScrewEntry;
import processkit.screwdriving.ScrewEntryController;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;
import robotkit.spatial.Wrench3;

class ScrewEntryTests {
  static var checks = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  public static function run():Void {
    var sensorPose = new Transform3(Vec3.zero(), Quat.identity());
    var tcpPose = new Transform3(new Vec3(0, 0, 0.03), Quat.identity());
    var tcpFromSensor = ForceGuidedScrewEntry.sensorTransformAtTcp(sensorPose, tcpPose);
    var shiftedWrench = tcpFromSensor.transformWrench(new Wrench3(new Vec3(10, 0, 0), Vec3.zero()));
    check(Math.abs(tcpFromSensor.translation.z + 0.03) < 1e-12 &&
      Math.abs(shiftedWrench.torque.y + 0.3) < 1e-12,
      "A sensor wrench is shifted from the mounted sensor origin to the screwdriver TCP");
    var turnedSensor = new Transform3(Vec3.zero(), Quat.fromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2));
    var rotatedWrench = ForceGuidedScrewEntry.sensorTransformAtTcp(turnedSensor, sensorPose)
      .transformWrench(new Wrench3(new Vec3(1, 0, 0), Vec3.zero()));
    check(Math.abs(rotatedWrench.force.x) < 1e-12 && Math.abs(rotatedWrench.force.y - 1) < 1e-12,
      "A sensor wrench is rotated into the screwdriver TCP frame in the correct direction");
    var controller = new CartesianAdmittance(new Wrench3(new Vec3(10, 0, 0), Vec3.zero()),
      [1, 1, 1, 1, 1, 1], [10, 10, 10, 10, 10, 10], 0.05, 0.1, 0.01);
    var first = controller.step(Wrench3.zero(), 0.01);
    check(Math.abs(first.linearX + 0.05) < 1e-12 && first.linearY == 0 && first.angularZ == 0,
      "A force deficit moves against the measured external wrench at a bounded speed");
    var settled = controller.step(new Wrench3(new Vec3(10, 0, 0), Vec3.zero()), 0.01);
    check(Math.abs(settled.linearX + 0.045) < 1e-12,
      "Measured target force damps the prior correction instead of accumulating motion");
    controller.stop();
    var disturbed = controller.step(new Wrench3(new Vec3(-1000, 0, 0), new Vec3(0, 0, 1000)), 0.01);
    check(disturbed.linearX < 0 && Math.abs(disturbed.linearX) <= controller.maxLinearSpeed + 1e-12 &&
      Math.abs(disturbed.angularZ) <= controller.maxAngularSpeed + 1e-12,
      "Large force disturbances remain inside explicit linear and angular speed bounds");
    controller.stop();
    var exactPeriod = controller.step(Wrench3.zero(), 0.01);
    controller.stop();
    var quantizedPeriod = controller.step(Wrench3.zero(), 0.010000001);
    check(Math.abs(exactPeriod.linearX - quantizedPeriod.linearX) < 1e-12,
      "A one-nanosecond clock quantization excess integrates only the commissioned period");
    var rejected = false;
    try controller.step(Wrench3.zero(), 0.010000003) catch (_:Dynamic) rejected = true;
    check(rejected, "An integration period beyond the commissioned bound is rejected");
    rejected = false;
    try new CartesianAdmittance(Wrench3.zero(), [1, 0, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1], 0.1, 0.1, 0.01)
      catch (_:Dynamic) rejected = true;
    check(rejected, "Missing or zero commissioned gains cannot silently enable force control");


    // Controller transitions under scripted measurements: u = x, v = y, advance along z.
    var limits = new ScrewEntryLimits(5.0, 2.0, 2, 0.002, 0.004, 0.0016, 0.0004, 0.002, 10.0, 0.00015, 0.0003, 0.5);
    function entry():ScrewEntryController
      return new ScrewEntryController(limits, new Vec3(1, 0, 0), new Vec3(0, 1, 0), 0.04, 0.00002,
        new CartesianAdmittance(new Wrench3(new Vec3(0, 0, -5), Vec3.zero()), [80, 80, 80, 1, 1, 1],
          [4000, 4000, 4000, 20, 20, 20], 0.004, 0.01, 0.01));
    function pressing(newtons:Float):Wrench3 return new Wrench3(new Vec3(0, 0, -newtons), Vec3.zero());
    var free = entry();
    var descent = free.step(new Vec3(0, 0, 0.01), Wrench3.zero(), 0.01);
    check(free.phase == Approach && Math.abs(descent.linearZ - limits.approachSpeed) < 1e-12 &&
      descent.linearX == 0 && descent.linearY == 0, "Unloaded, the screw descends at the approach speed on its axis");
    var loaded = free.step(new Vec3(0, 0, 0.023), pressing(8), 0.01);
    check(free.phase == Approach && loaded.linearZ < limits.approachSpeed,
      "One loaded sample hands the axis to force regulation before contact is confirmed");
    free.step(new Vec3(0, 0, 0.023), pressing(8), 0.01);
    check(free.phase == Search && free.searches == 1 && free.contactDepth == 0.023,
      "Confirmed contact above the CAD plane searches the touched surface");
    var sweep = free.step(new Vec3(0, 0, 0.023), pressing(5), 0.01);
    check(free.spiralRadius() > 0 && Math.sqrt(sweep.linearX * sweep.linearX + sweep.linearY * sweep.linearY) > 0,
      "The spiral reference advances while the tool tracks it");
    free.step(new Vec3(0, 0, 0.023 + limits.entryAdvance), pressing(1), 0.01);
    check(free.phase == Descend, "Dropping the entry advance below the touched surface descends through the hole");
    var guided = free.step(new Vec3(0.0003, 0, 0.03), new Wrench3(new Vec3(-4, 0, 0), Vec3.zero()), 0.01);
    check(free.phase == Descend && guided.linearX < 0 && Math.abs(guided.linearZ - limits.approachSpeed) < 1e-12,
      "In a bore the wall's push, not a held target, sets the lateral motion");
    free.step(new Vec3(0.0002, 0, 0.0404), Wrench3.zero(), 0.01);
    check(free.phase == Seated && free.failure == null,
      "Reaching the CAD plane plus the entry advance is the tightening handover, touching or not");
    var flat = entry();
    for (_ in 0...2) flat.step(new Vec3(0, 0, 0.04), pressing(6), 0.01);
    check(flat.phase == Search, "Contact on the CAD plane itself is a surface to search, not a seated screw");
    var stalled = entry();
    for (_ in 0...2) stalled.step(new Vec3(0, 0, 0.023), pressing(6), 0.01);
    var steps = 0;
    while (stalled.phase == Search && steps < 1000) { stalled.step(new Vec3(0, 0, 0.023), pressing(6), 0.01); steps++; }
    check(stalled.phase == Failed && stalled.failure.indexOf("no progress") >= 0 && steps < 100,
      'A tool that cannot follow the spiral fails after the stall time: ${stalled.failure}');
    rejected = false;
    try new ScrewEntryController(limits, new Vec3(1, 0, 0), new Vec3(0, 1, 0), 0.04, 0.00002,
      new CartesianAdmittance(new Wrench3(new Vec3(1, 0, -5), Vec3.zero()), [1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1], 0.004, 0.01, 0.01)) catch (_:Dynamic) rejected = true;
    check(rejected, "The admittance may regulate only the axial contact force");
    Sys.println('Screw entry control: $checks assertions passed');
  }
}
