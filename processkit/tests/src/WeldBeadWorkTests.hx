import processkit.tool.WeldBead;
import processkit.tool.WeldBeadWork;
import processkit.tool.GroundedWork;
import processkit.tool.GroundedWork.WeldBodyPose;
import processkit.tool.WeldArcModel;

class WeldBeadWorkTests {
  static var checks:Int = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 1e-7, '$message: expected $expected, got $actual');

  public static function run():Void {
    var bead = new WeldBead([0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], 1.2, 0.95);
    var frame:WeldBodyPose = {position: [0.0, 0.0, 0.0], rotation: [0.0, 0.0, 0.0, 1.0]};
    var work = new WeldBeadWork(bead, () -> frame);
    var grounded = new GroundedWork().addWork(work);
    check(work.distance(0.005, 0.0025, 0.0025) == Math.POSITIVE_INFINITY, "bare seam adds no grounded metal");
    var area = 0.005 * 0.005 / 2;
    var rate = 8.0 / 60 * bead.wireArea * bead.depositionEfficiency;
    for (station in 0...bead.count) bead.step(area * bead.binLength(station) / rate, true, 8,
      bead.pointAt(bead.stationAt(station) + bead.binLength(station) / 2));
    near(bead.leg(5), 0.005, "actual wire volume makes a 5 mm root");
    check(work.vertices(5).length == 18, "station is a six-vertex prism");
    near(grounded.distance(0.0055, 0.0025, 0.0025), 0, "later tip touches prior exposed face");
    near(grounded.ray(0.0055, 0.004, 0.004, 0, -Math.sqrt(0.5), -Math.sqrt(0.5), 0.01),
      0.003 / Math.sqrt(2), "wire first meets the bead face");
    check(grounded.distance(0.0055, 0.001, 0.001) < 0, "inside deposited metal is signed");
    var model = new WeldArcModel({maxCurrentA: 250.0, efficiency: 0.85, wireDiameterMm: 1.2, stickoutMm: 15.0});
    var input:processkit.tool.WeldArcModel.WeldArcInput = {arcCommanded: false, wireSpeed: 8.0, voltageSet: 24.0, supplyReady: true,
      tipDistance: grounded.distance(0.0055, 0.0025, 0.0025),
      wireDistance: grounded.ray(0.0055, 0.0025, 0.0025, 0, -Math.sqrt(0.5), -Math.sqrt(0.5), 0.005)};
    check(model.step(0.0, input).touch, "touch senses the root with the arc off");
    input.arcCommanded = true;
    var reading = model.step(0.1, input);
    check(reading.arc && !reading.touch, 'later pass strikes on the root without any base hull: arc=${reading.arc}, touch=${reading.touch}, fault=${reading.fault}');
    frame.position = [0.012, -0.02, 0.03];
    frame.rotation = [0.0, 0.0, Math.sqrt(0.5), Math.sqrt(0.5)];
    near(work.distance(0.0095, -0.0145, 0.0325), 0, "bead follows translated and rotated workpiece");
    near(work.ray(0.008, -0.0145, 0.034, Math.sqrt(0.5), 0, -Math.sqrt(0.5), 0.01),
      0.003 / Math.sqrt(2), "ray follows workpiece rotation");
    bead.reset();
    check(work.distance(0.0095, -0.0145, 0.0325) == Math.POSITIVE_INFINITY, "reset clears cached geometry");
    check(work.vertices(5).length == 0, "reset clears clearance vertices too");
    Sys.println('Weld bead work tests passed: $checks assertions');
  }
}
