import haxe.Int64;
import processkit.ContactSearchRunner;
import processkit.tool.WeldSensor;
import processkit.tool.WeldArcModel;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;
import robotkit.manipulation.Manipulator;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
import robotkit.runtime.SimulationHarness;
import robotkit.simulation.SimulatedRobot;
import robotkit.spatial.Vec3;
import motionkit.robot.ServoSession;

class ContactSearchMotionTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function run(dropFeedback:Bool, noTouch:Bool):Void {
    var model = new RobotModel("contact-search-axis");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var joint = model.addJoint(new Joint("probe", JointType.Prismatic, base, tool));
    joint.axis = [0.0, 0.0, 1.0];
    joint.limits.lower = -0.1; joint.limits.upper = 0.1;
    joint.limits.velocity = 0.1; joint.limits.maxAcceleration = 1.0;
    var flange = model.addFrame(new Frame("tip", tool));
    var arm = new Manipulator(model, base.id, flange.id);
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    blueprint.sensors.push(new RobotRuntimeSensorBlueprint("torch", WeldSensor.KIND, "tip", "tool", 1,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("probe", runtime, model.name, ["base", "tool"], ["probe"]);
    var servo = new ServoSession(robot, arm);
    var runner = new ContactSearchRunner(servo, "torch", new Vec3(0, 0, -1), 0.04, 0.005,
      0.02, 0.002, WeldArcModel.TOUCH_TOLERANCE);
    var sensing = new WeldArcModel({maxCurrentA: 300.0, efficiency: 0.85, wireDiameterMm: 1.2, stickoutMm: 15.0});
    var tick = 0;
    var firstTouch = 0.0;
    var touched = false;
    var fastest = 0.0;
    while (tick < 1200 && !runner.stopped) {
      harness.step(Int64.ofInt(tick));
      tick++;
      var snapshot = robot.snapshot();
      var q = snapshot.positions.get(0);
      fastest = Math.max(fastest, Math.abs(snapshot.velocities.get(0)));
      if (!dropFeedback || tick < 20) {
        var gap = noTouch ? 1.0 : Math.max(0.0, q + 0.02);
        var reading = sensing.step(0.01, {arcCommanded: false, voltageSet: 20.0, wireSpeed: 0.0,
          supplyReady: true, tipDistance: gap, wireDistance: gap});
        if (reading.touch && !touched) { firstTouch = q; touched = true; }
        runtime.publishSensorFrame("torch", WeldSensor.values(reading), Int64.ofInt(tick),
          snapshot.sourceTimestampNs, snapshot.sourceClockId);
      }
      runner.update();
    }
    check(runner.stopped, "Probe completes braking within a bounded number of ticks");
    check(fastest > 0.001 && fastest <= 0.0051, "Probe executes bounded inward native motion");
    if (dropFeedback || noTouch) {
      check(!runner.completed() && runner.search.failure != null && runner.search.contact == null,
        "Missing contact or stale feedback fails without a registration point");
      if (dropFeedback) check(robot.snapshot().positions.get(0) > -0.002, "Stale feedback stops well before the material");
    } else {
      check(runner.completed() && runner.search.failure == null, 'Executed contact succeeds: ${runner.search.failure}');
      var contact:Vec3 = cast runner.search.contact;
      check(Math.abs(contact.z + 0.02) < 0.00006, "Calibrated measured contact recovers the physical plane within one servo period");
      check(Math.abs(contact.z - (firstTouch - WeldArcModel.TOUCH_TOLERANCE)) < 1e-12,
        "Contact is captured at detection rather than at the later braking endpoint");
    }
    for (_ in 0...10) harness.step(Int64.ofInt(tick++));
    check(Math.abs(robot.snapshot().velocities.get(0)) < 1e-6, "The native joint is at rest after probing");
    servo.dispose(); harness.dispose();
  }
  public static function main():Void {
    run(false, false); run(true, false); run(false, true);
    Sys.println('Contact search native motion: $checks assertions passed');
  }
}
