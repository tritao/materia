import haxe.Int64;
import processkit.ContactSearchRunner;
import processkit.ProbeMotionPlanner;
import processkit.ContactProbeRunner;
import processkit.ContactProbeRunner.ContactProbeRequest;
import motionkit.robot.ManipulatorMotion;
import processkit.WeldingPlanRunner;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.manipulation.ArmClearance;
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
  static function axis(flip:Bool = false):{model:RobotModel, arm:Manipulator, base:Link, tool:Link} {
    var model = new RobotModel("contact-search-axis");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var joint = model.addJoint(new Joint("probe", JointType.Prismatic, base, tool));
    joint.axis = [0.0, 0.0, 1.0];
    joint.limits.lower = -0.1; joint.limits.upper = 0.1;
    joint.limits.velocity = 0.1; joint.limits.maxAcceleration = 1.0;
    var flange = model.addFrame(new Frame("tip", tool));
    if (flip) flange.rotation = [1.0, 0.0, 0.0, 0.0];
    var arm = new Manipulator(model, base.id, flange.id);
    return {model: model, arm: arm, base: base, tool: tool};
  }
  static function run(dropFeedback:Bool, noTouch:Bool):Void {
    var fixture = axis();
    var model = fixture.model, arm = fixture.arm, base = fixture.base, tool = fixture.tool;
    if (!dropFeedback && !noTouch) {
      var planning = WeldingPlanRunner.planning(arm, 1.0);
      var probePlanner = new ProbeMotionPlanner(arm, planning.compiler);
      var approach = probePlanner.approach(new Transform3(new Vec3(0, 0, 0.03), Quat.identity()), [0.0], 32);
      check(Math.abs(approach.endJoints[0] - 0.03) < 5e-5, 'Checked approach ends at the requested search pose (${approach.endJoints[0]})');
      var retreat = probePlanner.line(Transform3.identity(), approach.endJoints, 0.01);
      check(Math.abs(retreat.endJoints[0]) < 5e-5, "Checked straight retreat returns along the probe direction");
      var forbidden = false;
      try probePlanner.approach(new Transform3(new Vec3(0, 0, 0.2), Quat.identity()), [0.0], 32) catch (_:Dynamic) forbidden = true;
      check(forbidden, "Probe approach rejects a target outside the mechanical travel");
      function cube(z:Float):Array<Float> return [for (x in [-0.001, 0.001]) for (y in [-0.001, 0.001])
        for (height in [z - 0.001, z + 0.001]) for (value in [x, y, height]) value];
      var blocked = new ArmClearance(arm, [
        {name: "tool", link: tool.id, vertices: cube(0), tool: true},
        {name: "fixture", link: base.id, vertices: cube(0.015), tool: false}], [0.0], 0.003);
      var guarded = new ProbeMotionPlanner(arm, planning.compiler, blocked);
      check(guarded.stoppingClear([0.0], [0.0], 0.02), "A stationary clear probe has a safe braking sweep");
      check(!guarded.stoppingClear([0.008], [0.1], 0.02), "A clear current posture can still have an obstructed braking sweep");
      forbidden = false;
      try guarded.approach(new Transform3(new Vec3(0, 0, 0.03), Quat.identity()), [0.0], 32) catch (_:Dynamic) forbidden = true;
      check(forbidden, "A clear approach endpoint does not authorize travel through a fixture");
      forbidden = false;
      try guarded.line(new Transform3(new Vec3(0, 0, 0.03), Quat.identity()), [0.0], 0.01) catch (_:Dynamic) forbidden = true;
      check(forbidden, "Compiled straight probe motion checks intervening clearance");
    }
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
    check(Math.abs(robot.snapshot().velocities.get(0)) < 1e-5, "Search handoff requires observed joint rest before reporting stopped");
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
  static function completeProbe():Void {
    var fixture = axis(true);
    var model = fixture.model, arm = fixture.arm;
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    var channels = {arc: "torch.arc", wireSpeed: "torch.wire", voltage: "torch.voltage"};
    blueprint.addTool(new processkit.tool.WeldChannels(channels.arc, channels.wireSpeed, channels.voltage));
    blueprint.sensors.push(new RobotRuntimeSensorBlueprint("torch", WeldSensor.KIND, "tip", "tool", 1,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("complete-probe", runtime, model.name, ["base", "tool"], ["probe"]);
    var planning = WeldingPlanRunner.planning(arm, 1.0);
    var motion = new ManipulatorMotion(robot, planning.compiler, (_) -> null, () -> runtime.pollEvents(), [0]);
    var probe = new ContactProbeRunner(motion, new ProbeMotionPlanner(arm, planning.compiler), channels, "torch",
      () -> new ServoSession(robot, arm));
    var prepared = new Transform3(new Vec3(0, 0, 0.01), arm.tcpPose([0.0]).rotation);
    probe.start(new ContactProbeRequest(prepared, new Vec3(0, 0, -1), 0.04, 0.005, 0.0005, 0.002,
      WeldArcModel.TOUCH_TOLERANCE));
    var sensing = new WeldArcModel({maxCurrentA: 300.0, efficiency: 0.85, wireDiameterMm: 1.2, stickoutMm: 15.0});
    var tick = 0, safe = true, touchEpisodes = 0, touching = false;
    while (tick < 2400 && probe.running()) {
      harness.step(Int64.ofInt(tick++));
      var snapshot = robot.snapshot();
      var on = switch runtime.channelValue(channels.arc) { case Digital(value): value; default: true; };
      var feed = switch runtime.channelValue(channels.wireSpeed) { case Analog(value): value; default: -1.0; };
      safe = safe && !on && feed == 0;
      var gap = Math.max(0.0, snapshot.positions.get(0) + 0.02);
      var reading = sensing.step(0.01, {arcCommanded: on, voltageSet: 20.0, wireSpeed: feed,
        supplyReady: true, tipDistance: gap, wireDistance: gap});
      if (reading.touch && !touching) touchEpisodes++;
      touching = reading.touch;
      runtime.publishSensorFrame("torch", WeldSensor.values(reading), Int64.ofInt(tick), snapshot.sourceTimestampNs, snapshot.sourceClockId);
      probe.update(0.01);
    }
    check(probe.completed() && probe.failure == null, 'The complete checked/refined probe succeeds: ${probe.failure}, tick=$tick, q=${robot.snapshot().positions.get(0)}, contacts=$touchEpisodes');
    var point:Vec3 = cast probe.contact;
    check(point != null && Math.abs(point.z + 0.02) < 0.000006, "Fine probing improves the calibrated observation to within 6 micrometres");
    check(touchEpisodes == 2, "The executed probe withdraws and measures a fresh second contact");
    check(safe, "Arc and wire remain off throughout approach, sensing, refinement and retreat");
    check(Math.abs(robot.snapshot().positions.get(0) - 0.01) < 0.0001, "Completed probing returns to the prepared air pose");
    check(Math.abs(robot.snapshot().velocities.get(0)) < 1e-5, "Completed retreat leaves the joint at rest");
    harness.dispose();
  }
  public static function main():Void {
    run(false, false); run(true, false); run(false, true); completeProbe();
    Sys.println('Contact search native motion: $checks assertions passed');
  }
}
