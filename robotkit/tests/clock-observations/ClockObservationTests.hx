import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.VirtualDeviceOptions;
import robotkit.simulation.SimulatedRobot;
import robotkit.time.SourceClock;

class ClockObservationTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }

  static function exerciseTripSwitch():Void {
    var model = new RobotModel("switch-clock-test");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var joint = model.addJoint(new Joint("axis", JointType.Revolute, base, tool));
    joint.limits = new JointLimits(-2, 2, 3, 10, 10);
    var frame = model.addFrame(new robotkit.model.Frame("home-frame", base));
    var sensor = new robotkit.model.Sensor("home", "trip_switch");
    sensor.frame = frame;
    model.addSensor(sensor);
    model.addSwitch(new robotkit.model.JointSwitch("home", "axis", "home-frame", "home", -1, -0.01, 0, 0));
    var harness = new SimulationHarness(0.01);
    try {
      var runtime = harness.simulation.addRobot(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()));
      var robot = new SimulatedRobot("switch-clock-test", runtime, "switch-clock-test", ["base", "tool"], ["axis"]);
      var previousClock = "";
      for (epoch in 0...3) {
        for (_ in 0...30) harness.step();
        var snapshot = robot.snapshot();
        check(snapshot.sourceClockId != previousClock, "Trip switch reset creates a fresh joint clock epoch");
        var observed:Null<robotkit.core.SensorFrame> = null;
        for (index in 0...snapshot.sensors.length)
          if (snapshot.sensors.get(index).sensorId == "home") observed = snapshot.sensors.get(index);
        check(observed != null, "The physical trip switch publishes an observation");
        if (observed == null) throw "Missing physical trip switch observation";
        check(observed.sourceClockId == snapshot.sourceClockId, "Trip switch follows its endpoint clock after reset");
        check(observed.sourceTimestampNs == snapshot.sourceTimestampNs, "Trip switch and joint observations share source time");
        previousClock = snapshot.sourceClockId;
        if (epoch == 0) harness.reset(); else if (epoch == 1) harness.resetRobot(0);
      }
      robot.close();
    } catch (error:Dynamic) { harness.dispose(); throw error; }
    harness.dispose();
  }

  static function exercise(device:Bool):Void {
    var model = new RobotModel("clock-test");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var joint = model.addJoint(new Joint("axis", JointType.Revolute, base, tool));
    joint.limits = new JointLimits(-2, 2, 3, 10, 10);
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    blueprint.sensors.resize(0);
    if (!device) blueprint.sensors.push(new RobotRuntimeSensorBlueprint("encoder", "joint_encoder", "base", "base", 0,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    blueprint.sensors.push(new RobotRuntimeSensorBlueprint("external", "tool_contact", "tool", "tool", 1,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    var options:Null<VirtualDeviceOptions> = null;
    if (device) {
      options = new VirtualDeviceOptions();
      options.offsetTicks = Int64.ofInt(70000);
      options.driftPpm = 125;
    }
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint, null, options);
    var robot = new SimulatedRobot("clock-test", runtime, "clock-test", ["base", "tool"], ["axis"]);
    try {
      for (_ in 0...30) harness.step();
      var first = robot.snapshot();
      check(first.sourceClockId.indexOf(device ? "robotkit.device." : "robotkit.simulation.") == 0,
        "The endpoint factory declares the actual joint clock");
      check(first.sourceClockId == runtime.endpoint.sourceClockId(), "Robot adapter retains endpoint clock identity");
      if (device) check(Int64.toFloat(first.sourceTimestampNs) > 340000000.0,
        "Board time retains its configured offset instead of being relabelled as simulation time");
      else {
        check(first.sensors.length == 1, "The measured encoder is published");
        check(first.sensors.get(0).sourceClockId == first.sourceClockId, "Native encoder shares the endpoint clock");
      }
      runtime.publishSensorFrame("external", [0.0], Int64.ofInt(1), first.sourceTimestampNs, "unspecified");
      var unknown = robot.snapshot();
      var external = unknown.sensors.get(unknown.sensors.length - 1);
      check(external.sourceClockId == "unspecified", "Unknown external clocks remain unresolved");
      runtime.publishSensorFrame("external", [0.0], Int64.ofInt(2), first.sourceTimestampNs, "independent.device.0");
      var explicit = robot.snapshot();
      check(explicit.sensors.get(explicit.sensors.length - 1).sourceClockId == "independent.device.0",
        "External device clocks are preserved rather than reassigned");
      for (_ in 0...100) harness.step();
      var later = robot.snapshot();
      var elapsed = Int64.toFloat(later.sourceTimestampNs - first.sourceTimestampNs);
      check(Math.abs(elapsed - (device ? 1000125000.0 : 1000000000.0)) < (device ? 11000000.0 : 10.0),
        "Joint timestamps follow their endpoint oscillator including board drift");
      harness.reset();
      check(runtime.endpoint.sourceClockId() != later.sourceClockId, "Shared-session reset invalidates the old epoch immediately");
      var resetClock = runtime.endpoint.sourceClockId();
      for (_ in 0...30) harness.step();
      var after = robot.snapshot();
      check(after.sourceClockId == resetClock, "New observations use the reset epoch");
      check(after.sourceClockId != first.sourceClockId, "Repeating a prior timestamp cannot reuse its epoch");
      if (!device) check(after.sourceTimestampNs == first.sourceTimestampNs, "Regression test repeats the old numeric timestamp");
      harness.resetRobot(0);
      check(runtime.endpoint.sourceClockId() != resetClock, "Individual robot reset also invalidates its epoch");
    } catch (error:Dynamic) { harness.dispose(); throw error; }
    robot.close(); harness.dispose();
  }

  public static function main():Void {
    if (Sys.args().length > 0 && Sys.args()[0] == "--identity") {
      Sys.println(new SourceClock("device").id()); return;
    }
    var a = new SourceClock("device"), b = new SourceClock("device");
    check(a.id() != b.id(), "Independent owners never infer a shared clock from its domain name");
    var old = a.id(); a.observe(Int64.ofInt(10)); a.observe(Int64.ofInt(9));
    check(a.id() != old, "Unexpected clock regression creates a distinct epoch");
    exercise(false); exercise(true); exerciseTripSwitch();
    Sys.println('Clock observations: $checks assertions passed');
  }
}
