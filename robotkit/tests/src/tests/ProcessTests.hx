package tests;

import haxe.Int64;
import motionkit.event.EventValue;
import processkit.motion.ToolpathPosePath;
import processkit.path.Toolpath;
import processkit.path.ToolpathPoint;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.ChannelToolAdapter;
import robotkit.tool.SimulatedSprayer;
import robotkit.tool.SimulatedGripper;
import robotkit.tool.SimulatedVacuum;
import robotkit.tool.Tool;
import robotkit.tool.ToolRuntime;
import robotkit.tool.ToolRuntimeSelection;
import robotkit.tool.SimulatedToolSensorAdapter;
import robotkit.execution.FiredProcessEvent;
import robotkit.execution.ProcessEventValue;
import robotkit.core.RobotSnapshot;
import robotkit.core.SensorFrame;

/** Toolpath authoring and B1 process-path conversion acceptance. */
class ProcessTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testToolpathConversion();
    testScheduledToolEvents();
    testMultiCapabilityTool();
    testSelectedToolSensorFrames();
    testSimulationSensorClock();
    Sys.println('RobotKit process tests passed ($assertions assertions)');
    return assertions;
  }

  static function testToolpathConversion():Void {
    var identity = Quat.identity();
    var p0 = new ToolpathPoint(new Transform3(new Vec3(0.0, 0.0, 0.0), identity), 0.1, false);
    var p1 = new ToolpathPoint(new Transform3(new Vec3(1.0, 0.0, 0.0), identity), 0.1, true);
    var p2 = new ToolpathPoint(new Transform3(new Vec3(1.0, 2.0, 0.0), identity), 0.1, true);
    var p3 = new ToolpathPoint(new Transform3(new Vec3(1.0, 2.0, 0.0), identity), 0.1, false);
    var toolpath = new Toolpath("work", [p0, p1, p2, p3]);

    check(approx(toolpath.length(), 3.0, 1e-9), "Toolpath length sums consecutive point distances");
    check(approx(toolpath.processOnLength(), 2.0, 1e-9),
      "Toolpath process-on length counts departing process moves");
    var segments = toolpath.segmentByProcess();
    check(segments.approach.points.length == 1 && segments.process.points.length == 2 &&
      segments.retract.points.length == 1, "Toolpath process runs retain their authored points");

    var path = ToolpathPosePath.convert(toolpath, "sprayer.flow");
    check(path.frameId == "work" && approx(path.length(), 3.0, 1e-9),
      "B1 conversion keeps the frame and geometric length");
    check(path.primitives.length == 2,
      "B1 conversion drops the duplicate zero-length terminal move");
    check(path.events.length == 2 &&
      approx(path.events[0].distance, 1.0, 1e-9) &&
      approx(path.events[1].distance, 3.0, 1e-9),
      "Process transitions are placed at path distance");
    check(switch [path.events[0].value, path.events[1].value] {
      case [EventValue.Digital(true), EventValue.Digital(false)]: true;
      case _: false;
    }, "Process transitions turn the tool on and off");
    var midpoint = path.waypointAt(2.0).pose;
    check(approx(midpoint.x, 1.0, 1e-9) && approx(midpoint.y, 1.0, 1e-9),
      "B1 path evaluates the process span in its frame");
  }

  static function testScheduledToolEvents():Void {
    var sprayer = new SimulatedSprayer();
    var adapter = new ChannelToolAdapter();
    adapter.bindSprayerFlow("sprayer.flow", sprayer, 1.5);
    adapter.apply(new FiredProcessEvent(Int64.ofInt(7), "sprayer.flow",
      ProcessEventValue.Digital(true), Int64.ofInt(200), Int64.ofInt(210), 1));
    adapter.apply(new FiredProcessEvent(Int64.ofInt(7), "sprayer.flow",
      ProcessEventValue.Digital(false), Int64.ofInt(400), Int64.ofInt(410), 1));
    check(sprayer.history.length == 2, "Each runtime record changes the sprayer once");
    check(sprayer.history[0].flow == 1.5 && sprayer.history[1].flow == 0.0,
      "Sprayer follows planned flow events");
    check(Int64.compare(sprayer.history[0].timestampNs, Int64.ofInt(200)) == 0 &&
      Int64.compare(sprayer.history[1].timestampNs, Int64.ofInt(400)) == 0,
      "Sprayer timestamps use scheduled trajectory time");
  }

  static function testMultiCapabilityTool():Void {
    var gripper = new SimulatedGripper();
    var vacuum = new SimulatedVacuum();
    var tool = new Tool("combination", "combination", Transform3.identity());
    var runtime = new ToolRuntime(tool, gripper, vacuum);
    check(runtime.tool == tool && runtime.gripper == gripper && runtime.vacuum == vacuum,
      "One mounted tool exposes both independent capabilities");

    var adapter = new ChannelToolAdapter();
    adapter.bindGripper("tool.grip", runtime.gripper);
    adapter.bindVacuum("tool.vacuum", runtime.vacuum);
    adapter.apply(new FiredProcessEvent(Int64.ofInt(1), "tool.grip",
      ProcessEventValue.Digital(true), Int64.ofInt(100), Int64.ofInt(105), 1));
    adapter.apply(new FiredProcessEvent(Int64.ofInt(1), "tool.vacuum",
      ProcessEventValue.Digital(true), Int64.ofInt(200), Int64.ofInt(205), 1));
    check(!gripper.isGrasped() && !vacuum.isHolding(),
      "Commands alone do not claim a successful pickup");
    gripper.observeContact(false, Int64.ofInt(220));
    vacuum.observeVacuumKpa(20, Int64.ofInt(230));
    check(!gripper.isGrasped() && !vacuum.isHolding(),
      "Missed contact and low vacuum report no pickup");
    gripper.observeContact(true, Int64.ofInt(240));
    vacuum.observeVacuumKpa(45, Int64.ofInt(250));
    check(gripper.isGrasped() && vacuum.isHolding(),
      "Contact and sufficient vacuum confirm independent pickups");
    adapter.apply(new FiredProcessEvent(Int64.ofInt(1), "tool.grip",
      ProcessEventValue.Digital(false), Int64.ofInt(300), Int64.ofInt(305), 1));
    check(gripper.isOpen() && vacuum.isHolding(), "Opening the gripper leaves vacuum active");
    adapter.apply(new FiredProcessEvent(Int64.ofInt(1), "tool.vacuum",
      ProcessEventValue.Digital(false), Int64.ofInt(400), Int64.ofInt(405), 1));
    check(!vacuum.isEnabled() && !vacuum.isHolding(), "Vacuum off releases its pickup");
    gripper.close(Int64.ofInt(450));
    vacuum.enable(Int64.ofInt(460));
    check(!gripper.isGrasped() && !vacuum.isHolding() && vacuum.vacuumKpa() == 0,
      "New commands require fresh feedback after a release");
    gripper.observeContact(true, Int64.ofInt(470));
    vacuum.observeVacuumKpa(40, Int64.ofInt(480));
    check(gripper.isGrasped() && vacuum.isHolding(),
      "Feedback at the vacuum threshold confirms a new pickup");
    gripper.observeContact(false, Int64.ofInt(490));
    vacuum.observeVacuumKpa(39, Int64.ofInt(500));
    check(!gripper.isGrasped() && !vacuum.isHolding(),
      "Lost contact or pressure clears observed pickup");
    check(gripper.history.length == 3 && vacuum.history.length == 3 &&
      Int64.compare(vacuum.history[0].timestampNs, Int64.ofInt(200)) == 0,
      "Each capability records its own scheduled commands");
    check(gripper.observations.length == 4 && vacuum.observations.length == 4 &&
      Int64.compare(vacuum.observations[1].timestampNs, Int64.ofInt(250)) == 0,
      "Sensor observations are recorded separately from commands");
    var rejected = false;
    try adapter.apply(new FiredProcessEvent(Int64.ofInt(1), "tool.vacuum",
      ProcessEventValue.Analog(1.0), Int64.ofInt(500), Int64.ofInt(505), 1))
    catch (_:Dynamic) rejected = true;
    check(rejected && vacuum.history.length == 3,
      "Vacuum rejects non-digital events without changing state");
    rejected = false;
    try vacuum.observeVacuumKpa(Math.NaN, Int64.ofInt(510)) catch (_:Dynamic) rejected = true;
    check(rejected && vacuum.observations.length == 4,
      "Invalid vacuum measurements do not change observed state");
  }

  static function testSelectedToolSensorFrames():Void {
    var first = new ToolRuntime(new Tool("first", "first", Transform3.identity()),
      new SimulatedGripper(), new SimulatedVacuum());
    first.bindGripper("first.close");
    first.bindVacuum("first.vacuum");
    var second = new ToolRuntime(new Tool("second", "second", Transform3.identity()),
      new SimulatedGripper());
    second.bindGripper("second.close");
    var selection = new ToolRuntimeSelection();
    var sensors = new SimulatedToolSensorAdapter(selection);
    sensors.bindGripperContact(first, "first/contact");
    sensors.bindVacuumPressure(first, "first/pressure");
    sensors.bindGripperContact(second, "second/contact");
    selection.select(first, Int64.ofInt(100));
    selection.apply(new FiredProcessEvent(Int64.ofInt(1), "first.close",
      ProcessEventValue.Digital(true), Int64.ofInt(101), Int64.ofInt(101), 1));
    selection.apply(new FiredProcessEvent(Int64.ofInt(1), "first.vacuum",
      ProcessEventValue.Digital(true), Int64.ofInt(102), Int64.ofInt(102), 1));
    var missed = new RobotSnapshot("robot", Int64.ofInt(1), Int64.ofInt(110),
      [], [], [], 0, 0, null, [toolSensor("first/contact", "tool_contact", 1, 110, 0),
        toolSensor("first/pressure", "tool_vacuum_kpa", 1, 110, 20)]);
    check(sensors.applySnapshot(missed) == 2 && !first.gripper.isGrasped() &&
      !first.vacuum.isHolding(), "missed pickup sensor frames leave both capabilities unheld");
    var pickup = new RobotSnapshot("robot", Int64.ofInt(2), Int64.ofInt(120),
      [], [], [], 0, 0, null, [toolSensor("first/contact", "tool_contact", 2, 120, 1),
        toolSensor("first/pressure", "tool_vacuum_kpa", 2, 120, 45)]);
    check(sensors.applySnapshot(pickup) == 2 && first.gripper.isGrasped() &&
      first.vacuum.isHolding(), "fresh sensor frames confirm both pickups");
    check(sensors.applySnapshot(pickup) == 0, "repeated snapshot sequences do not replay feedback");

    selection.select(second, Int64.ofInt(150));
    check(!first.gripper.isGrasped() && !first.vacuum.isHolding(),
      "tool change releases the previous configuration");
    selection.apply(new FiredProcessEvent(Int64.ofInt(2), "second.close",
      ProcessEventValue.Digital(true), Int64.ofInt(151), Int64.ofInt(151), 1));
    check(!sensors.apply(toolSensor("first/contact", "tool_contact", 3, 160, 1)) &&
      !sensors.apply(toolSensor("second/contact", "tool_contact", 1, 140, 1)),
      "inactive and pre-selection observations are ignored");
    check(sensors.apply(toolSensor("second/contact", "tool_contact", 2, 170, 1)) &&
      second.gripper.isGrasped(), "active tool accepts fresh contact feedback");
    check(sensors.apply(toolSensor("second/contact", "tool_contact", 3, 180, 0)) &&
      !second.gripper.isGrasped(), "lost contact clears the active grasp");
  }

  static function testSimulationSensorClock():Void {
    var tool = new ToolRuntime(new Tool("sim-cup", "sim-cup", Transform3.identity()),
      null, new SimulatedVacuum());
    var selection = new ToolRuntimeSelection();
    var sensors = new SimulatedToolSensorAdapter(selection);
    sensors.bindVacuumPressure(tool, "sim/pressure");
    selection.select(tool, Int64.ofInt(10), "robotkit.simulation");
    tool.vacuum.enable(Int64.ofInt(10));
    var frame = new SensorFrame("sim/pressure", "tool_vacuum_kpa", "tool",
      Int64.ofInt(1), Int64.ofInt(11), [45.0], Int64.ofInt(11),
      "", null, null, "robotkit.simulation");
    check(sensors.apply(frame) && tool.vacuum.isHolding(),
      "simulation-clock pressure feedback confirms a pickup");
    var wrongClock = false;
    try sensors.apply(toolSensor("sim/pressure", "tool_vacuum_kpa", 2, 12, 0))
    catch (error:Dynamic) wrongClock = Std.string(error).indexOf("different source clock") >= 0;
    check(wrongClock, "pressure feedback rejects a clock different from tool selection");
  }

  static function toolSensor(id:String, kind:String, sequence:Int, timestamp:Int, value:Float):SensorFrame
    return new SensorFrame(id, kind, "tool", Int64.ofInt(sequence), Int64.ofInt(timestamp),
      [value], Int64.ofInt(timestamp), "", null, null, "robotkit.monotonic");

  static function approx(a:Float, b:Float, tolerance:Float):Bool
    return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
