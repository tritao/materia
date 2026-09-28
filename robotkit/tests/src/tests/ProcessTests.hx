package tests;

import haxe.Int64;
import motionkit.event.EventValue;
import motionkit.robot.ToolpathPosePath;
import robotkit.process.Toolpath;
import robotkit.process.ToolpathPoint;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.ChannelToolAdapter;
import robotkit.tool.SimulatedSprayer;
import robotkit.tool.SimulatedGripper;
import robotkit.tool.SimulatedVacuum;
import robotkit.tool.Tool;
import robotkit.tool.ToolRuntime;
import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventValue;

/** Toolpath authoring and B1 process-path conversion acceptance. */
class ProcessTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testToolpathConversion();
    testScheduledToolEvents();
    testMultiCapabilityTool();
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

  static function approx(a:Float, b:Float, tolerance:Float):Bool
    return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
