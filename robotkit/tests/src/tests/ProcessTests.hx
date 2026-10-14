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
import robotkit.tool.ToolChangeTransaction;
import robotkit.tool.ToolChangeTransaction.ToolChangePhase;
import robotkit.tool.ChangerLock;
import robotkit.tool.ChangerLockObservation;
import robotkit.tool.SimulatedChangerLock;
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
    testToolProcessOwnership();
    testToolChangeTransaction();
    testToolChangeCommandRetry();
    testToolChangeRejectsStaleObservation();
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

  static function testToolProcessOwnership():Void {
    var gripper = new SimulatedGripper();
    var first = new ToolRuntime(new Tool("gripper", "gripper", Transform3.identity()), gripper);
    first.bindGripper("gripper.close");
    var second = new ToolRuntime(new Tool("screwdriver", "screwdriver", Transform3.identity()));
    var selection = new ToolRuntimeSelection();
    selection.select(first, Int64.ofInt(10));
    selection.beginProcess("pick:part-17");
    check(selection.activeProcessOwner() == "pick:part-17" &&
      selection.activeProcessToolId() == "gripper", "a process leases the selected tool identity");

    var rejected = false;
    try selection.select(second, Int64.ofInt(20)) catch (error:Dynamic)
      rejected = Std.string(error).indexOf("owns tool") >= 0;
    check(rejected && selection.active() == first,
      "tool identity cannot change while its process still owns outputs");
    rejected = false;
    try selection.apply(new FiredProcessEvent(Int64.ofInt(1), "gripper.close",
      ProcessEventValue.Digital(true), Int64.ofInt(11), Int64.ofInt(11), 1)) catch (error:Dynamic)
      rejected = Std.string(error).indexOf("needs owner") >= 0;
    check(rejected && gripper.isOpen(), "unowned output cannot enter a live process");
    rejected = false;
    try selection.applyFor("fastening:screw-4", new FiredProcessEvent(Int64.ofInt(1), "gripper.close",
      ProcessEventValue.Digital(true), Int64.ofInt(12), Int64.ofInt(12), 1)) catch (error:Dynamic)
      rejected = Std.string(error).indexOf("does not own") >= 0;
    check(rejected && gripper.isOpen(), "another process cannot write the leased tool");

    selection.applyFor("pick:part-17", new FiredProcessEvent(Int64.ofInt(1), "gripper.close",
      ProcessEventValue.Digital(true), Int64.ofInt(13), Int64.ofInt(13), 1));
    check(!gripper.isOpen(), "the owning process can write its selected tool");
    rejected = false;
    try selection.finishProcess("fastening:screw-4") catch (error:Dynamic)
      rejected = Std.string(error).indexOf("cannot release") >= 0;
    check(rejected && selection.activeProcessOwner() == "pick:part-17",
      "a different process cannot release the tool lease");
    rejected = false;
    try selection.finishProcess("pick:part-17") catch (error:Dynamic)
      rejected = Std.string(error).indexOf("outputs are unsafe") >= 0;
    check(rejected && selection.activeProcessOwner() == "pick:part-17",
      "a process cannot release its lease while a tool output remains active");
    selection.applyFor("pick:part-17", new FiredProcessEvent(Int64.ofInt(2), "gripper.close",
      ProcessEventValue.Digital(false), Int64.ofInt(14), Int64.ofInt(14), 1));
    selection.finishProcess("pick:part-17");
    selection.select(second, Int64.ofInt(20));
    check(selection.active() == second && selection.activeProcessOwner() == null,
      "a stopped process releases the outgoing tool for a confirmed exchange");
  }

  static function testToolChangeTransaction():Void {
    var lock = new SimulatedChangerLock("gripper");
    var gripper = new ToolRuntime(new Tool("gripper", "gripper", Transform3.identity()),
      new SimulatedGripper(), null, null, null, null, lock, "gripper");
    gripper.bindChangerLock("changer.lock");
    var screwdriver = new ToolRuntime(new Tool("screwdriver", "screwdriver", Transform3.identity()),
      null, null, null, null, null, lock, "screwdriver");
    screwdriver.bindChangerLock("changer.lock");
    var selection = new ToolRuntimeSelection();
    selection.select(gripper, Int64.ofInt(5));
    var change = new ToolChangeTransaction(selection, [gripper, screwdriver], Int64.ofInt(10));
    change.begin("screwdriver", Int64.ofInt(10));
    check(change.phase == ToolChangePhase.UnlockRequested && selection.active() == null &&
      !lock.observation().locked, "tool exchange safes routing and requests unlock after docking");
    var rejected = false;
    try change.confirmUnlocked(Int64.ofInt(9)) catch (error:Dynamic)
      rejected = Std.string(error).indexOf("event time moved backwards") >= 0;
    check(rejected && change.phase == ToolChangePhase.UnlockRequested,
      "tool exchange rejects lock feedback events that move backward in time");
    change.confirmUnlocked(Int64.ofInt(10));
    check(change.phase == ToolChangePhase.OutgoingUnlocked,
      "tool exchange waits for measured unlocked feedback");
    rejected = false;
    try change.confirmAttached(Int64.ofInt(11)) catch (_:Dynamic) rejected = true;
    check(rejected && change.phase == ToolChangePhase.OutgoingUnlocked,
      "incoming attachment cannot skip outgoing detachment");
    lock.detachTool(Int64.ofInt(12));
    change.confirmDetached(Int64.ofInt(12));
    lock.attachTool("screwdriver", Int64.ofInt(13));
    change.confirmAttached(Int64.ofInt(13));
    check(change.phase == ToolChangePhase.LockRequested && lock.observation().locked,
      "tool exchange requests lock only after measured incoming identity");
    var lockCommands = lock.history.length;
    change.requestLock(Int64.ofInt(13));
    check(lock.history.length == lockCommands,
      "retrying an already confirmed lock does not issue another command");
    change.confirmLocked(Int64.ofInt(13));
    check(change.phase == ToolChangePhase.Complete && selection.active() == screwdriver &&
      lock.observation().toolId == "screwdriver",
      "tool exchange selects the new runtime only after measured lock confirmation");
  }

  static function testToolChangeCommandRetry():Void {
    var lock = new RetryChangerLock("gripper");
    var gripper = new ToolRuntime(new Tool("gripper", "gripper", Transform3.identity()),
      null, null, null, null, null, lock, "gripper");
    gripper.bindChangerLock("changer.lock");
    var screwdriver = new ToolRuntime(new Tool("screwdriver", "screwdriver", Transform3.identity()),
      null, null, null, null, null, lock, "screwdriver");
    screwdriver.bindChangerLock("changer.lock");
    var selection = new ToolRuntimeSelection();
    selection.select(gripper, Int64.ofInt(5));
    var change = new ToolChangeTransaction(selection, [gripper, screwdriver], Int64.ofInt(10));
    lock.failNextUnlock = true;
    var interrupted = false;
    try change.begin("screwdriver", Int64.ofInt(10)) catch (_:Dynamic) interrupted = true;
    check(interrupted && change.phase == ToolChangePhase.UnlockRequested && selection.active() == null,
      "failed unlock command retains the exchange phase and safe runtime selection");
    change.requestUnlock(Int64.ofInt(11));
    change.confirmUnlocked(Int64.ofInt(11));
    lock.detachTool(Int64.ofInt(12));
    change.confirmDetached(Int64.ofInt(12));
    lock.attachTool("screwdriver", Int64.ofInt(13));
    lock.failNextLock = true;
    interrupted = false;
    try change.confirmAttached(Int64.ofInt(13)) catch (_:Dynamic) interrupted = true;
    check(interrupted && change.phase == ToolChangePhase.LockRequested &&
      !lock.observation().locked && selection.active() == null,
      "failed lock command retains the incoming mount without selecting its runtime");
    change.requestLock(Int64.ofInt(14));
    change.confirmLocked(Int64.ofInt(14));
    check(change.phase == ToolChangePhase.Complete && selection.active() == screwdriver,
      "a retried physical lock completes the exchange before selecting the new runtime");
  }

  static function testToolChangeRejectsStaleObservation():Void {
    var lock = new SimulatedChangerLock("gripper");
    var gripper = new ToolRuntime(new Tool("gripper", "gripper", Transform3.identity()),
      null, null, null, null, null, lock, "gripper");
    gripper.bindChangerLock("changer.lock");
    var screwdriver = new ToolRuntime(new Tool("screwdriver", "screwdriver", Transform3.identity()),
      null, null, null, null, null, lock, "screwdriver");
    screwdriver.bindChangerLock("changer.lock");
    var selection = new ToolRuntimeSelection();
    selection.select(gripper, Int64.ofInt(5));
    var rejectedAge = false;
    try new ToolChangeTransaction(selection, [gripper, screwdriver], Int64.ofInt(0))
    catch (error:Dynamic) rejectedAge = Std.string(error).indexOf("positive commissioned") >= 0;
    check(rejectedAge && selection.active() == gripper,
      "tool exchange requires an explicit positive lock-feedback freshness limit");
    var change = new ToolChangeTransaction(selection, [gripper, screwdriver], Int64.ofInt(2));
    var rejected = false;
    try change.begin("screwdriver", Int64.ofInt(10)) catch (error:Dynamic)
      rejected = Std.string(error).indexOf("stale") >= 0;
    check(rejected && change.phase == ToolChangePhase.Idle && selection.active() == gripper,
      "stale lock and tool-identity feedback cannot begin a tool exchange");

    var advancingLock = new SimulatedChangerLock("gripper");
    var advancingGripper = new ToolRuntime(new Tool("gripper", "gripper", Transform3.identity()),
      null, null, null, null, null, advancingLock, "gripper");
    advancingGripper.bindChangerLock("changer.lock");
    var advancingScrewdriver = new ToolRuntime(new Tool("screwdriver", "screwdriver", Transform3.identity()),
      null, null, null, null, null, advancingLock, "screwdriver");
    advancingScrewdriver.bindChangerLock("changer.lock");
    var advancingSelection = new ToolRuntimeSelection();
    advancingSelection.select(advancingGripper, Int64.ofInt(5));
    var advancing = new ToolChangeTransaction(advancingSelection,
      [advancingGripper, advancingScrewdriver], Int64.ofInt(10));
    advancing.begin("screwdriver", Int64.ofInt(10));
    rejected = false;
    try advancing.confirmUnlocked(Int64.ofInt(21)) catch (error:Dynamic)
      rejected = Std.string(error).indexOf("stale") >= 0;
    check(rejected && advancing.phase == ToolChangePhase.UnlockRequested &&
      advancingSelection.active() == null,
      "tool exchange rechecks feedback freshness after unlock while preserving its retryable phase");
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

private class RetryChangerLock implements ChangerLock {
  final simulated:SimulatedChangerLock;
  public var failNextUnlock:Bool = false;
  public var failNextLock:Bool = false;

  public function new(initialToolId:String) {
    simulated = new SimulatedChangerLock(initialToolId);
  }

  public function lock(timestampNs:Int64):Void {
    if (failNextLock) {
      failNextLock = false;
      throw "simulated changer lock command interruption";
    }
    simulated.lock(timestampNs);
  }

  public function unlock(timestampNs:Int64):Void {
    if (failNextUnlock) {
      failNextUnlock = false;
      throw "simulated changer unlock command interruption";
    }
    simulated.unlock(timestampNs);
  }

  public function observation():ChangerLockObservation return simulated.observation();
  public function detachTool(timestampNs:Int64):Void simulated.detachTool(timestampNs);
  public function attachTool(toolId:String, timestampNs:Int64):Void simulated.attachTool(toolId, timestampNs);
}
