package tests;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.execution.ProcessEventValue;
import robotkit.execution.ProcessTimedEvent;
import robotkit.execution.TrajectorySegment;
import robotkit.model.Joint;
import robotkit.model.JointLimits;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.VirtualDeviceOptions;
import robotkit.runtime.VirtualHeaters;
import robotkit.runtime.VirtualHeaters.HeaterSetup;
import robotkit.tool.HeaterChannels;
import robotkit.tool.HeaterSensor;

/**
 * The heaters of a virtual RKD6 device through the whole stack: the host sets targets on analog channels, the board's
 * controller (PID, watchdog, safe state) heats an assumed thermal plant, and its reports reach the runtime as
 * `tool_heater` sensor frames. A hotend and a bed on a robot of one joint, with a part fan.
 */
class HeaterDeviceTests {
  static var assertions = 0;
  static inline var HOTEND = "heater.hotend.target";
  static inline var BED = "heater.bed.target";
  static inline var FAN = "fan.part";
  /** Seconds per simulation step. */
  static inline var STEP = 0.01;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  public static function run():Int {
    assertions = 0;
    testHeatUpAndHold();
    testWatchdogFaults();
    testStopsAndPauses();
    testColdExtrusion();
    Sys.println('RobotKit heater device tests passed ($assertions assertions)');
    return assertions;
  }

  static function setups():Array<HeaterSetup> {
    var hotend = new HeaterSetup("hotend", "hotend", 0, HOTEND);
    hotend.fanChannel = FAN; hotend.extruderActuator = 0; hotend.minExtrusionTemperature = 170.0;
    hotend.fanLoss = 0.05;
    var bed = new HeaterSetup("bed", "bed", 1, BED);
    bed.kp = 0.2; bed.ki = 0.0008; bed.kd = 2.0; bed.maxTemperature = 120.0; bed.gainWindowSeconds = 120.0;
    bed.hysteresisCelsius = 10.0; bed.hysteresisWindowSeconds = 20.0;
    bed.heatCapacity = 500.0; bed.power = 250.0; bed.loss = 0.7;
    return [hotend, bed];
  }

  /** A robot of one joint (the extruder's) with a hotend and a bed, a part fan, and a plan that holds still for `seconds`. */
  static function rig(seconds:Float, hotendTarget:Float, bedTarget:Float, fan:Float, ?joint:Array<Float>):{harness:SimulationHarness, runtime:RobotRuntime, heaters:VirtualHeaters, ticks:Int} {
    var model = new RobotModel("heaters");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var axis = model.addJoint(new Joint("extruder", JointType.Revolute, base, tool));
    axis.limits = new JointLimits(-100.0, 100.0, 50.0, 100.0, 100.0);
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    blueprint.addTool(new HeaterChannels(HOTEND, FAN));
    blueprint.addTool(new HeaterChannels(BED));
    for (name in ["hotend", "bed"])
      blueprint.sensors.push(new RobotRuntimeSensorBlueprint(name, HeaterSensor.KIND, "tip", "tool", 1, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    var options = new VirtualDeviceOptions();
    options.stepsPerUnit = [1000.0];
    VirtualHeaters.configure(options, blueprint, setups());
    var harness = new SimulationHarness(STEP);
    var runtime = harness.simulation.addRobot(blueprint, null, options);
    var heaters = new VirtualHeaters(harness.simulation, runtime, 0, setups());
    var tick = 0;
    for (_ in 1...21) harness.step(Int64.ofInt(++tick));
    var segment = new TrajectorySegment(Int64.ofInt(0), Int64.fromFloat(seconds * 1e9), [joint == null ? [0.0] : joint]);
    runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(900), Int64.ofInt(blueprint.revision),
      Int64.ofInt(blueprint.calibrationRevision), RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE |
      RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_EVENTS, [0.0], [0.0], [0.0], [segment], null, null, null, null, null, true,
      [new ProcessTimedEvent(Int64.ofInt(0), HOTEND, ProcessEventValue.Analog(hotendTarget)),
       new ProcessTimedEvent(Int64.ofInt(0), BED, ProcessEventValue.Analog(bedTarget)),
       new ProcessTimedEvent(Int64.ofInt(0), FAN, ProcessEventValue.Analog(fan))]), 1);
    return {harness: harness, runtime: runtime, heaters: heaters, ticks: tick};
  }

  /** Steps `seconds`; returns the tick count reached. */
  static function advance(rig:{harness:SimulationHarness, runtime:RobotRuntime, heaters:VirtualHeaters, ticks:Int}, seconds:Float):Int {
    var tick = rig.ticks;
    for (_ in 0...Math.round(seconds / STEP)) { rig.harness.step(Int64.ofInt(++tick)); rig.heaters.observe(); }
    return tick;
  }

  static function testHeatUpAndHold():Void {
    var r = rig(400.0, 210.0, 60.0, 0.0);
    var hotendAt = -1.0, bedAt = -1.0, tick = r.ticks, peak = 0.0;
    for (step in 0...Math.round(200.0 / STEP)) {
      r.harness.step(Int64.ofInt(++tick)); r.heaters.observe();
      var seconds = (step + 1) * STEP;
      if (hotendAt < 0 && r.heaters.reading("hotend").atTarget) hotendAt = seconds;
      if (bedAt < 0 && r.heaters.reading("bed").atTarget) bedAt = seconds;
      peak = Math.max(peak, r.heaters.reading("hotend").celsius);
    }
    Sys.println('heaters: hotend at 210 C after ${Math.round(hotendAt * 10) / 10} s (peak ${Math.round(peak * 10) / 10} C), bed at 60 C after ${Math.round(bedAt * 10) / 10} s');
    check(hotendAt > 40 && hotendAt < 100, 'the hotend reaches 210 C in about a minute, took $hotendAt s');
    check(bedAt > 50 && bedAt < 160, 'the bed reaches 60 C in a minute or two, took $bedAt s');
    var hotend = r.heaters.reading("hotend");
    check(hotend.fault == 0 && Math.abs(hotend.celsius - 210.0) < 2.0 && hotend.targetCelsius == 210.0 && hotend.duty > 0.0 && hotend.duty < 1.0,
      "the hotend holds its target with a duty between off and full");
    // The sensors in the runtime carry the same frame, in the order the contract fixes.
    var frame:Null<robotkit.core.SensorFrame> = null;
    for (candidate in r.runtime.snapshot().sensors.toArray()) if (candidate.sensorId == "hotend") frame = candidate;
    check(frame != null && frame.kind == HeaterSensor.KIND && frame.values.length == HeaterSensor.COUNT &&
      Math.abs(frame.values.get(HeaterSensor.TEMPERATURE) - hotend.celsius) < 1.0, "the runtime's tool_heater sensor holds the reading");
    check(peak < 216.0, "the hotend overshoots by a few degrees at most");
    r.harness.dispose();
  }

  static function testWatchdogFaults():Void {
    // A fault in each of the ways a heater runs away; each switches the target channels off and stops the robot.
    for (case_ in [{label: "a dead element", kind: VirtualHeaters.ELEMENT, value: 0.0, code: HeaterSensor.NOT_HEATING},
        {label: "a fallen thermistor", kind: VirtualHeaters.SENSOR_ATTACHED, value: 0.0, code: HeaterSensor.NOT_HEATING},
        {label: "a draught", kind: VirtualHeaters.BLAST, value: 3.0, code: HeaterSensor.NOT_HEATING},
        {label: "a broken thermistor wire", kind: VirtualHeaters.SENSOR_OPEN, value: 1.0, code: HeaterSensor.SENSOR_OPEN},
        {label: "a shorted thermistor", kind: VirtualHeaters.SENSOR_SHORT, value: 1.0, code: HeaterSensor.SENSOR_SHORT}]) {
      var r = rig(400.0, 210.0, 60.0, 0.0);
      r.heaters.setInput("hotend", case_.kind, case_.value);
      advance(r, 60.0);
      var hotend = r.heaters.reading("hotend");
      check(hotend.fault == case_.code, '${case_.label} is reported as fault ${case_.code}, got ${hotend.fault}: ${HeaterSensor.faultMessage(hotend.fault)}');
      check(hotend.duty == 0.0, '${case_.label} switches the hotend off');
      check(r.runtime.snapshot().safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT || r.runtime.snapshot().faultCode != 0,
        '${case_.label} stops the robot');
      check(r.heaters.reading("bed").targetCelsius == 0.0 && r.heaters.reading("bed").duty == 0.0, '${case_.label} takes the bed off too');
      r.harness.dispose();
    }
  }

  static function testStopsAndPauses():Void {
    // A commanded stop (a pause) keeps both targets; an emergency stop lets go of them and the heaters cool.
    var held = rig(400.0, 210.0, 60.0, 0.0);
    advance(held, 80.0);
    held.runtime.submitStop(2, false);
    advance(held, 20.0);
    check(held.heaters.reading("hotend").targetCelsius == 210.0 && held.heaters.reading("hotend").celsius > 200.0, "a pause keeps the hotend hot");
    held.harness.dispose();
    var cut = rig(400.0, 210.0, 60.0, 0.0);
    advance(cut, 80.0);
    cut.runtime.submitStop(2, true);
    advance(cut, 5.0);
    check(cut.heaters.reading("hotend").targetCelsius == 0.0 && cut.heaters.reading("hotend").duty == 0.0, "an emergency stop switches the hotend off");
    check(cut.heaters.reading("bed").targetCelsius == 0.0, "an emergency stop switches the bed off");
    cut.harness.dispose();
  }

  static function testColdExtrusion():Void {
    // The extruder's joint moves forward while the hotend is cold: the board faults.
    var r = rig(60.0, 210.0, 0.0, 0.0, [0.0, 0.05]);
    advance(r, 10.0);
    var hotend = r.heaters.reading("hotend");
    check(hotend.fault == HeaterSensor.COLD_EXTRUSION, 'a cold hotend may not extrude: fault ${hotend.fault}');
    check(HeaterSensor.faultMessage(hotend.fault).indexOf("cold extrusion") >= 0, "and says so");
    r.harness.dispose();
  }
}
