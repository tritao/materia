import haxe.Int64;
import cnckit.CncCompiler;
import motionkit.event.EventValue;
import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.ToolpathMotionBinding;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MachineKitRobotCompiler;
import motionkit.robot.MotionSystem;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.program.MotionProgram;
import robotkit.model.Actuator;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.VirtualDeviceOptions;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import robotkit.simulation.SimulatedRobot;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;


class ToolpathTestSupport extends MotionKitTestSupport {
  public static function cncBinding(cnc:MotionCncRig, blueprint:MotionSystemBlueprint):ToolpathMotionBinding {
    return new ToolpathMotionBinding(cnc.binding, blueprint);
  }

  public static function cncProgram(cnc:MotionCncRig, source:String):MotionProgram {
    var parsed = CncCompiler.compileDetailed(source, cnc.controller, cnc.start, cnc.binding.travel);
    for (diagnostic in parsed.diagnostics) if (diagnostic.severity
      == cnckit.CncDiagnostic.CncSeverity.Error) throw diagnostic.toString();
    var lowered = ToolpathMotion.lower(parsed.program, cnc.binding);
    if (lowered.program == null) throw "G-code contains no executable motion or barrier";
    return lowered.program;
  }

  public static function compileCnc(
    binding:ToolpathMotionBinding,
    cnc:MotionCncRig,
    source:String,
    joints:Array<Float>,
    planId:Int64
  ):motionkit.robot.CompiledProgram {
    var parsed = CncCompiler.compileDetailed(source, cnc.controller, cnc.start, cnc.binding.travel);
    for (diagnostic in parsed.diagnostics) if (diagnostic.severity
      == cnckit.CncDiagnostic.CncSeverity.Error) throw diagnostic.toString();
    return binding.compile(parsed.program, joints, planId);
  }
  public function cncTrial(virtualDevice:Bool, ?linkLoss:Bool = false):Array<Float> {
    var blueprint = MachineKitRobotCompiler.compileGantry(
      new machinekit.gantry.Gantry(new machinekit.gantry.GantrySpec(200, 200, 200)),
      0.01,
      0.04
    );
    for (channel in ["spindle.speed",
      "spindle.direction"]) blueprint.runtime.channels.push(new ProcessChannelDeclaration(
        channel,
        ProcessEventValue.Analog(0.0)
      ));
    var options:Null<VirtualDeviceOptions> = null;
    var wired:Null<DeviceBinding> = null;
    if (virtualDevice) {
      options = new VirtualDeviceOptions();
      var inputs:Array<robotkit.device.DeviceInput> = [];
      for (contact in blueprint.model.switches) {
        var drives:Array<robotkit.model.Actuator> = [];
        for (actuator in blueprint.model.actuators) switch actuator.transmission {
          case SimpleTransmission(shaft, _, _) if (shaft == contact.driveJoint):
            drives.push(actuator);
          case _:
        }
        if (drives.length != 1) throw "CNC switch needs one physical drive side: " + contact.id;
        inputs.push(new robotkit.device.DeviceInput(inputs.length, contact.id, drives[0].id, false));
      }
      wired = DeviceBinding.bind(
        blueprint.model,
        new DeviceLayout(DeviceLayout.forActuators(blueprint.model, 2).channels, inputs),
        options.stepTickHz
      );
      options.actuators = wired.virtualActuators();
      options.inputs = wired.virtualInputs();
    }
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime, null, options);
    if (virtualDevice) for (tick in 1...21) simulationHarness.step(Int64.ofInt(tick));
    var robot = new SimulatedRobot(
      "cnc-gantry",
      runtime,
      blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]
    );
    if (virtualDevice) {
      var sides = robotkit.device.DeviceHomingSides.install(runtime, blueprint.runtime, cast wired, 0.01);
      var homing = new motionkit.robot.MotionSystem(robot, blueprint);
      homing.configureRuntimeHoming(runtime, () -> {}, sides);
      homing.home();
      var ticks = 0;
      while (homing.homingStatus() != "Complete" && ticks++ < 60000) {
        simulationHarness.step();
        homing.update(0.01);
      }
      check(homing.homingStatus() == "Complete", "CNC device establishes switch references");
      var homed = robot.snapshot().positions.toArray();
      for (axis in blueprint.axes) {
        var index = -1;
        for (jointIndex in 0...blueprint.model.joints.length) if (blueprint.model.joints[jointIndex].id
          == axis.id) index = jointIndex;
        check(
          index >= 0 && homed[index] >= axis.lowerLimit && homed[index] <= axis.upperLimit,
          'CNC home on ${axis.id} stays within its soft travel: ${index < 0 ? "missing axis" : homed[index]}'
        );
      }
    } else homeGantryFixture(blueprint, runtime, simulationHarness, robot);
    var cnc = new MotionCncRig("work", "x", "y", "z", 0.01, null, 0.001);
    var state = runtime.snapshot();
    var binding = new ToolpathMotionBinding(cnc.binding, blueprint, state.modelRevision, state.calibrationRevision);
    var program = cncProgram(cnc, "G21 G90 G17\nS12000 M3\nG0 X10 Y10\nF600 G3 X20 Y20 I0 J10\nM5\nM2\n");
    var motion = new ManipulatorMotion(
      robot,
      binding.compiler,
      function(channel) return channel == "spindle.at_speed" ? EventValue.Digital(true) : null,
      function() return runtime.pollEvents()
    );
    motion.run(program);
    var afterHome = robot.snapshot();
    check(
      motion.running,
      'CNC program starts: ${motion.failure}; observed=${afterHome.positions.toArray()}, '
      + 'setpoints=${afterHome.setpointPositions.toArray()}'
    );
    var trace:Array<Float> = [];
    var holdIssued = false, holdTicks = 0, linkCut = false;
    for (tick in 0...3000) {
      if (!linkCut) motion.update(0.01);
      simulationHarness.step();
      var q = robot.snapshot().positions.toArray();
      trace.push(q[0]);
      trace.push(q[1]);
      var projection = Math.max(0.0, Math.min(1.0, (q[0] + q[1]) / 0.02));
      var lineError = Math.sqrt((q[0]
        - projection * 0.01) * (q[0] - projection * 0.01) + (q[1] - projection * 0.01) * (q[1] - projection * 0.01));
      var radius = Math.sqrt((q[0] - 0.01) * (q[0] - 0.01) + (q[1] - 0.02) * (q[1] - 0.02));
      var arcError = q[0] >= 0.01 && q[1] <= 0.02 ? Math.abs(radius - 0.01) : Math.min(
        Math.sqrt((q[0] - 0.01) * (q[0] - 0.01) + (q[1] - 0.01) * (q[1] - 0.01)),
        Math.sqrt((q[0] - 0.02) * (q[0] - 0.02) + (q[1] - 0.02) * (q[1] - 0.02))
      );
      check(
        Math.min(lineError, arcError) <= cnc.binding.positionTolerance + 1e-5,
        "CNC recorded position stays on the authored rapid or arc"
      );
      if (!holdIssued && !linkCut && q[0] > 0.0105 && q[1] > 0.0101) {
        if (linkLoss) {
          simulation.cutVirtualDeviceLink(0, true);
          linkCut = true;
        } else {
          motion.hold();
          holdIssued = true;
        }
      }
      if (holdIssued && holdTicks++ == 35) motion.resume();
      if ((holdIssued || linkCut) && q[0] > 0.0105 && q[1] > 0.0101) {
        var radial = Math.sqrt((q[0] - 0.01) * (q[0] - 0.01) + (q[1] - 0.02) * (q[1] - 0.02));
        check(Math.abs(radial - 0.01) <= 0.001, "CNC feed hold and resume stay on the programmed arc");
      }
      if (linkCut && robot.fault() != null) break;
      if (!motion.running) break;
    }
    if (linkLoss) {
      check(linkCut, "CNC link was cut during the programmed arc");
      check(robot.fault() != null, "CNC device latches link-loss fault");
      var previous = robot.snapshot().positions.toArray();
      var quiet = 0;
      for (extra in 0...300) {
        simulationHarness.step();
        var current = robot.snapshot().positions.toArray();
        if (Math.abs(current[0] - previous[0]) < 1e-6 && Math.abs(current[1] - previous[1]) < 1e-6) quiet++;
        else quiet = 0;
        previous = current;
        if (quiet >= 20) break;
      }
      check(quiet >= 20, "CNC link loss reaches a controlled stop");
      check(
        previous[0] <= 0.02 + cnc.binding.positionTolerance && previous[1] <= 0.02 + cnc.binding.positionTolerance,
        "CNC link-loss stop stays within the programmed axis bounds"
      );
      simulationHarness.dispose();
      return trace;
    }
    check(motion.completed, 'CNC program completes: ${motion.failure}');
    check(holdIssued, "CNC feed hold was issued during the arc");
    near(robot.snapshot().positions.get(0), 0.02, "CNC finishes X", 2e-4);
    near(robot.snapshot().positions.get(1), 0.02, "CNC finishes Y", 2e-4);
    if (virtualDevice) for (extra in 0...20) simulationHarness.step();
    var events = motion.firedEvents();
    check(Lambda.exists(events, function(event) return event.channel == "spindle.speed" && switch event.value {
      case ProcessEventValue.Analog(value):
        value == 12000.0;
      case _:
        false;
    }
    ), "CNC spindle start fires");
    check(Lambda.exists(events, function(event) return event.channel == "spindle.speed" && switch event.value {
      case ProcessEventValue.Analog(value):
        value == 0.0;
      case _:
        false;
    }
    ), "CNC spindle stop fires");
    simulationHarness.dispose();
    return trace;
  }

}
