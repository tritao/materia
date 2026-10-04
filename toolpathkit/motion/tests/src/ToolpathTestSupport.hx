import haxe.Int64;
import haxe.io.Bytes;
import cnckit.CncCompiler;
import machinekit.assembly.LinearAxis;
import cadkit.modeling.AssemblyModel;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import materia.assembly.AssemblyFrames;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MotionOptions;
import motionkit.Pose;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.event.ChannelDeclaration;
import motionkit.event.ChannelKind;
import motionkit.event.EventValue;
import motionkit.event.HoldPolicy;
import motionkit.event.PathEvent;
import motionkit.event.TimedEvent;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.OpwKinematics;
import motionkit.robot.AxisKinematics;
import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.ToolpathMotionBinding;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.robot.PathConfigurationSelector;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MachineKitRobotCompiler;
import motionkit.robot.MotionSystem;
import motionkit.robot.SessionState;
import motionkit.robot.StopDisposition;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.path.ArcSegment;
import motionkit.path.CornerBlender;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.path.NativeJointPath;
import motionkit.path.PathTimeLaw;
import motionkit.path.PathTimeLaw.PathTimeStage;
import motionkit.path.PosePath;
import motionkit.path.PoseLine;
import motionkit.path.PoseArc;
import motionkit.path.PoseWaypoint;
import motionkit.path.OrientationPolicy;
import processkit.motion.ToolpathPosePath;
import processkit.path.Toolpath;
import processkit.path.ToolpathPoint;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import motionkit.planner.PathPlanningOptions;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingBackend;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.SimplePathTiming;
import motionkit.planner.ToppraPathTiming;
import motionkit.planner.BindingConstraint.BindingConstraintKind;
import motionkit.program.Blend;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.trajectory.MotionLimits;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ExecutionPlan;
import trajectorykit.validation.ValidationGuarantee;
import motionkit.trajectory.PlanLimitError;
import motionkit.trajectory.ValidationLimits;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.Frame;
import robotkit.model.RobotModel;
import robotkit.model.Actuator;
import robotkit.model.Transmission;
import robotkit.model.JointCoupling;
import robotkit.manipulation.Manipulator;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.VirtualDeviceOptions;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import robotkit.runtime.VirtualActuatorOptions;
import robotkit.runtime.RobotRuntimeError;
import robotkit.runtime.RobotRuntimeCompiler;
import RobotKitRuntime;
import robotkit.recording.RecordingRobot;
import robotkit.recording.ReplayRobot;
import robotkit.recording.RobotRecording;
import robotkit.simulation.SimulatedRobot;
import robotkit.core.RobotCommand;
import robotkit.core.Robot;
import robotkit.core.RobotCapabilities;
import robotkit.core.RobotDescription;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;
import robotkit.core.RobotStatus;
import robotkit.runtime.RuntimeRobotAdapter;
import robotkit.core.SensorFrame;
import robotkit.core.StopMode;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;
import robotkit.execution.TrajectorySegment;


class ToolpathTestSupport extends MotionKitTestSupport {
  public static function cncBinding(cnc:MotionCncRig,
      blueprint:MotionSystemBlueprint):ToolpathMotionBinding {
    return new ToolpathMotionBinding(cnc.binding, blueprint);
  }

  public static function cncProgram(cnc:MotionCncRig, source:String):MotionProgram {
    var parsed = CncCompiler.compileDetailed(source, cnc.controller,
      cnc.start, cnc.binding.travel);
    for (diagnostic in parsed.diagnostics)
      if (diagnostic.severity == cnckit.CncDiagnostic.CncSeverity.Error)
        throw diagnostic.toString();
    var lowered = ToolpathMotion.lower(parsed.program, cnc.binding);
    if (lowered.program == null) throw "G-code contains no executable motion or barrier";
    return lowered.program;
  }

  public static function compileCnc(binding:ToolpathMotionBinding,
      cnc:MotionCncRig, source:String, joints:Array<Float>,
      planId:Int64):motionkit.robot.CompiledProgram {
    var parsed = CncCompiler.compileDetailed(source, cnc.controller,
      cnc.start, cnc.binding.travel);
    for (diagnostic in parsed.diagnostics)
      if (diagnostic.severity == cnckit.CncDiagnostic.CncSeverity.Error)
        throw diagnostic.toString();
    return binding.compile(parsed.program, joints, planId);
  }
  public function cncTrial(virtualDevice:Bool, ?linkLoss:Bool = false):Array<Float> {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 200), 0.01, 0.04);
    for (channel in ["spindle.speed", "spindle.direction"])
      blueprint.runtime.channels.push(new ProcessChannelDeclaration(channel,
        ProcessEventValue.Analog(0.0)));
    var options:Null<VirtualDeviceOptions> = null;
    if (virtualDevice) {
      options = new VirtualDeviceOptions();
      // The motors' 200 full steps at 16 microsteps a turn, wired in model order.
      var binding = DeviceBinding.bind(blueprint.model,
        new DeviceLayout([for (index in 0...blueprint.model.actuators.length)
        new robotkit.device.DeviceChannel(index, blueprint.model.actuators[index].id, 1, 2)]), options.stepTickHz);
      options.actuators = binding.virtualActuators();
    }
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime, null, options);
    if (virtualDevice) for (tick in 1...21) simulationHarness.step(Int64.ofInt(tick));
    var robot = new SimulatedRobot("cnc-gantry", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var cnc = new MotionCncRig("work", "x", "y", "z", 0.01,
      null, 0.001);
    var binding = cncBinding(cnc, blueprint);
    var program = cncProgram(cnc,
      "G21 G90 G17\nS12000 M3\nG0 X10 Y10\nF600 G3 X20 Y20 I0 J10\nM5\nM2\n");
    var motion = new ManipulatorMotion(robot, binding.compiler,
      function(channel) return channel == "spindle.at_speed" ?
        EventValue.Digital(true) : null,
      function() return runtime.pollEvents());
    motion.run(program);
    check(motion.running, 'CNC program starts: ${motion.failure}');
    var trace:Array<Float> = [];
    var holdIssued = false, holdTicks = 0, linkCut = false;
    for (tick in 0...3000) {
      if (!linkCut) motion.update(0.01);
      simulationHarness.step(Int64.ofInt(virtualDevice ? tick + 21 : tick));
      var q = robot.snapshot().positions.toArray();
      trace.push(q[0]); trace.push(q[1]);
      var projection = Math.max(0.0, Math.min(1.0, (q[0] + q[1]) / 0.02));
      var lineError = Math.sqrt((q[0] - projection * 0.01) *
        (q[0] - projection * 0.01) + (q[1] - projection * 0.01) *
        (q[1] - projection * 0.01));
      var radius = Math.sqrt((q[0] - 0.01) * (q[0] - 0.01) +
        (q[1] - 0.02) * (q[1] - 0.02));
      var arcError = q[0] >= 0.01 && q[1] <= 0.02 ?
        Math.abs(radius - 0.01) : Math.min(
          Math.sqrt((q[0] - 0.01) * (q[0] - 0.01) +
            (q[1] - 0.01) * (q[1] - 0.01)),
          Math.sqrt((q[0] - 0.02) * (q[0] - 0.02) +
            (q[1] - 0.02) * (q[1] - 0.02)));
      check(Math.min(lineError, arcError) <= cnc.binding.positionTolerance + 1e-5,
        "CNC recorded position stays on the authored rapid or arc");
      if (!holdIssued && !linkCut && q[0] > 0.0105 && q[1] > 0.0101) {
        if (linkLoss) {
          simulation.cutVirtualDeviceLink(0, true);
          linkCut = true;
        } else {
          motion.hold(); holdIssued = true;
        }
      }
      if (holdIssued && holdTicks++ == 35) motion.resume();
      if ((holdIssued || linkCut) && q[0] > 0.0105 && q[1] > 0.0101) {
        var radial = Math.sqrt((q[0] - 0.01) * (q[0] - 0.01) +
          (q[1] - 0.02) * (q[1] - 0.02));
        check(Math.abs(radial - 0.01) <= 0.001,
          "CNC feed hold and resume stay on the programmed arc");
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
        simulationHarness.step(Int64.ofInt(4000 + extra));
        var current = robot.snapshot().positions.toArray();
        if (Math.abs(current[0] - previous[0]) < 1e-6 &&
            Math.abs(current[1] - previous[1]) < 1e-6) quiet++;
        else quiet = 0;
        previous = current;
        if (quiet >= 20) break;
      }
      check(quiet >= 20, "CNC link loss reaches a controlled stop");
      check(previous[0] <= 0.02 + cnc.binding.positionTolerance &&
        previous[1] <= 0.02 + cnc.binding.positionTolerance,
        "CNC link-loss stop stays within the programmed axis bounds");
      simulationHarness.dispose();
      return trace;
    }
    check(motion.completed, 'CNC program completes: ${motion.failure}');
    check(holdIssued, "CNC feed hold was issued during the arc");
    near(robot.snapshot().positions.get(0), 0.02, "CNC finishes X", 2e-4);
    near(robot.snapshot().positions.get(1), 0.02, "CNC finishes Y", 2e-4);
    if (virtualDevice) for (extra in 0...20)
      simulationHarness.step(Int64.ofInt(4000 + extra));
    var events = motion.firedEvents();
    check(Lambda.exists(events, function(event) return event.channel == "spindle.speed" &&
      switch event.value { case ProcessEventValue.Analog(value): value == 12000.0;
        case _: false; }), "CNC spindle start fires");
    check(Lambda.exists(events, function(event) return event.channel == "spindle.speed" &&
      switch event.value { case ProcessEventValue.Analog(value): value == 0.0;
        case _: false; }), "CNC spindle stop fires");
    simulationHarness.dispose();
    return trace;
  }

}
