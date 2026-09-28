import haxe.Int64;
import haxe.io.Bytes;
import cnckit.CncMachine;
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
import motionkit.event.TimedEvent;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.OpwKinematics;
import motionkit.robot.AxisKinematics;
import toolpathkit.motion.MachineBinding;
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
import motionkit.robot.ToolpathPosePath;
import robotkit.process.Toolpath;
import robotkit.process.ToolpathPoint;
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
import motionkit.trajectory.ValidationGuarantee;
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
import robotkit.manipulation.ChainTip;
import robotkit.manipulation.KinematicChain;
import robotkit.manipulation.Manipulator;
import robotkit.runtime.Simulation;
import robotkit.runtime.VirtualDeviceOptions;
import robotkit.runtime.VirtualActuatorOptions;
import robotkit.runtime.RobotRuntimeError;
import robotkit.runtime.RobotRuntimeCompiler;
import RobotKitRuntime;
import robotkit.world.RecordingRobot;
import robotkit.world.ReplayRobot;
import robotkit.world.RobotRecording;
import robotkit.world.SimulatedRobot;
import robotkit.world.RobotCommand;
import robotkit.world.Robot;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotDescription;
import robotkit.world.RobotFault;
import robotkit.world.RobotId;
import robotkit.world.RobotSnapshot;
import robotkit.world.RobotStatus;
import robotkit.world.RuntimeRobotAdapter;
import robotkit.world.SensorFrame;
import robotkit.world.StopMode;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;
import robotkit.world.TrajectorySegment;


class MotionKitTestSupport {
  public static function cncBinding(cnc:CncMachine,
      blueprint:MotionSystemBlueprint):ToolpathMotionBinding {
    var machine = new MachineBinding(cnc.frameId, cnc.xAxisId, cnc.yAxisId,
      cnc.zAxisId, cnc.rapidSpeed, cnc.initialPosition,
      cnc.positionTolerance, cnc.orientationTolerance,
      cnc.maxBlendTurnAngleRadians);
    for (code in 54...60)
      machine.setSetupOffset(cnc.controller.setupId(code), cnc.controller.workOffset(code));
    if (cnc.travelLower != null && cnc.travelUpper != null)
      machine.setTravelEnvelope(cnc.travelLower, cnc.travelUpper);
    var binding = new ToolpathMotionBinding(machine, blueprint);
    cnc.setTravelEnvelope(machine.travelLower, machine.travelUpper);
    return binding;
  }

  public static function cncProgram(cnc:CncMachine, source:String):MotionProgram {
    var parsed = new CncCompiler(cnc).compileDetailed(source);
    for (diagnostic in parsed.diagnostics)
      if (diagnostic.severity == cnckit.CncDiagnostic.CncSeverity.Error)
        throw diagnostic.toString();
    var machine = new MachineBinding(cnc.frameId, cnc.xAxisId, cnc.yAxisId,
      cnc.zAxisId, cnc.rapidSpeed, cnc.initialPosition,
      cnc.positionTolerance, cnc.orientationTolerance,
      cnc.maxBlendTurnAngleRadians);
    for (code in 54...60)
      machine.setSetupOffset(cnc.controller.setupId(code), cnc.controller.workOffset(code));
    if (cnc.travelLower != null && cnc.travelUpper != null)
      machine.setTravelEnvelope(cnc.travelLower, cnc.travelUpper);
    var lowered = ToolpathMotion.lower(parsed.ops, machine);
    if (lowered.program == null) throw "G-code contains no executable motion or barrier";
    return lowered.program;
  }

  public static function compileCnc(binding:ToolpathMotionBinding,
      cnc:CncMachine, source:String, joints:Array<Float>,
      planId:Int64):motionkit.robot.CompiledProgram {
    var parsed = new CncCompiler(cnc).compileDetailed(source);
    for (diagnostic in parsed.diagnostics)
      if (diagnostic.severity == cnckit.CncDiagnostic.CncSeverity.Error)
        throw diagnostic.toString();
    return binding.compile(parsed.ops, joints, planId);
  }
  public static var assertions:Int = 0;
  public function new() {}

  public function buildContractArmFixture():{model:RobotModel, chain:KinematicChain} {
    var model = new RobotModel("motionkit-contract-arm");
    var links = [for (name in ["base", "shoulder", "upper-arm", "forearm",
      "wrist-1", "wrist-2", "wrist-3"]) model.addLink(new Link(name))];
    var offsets = [[0.0, 0.0, 0.089159], [0.0, 0.13585, 0.0],
      [0.0, -0.1197, 0.425], [0.0, 0.0, 0.39225],
      [0.0, 0.10915, 0.0], [0.0, 0.0, 0.09465]];
    var axes = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
      [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]];
    for (joint in 0...6) {
      var value = model.addJoint(new Joint('joint-$joint', JointType.Revolute,
        links[joint], links[joint + 1]));
      value.parentFramePosition = offsets[joint];
      value.axis = axes[joint];
      value.limits.lower = -2.0 * Math.PI;
      value.limits.upper = 2.0 * Math.PI;
    }
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = [0.0, 0.0823, 0.0];
    return {model: model,
      chain: new KinematicChain(model, links[0].id, ChainTip.Frame(flange.id))};
  }

  public function poseRotationDelta(from:Pose3, to:Pose3, scale:Float):Array<Float> {
    var x = to.qw * -from.qx + to.qx * from.qw + to.qy * -from.qz - to.qz * -from.qy;
    var y = to.qw * -from.qy - to.qx * -from.qz + to.qy * from.qw + to.qz * -from.qx;
    var z = to.qw * -from.qz + to.qx * -from.qy - to.qy * -from.qx + to.qz * from.qw;
    var w = to.qw * from.qw - to.qx * -from.qx - to.qy * -from.qy - to.qz * -from.qz;
    if (w < 0.0) { x = -x; y = -y; z = -z; w = -w; }
    var sinHalf = Math.sqrt(x * x + y * y + z * z);
    if (sinHalf < 1e-12) return [0.0, 0.0, 0.0];
    var angleScale = 2.0 * Math.atan2(sinHalf, w) * scale / sinHalf;
    return [x * angleScale, y * angleScale, z * angleScale];
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
      for (index in 0...blueprint.model.actuators.length) {
        var actuator = blueprint.model.actuators[index];
        switch actuator.transmission {
          case SimpleTransmission(_, ratio, offset):
            options.actuators.push(new VirtualActuatorOptions(actuator.id,
              index, ratio, offset, 3200.0 / (2.0 * Math.PI),
              0.01 * Math.abs(ratio), 2));
        }
      }
    }
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime, null, options);
    if (virtualDevice) for (tick in 1...21) simulation.step(Int64.ofInt(tick));
    var robot = new SimulatedRobot("cnc-gantry", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var cnc = new CncMachine("work", "x", "y", "z", 0.01,
      null, 0.001);
    var binding = cncBinding(cnc, blueprint);
    var program = cncProgram(cnc,
      "G21 G90 G17\nS12000 M3\nG0 X10 Y10\nF600 G3 X20 Y20 I0 J10\nM5\nM2\n");
    var motion = new ManipulatorMotion(robot, binding.compiler,
      function(_) return null, function() return runtime.pollEvents());
    motion.run(program);
    check(motion.running, 'CNC program starts: ${motion.failure}');
    var trace:Array<Float> = [];
    var holdIssued = false, holdTicks = 0, linkCut = false;
    for (tick in 0...3000) {
      if (!linkCut) motion.update(0.01);
      simulation.step(Int64.ofInt(virtualDevice ? tick + 21 : tick));
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
      check(Math.min(lineError, arcError) <= cnc.positionTolerance + 1e-5,
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
        simulation.step(Int64.ofInt(4000 + extra));
        var current = robot.snapshot().positions.toArray();
        if (Math.abs(current[0] - previous[0]) < 1e-6 &&
            Math.abs(current[1] - previous[1]) < 1e-6) quiet++;
        else quiet = 0;
        previous = current;
        if (quiet >= 20) break;
      }
      check(quiet >= 20, "CNC link loss reaches a controlled stop");
      check(previous[0] <= 0.02 + cnc.positionTolerance &&
        previous[1] <= 0.02 + cnc.positionTolerance,
        "CNC link-loss stop stays within the programmed axis bounds");
      simulation.dispose();
      return trace;
    }
    check(motion.completed, 'CNC program completes: ${motion.failure}');
    check(holdIssued, "CNC feed hold was issued during the arc");
    near(robot.snapshot().positions.get(0), 0.02, "CNC finishes X", 2e-4);
    near(robot.snapshot().positions.get(1), 0.02, "CNC finishes Y", 2e-4);
    if (virtualDevice) for (extra in 0...20)
      simulation.step(Int64.ofInt(4000 + extra));
    var events = motion.firedEvents();
    check(Lambda.exists(events, function(event) return event.channel == "spindle.speed" &&
      switch event.value { case ProcessEventValue.Analog(value): value == 12000.0;
        case _: false; }), "CNC spindle start fires");
    check(Lambda.exists(events, function(event) return event.channel == "spindle.speed" &&
      switch event.value { case ProcessEventValue.Analog(value): value == 0.0;
        case _: false; }), "CNC spindle stop fires");
    simulation.dispose();
    return trace;
  }

  public function runMotion(machine:MotionSystem, simulation:Simulation):Void {
    var tick = 0;
    while (machine.isMoving()) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
      if (tick > 2000) {
        var snapshot = machine.robot.snapshot();
        throw 'MotionKit trajectory did not complete: safety=${snapshot.safety} fault=${snapshot.faultCode} session=${snapshot.sessionState} active=${snapshot.trajectoryActive} queue=${snapshot.trajectoryQueueDepth} time=${snapshot.trajectoryTimeNs} duration=${snapshot.trajectoryDurationNs} committed=${snapshot.committedUntilNs}';
      }
    }
    for (_ in 0...4) simulation.step(Int64.ofInt(tick++));
  }

  /**
   * Starts a streamed x move, injects an event at eventTick, optionally
   * resumes once the machine has come to rest, and runs until everything has
   * settled. Returns the observed x position after every tick.
   */
  public function gantryTrial(queueSupport:Bool, eventTick:Int, event:MotionSystem -> Void,
      resumeAfterStop:Bool, ?begin:MotionSystem -> Void):Array<Float> {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 60), new LinearAxis(23, 10, 40), 0.1, 0.4);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new RuntimeRobotAdapter("limits", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name], false, false,
      "simulated runtime fault", queueSupport);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    if (begin == null) machine.moveAxes([new AxisTarget("x", 0.15)], new MotionOptions(0.05, 0.4));
    else begin(machine);
    var positions:Array<Float> = [];
    var tick = 0;
    var stillTicks = 0;
    function step():Void {
      machine.update();
      try simulation.step(Int64.ofInt(tick++)) catch (error:Dynamic)
        throw 'gantry trial (queue $queueSupport, event at $eventTick) failed at tick $tick: $error';
      var position = robot.snapshot().positions.get(0);
      stillTicks = positions.length > 0 &&
        Math.abs(position - positions[positions.length - 1]) < 1e-12 ? stillTicks + 1 : 0;
      positions.push(position);
      if (tick > 3000) throw 'gantry trial at tick $eventTick did not settle';
    }
    for (_ in 0...eventTick) step();
    event(machine);
    if (resumeAfterStop) {
      stillTicks = 0;
      while (stillTicks < 3) step();
      machine.resume();
    }
    stillTicks = 0;
    while (machine.isMoving() || stillTicks < 3) step();
    simulation.dispose();
    return positions;
  }

  public function peakSecondDifference(positions:Array<Float>):Float {
    var peak = 0.0;
    for (index in 2...positions.length)
      peak = Math.max(peak, Math.abs(positions[index] - 2.0 * positions[index - 1] +
        positions[index - 2]) / (0.01 * 0.01));
    return peak;
  }

  /**
   * Like gantryTrial, over any rig, recording every joint each tick: begin a
   * motion, inject an event at eventTick, optionally resume once at rest, and
   * run until settled.
   */
  public function rigTrial(rig:TrialRig, eventTick:Int, event:MotionSystem -> Void,
      resumeAfterStop:Bool, begin:MotionSystem -> Void):Array<Array<Float>> {
    var machine = rig.machine;
    begin(machine);
    var positions:Array<Array<Float>> = [];
    var tick = 0;
    var stillTicks = 0;
    function step():Void {
      machine.update();
      rig.simulation.step(Int64.ofInt(tick++));
      var current = rig.robot.snapshot().positions.toArray();
      var still = positions.length > 0;
      if (still)
        for (joint in 0...current.length)
          if (Math.abs(current[joint] - positions[positions.length - 1][joint]) >= 1e-12) still = false;
      stillTicks = still ? stillTicks + 1 : 0;
      positions.push(current);
      if (tick > 4000) throw 'rig trial at tick $eventTick did not settle';
    }
    for (_ in 0...eventTick) step();
    event(machine);
    if (resumeAfterStop) {
      stillTicks = 0;
      while (stillTicks < 3) step();
      machine.resume();
    }
    stillTicks = 0;
    while (machine.isMoving() || stillTicks < 3) step();
    rig.simulation.dispose();
    return positions;
  }

  public function jointPeak(positions:Array<Array<Float>>, joint:Int):Float {
    var peak = 0.0;
    for (index in 2...positions.length)
      peak = Math.max(peak, Math.abs(positions[index][joint] - 2.0 * positions[index - 1][joint] +
        positions[index - 2][joint]) / (0.01 * 0.01));
    return peak;
  }

  public function gantryRig(queueSupport:Bool):TrialRig {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 60), new LinearAxis(23, 10, 40), 0.1, 0.4);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new RuntimeRobotAdapter("rig", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name], false, false,
      "simulated runtime fault", queueSupport);
    return new TrialRig(MotionSystem.fromBlueprint(robot, blueprint), simulation, robot);
  }

  public function segmentDistance(x:Float, y:Float, ax:Float, ay:Float, bx:Float, by:Float):Float {
    var dx = bx - ax, dy = by - ay;
    var alpha = Math.max(0.0, Math.min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)));
    var px = ax + alpha * dx - x, py = ay + alpha * dy - y;
    return Math.sqrt(px * px + py * py);
  }

  public function dualMotorRig(queueSupport:Bool):TrialRig {
    // The second motor is geared 2:1 and mounted reversed, so its joint moves
    // twice as far, the other way, with twice the speed and acceleration.
    var model = new RobotModel("dual-motor-geared");
    var base = model.addLink(new Link("gantry.base"));
    var left = model.addLink(new Link("gantry.left"));
    var right = model.addLink(new Link("gantry.right"));
    var leftJoint = model.addJoint(new Joint("x.left", JointType.Prismatic, base, left, "x.left"));
    var rightJoint = model.addJoint(new Joint("x.right", JointType.Prismatic, left, right, "x.right"));
    leftJoint.limits.lower = 0.0;
    leftJoint.limits.upper = 0.08;
    leftJoint.limits.velocity = 0.1;
    leftJoint.limits.maxAcceleration = 0.4;
    rightJoint.limits.lower = -0.16;
    rightJoint.limits.upper = 0.0;
    rightJoint.limits.velocity = 0.2;
    rightJoint.limits.maxAcceleration = 0.8;
    var blueprint = MotionSystemBlueprint.fromRobotModel(model, [
      new MotionAxisBlueprint("x", ["x.left", "x.right"], 0.0, 0.08, 0.08, 0.4, 0.0,
        [1.0, -2.0], [0.0, 0.0])
    ]);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var recording = new RobotRecording();
    var robot = new RecordingRobot(new RuntimeRobotAdapter("dual-motor-geared", runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name], false, false,
      "simulated runtime fault", queueSupport), recording);
    return new TrialRig(MotionSystem.fromBlueprint(robot, blueprint), simulation, robot, recording);
  }

  /** Unwraps a move that started at once because the machine was at rest. */
  public function planned(value:Null<Trajectory>):Trajectory {
    if (value == null) throw "Move was deferred behind a stop but was expected to start at once";
    return cast value;
  }

  public function trajectoryStates(value:Trajectory):Array<motionkit.trajectory.TrajectoryState> {
    var result = [];
    var duration = value.durationSeconds();
    for (index in 0...101)
      result.push(value.evaluate(duration * index / 100.0));
    return result;
  }

  public function peakChordAcceleration(value:Trajectory, joint:Int):Float {
    var peak = 0.0;
    for (index in 0...501)
      peak = Math.max(peak, Math.abs(value.evaluate(
        value.durationSeconds() * index / 500.0).accelerations[joint]));
    return peak;
  }

  public function check(value:Bool, message:String):Void {
    if (!value) throw message;
    MotionKitTestSupport.assertions++;
  }

  public function near(actual:Float, expected:Float, message:String,
      tolerance:Float = 1e-6):Void {
    check(Math.abs(actual - expected) <= tolerance * Math.max(1.0, Math.abs(expected)),
      '$message: $actual != $expected');
  }

  public function throws(action:Void -> Void, message:String):Void {
    var didThrow = false;
    try action() catch (_:Dynamic) didThrow = true;
    check(didThrow, message);
  }}

/** Deterministic IK branch switch at a synthetic wrist singularity. */
class WristBranchSolver implements KinematicsSolver {
  final jump:Bool;
  public function new(?jump:Bool = true) this.jump = jump;
  public function jointCount():Int return 6;
  public function forward(q:Array<Float>):Pose3 return new Pose3(q[0]);
  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance):Null<Array<Float>> {
    var q = seed.copy();
    q[0] = target.x;
    q[4] = !jump || target.x < 0.5 ? 0.1 : -2.0;
    return q;
  }
  public function sampleCandidates(target:Pose3, maxCount:Int,
      tolerance:IkTolerance):Array<Array<Float>>
    return [solvePose(target, [for (_ in 0...6) 0.0], tolerance)];
  public function solveDifferential(q:Array<Float>, twist:Twist6):Null<Array<Float>>
    return [for (_ in 0...6) 0.0];
}

class PlanarSolver implements KinematicsSolver {
  public function new() {}
  public function jointCount():Int return 6;
  public function forward(q:Array<Float>):Pose3 return new Pose3(q[0], q[1]);
  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance):Null<Array<Float>> {
    var q = seed.copy();
    q[0] = target.x; q[1] = target.y;
    return q;
  }
  public function sampleCandidates(target:Pose3, maxCount:Int,
      tolerance:IkTolerance):Array<Array<Float>>
    return [solvePose(target, [for (_ in 0...6) 0.0], tolerance)];
  public function solveDifferential(q:Array<Float>, twist:Twist6):Null<Array<Float>>
    return [twist.linearX, twist.linearY, 0.0, 0.0, 0.0, 0.0];
}

/** Virtual robot probe for state transitions and runtime command assertions. */
class SessionTransitionRobot implements Robot {
  public final inner:Robot;
  public final commands:Array<RobotCommand> = [];
  public var stops:Int = 0;
  public var faultOverride:Int = 0;
  public var rejectNext:Bool = false;

  public function new(inner:Robot) this.inner = inner;
  public function id():RobotId return inner.id();
  public function status():RobotStatus return inner.status();
  public function description():RobotDescription return inner.description();
  public function capabilities():RobotCapabilities return inner.capabilities();
  public function snapshot():RobotSnapshot {
    var value = inner.snapshot();
    if (faultOverride == 0) return value;
    return new RobotSnapshot(value.id, value.sourceSequence, value.sourceTimestampNs,
      value.positions.toArray(), value.velocities.toArray(), value.efforts.toArray(),
      value.mode, faultOverride, value.receivedTimestampNs, value.sensors.toArray(),
      value.sourceClockId, value.receivedClockId, value.safety,
      value.trajectoryQueueDepth, value.trajectoryActive, value.trajectoryTimeNs,
      value.trajectoryDurationNs, value.trajectoryTag, value.trajectoryTagTimeNs,
      value.sessionState, value.activePlanId, value.committedUntilNs,
      value.queueEndTimeNs);
  }
  public function sensors():Array<SensorFrame> return inner.sensors();
  public function fault():Null<RobotFault> return inner.fault();
  public function submit(command:RobotCommand):Void {
    if (rejectNext) {
      rejectNext = false;
      throw new RobotRuntimeError(RobotKitRuntimeConstants.RK_ERROR_INVALID_STATE,
        "runtime.submitPlan");
    }
    inner.submit(command);
    commands.push(command);
  }
  public function stop(mode:StopMode):Void {
    inner.stop(mode);
    stops++;
  }
  public function resetSafety():Void inner.resetSafety();
  public function setChangeListener(listener:Null<RobotId -> Void>):Void
    inner.setChangeListener(listener);
  public function close():Void inner.close();

  public function planCount():Int return commandCount("plan");
  public function commandCount(kind:String):Int {
    var count = 0;
    for (command in commands) switch command {
      case ExecutionPlan(_): if (kind == "plan") count++;
      case Resume: if (kind == "resume") count++;
      case Hold: if (kind == "hold") count++;
      case Abort: if (kind == "abort") count++;
      case _:
    }
    return count;
  }
}

class SessionTransitionRig {
  public final simulation:Simulation;
  public final robot:SessionTransitionRobot;
  public final machine:MotionSystem;
  public final options:MotionOptions = new MotionOptions(0.05, 0.2);
  var tick:Int = 0;

  public function new(id:String) {
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(
      new LinearAxis(23, 10, 80), "x", 0.08, 0.4);
    simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    robot = new SessionTransitionRobot(new SimulatedRobot(id, runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]));
    machine = MotionSystem.fromBlueprint(robot, blueprint);
  }

  public function start():Void {
    machine.moveAxes([new AxisTarget("x", 0.07)], options);
    advance(5);
  }

  public function advance(count:Int):Void {
    for (_ in 0...count) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
    }
  }

  public function settleStop():Void {
    var steps = 0;
    while (machine.sessionState() != SessionState.Idle &&
        machine.sessionState() != SessionState.Running) {
      advance(1);
      if (++steps > 500) throw "session stop did not settle";
    }
  }

  public function settleHold():Void {
    var steps = 0;
    while (machine.sessionState() == SessionState.Holding) {
      advance(1);
      if (++steps > 500) throw "session hold did not settle";
    }
  }

  public function dispose():Void simulation.dispose();
}

/** Robot wrapper that can hold submitted commands back, to simulate transport delay. */
class LaggingRobot implements Robot {
  public var lagging:Bool = false;
  public var lateRejections:Int = 0;
  public var replacementAttempts:Int = 0;
  public var lastReplacementLeadNs:Int64 = Int64.ofInt(0);
  final inner:Robot;
  var heldCommands:Array<RobotCommand> = [];

  public function new(inner:Robot) this.inner = inner;

  public function release():Void {
    lagging = false;
    for (command in heldCommands) inner.submit(command);
    heldCommands = [];
  }

  public function id():RobotId return inner.id();
  public function status():RobotStatus return inner.status();
  public function description():RobotDescription return inner.description();
  public function capabilities():RobotCapabilities return inner.capabilities();
  public function snapshot():RobotSnapshot return inner.snapshot();
  public function sensors():Array<SensorFrame> return inner.sensors();
  public function fault():Null<RobotFault> return inner.fault();
  public function submit(command:RobotCommand):Void {
    switch command {
      case ExecutionPlan(plan):
        if (Int64.compare(plan.replaceAfterPlanId, Int64.ofInt(0)) != 0) {
          replacementAttempts++;
          lastReplacementLeadNs = Int64.sub(plan.replaceAfterTimeNs,
            inner.snapshot().committedUntilNs);
          if (lateRejections > 0) {
            lateRejections--;
            throw new RobotRuntimeError(RobotKitRuntimeConstants.RK_ERROR_INVALID_STATE,
              "runtime.submitPlan");
          }
        }
      case _:
    }
    if (lagging) heldCommands.push(command);
    else inner.submit(command);
  }
  public function stop(mode:StopMode):Void inner.stop(mode);
  public function resetSafety():Void inner.resetSafety();
  public function setChangeListener(listener:Null < RobotId -> Void >):Void
    inner.setChangeListener(listener);
  public function close():Void inner.close();
}

/** Virtual arm with an injectable snapshot fault for lifecycle transitions. */
class FaultingArmRobot extends SimulatedRobot {
  public var faultOverride:Int = 0;
  public final commands:Array<RobotCommand> = [];

  public function new(id:RobotId, runtime:robotkit.runtime.RobotRuntime,
      name:String, links:Array<String>, joints:Array<String>)
    super(id, runtime, name, links, joints);

  override public function submit(command:RobotCommand):Void {
    super.submit(command);
    commands.push(command);
  }

  public function commandCount(kind:String):Int {
    var count = 0;
    for (command in commands) if (switch command {
      case Hold: kind == "hold";
      case Resume: kind == "resume";
      case Abort: kind == "abort";
      case ExecutionPlan(_): kind == "plan";
      case _: false;
    }) count++;
    return count;
  }

  override public function snapshot():RobotSnapshot {
    var value = super.snapshot();
    if (faultOverride == 0) return value;
    return new RobotSnapshot(value.id, value.sourceSequence,
      value.sourceTimestampNs, value.positions.toArray(),
      value.velocities.toArray(), value.efforts.toArray(), value.mode,
      faultOverride, value.receivedTimestampNs, value.sensors.toArray(),
      value.sourceClockId, value.receivedClockId, value.safety,
      value.trajectoryQueueDepth, value.trajectoryActive,
      value.trajectoryTimeNs, value.trajectoryDurationNs,
      value.trajectoryTag, value.trajectoryTagTimeNs, value.sessionState,
      value.activePlanId, value.committedUntilNs, value.queueEndTimeNs);
  }
}

/** A simulated machine for sweep trials. */
class TrialRig {
  public final machine:MotionSystem;
  public final simulation:Simulation;
  public final robot:Robot;
  public final recording:Null<RobotRecording>;

  public function new(machine:MotionSystem, simulation:Simulation, robot:Robot,
      ?recording:RobotRecording) {
    this.machine = machine;
    this.simulation = simulation;
    this.robot = robot;
    this.recording = recording;
  }
}
