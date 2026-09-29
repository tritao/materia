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
import robotkit.runtime.SimulationHarness;
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

import MotionKitTestSupport.WristBranchSolver;
import MotionKitTestSupport.PlanarSolver;
import MotionKitTestSupport.SessionTransitionRig;
import MotionKitTestSupport.LaggingRobot;
import MotionKitTestSupport.TrialRig;

class SessionTests extends MotionKitTestSupport {
  public function new() { super(); }

  public function testNormalAbortWaitsForRest():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.05, 0.2);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("abort-replacement", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var recording = new RobotRecording();
    var instrumented = new RecordingRobot(robot, recording);
    var machine = MotionSystem.fromBlueprint(instrumented, blueprint);
    var options = new MotionOptions(0.05, 0.2);
    machine.moveAxes([new AxisTarget("x", 0.07)], options);

    var tick = 0;
    for (_ in 0...5) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
    }
    check(runtime.snapshot().trajectoryActive,
      "abort replacement test starts with a running native trajectory");

    machine.abort();
    check(machine.isMoving(), "normal abort remains moving while the runtime stops");
    var replacementTargets = [new AxisTarget("x", 0.01)];
    check(machine.moveAxes(replacementTargets, options) == null,
      "move after abort is deferred until the runtime reaches rest");
    replacementTargets[0] = new AxisTarget("x", 0.04);
    var queuedTargets = [new AxisTarget("x", 0.025)];
    check(machine.queueAxes(queuedTargets, options) == null,
      "queued move waits behind the deferred replacement");
    queuedTargets[0] = new AxisTarget("x", 0.05);
    check(machine.isMoving(), "deferred move remains moving while the stop settles");

    var stopTicks = 0;
    while (runtime.snapshot().trajectoryActive ||
        runtime.snapshot().sessionState == RobotKitRuntimeConstants.RK_SESSION_STOPPING) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      stopTicks++;
      if (stopTicks > 500) throw "normal abort did not settle";
    }
    check(machine.isMoving(), "deferred replacement remains visible at runtime rest");
    var settledPosition = robot.snapshot().positions.get(0);
    var planCount = 0;
    for (command in recording.commands) switch command {
      case ExecutionPlan(_): planCount++;
      case _:
    }
    check(planCount == 1,
      "replacement plan is not submitted before the runtime reports rest");

    machine.update();
    var replacement = machine.trajectory();
    check(replacement != null, "deferred replacement starts after the stop settles");
    near(cast(replacement, Trajectory).evaluate(0.0).positions[0], settledPosition,
      "replacement starts from the position where the stop settled", 1e-5);
    near(cast(replacement, Trajectory).evaluate(
      cast(replacement, Trajectory).durationSeconds()).positions[0], 0.01,
      "deferred replacement uses targets captured before caller mutation", 1e-5);
    runMotion(machine, simulationHarness);
    near(robot.snapshot().positions.get(0), 0.025,
      "queued move uses targets captured before the caller mutates its array", 1e-5);
    simulationHarness.dispose();
  }

  /** Exercise the MotionSession transition table through a virtual runtime. */
  public function testMotionSessionTransitions():Void {
    function expect(rig:SessionTransitionRig, label:String, expected:SessionState,
        commands:Int, action:Void -> Void):Void {
      var before = rig.robot.commands.length;
      action();
      check(rig.machine.sessionState() == expected,
        '$label transitions to ${Std.string(expected)}');
      check(rig.robot.commands.length - before == commands,
        '$label sends $commands command(s) to the runtime');
    }

    var idleRig = new SessionTransitionRig("session-idle-hold");
    expect(idleRig, "idle + hold", Held, 0, () -> idleRig.machine.hold());
    expect(idleRig, "held + queue", Held, 0, () ->
      idleRig.machine.queueAxes([new AxisTarget("x", 0.02)], idleRig.options));
    expect(idleRig, "held + resume", Running, 1, () -> idleRig.machine.resume());
    idleRig.dispose();

    // Running + abort -> Stopping(Discard); a new move replaces the discard
    // request without submitting a plan until runtime rest is observed.
    var abortRig = new SessionTransitionRig("session-abort");
    abortRig.start();
    expect(abortRig, "running + abort", Stopping(Discard), 1,
      () -> abortRig.machine.abort());
    var planCount = abortRig.robot.planCount();
    expect(abortRig, "stopping + move", Stopping(Replan), 0, () -> {
      check(abortRig.machine.moveAxes([new AxisTarget("x", 0.02)], abortRig.options) == null,
        "move after abort is deferred");
    });
    check(abortRig.robot.planCount() == planCount,
      "deferred move sends no plan before rest");
    abortRig.settleStop();
    check(abortRig.machine.sessionState() == Running &&
      abortRig.robot.planCount() == planCount + 1,
      "stopping + rest starts the deferred plan");
    abortRig.dispose();

    var stopRig = new SessionTransitionRig("session-stop-hold");
    stopRig.start();
    stopRig.machine.abort();
    expect(stopRig, "stopping + hold", Stopping(Discard), 0,
      () -> stopRig.machine.hold());
    expect(stopRig, "stopping + resume", Stopping(Discard), 0,
      () -> stopRig.machine.resume());
    stopRig.settleStop();
    check(stopRig.machine.sessionState() == Idle,
      "stopping + rest discards aborted motion");
    stopRig.dispose();

    var holdRig = new SessionTransitionRig("session-hold-resume");
    holdRig.start();
    expect(holdRig, "running + hold", Holding, 1,
      () -> holdRig.machine.hold());
    expect(holdRig, "holding + resume", Holding, 0,
      () -> holdRig.machine.resume());
    holdRig.settleHold();
    check(holdRig.machine.sessionState() == Running &&
      holdRig.robot.commandCount("resume") == 1,
      "holding + rest sends one deferred resume");
    holdRig.dispose();

    var heldRig = new SessionTransitionRig("session-held-resume");
    heldRig.start();
    heldRig.machine.hold();
    heldRig.settleHold();
    check(heldRig.machine.sessionState() == Held &&
      heldRig.robot.commandCount("resume") == 0,
      "holding + rest becomes held without resuming");
    expect(heldRig, "held + resume", Running, 1,
      () -> heldRig.machine.resume());
    heldRig.dispose();

    var jogRig = new SessionTransitionRig("session-jog");
    jogRig.machine.jog("x", 0.05, 2.0);
    jogRig.advance(10);
    var beforeReplacement = jogRig.robot.planCount();
    expect(jogRig, "running + jog replacement", Running, 1, () -> {
      check(jogRig.machine.jog("x", 0.02, 1.0) != null,
        "running jog is replaced without stopping");
    });
    check(jogRig.robot.planCount() == beforeReplacement + 1,
      "jog replacement submits one plan");
    jogRig.advance(5);
    expect(jogRig, "running + continued jog", Running, 1, () -> {
      check(jogRig.machine.jog("x", 0.03, 1.0) != null,
        "jog continues after replacement");
    });
    check(jogRig.robot.stops == 0,
      "continued jog sends no stop to the runtime");
    jogRig.dispose();

    var faultRig = new SessionTransitionRig("session-fault");
    faultRig.start();
    faultRig.machine.abort();
    faultRig.robot.faultOverride = 42;
    var faultCommands = faultRig.robot.commands.length;
    throws(() -> faultRig.machine.update(),
      "snapshot fault while stopping interrupts the update");
    check(faultRig.machine.sessionState() == Faulted &&
      faultRig.robot.commands.length == faultCommands,
      "stopping + fault latches Faulted without another command");
    throws(() -> faultRig.machine.moveAxes([new AxisTarget("x", 0.01)]),
      "faulted session rejects new motion");
    faultRig.robot.faultOverride = 0;
    faultRig.machine.reset();
    check(faultRig.machine.sessionState() == Idle,
      "faulted + explicit reset returns to idle");
    faultRig.dispose();

    var rejectRig = new SessionTransitionRig("session-rejection");
    rejectRig.robot.rejectNext = true;
    throws(() -> rejectRig.machine.moveAxes([new AxisTarget("x", 0.07)], rejectRig.options),
      "rejected initial plan reports an error");
    check(rejectRig.machine.sessionState() == Faulted,
      "rejected submission latches Faulted");
    rejectRig.dispose();
  }

  public function testRuntimeSynchronizedHolding():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.2);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("runtime-synchronized-hold", runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var options = new MotionOptions(0.05, 0.2);
    var tick = 0;

    var move = planned(machine.moveAxes([new AxisTarget("x", 0.06)], options));
    var previousProgress = machine.progress();
    for (_ in 0...8) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      var progress = machine.progress();
      check(progress + 1e-9 >= previousProgress,
        "runtime-synchronized progress never moves backwards");
      previousProgress = progress;
      var snapshot = runtime.snapshot();
      if (Int64.compare(snapshot.trajectoryTag, Int64.ofInt(0)) != 0) {
        var runtimeTime = Int64.toFloat(snapshot.trajectoryTagTimeNs) /
          1000000000.0;
        check(Math.abs(progress - Math.min(1.0, runtimeTime / move.durationSeconds())) < 1e-6,
          "progress follows the runtime trajectory tag clock");
      }
    }

    for (cycle in 0...2) {
      machine.hold();
      var stopTicks = 0;
      while (runtime.snapshot().sessionState != RobotKitRuntimeConstants.RK_SESSION_HELD) {
        check(!machine.update(), "held motion does not submit host-clock samples");
        simulationHarness.step(Int64.ofInt(tick++));
        stopTicks += 1;
        if (stopTicks > 200) throw "runtime stop did not settle";
      }
      var heldPosition = robot.snapshot().positions.get(0);
      machine.resume();
      while (machine.isHolding()) {
        machine.update();
        simulationHarness.step(Int64.ofInt(tick++));
        stopTicks += 1;
        if (stopTicks > 400) throw "held motion did not resume";
      }
      near(robot.snapshot().positions.get(0), heldPosition,
        "resume starts at the runtime-reported stop position", 1e-4);
      check(machine.progress() + 1e-9 >= previousProgress,
        'progress never moves backwards across hold/resume cycle $cycle');
      previousProgress = machine.progress();
      for (_ in 0...4) {
        machine.update();
        simulationHarness.step(Int64.ofInt(tick++));
        var progress = machine.progress();
        check(progress + 1e-9 >= previousProgress,
          "progress remains monotonic after resuming");
        previousProgress = progress;
      }
    }
    while (machine.isMoving()) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      if (tick > 2000) throw "held trajectory did not complete";
    }
    check(!machine.isHolding() && machine.queueDepth() == 0,
      "repeated hold/resume leaves no buffered motion");
    near(robot.snapshot().positions.get(0), 0.06,
      "repeated hold/resume reaches the planned endpoint", 1e-5);
    simulationHarness.dispose();

    var queuedBlueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.2);
    var queuedSimulationHarness = new SimulationHarness(0.01);
    var queuedSimulation = queuedSimulationHarness.simulation;
    var queuedRuntime = queuedSimulation.addRobot(queuedBlueprint.runtime);
    var queuedRobot = new SimulatedRobot("queued-hold", queuedRuntime,
      queuedBlueprint.model.name, [for (link in queuedBlueprint.model.links) link.name],
      [for (joint in queuedBlueprint.model.joints) joint.name]);
    var recording = new RobotRecording();
    var queuedMachine = MotionSystem.fromBlueprint(new RecordingRobot(queuedRobot, recording),
      queuedBlueprint);
    queuedMachine.queueAxes([new AxisTarget("x", 0.02)], options);
    queuedMachine.queueAxes([new AxisTarget("x", 0.05)], options);
    var firstTag = switch (recording.commands[0]) {
      case RobotCommand.TrajectoryChunk(chunk): chunk.tag;
      case _: Int64.ofInt(0);
    };
    tick = 0;
    while (Int64.compare(queuedRuntime.snapshot().trajectoryTag, firstTag) == 0 ||
        Int64.compare(queuedRuntime.snapshot().trajectoryTag, Int64.ofInt(0)) == 0) {
      queuedMachine.update();
      queuedSimulationHarness.step(Int64.ofInt(tick++));
      if (tick > 2000) throw "queued trajectory did not reach its second move";
    }
    queuedMachine.hold();
    while (queuedRuntime.snapshot().sessionState != RobotKitRuntimeConstants.RK_SESSION_HELD) {
      queuedMachine.update();
      queuedSimulationHarness.step(Int64.ofInt(tick++));
      if (tick > 2200) throw "second queued trajectory did not stop";
    }
    var queuedHoldPosition = queuedRobot.snapshot().positions.get(0);
    queuedMachine.resume();
    while (queuedMachine.isMoving()) {
      queuedMachine.update();
      queuedSimulationHarness.step(Int64.ofInt(tick++));
      if (tick > 3000) throw "second queued trajectory did not resume";
    }
    check(queuedRobot.snapshot().positions.get(0) >= queuedHoldPosition - 1e-4,
      "second queued trajectory resumes from its stop path");
    near(queuedRobot.snapshot().positions.get(0), 0.05,
      "hold during a queued trajectory preserves later motion", 1e-5);
    queuedSimulationHarness.dispose();

    var squareBlueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 80), new LinearAxis(23, 10, 80),
      new LinearAxis(23, 10, 80), 0.08, 0.2);
    var squareSimulationHarness = new SimulationHarness(0.01);
    var squareSimulation = squareSimulationHarness.simulation;
    var squareRuntime = squareSimulation.addRobot(squareBlueprint.runtime);
    var squareRobot = new SimulatedRobot("square-hold", squareRuntime,
      squareBlueprint.model.name, [for (link in squareBlueprint.model.links) link.name],
      [for (joint in squareBlueprint.model.joints) joint.name]);
    var squareMachine = MotionSystem.fromBlueprint(squareRobot, squareBlueprint);
    var squarePath = squareMachine.movePath(GeometricPath.lines([
      new PathPoint(0.0, 0.0, 0.0), new PathPoint(0.02, 0.0, 0.0),
      new PathPoint(0.02, 0.02, 0.0), new PathPoint(0.0, 0.02, 0.0),
      new PathPoint(0.0, 0.0, 0.0)
    ]), PathPlanningOptions.exactStopMode(), new MotionOptions(0.04, 0.2));
    var cornerStops = 0;
    for (segment in squarePath.segments()) {
      var state = squarePath.evaluate(Int64.toFloat(segment.timeFromStartNs) * 1e-9);
      for (corner in [new PathPoint(0.02, 0.0), new PathPoint(0.02, 0.02),
          new PathPoint(0.0, 0.02)])
        if (Math.abs(state.positions[0] - corner.x) < 1e-8 &&
            Math.abs(state.positions[1] - corner.y) < 1e-8 &&
            Math.abs(state.velocities[0]) < 1e-7 && Math.abs(state.velocities[1]) < 1e-7)
          cornerStops++;
    }
    check(cornerStops >= 3, "TOPP-RA square stops at every authored corner");
    var squareReport = squareMachine.lastPathValidationReport;
    if (squareReport == null) throw "TOPP-RA path did not record validation";
    check(squareReport.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE].status ==
      MotionKitNativeConstants.MK_CHECK_PASSED &&
      squareReport.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE].method ==
      MotionKitNativeConstants.MK_CHECK_METHOD_SAMPLED,
      "TOPP-RA task-space check reports sampled path tolerance");
    check(squareReport.checks[MotionKitNativeConstants.MK_CHECK_JERK].status ==
      MotionKitNativeConstants.MK_CHECK_UNCHECKED,
      "TOPP-RA reports jerk as unchecked");
    var invalidPathRejected = false;
    try squareMachine.queuePath(null) catch (_:Dynamic) invalidPathRejected = true;
    check(invalidPathRejected && squareMachine.lastPathValidationReport == null &&
      squareMachine.lastPathPlanningDiagnostics.length == 0,
      "failed path planning clears the prior validation report and diagnostics");
    tick = 0;
    var reachedThirdLeg = false;
    while (squareMachine.isMoving()) {
      squareMachine.update();
      squareSimulationHarness.step(Int64.ofInt(tick++));
      var position = squareRobot.snapshot().positions;
      if (position.get(1) > 0.019 && position.get(0) < 0.019) {
        reachedThirdLeg = true;
        break;
      }
      if (tick > 3000) throw "square path did not reach its third leg";
    }
    check(reachedThirdLeg, "square hold test reaches the third path leg");
    squareMachine.hold();
    while (squareRuntime.snapshot().sessionState != RobotKitRuntimeConstants.RK_SESSION_HELD) {
      squareMachine.update();
      squareSimulationHarness.step(Int64.ofInt(tick++));
      if (tick > 3400) throw "square third-leg stop did not settle";
    }
    var stopped = squareRobot.snapshot().positions;
    squareMachine.resume();
    var previousX = stopped.get(0);
    while (squareMachine.isMoving()) {
      squareMachine.update();
      squareSimulationHarness.step(Int64.ofInt(tick++));
      var position = squareRobot.snapshot().positions;
      if (previousX > 0.0001 && position.get(0) > 0.0001) {
        check(Math.abs(position.get(1) - 0.02) < 0.001,
          "square resume stays on the third path leg");
        check(position.get(0) <= previousX + 1e-5,
          "square resume does not jump backwards along the third leg");
      }
      previousX = position.get(0);
      if (tick > 5000) throw "square path did not complete after resume";
    }
    near(squareRobot.snapshot().positions.get(0), 0.0,
      "square path resumes to its final X endpoint", 1e-5);
    near(squareRobot.snapshot().positions.get(1), 0.0,
      "square path resumes to its final Y endpoint", 1e-5);
    squareSimulationHarness.dispose();
  }

  /**
   * Holds at every other tick of one streamed move and checks each stop. The
   * move accelerates and brakes at the joint limit itself, so holds during
   * those phases catch a stop that adds its own deceleration on top, and
   * holds around the streaming refill points catch a stop that runs out of
   * queued path.
   */
  public function testHoldDecelerationStaysWithinLimitsThroughoutMove():Void {
    var limit = 0.4;
    var target = 0.15;
    var moveTicks = 0;
    var holdTick = 2;
    var worstAcceleration = 0.0;
    var worstTick = -1;
    while (moveTicks == 0 || holdTick < moveTicks) {
      var blueprint = MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 200),
        new LinearAxis(23, 10, 60), new LinearAxis(23, 10, 40), 0.1, limit);
      var simulationHarness = new SimulationHarness(0.01);
      var simulation = simulationHarness.simulation;
      var runtime = simulation.addRobot(blueprint.runtime);
      var robot = new SimulatedRobot("hold-sweep", runtime, blueprint.model.name,
        [for (link in blueprint.model.links) link.name],
        [for (joint in blueprint.model.joints) joint.name]);
      var machine = MotionSystem.fromBlueprint(robot, blueprint);
      var move = planned(machine.moveAxes([new AxisTarget("x", target)],
        new MotionOptions(0.05, limit)));
      if (moveTicks == 0) moveTicks = Math.ceil(move.durationSeconds() / 0.01);
      var tick = 0;
      var positions:Array<Float> = [];
      for (_ in 0...holdTick) {
        machine.update();
        simulationHarness.step(Int64.ofInt(tick++));
        positions.push(robot.snapshot().positions.get(0));
      }
      machine.hold();
      for (_ in 0...40) {
        machine.update();
        simulationHarness.step(Int64.ofInt(tick++));
        positions.push(robot.snapshot().positions.get(0));
      }
      var peak = 0.0;
      var backwards = false;
      for (index in 2...positions.length) {
        peak = Math.max(peak, Math.abs(positions[index] - 2.0 * positions[index - 1] +
          positions[index - 2]) / (0.01 * 0.01));
        if (positions[index] < positions[index - 1] - 1e-9) backwards = true;
      }
      if (peak > worstAcceleration) {
        worstAcceleration = peak;
        worstTick = holdTick;
      }
      check(!backwards, 'hold at tick $holdTick never reverses along the path');
      var last = positions[positions.length - 1];
      check(last <= target + 1e-9, 'hold at tick $holdTick stops within the planned move');
      check(Math.abs(last - positions[positions.length - 2]) < 1e-9,
        'hold at tick $holdTick comes to rest');
      var session = runtime.snapshot().sessionState;
      check(session == RobotKitRuntimeConstants.RK_SESSION_HELD ||
        (session == RobotKitRuntimeConstants.RK_SESSION_IDLE &&
          Math.abs(last - target) < 1e-5),
        'hold at tick $holdTick pauses or completes its path');
      simulationHarness.dispose();
      holdTick += 2;
    }
    check(worstAcceleration <= limit * 1.05,
      'every hold stays within the joint acceleration limit (worst ${worstAcceleration} at tick $worstTick)');
  }

  public function testImmediateMotionReplacesNativeQueue():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.4);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("replace-queue", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var recording = new RobotRecording();
    var instrumented = new RecordingRobot(robot, recording);
    var machine = MotionSystem.fromBlueprint(instrumented, blueprint);
    var options = new MotionOptions(0.05, 0.2);

    machine.moveAxes([new AxisTarget("x", 0.06)], options);
    machine.moveAxes([new AxisTarget("x", 0.01)], options);
    check(recording.commands.length == 2,
      'immediate replacement submits the replacement plan: ${recording.commands}');
    switch recording.commands[1] {
      case ExecutionPlan(_):
        check(true, "immediate replacement submits the new plan");
      case JointTargets(_, _):
        throw "immediate replacement did not submit a plan";
      case TrajectoryChunk(_):
        throw "immediate replacement submitted a legacy point chunk";
      case Hold | Resume | Abort:
        throw "immediate replacement unexpectedly submitted a lifecycle command";
    }
    runMotion(machine, simulationHarness);
    near(robot.snapshot().positions.get(0), 0.01,
      "immediate motion replaces stale native trajectory motion", 1e-5);
    simulationHarness.dispose();
  }

  public function testSmoothReplacementRetriesLateSubmission():Void {
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(new LinearAxis(23, 10, 80),
      "x", 0.08, 0.4);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var base = new SimulatedRobot("retry-replacement", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var robot = new LaggingRobot(base);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var options = new MotionOptions(0.05, 0.2);
    machine.moveAxes([new AxisTarget("x", 0.06)], options);
    for (tick in 0...5) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick));
    }
    robot.lateRejections = 1;
    var replacement = machine.moveAxes([new AxisTarget("x", 0.04)], options);
    check(replacement != null, "one late rejection is retried from a fresh snapshot");
    check(robot.replacementAttempts == 2,
      "late replacement submits exactly one retry");
    check(Int64.compare(robot.lastReplacementLeadNs, Int64.ofInt(20000000)) >= 0,
      "smooth replacement anchors at least two owner periods ahead");
    runMotion(machine, simulationHarness);
    near(base.snapshot().positions.get(0), 0.04, "retried replacement reaches target", 1e-5);
    robot.lateRejections = 2;
    machine.moveAxes([new AxisTarget("x", 0.06)], options);
    for (tick in 0...5) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick + 500));
    }
    var fallback = machine.moveAxes([new AxisTarget("x", 0.02)], options);
    check(fallback == null && machine.isMoving(),
      "two late rejections defer the target behind a stop");
    check(robot.lateRejections == 0,
      "stop-first fallback follows exactly two rejected attempts");
    runMotion(machine, simulationHarness);
    near(base.snapshot().positions.get(0), 0.02,
      "stop-first fallback reaches target", 1e-5);
    simulationHarness.dispose();
  }

  public function testFreeRunningSmoothReplacement():Void {
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(new LinearAxis(23, 10, 80),
      "x", 0.08, 0.4);
    blueprint.replacementOwnerPeriodSeconds = 0.001;
    var simulationHarness = new SimulationHarness(0.001);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("free-running-replacement", runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    simulationHarness.start();
    machine.jog("x", 0.03, 1.0, 0.2);
    for (index in 0...12) {
      Sys.sleep(0.004);
      machine.update();
      var result = machine.jog("x", index % 2 == 0 ? 0.02 : 0.035, 0.5, 0.2);
      check(result != null || machine.isMoving(),
        "free-running jog applies a replacement or defers behind a stop");
      var observation = robot.snapshot();
      check(Math.abs(observation.velocities.get(0)) <= 0.08 + 1e-5,
        "free-running replacement stays within velocity limit");
      check(observation.positions.get(0) >= -1e-6 &&
        observation.positions.get(0) <= 0.08 + 1e-6,
        "free-running replacement stays within travel limits");
    }
    simulationHarness.stop();
    simulationHarness.dispose();
  }

  /**
   * Replacing motion while moving and resuming after a hold must both stay
   * within the joint acceleration limit on the plan execution path.
   */
  public function testMotionChangesStayWithinLimits():Void {
    var limit = 0.4;
    for (queueSupport in [true]) {
      var label = "buffered";
      var worstReplace = 0.0;
      var worstJog = 0.0;
      var worstResume = 0.0;
      var worstReplaceTick = -1;
      var worstJogTick = -1;
      var worstResumeTick = -1;
      var eventTick = 4;
      while (eventTick < 330) {
        var replaced = gantryTrial(queueSupport, eventTick,
          machine -> machine.moveAxes([new AxisTarget("x", 0.02)],
            new MotionOptions(0.05, limit)), false);
        var replacePeak = peakSecondDifference(replaced);
        if (replacePeak > worstReplace) {
          worstReplace = replacePeak;
          worstReplaceTick = eventTick;
        }
        near(replaced[replaced.length - 1], 0.02,
          '$label move replaced at tick $eventTick reaches its new target', 1e-5);
        var furthest = 0.0;
        for (position in replaced) furthest = Math.max(furthest, position);
        check(furthest <= 0.15 + 1e-9,
          '$label move replaced at tick $eventTick stays within the first move');

        var jogged = gantryTrial(queueSupport, eventTick,
          machine -> machine.jog("x", -0.05, 0.5), false);
        var jogPeak = peakSecondDifference(jogged);
        if (jogPeak > worstJog) {
          worstJog = jogPeak;
          worstJogTick = eventTick;
        }

        var resumed = gantryTrial(queueSupport, eventTick, machine -> machine.hold(), true);
        var resumePeak = peakSecondDifference(resumed);
        if (resumePeak > worstResume) {
          worstResume = resumePeak;
          worstResumeTick = eventTick;
        }
        near(resumed[resumed.length - 1], 0.15,
          '$label move held and resumed at tick $eventTick reaches its target', 1e-5);
        eventTick += 8;
      }
      check(worstReplace <= limit * 1.05,
        '$label move replaced while moving stays within the limit (worst $worstReplace at tick $worstReplaceTick)');
      check(worstJog <= limit * 1.05,
        '$label jog while moving stays within the limit (worst $worstJog at tick $worstJogTick)');
      check(worstResume <= limit * 1.05,
        '$label resume stays within the limit (worst $worstResume at tick $worstResumeTick)');
    }
  }

  /**
   * A jog issued while the same axis is jogging changes speed or direction
   * without stopping first, within the acceleration limit, on the plan path.
   */
  public function testContinuousJog():Void {
    var limit = 0.4;
    for (queueSupport in [true]) {
      var label = "buffered";
      for (secondVelocity in [0.08, 0.02, -0.05]) {
        var eventTick = 10;
        while (eventTick <= 150) {
          var continued = false;
          var positions = gantryTrial(queueSupport, eventTick, machine -> {
            continued = machine.jog("x", secondVelocity, 1.0) != null;
          }, false, machine -> machine.jog("x", 0.05, 2.0));
          var context = '$label jog changed to $secondVelocity at tick $eventTick';
          check(continued, '$context continues without stopping first');
          check(peakSecondDifference(positions) <= limit * 1.05,
            '$context stays within the limit (peak ${peakSecondDifference(positions)})');
          // Count ticks at rest before the motion finally settles.
          var settled = positions.length - 1;
          while (settled > 0 && Math.abs(positions[settled] - positions[settled - 1]) < 1e-12)
            settled--;
          var pauses = 0;
          for (index in (eventTick + 1)...settled)
            if (Math.abs(positions[index] - positions[index - 1]) < 1e-12) pauses++;
          // A reversal passes through zero speed, but never dwells there.
          check(pauses <= (secondVelocity < 0.0 ? 1 : 0), '$context never pauses ($pauses)');
          eventTick += 20;
        }
      }
    }
  }

  /**
   * A replacement that arrives after its committed point is rejected
   * explicitly; the original jog remains safe and completes normally.
   */
  public function testLateJogReplacementRejectsLateArrival():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 60), new LinearAxis(23, 10, 40), 0.1, 0.4);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new LaggingRobot(new SimulatedRobot("late-splice", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]));
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    machine.jog("x", 0.05, 2.0);
    var positions:Array<Float> = [];
    var tick = 0;
    function step():Void {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      positions.push(robot.snapshot().positions.get(0));
      if (tick > 2000) throw "late jog splice did not settle";
    }
    for (_ in 0...40) step();
    robot.lagging = true;
    check(machine.jog("x", 0.02, 1.0) != null, "jog change is planned as a continuation");
    for (_ in 0...8) step();
    throws(() -> robot.release(), "late native replacement is rejected explicitly");
    for (_ in 0...250) {
      simulationHarness.step(Int64.ofInt(tick++));
      positions.push(robot.snapshot().positions.get(0));
    }
    check(peakSecondDifference(positions) <= 0.4 * 1.05,
      'original jog stays within the limit (peak ${peakSecondDifference(positions)})');
    near(positions[positions.length - 1], 0.1,
      "rejected replacement leaves the original jog to complete", 1e-5);
    simulationHarness.dispose();
  }

  /**
   * Holding and resuming along a path with an arc, and along a blended
   * corner, stays on the path and within the joint limits.
   */
  public function testPathHoldsStayOnPathWithinLimits():Void {
    var limit = 0.4;
    // A line, a quarter arc tangent to it, and a line tangent to the arc.
    function arcPath():GeometricPath
      return new GeometricPath([
        new LineSegment(new PathPoint(0.0, 0.0, 0.0), new PathPoint(0.05, 0.0, 0.0)),
        new ArcSegment(new PathPoint(0.05, 0.03, 0.0), 0.03, -Math.PI * 0.5, Math.PI * 0.5),
        new LineSegment(new PathPoint(0.08, 0.03, 0.0), new PathPoint(0.08, 0.06, 0.0))
      ]);
    function arcDistance(x:Float, y:Float):Float {
      var angle = Math.atan2(y - 0.03, x - 0.05);
      var radial = Math.sqrt((x - 0.05) * (x - 0.05) + (y - 0.03) * (y - 0.03));
      var onArc = angle >= -Math.PI * 0.5 && angle <= 0.0 ? Math.abs(radial - 0.03) : 1.0;
      return Math.min(onArc, Math.min(segmentDistance(x, y, 0.0, 0.0, 0.05, 0.0),
        segmentDistance(x, y, 0.08, 0.03, 0.08, 0.06)));
    }
    function cornerDistance(x:Float, y:Float):Float
      return Math.min(segmentDistance(x, y, 0.0, 0.0, 0.05, 0.0),
        segmentDistance(x, y, 0.05, 0.0, 0.05, 0.05));
    var cases:Array<{label:String, begin:MotionSystem -> Void, distance:(Float, Float) -> Float}> = [
      {label: "arc path", distance: arcDistance, begin: machine -> machine.movePath(arcPath(),
        PathPlanningOptions.exactStopMode(), new MotionOptions(0.05, limit))},
      {label: "exact-stop corner", distance: cornerDistance, begin: machine -> machine.movePath(
        GeometricPath.lines([new PathPoint(0.0, 0.0, 0.0), new PathPoint(0.05, 0.0, 0.0),
          new PathPoint(0.05, 0.05, 0.0)]), PathPlanningOptions.exactStopMode(),
        new MotionOptions(0.05, limit))}
    ];
    for (queueSupport in [true]) {
      var mode = "buffered";
      for (entry in cases) {
        // Blend mode changes velocity at a corner by design, so compare with
        // the uninterrupted move rather than the bare limit.
        var baseline = rigTrial(gantryRig(queueSupport), 0, _ -> {}, false, entry.begin);
        // Time-warping changes which 10 ms sample straddles a blended corner;
        // keep a small absolute allowance on this discrete second difference.
        var allowed = [for (joint in 0...2)
          Math.max(limit, jointPeak(baseline, joint)) * 1.05 + 0.01];
        var eventTicks = baseline.length;
        var eventTick = 3;
        while (eventTick < eventTicks) {
          var positions = rigTrial(gantryRig(queueSupport), eventTick,
            machine -> machine.hold(), true, entry.begin);
          var context = '$mode ${entry.label} held at tick $eventTick';
          for (joint in 0...2)
            check(jointPeak(positions, joint) <= allowed[joint],
              '$context keeps joint $joint within its limit (peak ${jointPeak(positions, joint)})');
          var furthest = 0.0;
          for (position in positions)
            furthest = Math.max(furthest, entry.distance(position[0], position[1]));
          check(furthest <= 2e-5, '$context stays on the path (off by $furthest)');
          var end = positions[positions.length - 1], expectedEnd = baseline[baseline.length - 1];
          check(Math.abs(end[0] - expectedEnd[0]) <= 1e-5 && Math.abs(end[1] - expectedEnd[1]) <= 1e-5,
            '$context reaches the path end');
          eventTick += 9;
        }
      }
    }
  }

  /**
   * A dual-motor axis whose motors have different gearing keeps each joint
   * within its own limits, and the motors coordinated, through holds,
   * resumes, and replacement moves.
   */
  public function testDualMotorAxisChangesStayWithinJointLimits():Void {
    var limits = [0.4, 0.8];
    var begin:MotionSystem -> Void = machine -> {
      machine.moveAxes([new AxisTarget("x", 0.06)], new MotionOptions(0.08, 0.4));
    };
    for (queueSupport in [true]) {
      var mode = "buffered";
      var baseline = rigTrial(dualMotorRig(queueSupport), 0, _ -> {}, false, begin);
      var eventTick = 3;
      while (eventTick < baseline.length) {
        var trials = [
          {label: "held", resume: true, event: (machine:MotionSystem) -> machine.hold()},
          {label: "replaced", resume: false, event: (machine:MotionSystem) -> {
            machine.moveAxes([new AxisTarget("x", 0.01)], new MotionOptions(0.08, 0.4));
          }}
        ];
        for (trial in trials) {
          var rig = dualMotorRig(queueSupport);
          var positions = rigTrial(rig, eventTick, trial.event, trial.resume, begin);
          var context = '$mode dual-motor axis ${trial.label} at tick $eventTick';
          for (joint in 0...2)
            check(jointPeak(positions, joint) <= limits[joint] * 1.05,
              '$context keeps joint $joint within its limit (peak ${jointPeak(positions, joint)})');
          // The simulated motors track with different loads, so coordination
          // is checked on every commanded position rather than the measured one.
          var worstSkew = 0.0;
          var recording = rig.recording;
          if (recording != null)
            for (command in recording.commands)
              switch command {
                case JointTargets(_, _) | TrajectoryChunk(_):
                  throw "MotionSystem submitted a non-plan motion command";
                case ExecutionPlan(plan):
                  worstSkew = Math.max(worstSkew,
                    Math.abs(plan.startPosition.get(1) + 2.0 * plan.startPosition.get(0)));
                  for (segment in plan.segments)
                    for (degree in 0...(segment.degree + 1))
                      worstSkew = Math.max(worstSkew,
                        Math.abs(segment.coefficients[1][degree] +
                          2.0 * segment.coefficients[0][degree]));
                case Hold | Resume | Abort: continue;
              }
          check(worstSkew <= 1e-12, '$context commands the motors coordinated (skew $worstSkew)');
        }
        eventTick += 6;
      }
    }
  }

}
