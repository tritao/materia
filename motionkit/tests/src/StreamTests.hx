import haxe.Int64;
import haxe.io.Bytes;
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
import motionkit.axis.MotionAxis;
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

class StreamTests extends MotionKitTestSupport {
  public function new() { super(); }

  public function testPlanCapableReplayRecordsMotionPlan():Void {
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(
      new LinearAxis(23, 10, 80), "x", 0.1, 0.4);
    var source = new RobotRecording();
    var initial = [for (_ in blueprint.model.joints) 0.0];
    var axisMapping = new MotionAxis(blueprint.axes[0],
      [for (joint in blueprint.model.joints) joint.id]);
    axisMapping.writeLogicalPosition(initial, axisMapping.homePosition);
    source.recordSnapshot(new RobotSnapshot("plan-replay", Int64.ofInt(0),
      Int64.ofInt(0), initial, [for (_ in initial) 0.0], [for (_ in initial) 0.0], 0, 0));
    var description = new RobotDescription("plan-replay", blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var capabilities = new RobotCapabilities("plan-replay", blueprint.model.joints.length,
      true, false, false, false, true, true);
    var unsupported = new ReplayRobot("plan-replay", source, description);
    throws(function() MotionSystem.fromBlueprint(unsupported, blueprint),
      "MotionSystem rejects a robot without queue and plan capabilities");
    unsupported.close();
    var replay = new ReplayRobot("plan-replay", source, description, capabilities);
    var machine = MotionSystem.fromBlueprint(replay, blueprint);
    machine.moveAxes([new AxisTarget("x", 0.02)]);
    check(replay.generatedCommands.commands.length == 1,
      "plan-capable replay records one generated command");
    switch replay.generatedCommands.commands[0] {
      case ExecutionPlan(plan):
        check(plan.segments.length > 0,
          "plan-capable replay records generated polynomial segments");
      case _:
        throw "plan-capable replay did not record an execution plan";
    }
    replay.close();
  }

  public function testBufferedExecution():Void {
    var xAxis = new LinearAxis(23, 10, 80);
    var yAxis = new LinearAxis(23, 10, 60);
    var zAxis = new LinearAxis(23, 10, 40);
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(xAxis, yAxis, zAxis, 0.1, 0.4);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("buffered-gantry", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var recording = new RobotRecording();
    var instrumented = new RecordingRobot(robot, recording);
    var machine = MotionSystem.fromBlueprint(instrumented, blueprint);
    var options = new MotionOptions(0.05, 0.2);
    check(machine.robot.capabilities().supportsTrajectoryQueue,
      "simulation runtime advertises trajectory queue support");

    var first = planned(machine.queueAxes([new AxisTarget("x", 0.02)], options));
    var second = planned(machine.queueAxes([new AxisTarget("x", 0.04)], options));
    check(machine.queueDepth() == 2, "buffer reports active and waiting trajectories");
    check(machine.queuedDurationSeconds() > first.durationSeconds(),
      "buffer reports the duration of waiting motion");
    near(second.evaluate(0.0).positions[0], 0.02,
      "queued axis motion starts at the previous trajectory endpoint");
    near(machine.progress(), 0.0, "buffer starts with zero progress");
    check(recording.commands.length == 1, "buffer submits the first move as one plan");
    switch recording.commands[0] {
      case TrajectoryChunk(chunk):
        throw "buffer submitted a legacy point chunk";
      case JointTargets(_, _):
        throw "buffer unexpectedly fell back to sample-by-sample targets";
      case ExecutionPlan(plan):
        check(plan.segments.length > 0, "trajectory plan carries polynomial segments");
      case Hold | Resume | Abort:
        throw "buffer submitted a lifecycle command before motion started";
    }

    var tick = 0;
    for (iteration in 0...5) {
      check(machine.update(), "buffer remains active while its first move is running");
      simulationHarness.step(Int64.ofInt(tick++));
      if (iteration == 0) {
        var nativeProgress = runtime.snapshot();
        check(nativeProgress.trajectoryActive && nativeProgress.trajectoryQueueDepth > 0,
          "native runtime reports active trajectory queue progress");
        check(nativeProgress.trajectoryDurationNs > nativeProgress.trajectoryTimeNs,
          "native runtime reports trajectory duration beyond current time");
      }
    }
    // Let the runtime advance while the host-side clock is stalled. Hold must
    // resume from the runtime's authoritative trajectory time, not replay
    // source samples that are already behind the actual machine pose.
    for (_ in 0...5) simulationHarness.step(Int64.ofInt(tick++));
    var beforeHold = robot.snapshot().positions.get(0);
    machine.hold();
    check(machine.isHolding(), "buffer reports controlled hold");
    check(machine.queueDepth() == 2, "hold preserves active and waiting trajectories");
    for (_ in 0...5) {
      check(!machine.update(), "held buffer does not submit motion commands");
      simulationHarness.step(Int64.ofInt(tick++));
    }
    var heldPosition = robot.snapshot().positions.get(0);
    check(heldPosition > beforeHold && heldPosition < 0.02,
      "controlled hold decelerates before coming to rest");
    var holdTicks = 0;
    while (runtime.snapshot().sessionState != RobotKitRuntimeConstants.RK_SESSION_HELD) {
      check(!machine.update(), "held buffer remains paused after deceleration");
      simulationHarness.step(Int64.ofInt(tick++));
      holdTicks += 1;
      if (holdTicks > 200) throw "controlled hold did not settle";
    }
    var stoppedPosition = robot.snapshot().positions.get(0);
    check(stoppedPosition >= heldPosition,
      "controlled hold follows the path while slowing down");
    for (_ in 0...2) {
      simulationHarness.step(Int64.ofInt(tick++));
    }
    near(robot.snapshot().positions.get(0), stoppedPosition,
      "controlled hold remains stopped after deceleration", 1e-5);

    machine.update();
    machine.resume();
    check(!machine.isHolding(), "buffer resumes from controlled hold");
    for (_ in 0...2) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
    }
    var resumedPosition = robot.snapshot().positions.get(0);
    check(resumedPosition >= heldPosition - 1e-6,
      "resume does not replay a stale trajectory sample backwards");
    while (machine.isMoving()) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      if (tick > 2000) throw "buffered MotionKit trajectory did not complete";
    }
    near(robot.snapshot().positions.get(0), 0.04,
      "buffered trajectories execute in order", 1e-5);
    check(machine.queueDepth() == 0, "buffer empties after the final trajectory");
    near(machine.progress(), 1.0, "buffer reports completed progress");

    machine.queueAxes([new AxisTarget("x", 0.01)], options);
    check(machine.queueDepth() == 1, "buffer accepts a new trajectory after completion");
    machine.abort();
    check(machine.queueDepth() == 0 && !machine.isMoving(),
      "abort clears active and waiting trajectories");
    check(!machine.isHolding(), "abort clears controlled hold state");
    simulationHarness.dispose();
  }

  public function testLongBufferedExecution():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.4);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("long-buffer", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var recording = new RobotRecording();
    var instrumented = new RecordingRobot(robot, recording);
    var machine = MotionSystem.fromBlueprint(instrumented, blueprint);
    var axisMapping = machine.axis("x");
    if (axisMapping == null) throw "Long buffered axis is missing";
    var samples:Array<Array<Float>> = [];
    for (index in 0...601) {
      var sample = [for (_ in blueprint.model.joints) 0.0];
      axisMapping.writeLogicalPosition(sample, 0.05 * index / 600.0);
      samples.push(sample);
    }
    var trajectory = Trajectory.fromPositionSamples(
      [for (index in 0...601) index * 0.01],
      samples);
    machine.queueTrajectory(trajectory);
    function submittedSegments():Int {
      var total = 0;
      for (command in recording.commands) switch command {
        case RobotCommand.ExecutionPlan(plan): total += plan.segments.length;
        case _:
      }
      return total;
    }
    // A quarter second of the six-second, 600-segment trajectory to start with.
    var initialSegments = submittedSegments();
    check(initialSegments >= 25 && initialSegments <= 40,
      'long trajectory starts with a short native plan window, got $initialSegments segments');
    // Updates top the window up a tenth of a second at a time, to two seconds and no further.
    for (_ in 0...30) machine.update();
    var windowSegments = submittedSegments();
    check(windowSegments >= 200 && windowSegments <= 212,
      'the streamer fills a two-second window before the owner advances, got $windowSegments segments');
    var windowPlans = recording.commands.length;
    machine.update();
    check(recording.commands.length == windowPlans, "the streamer stops at a full window");

    var tick = 0;
    simulationHarness.step(Int64.ofInt(tick++));
    while (machine.isMoving()) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      if (tick > 1200) throw "long buffered trajectory did not complete";
    }
    for (_ in 0...4) simulationHarness.step(Int64.ofInt(tick++));
    check(recording.commands.length >= 3,
      "long trajectory refills native chunks before the queue drains");
    for (command in recording.commands) switch command {
      case RobotCommand.TrajectoryChunk(chunk):
        throw "long trajectory submitted a legacy point chunk";
      case RobotCommand.JointTargets(_, _):
        throw "long trajectory unexpectedly fell back to sample-by-sample targets";
      case RobotCommand.ExecutionPlan(plan):
        check(plan.segments.length <= 27,
          "streamed trajectory plans hold at most a quarter second of motion each");
      case Hold | Resume | Abort:
        throw "long trajectory unexpectedly submitted a lifecycle command";
    }
    near(instrumented.snapshot().positions.get(0), 0.05,
      "streamed trajectory reaches its final position", 1e-5);
    near(machine.progress(), 1.0, "streamed trajectory reports completed progress");
    simulationHarness.dispose();
  }

  public function testHoldRefillsNearChunkBoundary():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.4);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("hold-refill-boundary", runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var axisMapping = machine.axis("x");
    if (axisMapping == null) throw "Hold refill axis is missing";
    var samples:Array<Array<Float>> = [];
    for (index in 0...2001) {
      var sample = [for (_ in blueprint.model.joints) 0.0];
      axisMapping.writeLogicalPosition(sample, 0.06 * index / 2000.0);
      samples.push(sample);
    }
    machine.queueTrajectory(Trajectory.fromPositionSamples(
      [for (index in 0...2001) index * 0.005],
      samples));

    var tick = 0;
    var previousPreHoldPosition = 0.0;
    var preHoldPosition = 0.0;
    for (_ in 0...252) {
      previousPreHoldPosition = preHoldPosition;
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      preHoldPosition = robot.snapshot().positions.get(0);
    }
    var beforeHoldSnapshot = robot.snapshot();
    var beforeHold = beforeHoldSnapshot.positions.get(0);
    machine.hold();
    var previousPosition = beforeHold;
    var previousVelocity = (beforeHold - previousPreHoldPosition) / 0.01;
    var peakAcceleration = 0.0;
    var stopTicks = 0;
    while (runtime.snapshot().sessionState != RobotKitRuntimeConstants.RK_SESSION_HELD) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      var position = robot.snapshot().positions.get(0);
      var velocity = (position - previousPosition) / 0.01;
      peakAcceleration = Math.max(peakAcceleration,
        Math.abs(velocity - previousVelocity) / 0.01);
      previousPosition = position;
      previousVelocity = velocity;
      stopTicks += 1;
      if (stopTicks > 200) throw "near-boundary controlled hold did not settle";
    }
    // Hold is a runtime safety action: it may brake harder than the requested
    // planning acceleration, up to the physical cap from the drive.
    var physicalCap = blueprint.runtime.joints[0].maxAcceleration;
    if (physicalCap == null) throw "Buffered hold needs a physical acceleration ceiling";
    check(peakAcceleration <= physicalCap * 1.05,
      'hold near a streamed refill boundary exceeds its physical deceleration cap: $peakAcceleration versus $physicalCap m/s²');
    check(previousPosition > beforeHold,
      "hold near a streamed refill boundary continues along the path to rest");
    simulationHarness.dispose();
  }

}
