import haxe.Int64;
import haxe.io.Bytes;
import cnckit.CncMachine;
import cnckit.CncTool;
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
import motionkit.robot.CncMotionBinding;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.PathConfigurationSelector;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.MachineKitRobotCompiler;
import motionkit.robot.MotionSystem;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.path.ArcSegment;
import motionkit.path.CircularPlane;
import motionkit.path.CircularSegment;
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

class MotionKitBootstrapTests {
  static var assertions:Int = 0;

  public static function main():Void {
    if (Sys.getEnv("MOTIONKIT_CNC_ONLY") == "1") {
      testCircularSegments();
      testCncProgramBinding();
      testPhysicalAssemblyCncBinding();
      Sys.println('CNC focused tests passed ($assertions assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_C7_ONLY") == "1") {
      testVirtualCncProgram();
      Sys.println('C7 focused tests passed ($assertions assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_MACHINEKIT_ONLY") == "1") {
      testLinearAxisCompilesToRobotModel();
      testCompiledXYZGantryRunsThroughSimulation();
      testMachineKitLeadScrewThroughVirtualDevice();
      Sys.println('MachineKit compiler tests passed ($assertions assertions)');
      return;
    }
    if (Sys.getEnv("MOTIONKIT_C4_ONLY") == "1") {
      testOpwKinematics();
      Sys.println('C4 focused tests passed ($assertions assertions)');
      return;
    }
    testPoseProcessPath();
    testMotionEventContracts();
    testKinematicsContract();
    testOpwKinematics();
    testMotionProgramContracts();
    testProgramCompiler();
    testPathConfigurationSelector();
    testAxisKinematics();
    testCncProgramBinding();
    testPhysicalAssemblyCncBinding();
    testVirtualCncProgram();
    testManipulatorMotion();
    testSimplePathTimingContract();
    testNativePathLowering();
    testToppraPathTiming();
    testGeometricPathPrimitives();
    testNativeTrajectoryRoundTrip();
    testNativeValidationAndPlan();
    testPlannerIsDeterministicAndBounded();
    testToppraExactStopsAndBindings();
    testToppraCircleAcceleration();
    testLinearAxisCompilesToRobotModel();
    testLeadScrewActuatorRateLimitsPlans();
    testTransmissionDerivedAxisMapping();
    testCompiledAxisRunsThroughSimulation();
    testHomingAndJogging();
    testMoveLinearUsesPlannerLimits();
    testCompiledXYZGantryRunsThroughSimulation();
    testMachineKitLeadScrewThroughVirtualDevice();
    testDualMotorAxisRunsThroughSimulation();
    testBufferedExecution();
    testPlanCapableReplayRecordsMotionPlan();
    testLongBufferedExecution();
    testHoldRefillsNearChunkBoundary();
    testHoldDecelerationStaysWithinLimitsThroughoutMove();
    testRuntimeSynchronizedHolding();
    testImmediateMotionReplacesNativeQueue();
    testSmoothReplacementRetriesLateSubmission();
    testFreeRunningSmoothReplacement();
    testMotionChangesStayWithinLimits();
    testContinuousJog();
    testLateJogReplacementRejectsLateArrival();
    testPathHoldsStayOnPathWithinLimits();
    testToleranceBlend();
    testDualMotorAxisChangesStayWithinJointLimits();
    Sys.println('MotionKit bootstrap tests passed ($assertions assertions)');
  }

  static function testNativePathLowering():Void {
    var samples = new JointPathSamples([0.0, 1.0, 2.0],
      [[0.0], [1.0], [4.0]], [[0.0], [2.0], [4.0]], [[2.0], [2.0], [2.0]]);
    var path = new NativeJointPath(samples);
    var law = new PathTimeLaw([
      new PathTimeStage(Trajectory.nanoseconds(0.0), Trajectory.nanoseconds(1.0),
        0.0, 1.0, 0.0),
      new PathTimeStage(Trajectory.nanoseconds(1.0), Trajectory.nanoseconds(1.0),
        1.0, 1.0, 0.0)
    ]);
    near(law.distanceToTime(1.0), 1.0, "native path time law preserves knots");
    var trajectory = path.lower(law, 1e-6);
    near(trajectory.evaluate(1.5).positions[0], 2.25,
      "native path lowering follows quadratic path");
    trajectory.dispose();
    law.dispose();
    path.dispose();
  }

  static function testToppraPathTiming():Void {
    var path = new JointPathSamples([0.0, 1.0], [[0.0], [1.0]],
      [[1.0], [1.0]], [[0.0], [0.0]]);
    var timed = new ToppraPathTiming(1e-8).time(path,
      new PathTimingLimits([0.4], [1.0]));
    check(timed.trajectory.durationSeconds() > 2.8 &&
      timed.trajectory.durationSeconds() < 3.1,
      'TOPP-RA respects velocity limit (${timed.trajectory.durationSeconds()})');
    near(timed.distanceToTime(1.0), timed.trajectory.durationSeconds(),
      "TOPP-RA end distance maps to end time", 1e-6);
    check(timed.bindingConstraints.length > 0,
      "TOPP-RA reports binding constraints");
    timed.releaseDistanceMap();
    timed.trajectory.dispose();
  }

  static function testPoseProcessPath():Void {
    var a = new PoseWaypoint(new Pose3(), 0.001, 0.01);
    var b = new PoseWaypoint(new Pose3(1.0, 0.0, 0.0), 0.002, 0.02);
    var path = new PosePath("work", [new PoseLine(a, b, OrientationPolicy.Interpolated, 0.1, 0.2)]);
    near(path.length(), 1.0, "line length", 1e-9);
    near(path.poseAt(0.5).x, 0.5, "line midpoint", 1e-9);
    near(path.waypointAt(1.0).positionTolerance, 0.002, "endpoint tolerance", 1e-9);
    var arc = new PoseArc(new PoseWaypoint(new Pose3(1,0,0), 0.001, 0.01),
      new PoseWaypoint(new Pose3(0,1,0), 0.001, 0.01),
      new PoseWaypoint(new Pose3(-1,0,0), 0.001, 0.01),
      OrientationPolicy.Interpolated, 0.2);
    near(arc.length(), Math.PI, "semicircle length", 1e-9);
    near(arc.waypointAt(arc.length()/2).pose.y, 1.0, "arc through via", 1e-9);
    var rotated = new PoseLine(a, new PoseWaypoint(new Pose3(0,0,0,0,0,1,0), 0.001, 0.01),
      OrientationPolicy.Interpolated, 0.1, 0.2);
    near(rotated.length(), 0.1*Math.PI, "rotation metric", 1e-9);
    var points = [false, true, false, true, false, true, false];
    var toolpoints:Array<ToolpathPoint> = [];
    for (i in 0...points.length)
      toolpoints.push(new ToolpathPoint(new Transform3(new Vec3(i * 0.1, 0, 0), Quat.identity()), 0.2, points[i]));
    var converted = ToolpathPosePath.convert(new Toolpath("work", toolpoints), "sprayer.flow");
    check(converted.events.length == 6, "three process spans have six events");
    near(converted.events[0].distance, 0.1, "first process transition", 1e-9);
    near(converted.events[5].distance, 0.6, "last process transition", 1e-9);
  }

  static function testMotionEventContracts():Void {
    var pathEvent = new PathEvent(0.25, "sprayer.flow", EventValue.Analog(0.4),
      0.02, HoldPolicy.SafeWhileHeld);
    near(pathEvent.distance, 0.25, "path event retains its authored distance");
    near(pathEvent.leadSeconds, 0.02, "path event retains its actuator lead");
    check(pathEvent.channel == "sprayer.flow", "path event retains its channel");
    check(switch pathEvent.value {
      case Analog(value): Math.abs(value - 0.4) < 1e-12;
      case _: false;
    }, "path event retains its typed value");

    var timed = new TimedEvent(Int64.parseString("123456789"), "sprayer.enabled",
      EventValue.Digital(true), HoldPolicy.RestoreOnResume);
    check(Int64.compare(timed.timeNs, Int64.parseString("123456789")) == 0,
      "timed event uses plan-relative nanoseconds");
    check(switch timed.value { case Digital(value): value; case _: false; },
      "timed event retains a digital value");

    var channel = new ChannelDeclaration("sprayer.flow", ChannelKind.Analog,
      EventValue.Analog(0.0));
    check(channel.id == "sprayer.flow", "channel declaration retains its stable ID");
    check(switch channel.safeValue { case Analog(value): value == 0.0; case _: false; },
      "channel declaration retains its safe value");

    throws(function() new PathEvent(-0.1, "sprayer.flow", EventValue.Analog(0.0)),
      "path event rejects a negative distance");
    throws(function() new PathEvent(0.0, " ", EventValue.Digital(false)),
      "path event rejects an empty channel");
    throws(function() new PathEvent(0.0, "sprayer.flow", EventValue.Analog(Math.NaN)),
      "path event rejects a non-finite analog value");
    throws(function() new PathEvent(0.0, "sprayer.command",
      EventValue.Process("", 0.0)), "path event rejects an empty process command");
    throws(function() new TimedEvent(Int64.ofInt(-1), "sprayer.enabled",
      EventValue.Digital(false)), "timed event rejects a negative path time");
    throws(function() new ChannelDeclaration("sprayer.flow", ChannelKind.Analog,
      EventValue.Digital(false)), "channel declaration rejects a mismatched safe value");
  }

  static function testPathConfigurationSelector():Void {
    var selector = new PathConfigurationSelector(new PlanarSolver(),
      [for (_ in 0...6) -20.0], [for (_ in 0...6) 20.0],
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 1.0]);
    var candidate = (x:Float) -> [x, 0.0, 0.0, 0.0, 0.0, 0.0];
    var chosen = selector.select([0.0, 0.5, 1.0], [
      [candidate(0.0), candidate(10.0)],
      [candidate(1.0), candidate(9.0)], [candidate(10.0)]]);
    near(chosen[0][0], 10.0, "Descartes avoids an unreachable nearest branch");
    near(chosen[1][0], 9.0, "Descartes keeps the continuous branch");
    var narrow = new PathConfigurationSelector(new PlanarSolver(),
      [for (_ in 0...6) -20.0], [for (_ in 0...6) 20.0],
      [for (_ in 0...6) 0.5], [for (_ in 0...6) 1.0]);
    var message = "";
    try narrow.select([0.0, 0.5, 1.0], [
      [candidate(0.0), candidate(10.0)],
      [candidate(1.0), candidate(9.0)], [candidate(10.0)]])
    catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf("0.500000") >= 0,
      "Descartes reports the first disconnected sample distance");
  }

  static function testProgramCompiler():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(new Manipulator(fixture.model, fixture.chain), 1e-8);
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(1));
    for (joint in 0...6) limits.jerk(joint, 20.0);
    var velocity = [for (_ in 0...6) 2.0];
    var acceleration = [for (_ in 0...6) 4.0];
    var jerk = [for (_ in 0...6) 20.0];
    var compiler = new ProgramCompiler(solver, limits, "work", velocity,
      acceleration, jerk);
    var start = [0.2, -0.4, 0.6, 0.1, 0.4, -0.2];
    var goal = start.copy(); goal[0] += 0.05;
    var program = new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget(goal), new MotionOptions(), Blend.ExactStop)]);
    var compiled = compiler.compile(program, start, Int64.ofInt(100));
    check(compiled.blocks.length == 1 && compiled.blocks[0].plans.length == 1,
      "program compiler lowers a joint move to one plan");
    check(compiled.blocks[0].plans[0].report.checks[
      MotionKitNativeConstants.MK_CHECK_JERK].status ==
      MotionKitNativeConstants.MK_CHECK_PASSED,
      "Ruckig MoveJ reports jerk checked");
    near(compiled.blocks[0].plans[0].evaluate(
      compiled.blocks[0].plans[0].durationSeconds).positions[0], goal[0],
      "program compiler reaches the MoveJ target", 1e-6);
    compiled.dispose();

    var startPose = solver.forward(start);
    var endQ = start.copy(); endQ[0] += 0.025;
    var endPose = solver.forward(endQ);
    var path = new PosePath("work", [new PoseLine(
      new PoseWaypoint(startPose, 0.005, 0.02),
      new PoseWaypoint(endPose, 0.005, 0.02),
      OrientationPolicy.Interpolated, 0.1, 0.1)]);
    var pathProgram = new MotionProgram([
      MotionOp.FollowPath(path, "work", 0.1,
        [new PathEvent(path.length() * 0.5, "sprayer.enabled",
          EventValue.Digital(true))]),
      MotionOp.SetOutput("sprayer.enabled", EventValue.Digital(false)),
      MotionOp.WaitInput("sprayer.ready", InputPredicate.Equals(
        EventValue.Digital(true)), 2.0),
      MotionOp.MoveJ(MoveTarget.JointTarget(start), new MotionOptions(),
        Blend.ExactStop),
      MotionOp.Dwell(0.1)
    ]);
    var lowered = compiler.compile(pathProgram, start, Int64.ofInt(200));
    check(lowered.blocks.length == 2 && lowered.blocks[0].plans.length == 1 &&
      lowered.blocks[1].plans.length == 1,
      "WaitInput splits a path and a return move into separate blocks");
    var pathPlan = lowered.blocks[0].plans[0];
    check(pathPlan.events.length == 2, "path event and SetOutput reach the plan");
    check(Int64.compare(pathPlan.events[0].timeNs, Int64.ofInt(0)) > 0 &&
      Int64.toFloat(pathPlan.events[0].timeNs) <
        pathPlan.durationSeconds * 1e9,
      "path event is placed inside the timed path");
    near(Int64.toFloat(pathPlan.events[1].timeNs) * 1e-9,
      pathPlan.durationSeconds, "SetOutput fires at the prior move end", 1e-9);
    check(pathPlan.report.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE].status ==
      MotionKitNativeConstants.MK_CHECK_PASSED,
      "sampled task-space validation is recorded in the plan");
    lowered.dispose();

    var line = compiler.compile(new MotionProgram([MotionOp.MoveL(endPose,
      "work", 0.1, Blend.ExactStop)]), start, Int64.ofInt(300));
    check(line.blocks[0].plans[0].report.checks[
      MotionKitNativeConstants.MK_CHECK_TASK_SPACE].status ==
      MotionKitNativeConstants.MK_CHECK_PASSED,
      "MoveL meets the sampled Cartesian tolerance");
    line.dispose();
    var poseMove = compiler.compile(new MotionProgram([MotionOp.MoveJ(
      MoveTarget.PoseTarget(solver.forward([for (_ in 0...6) 0.0]), "work", null),
      new MotionOptions(), Blend.ExactStop)]), start, Int64.ofInt(301));
    check(poseMove.blocks[0].plans.length == 1,
      "MoveJ resolves a reachable 6R pose through candidate IK");
    poseMove.dispose();

    var branchSolver = new WristBranchSolver();
    var branchCompiler = new ProgramCompiler(branchSolver, limits, "work", velocity,
      acceleration, jerk, null, 0.05, 0.5);
    var branchPath = new PosePath("work", [new PoseLine(
      new PoseWaypoint(new Pose3(0.0), 0.005, 0.02),
      new PoseWaypoint(new Pose3(1.0), 0.005, 0.02),
      OrientationPolicy.Fixed, 0.1, 0.1)]);
    var branchError = "";
    try branchCompiler.compile(new MotionProgram([MotionOp.FollowPath(branchPath,
      "work", 0.1, [])]), [0.0, 0.0, 0.0, 0.0, 0.1, 0.0], Int64.ofInt(400))
    catch (error:Dynamic) branchError = Std.string(error);
    check(branchError.indexOf("op 0 IK discontinuity at path distance") >= 0,
      "a wrist-branch jump is rejected with its op and path distance");

    var linearSolver = new WristBranchSolver(false);
    var linearCompiler = new ProgramCompiler(linearSolver, limits, "work", velocity,
      acceleration, jerk, null, 0.05);
    var linearPath = new PosePath("work", [new PoseLine(
      new PoseWaypoint(new Pose3(0.0), 0.005, 0.02),
      new PoseWaypoint(new Pose3(0.1), 0.005, 0.02),
      OrientationPolicy.Fixed, 0.1, 0.1)]);
    var linearProgram = new MotionProgram([MotionOp.FollowPath(linearPath,
      "work", 0.1, [new PathEvent(0.05, "sprayer.enabled",
        EventValue.Digital(true), 0.02)])]);
    var linearPlan = linearCompiler.compile(linearProgram,
      [0.0, 0.0, 0.0, 0.0, 0.1, 0.0], Int64.ofInt(500));
    var reference = new ToppraPathTiming().time(new JointPathSamples(
      [0.0, 0.05, 0.1],
      [[0.0, 0.0, 0.0, 0.0, 0.1, 0.0],
       [0.05, 0.0, 0.0, 0.0, 0.1, 0.0],
       [0.1, 0.0, 0.0, 0.0, 0.1, 0.0]],
      [for (_ in 0...3) [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
      [for (_ in 0...3) [for (_ in 0...6) 0.0]]),
      new PathTimingLimits(velocity, acceleration, [0.1, 0.1]));
    check(Int64.compare(linearPlan.blocks[0].plans[0].events[0].timeNs,
      Trajectory.nanoseconds(reference.distanceToTime(0.05) - 0.02)) == 0,
      "FollowPath event follows the lowered distance-to-time law and lead");
    reference.releaseDistanceMap();
    reference.trajectory.dispose();
    linearPlan.dispose();
    var bounded = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(1));
    bounded.position(0, -0.1, 0.05);
    var boundedCompiler = new ProgramCompiler(linearSolver, bounded, "work", velocity,
      acceleration, jerk, null, 0.05);
    var limitError = "";
    try boundedCompiler.compile(linearProgram,
      [0.0, 0.0, 0.0, 0.0, 0.1, 0.0], Int64.ofInt(501))
    catch (error:Dynamic) limitError = Std.string(error);
    check(limitError.indexOf("op 0 joint limit 0 at path distance") >= 0,
      "path joint-limit diagnostics identify the op, joint, and distance");
    var yaw = new Pose3(0.0, 0.0, 0.0, 0.0, 0.0,
      Math.sin(Math.PI / 4.0), Math.cos(Math.PI / 4.0));
    var freePath = new PosePath("work", [new PoseLine(
      new PoseWaypoint(new Pose3(), 0.005, 0.02),
      new PoseWaypoint(yaw, 0.005, 0.02),
      OrientationPolicy.FreeAboutTool, 0.1, 0.1)]);
    var freePlan = linearCompiler.compile(new MotionProgram([MotionOp.FollowPath(
      freePath, "work", 0.1, [])]), [0.0, 0.0, 0.0, 0.0, 0.1, 0.0],
      Int64.ofInt(502));
    check(freePlan.blocks[0].plans[0].report.checks[
      MotionKitNativeConstants.MK_CHECK_TASK_SPACE].status ==
      MotionKitNativeConstants.MK_CHECK_PASSED,
      "FreeAboutTool accepts a free twist about the tool axis");
    freePlan.dispose();

    var planarCompiler = new ProgramCompiler(new PlanarSolver(), limits,
      "work", velocity, acceleration, jerk, null, 0.005);
    var blended = planarCompiler.compile(new MotionProgram([
      MotionOp.MoveL(new Pose3(0.05, 0.0), "work", 0.1,
        Blend.ToleranceBlend(0.005)),
      MotionOp.MoveL(new Pose3(0.05, 0.05), "work", 0.1, Blend.ExactStop)
    ]), [for (_ in 0...6) 0.0], Int64.ofInt(503));
    check(blended.blocks.length == 1 && blended.blocks[0].plans.length == 1 &&
      blended.notes.length == 1 && blended.notes[0].indexOf("tolerance blended") >= 0,
      "planar MoveL corner uses C3 tolerance blending in one timed plan");
    var blendedPlan = blended.blocks[0].plans[0];
    check(blendedPlan.report.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE].status ==
      MotionKitNativeConstants.MK_CHECK_PASSED,
      "blended Cartesian plan passes authored-corner task-space validation");
    check(blendedPlan.report.checks[MotionKitNativeConstants.MK_CHECK_JERK].status ==
      MotionKitNativeConstants.MK_CHECK_UNCHECKED,
      "TOPP-RA Cartesian plan reports jerk unchecked");
    near(blendedPlan.report.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE].limit,
      0.005, "blended task-space report uses the authored tolerance");
    blended.dispose();
    // A 150 degree authored corner stresses the fillet setback formula.
    var turn = Math.PI / 6.0;
    var shallow = planarCompiler.compile(new MotionProgram([
        MotionOp.MoveL(new Pose3(0.05, 0.0), "work", 0.1,
          Blend.ToleranceBlend(0.001)),
        MotionOp.MoveL(new Pose3(0.05 + 0.05 * Math.cos(turn),
          0.05 * Math.sin(turn)), "work", 0.1, Blend.ExactStop)
      ]), [for (_ in 0...6) 0.0], Int64.ofInt(504));
    var shallowCheck = shallow.blocks[0].plans[0].report.checks[
      MotionKitNativeConstants.MK_CHECK_TASK_SPACE];
    check(shallowCheck.status == MotionKitNativeConstants.MK_CHECK_PASSED &&
      shallowCheck.value <= 0.001 + 1e-9 && shallowCheck.limit == 0.001,
      "150 degree authored corner is checked against its tolerance");
    shallow.dispose();
  }

  static function testManipulatorMotion():Void {
    var fixture = buildContractArmFixture();
    for (joint in fixture.model.joints) joint.limits.maxAcceleration = 4.0;
    var simulation = new Simulation(0.01);
    var blueprint = RobotRuntimeCompiler.compile(fixture.model);
    blueprint.channels.push(new ProcessChannelDeclaration("sprayer.enabled",
      ProcessEventValue.Digital(false)));
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("program-arm", runtime, fixture.model.name,
      [for (link in fixture.model.links) link.name],
      [for (joint in fixture.model.joints) joint.name]);
    var solver = new ManipulatorKinematics(new Manipulator(fixture.model, fixture.chain), 1e-8);
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(0));
    var compiler = new ProgramCompiler(solver, limits, "work",
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 4.0], [for (_ in 0...6) 20.0]);
    var unsupported = new RuntimeRobotAdapter("unsupported-arm", runtime,
      fixture.model.name, [for (link in fixture.model.links) link.name],
      [for (joint in fixture.model.joints) joint.name], false, false,
      "simulated runtime fault", false);
    throws(function() new ManipulatorMotion(unsupported, compiler,
      function(_) return null, function() return runtime.pollEvents()),
      "manipulator requires plan support at construction");
    var ready = false;
    var motion = new ManipulatorMotion(robot, compiler,
      function(_) return EventValue.Digital(ready),
      function() return runtime.pollEvents());
    motion.run(new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget([0.1]), new MotionOptions(), Blend.ExactStop)]));
    check(!motion.running && motion.failure != null &&
      motion.failure.indexOf("Motion program op 0") >= 0,
      "manipulator reports compiler diagnostics");
    var first = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0];
    var second = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    motion.run(new MotionProgram([
      MotionOp.MoveJ(MoveTarget.JointTarget(first), new MotionOptions(), Blend.ExactStop),
      MotionOp.SetOutput("sprayer.enabled", EventValue.Digital(true)),
      MotionOp.WaitInput("ready", InputPredicate.Equals(EventValue.Digital(true)), 3.0),
      MotionOp.MoveJ(MoveTarget.JointTarget(second), new MotionOptions(), Blend.ExactStop),
      MotionOp.SetOutput("sprayer.enabled", EventValue.Digital(false))
    ]));
    for (tick in 0...400) {
      if (tick == 100) ready = true;
      motion.update(0.01);
      simulation.step(Int64.ofInt(tick));
      if (!motion.running) break;
    }
    check(motion.completed && motion.failure == null,
      'manipulator executes two blocks across WaitInput: ${motion.failure}, running=${motion.running}, block=${motion.progress().block}');
    near(robot.snapshot().positions.get(0), 0.0,
      "manipulator returns to initial joint position", 1e-3);
    check(motion.firedEvents().length >= 2,
      "manipulator reports process events from both blocks");
    ready = false;
    motion.run(new MotionProgram([
      MotionOp.MoveJ(MoveTarget.JointTarget(first), new MotionOptions(), Blend.ExactStop),
      MotionOp.SetOutput("sprayer.enabled", EventValue.Digital(true)),
      MotionOp.WaitInput("ready", InputPredicate.Equals(EventValue.Digital(true)), 0.2)
    ]));
    for (tick in 0...400) {
      motion.update(0.01);
      simulation.step(Int64.ofInt(500 + tick));
      if (!motion.running) break;
    }
    check(!motion.completed && motion.failure != null &&
      motion.failure.indexOf("timed out") >= 0,
      "barrier timeout fails the manipulator program with a diagnostic");
    simulation.step(Int64.ofInt(1000));
    check(Lambda.exists(motion.firedEvents(), function(event) return switch event.value {
      case ProcessEventValue.Digital(enabled): !enabled;
      case _: false;
    }),
      "timeout abort produces a safe output transition");
    simulation.dispose();

    var pathSimulation = new Simulation(0.01);
    var pathBlueprint = RobotRuntimeCompiler.compile(fixture.model);
    pathBlueprint.channels.push(new ProcessChannelDeclaration("sprayer.enabled",
      ProcessEventValue.Digital(false)));
    var pathRuntime = pathSimulation.addRobot(pathBlueprint);
    var pathRobot = new SimulatedRobot("path-arm", pathRuntime, fixture.model.name,
      [for (link in fixture.model.links) link.name],
      [for (joint in fixture.model.joints) joint.name]);
    var pathMotion = new ManipulatorMotion(pathRobot, compiler,
      function(_) return null, function() return pathRuntime.pollEvents());
    var origin = [0.2, -0.4, 0.6, 0.1, 0.4, -0.2];
    var destination = origin.copy(); destination[0] = 0.2;
    destination[0] += 0.2;
    var pathTick = 0;
    pathMotion.run(new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget(origin), new MotionOptions(), Blend.ExactStop)]));
    for (_ in 0...500) {
      pathMotion.update(0.01);
      pathSimulation.step(Int64.ofInt(pathTick++));
      if (!pathMotion.running) break;
    }
    check(pathMotion.completed, 'arm reaches FollowPath start: ${pathMotion.failure}');
    var path = new PosePath("work", [new PoseLine(
      new PoseWaypoint(solver.forward(origin), 0.01, 0.04),
      new PoseWaypoint(solver.forward(destination), 0.01, 0.04),
      OrientationPolicy.Interpolated, 0.1, 0.05)]);
    pathMotion.run(new MotionProgram([MotionOp.FollowPath(path, "work", 0.05,
      [new PathEvent(path.length() * 0.95, "sprayer.enabled",
        EventValue.Digital(true), 0.0, HoldPolicy.SafeWhileHeld)])]));
    check(pathMotion.running,
      'FollowPath starts: ${pathMotion.failure}');
    for (_ in 0...10) {
      pathMotion.update(0.01);
      pathSimulation.step(Int64.ofInt(pathTick++));
    }
    pathMotion.hold();
    for (_ in 10...50) {
      pathMotion.update(0.01);
      pathSimulation.step(Int64.ofInt(pathTick++));
    }
    var heldDistance = pathMotion.progress().pathDistance;
    var heldPose = solver.forward(pathRobot.snapshot().positions.toArray());
    check(motionkit.path.PoseMath.distance(heldPose,
      path.waypointAt(heldDistance).pose) < 0.02,
      "held FollowPath remains on the authored path");
    check(pathMotion.firedEvents().length == 0,
      "hold delays the later process event");
    pathMotion.resume();
    for (_ in 50...500) {
      pathMotion.update(0.01);
      pathSimulation.step(Int64.ofInt(pathTick++));
      if (!pathMotion.running) break;
    }
    check(pathMotion.completed && pathMotion.firedEvents().length > 0,
      'held FollowPath completes and fires its event after resume: ${pathMotion.failure}');
    pathMotion.run(new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget(origin), new MotionOptions(), Blend.ExactStop)]));
    for (_ in 0...5) {
      pathMotion.update(0.01);
      pathSimulation.step(Int64.ofInt(pathTick++));
    }
    pathMotion.abort();
    pathSimulation.step(Int64.ofInt(pathTick++));
    check(Lambda.exists(pathMotion.firedEvents(), function(event) return switch event.value {
      case ProcessEventValue.Digital(enabled): !enabled;
      case _: false;
    }), "manipulator abort restores the safe process value");
    pathSimulation.dispose();
  }

  static function testKinematicsContract():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(new Manipulator(fixture.model, fixture.chain), 1e-8);
    check(solver.jointCount() == 6, "kinematics adapter reports the manipulator joint count");

    var q = [0.3, -0.5, 0.8, -0.2, 0.6, -0.4];
    var target = solver.forward(q);
    var seed = [for (value in q) value + 0.03];
    var tolerance = new IkTolerance(1e-5, 1e-4, 200, 0.02, 1e-3);
    var solved = solver.solvePose(target, seed, tolerance);
    check(solved != null, "kinematics adapter solves a reachable TCP pose");
    var achieved = solver.forward(cast solved);
    near(achieved.x, target.x, "forward/solve round trip preserves TCP x", 1e-5);
    near(achieved.y, target.y, "forward/solve round trip preserves TCP y", 1e-5);
    near(achieved.z, target.z, "forward/solve round trip preserves TCP z", 1e-5);
    near(Math.abs(achieved.qx * target.qx + achieved.qy * target.qy +
      achieved.qz * target.qz + achieved.qw * target.qw), 1.0,
      "forward/solve round trip preserves TCP orientation", 1e-4);

    var zeroTarget = solver.forward([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    var firstCandidates = solver.sampleCandidates(zeroTarget, 4, tolerance);
    var secondCandidates = solver.sampleCandidates(zeroTarget, 4, tolerance);
    check(firstCandidates.length > 0 && firstCandidates.length == secondCandidates.length,
      "candidate sampling returns a deterministic non-empty set");
    for (candidate in 0...firstCandidates.length) {
      check(firstCandidates[candidate].length == 6,
        "candidate sampling returns complete joint vectors");
      for (joint in 0...6)
        near(firstCandidates[candidate][joint], secondCandidates[candidate][joint],
          "candidate sampling is identical for identical inputs", 1e-12);
    }

    var expectedQdot = [0.08, -0.04, 0.05, 0.03, -0.02, 0.06];
    var jacobian = fixture.chain.jacobian(q);
    var requested:Array<Float> = [];
    for (row in 0...6) {
      var value = 0.0;
      for (joint in 0...6) value += jacobian[row][joint] * expectedQdot[joint];
      requested.push(value);
    }
    var qdot = solver.solveDifferential(q, new Twist6(requested[0], requested[1], requested[2],
      requested[3], requested[4], requested[5]));
    check(qdot != null, "differential IK solves a reachable tool twist");
    var epsilon = 1e-6;
    var plus = q.copy();
    var minus = q.copy();
    for (joint in 0...6) {
      plus[joint] += cast(qdot, Array<Float>)[joint] * epsilon;
      minus[joint] -= cast(qdot, Array<Float>)[joint] * epsilon;
    }
    var posePlus = solver.forward(plus);
    var poseMinus = solver.forward(minus);
    near((posePlus.x - poseMinus.x) / (2.0 * epsilon), requested[0],
      "differential IK linear x matches a finite difference", 1e-5);
    near((posePlus.y - poseMinus.y) / (2.0 * epsilon), requested[1],
      "differential IK linear y matches a finite difference", 1e-5);
    near((posePlus.z - poseMinus.z) / (2.0 * epsilon), requested[2],
      "differential IK linear z matches a finite difference", 1e-5);
    var angular = poseRotationDelta(poseMinus, posePlus, 1.0 / (2.0 * epsilon));
    near(angular[0], requested[3], "differential IK angular x matches a finite difference", 1e-5);
    near(angular[1], requested[4], "differential IK angular y matches a finite difference", 1e-5);
    near(angular[2], requested[5], "differential IK angular z matches a finite difference", 1e-5);

    throws(function() new Pose3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0),
      "MotionKit pose rejects a non-unit quaternion");
    throws(function() new Twist6(Math.NaN, 0.0, 0.0, 0.0, 0.0, 0.0),
      "MotionKit twist rejects non-finite components");
    throws(function() new IkTolerance(0.0, 1e-3),
      "IK tolerance rejects a non-positive position tolerance");
  }

  static function testMotionProgramContracts():Void {
    var pose = new Pose3(0.2, 0.1, 0.3);
    var path = new PosePath("work", [new PoseLine(
      new PoseWaypoint(new Pose3(0.0, 0.0, 0.0), 0.005, 0.02),
      new PoseWaypoint(new Pose3(0.1, 0.0, 0.0), 0.005, 0.02),
      OrientationPolicy.Fixed, 0.1, 0.1)]);
    var events = [new PathEvent(0.02, "sprayer.enabled", EventValue.Digital(true)),
      new PathEvent(0.08, "sprayer.enabled", EventValue.Digital(false))];
    var program = new MotionProgram([
      MotionOp.MoveJ(MoveTarget.JointTarget([0.1, -0.2]), new MotionOptions(1.0, 2.0),
        Blend.ExactStop),
      MotionOp.MoveL(pose, "work", 0.2, Blend.ToleranceBlend(0.002)),
      MotionOp.MoveC(new Pose3(0.25, 0.15, 0.3), new Pose3(0.3, 0.1, 0.3),
        "work", 0.15, Blend.ExactStop),
      MotionOp.FollowPath(path, "work", 0.1, events),
      MotionOp.SetOutput("sprayer.enabled", EventValue.Digital(false)),
      MotionOp.WaitInput("sprayer.ready", InputPredicate.Equals(EventValue.Digital(true)), 2.0)
    ]);
    check(program.ops.length == 6, "motion program retains its ordered operations");
    check(MotionProgram.validate(program.ops) == null,
      "a structurally valid motion program has no validation error");

    var emptyError = MotionProgram.validate([]);
    check(emptyError != null && emptyError.indexOf("at least one operation") >= 0,
      "empty program validation explains the missing operation");
    var frameError = MotionProgram.validate([
      MotionOp.MoveL(pose, " ", 0.1, Blend.ExactStop)]);
    check(frameError != null && frameError.indexOf("op 0") >= 0 &&
      frameError.indexOf("frame") >= 0,
      "program validation identifies an operation with a missing frame");
    var feedError = MotionProgram.validate([
      MotionOp.MoveL(pose, "work", Math.NaN, Blend.ExactStop)]);
    check(feedError != null && feedError.indexOf("feed") >= 0,
      "program validation rejects a non-finite feed");
    var blendError = MotionProgram.validate([
      MotionOp.MoveL(pose, "work", 0.1, Blend.ToleranceBlend(0.01)),
      MotionOp.SetOutput("sprayer.enabled", EventValue.Digital(false))]);
    check(blendError != null && blendError.indexOf("consecutive moves") >= 0,
      "program validation keeps tolerance blends between moves");
    var timeoutError = MotionProgram.validate([
      MotionOp.WaitInput("sprayer.ready", InputPredicate.Equals(EventValue.Digital(true)), 0.0)]);
    check(timeoutError != null && timeoutError.indexOf("timeout") >= 0,
      "program validation rejects a non-positive wait timeout");
    var eventOrderError = MotionProgram.validate([
      MotionOp.FollowPath(path, "work", 0.1, [events[1], events[0]])]);
    check(eventOrderError != null && eventOrderError.indexOf("sorted") >= 0,
      "program validation rejects unsorted path events");
    throws(function() new MotionProgram([]),
      "motion program construction rejects invalid structure");
  }

  static function testSimplePathTimingContract():Void {
    var path = new JointPathSamples([0.0, 0.4, 1.0], [[0.0], [0.4], [1.0]],
      [[1.0], [1.0], [1.0]], [[0.0], [0.0], [0.0]]);
    var limits = new PathTimingLimits([0.4], [0.8], [0.3, 0.25], 0.0, 0.0);
    var backend:PathTimingBackend = new SimplePathTiming(0.01);
    var timed = backend.time(path, limits);
    near(timed.trajectory.evaluate(0.0).positions[0], 0.0,
      "simple path timing starts at the authored joint position", 1e-12);
    near(timed.trajectory.evaluate(timed.trajectory.durationSeconds()).positions[0], 1.0,
      "simple path timing reaches the authored joint endpoint", 1e-12);
    near(timed.distanceToTime(0.0), 0.0,
      "distance-to-time map hits the path start exactly", 1e-12);
    near(timed.distanceToTime(1.0), timed.trajectory.durationSeconds(),
      "distance-to-time map hits the path end exactly", 1e-9);
    var previousTime = -1.0;
    for (sample in 0...21) {
      var time = timed.distanceToTime(sample / 20.0);
      check(time >= previousTime, "distance-to-time map is monotonic");
      previousTime = time;
    }
    var previousVelocity:Null<Float> = null;
    var previousDuration = 0.0;
    for (segment in timed.trajectory.segments()) {
      var velocity = segment.coefficients[0][1];
      check(Math.abs(velocity) <= 0.3000001,
        "simple path timing respects the tightest span speed cap");
      var duration = Int64.toFloat(segment.durationNs) * 1e-9;
      if (previousVelocity != null) {
        var averageDuration = 0.5 * (previousDuration + duration);
        check(Math.abs(velocity - previousVelocity) <= 0.8 * averageDuration + 0.001,
          "simple path timing respects joint-derived acceleration between chords");
      }
      previousVelocity = velocity;
      previousDuration = duration;
    }
    check(timed.bindingConstraints.length > 0,
      "simple path timing reports a binding constraint");
    timed.trajectory.dispose();

    throws(function() new JointPathSamples([0.0, 0.0], [[0.0], [1.0]],
      [[1.0], [1.0]], [[0.0], [0.0]]),
      "joint path samples require strictly increasing path positions");
    throws(function() new PathTimingLimits([0.0], [1.0]),
      "path timing requires positive joint velocity limits");
  }

  static function testOpwKinematics():Void {
    var model = new RobotModel("opw-abb-test");
    var links = [for (index in 0...7) model.addLink(new Link('opw-link-$index'))];
    var positions = [[0.0, 0.0, 0.0], [0.1, 0.0, 0.615],
      [0.0, 0.0, 0.705], [-0.135, 0.0, 0.755],
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
    var axes = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0],
      [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
      [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for (index in 0...6) {
      var joint = model.addJoint(new Joint('opw-joint-$index', JointType.Revolute,
        links[index], links[index + 1]));
      joint.parentFramePosition = positions[index];
      joint.axis = axes[index];
      joint.limits.lower = -2.0 * Math.PI;
      joint.limits.upper = 2.0 * Math.PI;
    }
    var flange = model.addFrame(new Frame("opw-flange", links[6]));
    flange.position = [0.0, 0.0, 0.085];
    var chain = new KinematicChain(model, links[0].id, ChainTip.Frame(flange.id));
    var manipulator = new Manipulator(model, chain);
    var solver = new OpwKinematics(model, manipulator);
    near(solver.parameters.a1, 0.1, "OPW extracts a1", 1e-9);
    near(solver.parameters.a2, -0.135, "OPW extracts a2", 1e-9);
    near(solver.parameters.c1, 0.615, "OPW extracts c1", 1e-9);
    var q = [0.2, -0.3, 0.4, 0.5, -0.6, 0.7];
    var reference = new ManipulatorKinematics(manipulator).forward(q);
    var actual = solver.forward(q);
    near(actual.x, reference.x, "OPW forward agrees with RobotKit X", 1e-9);
    near(actual.y, reference.y, "OPW forward agrees with RobotKit Y", 1e-9);
    near(actual.z, reference.z, "OPW forward agrees with RobotKit Z", 1e-9);
    var candidates = solver.sampleCandidates(reference, 8, new IkTolerance());
    var found = false;
    for (candidate in candidates) {
      var error = 0.0;
      for (index in 0...6)
        error += Math.abs(candidate[index] - q[index]);
      if (error < 1e-8) found = true;
    }
    check(found, "OPW analytic candidates include the authored joint pose");
    var next = q.copy(); next[0] += 0.04;
    var selector = new PathConfigurationSelector(solver,
      [for (_ in 0...6) -2.0 * Math.PI],
      [for (_ in 0...6) 2.0 * Math.PI],
      [for (_ in 0...6) 0.5], [for (_ in 0...6) 1.0]);
    var chosen = selector.selectPoses([0.0, 0.04],
      [solver.forward(q), solver.forward(next)], q, new IkTolerance());
    near(chosen[1][0], next[0],
      "Descartes samples OPW branches natively across a path", 1e-6);
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(0));
    var compiler = new ProgramCompiler(solver, limits, "work",
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 4.0],
      [for (_ in 0...6) 20.0]);
    check(compiler.configurationSelector != null,
      "analytic arm programs use Descartes configuration selection");
    var program = new MotionProgram([MotionOp.MoveL(solver.forward(next),
      "work", 0.1, Blend.ExactStop)]);
    var compiled = compiler.compile(program, q, Int64.ofInt(901));
    check(compiled.blocks[0].plans.length == 1,
      "OPW arm path compiles through the shared ProgramCompiler");
    compiled.dispose();
    function published(name:String, values:Array<Float>, offsets:Array<Float>,
        signs:Array<Int>):Void {
      var fixture = new RobotModel(name);
      var parts = [for (index in 0...7) fixture.addLink(new Link('$name-$index'))];
      var origins = [[0.0, 0.0, 0.0], [values[0], values[2], values[3]],
        [0.0, 0.0, values[4]], [values[1], 0.0, values[5]],
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
      for (index in 0...6) {
        var direction = index == 0 || index == 3 || index == 5
          ? new Vec3(0.0, 0.0, 1.0) : new Vec3(0.0, 1.0, 0.0);
        var joint = fixture.addJoint(new Joint('$name-joint-$index',
          JointType.Revolute, parts[index], parts[index + 1]));
        joint.parentFramePosition = origins[index];
        joint.parentFrameRotation = Quat.fromAxisAngle(direction,
          -offsets[index]).toArray();
        joint.axis = direction.scale(signs[index]).toArray();
        joint.limits.lower = -2.0 * Math.PI;
        joint.limits.upper = 2.0 * Math.PI;
      }
      var tool = fixture.addFrame(new Frame('$name-flange', parts[6]));
      tool.position = [0.0, 0.0, values[6]];
      var chain = new KinematicChain(fixture, parts[0].id, ChainTip.Frame(tool.id));
      var robot = new Manipulator(fixture, chain);
      var analytic = new OpwKinematics(fixture, robot);
      for (index in 0...7) {
        var extracted = [analytic.parameters.a1, analytic.parameters.a2,
          analytic.parameters.b, analytic.parameters.c1, analytic.parameters.c2,
          analytic.parameters.c3, analytic.parameters.c4][index];
        near(extracted, values[index], '$name OPW parameter $index', 1e-9);
      }
      for (index in 0...6) {
        near(analytic.parameters.offsets[index], offsets[index],
          '$name OPW offset $index', 1e-9);
        check(analytic.parameters.signCorrections[index] == signs[index],
          '$name OPW sign $index');
      }
      var probe = [0.17, -0.24, 0.32, -0.41, 0.53, -0.68];
      var expected = new ManipulatorKinematics(robot).forward(probe);
      var actual = analytic.forward(probe);
      near(actual.x, expected.x, '$name FK X', 1e-9);
      near(actual.y, expected.y, '$name FK Y', 1e-9);
      near(actual.z, expected.z, '$name FK Z', 1e-9);
    }
    published("ABB IRB2400", [0.1, -0.135, 0.0, 0.615, 0.705, 0.755, 0.085],
      [0.0, 0.0, -Math.PI * 0.5, 0.0, 0.0, 0.0], [1, 1, 1, 1, 1, 1]);
    published("KUKA KR6", [0.025, -0.035, 0.0, 0.4, 0.315, 0.365, 0.08],
      [0.0, -Math.PI * 0.5, 0.0, 0.0, 0.0, 0.0], [-1, 1, 1, -1, 1, -1]);
    published("Fanuc R2000", [0.72, -0.225, 0.0, 0.6, 1.075, 1.28, 0.235],
      [0.0, 0.0, -Math.PI * 0.5, 0.0, 0.0, 0.0], [1, 1, 1, 1, 1, 1]);
    published("Stäubli TX40", [0.0, 0.0, 0.035, 0.32, 0.225, 0.225, 0.065],
      [0.0, 0.0, -Math.PI * 0.5, 0.0, 0.0, 0.0], [1, 1, 1, 1, 1, 1]);
    var bad = buildContractArmFixture();
    var diagnostic = "";
    try new OpwKinematics(bad.model, new Manipulator(bad.model, bad.chain))
    catch (error:Dynamic) diagnostic = Std.string(error);
    check(diagnostic.indexOf("joint-3") >= 0,
      "non-spherical UR5 wrist names its violating joint");
  }

  static function buildContractArmFixture():{model:RobotModel, chain:KinematicChain} {
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

  static function poseRotationDelta(from:Pose3, to:Pose3, scale:Float):Array<Float> {
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

  static function testGeometricPathPrimitives():Void {
    var path = GeometricPath.lines([new PathPoint(0.0, 0.0, 0.0),
      new PathPoint(0.1, 0.0, 0.0), new PathPoint(0.1, 0.1, 0.0)]);
    near(path.totalLength, 0.2, "line path accumulates primitive lengths");
    near(path.pointAt(0.05).x, 0.05, "line path samples its first primitive");
    near(path.pointAt(0.15).y, 0.05, "line path samples its second primitive");

    var arc = new ArcSegment(new PathPoint(0.1, 0.1, 0.0), 0.1,
      -Math.PI * 0.5, Math.PI * 0.5);
    near(arc.start.x, 0.1, "arc starts at its authored angle");
    near(arc.start.y, 0.0, "arc starts on its authored circle");
    near(arc.end.x, 0.2, "arc ends at its swept angle");
    near(arc.end.y, 0.1, "arc ends on its authored circle");
    near(arc.tangentAt(0.0)[0], 1.0, "arc tangent follows increasing distance");
    near(arc.tangentAt(arc.length())[1], 1.0, "arc tangent rotates with the circle");
  }

  static function testNativeTrajectoryRoundTrip():Void {
    var native = Trajectory.fromPositionSamples([0.0, 1.0, 2.0],
      [[0.0], [2.0], [5.0]]);
    near(native.durationSeconds(), 2.0, "native trajectory duration");
    check(native.jointCount() == 1, "native trajectory joint count");
    for (tick in 0...2001) {
      var time = tick * 0.001;
      var expected = time <= 1.0 ? 2.0 * time : 2.0 + 3.0 * (time - 1.0);
      near(native.evaluate(time).positions[0], expected,
        "native degree-1 positions match authored chords", 1e-12);
    }
    near(native.evaluate(0.5).velocities[0], 2.0,
      "native velocity is the chord slope, not authored sample velocity");
    near(native.evaluate(1.0).velocities[0], 3.0,
      "native velocity is right-continuous at a knot");
    near(native.evaluate(0.5).accelerations[0], 0.0,
      "native degree-1 acceleration is zero");
    var estimate = native.estimatePathDerivatives(0.5, 1.0);
    near(estimate.velocities[0], 2.0, "degree-1 stop estimate uses chord velocity");
    near(estimate.accelerations[0], 1.0,
      "degree-1 stop estimate sees a later chord velocity");
    native.dispose();

    var deduplicated = Trajectory.fromPositionSamples([0.0, 0.0, 1.0],
      [[0.0], [0.0], [1.0]]);
    check(deduplicated.segments().length == 1,
      "identical coincident samples do not create zero-duration segments");
    deduplicated.dispose();
    throws(() -> {
      Trajectory.fromPositionSamples([0.0, 0.0, 1.0],
        [[0.0], [0.1], [1.0]]);
    }, "conflicting positions at one timestamp remain invalid");

    var longTrajectory = Trajectory.fromPositionSamples([0.0, 3.0],
      [[0.0], [3.0]]);
    check(Int64.compare(Trajectory.nanoseconds(2.21), Int64.parseString("2210000000")) == 0,
      'times past the signed 32-bit nanosecond boundary retain their value: '
        + '${Int64.toStr(Trajectory.nanoseconds(2.21))}');
    near(longTrajectory.evaluate(2.5).positions[0], 2.5,
      "long native trajectory evaluates beyond 2.147 seconds");
    var longLimits = new ValidationLimits(1, Int64.ofInt(12), Int64.ofInt(3));
    longLimits.position(0, 0.0, 3.0);
    longLimits.velocity(0, 1.1);
    var longPlan = ExecutionPlan.create(longTrajectory, longLimits, Int64.ofInt(45),
      [0.0], [0.0], [0.0], [0.01], [0.01], [0.01]);
    near(longPlan.evaluate(2.5).positions[0], 2.5,
      "long execution plan evaluates beyond 2.147 seconds");
    longPlan.dispose();
    longTrajectory.dispose();
  }

  static function testNativeValidationAndPlan():Void {
    var trajectory = Trajectory.fromPositionSamples([0.0, 1.0], [[0.0], [1.0]]);
    var limits = new ValidationLimits(1, Int64.ofInt(12), Int64.ofInt(3));
    limits.position(0, 0.0, 1.0);
    limits.velocity(0, 0.8);
    var report = trajectory.validate(limits);
    check(report.hasFailure(), "chord speed above claimed limit fails validation");
    near(report.checks[MotionKitNativeConstants.MK_CHECK_VELOCITY].value, 1.0,
      "validation records chord speed");
    near(report.checks[MotionKitNativeConstants.MK_CHECK_VELOCITY].margin, -0.2,
      "validation reports signed limit margin");
    near(report.checks[MotionKitNativeConstants.MK_CHECK_VELOCITY].tolerance, 0.8e-9,
      "validation reports comparison tolerance", 1e-12);
    check(report.checks[MotionKitNativeConstants.MK_CHECK_JERK].status ==
      MotionKitNativeConstants.MK_CHECK_UNCHECKED, "unclaimed jerk is unchecked");
    check(report.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE].status ==
      MotionKitNativeConstants.MK_CHECK_UNCHECKED, "task-space slot is reserved");
    check(Int64.compare(
      report.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE].resolutionNs,
      Int64.ofInt(0)) == 0, "unset task-space check has no sampling resolution");
    report.setTaskSpace(MotionKitNativeConstants.MK_CHECK_FAILED, 0.006, 0.75,
      0.005, Int64.ofInt(1000000));
    var taskSpace = report.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE];
    check(taskSpace.status == MotionKitNativeConstants.MK_CHECK_FAILED,
      "Haxe wrapper records task-space status");
    check(taskSpace.method == MotionKitNativeConstants.MK_CHECK_METHOD_SAMPLED,
      "task-space report identifies sampled validation");
    near(taskSpace.value, 0.006, "Haxe wrapper records worst task-space deviation");
    near(taskSpace.timeSeconds, 0.75, "Haxe wrapper records worst task-space time");
    near(taskSpace.limit, 0.005, "Haxe wrapper records task-space tolerance");
    check(Int64.compare(taskSpace.resolutionNs, Int64.ofInt(1000000)) == 0,
      "Haxe wrapper records task-space sampling resolution");
    check(Int64.compare(report.executorTimeResolutionNs, Int64.ofInt(1)) == 0,
      "host validation defaults to 1 ns");
    limits.timeResolutionNs(Int64.ofInt(2));
    report = trajectory.validate(limits);
    check(Int64.compare(report.executorTimeResolutionNs, Int64.ofInt(2)) == 0,
      "validation reports requested executor resolution");
    check(report.unresolvedAssumptions.length > 0 &&
      report.unresolvedAssumptions[0].length > 0,
      "unresolved assumptions are available to Haxe callers");
    try {
      ExecutionPlan.create(trajectory, limits, Int64.ofInt(44), [0.0], [0.0],
        [0.0], [0.01], [0.01], [0.01]);
      throw "expected plan limit rejection";
    } catch (error:PlanLimitError) {
      check(error.report.hasFailure(), "plan rejection carries validation report");
    }
    limits.velocity(0, 1.1);
    var timed = new TimedEvent(Int64.ofInt(500000000), "sprayer.flow",
      EventValue.Analog(1.25), HoldPolicy.RestoreOnResume);
    var plan = ExecutionPlan.create(trajectory, limits, Int64.ofInt(44), [0.0],
      [0.0], [0.0], [0.01], [0.01], [0.01], [timed]);
    check(!plan.report.hasFailure(), "valid plan has no failed check");
    check(plan.events.length == 1 && plan.events[0].channel == "sprayer.flow",
      "native execution plan retains timed events");
    near(plan.evaluate(0.5).positions[0], 0.5, "plan owns evaluable trajectory");
    trajectory.dispose();
    near(plan.evaluate(0.75).positions[0], 0.75, "plan deep copies trajectory");
    plan.dispose();
  }

  static function testPlannerIsDeterministicAndBounded():Void {
    var limits = new MotionLimits(1.0, 2.0, 10.0);
    function generate():Trajectory return Trajectory.generateStateToState([0.0, 0.0],
      [0.0, 0.0], [0.0, 0.0], [1.0, -0.25], [limits.maxVelocity, limits.maxVelocity],
      [limits.maxAcceleration, limits.maxAcceleration], [limits.maxJerk, limits.maxJerk]);
    var first = generate();
    var second = generate();
    check(first.durationSeconds() > 0.0, "planner produces a timed trajectory");
    near(first.durationSeconds(), second.durationSeconds(), "planner duration is deterministic");
    for (i in 0...101) {
      var time = first.durationSeconds() * i / 100.0;
      var a = first.evaluate(time);
      var b = second.evaluate(time);
      for (joint in 0...2) {
        near(a.positions[joint], b.positions[joint],
          "planner position is deterministic");
        check(Math.abs(a.velocities[joint]) <= limits.maxVelocity + 1e-6,
          "planner respects velocity limit");
        check(Math.abs(a.accelerations[joint]) <= limits.maxAcceleration + 1e-6,
          "planner respects acceleration limit");
      }
    }
    near(first.evaluate(0.0).positions[0], 0.0, "trajectory starts at the requested position");
    near(first.evaluate(first.durationSeconds()).positions[0], 1.0,
      "trajectory ends at the requested position");
    first.dispose();
    second.dispose();
  }

  static function testToppraExactStopsAndBindings():Void {
    var path = new JointPathSamples([0.0, 0.05, 0.1], [[0.0], [0.05], [0.1]],
      [[1.0], [1.0], [1.0]], [[0.0], [0.0], [0.0]]);
    var timed = new ToppraPathTiming(1e-8).time(path,
      new PathTimingLimits([0.2], [1.0]));
    near(timed.trajectory.evaluate(0.0).velocities[0], 0.0,
      "TOPP-RA exact stop begins at rest", 1e-8);
    near(timed.trajectory.evaluate(timed.trajectory.durationSeconds()).velocities[0], 0.0,
      "TOPP-RA exact stop ends at rest", 1e-8);
    check(timed.bindingConstraints.length > 0,
      "TOPP-RA records a binding joint constraint");
    check(Lambda.exists(timed.bindingConstraints, binding ->
      binding.jointIndex == 0 && binding.kind == BindingConstraintKind.JointVelocity),
      "TOPP-RA names the saturated joint and velocity limit");
    timed.releaseDistanceMap();
    timed.trajectory.dispose();
    var sprint = new ToppraPathTiming(1e-8).time(path,
      new PathTimingLimits([10.0], [2.0]));
    var analyticTwoLegs = 4.0 * Math.sqrt(0.1 / 2.0);
    var sprintDuration = sprint.trajectory.durationSeconds();
    check(2.0 * sprintDuration <= analyticTwoLegs * 1.01,
      "TOPP-RA straight exact stops stay within 1% of analytic minimum");
    sprint.releaseDistanceMap();
    sprint.trajectory.dispose();
    var capped = new ToppraPathTiming(1e-8).time(path,
      new PathTimingLimits([10.0], [2.0], [0.2, 0.2]));
    check(capped.trajectory.durationSeconds() > sprintDuration,
      "TOPP-RA feed cap lengthens a straight move");
    check(Lambda.exists(capped.bindingConstraints, binding ->
      binding.kind == BindingConstraintKind.SpeedCap && binding.jointIndex == -1),
      "TOPP-RA identifies an authored feed cap");
    capped.releaseDistanceMap();
    capped.trajectory.dispose();
  }

  static function testToppraCircleAcceleration():Void {
    var radius = 0.1;
    var total = 2.0 * Math.PI * radius;
    var distances:Array<Float> = [];
    var positions:Array<Array<Float>> = [];
    var first:Array<Array<Float>> = [];
    var second:Array<Array<Float>> = [];
    for (index in 0...17) {
      var angle = 2.0 * Math.PI * index / 16.0;
      distances.push(total * index / 16.0);
      positions.push([radius * Math.cos(angle), radius * Math.sin(angle)]);
      first.push([-Math.sin(angle), Math.cos(angle)]);
      second.push([-Math.cos(angle) / radius, -Math.sin(angle) / radius]);
    }
    var timed = new ToppraPathTiming().time(
      new JointPathSamples(distances, positions, first, second),
      new PathTimingLimits([2.0, 2.0], [1.0, 1.0]));
    var peak = 0.0;
    var peakAngle = 0.0;
    for (index in 0...201) {
      var state = timed.trajectory.evaluate(timed.trajectory.durationSeconds() * index / 200.0);
      var speed = Math.sqrt(state.velocities[0] * state.velocities[0] +
        state.velocities[1] * state.velocities[1]);
      if (speed > peak) {
        peak = speed;
        peakAngle = Math.atan2(state.positions[1], state.positions[0]);
      }
      check(Math.abs(state.accelerations[0]) <= 1.001 &&
        Math.abs(state.accelerations[1]) <= 1.001,
        "TOPP-RA circle respects both joint acceleration budgets");
    }
    var peakBound = Math.sqrt(radius /
      Math.max(Math.abs(Math.cos(peakAngle)), Math.abs(Math.sin(peakAngle))));
    check(peak <= peakBound * 1.01,
      'TOPP-RA circle peak speed $peak respects per-joint bound $peakBound');
    timed.releaseDistanceMap();
    timed.trajectory.dispose();
  }

  static function testLinearAxisCompilesToRobotModel():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    check(blueprint.model.links.length == 2, "linear axis compiles base and carriage links");
    check(blueprint.model.joints.length == 1, "linear axis compiles one prismatic joint");
    var joint = blueprint.model.joints[0];
    check(joint.id == "x" && joint.name == "x", "linear axis uses a stable joint ID");
    check(joint.type == robotkit.model.JointType.Prismatic, "linear axis is prismatic");
    near(joint.parentFramePosition[2], (axis.screwStart + axis.travelMin) * 0.001,
      "MachineKit travel origin is converted to metres");
    near(joint.limits.upper, 0.08, "MachineKit stroke becomes the logical upper limit");
    near(joint.limits.velocity, 0.1, "compiled actuator rate is retained");
    check(blueprint.model.actuators.length == 1 &&
      blueprint.model.actuators[0].id.indexOf(axis.motor.designation) >= 0,
      "compiled actuator retains motor identity");
    var expectedRatio = 2.0 * Math.PI /
      (axis.nut.travelPerRevolution() * MachineKitRobotCompiler.MILLIMETRES_TO_METRES);
    check(switch blueprint.model.actuators[0].transmission {
      case SimpleTransmission(jointId, ratio, offset):
        jointId == joint.id && Math.abs(ratio - expectedRatio) < 1e-9 && offset == 0.0;
    }, "compiled actuator carries the lead-screw rad/m ratio");
    check(expectedRatio < 0.0,
      "right-hand screw actuator rotates negative to move the carriage along +Z");
    near(blueprint.axes[0].jointScales[0], 1.0,
      "transmission-derived single-joint mapping keeps the old scale");
    near(blueprint.axes[0].jointOffsets[0], 0.0,
      "transmission-derived single-joint mapping keeps the old offset");
  }

  static function testLeadScrewActuatorRateLimitsPlans():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    var actuator = blueprint.model.actuators[0];
    var ratio = switch actuator.transmission {
      case SimpleTransmission(_, value, _): Math.abs(value);
    };
    actuator.maxRate = 0.02 * ratio;
    var limited = RobotRuntimeCompiler.compile(blueprint.model);
    near(limited.joints[0].maxRate, 0.02,
      "lead-screw motor rate converts to the tighter joint-space limit");
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(limited);
    var plan = new ExecutionPlanSubmission(Int64.ofInt(1), Int64.ofInt(limited.revision),
      Int64.ofInt(limited.calibrationRevision),
      RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,
      [0.0], [0.0], [0.0],
      [new TrajectorySegment(Int64.ofInt(0), Int64.ofInt(1000000000), [[0.0, 0.03]])]);
    var rejectedForLimit = false;
    try runtime.submitPlan(plan, 1) catch (error:Dynamic) {
      if (Std.isOfType(error, RobotRuntimeError)) {
        var nativeError:RobotRuntimeError = cast error;
        rejectedForLimit = nativeError.status == RobotKitRuntimeConstants.RK_ERROR_LIMIT;
      }
    }
    check(rejectedForLimit,
      "runtime rejects a plan faster than the converted motor rate limit");
    simulation.dispose();
  }

  static function testTransmissionDerivedAxisMapping():Void {
    var model = new RobotModel("dual-drive-map");
    var base = model.addLink(new Link("base"));
    var left = model.addLink(new Link("left"));
    var right = model.addLink(new Link("right"));
    var first = model.addJoint(new Joint("x.left", JointType.Prismatic, base, left));
    var second = model.addJoint(new Joint("x.right", JointType.Prismatic, left, right));
    first.limits.lower = 0.0;
    first.limits.upper = 0.08;
    second.limits.lower = -0.16;
    second.limits.upper = 0.01;
    model.addActuator(new Actuator("left-motor", 0.0, 1.0,
      Transmission.SimpleTransmission(first.id, 1.0, 0.0)));
    model.addActuator(new Actuator("right-motor", 0.0, 1.0,
      Transmission.SimpleTransmission(second.id, -0.5, 0.01)));
    var authored = new MotionAxisBlueprint("x", [first.id, second.id],
      0.0, 0.08, 0.08, 0.4);
    var derived = MotionSystemBlueprint.fromRobotModel(model, [authored]);
    near(derived.axes[0].jointScales[0], 1.0,
      "first transmitted joint defines the logical coordinate");
    near(derived.axes[0].jointScales[1], -2.0,
      "transmission ratio derives the old dual-motor scale");
    near(derived.axes[0].jointOffsets[1], 0.01,
      "transmission offset is retained in joint coordinates");
    var explicit = new MotionAxisBlueprint("x", [first.id, second.id],
      0.0, 0.08, 0.08, 0.4, 0.0, [1.0, -3.0], [0.0, 0.02]);
    var overridden = MotionSystemBlueprint.fromRobotModel(model, [explicit]);
    near(overridden.axes[0].jointScales[1], -3.0,
      "deprecated explicit scale still overrides the transmission");
    near(overridden.axes[0].jointOffsets[1], 0.02,
      "deprecated explicit offset still overrides the transmission");
    model.addCoupling(new JointCoupling("gantry-gears", first.id, second.id, -1.5, 0.02));
    var coupled = MotionSystemBlueprint.fromRobotModel(model, [authored]);
    near(coupled.axes[0].jointScales[1], -1.5,
      "model joint coupling derives the axis follower scale");
    near(coupled.axes[0].jointOffsets[1], 0.02,
      "model joint coupling derives the axis follower offset");
    check(coupled.runtime.couplings.length == 1,
      "runtime blueprint retains the model joint coupling");
    model.actuators.pop();
    var leaderDriven = MotionSystemBlueprint.fromRobotModel(model, [authored]);
    near(leaderDriven.axes[0].jointScales[1], -1.5,
      "one transmitted leader drives a coupled follower axis");
  }

  static function testAxisKinematics():Void {
    var model = new RobotModel("coupled-xyz");
    var links = [for (index in 0...5) model.addLink(new Link('axis-link-$index'))];
    var ids = ["x.leader", "x.follower", "y", "z"];
    for (index in 0...4) {
      var joint = model.addJoint(new Joint(ids[index], JointType.Prismatic,
        links[index], links[index + 1]));
      joint.limits.lower = -0.1;
      joint.limits.upper = 0.1;
    }
    model.addCoupling(new JointCoupling("x-gears", "x.leader",
      "x.follower", -1.5, 0.02));
    var blueprint = MotionSystemBlueprint.fromRobotModel(model, [
      new MotionAxisBlueprint("x", ["x.leader", "x.follower"], -0.05, 0.05,
        0.1, 0.4),
      new MotionAxisBlueprint("y", ["y"], -0.05, 0.05, 0.1, 0.4),
      new MotionAxisBlueprint("z", ["z"], -0.05, 0.05, 0.1, 0.4)]);
    var solver = new AxisKinematics(blueprint);
    var target = new Pose3(0.01, 0.02, -0.03);
    var q = solver.solvePose(target, [0.0, 0.02, 0.0, 0.0], new IkTolerance());
    check(q != null, "axis IK reaches a pose within logical limits");
    if (q == null) throw "axis IK returned no solution";
    near(q[0], 0.01, "axis IK maps the X leader");
    near(q[1], 0.005, "axis IK keeps the coupled follower in proportion");
    near(solver.forward(q).y, 0.02, "axis FK maps Y to the tool");
    check(solver.sampleCandidates(target, 8, new IkTolerance()).length == 1,
      "axis IK exposes one exact candidate");
    var velocity = solver.solveDifferential(q,
      new Twist6(0.02, 0.01, -0.01, 0.0, 0.0, 0.0));
    check(velocity != null, "axis differential IK maps a linear twist");
    if (velocity == null) throw "axis differential IK returned no solution";
    near(velocity[0], 0.02, "axis differential IK maps X velocity");
    near(velocity[1], -0.03,
      "axis differential IK keeps follower velocity in proportion");
    check(solver.solvePose(new Pose3(0.06), q, new IkTolerance()) == null,
      "axis IK rejects poses beyond logical limits");
  }

  static function testCompiledAxisRunsThroughSimulation():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("single-axis", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);

    machine.home();
    check(machine.isMoving(), "home creates a motion command");
    machine.update();
    simulation.step(Int64.ofInt(0));
    check(!machine.isMoving(), "zero-distance home completes deterministically");

    machine.moveAxes([new AxisTarget("x", 0.04)], new MotionOptions(0.08, 0.4));
    var tick = 1;
    while (machine.isMoving()) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
      if (tick > 1000) throw "single-axis trajectory did not complete";
    }
    for (_ in 0...4) simulation.step(Int64.ofInt(tick++));
    var snapshot = robot.snapshot();
    near(snapshot.positions.get(0), 0.04, "simulated robot reaches the MotionKit axis target", 1e-5);
    simulation.dispose();
  }

  static function testHomingAndJogging():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("jog-axis", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);

    machine.home();
    runMotion(machine, simulation);
    near(robot.snapshot().positions.get(0), 0.0, "homing returns the axis to its authored home", 1e-5);

    var forward = planned(machine.jog("x", 0.02, 1.0));
    near(forward.evaluate(0.0).velocities[0], 0.0,
      "jog starts at rest");
    near(forward.evaluate(forward.durationSeconds()).velocities[0], 0.0,
      "jog ends at rest");
    for (sample in trajectoryStates(forward))
      check(Math.abs(sample.accelerations[0]) <= 0.4 + 1e-9,
        "jog respects its acceleration limit");
    near(forward.evaluate(forward.durationSeconds()).positions[0], 0.02,
      "jog plans the requested logical displacement");
    runMotion(machine, simulation);
    near(robot.snapshot().positions.get(0), 0.02,
      "positive jog reaches its target", 1e-5);

    machine.jog("x", -0.01, 0.5);
    runMotion(machine, simulation);
    near(robot.snapshot().positions.get(0), 0.015,
      "negative jog follows the same logical axis API", 1e-5);

    var clamped = planned(machine.jog("x", 0.1, 2.0));
    near(clamped.evaluate(clamped.durationSeconds()).positions[0], 0.08,
      "jog clamps its endpoint to the authored upper limit");
    runMotion(machine, simulation);
    near(robot.snapshot().positions.get(0), 0.08,
      "clamped jog stops at the axis limit", 1e-5);
    throws(function() machine.jog("x", 0.1001, 1.0),
      "jog rejects a velocity above the axis rate limit");
    throws(function() machine.jog("x", 0.0, 1.0),
      "jog rejects a zero velocity");
    simulation.dispose();
  }

  static function testMoveLinearUsesPlannerLimits():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 80), new LinearAxis(23, 10, 80),
      new LinearAxis(23, 10, 80), 0.2, 2.0);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("linear-planner-limits", runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var options = new MotionOptions(0.2, 2.0);
    var xOnly = planned(machine.moveLinear(Pose.xyz(0.02, 0.0, 0.0),
      Feed.metresPerSecond(0.2), options));
    var peakAcceleration = peakChordAcceleration(xOnly, 0);
    check(peakAcceleration > 1.9,
      "moveLinear uses an authored 2 m/s² acceleration limit");

    runMotion(machine, simulation);

    var diagonal = planned(machine.moveLinear(Pose.xyz(0.03, 0.03, 0.03),
      Feed.metresPerSecond(0.2), options));
    for (joint in 0...3)
      check(peakChordAcceleration(diagonal, joint) <= 2.0 + 1e-6,
        "diagonal moveLinear chords stay within per-axis acceleration caps");
    simulation.dispose();
  }

  static function testCompiledXYZGantryRunsThroughSimulation():Void {
    var xAxis = new LinearAxis(23, 10, 80);
    var yAxis = new LinearAxis(23, 10, 60);
    var zAxis = new LinearAxis(23, 10, 40);
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(xAxis, yAxis, zAxis, 0.1, 0.4);
    check(blueprint.model.links.length == 4, "XYZ gantry compiles one base and three carriages");
    check(blueprint.model.joints.length == 3, "XYZ gantry compiles three prismatic joints");
    for (i in 0...3) {
      var joint = blueprint.model.joints[i];
      check(joint.id == ["x", "y", "z"][i], "XYZ gantry uses stable joint IDs");
      check(joint.type == robotkit.model.JointType.Prismatic,
        "XYZ gantry joints are prismatic");
      near(joint.limits.velocity, 0.1, "XYZ gantry retains the actuator rate limit");
    }
    near(blueprint.model.joints[0].parentFramePosition[0],
      (xAxis.screwStart + xAxis.travelMin) * 0.001, "X carriage frame is compiled in metres");
    near(blueprint.model.joints[1].parentFramePosition[1],
      (yAxis.screwStart + yAxis.travelMin) * 0.001, "Y carriage frame is compiled in metres");
    near(blueprint.model.joints[2].parentFramePosition[0],
      -(zAxis.screwStart + zAxis.travelMin) * 0.001,
      "Z carriage frame compensates the inherited gantry orientation");

    var chain = new KinematicChain(blueprint.model, "gantry.base",
      ChainTip.Link("z.carriage"));
    var tip = chain.forwardKinematics([0.0, 0.0, 0.0]).translation;
    near(tip.x, (xAxis.screwStart + xAxis.travelMin) * 0.001,
      "XYZ gantry forward kinematics preserves X origin");
    near(tip.y, (yAxis.screwStart + yAxis.travelMin) * 0.001,
      "XYZ gantry forward kinematics preserves Y origin");
    near(tip.z, (zAxis.screwStart + zAxis.travelMin) * 0.001,
      "XYZ gantry forward kinematics preserves Z origin");
    var jacobian = chain.jacobian([0.0, 0.0, 0.0]);
    near(jacobian[0][0], 1.0, "XYZ gantry X joint moves along world X");
    near(jacobian[1][1], 1.0, "XYZ gantry Y joint moves along world Y");
    near(jacobian[2][2], 1.0, "XYZ gantry Z joint moves along world Z");
    near(jacobian[1][0], 0.0, "XYZ gantry X joint has no world Y component");
    near(jacobian[2][0], 0.0, "XYZ gantry X joint has no world Z component");
    near(jacobian[0][1], 0.0, "XYZ gantry Y joint has no world X component");
    near(jacobian[2][1], 0.0, "XYZ gantry Y joint has no world Z component");
    near(jacobian[0][2], 0.0, "XYZ gantry Z joint has no world X component");
    near(jacobian[1][2], 0.0, "XYZ gantry Z joint has no world Y component");

    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("xyz-gantry", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    check(machine.axis("x") != null && machine.axis("y") != null && machine.axis("z") != null,
      "XYZ gantry exposes all logical axes");

    machine.home();
    runMotion(machine, simulation);
    var home = robot.snapshot();
    near(home.positions.get(0), 0.0, "XYZ gantry homes X");
    near(home.positions.get(1), 0.0, "XYZ gantry homes Y");
    near(home.positions.get(2), 0.0, "XYZ gantry homes Z");
    throws(function() machine.moveAxes([new AxisTarget("x", 0.081)]),
      "XYZ gantry rejects an out-of-range axis target");

    machine.moveAxes([
      new AxisTarget("x", 0.02), new AxisTarget("y", 0.01), new AxisTarget("z", 0.015)
    ], new MotionOptions(0.08, 0.4));
    runMotion(machine, simulation);
    var firstMove = robot.snapshot();
    near(firstMove.positions.get(0), 0.02, "XYZ gantry reaches X axis target", 1e-5);
    near(firstMove.positions.get(1), 0.01, "XYZ gantry reaches Y axis target", 1e-5);
    near(firstMove.positions.get(2), 0.015, "XYZ gantry reaches Z axis target", 1e-5);

    var linear = planned(machine.moveLinear(Pose.xyz(0.03, 0.02, 0.025), Feed.mmPerSecond(50)));
    var midpoint = linear.evaluate(linear.durationSeconds() * 0.5);
    var xAlpha = (midpoint.positions[0] - 0.02) / 0.01;
    var yAlpha = (midpoint.positions[1] - 0.01) / 0.01;
    var zAlpha = (midpoint.positions[2] - 0.015) / 0.01;
    near(xAlpha, yAlpha, "Cartesian move preserves the X/Y line", 1e-5);
    near(xAlpha, zAlpha, "Cartesian move preserves the X/Z line", 1e-5);
    runMotion(machine, simulation);
    var linearMove = robot.snapshot();
    near(linearMove.positions.get(0), 0.03, "Cartesian move reaches X target", 1e-5);
    near(linearMove.positions.get(1), 0.02, "Cartesian move reaches Y target", 1e-5);
    near(linearMove.positions.get(2), 0.025, "Cartesian move reaches Z target", 1e-5);

    var cornerPath = GeometricPath.lines([new PathPoint(0.03, 0.02, 0.025),
      new PathPoint(0.04, 0.02, 0.025), new PathPoint(0.04, 0.03, 0.025)]);
    var cornerMove = machine.movePath(cornerPath, PathPlanningOptions.exactStopMode(),
      new MotionOptions(0.05, 0.2));
    check(cornerMove.segments().length > 1, "MotionSystem exposes buffered line-path planning");
    var axisSolver = new AxisKinematics(blueprint);
    var limits = new ValidationLimits(3, Int64.ofInt(blueprint.runtime.revision),
      Int64.ofInt(blueprint.runtime.calibrationRevision));
    for (joint in 0...3) limits.position(joint,
      blueprint.model.joints[joint].limits.lower,
      blueprint.model.joints[joint].limits.upper);
    var compiler = new ProgramCompiler(axisSolver, limits, "work",
      [for (_ in 0...3) 0.1], [for (_ in 0...3) 0.4],
      [for (_ in 0...3) 10.0], null, 0.005);
    var pose = (point:PathPoint) -> new PoseWaypoint(
      new Pose3(point.x, point.y, point.z), 0.005, 0.02);
    var programPath = new PosePath("work", [
      new PoseLine(pose(new PathPoint(0.03, 0.02, 0.025)),
        pose(new PathPoint(0.04, 0.02, 0.025)), OrientationPolicy.Fixed,
        0.1, 0.05),
      new PoseLine(pose(new PathPoint(0.04, 0.02, 0.025)),
        pose(new PathPoint(0.04, 0.03, 0.025)), OrientationPolicy.Fixed,
        0.1, 0.05)]);
    var compiled = compiler.compile(new MotionProgram([
      MotionOp.FollowPath(programPath, "work", 0.05, [])]),
      robot.snapshot().positions.toArray(), Int64.ofInt(700));
    var programPlan = compiled.blocks[0].plans[0];
    var programEnd = programPlan.evaluate(programPlan.durationSeconds).positions;
    var movePathEnd = cornerMove.evaluate(cornerMove.durationSeconds()).positions;
    for (joint in 0...3)
      near(programEnd[joint], movePathEnd[joint],
        'gantry ProgramCompiler matches movePath joint $joint', 1e-5);
    compiled.dispose();
    runMotion(machine, simulation);
    var cornerEnd = robot.snapshot();
    near(cornerEnd.positions.get(0), 0.04, "line path reaches its X endpoint", 1e-5);
    near(cornerEnd.positions.get(1), 0.03, "line path reaches its Y endpoint", 1e-5);
    simulation.dispose();
  }

  static function testPhysicalAssemblyCncBinding():Void {
    var assembly = new AssemblyModel();
    var axes = [new LinearAxis(23, 10, 80), new LinearAxis(23, 10, 80),
      new LinearAxis(23, 10, 80)];
    var ids = ["x", "y", "z"];
    var bindings:Array<motionkit.robot.MachineKitRobotCompiler.AssemblyAxisBinding> = [];
    for (index in 0...3) {
      var id = ids[index], axis = axes[index];
      axis.motor.addTo(assembly, '$id.motor');
      axis.coupling.addTo(assembly, '$id.coupling');
      axis.carriage.addTo(assembly, '$id.carriage');
      assembly.mate('$id.shaft', "continuous", '$id.motor', "shaftTip",
        '$id.coupling', "axis");
      assembly.mateOnAxis('$id.travel', "prismatic", '$id.motor', "shaftTip",
        '$id.carriage', "bore", {x: 0, y: 1, z: 0}, axis.travelMin,
        {lower: axis.travelMin, upper: axis.travelMax, velocity: 100, effort: null});
      var ratio = 2 * Math.PI / axis.nut.travelPerRevolution();
      assembly.couple('$id.lead', '$id.travel', '$id.shaft', ratio,
        -axis.travelMin * ratio);
      if (index > 0) {
        assembly.connector('${ids[index - 1]}.carriage', "stage", AssemblyFrames.identity());
        assembly.connector('$id.motor', "stage", AssemblyFrames.identity());
        assembly.mate('$id.mount', "fixed", '${ids[index - 1]}.carriage', "stage",
          '$id.motor', "stage");
      }
      bindings.push({id: id, axis: axis, motorOccurrenceId: '$id.motor',
        shaftJointId: '$id.shaft', travelJointId: '$id.travel'});
    }
    var definition = assembly.definition("physical-gantry");
    var vertices = Bytes.alloc(4 * 24);
    var support = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
      0.0, 10.0, 0.0, 0.0, 0.0, 10.0];
    for (index in 0...support.length) vertices.setDouble(index * 8, support[index]);
    var parts = AssemblyPhysicalPartView.fromSceneArtifact({metresPerUnit: 0.001, parts: [
      for (component in definition.definitions) {
        id: component.id, name: component.id, red: 0.5, green: 0.5, blue: 0.5,
        materialId: "steel", materialDensity: 7850.0, volume: 1000.0,
        centerOfMass: [0.0, 0.0, 0.0],
        inertia: [1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0],
        vertexCount: 4, indexCount: 0, vertices: vertices,
        normals: Bytes.alloc(0), indices: Bytes.alloc(0), faceRanges: []
      }
    ]});
    var physical = AssemblySimulationBridge.toRobotModel(definition, parts);
    var mismatched = bindings.copy();
    mismatched[0] = {id: "x", axis: axes[0], motorOccurrenceId: "wrong.motor",
      shaftJointId: "x.shaft", travelJointId: "x.travel"};
    var rejected = false;
    try MachineKitRobotCompiler.compileAssemblyAxes(physical.model, mismatched,
      0.1, 0.4) catch (_:Dynamic) rejected = true;
    check(rejected && physical.model.actuators.length == 0,
      "assembly drive attachment rejects a mismatched motor without changing the model");
    var blueprint = MachineKitRobotCompiler.compileAssemblyAxes(physical.model,
      bindings, 0.1, 0.4);
    check(blueprint.model.links.length == 10 && blueprint.model.actuators.length == 3,
      "physical gantry keeps part links and attaches three motor actuators");
    check(blueprint.model.couplings.length == 3,
      "physical gantry keeps its lead-screw joint couplings");
    var hasTenMillimetreVertex = false;
    for (value in physical.linkCollisionHulls[1])
      if (Math.abs(value - 0.01) < 1e-12) hasTenMillimetreVertex = true;
    check(physical.linkCollisionHulls.length == blueprint.model.links.length &&
      physical.linkCollisionHulls[0] == null && hasTenMillimetreVertex,
      "physical assembly passes upstream hulls in link order and SI units");
    var cnc = new CncMachine("work", "x", "y", "z", 0.08);
    var result = new CncMotionBinding(cnc, blueprint).compile(
      "G21 G90 G17\nS12000 M3\nG0 X10 Y10\nF600 G1 X20\nG3 X10 Y20 I-10 J0\nM5\nM2\n",
      [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(990));
    check(result.blocks.length > 0, "physical gantry CNC compiles through ProgramCompiler");
    var plans = result.blocks[result.blocks.length - 1].plans;
    var end = plans[plans.length - 1].evaluate(plans[plans.length - 1].durationSeconds).positions;
    var kinematics = new AxisKinematics(blueprint);
    near(kinematics.forward(end).x, 0.01, "physical gantry arc ends at X", 1e-5);
    near(kinematics.forward(end).y, 0.02, "physical gantry arc ends at Y", 1e-5);
    var endState = plans[plans.length - 1].evaluate(plans[plans.length - 1].durationSeconds);
    var endSpeed = 0.0;
    for (speed in endState.velocities) endSpeed = Math.max(endSpeed, Math.abs(speed));
    check(endSpeed <= 1e-6, 'physical gantry ends at rest (speed=$endSpeed)');
    var lastSegments = plans[plans.length - 1].segments();
    var terminal = lastSegments[lastSegments.length - 1];
    var terminalSeconds = Int64.toFloat(terminal.durationNs) * 1e-9;
    var terminalSpeed = 0.0;
    for (coefficients in terminal.coefficients) {
      var speed = 0.0;
      for (degree in 1...coefficients.length)
        speed += degree * coefficients[degree] * Math.pow(terminalSeconds, degree - 1);
      terminalSpeed = Math.max(terminalSpeed, Math.abs(speed));
    }
    check(terminalSpeed <= 1e-6,
      'physical gantry terminal polynomial ends at rest (speed=$terminalSpeed)');
    var maximumCouplingResidual = 0.0;
    for (block in result.blocks) for (plan in block.plans) for (segment in plan.segments())
      for (coupling in blueprint.model.couplings) {
        var leader = -1, follower = -1;
        for (joint in 0...blueprint.model.joints.length) {
          if (blueprint.model.joints[joint].id == coupling.leader) leader = joint;
          if (blueprint.model.joints[joint].id == coupling.follower) follower = joint;
        }
        for (degree in 0...segment.coefficients[leader].length)
          maximumCouplingResidual = Math.max(maximumCouplingResidual,
            Math.abs(segment.coefficients[follower][degree] -
              coupling.ratio * segment.coefficients[leader][degree] -
              (degree == 0 ? coupling.offset : 0.0)));
      }
    check(maximumCouplingResidual <= 1e-6,
      'physical gantry path preserves coupling polynomial (residual=$maximumCouplingResidual)');
    for (block in result.blocks) for (plan in block.plans) {
      var segments = plan.segments();
      for (first in [0, 75, 150, 225]) if (first < segments.length) {
        var state = plan.evaluate(Int64.toFloat(segments[first].timeFromStartNs) * 1e-9);
        for (coupling in blueprint.model.couplings) {
          var leader = -1, follower = -1;
          for (joint in 0...blueprint.model.joints.length) {
            if (blueprint.model.joints[joint].id == coupling.leader) leader = joint;
            if (blueprint.model.joints[joint].id == coupling.follower) follower = joint;
          }
          check(Math.abs(state.accelerations[follower] -
            coupling.ratio * state.accelerations[leader]) < 1e-6,
            'physical gantry chunk acceleration follows coupling at $first');
        }
      }
    }
    result.dispose();
    for (channel in ["spindle.speed", "spindle.direction"])
      blueprint.runtime.channels.push(new ProcessChannelDeclaration(channel,
        ProcessEventValue.Analog(0.0)));
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobotAtPose(blueprint.runtime,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], null, null,
      physical.linkCollisionHulls);
    var robot = new SimulatedRobot("physical-cnc", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var binding = new CncMotionBinding(cnc, blueprint);
    var motion = new ManipulatorMotion(robot, binding.compiler,
      function(_) return null, function() return runtime.pollEvents());
    motion.run(new CncCompiler(cnc).compile(
      "G21 G90 G17\nS12000 M3\nG0 X10 Y10\nF600 G1 X20\nG3 X10 Y20 I-10 J0\nM5\nM2\n"));
    for (tick in 0...3000) {
      motion.update(0.01);
      simulation.step(Int64.ofInt(tick));
      if (!motion.running) break;
    }
    check(motion.completed, 'physical assembly CNC completes: ${motion.failure}');
    var finalPose = kinematics.forward(robot.snapshot().positions.toArray());
    near(finalPose.x, 0.01, "physical assembly CNC executes X", 2e-4);
    near(finalPose.y, 0.02, "physical assembly CNC executes Y", 2e-4);
    simulation.dispose();
  }

  static function testCncProgramBinding():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 200), 0.1, 0.4);
    var cnc = new CncMachine("work", "x", "y", "z", 0.08);
    var binding = new CncMotionBinding(cnc, blueprint);
    check(cnc.travelLower != null && cnc.travelUpper != null,
      "CNC binding derives a machine travel envelope");
    var travelError = "";
    try binding.compile("G21 G0 X500\nM2\n", [0.0, 0.0, 0.0],
      Int64.ofInt(899))
    catch (error:Dynamic) travelError = Std.string(error);
    check(travelError.indexOf("G-code line 1") >= 0 &&
      travelError.indexOf("X travel") >= 0,
      'bound CNC travel error names the G-code line and axis: $travelError');
    var result = binding.compile("G21 G90 G17\nS12000 M3\nG0 X10 Y10\n" +
      "F600 G1 X20\nG3 X10 Y20 I-10 J0\nM5\nM2\n",
      [0.0, 0.0, 0.0], Int64.ofInt(900));
    check(result.blocks.length > 0, "CNC ProgramCompiler emits execution blocks");
    var last = result.blocks[result.blocks.length - 1].plans;
    check(last.length > 0, "CNC arc is lowered into an execution plan");
    var plan = last[last.length - 1];
    var end = plan.evaluate(plan.durationSeconds).positions;
    near(end[0], 0.01, "CNC arc ends at X", 1e-5);
    near(end[1], 0.02, "CNC arc ends at Y", 1e-5);
    result.dispose();
    cnc.setTool(new CncTool(2, 0.0, 0.002));
    var compensated = binding.compile("G21 G90 F600 G41 D2 G1 X10\n" +
      "G1 X20\nG1 X20 Y10\nG40 G1 X20 Y20\nM2\n",
      [0.0, 0.0, 0.0], Int64.ofInt(925));
    check(compensated.blocks.length > 0,
      "compensated contour lowers through MotionKit");
    compensated.dispose();
    for (arc in ["G17 G2 X5 Y5 Z5 I5 J0",
        "G18 G3 X5 Y5 Z5 I5 K0", "G19 G2 X5 Y5 Z5 J5 K0"]) {
      var helix = binding.compile('G21 G90 F600 $arc\nM2\n',
        [0.0, 0.0, 0.0], Int64.ofInt(950));
      var block = helix.blocks[helix.blocks.length - 1];
      var finalPlan = block.plans[block.plans.length - 1];
      var finalPose = binding.solver.forward(
        finalPlan.evaluate(finalPlan.durationSeconds).positions);
      near(finalPose.x, 0.005, '$arc ends at X', 1e-5);
      near(finalPose.y, 0.005, '$arc ends at Y', 1e-5);
      near(finalPose.z, 0.005, '$arc ends at Z', 1e-5);
      helix.dispose();
    }
  }

  static function testCircularSegments():Void {
    var cases = [CircularPlane.XY, CircularPlane.XZ, CircularPlane.YZ];
    for (plane in cases) {
      var center = switch plane {
        case XY: new PathPoint(0.02, 0.0, 0.0);
        case XZ: new PathPoint(0.02, 0.0, 0.0);
        case YZ: new PathPoint(0.0, 0.02, 0.0);
      };
      var circular = new CircularSegment(center, 0.02, Math.PI,
        -Math.PI * 0.5, plane, 0.01);
      near(circular.length(), Math.sqrt(Math.pow(Math.PI * 0.01, 2) + 0.0001),
        'helical $plane length');
      var tangent = circular.tangentAt(circular.length() * 0.5);
      near(Math.sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1] +
        tangent[2] * tangent[2]), 1.0, 'helical $plane tangent is unit');
      var midpoint = circular.pointAt(circular.length() * 0.5);
      near(circular.distanceTo(midpoint), 0.0, 'helical $plane distance', 1e-9);
      var authored = new GeometricPath([circular]);
      var rig = gantryRig(true);
      var timed = rig.machine.movePath(authored,
        PathPlanningOptions.exactStopMode(), new MotionOptions(0.04, 0.4));
      check(timed.durationSeconds() > 0.0,
        'helical $plane is timed as joint motion');
      var timedEnd = timed.evaluate(timed.durationSeconds()).positions;
      near(rig.machine.axis("x").logicalPosition(timedEnd), circular.end.x,
        'helical $plane reaches X', 1e-5);
      near(rig.machine.axis("y").logicalPosition(timedEnd), circular.end.y,
        'helical $plane reaches Y', 1e-5);
      near(rig.machine.axis("z").logicalPosition(timedEnd), circular.end.z,
        'helical $plane reaches Z', 1e-5);
      rig.simulation.dispose();
      var blended = CornerBlender.blend(new GeometricPath([circular,
        new LineSegment(circular.end, new PathPoint(circular.end.x + 0.01,
          circular.end.y, circular.end.z))]), 0.001, Math.PI * 0.9);
      check(blended.diagnostics.length == 1 &&
        blended.path.primitives.length == 2,
        'helical $plane corner is an exact stop');
      var blendedRig = gantryRig(true);
      var blendedRun = blendedRig.machine.movePath(new GeometricPath([circular,
        new LineSegment(circular.end, new PathPoint(circular.end.x + 0.01,
          circular.end.y, circular.end.z))]), PathPlanningOptions.blend(0.001),
        new MotionOptions(0.04, 0.4));
      check(blendedRun.durationSeconds() > 0.0 &&
        blendedRig.machine.lastPathPlanningDiagnostics.length == 1,
        'helical $plane stays executable with an exact-stop blend fallback');
      blendedRig.simulation.dispose();
      var blueprint = MachineKitRobotCompiler.compileXYZGantry(
        new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
        new LinearAxis(23, 10, 200), 0.1, 0.4);
      var binding = new CncMotionBinding(
        new CncMachine("work", "x", "y", "z", 0.08), blueprint);
      var primitive = new cnckit.CncPosePrimitive(circular, 0.05, 0.0005, 0.02);
      var path = new PosePath("work", [primitive]).withAuthoredGeometry(authored, 0.001);
      var compiled = binding.compiler.compile(new MotionProgram([
        MotionOp.FollowPath(path, "work", 0.05, [])]),
        [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(901));
      check(compiled.blocks.length == 1, 'helical $plane compiles through TOPP-RA');
      compiled.dispose();
    }
  }

  static function testVirtualCncProgram():Void {
    var deterministic = cncTrial(false);
    var repeated = cncTrial(false);
    check(deterministic.length == repeated.length,
      "CNC simulated run has deterministic sample count");
    for (index in 0...deterministic.length)
      near(deterministic[index], repeated[index],
        'CNC deterministic sample $index', 1e-9);
    var device = cncTrial(true);
    check(device.length > 0, "CNC program runs through virtual steppers");
    var disconnected = cncTrial(true, true);
    check(disconnected.length > 0, "CNC link-loss trial recorded device positions");
  }

  static function cncTrial(virtualDevice:Bool, ?linkLoss:Bool = false):Array<Float> {
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
    var binding = new CncMotionBinding(cnc, blueprint);
    var program = new CncCompiler(cnc).compile(
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

  static function testDualMotorAxisRunsThroughSimulation():Void {
    var model = new RobotModel("dual-motor-x");
    var base = model.addLink(new Link("gantry.base"));
    var left = model.addLink(new Link("gantry.left"));
    var right = model.addLink(new Link("gantry.right"));
    var leftJoint = model.addJoint(new Joint("x.left", JointType.Prismatic, base, left, "x.left"));
    var rightJoint = model.addJoint(new Joint("x.right", JointType.Prismatic, left, right, "x.right"));
    for (joint in [leftJoint, rightJoint]) {
      joint.limits.lower = 0.0;
      joint.limits.upper = 0.08;
      joint.limits.velocity = 0.1;
    }
    var blueprint = MotionSystemBlueprint.fromRobotModel(model, [
      new MotionAxisBlueprint("x", ["x.left", "x.right"], 0.0, 0.08, 0.08, 0.4)
    ]);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("dual-motor-x", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var logicalAxis = machine.axis("x");
    check(logicalAxis != null && logicalAxis.jointIndices.length == 2,
      "one logical axis exposes both dual-motor joints");

    machine.home();
    runMotion(machine, simulation);
    machine.moveAxes([new AxisTarget("x", 0.035)], new MotionOptions(0.08, 0.4));
    runMotion(machine, simulation);
    var snapshot = robot.snapshot();
    near(snapshot.positions.get(0), 0.035, "dual-motor axis reaches its logical target on motor one", 1e-5);
    near(snapshot.positions.get(1), 0.035, "dual-motor axis reaches its logical target on motor two", 1e-5);
    simulation.dispose();
  }

  static function testMachineKitLeadScrewThroughVirtualDevice():Void {
    var screw = new LeadScrewThread(MetricTrapezoidal, 10, 2, 4);
    var axis = new LinearAxis(23, 10, 80, null, 30, screw);
    near(axis.nut.travelPerRevolution(), -8.0, "four-start right-hand screw moves -8 mm per positive revolution");
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.01, 0.04);
    var ratio = 2.0 * Math.PI /
      (axis.nut.travelPerRevolution() * MachineKitRobotCompiler.MILLIMETRES_TO_METRES);
    var options = new VirtualDeviceOptions();
    options.actuators = [new VirtualActuatorOptions(blueprint.model.actuators[0].id, 0, ratio, 0.0,
      3200.0 / (2.0 * Math.PI), 0.01 * Math.abs(ratio), 2)];
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime, null, options);
    for (tick in 1...21) simulation.step(Int64.ofInt(tick));
    var robot = new SimulatedRobot("lead-screw-virtual", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var before = simulation.linkPose(0, 1);
    machine.moveAxes([new AxisTarget("x", 0.02)], new MotionOptions(0.01, 0.04));
    runMotion(machine, simulation);
    for (tick in 0...20) simulation.step(Int64.ofInt(tick));
    var finalJoint = robot.snapshot().positions.get(0);
    check(Math.abs(finalJoint - 0.02) <= 1.0 / 400000.0 + 1e-6,
      "MachineKit lead screw follows the virtual RKD6 step position");
    var after = simulation.linkPose(0, 1);
    check(Math.abs(after.position[2] - before.position[2] - 0.02) <=
      1.0 / 400000.0 + 1e-6, "lead screw step position moves the SimKit carriage");
    simulation.dispose();
  }

  static function runMotion(machine:MotionSystem, simulation:Simulation):Void {
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

  static function testPlanCapableReplayRecordsMotionPlan():Void {
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(
      new LinearAxis(23, 10, 80), "x", 0.1, 0.4);
    var source = new RobotRecording();
    source.recordSnapshot(new RobotSnapshot("plan-replay", Int64.ofInt(0),
      Int64.ofInt(0), [0.0], [0.0], [0.0], 0, 0));
    var description = new RobotDescription("plan-replay", blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var capabilities = new RobotCapabilities("plan-replay", 1,
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

  static function testBufferedExecution():Void {
    var xAxis = new LinearAxis(23, 10, 80);
    var yAxis = new LinearAxis(23, 10, 60);
    var zAxis = new LinearAxis(23, 10, 40);
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(xAxis, yAxis, zAxis, 0.1, 0.4);
    var simulation = new Simulation(0.01);
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
      simulation.step(Int64.ofInt(tick++));
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
    for (_ in 0...5) simulation.step(Int64.ofInt(tick++));
    var beforeHold = robot.snapshot().positions.get(0);
    machine.hold();
    check(machine.isHolding(), "buffer reports controlled hold");
    check(machine.queueDepth() == 2, "hold preserves active and waiting trajectories");
    for (_ in 0...5) {
      check(!machine.update(), "held buffer does not submit motion commands");
      simulation.step(Int64.ofInt(tick++));
    }
    var heldPosition = robot.snapshot().positions.get(0);
    check(heldPosition > beforeHold && heldPosition < 0.02,
      "controlled hold decelerates before coming to rest");
    var holdTicks = 0;
    while (runtime.snapshot().sessionState != RobotKitRuntimeConstants.RK_SESSION_HELD) {
      check(!machine.update(), "held buffer remains paused after deceleration");
      simulation.step(Int64.ofInt(tick++));
      holdTicks += 1;
      if (holdTicks > 200) throw "controlled hold did not settle";
    }
    var stoppedPosition = robot.snapshot().positions.get(0);
    check(stoppedPosition >= heldPosition,
      "controlled hold follows the path while slowing down");
    for (_ in 0...2) {
      simulation.step(Int64.ofInt(tick++));
    }
    near(robot.snapshot().positions.get(0), stoppedPosition,
      "controlled hold remains stopped after deceleration", 1e-5);

    machine.resume();
    check(!machine.isHolding(), "buffer resumes from controlled hold");
    for (_ in 0...2) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
    }
    var resumedPosition = robot.snapshot().positions.get(0);
    check(resumedPosition >= heldPosition - 1e-6,
      "resume does not replay a stale trajectory sample backwards");
    while (machine.isMoving()) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
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
    simulation.dispose();
  }

  static function testLongBufferedExecution():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.4);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("long-buffer", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var recording = new RobotRecording();
    var instrumented = new RecordingRobot(robot, recording);
    var machine = MotionSystem.fromBlueprint(instrumented, blueprint);
    var trajectory = Trajectory.fromPositionSamples(
      [for (index in 0...601) index * 0.01],
      [for (index in 0...601) [0.05 * index / 600.0]]);
    machine.queueTrajectory(trajectory);
    check(recording.commands.length >= 2,
      "long trajectory starts with a bounded native plan window");
    var initialPlanCount = recording.commands.length;

    var tick = 0;
    machine.update();
    check(recording.commands.length == initialPlanCount,
      "streamer keeps its initial plan window until the owner advances");
    simulation.step(Int64.ofInt(tick++));
    while (machine.isMoving()) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
      if (tick > 1200) throw "long buffered trajectory did not complete";
    }
    for (_ in 0...4) simulation.step(Int64.ofInt(tick++));
    check(recording.commands.length >= 3,
      "long trajectory refills native chunks before the queue drains");
    for (command in recording.commands) switch command {
      case RobotCommand.TrajectoryChunk(chunk):
        throw "long trajectory submitted a legacy point chunk";
      case RobotCommand.JointTargets(_, _):
        throw "long trajectory unexpectedly fell back to sample-by-sample targets";
      case RobotCommand.ExecutionPlan(plan):
        check(plan.segments.length <= 128,
          "streamed trajectory plans stay within the native segment limit");
      case Hold | Resume | Abort:
        throw "long trajectory unexpectedly submitted a lifecycle command";
    }
    near(instrumented.snapshot().positions.get(0), 0.05,
      "streamed trajectory reaches its final position", 1e-5);
    near(machine.progress(), 1.0, "streamed trajectory reports completed progress");
    simulation.dispose();
  }

  static function testRuntimeSynchronizedHolding():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.2);
    var simulation = new Simulation(0.01);
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
      simulation.step(Int64.ofInt(tick++));
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
        simulation.step(Int64.ofInt(tick++));
        stopTicks += 1;
        if (stopTicks > 200) throw "runtime stop did not settle";
      }
      var heldPosition = robot.snapshot().positions.get(0);
      machine.resume();
      while (machine.isHolding()) {
        machine.update();
        simulation.step(Int64.ofInt(tick++));
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
        simulation.step(Int64.ofInt(tick++));
        var progress = machine.progress();
        check(progress + 1e-9 >= previousProgress,
          "progress remains monotonic after resuming");
        previousProgress = progress;
      }
    }
    while (machine.isMoving()) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
      if (tick > 2000) throw "held trajectory did not complete";
    }
    check(!machine.isHolding() && machine.queueDepth() == 0,
      "repeated hold/resume leaves no buffered motion");
    near(robot.snapshot().positions.get(0), 0.06,
      "repeated hold/resume reaches the planned endpoint", 1e-5);
    simulation.dispose();

    var queuedBlueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.2);
    var queuedSimulation = new Simulation(0.01);
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
      queuedSimulation.step(Int64.ofInt(tick++));
      if (tick > 2000) throw "queued trajectory did not reach its second move";
    }
    queuedMachine.hold();
    while (queuedRuntime.snapshot().sessionState != RobotKitRuntimeConstants.RK_SESSION_HELD) {
      queuedMachine.update();
      queuedSimulation.step(Int64.ofInt(tick++));
      if (tick > 2200) throw "second queued trajectory did not stop";
    }
    var queuedHoldPosition = queuedRobot.snapshot().positions.get(0);
    queuedMachine.resume();
    while (queuedMachine.isMoving()) {
      queuedMachine.update();
      queuedSimulation.step(Int64.ofInt(tick++));
      if (tick > 3000) throw "second queued trajectory did not resume";
    }
    check(queuedRobot.snapshot().positions.get(0) >= queuedHoldPosition - 1e-4,
      "second queued trajectory resumes from its stop path");
    near(queuedRobot.snapshot().positions.get(0), 0.05,
      "hold during a queued trajectory preserves later motion", 1e-5);
    queuedSimulation.dispose();

    var squareBlueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 80), new LinearAxis(23, 10, 80),
      new LinearAxis(23, 10, 80), 0.08, 0.2);
    var squareSimulation = new Simulation(0.01);
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
    tick = 0;
    var reachedThirdLeg = false;
    while (squareMachine.isMoving()) {
      squareMachine.update();
      squareSimulation.step(Int64.ofInt(tick++));
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
      squareSimulation.step(Int64.ofInt(tick++));
      if (tick > 3400) throw "square third-leg stop did not settle";
    }
    var stopped = squareRobot.snapshot().positions;
    squareMachine.resume();
    var previousX = stopped.get(0);
    while (squareMachine.isMoving()) {
      squareMachine.update();
      squareSimulation.step(Int64.ofInt(tick++));
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
    squareSimulation.dispose();
  }

  static function testHoldRefillsNearChunkBoundary():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.4);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("hold-refill-boundary", runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    machine.queueTrajectory(Trajectory.fromPositionSamples(
      [for (index in 0...2001) index * 0.005],
      [for (index in 0...2001) [0.06 * index / 2000.0]]));

    var tick = 0;
    var previousPreHoldPosition = 0.0;
    var preHoldPosition = 0.0;
    for (_ in 0...252) {
      previousPreHoldPosition = preHoldPosition;
      machine.update();
      simulation.step(Int64.ofInt(tick++));
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
      simulation.step(Int64.ofInt(tick++));
      var position = robot.snapshot().positions.get(0);
      var velocity = (position - previousPosition) / 0.01;
      peakAcceleration = Math.max(peakAcceleration,
        Math.abs(velocity - previousVelocity) / 0.01);
      previousPosition = position;
      previousVelocity = velocity;
      stopTicks += 1;
      if (stopTicks > 200) throw "near-boundary controlled hold did not settle";
    }
    check(peakAcceleration <= 0.4 * 1.05,
      "hold near a streamed refill boundary keeps deceleration bounded");
    check(previousPosition > beforeHold,
      "hold near a streamed refill boundary continues along the path to rest");
    simulation.dispose();
  }

  /**
   * Holds at every other tick of one streamed move and checks each stop. The
   * move accelerates and brakes at the joint limit itself, so holds during
   * those phases catch a stop that adds its own deceleration on top, and
   * holds around the streaming refill points catch a stop that runs out of
   * queued path.
   */
  static function testHoldDecelerationStaysWithinLimitsThroughoutMove():Void {
    var limit = 0.4;
    var target = 0.15;
    var moveTicks = 0;
    var holdTick = 2;
    var worstAcceleration = 0.0;
    var worstTick = -1;
    while (moveTicks == 0 || holdTick < moveTicks) {
      var blueprint = MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 200),
        new LinearAxis(23, 10, 60), new LinearAxis(23, 10, 40), 0.1, limit);
      var simulation = new Simulation(0.01);
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
        simulation.step(Int64.ofInt(tick++));
        positions.push(robot.snapshot().positions.get(0));
      }
      machine.hold();
      for (_ in 0...40) {
        machine.update();
        simulation.step(Int64.ofInt(tick++));
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
      simulation.dispose();
      holdTick += 2;
    }
    check(worstAcceleration <= limit * 1.05,
      'every hold stays within the joint acceleration limit (worst ${worstAcceleration} at tick $worstTick)');
  }

  static function testImmediateMotionReplacesNativeQueue():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.08, 0.4);
    var simulation = new Simulation(0.01);
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
    runMotion(machine, simulation);
    near(robot.snapshot().positions.get(0), 0.01,
      "immediate motion replaces stale native trajectory motion", 1e-5);
    simulation.dispose();
  }

  static function testSmoothReplacementRetriesLateSubmission():Void {
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(new LinearAxis(23, 10, 80),
      "x", 0.08, 0.4);
    var simulation = new Simulation(0.01);
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
      simulation.step(Int64.ofInt(tick));
    }
    robot.lateRejections = 1;
    var replacement = machine.moveAxes([new AxisTarget("x", 0.04)], options);
    check(replacement != null, "one late rejection is retried from a fresh snapshot");
    check(robot.replacementAttempts == 2,
      "late replacement submits exactly one retry");
    check(Int64.compare(robot.lastReplacementLeadNs, Int64.ofInt(20000000)) >= 0,
      "smooth replacement anchors at least two owner periods ahead");
    runMotion(machine, simulation);
    near(base.snapshot().positions.get(0), 0.04, "retried replacement reaches target", 1e-5);
    robot.lateRejections = 2;
    machine.moveAxes([new AxisTarget("x", 0.06)], options);
    for (tick in 0...5) {
      machine.update();
      simulation.step(Int64.ofInt(tick + 500));
    }
    var fallback = machine.moveAxes([new AxisTarget("x", 0.02)], options);
    check(fallback == null && machine.isMoving(),
      "two late rejections defer the target behind a stop");
    check(robot.lateRejections == 0,
      "stop-first fallback follows exactly two rejected attempts");
    runMotion(machine, simulation);
    near(base.snapshot().positions.get(0), 0.02,
      "stop-first fallback reaches target", 1e-5);
    simulation.dispose();
  }

  static function testFreeRunningSmoothReplacement():Void {
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(new LinearAxis(23, 10, 80),
      "x", 0.08, 0.4);
    blueprint.replacementOwnerPeriodSeconds = 0.001;
    var simulation = new Simulation(0.001);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("free-running-replacement", runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    simulation.start();
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
    simulation.stop();
    simulation.dispose();
  }

  /**
   * Starts a streamed x move, injects an event at eventTick, optionally
   * resumes once the machine has come to rest, and runs until everything has
   * settled. Returns the observed x position after every tick.
   */
  static function gantryTrial(queueSupport:Bool, eventTick:Int, event:MotionSystem -> Void,
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

  static function peakSecondDifference(positions:Array<Float>):Float {
    var peak = 0.0;
    for (index in 2...positions.length)
      peak = Math.max(peak, Math.abs(positions[index] - 2.0 * positions[index - 1] +
        positions[index - 2]) / (0.01 * 0.01));
    return peak;
  }

  /**
   * Replacing motion while moving and resuming after a hold must both stay
   * within the joint acceleration limit on the plan execution path.
   */
  static function testMotionChangesStayWithinLimits():Void {
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
  static function testContinuousJog():Void {
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
  static function testLateJogReplacementRejectsLateArrival():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 60), new LinearAxis(23, 10, 40), 0.1, 0.4);
    var simulation = new Simulation(0.01);
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
      simulation.step(Int64.ofInt(tick++));
      positions.push(robot.snapshot().positions.get(0));
      if (tick > 2000) throw "late jog splice did not settle";
    }
    for (_ in 0...40) step();
    robot.lagging = true;
    check(machine.jog("x", 0.02, 1.0) != null, "jog change is planned as a continuation");
    for (_ in 0...8) step();
    throws(() -> robot.release(), "late native replacement is rejected explicitly");
    for (_ in 0...250) {
      simulation.step(Int64.ofInt(tick++));
      positions.push(robot.snapshot().positions.get(0));
    }
    check(peakSecondDifference(positions) <= 0.4 * 1.05,
      'original jog stays within the limit (peak ${peakSecondDifference(positions)})');
    near(positions[positions.length - 1], 0.1,
      "rejected replacement leaves the original jog to complete", 1e-5);
    simulation.dispose();
  }

  /**
   * Like gantryTrial, over any rig, recording every joint each tick: begin a
   * motion, inject an event at eventTick, optionally resume once at rest, and
   * run until settled.
   */
  static function rigTrial(rig:TrialRig, eventTick:Int, event:MotionSystem -> Void,
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

  static function jointPeak(positions:Array<Array<Float>>, joint:Int):Float {
    var peak = 0.0;
    for (index in 2...positions.length)
      peak = Math.max(peak, Math.abs(positions[index][joint] - 2.0 * positions[index - 1][joint] +
        positions[index - 2][joint]) / (0.01 * 0.01));
    return peak;
  }

  static function gantryRig(queueSupport:Bool):TrialRig {
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

  static function segmentDistance(x:Float, y:Float, ax:Float, ay:Float, bx:Float, by:Float):Float {
    var dx = bx - ax, dy = by - ay;
    var alpha = Math.max(0.0, Math.min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)));
    var px = ax + alpha * dx - x, py = ay + alpha * dy - y;
    return Math.sqrt(px * px + py * py);
  }

  static function testToleranceBlend():Void {
    var tolerance = 0.0005;
    for (degrees in [10, 45, 90, 135, 150]) {
      var angle = degrees * Math.PI / 180.0;
      var corner = new PathPoint(0.1, 0.0);
      var end = new PathPoint(0.1 + 0.1 * Math.cos(angle),
        0.1 * Math.sin(angle));
      var authored = GeometricPath.lines([new PathPoint(), corner, end]);
      var result = CornerBlender.blend(authored, tolerance, Math.PI * 0.9);
      check(result.path.primitives.length == 3,
        '$degrees degree corner receives a circular blend');
      var fillet = result.path.primitives[1];
      var middle = fillet.pointAt(fillet.length() * 0.5);
      var setback = middle.distanceTo(corner);
      check(setback <= tolerance * 1.01,
        '$degrees degree corner setback stays within tolerance');
      if (degrees <= 45)
        check(setback >= tolerance * 0.9,
          '$degrees degree corner uses its available tolerance');
      var worst = 0.0;
      for (sample in 0...1001) {
        var point = fillet.pointAt(fillet.length() * sample / 1000.0);
        worst = Math.max(worst, Math.min(
          segmentDistance(point.x, point.y, 0.0, 0.0, corner.x, corner.y),
          segmentDistance(point.x, point.y, corner.x, corner.y, end.x, end.y)));
      }
      check(worst <= tolerance * 1.01,
        '$degrees degree fillet stays within the authored corner polyline');
    }
    var mixedCorner = new GeometricPath([
      new LineSegment(new PathPoint(0.0, 0.0), new PathPoint(0.05, 0.0)),
      new ArcSegment(new PathPoint(0.07, 0.0), 0.02, Math.PI, -Math.PI * 0.5)
    ]);
    var mixedGeometry = CornerBlender.blend(mixedCorner, 0.0004, Math.PI * 5.0 / 6.0);
    check(mixedGeometry.path.primitives.length == 3 && mixedGeometry.diagnostics.length == 0,
      "line-to-arc corner receives a tolerance blend");
    for (index in 0...2) {
      var a = mixedGeometry.path.primitives[index];
      var b = mixedGeometry.path.primitives[index + 1];
      check(a.pointAt(a.length()).distanceTo(b.pointAt(0.0)) < 1e-8,
        "mixed blend joins at the same point");
      var ta = a.tangentAt(a.length()), tb = b.tangentAt(0.0);
      check(ta[0] * tb[0] + ta[1] * tb[1] > 0.999999,
        "mixed blend has a continuous tangent");
    }
    var mixedBlend = mixedGeometry.path.primitives[1];
    for (index in 0...501) {
      var point = mixedBlend.pointAt(mixedBlend.length() * index / 500.0);
      var lineDistance = segmentDistance(point.x, point.y, 0.0, 0.0, 0.05, 0.0);
      var angle = Math.max(Math.PI * 0.5,
        Math.min(Math.PI, Math.atan2(point.y, point.x - 0.07)));
      var arcDistance = point.distanceTo(new PathPoint(0.07 + 0.02 * Math.cos(angle),
        0.02 * Math.sin(angle)));
      check(Math.min(lineDistance, arcDistance) <= 0.0004,
        "mixed blend stays within authored geometry tolerance");
    }
    var arcToLine = new GeometricPath([
      mixedCorner.primitives[1],
      new LineSegment(new PathPoint(0.07, 0.02), new PathPoint(0.07, 0.07))
    ]);
    var arcToArc = new GeometricPath([
      mixedCorner.primitives[1],
      new ArcSegment(new PathPoint(0.09, 0.02), 0.02, Math.PI, -Math.PI * 0.5)
    ]);
    check(CornerBlender.blend(arcToLine, 0.0004, Math.PI * 5.0 / 6.0)
      .path.primitives.length == 3, "arc-to-line corner receives a blend");
    check(CornerBlender.blend(arcToArc, 0.0004, Math.PI * 5.0 / 6.0)
      .path.primitives.length == 3, "arc-to-arc corner receives a blend");
    var mixedRig = gantryRig(true);
    var mixedTimed = mixedRig.machine.movePath(mixedCorner,
      PathPlanningOptions.blend(0.0005), new MotionOptions(0.08, 0.4));
    var mixedReport = mixedRig.machine.lastPathValidationReport;
    if (mixedReport == null) throw "Mixed blend did not record validation";
    check(mixedTimed.durationSeconds() > 0.0 &&
      mixedReport.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE].status ==
        MotionKitNativeConstants.MK_CHECK_PASSED,
      "mixed blend times and validates through the runtime plan");
    runMotion(mixedRig.machine, mixedRig.simulation);
    near(mixedRig.robot.snapshot().positions.get(0), 0.07,
      "mixed blend executes to its X endpoint", 1e-5);
    near(mixedRig.robot.snapshot().positions.get(1), 0.02,
      "mixed blend executes to its Y endpoint", 1e-5);
    mixedRig.simulation.dispose();
    var path = GeometricPath.lines([new PathPoint(0.0, 0.0),
      new PathPoint(0.05, 0.0), new PathPoint(0.05, 0.05)]);
    var exactRig = gantryRig(true);
    var exact = exactRig.machine.movePath(path, PathPlanningOptions.exactStopMode(),
      new MotionOptions(0.08, 0.4));
    var blendedRig = gantryRig(true);
    var blended = blendedRig.machine.movePath(path, PathPlanningOptions.blend(0.0005),
      new MotionOptions(0.08, 0.4));
    check(blended.durationSeconds() < exact.durationSeconds(),
      '0.5 mm fillet ${blended.durationSeconds()} is faster than exact stop ${exact.durationSeconds()}');
    var closestCorner = 1.0;
    var cornerSpeed = 0.0;
    for (index in 0...501) {
      var state = blended.evaluate(blended.durationSeconds() * index / 500.0);
      var x = state.positions[0], y = state.positions[1];
      var deviation = Math.min(segmentDistance(x, y, 0.0, 0.0, 0.05, 0.0),
        segmentDistance(x, y, 0.05, 0.0, 0.05, 0.05));
      check(deviation <= 0.0005 + 1e-5,
        "0.5 mm fillet stays within its authored path tolerance");
      var cornerDistance = Math.sqrt((x - 0.05) * (x - 0.05) + y * y);
      if (cornerDistance < closestCorner) {
        closestCorner = cornerDistance;
        cornerSpeed = Math.sqrt(state.velocities[0] * state.velocities[0] +
          state.velocities[1] * state.velocities[1]);
      }
    }
    check(cornerSpeed > 1e-3, "0.5 mm fillet carries speed through the corner");
    var blendReport = blendedRig.machine.lastPathValidationReport;
    if (blendReport == null) throw "Blend path did not record validation";
    var taskCheck = blendReport.checks[MotionKitNativeConstants.MK_CHECK_TASK_SPACE];
    check(taskCheck.status == MotionKitNativeConstants.MK_CHECK_PASSED &&
      Math.abs(taskCheck.limit - 0.0005) < 1e-12,
      "blend validation checks the authored 0.5 mm tolerance");
    runMotion(blendedRig.machine, blendedRig.simulation);
    near(blendedRig.robot.snapshot().positions.get(0), 0.05,
      "blended path executes to its X endpoint", 1e-5);
    near(blendedRig.robot.snapshot().positions.get(1), 0.05,
      "blended path executes to its Y endpoint", 1e-5);
    exactRig.simulation.dispose();
    blendedRig.simulation.dispose();

    var reverse = GeometricPath.lines([new PathPoint(0.0, 0.0),
      new PathPoint(0.05, 0.0),
      new PathPoint(0.05 + 0.05 * Math.cos(Math.PI * 170.0 / 180.0),
        0.05 * Math.sin(Math.PI * 170.0 / 180.0))]);
    var reverseRig = gantryRig(true);
    var fallback = reverseRig.machine.movePath(reverse, PathPlanningOptions.blend(0.0005),
      new MotionOptions(0.08, 0.4));
    check(reverseRig.machine.lastPathPlanningDiagnostics.length > 0 &&
      reverseRig.machine.lastPathPlanningDiagnostics[0].indexOf("turn angle") >= 0,
      "near reversal reports an exact-stop fallback");
    var stoppedAtCorner = false;
    for (segment in fallback.segments()) {
      var state = fallback.evaluate(Int64.toFloat(segment.timeFromStartNs) * 1e-9);
      if (Math.abs(state.positions[0] - 0.05) < 1e-8 &&
          Math.abs(state.positions[1]) < 1e-8 &&
          Math.abs(state.velocities[0]) < 1e-7 && Math.abs(state.velocities[1]) < 1e-7)
        stoppedAtCorner = true;
    }
    check(stoppedAtCorner, "near reversal stops at the authored corner");
    reverseRig.simulation.dispose();
  }

  /**
   * Holding and resuming along a path with an arc, and along a blended
   * corner, stays on the path and within the joint limits.
   */
  static function testPathHoldsStayOnPathWithinLimits():Void {
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

  static function dualMotorRig(queueSupport:Bool):TrialRig {
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

  /**
   * A dual-motor axis whose motors have different gearing keeps each joint
   * within its own limits, and the motors coordinated, through holds,
   * resumes, and replacement moves.
   */
  static function testDualMotorAxisChangesStayWithinJointLimits():Void {
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

  /** Unwraps a move that started at once because the machine was at rest. */
  static function planned(value:Null<Trajectory>):Trajectory {
    if (value == null) throw "Move was deferred behind a stop but was expected to start at once";
    return cast value;
  }

  static function trajectoryStates(value:Trajectory):Array<motionkit.trajectory.TrajectoryState> {
    var result = [];
    var duration = value.durationSeconds();
    for (index in 0...101)
      result.push(value.evaluate(duration * index / 100.0));
    return result;
  }

  static function peakChordAcceleration(value:Trajectory, joint:Int):Float {
    var peak = 0.0;
    for (index in 0...501)
      peak = Math.max(peak, Math.abs(value.evaluate(
        value.durationSeconds() * index / 500.0).accelerations[joint]));
    return peak;
  }

  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
    assertions++;
  }

  static function near(actual:Float, expected:Float, message:String,
      tolerance:Float = 1e-6):Void {
    check(Math.abs(actual - expected) <= tolerance * Math.max(1.0, Math.abs(expected)),
      '$message: $actual != $expected');
  }

  static function throws(action:Void -> Void, message:String):Void {
    var didThrow = false;
    try action() catch (_:Dynamic) didThrow = true;
    check(didThrow, message);
  }
}

/** Deterministic IK branch switch at a synthetic wrist singularity. */
private class WristBranchSolver implements KinematicsSolver {
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

private class PlanarSolver implements KinematicsSolver {
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

/** Robot wrapper that can hold submitted commands back, to simulate transport delay. */
private class LaggingRobot implements Robot {
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

/** A simulated machine for sweep trials. */
private class TrialRig {
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
