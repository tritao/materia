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
import motionkit.robot.CncMotionBinding;
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

import MotionKitTestSupport.WristBranchSolver;
import MotionKitTestSupport.PlanarSolver;
import MotionKitTestSupport.SessionTransitionRig;
import MotionKitTestSupport.LaggingRobot;
import MotionKitTestSupport.TrialRig;

class ProgramTests extends MotionKitTestSupport {
  public function new() { super(); }

  public function testProgramStartTolerances():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 80), new LinearAxis(23, 10, 80),
      new LinearAxis(23, 10, 80), 0.1, 0.4);
    var solver = new AxisKinematics(blueprint);
    var limits = new ValidationLimits(3, Int64.ofInt(blueprint.runtime.revision),
      Int64.ofInt(blueprint.runtime.calibrationRevision));
    for (joint in 0...3) {
      limits.position(joint, 0.0, 0.08);
      limits.velocity(joint, 0.1);
      limits.acceleration(joint, 0.4);
      limits.jerk(joint, 10.0);
    }
    var rejectedLength = false;
    try new ProgramCompiler(solver, limits, "work", [0.1, 0.1, 0.1],
      [0.4, 0.4, 0.4], [10.0, 10.0, 10.0],
      new StartTolerances([0.00001], [0.02, 0.02, 0.02], [0.02, 0.02, 0.02]))
    catch (_:Dynamic) rejectedLength = true;
    check(rejectedLength, "program compiler rejects incomplete start tolerances");
    var rejectedNegative = false;
    try new StartTolerances([0.00001, -1.0, 0.00001],
      [0.02, 0.02, 0.02], [0.02, 0.02, 0.02])
    catch (_:Dynamic) rejectedNegative = true;
    check(rejectedNegative, "program compiler rejects negative start tolerances");
    var rejectedNonfinite = false;
    try new StartTolerances([0.00001, Math.NaN, 0.00001],
      [0.02, 0.02, 0.02], [0.02, 0.02, 0.02])
    catch (_:Dynamic) rejectedNonfinite = true;
    check(rejectedNonfinite, "program compiler rejects non-finite start tolerances");

    var compiler = new ProgramCompiler(solver, limits, "work", [0.1, 0.1, 0.1],
      [0.4, 0.4, 0.4], [10.0, 10.0, 10.0],
      StartTolerances.uniform(3, 0.00001, 0.02, 0.02));
    var compiled = compiler.compile(new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget([0.01, 0.0, 0.0]), new MotionOptions(),
      Blend.ExactStop)]), [0.001, 0.0, 0.0], Int64.ofInt(812));
    var plan = compiled.blocks[0].plans[0];
    near(plan.copyPositionTolerances()[0], 0.00001,
      "program compiler preserves tight start position tolerance", 1e-12);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var segments = [for (segment in plan.segments())
      new TrajectorySegment(segment.timeFromStartNs, segment.durationNs,
        segment.coefficients)];
    var submission = new ExecutionPlanSubmission(plan.planId,
      plan.modelRevision, plan.calibrationRevision,
      RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,
      plan.copyStartPositions(), plan.copyStartVelocities(),
      plan.copyStartAccelerations(), segments, null, null,
      plan.copyPositionTolerances(), plan.copyVelocityTolerances(),
      plan.copyAccelerationTolerances());
    var rejectedAtRuntime = false;
    try runtime.submitPlan(submission, 1) catch (error:Dynamic) {
      if (Std.isOfType(error, RobotRuntimeError)) {
        var nativeError:RobotRuntimeError = cast error;
        rejectedAtRuntime = nativeError.status ==
          RobotKitRuntimeConstants.RK_ERROR_INVALID_STATE;
      }
    }
    check(rejectedAtRuntime,
      "runtime rejects compiled program whose start exceeds tight tolerance");
    simulation.dispose();
    compiled.dispose();
  }

  public function testProgramCompiler():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(new Manipulator(fixture.model, fixture.chain), 1e-8);
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(1));
    for (joint in 0...6) limits.jerk(joint, 20.0);
    var velocity = [for (_ in 0...6) 2.0];
    var acceleration = [for (_ in 0...6) 4.0];
    var jerk = [for (_ in 0...6) 20.0];
    var compiler = new ProgramCompiler(solver, limits, "work", velocity,
      acceleration, jerk, StartTolerances.uniform(6, 0.02, 0.02, 0.02));
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
    check(switch pathPlan.guarantees().taskSpace {
      case Sampled(resolutionNs): Int64.compare(resolutionNs, Int64.ofInt(1000000)) <= 0;
      case _: false;
    }, "plan telemetry reports task-space sampling at most 1 ms apart");
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
      acceleration, jerk, StartTolerances.uniform(6, 0.02, 0.02, 0.02), null, 0.05, 0.5);
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
      acceleration, jerk, StartTolerances.uniform(6, 0.02, 0.02, 0.02), null, 0.05);
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
      acceleration, jerk, StartTolerances.uniform(6, 0.02, 0.02, 0.02), null, 0.05);
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
      "work", velocity, acceleration, jerk,
      StartTolerances.uniform(6, 0.02, 0.02, 0.02), null, 0.005);
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

  public function testManipulatorMotion():Void {
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
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 4.0], [for (_ in 0...6) 20.0],
      StartTolerances.uniform(6, 0.02, 0.02, 0.02));
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
    check(motion.progress().guarantees != null,
      "manipulator progress includes the active plan guarantees");
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

  public function testMotionProgramContracts():Void {
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

}
