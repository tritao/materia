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
import motionkit.robot.ProgramBlocks;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.ProgramPlanner;
import motionkit.robot.StartTolerances;
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
import motionkit.path.PosePrimitive;
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

import MotionKitTestSupport.WristBranchSolver;
import MotionKitTestSupport.PlanarSolver;
import MotionKitTestSupport.SessionTransitionRig;
import MotionKitTestSupport.LaggingRobot;
import MotionKitTestSupport.TrialRig;
import MotionKitTestSupport.FaultingArmRobot;
import processkit.ChannelProcessDevice;
import processkit.FeedChangePolicy;
import processkit.ProcessRecipe;
import processkit.ProcessRun;
import processkit.ProcessRunState;
import robotkit.tool.ChannelToolAdapter;
import robotkit.tool.SimulatedSprayer;

class ProgramTests extends MotionKitTestSupport {
  public function new() { super(); }

  public function testProgramStartTolerances():Void {
    var blueprint = MachineKitRobotCompiler.compileGantry(
      new machinekit.gantry.Gantry(new machinekit.gantry.GantrySpec(80, 80, 80)),
      0.1, 0.4);
    var solver = new AxisKinematics(blueprint);
    var model = blueprint.model;
    var ids = [for (joint in model.joints) joint.id];
    var simulationHarness = new SimulationHarness(0.01);
    var runtime = simulationHarness.simulation.addRobot(blueprint.runtime);
    var fixtureRobot = new robotkit.simulation.SimulatedRobot("start-tolerances", runtime,
      model.name, [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    homeGantryFixture(blueprint, runtime, simulationHarness, fixtureRobot);
    var count = ids.length;
    var scales = [for (_ in ids) 0.0];
    for (axis in blueprint.axes) for (slot in 0...axis.jointIds.length)
      scales[ids.indexOf(axis.jointIds[slot])] = Math.abs(axis.jointScales[slot]);
    var speeds = [for (joint in model.joints) joint.limits.requireVelocity()];
    var accelerations = [for (joint in model.joints) joint.limits.requireAcceleration()];
    var jerks = [for (scale in scales) 10.0 * scale];
    var limits = new ValidationLimits(count, Int64.ofInt(blueprint.runtime.revision),
      runtime.snapshot().calibrationRevision);
    for (joint in 0...count) {
      var bound = model.joints[joint].limits;
      limits.position(joint, bound.lower, bound.upper);
      limits.velocity(joint, speeds[joint]);
      limits.acceleration(joint, accelerations[joint]);
      limits.jerk(joint, jerks[joint]);
    }
    var rejectedLength = false;
    try new ProgramCompiler(solver, limits, "work", speeds, accelerations, jerks,
      new StartTolerances([0.00001], [for (_ in ids) 0.02], [for (_ in ids) 0.02]))
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

    var compiler = new ProgramCompiler(solver, limits, "work", speeds, accelerations, jerks,
      new StartTolerances([for (scale in scales) 0.00001 * scale],
        [for (scale in scales) 0.02 * scale], [for (scale in scales) 0.02 * scale]),
      null, 0.01, 0.5, 0.005, 0.02, null, null,
      [for (scale in scales) 0.5 * scale], ids, model.couplings);
    var start = [for (_ in ids) 0.0], goal = start.copy();
    for (slot in 0...blueprint.axes[0].jointIds.length) {
      var joint = ids.indexOf(blueprint.axes[0].jointIds[slot]);
      start[joint] = blueprint.axes[0].jointScales[slot] * 0.001;
      goal[joint] = blueprint.axes[0].jointScales[slot] * 0.01;
    }
    var compiled = compiler.compile(new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget(goal), new MotionOptions(), Blend.ExactStop)]), start, Int64.ofInt(812));
    var plan = compiled.blocks[0].plans[0];
    near(plan.copyPositionTolerances()[0], 0.00001,
      "program compiler preserves tight start position tolerance", 1e-12);
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
    try fixtureRobot.submit(robotkit.core.RobotCommand.ExecutionPlan(submission)) catch (error:Dynamic) {
      if (Std.isOfType(error, RobotRuntimeError)) {
        var nativeError:RobotRuntimeError = cast error;
        rejectedAtRuntime = nativeError.status ==
          RobotKitRuntimeConstants.RK_ERROR_INVALID_STATE;
        if (!rejectedAtRuntime) throw error;
      }
    }
    check(rejectedAtRuntime,
      "runtime rejects compiled program whose start exceeds tight tolerance");
    simulationHarness.dispose();
    compiled.dispose();

  }

  /** D3: a 7-axis arm keeps its swivel point by point, and a path picks and smooths it as a whole. */
  public function testRedundantArmPaths():Void {
    var fixture = buildSevenAxisArmFixture();
    var arm = fixture.arm;
    check(arm.redundant(), "a 7-axis arm has a swivel by default");
    var start = [0.3, 0.6, 0.5, -1.2, 0.3, 0.8, 0.2];
    var swivel = arm.swivelAngle(start);
    check(Math.isFinite(swivel), 'the swivel is defined at the start ($swivel)');
    var pose = arm.tcpPose(start);
    var turned = arm.solve(pose, start, new robotkit.manipulation.IkOptions().atSwivel(swivel + 0.5));
    check(turned.converged, "the same tool pose solves at another swivel");
    near(arm.swivelAngle(turned.q), swivel + 0.5, "the arm turns to the asked swivel", 1e-3);
    var reached = arm.tcpPose(turned.q).translation;
    near(reached.x, pose.translation.x, "the tool stays put while the elbow turns (x)", 1e-4);
    near(reached.z, pose.translation.z, "the tool stays put while the elbow turns (z)", 1e-4);

    // Point by point, the elbow no longer drifts with each solve.
    var solver = new ManipulatorKinematics(arm, 1e-8);

    // A joint rate for a tool twist moves the swivel exactly at the asked rate.
    var askedSwivelRate = 0.7;
    var twist = new motionkit.kinematics.Twist6(0.05, 0.0, -0.02, 0.0, 0.1, 0.0);
    var rate = solver.solveDifferential(start, twist, [askedSwivelRate], null);
    check(rate != null, "a 7-axis arm has a joint rate for a tool twist");
    var tool = arm.tcpJacobian(start), wanted = twist.toArray();
    for (row in 0...6) {
      var made = 0.0;
      for (j in 0...7) made += tool[row * 7 + j] * rate[j];
      near(made, wanted[row], 'the rate makes the twist (row $row)', 1e-9);
    }
    var gradient = arm.swivelJacobian(start);
    var swivelRate = 0.0;
    for (j in 0...7) swivelRate += gradient[j] * rate[j];
    near(swivelRate, askedSwivelRate, "the swivel moves at the asked rate", 1e-9);
    var step = 1e-6;
    near(gradient[3], (arm.swivelAngle([for (j in 0...7) j == 3 ? start[j] + step : start[j]]) -
      arm.swivelAngle([for (j in 0...7) j == 3 ? start[j] - step : start[j]])) / (2 * step),
      "the swivel Jacobian is the angle's derivative", 1e-5);
    var tolerance = new IkTolerance();
    var startPose = solver.forward(start);
    var q = start, drift = 0.0;
    for (i in 1...21) {
      var target = new Pose3(startPose.x, startPose.y + 0.01 * i, startPose.z, startPose.qx, startPose.qy,
        startPose.qz, startPose.qw);
      var solved = solver.solvePose(target, q, tolerance, null);
      check(solved != null, 'point IK reaches step $i');
      q = solved;
      drift = Math.max(drift, Math.abs(arm.swivelAngle(q) - swivel));
    }
    check(drift < 1e-3, 'point IK keeps the swivel along a 20 cm move ($drift rad)');

    // The same line solved point by point, each solve seeded by the last: the baseline.
    var endPose = new Pose3(startPose.x - 0.1, startPose.y + 0.3, startPose.z - 0.15, startPose.qx, startPose.qy,
      startPose.qz, startPose.qw);
    var pointTravel = 0.0;
    {
      var chain = start;
      for (i in 1...36) {
        var f = i / 35.0;
        var target = new Pose3(startPose.x - 0.1 * f, startPose.y + 0.3 * f, startPose.z - 0.15 * f, startPose.qx,
          startPose.qy, startPose.qz, startPose.qw);
        var solved = solver.solvePose(target, chain, tolerance, null);
        check(solved != null, 'the line solves point by point at $f');
        for (j in 0...7) pointTravel += Math.abs(solved[j] - chain[j]);
        chain = solved;
      }
    }

    // The resolver's rate of its smoothed swivel along the line is the curve's own derivative: it matches the
    // centred difference of the solved swivels (which sit exactly on that curve) to the difference's own accuracy.
    {
      var distances = [for (i in 0...36) 0.5 * i / 35.0], poses:Array<Pose3> = [];
      for (i in 0...36) {
        var f = i / 35.0;
        poses.push(new Pose3(startPose.x - 0.1 * f, startPose.y + 0.3 * f, startPose.z - 0.15 * f, startPose.qx,
          startPose.qy, startPose.qz, startPose.qw));
      }
      var solved = solver.solvePathWithRates(new motionkit.kinematics.PathRequest(distances, poses, start, tolerance,
        [for (_ in 0...7) 0.5], [for (_ in 0...7) 2.0], 48));
      var exact = solved.redundancyRates;
      check(exact != null && exact.length == 36 && exact[0].length == 1, "a redundant path reports its swivel rates");
      if (exact != null) {
        var worstRate = 0.0, scale = 0.0;
        // Away from the ends: the first sample is pinned to the start, which the smoothed curve is not.
        for (i in 4...32) {
          var turn = arm.swivelAngle(solved.configurations[i + 1]) - arm.swivelAngle(solved.configurations[i - 1]);
          while (turn > Math.PI) turn -= 2.0 * Math.PI;
          while (turn < -Math.PI) turn += 2.0 * Math.PI;
          worstRate = Math.max(worstRate, Math.abs(exact[i][0] - turn / (distances[i + 1] - distances[i - 1])));
          scale = Math.max(scale, Math.abs(exact[i][0]));
        }
        check(worstRate <= 0.05 * Math.max(scale, 1e-3) + 1e-4,
          'the swivel rates are the smoothed curve\'s derivative (worst ${worstRate} against a scale of ${scale})');
      }
    }

    // A compiled line: the swivel is chosen for the whole path and changes smoothly.
    var limits = new ValidationLimits(7, Int64.ofInt(1), Int64.ofInt(1));
    for (joint in 0...7) limits.jerk(joint, 20.0);
    var compiler = new ProgramCompiler(solver, limits, "work", [for (_ in 0...7) 2.0], [for (_ in 0...7) 4.0],
      [for (_ in 0...7) 20.0], StartTolerances.uniform(7, 0.02, 0.02, 0.02));
    var line = compiler.compile(new MotionProgram([MotionOp.MoveL(endPose, "work", 0.1, Blend.ExactStop)]), start,
      Int64.ofInt(400));
    var plan = line.blocks[0].plans[0];
    check(plan.report.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].status ==
      TrajectoryCoreConstants.MK_CHECK_PASSED, "the 7-axis line meets the Cartesian tolerance");
    var samples = 200;
    var angles:Array<Float> = [];
    var worstJump = 0.0, planTravel = 0.0;
    var previous:Array<Float> = null;
    for (k in 0...(samples + 1)) {
      var joints = plan.evaluate(plan.durationSeconds * k / samples).positions;
      angles.push(arm.swivelAngle(joints));
      if (previous != null) for (j in 0...7) {
        worstJump = Math.max(worstJump, Math.abs(joints[j] - previous[j]));
        planTravel += Math.abs(joints[j] - previous[j]);
      }
      previous = joints;
    }
    var worstBend = 0.0;
    for (k in 1...samples) worstBend = Math.max(worstBend, Math.abs(angles[k + 1] - 2.0 * angles[k] + angles[k - 1]));
    check(worstJump < 0.05, 'the 7-axis line moves its joints smoothly ($worstJump rad per sample)');
    check(worstBend < 5e-3, 'the swivel changes smoothly along the line ($worstBend)');
    // Choosing the swivel for the whole path moves the joints less than solving point by point.
    check(planTravel < 0.8 * pointTravel, 'the path moves the joints less than point-by-point IK ($planTravel vs $pointTravel rad)');
    line.dispose();
  }

  /** D6: a path on a turning workpiece, planned over the arm, its rail and the positioner as one plan. */
  public function testExternalPathPosture():Void {
    var fixture = buildWorkcellFixture();
    var flange = [for (frame in fixture.model.frames) if (frame.name == "flange") frame][0];
    // The last wrist axis is local +Y; make it the tool's free wire axis.
    flange.rotation = [-Math.sqrt(0.5), 0.0, 0.0, Math.sqrt(0.5)];
    var work = [for (frame in fixture.model.frames) if (frame.name == "work") frame][0];
    var group = new robotkit.manipulation.KinematicGroup(fixture.model, fixture.model.links[0].id,
      flange.id, work.id, null, null, null);
    var solver = new ManipulatorKinematics(group, 1e-8);
    var preferred = [0.75, 1.57, -1.2, 1.6, -1.97, -1.57, 0.0, 0.0];
    var start = preferred.copy(); start[6] = 0.5;
    solver.preferredPosture = preferred;
    var target = solver.forward(preferred);
    var route = solver.solvePath(new motionkit.kinematics.PathRequest([0.0, 0.05],
      [solver.forward(start), target], start, new IkTolerance(), [for (_ in 0...8) 1.0],
      [for (_ in 0...8) 1.0], 4, [OrientationPolicy.FreeAboutTool, OrientationPolicy.FreeAboutTool]));
    var end = route[1];
    check(end != null && Math.abs(end[6]) < 0.01,
      "whole-path external redundancy retains the requested arm posture instead of the seed's wrist winding");
    if (end != null) {
      var reached = solver.forward(end);
      check(Math.sqrt(Math.pow(reached.x - target.x, 2) + Math.pow(reached.y - target.y, 2) +
        Math.pow(reached.z - target.z, 2)) < 1e-4, "posture preference preserves the hard TCP target");
      check(motionkit.robot.ToolFreedom.orientationError(reached, target, OrientationPolicy.FreeAboutTool) < 0.001,
        "posture preference preserves the hard wire direction");
    }
  }

  public function testCoordinatedExternalAxes():Void {
    var fixture = buildWorkcellFixture();
    var cell = fixture.group;
    check(cell.dofCount() == 8, 'the group holds the rail, the arm and the turntable (${cell.dofCount()})');
    check(cell.external[0] && !cell.external[1] && cell.external[7], "the rail and the turntable are external axes");
    check(cell.swivel == null, "six arm joints plus external axes do not create a seven-joint swivel");
    var solver = new ManipulatorKinematics(cell, 1e-8);
    check(Std.isOfType(solver.redundancy(), motionkit.robot.ExternalAxesParameterization),
      "derived external axes use external-axis redundancy parameterization");
    var tolerance = new IkTolerance();
    // Rail in front of the table, arm reaching forward with the tool pointing down.
    var start = [0.75, 1.57, -1.2, 1.6, -1.97, -1.57, 0.0, 0.0];
    var here = solver.forward(start);
    var preferred = cell.solve(cell.tcpPose(start), start,
      new robotkit.manipulation.IkOptions().preferring(start));
    check(preferred.converged && preferred.q.length == 8, "posture preference retains the external coordinates");

    // The relative Jacobian is the derivative of the tool pose in the work frame.
    var jacobian = cell.tcpJacobian(start);
    var eps = 1e-6;
    for (column in [0, 2, 7]) {
      var plus = start.copy(); plus[column] += eps;
      var minus = start.copy(); minus[column] -= eps;
      var a = solver.forward(plus), b = solver.forward(minus);
      near(jacobian[column], (a.x - b.x) / (2 * eps), 'relative Jacobian x column $column', 1e-6);
      near(jacobian[8 + column], (a.y - b.y) / (2 * eps), 'relative Jacobian y column $column', 1e-6);
    }

    // A 0.3 m circle round the workpiece, starting on the side facing the arm, the tool pointing down and
    // turning with the circle's tangent (as a torch or cutter follows a seam). The far side is out of the
    // arm's reach, so the turntable must bring it round, and in the world the tool then barely turns.
    var radius = 0.3, lines:Array<PosePrimitive> = [];
    function onCircle(turn:Float):Pose3 {
      var angle = -0.5 * Math.PI + turn, half = 0.5 * turn;
      // Rz(turn) · (pointing down: half a turn about X).
      return new Pose3(radius * Math.cos(angle), radius * Math.sin(angle), 0.15, Math.cos(half), Math.sin(half), 0, 0);
    }
    var points = [for (k in 0...33) onCircle(2.0 * Math.PI * k / 32)];
    // Enter the circle in the configuration that leaves the turntable nearest home.
    var options = solver.sampleCandidates(points[0], 8, tolerance, null);
    check(options.length > 0, "the circle's first point is reachable");
    var entry = options[0];
    for (option in options) if (Math.abs(option[7]) < Math.abs(entry[7])) entry = option;
    solver.preferredPosture = entry;
    for (k in 0...32) lines.push(new PoseLine(new PoseWaypoint(points[k], 0.0005, 0.005),
      new PoseWaypoint(points[k + 1], 0.0005, 0.005), OrientationPolicy.Interpolated, 0.1, 0.1));
    var path = new PosePath("work", lines);
    var blueprint = RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile());
    var limits = new ValidationLimits(8, Int64.ofInt(blueprint.revision), Int64.ofInt(blueprint.calibrationRevision));
    for (joint in 0...8) limits.jerk(joint, 20.0);
    var compiler = new ProgramCompiler(solver, limits, "work", [for (_ in 0...8) 1.0], [for (_ in 0...8) 2.0],
      [for (_ in 0...8) 20.0], StartTolerances.uniform(8, 0.02, 0.02, 0.02), null, 0.0025);
    var compiled = compiler.compile(new MotionProgram([MotionOp.FollowPath(path, "work", 0.05, [])]), entry,
      Int64.ofInt(500));
    // The polygon's corners are sharp, so the circle is one plan per side, each stopping at its corner.
    var plans = [for (block in compiled.blocks) for (plan in block.plans) plan];
    check(plans.length == 32, 'the circle is planned side by side (${plans.length} plans)');
    for (plan in plans)
      check(plan.report.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].status ==
        TrajectoryCoreConstants.MK_CHECK_PASSED, "the tool follows the circle on the turning workpiece");
    var plan = plans[plans.length - 1];
    var turned = 0.0, railMoved = 0.0, armMoved = 0.0;
    var first = plans[0].evaluate(0.0).positions;
    for (each in plans)
      for (k in 0...11) {
        var q = each.evaluate(each.durationSeconds * k / 10).positions;
        turned = Math.max(turned, Math.abs(q[7] - first[7]));
        railMoved = Math.max(railMoved, Math.abs(q[0] - first[0]));
        for (j in 1...7) armMoved = Math.max(armMoved, Math.abs(q[j] - first[j]));
      }
    check(turned > Math.PI, 'the positioner turns the workpiece round ($turned rad)');
    // The search weighs external-axis motion at a tenth of the arm's: the work turns, the arm barely moves.
    check(armMoved < 0.1 * turned, 'the arm moves far less than the turntable ($armMoved rad against $turned)');

    // One plan drives every joint of the cell on one clock.
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("workcell", runtime, fixture.model.name, [for (link in fixture.model.links) link.name],
      [for (joint in fixture.model.joints) joint.name]);
    robot.submit(RobotCommand.JointTargets([for (j in 0...8) robotkit.core.JointTarget.position(j, entry[j])], null));
    var tick = 0;
    for (_ in 0...300) harness.step(Int64.ofInt(++tick));
    // Executed through the program runner, which streams the long plan to the runtime in chunks.
    var motion = new ManipulatorMotion(robot, compiler, function(_) return null,
      function() return {events: [], overflow: false}, cell.jointIndices());
    motion.run(new MotionProgram([MotionOp.FollowPath(path, "work", 0.05, [])]));
    function offPath(tool:Pose3):Float {
      var best = Math.POSITIVE_INFINITY;
      for (k in 0...32) {
        var a = points[k], b = points[k + 1];
        var dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
        var f = Math.max(0.0, Math.min(1.0, ((tool.x - a.x) * dx + (tool.y - a.y) * dy + (tool.z - a.z) * dz) /
          (dx * dx + dy * dy + dz * dz)));
        var ex = tool.x - a.x - f * dx, ey = tool.y - a.y - f * dy, ez = tool.z - a.z - f * dz;
        best = Math.min(best, Math.sqrt(ex * ex + ey * ey + ez * ez));
      }
      return best;
    }
    var worst = 0.0, guard = 0;
    while (!motion.completed && motion.failure == null && guard++ < 10000) {
      motion.update(0.01);
      harness.step(Int64.ofInt(++tick));
      var q = [for (j in 0...8) robot.snapshot().positions.get(j)];
      worst = Math.max(worst, offPath(solver.forward(q)));
    }
    check(motion.completed, 'the coordinated program completes (${motion.failure})');
    check(worst < 1e-3, 'executed, the tool stays on the path in the work frame ($worst m)');
    near(robot.snapshot().positions.get(7), plan.evaluate(plan.durationSeconds).positions[7],
      "the turntable ends where the plan does", 1e-4);
    harness.dispose();
    compiled.dispose();
  }

  public function testProgramCompiler():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(fixture.arm, 1e-8);
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
      TrajectoryCoreConstants.MK_CHECK_JERK].status ==
      TrajectoryCoreConstants.MK_CHECK_PASSED,
      "Ruckig MoveJ reports jerk checked");
    near(compiled.blocks[0].plans[0].evaluate(
      compiled.blocks[0].plans[0].durationSeconds).positions[0], goal[0],
      "program compiler reaches the MoveJ target", 1e-6);
    compiled.dispose();

    var outputBeforeWait = compiler.compile(new MotionProgram([
      MotionOp.SetOutput("spindle.speed", EventValue.Analog(8000.0)),
      MotionOp.WaitInput("spindle.at_speed", InputPredicate.Equals(
        EventValue.Digital(true)), 3.0)
    ]), start, Int64.ofInt(150));
    check(outputBeforeWait.blocks.length == 1 &&
      outputBeforeWait.blocks[0].plans.length == 1,
      "output before a wait gets a plan in the preceding block");
    var startup = outputBeforeWait.blocks[0].plans[0];
    check(startup.events.length == 1 &&
      Int64.compare(startup.events[0].timeNs, Int64.ofInt(0)) == 0,
      "spindle output fires at the start of the wait's preceding plan");
    near(startup.durationSeconds, 0.01,
      "stationary output plan lasts one controller cycle", 1e-9);
    near(startup.evaluate(startup.durationSeconds).positions[0], start[0],
      "output plan holds the starting position", 1e-9);
    outputBeforeWait.dispose();

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
    check(pathPlan.report.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].status ==
      TrajectoryCoreConstants.MK_CHECK_PASSED,
      "sampled task-space validation is recorded in the plan");
    check(switch pathPlan.guarantees().taskSpace {
      case Sampled(resolutionNs): Int64.compare(resolutionNs, Int64.ofInt(1000000)) <= 0;
      case _: false;
    }, "plan telemetry reports task-space sampling at most 1 ms apart");
    lowered.dispose();

    var line = compiler.compile(new MotionProgram([MotionOp.MoveL(endPose,
      "work", 0.1, Blend.ExactStop)]), start, Int64.ofInt(300));
    check(line.blocks[0].plans[0].report.checks[
      TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].status ==
      TrajectoryCoreConstants.MK_CHECK_PASSED,
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
    // A path that turns back stops at its corner: two plans, planned a step each, so a long path
    // does not hold up the plans before it.
    var cornerPath = new PosePath("work", [
      new PoseLine(new PoseWaypoint(new Pose3(0.0), 0.005, 0.02),
        new PoseWaypoint(new Pose3(0.1), 0.005, 0.02), OrientationPolicy.Fixed, 0.1, 0.1),
      new PoseLine(new PoseWaypoint(new Pose3(0.1), 0.005, 0.02),
        new PoseWaypoint(new Pose3(0.05), 0.005, 0.02), OrientationPolicy.Fixed, 0.1, 0.1)]);
    var cornerBlocks = new ProgramBlocks();
    var stepping = linearCompiler.begin(new MotionProgram([MotionOp.FollowPath(cornerPath, "work", 0.1, [])]),
      [0.0, 0.0, 0.0, 0.0, 0.1, 0.0], Int64.ofInt(503), 0, 1.0, cornerBlocks);
    var delivered:Array<Int> = [];
    while (stepping.step()) delivered.push(cornerBlocks.blocks[0].plans.length);
    check(delivered.join(",") == "0,1" && cornerBlocks.done && cornerBlocks.blocks[0].plans.length == 2,
      'a path is planned one stretch between corners a step, got $delivered');
    stepping.dispose();
    cornerBlocks.dispose();
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
      TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].status ==
      TrajectoryCoreConstants.MK_CHECK_PASSED,
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
    check(blendedPlan.report.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].status ==
      TrajectoryCoreConstants.MK_CHECK_PASSED,
      "blended Cartesian plan passes authored-corner task-space validation");
    check(blendedPlan.report.checks[TrajectoryCoreConstants.MK_CHECK_JERK].status ==
      TrajectoryCoreConstants.MK_CHECK_UNCHECKED,
      "TOPP-RA Cartesian plan reports jerk unchecked");
    near(blendedPlan.report.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].limit,
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
      TrajectoryCoreConstants.MK_CHECK_TASK_SPACE];
    check(shallowCheck.status == TrajectoryCoreConstants.MK_CHECK_PASSED &&
      shallowCheck.value <= 0.001 + 1e-9 && shallowCheck.limit == 0.001,
      "150 degree authored corner is checked against its tolerance");
    shallow.dispose();
  }

  /**
    A ProgramPlanner plans on a worker thread what a synchronous compile
    plans, stays its lookahead ahead of the plans started, and cancels
    cleanly while it waits.
  **/
  public function testProgramPlanner():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(fixture.arm, 1e-8);
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(1));
    for (joint in 0...6) limits.jerk(joint, 20.0);
    var compiler = new ProgramCompiler(solver, limits, "work", [for (_ in 0...6) 2.0],
      [for (_ in 0...6) 4.0], [for (_ in 0...6) 20.0], StartTolerances.uniform(6, 0.02, 0.02, 0.02));
    var start = [0.2, -0.4, 0.6, 0.1, 0.4, -0.2];
    var ops:Array<MotionOp> = [];
    for (index in 0...6) {
      var goal = start.copy();
      goal[0] += index % 2 == 0 ? 0.05 : 0.0;
      ops.push(MotionOp.MoveJ(MoveTarget.JointTarget(goal), new MotionOptions(), Blend.ExactStop));
      if (index == 2) ops.push(MotionOp.Dwell(0.1));
    }
    var program = new MotionProgram(ops);
    var compiled = compiler.compile(program, start, Int64.ofInt(300));
    var planner = new ProgramPlanner(compiler, program, start, Int64.ofInt(300), 0, 1.0, 1e9);
    while (!planner.blocks.done && planner.failure == null) planner.waitForMore();
    var planned = planner.blocks.finishedBlocks();
    var same = planner.failure == null && planned.length == compiled.blocks.length;
    if (same) for (index in 0...planned.length) {
      var expected = compiled.blocks[index].plans, actual = planned[index].plans;
      same = same && actual.length == expected.length &&
        Std.string(planned[index].barrier) == Std.string(compiled.blocks[index].barrier);
      if (same) for (plan in 0...actual.length)
        same = same && Math.abs(actual[plan].durationSeconds - expected[plan].durationSeconds) < 1e-12 &&
          Int64.compare(actual[plan].planId, expected[plan].planId) == 0;
    }
    check(same && Int64.compare(planner.nextPlanId, Int64.ofInt(306)) == 0,
      "the planner's worker plans the blocks a synchronous compile plans");
    planner.dispose();
    compiled.dispose();

    // With almost no lookahead, the worker delivers one plan beyond those started, then waits.
    var paced = new ProgramPlanner(compiler, program, start, Int64.ofInt(400), 0, 1.0, 1e-6);
    paced.waitForMore();
    Sys.sleep(0.2);
    paced.poll();
    check(paced.blocks.blocks[0].plans.length == 1 && !paced.isStopped(),
      "the planner waits once its lookahead is planned");
    paced.started(1);
    while (paced.blocks.blocks[0].plans.length < 2) paced.waitForMore();
    check(paced.blocks.blocks[0].plans.length == 2, "starting a plan lets the planner plan the next");
    var delivered = paced.blocks.blocks[0].plans.copy();
    paced.dispose();
    var waited = 0;
    while (!paced.isStopped() && waited++ < 200) Sys.sleep(0.01);
    check(paced.isStopped() && delivered[0].isClosed() && delivered[1].isClosed(),
      "disposing a waiting planner stops its worker and disposes its plans");

    // Keep both workers waiting on bounded lookahead before checking their registration.
    // An unbounded worker may finish before active() is observed.
    var running = new ProgramPlanner(compiler, program, start, Int64.ofInt(500), 0, 1.0, 1e-6);
    var waiting = new ProgramPlanner(compiler, program, start, Int64.ofInt(600), 0, 1.0, 1e-6);
    running.waitForMore();
    waiting.waitForMore();
    check(ProgramPlanner.active() >= 2, "planners with a worker still going are counted");
    check(ProgramPlanner.shutdown() == 0 && ProgramPlanner.active() == 0 && running.isStopped() && waiting.isStopped(),
      "a shutdown stops every worker and waits for them");
    running.dispose();
    waiting.dispose();
  }

  public function testCompiledManipulatorMotion():Void {
    var fixture=buildContractArmFixture();
    for(joint in fixture.model.joints)joint.limits.maxAcceleration=4.0;
    var harness=new SimulationHarness(0.01);
    var blueprint=RobotRuntimeCompiler.compile(fixture.model,new robotkit.profile.RobotProfile());
    blueprint.channels.push(new ProcessChannelDeclaration("compiled.enabled",ProcessEventValue.Digital(false)));
    var runtime=harness.simulation.addRobot(blueprint);
    var robot=new SimulatedRobot("compiled-arm",runtime,fixture.model.name,
      [for(link in fixture.model.links)link.name],[for(joint in fixture.model.joints)joint.name]);
    var solver=new ManipulatorKinematics(fixture.arm,1e-8);
    var compiler=new ProgramCompiler(solver,new ValidationLimits(6,Int64.ofInt(1),Int64.ofInt(0)),"work",
      [for(_ in 0...6)2.0],[for(_ in 0...6)4.0],[for(_ in 0...6)20.0],
      StartTolerances.uniform(6,0.02,0.02,0.02));
    var ready=false,motion=new ManipulatorMotion(robot,compiler,_->EventValue.Digital(ready),()->runtime.pollEvents());
    var origin=[for(_ in 0...6)0.0],goal=origin.copy();goal[0]=0.02;
    var program=new MotionProgram([
      MotionOp.MoveJ(MoveTarget.JointTarget(goal),new MotionOptions(),Blend.ExactStop),
      MotionOp.SetOutput("compiled.enabled",EventValue.Digital(true)),
      MotionOp.WaitInput("ready",InputPredicate.Equals(EventValue.Digital(true)),3.0),
      MotionOp.Dwell(0.1),
      MotionOp.MoveJ(MoveTarget.JointTarget(origin),new MotionOptions(),Blend.ExactStop),
      MotionOp.SetOutput("compiled.enabled",EventValue.Digital(false))
    ]);
    var compiled=compiler.compile(program,origin,Int64.ofInt(700));
    var workers=ProgramPlanner.active();
    motion.setSpeedOverride(0.5);
    throws(()->motion.runCompiled(compiled),"compiled submission rejects a different execution speed override");
    motion.setSpeedOverride(1.0);motion.runCompiled(compiled);
    compiled.dispose(); // Ownership moved; this must not destroy queued plans.
    throws(()->ProgramPlanner.fromCompiled(compiled,Int64.ofInt(700)),"compiled plan ownership transfers only once");
    check(ProgramPlanner.active()==workers,"precompiled submission starts no planning worker");
    throws(()->motion.setSpeedOverride(0.5),"running compiled plans retain their validated process clock");
    near(motion.speedOverride,1.0,"rejected compiled override does not change motion state",0);
    var tick=0;
    for(i in 0...400){
      if(i==100)ready=true;
      motion.update(0.01);harness.step(Int64.ofInt(++tick));
      if(motion.completed || motion.failure!=null)break;
    }
    check(motion.completed && motion.failure==null,'precompiled motion completes through wait/dwell barriers: ${motion.failure}');
    near(robot.snapshot().positions.get(0),0.0,"precompiled program reaches its selected endpoint",1e-4);
    var fired=motion.firedEvents();
    check(Lambda.exists(fired,event->event.channel=="compiled.enabled" && switch event.value {
      case ProcessEventValue.Digital(true):true;default:false;
    }),"precompiled runtime retains engagement on events");
    check(Lambda.exists(fired,event->event.channel=="compiled.enabled" && switch event.value {
      case ProcessEventValue.Digital(false):true;default:false;
    }),"precompiled runtime retains engagement off events");
    var metrics=motion.planningMetrics();
    check(metrics.seconds==0 && metrics.numericIkSolves==0,"precompiled execution performs no additional compilation");
    var stale=compiler.compile(program,origin,Int64.ofInt(700));
    throws(()->motion.runCompiled(stale),"precompiled execution rejects previously used plan ids");stale.dispose();
    motion.run(new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(goal),new MotionOptions(),Blend.ExactStop)]));
    for(_ in 0...300){motion.update(0.01);harness.step(Int64.ofInt(++tick));if(motion.completed || motion.failure!=null)break;}
    check(motion.completed && motion.failure==null,"normal worker planning continues with unused ids after compiled execution");
    harness.dispose();
  }

  public function testManipulatorMotion():Void {
    var fixture = buildContractArmFixture();
    for (joint in fixture.model.joints) joint.limits.maxAcceleration = 4.0;
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var blueprint = RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile());
    blueprint.channels.push(new ProcessChannelDeclaration("sprayer.enabled",
      ProcessEventValue.Digital(false)));
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("program-arm", runtime, fixture.model.name,
      [for (link in fixture.model.links) link.name],
      [for (joint in fixture.model.joints) joint.name]);
    var solver = new ManipulatorKinematics(fixture.arm, 1e-8);
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(0));
    var compiler = new ProgramCompiler(solver, limits, "work",
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 4.0], [for (_ in 0...6) 20.0],
      StartTolerances.uniform(6, 0.02, 0.02, 0.02));
    var unsupported = new RuntimeRobotAdapter("unsupported-arm", runtime,
      fixture.model.name, [for (link in fixture.model.links) link.name],
      [for (joint in fixture.model.joints) joint.name], false, false,
      "simulated runtime fault", robotkit.core.ExecutionCapabilities.unavailable());
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
      simulationHarness.step(Int64.ofInt(tick));
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
      simulationHarness.step(Int64.ofInt(500 + tick));
      if (!motion.running) break;
    }
    check(!motion.completed && motion.failure != null &&
      motion.failure.indexOf("timed out") >= 0,
      "barrier timeout fails the manipulator program with a diagnostic");
    simulationHarness.step(Int64.ofInt(1000));
    check(Lambda.exists(motion.firedEvents(), function(event) return switch event.value {
      case ProcessEventValue.Digital(enabled): !enabled;
      case _: false;
    }),
      "timeout abort produces a safe output transition");
    simulationHarness.dispose();

    var pathSimulationHarness = new SimulationHarness(0.01);
    var pathSimulation = pathSimulationHarness.simulation;
    var pathBlueprint = RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile());
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
      pathSimulationHarness.step(Int64.ofInt(pathTick++));
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
      pathSimulationHarness.step(Int64.ofInt(pathTick++));
    }
    pathMotion.hold();
    for (_ in 10...50) {
      pathMotion.update(0.01);
      pathSimulationHarness.step(Int64.ofInt(pathTick++));
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
      pathSimulationHarness.step(Int64.ofInt(pathTick++));
      if (!pathMotion.running) break;
    }
    check(pathMotion.completed && pathMotion.firedEvents().length > 0,
      'held FollowPath completes and fires its event after resume: ${pathMotion.failure}');
    pathMotion.run(new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget(origin), new MotionOptions(), Blend.ExactStop)]));
    for (_ in 0...5) {
      pathMotion.update(0.01);
      pathSimulationHarness.step(Int64.ofInt(pathTick++));
    }
    pathMotion.abort();
    pathSimulationHarness.step(Int64.ofInt(pathTick++));
    check(Lambda.exists(pathMotion.firedEvents(), function(event) return switch event.value {
      case ProcessEventValue.Digital(enabled): !enabled;
      case _: false;
    }), "manipulator abort restores the safe process value");
    pathSimulationHarness.dispose();
  }

  public function testProcessRunVirtualArmRecovery():Void {
    var fixture = buildContractArmFixture();
    for (joint in fixture.model.joints) joint.limits.maxAcceleration = 4.0;
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var blueprint = RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile());
    blueprint.channels.push(new ProcessChannelDeclaration("paint.flow",
      ProcessEventValue.Analog(0.0)));
    var runtime = simulation.addRobot(blueprint);
    var robot = new FaultingArmRobot("process-arm", runtime, fixture.model.name,
      [for (link in fixture.model.links) link.name],
      [for (joint in fixture.model.joints) joint.name]);
    var solver = new ManipulatorKinematics(fixture.arm, 1e-8);
    var compiler = new ProgramCompiler(solver,
      new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(0)), "work",
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 4.0],
      [for (_ in 0...6) 20.0],
      StartTolerances.uniform(6, 0.02, 0.02, 0.02));
    var motion = new ManipulatorMotion(robot, compiler, (_) -> null,
      () -> runtime.pollEvents());
    var origin = [0.2, -0.4, 0.6, 0.1, 0.4, -0.2];
    var destination = origin.copy(); destination[0] += 0.2;
    var path = new PosePath("work", [new PoseLine(
      new PoseWaypoint(solver.forward(origin), 0.01, 0.04),
      new PoseWaypoint(solver.forward(destination), 0.01, 0.04),
      OrientationPolicy.Interpolated, 0.1, 0.05)]);
    var sprayer = new SimulatedSprayer();
    var adapter = new ChannelToolAdapter();
    adapter.bindSprayerFlow("paint.flow", sprayer, 1.0);
    var tick = 0;
    var device = new ChannelProcessDevice(adapter, "paint.flow",
      () -> Int64.ofInt(tick * 10000000));
    var recipe = new ProcessRecipe(0.05, 0.2, 0.05, 0.03,
      OrientationPolicy.Interpolated, 0.05, 2.0, 0.0, 0.03,
      FeedChangePolicy.Adapt);
    var run = new ProcessRun(recipe, path, device, "paint.flow", motion.session);
    var approach = origin.copy(); approach[0] -= 0.05;
    motion.run(new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget(approach), new MotionOptions(), Blend.ExactStop)]));
    for (_ in 0...600) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
      if (!motion.running) break;
    }
    check(motion.completed, 'arm reaches process start: ${motion.failure}');
    run.start(); run.update(0.0);
    check(run.state == Ready, "process device becomes ready on the virtual arm");
    motion.run(run.takeProgram());
    check(motion.running, 'initial process program starts: ${motion.failure}');
    var applied = 0;
    var distance = 0.0;
    for (_ in 0...1200) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
      var records = motion.firedEvents();
      run.applyRecords([for (i in applied...records.length) records[i]]);
      applied = records.length;
      distance = Math.max(distance, motion.progress().pathDistance);
      run.update(Math.min(distance, path.length()));
      if (sprayer.flow() > 0.0 && distance > path.length() * 0.4) break;
    }
    check(motion.running && sprayer.flow() > 0.0 && distance > path.length() * 0.4,
      'process flow is active midpass at $distance: ${motion.failure}');
    device.setFault("blocked nozzle");
    run.update(distance);
    check(run.state == ControlledInterruption && sprayer.flow() == 0.0,
      "process fault interrupts the pass and makes the tool safe");
    motion.abort();
    check(motion.sessionState() == Stopping(Discard),
      "arm stops before process recovery");
    robot.faultOverride = 42;
    motion.update(0.01);
    check(motion.sessionState() == Faulted,
      "runtime fault while stopping latches the shared session");
    device.setFault(null);
    run.update(distance);
    check(run.state == ControlledInterruption,
      "process recovery waits while the arm session is faulted");
    var settled = false;
    for (_ in 0...500) {
      simulationHarness.step(Int64.ofInt(tick++));
      var snapshot = runtime.snapshot();
      if (snapshot.sessionState == RobotKitRuntimeConstants.RK_SESSION_IDLE &&
          !snapshot.trajectoryActive && snapshot.trajectoryQueueDepth == 0) {
        settled = true;
        break;
      }
    }
    check(settled, "virtual arm settles before reset");
    robot.faultOverride = 0;
    motion.reset();
    check(motion.sessionState() == Idle, "explicit arm reset permits recovery");
    simulationHarness.step(Int64.ofInt(tick++));
    run.update(distance);
    check(run.state == Recovery, "process run enters recovery after arm reset");
    var continuation = run.takeProgram();
    var restart = Math.max(0.0, distance - recipe.recoveryBackoff);
    near(run.lastProgramStart, restart,
      "recovery backs up along the authored path", 1e-6);
    check(run.lastProgramStart < run.interruptedAt,
      "recovery overlaps the interrupted pass");
    var approachMatches = switch continuation.ops[0] {
      case MoveL(pose, _, _, _, _):
        motionkit.path.PoseMath.distance(pose,
          path.waypointAt(restart).pose) < 1e-6;
      case _: false;
    };
    check(approachMatches, "recovery approaches the backed-off path pose");
    motion.run(continuation);
    check(motion.running, 'recovery program starts from settled arm: ${motion.failure}');
    applied = 0;
    var flowRestored = false;
    var finalOffEvent = false;
    for (_ in 0...1200) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
      var records = motion.firedEvents();
      for (i in applied...records.length) {
        var record = records[i];
        if (record.channel == "paint.flow") switch record.value {
          case ProcessEventValue.Analog(value):
            if (value > 0.0) flowRestored = true;
            else if (flowRestored) finalOffEvent = true;
          case _:
        }
        run.applyRecords([record]);
      }
      applied = records.length;
      if (!motion.running) break;
    }
    check(motion.completed && motion.failure == null,
      'recovery program completes: ${motion.failure}');
    check(flowRestored, "runtime restores process output on the resumed pass");
    check(finalOffEvent && sprayer.flow() == 0.0,
      "runtime fires the final safe output event");
    run.update(path.length());
    check(run.state == Completion, "process run completes after resumed pass");
    simulationHarness.dispose();
  }

  public function testManipulatorSessionTransitions():Void {
    var fixture = buildContractArmFixture();
    for (joint in fixture.model.joints) joint.limits.maxAcceleration = 4.0;
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile()));
    var robot = new FaultingArmRobot("session-arm", runtime, fixture.model.name,
      [for (link in fixture.model.links) link.name],
      [for (joint in fixture.model.joints) joint.name]);
    var solver = new ManipulatorKinematics(fixture.arm, 1e-8);
    var compiler = new ProgramCompiler(solver,
      new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(0)), "work",
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 4.0],
      [for (_ in 0...6) 20.0],
      StartTolerances.uniform(6, 0.02, 0.02, 0.02));
    var motion = new ManipulatorMotion(robot, compiler, (_) -> null,
      () -> runtime.pollEvents());
    var target = [0.08, 0.0, 0.0, 0.0, 0.0, 0.0];
    var home = [for (_ in 0...6) 0.0];
    var longMove = new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget(target), new MotionOptions(), Blend.ExactStop)]);
    var homeMove = new MotionProgram([MotionOp.MoveJ(
      MoveTarget.JointTarget(home), new MotionOptions(), Blend.ExactStop)]);
    var tick = 0;
    motion.run(longMove);
    for (_ in 0...5) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
    }
    motion.abort();
    check(motion.sessionState() == Stopping(Discard) && motion.running &&
      robot.commandCount("abort") == 1,
      "arm abort keeps the program in Stopping while runtime slows");
    var commandsAtStop = robot.commands.length;
    motion.run(homeMove);
    check(motion.sessionState() == Stopping(Discard) && motion.running &&
      robot.commands.length == commandsAtStop,
      "arm accepts a replacement program during Stopping without submitting it");
    motion.hold(); motion.resume();
    check(motion.sessionState() == Stopping(Discard) &&
      robot.commands.length == commandsAtStop,
      "arm hold and resume during Stopping do not change state");
    for (_ in 0...800) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
      if (!motion.running) break;
    }
    check(motion.completed && motion.failure == null,
      'arm replacement starts after rest and completes: ${motion.failure}');
    near(robot.snapshot().positions.get(0), 0.0,
      "arm replacement reaches home from settled stop", 1e-3);

    motion.run(longMove);
    for (_ in 0...5) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
    }
    motion.hold();
    check(motion.sessionState() == Holding && robot.commandCount("hold") == 1,
      "arm running plus hold enters Holding");
    var resumesBefore = robot.commandCount("resume");
    motion.resume();
    check(motion.sessionState() == Holding &&
      robot.commandCount("resume") == resumesBefore,
      "arm resume during Holding waits for rest");
    for (_ in 0...800) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
      if (!motion.running) break;
    }
    check(motion.completed && robot.commandCount("resume") == resumesBefore + 1,
      'arm deferred resume sends one runtime command and completes: ${motion.failure}');

    motion.run(homeMove);
    for (_ in 0...5) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
    }
    // Let the owner finish while the host still has its last active plan.
    for (_ in 0...800) {
      simulationHarness.step(Int64.ofInt(tick++));
      if (!robot.snapshot().trajectoryActive) break;
    }
    check(!robot.snapshot().trajectoryActive, "the owner finishes before the host observes completion");
    var holdsBefore = robot.commandCount("hold");
    resumesBefore = robot.commandCount("resume");
    motion.hold();
    motion.update(0.01);
    simulationHarness.step(Int64.ofInt(tick++));
    check(motion.sessionState() == Held && robot.commandCount("hold") == holdsBefore &&
      robot.snapshot().faultCode == 0,
      "holding a finished owner plan stays local and preserves the program");
    motion.resume();
    for (_ in 0...800) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
      if (!motion.running) break;
    }
    check(motion.completed && motion.failure == null && robot.commandCount("resume") == resumesBefore,
      "resuming a hold at the finished plan advances without an empty runtime resume");

    motion.run(longMove);
    for (_ in 0...5) {
      motion.update(0.01); simulationHarness.step(Int64.ofInt(tick++));
    }
    motion.abort();
    var commandsBeforeFault = robot.commands.length;
    robot.faultOverride = 42;
    motion.update(0.01);
    check(motion.sessionState() == Faulted &&
      robot.commands.length == commandsBeforeFault,
      "arm snapshot fault during Stopping latches Faulted");
    throws(() -> motion.run(longMove),
      "arm fault blocks new programs until reset");
    robot.faultOverride = 0;
    motion.reset();
    check(motion.sessionState() == Idle,
      "arm explicit reset returns Faulted to Idle");
    simulationHarness.dispose();
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
