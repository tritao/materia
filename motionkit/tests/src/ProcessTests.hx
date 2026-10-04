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
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import robotkit.kinematics.RobotKinematics;
import robotkit.manipulation.Manipulator;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.Simulation;
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

import MotionKitTestSupport.WristBranchSolver;
import MotionKitTestSupport.PlanarSolver;
import MotionKitTestSupport.SessionTransitionRig;
import MotionKitTestSupport.LaggingRobot;
import MotionKitTestSupport.TrialRig;

class ProcessTests extends MotionKitTestSupport {
  public function new() { super(); }

  public function testPoseProcessPath():Void {
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

  public function testMotionEventContracts():Void {
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

  public function testLinearAxisCompilesToRobotModel():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    check(blueprint.model.links.length == 3, "linear axis compiles frame, carriage and screw rigid bodies");
    check(blueprint.model.joints.length == 2, "linear axis compiles travel and motor shaft joints");
    var joint = blueprint.model.joints[0];
    check(joint.id == "x/carriage-slide" && joint.name == "x/carriage-slide", "linear axis uses a stable joint ID");
    check(joint.type == robotkit.model.JointType.Prismatic, "linear axis is prismatic");
    near(joint.limits.upper, 0.08, "MachineKit stroke becomes the logical upper limit");
    near(joint.limits.requireVelocity(), Math.min(0.1, axis.motor.usableSpeed(24) /
      Math.abs(blueprint.model.couplings[0].ratio)), "compiled axis keeps the motor-derived rate");
    check(blueprint.model.actuators.length == 1 &&
      blueprint.model.actuators[0].id.indexOf(axis.motor.designation) >= 0,
      "compiled actuator retains motor identity");
    var expectedRatio = 2.0 * Math.PI /
      (axis.nut.travelPerRevolution() * MachineKitRobotCompiler.MILLIMETRES_TO_METRES);
    check(switch blueprint.model.actuators[0].transmission {
      case SimpleTransmission(jointId, ratio, offset):
        jointId == "x/coupling" && ratio == 1.0 && offset == 0.0;
    }, "the motor drives its physical shaft joint");
    near(blueprint.model.couplings[0].ratio, expectedRatio, "the resolved assembly supplies the screw ratio");
    check(expectedRatio < 0.0, "right-hand screw rotates negative to move the carriage along +Z");
    check(blueprint.model.actuators[0].assumed.indexOf("rotor inertia") >= 0,
      "physical axis compilation retains its motor assumption labels");
    near(blueprint.axes[0].jointScales[0], 1.0,
      "transmission-derived single-joint mapping keeps the old scale");
    near(blueprint.axes[0].jointOffsets[0], 0.0,
      "transmission-derived single-joint mapping keeps the old offset");
  }

  public function testLeadScrewActuatorRateLimitsPlans():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    var actuator = blueprint.model.actuators[0];
    var ratio = Math.abs(blueprint.model.couplings[0].ratio);
    actuator.maxRate = 0.02 * ratio;
    blueprint.model.materializeLimits();
    var limited = RobotRuntimeCompiler.compile(blueprint.model, new robotkit.profile.RobotProfile());
    near(limited.joints[0].requireRate(), 0.02,
      "lead-screw motor rate converts to the tighter joint-space limit");
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
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
    simulationHarness.dispose();
  }

  public function testCompiledAxisRunsThroughSimulation():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("single-axis", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);

    machine.home();
    check(machine.isMoving(), "home creates a motion command");
    machine.update();
    simulationHarness.step(Int64.ofInt(0));
    check(!machine.isMoving(), "zero-distance home completes deterministically");

    machine.moveAxes([new AxisTarget("x", 0.04)], new MotionOptions(0.08, 0.4));
    var tick = 1;
    while (machine.isMoving()) {
      machine.update();
      simulationHarness.step(Int64.ofInt(tick++));
      if (tick > 1000) throw "single-axis trajectory did not complete";
    }
    for (_ in 0...4) simulationHarness.step(Int64.ofInt(tick++));
    var snapshot = robot.snapshot();
    near(snapshot.positions.get(0), 0.04, "simulated robot reaches the MotionKit axis target", 1e-5);
    simulationHarness.dispose();
  }

  public function testHomingAndJogging():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("jog-axis", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);

    machine.home();
    runMotion(machine, simulationHarness);
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
    runMotion(machine, simulationHarness);
    near(robot.snapshot().positions.get(0), 0.02,
      "positive jog reaches its target", 1e-5);

    machine.jog("x", -0.01, 0.5);
    runMotion(machine, simulationHarness);
    near(robot.snapshot().positions.get(0), 0.015,
      "negative jog follows the same logical axis API", 1e-5);

    var jogging = machine.axis("x");
    if (jogging == null) throw "compiled axis has no logical x mapping";
    var clamped = planned(machine.jog("x", jogging.maxVelocity, 2.0));
    near(clamped.evaluate(clamped.durationSeconds()).positions[0], 0.08,
      "jog clamps its endpoint to the authored upper limit");
    runMotion(machine, simulationHarness);
    near(robot.snapshot().positions.get(0), 0.08,
      "clamped jog stops at the axis limit", 1e-5);
    throws(function() machine.jog("x", jogging.maxVelocity * 1.001, 1.0),
      "jog rejects a velocity above the axis rate limit");
    throws(function() machine.jog("x", 0.0, 1.0),
      "jog rejects a zero velocity");
    simulationHarness.dispose();
  }

  public function testCompiledXYZGantryRunsThroughSimulation():Void {
    var xAxis = new LinearAxis(23, 10, 80);
    var yAxis = new LinearAxis(23, 10, 60);
    var zAxis = new LinearAxis(23, 10, 40);
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(xAxis, yAxis, zAxis, 0.1, 0.4);
    check(blueprint.model.links.length == 7, "XYZ gantry compiles its frame, three carriages and three motor shafts");
    check(blueprint.model.joints.length == 6, "XYZ gantry retains three travels and three shaft joints");
    for (i in 0...3) {
      var joint = blueprint.model.joints[i];
      check(joint.id == ["x/carriage-slide", "y/carriage-slide", "z/carriage-slide"][i], "XYZ gantry uses stable joint IDs");
      check(joint.type == robotkit.model.JointType.Prismatic,
        "XYZ gantry joints are prismatic");
      near(joint.limits.requireVelocity(), Math.min(0.1, [xAxis, yAxis, zAxis][i].motor.usableSpeed(24) /
        Math.abs(blueprint.model.couplings[i].ratio)), "XYZ gantry retains each physical motor rate limit");
    }
    // The carriage is a link, not a flange frame: use the compiled model directly.
    var gantry = RobotKinematics.compile(blueprint.model);
    var carriage = gantry.bodyIndex("z/carriage");
    var gantrySnapshot = KinematicSnapshot.of(new KinematicState(gantry));
    var tip = gantrySnapshot.bodyPose(carriage);
    near(tip.x, (xAxis.screwStart + xAxis.travelMin - xAxis.carriage.connector("bore").frame.z) * 0.001,
      "XYZ gantry forward kinematics preserves the physical X carriage origin");
    near(tip.y, (yAxis.screwStart + yAxis.travelMin - yAxis.carriage.connector("bore").frame.z) * 0.001,
      "XYZ gantry forward kinematics preserves the physical Y carriage origin");
    near(tip.z, (zAxis.screwStart + zAxis.travelMin - zAxis.carriage.connector("bore").frame.z) * 0.001,
      "XYZ gantry forward kinematics preserves the physical Z carriage origin");
    if (gantry.dofCount() != 3 || gantry.bodyParentJoint[gantry.bodyIndex("assembly-root")] >= 0)
      throw "XYZ gantry should retain three independent DOFs and their coupled shafts under its root base";
    var flatJacobian = gantrySnapshot.bodyJacobian(carriage);
    var jacobian = [for (row in 0...6) [for (column in 0...3) flatJacobian[row * gantry.dofCount() + column]]];
    near(jacobian[0][0], 1.0, "XYZ gantry X joint moves along world X");
    near(jacobian[1][1], 1.0, "XYZ gantry Y joint moves along world Y");
    near(jacobian[2][2], 1.0, "XYZ gantry Z joint moves along world Z");
    near(jacobian[1][0], 0.0, "XYZ gantry X joint has no world Y component");
    near(jacobian[2][0], 0.0, "XYZ gantry X joint has no world Z component");
    near(jacobian[0][1], 0.0, "XYZ gantry Y joint has no world X component");
    near(jacobian[2][1], 0.0, "XYZ gantry Y joint has no world Z component");
    near(jacobian[0][2], 0.0, "XYZ gantry Z joint has no world X component");
    near(jacobian[1][2], 0.0, "XYZ gantry Z joint has no world Y component");

    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("xyz-gantry", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    check(machine.axis("x") != null && machine.axis("y") != null && machine.axis("z") != null,
      "XYZ gantry exposes all logical axes");

    machine.home();
    runMotion(machine, simulationHarness);
    var home = robot.snapshot();
    near(home.positions.get(0), 0.0, "XYZ gantry homes X");
    near(home.positions.get(1), 0.0, "XYZ gantry homes Y");
    near(home.positions.get(2), 0.0, "XYZ gantry homes Z");
    throws(function() machine.moveAxes([new AxisTarget("x", 0.081)]),
      "XYZ gantry rejects an out-of-range axis target");

    machine.moveAxes([
      new AxisTarget("x", 0.02), new AxisTarget("y", 0.01), new AxisTarget("z", 0.015)
    ], new MotionOptions(0.08, 0.4));
    runMotion(machine, simulationHarness);
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
    runMotion(machine, simulationHarness);
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
    var ids = [for (joint in blueprint.model.joints) joint.id];
    var scales = [for (_ in ids) 0.0];
    for (axis in blueprint.axes) for (slot in 0...axis.jointIds.length)
      scales[ids.indexOf(axis.jointIds[slot])] = Math.abs(axis.jointScales[slot]);
    var velocity = [for (joint in blueprint.model.joints) joint.limits.requireVelocity()];
    var acceleration = [for (joint in blueprint.model.joints) joint.limits.requireAcceleration()];
    var jerk = [for (scale in scales) 10.0 * scale];
    var limits = new ValidationLimits(ids.length, Int64.ofInt(blueprint.runtime.revision),
      Int64.ofInt(blueprint.runtime.calibrationRevision));
    for (joint in 0...ids.length) {
      var bound = blueprint.model.joints[joint].limits;
      limits.position(joint, bound.lower, bound.upper);
      limits.velocity(joint, velocity[joint]);
      limits.acceleration(joint, acceleration[joint]);
      limits.jerk(joint, jerk[joint]);
    }
    var compiler = new ProgramCompiler(axisSolver, limits, "work", velocity, acceleration, jerk,
      new StartTolerances([for (scale in scales) 0.02 * scale],
        [for (scale in scales) 0.02 * scale], [for (scale in scales) 0.02 * scale]),
      null, 0.005, 0.5, 0.005, 0.02, null, null,
      [for (scale in scales) 0.5 * scale], ids, blueprint.model.couplings);
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
    // The sharp corner is an exact stop: each side of it is its own plan.
    var programPlans = compiled.blocks[0].plans;
    check(programPlans.length == 2 && compiled.blocks[0].opIndices.join(",") == "0,0",
      "a followed path stops at its sharp corner, in two plans of one op");
    var programPlan = programPlans[programPlans.length - 1];
    var programEnd = programPlan.evaluate(programPlan.durationSeconds).positions;
    var movePathEnd = cornerMove.evaluate(cornerMove.durationSeconds()).positions;
    for (joint in 0...3)
      near(programEnd[joint], movePathEnd[joint],
        'gantry ProgramCompiler matches movePath joint $joint', 1e-5);
    compiled.dispose();
    runMotion(machine, simulationHarness);
    var cornerEnd = robot.snapshot();
    near(cornerEnd.positions.get(0), 0.04, "line path reaches its X endpoint", 1e-5);
    near(cornerEnd.positions.get(1), 0.03, "line path reaches its Y endpoint", 1e-5);
    simulationHarness.dispose();
  }

  public function testMachineKitLeadScrewThroughVirtualDevice():Void {
    var screw = new LeadScrewThread(MetricTrapezoidal, 10, 2, 4);
    var axis = new LinearAxis(23, 10, 80, null, 30, screw);
    near(axis.nut.travelPerRevolution(), -8.0, "four-start right-hand screw moves -8 mm per positive revolution");
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.01, 0.04);
    var ratio = 2.0 * Math.PI /
      (axis.nut.travelPerRevolution() * MachineKitRobotCompiler.MILLIMETRES_TO_METRES);
    var options = new VirtualDeviceOptions();
    // 200 full steps at 16 microsteps a turn.
    var binding = DeviceBinding.bind(blueprint.model,
      new DeviceLayout([for (index in 0...blueprint.model.actuators.length)
        new robotkit.device.DeviceChannel(index, blueprint.model.actuators[index].id, 1, 2)]), options.stepTickHz);
    near(blueprint.model.couplings[0].ratio, ratio, "the physical coupling takes its ratio from the screw");
    near(binding.channels[0].ratio, 1.0, "the binding drives the explicit motor shaft in radians");
    check(blueprint.model.joints[binding.channels[0].jointIndex].id == blueprint.axes[0].jointIds[1],
      "the virtual channel addresses the physical motor shaft");
    options.actuators = binding.virtualActuators();
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime, null, options);
    for (tick in 1...21) simulationHarness.step(Int64.ofInt(tick));
    var robot = new SimulatedRobot("lead-screw-virtual", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var carriage = -1;
    for (index in 0...blueprint.model.links.length)
      if (blueprint.model.links[index].id == "x/carriage") carriage = index;
    if (carriage < 0) throw "compiled axis has no physical carriage";
    var before = simulation.linkPose(0, carriage);
    machine.moveAxes([new AxisTarget("x", 0.02)], new MotionOptions(0.01, 0.04));
    runMotion(machine, simulationHarness);
    for (tick in 0...20) simulationHarness.step(Int64.ofInt(tick));
    var finalJoint = robot.snapshot().positions.get(0);
    check(Math.abs(finalJoint - 0.02) <= 1.0 / 400000.0 + 1e-6,
      "MachineKit lead screw follows the virtual RKD6 step position");
    var after = simulation.linkPose(0, carriage);
    check(Math.abs(after.position[2] - before.position[2] - 0.02) <=
      1.0 / 400000.0 + 1e-6, "lead screw step position moves the SimKit carriage");
    simulationHarness.dispose();
  }

}
