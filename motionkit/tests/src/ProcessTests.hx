import haxe.Int64;
import haxe.io.Bytes;
import cnckit.CncMachine;
import toolpathkit.tool.Tool;
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
import robotkit.runtime.SimulationHarness;
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

  public function testLeadScrewActuatorRateLimitsPlans():Void {
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

    var clamped = planned(machine.jog("x", 0.1, 2.0));
    near(clamped.evaluate(clamped.durationSeconds()).positions[0], 0.08,
      "jog clamps its endpoint to the authored upper limit");
    runMotion(machine, simulationHarness);
    near(robot.snapshot().positions.get(0), 0.08,
      "clamped jog stops at the axis limit", 1e-5);
    throws(function() machine.jog("x", 0.1001, 1.0),
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
    var limits = new ValidationLimits(3, Int64.ofInt(blueprint.runtime.revision),
      Int64.ofInt(blueprint.runtime.calibrationRevision));
    for (joint in 0...3) limits.position(joint,
      blueprint.model.joints[joint].limits.lower,
      blueprint.model.joints[joint].limits.upper);
    var compiler = new ProgramCompiler(axisSolver, limits, "work",
      [for (_ in 0...3) 0.1], [for (_ in 0...3) 0.4],
      [for (_ in 0...3) 10.0], StartTolerances.uniform(3, 0.02, 0.02, 0.02),
      null, 0.005);
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
    runMotion(machine, simulationHarness);
    var cornerEnd = robot.snapshot();
    near(cornerEnd.positions.get(0), 0.04, "line path reaches its X endpoint", 1e-5);
    near(cornerEnd.positions.get(1), 0.03, "line path reaches its Y endpoint", 1e-5);
    simulationHarness.dispose();
  }

  public function testPhysicalAssemblyCncBinding():Void {
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
    var result = MotionKitTestSupport.compileCnc(MotionKitTestSupport.cncBinding(cnc, blueprint), cnc,
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
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobotAtPose(blueprint.runtime,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], null, null,
      physical.linkCollisionHulls);
    var robot = new SimulatedRobot("physical-cnc", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var binding = MotionKitTestSupport.cncBinding(cnc, blueprint);
    var motion = new ManipulatorMotion(robot, binding.compiler,
      function(channel) return channel == "spindle.at_speed" ?
        EventValue.Digital(true) : null,
      function() return runtime.pollEvents());
    motion.run(MotionKitTestSupport.cncProgram(cnc,
      "G21 G90 G17\nS12000 M3\nG0 X10 Y10\nF600 G1 X20\nG3 X10 Y20 I-10 J0\nM5\nM2\n"));
    for (tick in 0...3000) {
      motion.update(0.01);
      simulationHarness.step(Int64.ofInt(tick));
      if (!motion.running) break;
    }
    check(motion.completed, 'physical assembly CNC completes: ${motion.failure}');
    var finalPose = kinematics.forward(robot.snapshot().positions.toArray());
    near(finalPose.x, 0.01, "physical assembly CNC executes X", 2e-4);
    near(finalPose.y, 0.02, "physical assembly CNC executes Y", 2e-4);
    simulationHarness.dispose();
  }

  public function testCncProgramBinding():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 200), 0.1, 0.4);
    var cnc = new CncMachine("work", "x", "y", "z", 0.08);
    var binding = MotionKitTestSupport.cncBinding(cnc, blueprint);
    check(cnc.travelLower != null && cnc.travelUpper != null,
      "CNC binding derives a machine travel envelope");
    var travelError = "";
    try MotionKitTestSupport.compileCnc(binding, cnc, "G21 G0 X500\nM2\n", [0.0, 0.0, 0.0],
      Int64.ofInt(899))
    catch (error:Dynamic) travelError = Std.string(error);
    check(travelError.indexOf("G-code line 1") >= 0 &&
      travelError.indexOf("X travel") >= 0,
      'bound CNC travel error names the G-code line and axis: $travelError');
    var result = MotionKitTestSupport.compileCnc(binding, cnc, "G21 G90 G17\nS12000 M3\nG0 X10 Y10\n" +
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
    cnc.toolLibrary.set(new Tool(2, 0.0, 0.002));
    var compensated = MotionKitTestSupport.compileCnc(binding, cnc, "G21 G90 F600 G41 D2 G1 X10\n" +
      "G1 X20\nG1 X20 Y10\nG40 G1 X20 Y20\nM2\n",
      [0.0, 0.0, 0.0], Int64.ofInt(925));
    check(compensated.blocks.length > 0,
      "compensated contour lowers through MotionKit");
    compensated.dispose();
    for (arc in ["G17 G2 X5 Y5 Z5 I5 J0",
        "G18 G3 X5 Y5 Z5 I5 K0", "G19 G2 X5 Y5 Z5 J5 K0"]) {
      var helix = MotionKitTestSupport.compileCnc(binding, cnc, 'G21 G90 F600 $arc\nM2\n',
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

  public function testVirtualCncProgram():Void {
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

  public function testMachineKitLeadScrewThroughVirtualDevice():Void {
    var screw = new LeadScrewThread(MetricTrapezoidal, 10, 2, 4);
    var axis = new LinearAxis(23, 10, 80, null, 30, screw);
    near(axis.nut.travelPerRevolution(), -8.0, "four-start right-hand screw moves -8 mm per positive revolution");
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.01, 0.04);
    var ratio = 2.0 * Math.PI /
      (axis.nut.travelPerRevolution() * MachineKitRobotCompiler.MILLIMETRES_TO_METRES);
    var options = new VirtualDeviceOptions();
    options.actuators = [new VirtualActuatorOptions(blueprint.model.actuators[0].id, 0, ratio, 0.0,
      3200.0 / (2.0 * Math.PI), 0.01 * Math.abs(ratio), 2)];
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime, null, options);
    for (tick in 1...21) simulationHarness.step(Int64.ofInt(tick));
    var robot = new SimulatedRobot("lead-screw-virtual", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var before = simulation.linkPose(0, 1);
    machine.moveAxes([new AxisTarget("x", 0.02)], new MotionOptions(0.01, 0.04));
    runMotion(machine, simulationHarness);
    for (tick in 0...20) simulationHarness.step(Int64.ofInt(tick));
    var finalJoint = robot.snapshot().positions.get(0);
    check(Math.abs(finalJoint - 0.02) <= 1.0 / 400000.0 + 1e-6,
      "MachineKit lead screw follows the virtual RKD6 step position");
    var after = simulation.linkPose(0, 1);
    check(Math.abs(after.position[2] - before.position[2] - 0.02) <=
      1.0 / 400000.0 + 1e-6, "lead screw step position moves the SimKit carriage");
    simulationHarness.dispose();
  }

}
