import haxe.Int64;
import machinekit.assembly.LinearAxis;
import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MachineKitRobotCompiler;
import motionkit.MotionOptions;
import motionkit.MotionSystem;
import motionkit.Pose;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.axis.MotionSystemBlueprint;
import motionkit.path.ArcSegment;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.planner.LineLookaheadPlanner;
import motionkit.planner.PathPlanningOptions;
import motionkit.planner.TrapezoidalPlanner;
import motionkit.trajectory.JointTrajectory;
import motionkit.trajectory.JointTrajectorySample;
import motionkit.trajectory.MotionLimits;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.manipulation.ChainTip;
import robotkit.manipulation.KinematicChain;
import robotkit.runtime.Simulation;
import robotkit.world.RecordingRobot;
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
import motionkit.planner.JogProfile;

class MotionKitBootstrapTests {
  static var assertions:Int = 0;

  public static function main():Void {
    testGeometricPathPrimitives();
    testPlannerIsDeterministicAndBounded();
    testLineLookaheadPlanner();
    testLinearAxisCompilesToRobotModel();
    testCompiledAxisRunsThroughSimulation();
    testHomingAndJogging();
    testMoveLinearUsesPlannerLimits();
    testCompiledXYZGantryRunsThroughSimulation();
    testDualMotorAxisRunsThroughSimulation();
    testBufferedExecution();
    testLongBufferedExecution();
    testHoldRefillsNearChunkBoundary();
    testHoldDecelerationStaysWithinLimitsThroughoutMove();
    testRuntimeSynchronizedHolding();
    testImmediateMotionReplacesNativeQueue();
    testMotionChangesStayWithinLimits();
    testJogProfile();
    testContinuousJog();
    testLateJogSpliceFallsBackToStop();
    testPathHoldsStayOnPathWithinLimits();
    testDualMotorAxisChangesStayWithinJointLimits();
    Sys.println('MotionKit bootstrap tests passed ($assertions assertions)');
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

  static function testPlannerIsDeterministicAndBounded():Void {
    var planner = new TrapezoidalPlanner(0.01);
    var limits = new MotionLimits(1.0, 2.0, 10.0);
    var first = planner.plan([0.0, 0.0], [1.0, -0.25], limits);
    var second = planner.plan([0.0, 0.0], [1.0, -0.25], limits);
    check(first.durationSeconds > 0.0, "planner produces a timed trajectory");
    check(first.samples.length == second.samples.length, "planner sample count is deterministic");
    for (i in 0...first.samples.length) {
      near(first.samples[i].timeSeconds, second.samples[i].timeSeconds,
        "planner sample time is deterministic");
      for (joint in 0...2) {
        near(first.samples[i].positions[joint], second.samples[i].positions[joint],
          "planner position is deterministic");
        check(Math.abs(first.samples[i].velocities[joint]) <= limits.maxVelocity + 1e-6,
          "planner respects velocity limit");
        check(Math.abs(first.samples[i].accelerations[joint]) <= limits.maxAcceleration + 1e-6,
          "planner respects acceleration limit");
      }
    }
    near(first.sample(0.0).positions[0], 0.0, "trajectory starts at the requested position");
    near(first.sample(first.durationSeconds).positions[0], 1.0,
      "trajectory ends at the requested position");
  }

  static function testLineLookaheadPlanner():Void {
    var path = GeometricPath.lines([new PathPoint(0.0, 0.0, 0.0),
      new PathPoint(0.1, 0.0, 0.0), new PathPoint(0.1, 0.1, 0.0)]);
    var planner = new LineLookaheadPlanner(0.01);
    var limits = new MotionLimits(1.0, 2.0, 0.0);
    var exact = planner.planPath(path, limits, PathPlanningOptions.exactStopMode());
    var blend = planner.planPath(path, limits, PathPlanningOptions.blend(0.01));
    check(exact.durationSeconds > blend.durationSeconds,
      "blending shortens a cornered path without changing its endpoints");

    var exactCorner = findCornerSample(exact);
    var blendCorner = findCornerSample(blend);
    near(exactCorner.velocities[0], 0.0, "exact-stop corner has no X velocity");
    near(exactCorner.velocities[1], 0.0, "exact-stop corner has no Y velocity");
    check(Math.sqrt(blendCorner.velocities[0] * blendCorner.velocities[0] +
      blendCorner.velocities[1] * blendCorner.velocities[1]) > 1e-6,
      "blend corner retains continuous path speed");

    for (trajectory in [exact, blend]) {
      for (i in 0...101) {
        var sample = trajectory.sample(trajectory.durationSeconds * i / 100.0);
        var onFirst = Math.abs(sample.positions[1]) <= 1e-7 &&
          sample.positions[0] >= -1e-7 && sample.positions[0] <= 0.1000001;
        var onSecond = Math.abs(sample.positions[0] - 0.1) <= 1e-7 &&
          sample.positions[1] >= -1e-7 && sample.positions[1] <= 0.1000001;
        check(onFirst || onSecond, "lookahead samples stay on the authored polyline");
      }
    }

    var repeated = planner.planPath(path, limits, PathPlanningOptions.blend(0.01));
    check(repeated.samples.length == blend.samples.length,
      "lookahead planning is deterministic");
    for (i in 0...blend.samples.length) {
      near(repeated.samples[i].timeSeconds, blend.samples[i].timeSeconds,
        "lookahead sample time is deterministic");
      near(repeated.samples[i].positions[0], blend.samples[i].positions[0],
        "lookahead position is deterministic");
      near(repeated.samples[i].positions[1], blend.samples[i].positions[1],
        "lookahead corner position is deterministic");
    }

    var shallowAngle = Math.PI / 18.0;
    var nearReversalAngle = Math.PI * 170.0 / 180.0;
    var shallowPath = GeometricPath.lines([new PathPoint(0.0, 0.0, 0.0),
      new PathPoint(0.1, 0.0, 0.0),
      new PathPoint(0.1 + 0.1 * Math.cos(shallowAngle), 0.1 * Math.sin(shallowAngle), 0.0)]);
    var nearReversalPath = GeometricPath.lines([new PathPoint(0.0, 0.0, 0.0),
      new PathPoint(0.1, 0.0, 0.0),
      new PathPoint(0.1 + 0.1 * Math.cos(nearReversalAngle),
        0.1 * Math.sin(nearReversalAngle), 0.0)]);
    var shallow = planner.planPath(shallowPath, limits, PathPlanningOptions.blend(0.01));
    var nearReversal = planner.planPath(nearReversalPath, limits,
      PathPlanningOptions.blend(0.01));
    var shallowCorner = findCornerSampleAt(shallow, 0.1, 0.0);
    var nearReversalCorner = findCornerSampleAt(nearReversal, 0.1, 0.0);
    var shallowSpeed = Math.sqrt(shallowCorner.velocities[0] * shallowCorner.velocities[0] +
      shallowCorner.velocities[1] * shallowCorner.velocities[1]);
    var nearReversalSpeed = Math.sqrt(nearReversalCorner.velocities[0] *
      nearReversalCorner.velocities[0] + nearReversalCorner.velocities[1] *
      nearReversalCorner.velocities[1]);
    check(shallowSpeed > 0.5, "a shallow line bend retains high blend speed");
    check(nearReversalSpeed < 0.2, "a near-reversal line bend slows for the corner");
    check(shallowSpeed > nearReversalSpeed * 4.0,
      "corner blend speed decreases as the interior angle closes");

    var arc = new ArcSegment(new PathPoint(0.1, 0.1, 0.0), 0.1,
      -Math.PI * 0.5, Math.PI * 0.5);
    var arcTrajectory = planner.planPath(new GeometricPath([arc]),
      new MotionLimits(0.5, 1.0), PathPlanningOptions.exactStopMode());
    for (sample in arcTrajectory.samples) {
      var dx = sample.positions[0] - 0.1;
      var dy = sample.positions[1] - 0.1;
      near(Math.sqrt(dx * dx + dy * dy), 0.1,
        "arc lookahead source samples stay on the authored circle", 1e-5);
      var accelerationNorm = Math.sqrt(sample.accelerations[0] * sample.accelerations[0] +
        sample.accelerations[1] * sample.accelerations[1] +
        sample.accelerations[2] * sample.accelerations[2]);
      check(accelerationNorm <= 1.0 + 1e-6,
        "arc acceleration stays within the combined acceleration budget");
    }
    near(arcTrajectory.samples[arcTrajectory.samples.length - 1].positions[0], 0.2,
      "arc planner reaches its endpoint");
  }

  static function findCornerSample(trajectory:motionkit.trajectory.JointTrajectory):motionkit.trajectory.JointTrajectorySample {
    return findCornerSampleAt(trajectory, 0.1, 0.0);
  }

  static function findCornerSampleAt(trajectory:motionkit.trajectory.JointTrajectory,
      x:Float, y:Float):motionkit.trajectory.JointTrajectorySample {
    for (sample in trajectory.samples)
      if (Math.abs(sample.positions[0] - x) <= 1e-7 && Math.abs(sample.positions[1] - y) <= 1e-7)
        return sample;
    throw "lookahead trajectory did not emit its corner sample";
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
    check(joint.drive != null && joint.drive.name.indexOf(axis.motor.designation) >= 0,
      "compiled actuator retains motor identity");
    check(joint.drive != null && joint.drive.name.indexOf("mm/rev") >= 0,
      "compiled actuator retains transmission identity");
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
    near(forward.samples[0].velocities[0], 0.0,
      "jog starts at rest");
    near(forward.samples[forward.samples.length - 1].velocities[0], 0.0,
      "jog ends at rest");
    for (sample in forward.samples)
      check(Math.abs(sample.accelerations[0]) <= 0.4 + 1e-9,
        "jog respects its acceleration limit");
    near(forward.samples[forward.samples.length - 1].positions[0], 0.02,
      "jog plans the requested logical displacement");
    runMotion(machine, simulation);
    near(robot.snapshot().positions.get(0), 0.02,
      "positive jog reaches its target", 1e-5);

    machine.jog("x", -0.01, 0.5);
    runMotion(machine, simulation);
    near(robot.snapshot().positions.get(0), 0.015,
      "negative jog follows the same logical axis API", 1e-5);

    var clamped = planned(machine.jog("x", 0.1, 2.0));
    near(clamped.samples[clamped.samples.length - 1].positions[0], 0.08,
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
    var peakAcceleration = 0.0;
    for (sample in xOnly.samples)
      peakAcceleration = Math.max(peakAcceleration, Math.abs(sample.accelerations[0]));
    check(peakAcceleration > 1.9,
      "moveLinear uses an authored 2 m/s² acceleration limit");

    var diagonal = planned(machine.moveLinear(Pose.xyz(0.03, 0.03, 0.03),
      Feed.metresPerSecond(0.2), options));
    for (sample in diagonal.samples) {
      for (joint in 0...3)
        check(Math.abs(sample.accelerations[joint]) <= 2.0 + 1e-9,
          "diagonal moveLinear samples stay within per-axis acceleration caps");
    }
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
    var midpoint = linear.sample(linear.durationSeconds * 0.5);
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
    var cornerMove = machine.movePath(cornerPath, PathPlanningOptions.blend(0.001),
      new MotionOptions(0.05, 0.2));
    check(cornerMove.samples.length > 2, "MotionSystem exposes buffered line-path planning");
    runMotion(machine, simulation);
    var cornerEnd = robot.snapshot();
    near(cornerEnd.positions.get(0), 0.04, "line path reaches its X endpoint", 1e-5);
    near(cornerEnd.positions.get(1), 0.03, "line path reaches its Y endpoint", 1e-5);
    simulation.dispose();
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

  static function runMotion(machine:MotionSystem, simulation:Simulation):Void {
    var tick = 0;
    while (machine.isMoving()) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
      if (tick > 2000) throw "MotionKit trajectory did not complete";
    }
    for (_ in 0...4) simulation.step(Int64.ofInt(tick++));
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
    check(machine.queuedDurationSeconds() > first.durationSeconds,
      "buffer reports the duration of waiting motion");
    near(second.samples[0].positions[0], 0.02,
      "queued axis motion starts at the previous trajectory endpoint");
    near(machine.progress(), 0.0, "buffer starts with zero progress");
    check(recording.commands.length == 1, "buffer submits the first move as one chunk");
    switch recording.commands[0] {
      case TrajectoryChunk(chunk):
        check(chunk.points.length > 1, "trajectory chunk carries timestamped samples");
      case JointTargets(_, _):
        throw "buffer unexpectedly fell back to sample-by-sample targets";
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
    while (runtime.snapshot().trajectoryActive) {
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
    var samples:Array<JointTrajectorySample> = [];
    for (index in 0...601)
      samples.push(new JointTrajectorySample(index * 0.01, [0.05 * index / 600.0]));
    var trajectory = new JointTrajectory(samples);
    machine.queueTrajectory(trajectory);
    check(recording.commands.length == 1, "long trajectory starts with one bounded native chunk");

    var tick = 0;
    machine.update();
    check(recording.commands.length == 1,
      "streamer waits for the owner cycle before appending a second chunk");
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
        check(chunk.points.length <= robotkit.world.TrajectoryChunk.MAX_POINTS,
          "streamed trajectory chunks stay within the native point limit");
      case RobotCommand.JointTargets(_, _):
        throw "long trajectory unexpectedly fell back to sample-by-sample targets";
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
        var runtimeTime = Std.parseFloat(Int64.toStr(snapshot.trajectoryTagTimeNs)) /
          1000000000.0;
        check(Math.abs(progress - Math.min(1.0, runtimeTime / move.durationSeconds)) < 1e-6,
          "progress follows the runtime trajectory tag clock");
      }
    }

    for (cycle in 0...2) {
      machine.hold();
      var stopTicks = 0;
      while (runtime.snapshot().trajectoryActive) {
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
    while (queuedRuntime.snapshot().trajectoryActive) {
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
    squareMachine.movePath(GeometricPath.lines([
      new PathPoint(0.0, 0.0, 0.0), new PathPoint(0.02, 0.0, 0.0),
      new PathPoint(0.02, 0.02, 0.0), new PathPoint(0.0, 0.02, 0.0),
      new PathPoint(0.0, 0.0, 0.0)
    ]), PathPlanningOptions.exactStopMode(), new MotionOptions(0.04, 0.2));
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
    while (squareRuntime.snapshot().trajectoryActive) {
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
    var samples:Array<JointTrajectorySample> = [];
    for (index in 0...2001)
      samples.push(new JointTrajectorySample(index * 0.005, [0.06 * index / 2000.0]));
    machine.queueTrajectory(new JointTrajectory(samples));

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
    while (runtime.snapshot().trajectoryActive) {
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
      if (moveTicks == 0) moveTicks = Math.ceil(move.durationSeconds / 0.01);
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
      check(!runtime.snapshot().trajectoryActive, 'hold at tick $holdTick finishes its stop');
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
    check(recording.commands.length == 4,
      "immediate replacement submits a runtime flush before each trajectory");
    switch recording.commands[2] {
      case JointTargets(_, _):
        check(true, "immediate replacement flush is ordered before its new chunk");
      case TrajectoryChunk(_):
        throw "immediate replacement submitted its chunk before the runtime flush";
    }
    switch recording.commands[3] {
      case TrajectoryChunk(_):
        check(true, "immediate replacement submits the new trajectory after its flush");
      case JointTargets(_, _):
        throw "immediate replacement did not submit a trajectory after its flush";
    }
    runMotion(machine, simulation);
    near(robot.snapshot().positions.get(0), 0.01,
      "immediate motion replaces stale native trajectory motion", 1e-5);
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
   * within the joint acceleration limit, whether the robot executes buffered
   * chunks or MotionKit streams position targets itself.
   */
  static function testMotionChangesStayWithinLimits():Void {
    var limit = 0.4;
    for (queueSupport in [true, false]) {
      var label = queueSupport ? "buffered" : "position-target";
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

  static function testJogProfile():Void {
    var limit = 0.4;
    function checkProfile(label:String, profile:JogProfile, start:Float, startVelocity:Float,
        expectedEnd:Float):Void {
      near(profile.positionAt(0.0), start, '$label starts at its position', 1e-12);
      near(profile.velocityAt(0.0), startVelocity, '$label starts at its velocity', 1e-12);
      near(profile.endPosition, expectedEnd, '$label rests at its end', 1e-12);
      near(profile.velocityAt(profile.durationSeconds), 0.0, '$label ends at rest', 1e-12);
      var step = 0.001;
      var count = Std.int(profile.durationSeconds / step) + 2;
      for (index in 1...count) {
        var change = Math.abs(profile.velocityAt(index * step) - profile.velocityAt((index - 1) * step));
        check(change <= limit * step * (1.0 + 1e-9) + 1e-12, '$label stays within its acceleration');
      }
    }
    checkProfile("jog from rest", new JogProfile(0.0, 0.0, 0.1, 0.05, limit), 0.0, 0.0, 0.1);
    checkProfile("slowing jog", new JogProfile(0.0, 0.08, 0.2, 0.02, limit), 0.0, 0.08, 0.2);
    checkProfile("speeding jog", new JogProfile(0.0, 0.02, 0.2, 0.08, limit), 0.0, 0.02, 0.2);
    checkProfile("reversing jog", new JogProfile(0.05, 0.05, 0.0, 0.05, limit), 0.05, 0.05, 0.0);
    // Braking from 0.05 m/s at 0.4 m/s^2 needs 3.125 mm, past a 1 mm target.
    checkProfile("overrunning jog", new JogProfile(0.0, 0.05, 0.001, 0.05, limit), 0.0, 0.05,
      0.05 * 0.05 / (2.0 * limit));
  }

  /**
   * A jog issued while the same axis is jogging changes speed or direction
   * without stopping first, within the acceleration limit, on both paths.
   */
  static function testContinuousJog():Void {
    var limit = 0.4;
    for (queueSupport in [true, false]) {
      var label = queueSupport ? "buffered" : "position-target";
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
   * When a jog change reaches the runtime after its splice point, the
   * runtime keeps the old jog and MotionKit recovers by stopping along it and
   * starting the new jog from rest, still within the limit.
   */
  static function testLateJogSpliceFallsBackToStop():Void {
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
    robot.release();
    var stillTicks = 0;
    while (machine.isMoving() || stillTicks < 3) {
      step();
      var count = positions.length;
      stillTicks = Math.abs(positions[count - 1] - positions[count - 2]) < 1e-12 ? stillTicks + 1 : 0;
    }
    check(peakSecondDifference(positions) <= 0.4 * 1.05,
      'late jog splice recovery stays within the limit (peak ${peakSecondDifference(positions)})');
    var rested = false;
    var movedAfterRest = false;
    for (index in 49...(positions.length - 3)) {
      var moving = Math.abs(positions[index] - positions[index - 1]) >= 1e-12;
      if (!moving) rested = true;
      else if (rested) movedAfterRest = true;
    }
    check(rested && movedAfterRest, "late jog splice stops, then starts the new jog from rest");
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
      {label: "blended corner", distance: cornerDistance, begin: machine -> machine.movePath(
        GeometricPath.lines([new PathPoint(0.0, 0.0, 0.0), new PathPoint(0.05, 0.0, 0.0),
          new PathPoint(0.05, 0.05, 0.0)]), PathPlanningOptions.blend(0.001),
        new MotionOptions(0.05, limit))}
    ];
    for (queueSupport in [true, false]) {
      var mode = queueSupport ? "buffered" : "position-target";
      for (entry in cases) {
        // Blend mode changes velocity at a corner by design, so compare with
        // the uninterrupted move rather than the bare limit.
        var baseline = rigTrial(gantryRig(queueSupport), 0, _ -> {}, false, entry.begin);
        var allowed = [for (joint in 0...2) Math.max(limit, jointPeak(baseline, joint)) * 1.05];
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
    for (queueSupport in [true, false]) {
      var mode = queueSupport ? "buffered" : "position-target";
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
                // On the buffered path, target batches are only the flush at
                // rest, which holds the measured pose.
                case JointTargets(_, _) if (queueSupport):
                case JointTargets(targets, _):
                  var byJoint = [0.0, 0.0];
                  for (target in targets) byJoint[target.joint] = target.target;
                  worstSkew = Math.max(worstSkew, Math.abs(byJoint[1] + 2.0 * byJoint[0]));
                case TrajectoryChunk(chunk):
                  for (point in chunk.points)
                    worstSkew = Math.max(worstSkew,
                      Math.abs(point.positions[1] + 2.0 * point.positions[0]));
              }
          check(worstSkew <= 1e-12, '$context commands the motors coordinated (skew $worstSkew)');
        }
        eventTick += 6;
      }
    }
  }

  /** Unwraps a move that started at once because the machine was at rest. */
  static function planned(value:Null<JointTrajectory>):JointTrajectory {
    if (value == null) throw "Move was deferred behind a stop but was expected to start at once";
    return cast value;
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

/** Robot wrapper that can hold submitted commands back, to simulate transport delay. */
private class LaggingRobot implements Robot {
  public var lagging:Bool = false;
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
