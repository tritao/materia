import haxe.Int64;
import machinekit.assembly.LinearAxis;
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
import motionkit.robot.MachineKitRobotCompiler;
import motionkit.robot.MotionSystem;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.path.ArcSegment;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.planner.LineLookaheadPlanner;
import motionkit.planner.PathPlanningOptions;
import motionkit.trajectory.MotionLimits;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.PlanLimitError;
import motionkit.trajectory.ValidationLimits;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Actuator;
import robotkit.model.Transmission;
import robotkit.manipulation.ChainTip;
import robotkit.manipulation.KinematicChain;
import robotkit.runtime.Simulation;
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
import robotkit.world.TrajectorySegment;

class MotionKitBootstrapTests {
  static var assertions:Int = 0;

  public static function main():Void {
    testMotionEventContracts();
    testGeometricPathPrimitives();
    testNativeTrajectoryRoundTrip();
    testNativeValidationAndPlan();
    testPlannerIsDeterministicAndBounded();
    testLineLookaheadPlanner();
    testLinearAxisCompilesToRobotModel();
    testLeadScrewActuatorRateLimitsPlans();
    testTransmissionDerivedAxisMapping();
    testCompiledAxisRunsThroughSimulation();
    testHomingAndJogging();
    testMoveLinearUsesPlannerLimits();
    testCompiledXYZGantryRunsThroughSimulation();
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
    testDualMotorAxisChangesStayWithinJointLimits();
    Sys.println('MotionKit bootstrap tests passed ($assertions assertions)');
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
    var plan = ExecutionPlan.create(trajectory, limits, Int64.ofInt(44), [0.0],
      [0.0], [0.0], [0.01], [0.01], [0.01]);
    check(!plan.report.hasFailure(), "valid plan has no failed check");
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

  static function testLineLookaheadPlanner():Void {
    var path = GeometricPath.lines([new PathPoint(0.0, 0.0, 0.0),
      new PathPoint(0.1, 0.0, 0.0), new PathPoint(0.1, 0.1, 0.0)]);
    var planner = new LineLookaheadPlanner(0.01);
    var limits = new MotionLimits(1.0, 2.0, 0.0);
    var exact = planner.planNativePath(path, limits, PathPlanningOptions.exactStopMode());
    var blend = planner.planNativePath(path, limits, PathPlanningOptions.blend(0.01));
    check(exact.durationSeconds() > blend.durationSeconds(),
      "blending shortens a cornered path without changing its endpoints");
    for (segment in blend.segments())
      check(segment.coefficients[0].length == 2,
        "native lookahead emits degree-1 segments");
    for (trajectory in [exact, blend]) {
      for (i in 0...101) {
        var point = trajectory.evaluate(trajectory.durationSeconds() * i / 100.0).positions;
        var onFirst = Math.abs(point[1]) <= 1e-7 &&
          point[0] >= -1e-7 && point[0] <= 0.1000001;
        var onSecond = Math.abs(point[0] - 0.1) <= 1e-7 &&
          point[1] >= -1e-7 && point[1] <= 0.1000001;
        check(onFirst || onSecond, "lookahead stays on the authored polyline");
      }
    }
    var repeated = planner.planNativePath(path, limits, PathPlanningOptions.blend(0.01));
    near(repeated.durationSeconds(), blend.durationSeconds(),
      "lookahead duration is deterministic");
    for (i in 0...101) {
      var time = blend.durationSeconds() * i / 100.0;
      for (joint in 0...3)
        near(repeated.evaluate(time).positions[joint], blend.evaluate(time).positions[joint],
          "lookahead position is deterministic");
    }

    var shallowAngle = Math.PI / 18.0;
    var nearReversalAngle = Math.PI * 170.0 / 180.0;
    function cornerPath(angle:Float):GeometricPath return GeometricPath.lines([
      new PathPoint(0.0, 0.0, 0.0), new PathPoint(0.1, 0.0, 0.0),
      new PathPoint(0.1 + 0.1 * Math.cos(angle), 0.1 * Math.sin(angle), 0.0)]);
    var shallow = planner.planNativePath(cornerPath(shallowAngle), limits,
      PathPlanningOptions.blend(0.01));
    var reversal = planner.planNativePath(cornerPath(nearReversalAngle), limits,
      PathPlanningOptions.blend(0.01));
    var shallowSpeed = cornerChordSpeed(shallow, 0.1, 0.0);
    var reversalSpeed = cornerChordSpeed(reversal, 0.1, 0.0);
    check(shallowSpeed > 0.5, "shallow bend retains high blend speed");
    check(reversalSpeed < 0.2, "near-reversal bend slows for the corner");
    check(shallowSpeed > reversalSpeed * 4.0,
      "corner speed decreases as the interior angle closes");

    var arc = new ArcSegment(new PathPoint(0.1, 0.1, 0.0), 0.1,
      -Math.PI * 0.5, Math.PI * 0.5);
    var arcTrajectory = planner.planNativePath(new GeometricPath([arc]),
      new MotionLimits(0.5, 1.0), PathPlanningOptions.exactStopMode());
    for (segment in arcTrajectory.segments()) {
      var point = segment.coefficients;
      var dx = point[0][0] - 0.1;
      var dy = point[1][0] - 0.1;
      near(Math.sqrt(dx * dx + dy * dy), 0.1,
        "arc knots stay on the authored circle", 1e-5);
    }
    near(arcTrajectory.evaluate(arcTrajectory.durationSeconds()).positions[0], 0.2,
      "arc planner reaches its endpoint");
    for (trajectory in [exact, blend, repeated, shallow, reversal, arcTrajectory])
      trajectory.dispose();
  }

  static function cornerChordSpeed(trajectory:Trajectory, x:Float, y:Float):Float {
    for (segment in trajectory.segments())
      if (Math.abs(segment.coefficients[0][0] - x) <= 1e-7 &&
          Math.abs(segment.coefficients[1][0] - y) <= 1e-7)
        return Math.sqrt(segment.coefficients[0][1] * segment.coefficients[0][1] +
          segment.coefficients[1][1] * segment.coefficients[1][1]);
    throw "lookahead trajectory did not emit its corner knot";
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
    var cornerMove = machine.movePath(cornerPath, PathPlanningOptions.blend(0.001),
      new MotionOptions(0.05, 0.2));
    check(cornerMove.segments().length > 1, "MotionSystem exposes buffered line-path planning");
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
    var segments = value.segments();
    var peak = 0.0;
    for (index in 1...segments.length) {
      var before = segments[index - 1];
      var after = segments[index];
      var seconds = 0.5 * (Int64.toFloat(before.durationNs) +
        Int64.toFloat(after.durationNs)) * 1e-9;
      if (seconds > 0.0)
        peak = Math.max(peak, Math.abs(after.coefficients[joint][1] -
          before.coefficients[joint][1]) / seconds);
    }
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
