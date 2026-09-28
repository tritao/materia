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

class PlannerTests extends MotionKitTestSupport {
  public function new() { super(); }

  public function testNativePathLowering():Void {
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

  public function testToppraPathTiming():Void {
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

  public function testSimplePathTimingContract():Void {
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

  public function testGeometricPathPrimitives():Void {
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

  public function testNativeTrajectoryRoundTrip():Void {
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

  public function testNativeValidationAndPlan():Void {
    var trajectory = Trajectory.fromPositionSamples([0.0, 1.0], [[0.0], [1.0]]);
    var limits = new ValidationLimits(1, Int64.ofInt(12), Int64.ofInt(3));
    limits.position(0, 0.0, 1.0);
    limits.velocity(0, 0.8);
    var report = trajectory.validate(limits);
    check(report.hasFailure(), "chord speed above claimed limit fails validation");
    var initialGuarantees = report.guarantees();
    check(initialGuarantees.jointPosition == Proven &&
      initialGuarantees.jointVelocity == Failed &&
      initialGuarantees.jerk == Unchecked &&
      initialGuarantees.taskSpace == Unchecked,
      "guarantee summary separates exact, failed, and unchecked checks");
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
    check(report.guarantees().taskSpace == Failed,
      "failed sampled task-space check remains a failure in the summary");
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

  public function testPlannerIsDeterministicAndBounded():Void {
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

  public function testToppraExactStopsAndBindings():Void {
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

  public function testToppraCircleAcceleration():Void {
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

  public function testMoveLinearUsesPlannerLimits():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 80), new LinearAxis(23, 10, 80),
      new LinearAxis(23, 10, 80), 0.2, 2.0);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
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

    runMotion(machine, simulationHarness);

    var diagonal = planned(machine.moveLinear(Pose.xyz(0.03, 0.03, 0.03),
      Feed.metresPerSecond(0.2), options));
    for (joint in 0...3)
      check(peakChordAcceleration(diagonal, joint) <= 2.0 + 1e-6,
        "diagonal moveLinear chords stay within per-axis acceleration caps");
    simulationHarness.dispose();
  }

  public function testToleranceBlend():Void {
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
    runMotion(mixedRig.machine, mixedRig.harness);
    near(mixedRig.robot.snapshot().positions.get(0), 0.07,
      "mixed blend executes to its X endpoint", 1e-5);
    near(mixedRig.robot.snapshot().positions.get(1), 0.02,
      "mixed blend executes to its Y endpoint", 1e-5);
    mixedRig.harness.dispose();
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
    runMotion(blendedRig.machine, blendedRig.harness);
    near(blendedRig.robot.snapshot().positions.get(0), 0.05,
      "blended path executes to its X endpoint", 1e-5);
    near(blendedRig.robot.snapshot().positions.get(1), 0.05,
      "blended path executes to its Y endpoint", 1e-5);
    exactRig.harness.dispose();
    blendedRig.harness.dispose();

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
    reverseRig.harness.dispose();
  }

  public function testCircularSegments():Void {
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
      rig.harness.dispose();
      var blended = CornerBlender.blend(new GeometricPath([circular,
        new LineSegment(circular.end, new PathPoint(circular.end.x + 0.01,
          circular.end.y, circular.end.z))]), 0.001, Math.PI * 0.9);
      check(blended.diagnostics.length == 1 &&
        blended.diagnosticCorners.length == 1 &&
        blended.diagnosticCorners[0] == 1 &&
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
      blendedRig.harness.dispose();
      var blueprint = MachineKitRobotCompiler.compileXYZGantry(
        new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
        new LinearAxis(23, 10, 200), 0.1, 0.4);
      var binding = MotionKitTestSupport.cncBinding(
        new CncMachine("work", "x", "y", "z", 0.08), blueprint);
      var primitive = new toolpathkit.motion.ToolpathPosePrimitive(circular, 0.05, 0.0005, 0.02);
      var path = new PosePath("work", [primitive]).withAuthoredGeometry(authored, 0.001);
      var compiled = binding.compiler.compile(new MotionProgram([
        MotionOp.FollowPath(path, "work", 0.05, [])]),
        [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(901));
      check(compiled.blocks.length == 1, 'helical $plane compiles through TOPP-RA');
      compiled.dispose();
    }
  }


}
