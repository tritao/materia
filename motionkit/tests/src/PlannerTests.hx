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
import motionkit.robot.EaikKinematics;
import motionkit.robot.AxisKinematics;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
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
    // Unit speed over two one-second stages, so distance equals time; times outside the law clamp to its ends.
    var distances = law.timesToDistances([0.0, 0.5, 1.0, 1.5, 2.0, 3.0, -1.0]);
    var expected = [0.0, 0.5, 1.0, 1.5, 2.0, 2.0, 0.0];
    check(distances.length == expected.length, "time-to-distance returns one distance per time");
    for (index in 0...expected.length)
      near(distances[index], expected[index], "time-to-distance follows the stages", 1e-12);
    check(law.timesToDistances([]).length == 0, "time-to-distance accepts an empty batch");
    throws(function() law.timesToDistances([0.5, Math.NaN]), "time-to-distance rejects non-finite times");
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
    // The direct inverse must agree with the forward map at every sample: time -> distance -> time.
    check(timed.hasDirectInverse(), "TOPP-RA timing offers the closed-form time-to-distance map");
    var duration = timed.trajectory.durationSeconds();
    var times = [for (sample in 0...401) duration * sample / 400];
    var mapped = timed.timesToDistances(times);
    var previousDistance = -1.0;
    for (index in 0...times.length) {
      check(mapped[index] >= previousDistance, "time-to-distance is monotonic");
      previousDistance = mapped[index];
      near(timed.distanceToTime(mapped[index]), times[index],
        "time-to-distance inverts distance-to-time", 1e-6);
    }
    near(mapped[0], 0.0, "time-to-distance starts at the path start", 1e-12);
    near(mapped[400], 1.0, "time-to-distance ends at the path end", 1e-6);
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
    check(!timed.hasDirectInverse(), "simple path timing has no closed-form inverse, so callers search");
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

  /**
    Queries along a long path cost what a short one does: distance to a path through its grid, the primitive at a
    distance by bisection, and a smooth path longer than a section is cut (and rests) at a primitive boundary.
  **/
  public function testLongPathQueries():Void {
    var seed = 20241009;
    function random():Float {
      seed = (seed * 1103515245 + 12345) & 0x7fffffff;
      return seed / 2147483648.0;
    }
    var primitives:Array<motionkit.path.PathPrimitive> = [];
    var previous = new PathPoint(0.0, 0.0, 0.0);
    for (_ in 0...300) {
      var next = new PathPoint(random() * 0.05, random() * 0.05, random() * 0.002);
      primitives.push(new LineSegment(previous, next));
      previous = next;
    }
    var arc = new ArcSegment(new PathPoint(0.06, 0.025, 0.001), 0.01, 0.0, Math.PI);
    primitives.push(arc);
    var path = new GeometricPath(primitives);
    function brute(point:PathPoint):Float {
      var best = Math.POSITIVE_INFINITY;
      for (primitive in primitives) {
        if (Std.isOfType(primitive, LineSegment)) {
          var line:LineSegment = cast primitive;
          var dx = line.end.x - line.start.x, dy = line.end.y - line.start.y, dz = line.end.z - line.start.z;
          var length2 = dx * dx + dy * dy + dz * dz;
          var t = length2 <= 0.0 ? 0.0 : Math.max(0.0, Math.min(1.0, ((point.x - line.start.x) * dx + (point.y - line.start.y) * dy + (point.z - line.start.z) * dz) / length2));
          best = Math.min(best, point.distanceTo(new PathPoint(line.start.x + t * dx, line.start.y + t * dy, line.start.z + t * dz)));
        } else for (step in 0...4001)
          best = Math.min(best, point.distanceTo(primitive.pointAt(primitive.length() * step / 4000)));
      }
      return best;
    }
    var worst = 0.0;
    for (_ in 0...300) {
      var point = new PathPoint(random() * 0.09 - 0.01, random() * 0.07 - 0.01, random() * 0.006 - 0.002);
      worst = Math.max(worst, Math.abs(path.distanceTo(point) - brute(point)));
    }
    var far = new PathPoint(1.0, 1.0, 1.0);
    worst = Math.max(worst, Math.abs(path.distanceTo(far) - brute(far)));
    check(worst < 2e-6, 'the grid finds the nearest point of a long path like a scan of every primitive does: $worst');

    // A machine's full-travel rapid on all three axes (the bench mill leaving its load pose) and a large arc: the grid
    // files them by the cells along them, not by the millions of cells of their bounding boxes.
    var rapid = new LineSegment(new PathPoint(0.0, 0.0, -0.25), new PathPoint(0.25, 0.15, 0.0));
    var wide = new ArcSegment(new PathPoint(0.125, 0.075, -0.1), 0.2, 0.0, 1.5 * Math.PI);
    var long:Array<motionkit.path.PathPrimitive> = [rapid, wide];
    var longPath = new GeometricPath(long);
    var longWorst = 0.0;
    for (_ in 0...200) {
      var point = new PathPoint(random() * 0.6 - 0.2, random() * 0.6 - 0.2, random() * 0.3 - 0.3);
      var dx = rapid.end.x - rapid.start.x, dy = rapid.end.y - rapid.start.y, dz = rapid.end.z - rapid.start.z;
      var t = Math.max(0.0, Math.min(1.0, ((point.x - rapid.start.x) * dx + (point.y - rapid.start.y) * dy + (point.z - rapid.start.z) * dz) / (dx * dx + dy * dy + dz * dz)));
      var scan = point.distanceTo(new PathPoint(rapid.start.x + t * dx, rapid.start.y + t * dy, rapid.start.z + t * dz));
      // Samples 9.4 um apart along the arc: the scan is at most 4.7 um long of the exact distance.
      for (step in 0...100001) scan = Math.min(scan, point.distanceTo(wide.pointAt(wide.length() * step / 100000)));
      longWorst = Math.max(longWorst, Math.abs(longPath.distanceTo(point) - scan));
    }
    check(longWorst < 5e-6, 'a full-travel three-axis rapid and a 200 mm arc are indexed and measured like a scan: $longWorst');

    var feed = 0.05;
    function waypoint(x:Float):PoseWaypoint return new PoseWaypoint(new Pose3(x, 0.0, 0.0), 1e-6, 1e-3);
    var lines:Array<motionkit.path.PosePrimitive> = [for (i in 0...8010)
      new PoseLine(waypoint(i * 0.001), waypoint((i + 1) * 0.001), OrientationPolicy.Fixed, 0.1, feed)];
    var pose = new PosePath("work", lines);
    check(pose.indexAt(0.0) == 0 && pose.indexAt(0.001) == 0 && pose.indexAfter(0.001) == 1 && pose.indexAt(0.0015) == 1,
      "a distance on a boundary belongs to the primitive that ends there, or the one that starts there");
    check(pose.indexAt(pose.length()) == lines.length - 1 && pose.indexAfter(pose.length()) == lines.length - 1,
      "the end of the path is in its last primitive");
    near(pose.startOf(2500), 2.5, "primitives start where the ones before them end", 1e-9);
    near(pose.waypointAt(2.5004).pose.x, 2.5004, "a pose along a long path", 1e-9);
    var sections = ProgramCompiler.timingSections(pose);
    check(sections.length == 1 && sections[0].path.primitives.length == lines.length && sections[0].offset == 0.0,
      'a straight path of ${lines.length} primitives is one section: a path rests where it turns, not where it is long');
    var turning:Array<motionkit.path.PosePrimitive> = lines.slice(0, 5000);
    turning.push(new PoseLine(waypoint(5.0), new PoseWaypoint(new Pose3(5.0, 0.001, 0.0), 1e-6, 1e-3), OrientationPolicy.Fixed, 0.1, feed));
    var corner = ProgramCompiler.timingSections(new PosePath("work", turning));
    check(corner.length == 2 && corner[0].path.primitives.length == 5000, "a turn still ends a section");
    near(corner[1].offset, 5.0, "each section knows where it starts", 1e-9);
  }

  /**
    A distance-driven joint (an extruder): continuous rate with ramps, the nominal feed conserved exactly over every
    run, joint position the integral of the rate, slices that start and end at rest.
  **/
  public function testDistanceDrive():Void {
    var drive = new motionkit.path.DistanceDrive(7, [{start: 0.01, end: 0.05, rate: 2.0}, {start: 0.05, end: 0.08, rate: 3.0}, {start: 0.09, end: 0.095, rate: 1.0}], 0.1, 0.002);
    near(drive.total(), 2.0 * 0.04 + 3.0 * 0.03 + 1.0 * 0.005, "the position change is the nominal feed, whatever the ramps", 1e-12);
    near(drive.offsetAt(0.1), drive.total(), "the offset at the end is the total", 1e-12);
    near(drive.offsetAt(0.01), 0.0, "nothing is fed before the first run", 1e-15);
    near(drive.rateAt(0.0), 0.0, "the plan starts at rest", 1e-15);
    near(drive.rateAt(0.1), 0.0, "and ends at rest", 1e-15);
    near(drive.rateAt(0.01), 0.0, "a run begins at rest", 1e-15);
    near(drive.rateAt(0.01 + 0.002), 3.0, "its start-up overshoots by half the rate over one ramp", 1e-12);
    near(drive.rateAt(0.01 + 0.004), 2.0, "and settles to the rate over two", 1e-12);
    // The rate is continuous: no step anywhere, so the joint's velocity needs no jump.
    var previous = drive.rateAt(0.0), worstStep = 0.0, integral = 0.0, spacing = 1e-6;
    for (i in 1...100001) {
      var at = i * spacing, rate = drive.rateAt(at);
      worstStep = Math.max(worstStep, Math.abs(rate - previous));
      integral += 0.5 * (rate + previous) * spacing;
      previous = rate;
    }
    check(worstStep < 3.0 * spacing / 0.001 * 1.01, 'the rate changes smoothly: $worstStep');
    near(integral, drive.total(), "the rate integrates to the position change", 1e-6);
    // A joint's second derivative is the rate's slope, with the corners' two sides apart.
    check(Math.abs(drive.curvatureAt(0.01 + 0.002, true) - drive.curvatureAt(0.01 + 0.002, false)) > 100.0,
      "the rate's slope jumps at a ramp's corner, and the two sides say so");
    var stations = drive.stations();
    check(stations.length > 10 && stations[0] == 0.0 && stations[stations.length - 1] == 0.1, "the path is sampled at the ramp corners");
    // A cut path: each piece starts and ends at rest and keeps its own share of the feed.
    var left = drive.slice(0.0, 0.03), right = drive.slice(0.03, 0.1);
    if (left == null || right == null) throw "a cut of the drive lost a piece";
    near(left.total() + right.total(), drive.total(), "the pieces of a cut path feed what the whole did", 1e-12);
    near(left.rateAt(left.pathLength), 0.0, "the first piece ends at rest", 1e-15);
    near(right.rateAt(0.0), 0.0, "the second begins at rest", 1e-15);
    check(drive.slice(0.095, 0.1) == null, "a stretch that feeds nothing has no drive");
    // A run too short for its ramps gets shorter ramps and feeds the same.
    var short = new motionkit.path.DistanceDrive(0, [{start: 0.0, end: 0.001, rate: 5.0}], 0.001, 0.002);
    near(short.total(), 0.005, "a short run feeds its nominal amount", 1e-12);
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
    near(report.checks[TrajectoryCoreConstants.MK_CHECK_VELOCITY].value, 1.0,
      "validation records chord speed");
    near(report.checks[TrajectoryCoreConstants.MK_CHECK_VELOCITY].margin, -0.2,
      "validation reports signed limit margin");
    near(report.checks[TrajectoryCoreConstants.MK_CHECK_VELOCITY].tolerance, 0.8e-9,
      "validation reports comparison tolerance", 1e-12);
    check(report.checks[TrajectoryCoreConstants.MK_CHECK_JERK].status ==
      TrajectoryCoreConstants.MK_CHECK_UNCHECKED, "unclaimed jerk is unchecked");
    check(report.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].status ==
      TrajectoryCoreConstants.MK_CHECK_UNCHECKED, "task-space slot is reserved");
    check(Int64.compare(
      report.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].resolutionNs,
      Int64.ofInt(0)) == 0, "unset task-space check has no sampling resolution");
    report.setTaskSpace(TrajectoryCoreConstants.MK_CHECK_FAILED, 0.006, 0.75,
      0.005, Int64.ofInt(1000000));
    var taskSpace = report.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE];
    check(taskSpace.status == TrajectoryCoreConstants.MK_CHECK_FAILED,
      "Haxe wrapper records task-space status");
    check(taskSpace.method == TrajectoryCoreConstants.MK_CHECK_METHOD_SAMPLED,
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

  /**
    A path lowered to hundreds of short segments a second is merged into longer ones where a quintic follows them: a
    ramp of constant-acceleration pieces becomes a few segments that keep every position, speed and acceleration of
    the original; a zigzag that no quintic follows is left as it was; chords are never merged.
  **/
  public function testMergedSegmentsFollowTheOriginals():Void {
    var pieces = 400, dt = 0.0001;
    var segments = [];
    var position = 0.0, speed = 0.0;
    for (i in 0...pieces) {
      var acceleration = i < pieces / 2 ? 10.0 : -10.0;
      segments.push({timeFromStartNs: Int64.ofInt(i * 100000), durationNs: Int64.ofInt(100000),
        coefficients: [[position, speed, 0.5 * acceleration], [2.0 * position, 2.0 * speed, acceleration]]});
      position += speed * dt + 0.5 * acceleration * dt * dt;
      speed += acceleration * dt;
    }
    var ramp = motionkit.trajectory.Trajectory.fromSegments(segments);
    var merged = ramp.mergeSegments([1e-6, 2e-6], [0.01, 0.02], [1.0, 2.0], [100.0, 200.0], [1000.0, 2000.0], [-1e9, -1e9], [1e9, 1e9], 0.01);
    check(merged.segments().length <= 8, 'the ramp merges to a few segments, got ${merged.segments().length} of $pieces');
    check(merged.durationSeconds() == ramp.durationSeconds(), "and keeps its duration");
    var worst = 0.0, worstSpeed = 0.0, worstAcceleration = 0.0;
    for (step in 0...2000) {
      var t = ramp.durationSeconds() * (step + 0.5) / 2000;
      var a = ramp.evaluate(t), b = merged.evaluate(t);
      for (joint in 0...2) {
        worst = Math.max(worst, Math.abs(a.positions[joint] - b.positions[joint]));
        worstSpeed = Math.max(worstSpeed, Math.abs(a.velocities[joint] - b.velocities[joint]));
        worstAcceleration = Math.max(worstAcceleration, Math.abs(a.accelerations[joint] - b.accelerations[joint]));
      }
    }
    check(worst <= 2e-6 && worstSpeed <= 0.02 && worstAcceleration <= 2.0,
      'the merged ramp follows the original: $worst m, $worstSpeed m/s, $worstAcceleration m/s^2');
    // A limit the original keeps to is kept: the ramp accelerates at exactly its acceleration limits (10 and 20), and no
    // merged quintic may overshoot them where it turns from speeding up to slowing down.
    var limited = ramp.mergeSegments([1e-6, 2e-6], [0.01, 0.02], [1.0, 2.0], [100.0, 200.0], [10.0, 20.0], [-1e9, -1e9], [1e9, 1e9], 0.01);
    var peak = [0.0, 0.0];
    for (step in 0...4000) {
      var state = limited.evaluate(limited.durationSeconds() * (step + 0.5) / 4000);
      for (joint in 0...2) peak[joint] = Math.max(peak[joint], Math.abs(state.accelerations[joint]));
    }
    check(peak[0] <= 10.0 * (1 + 1e-9) && peak[1] <= 20.0 * (1 + 1e-9) && limited.segments().length < pieces,
      'a merge keeps to the limits the pieces keep to: peaks ${peak[0]} and ${peak[1]} in ${limited.segments().length} segments');
    // A zigzag of 0.1 ms pieces turns more often than a quintic can follow.
    var zigzag = [for (i in 0...100) {timeFromStartNs: Int64.ofInt(i * 100000), durationNs: Int64.ofInt(100000),
      coefficients: [[i % 2 == 0 ? 0.0 : 0.0005, i % 2 == 0 ? 5.0 : -5.0, 0.0], [0.0, 0.0, 0.0]]}];
    var saw = motionkit.trajectory.Trajectory.fromSegments(zigzag);
    var kept = saw.mergeSegments([1e-6, 1e-6], [0.01, 0.01], [1.0, 1.0], [100.0, 100.0], [1000.0, 1000.0], [-1e9, -1e9], [1e9, 1e9], 0.01);
    check(kept.segments().length > 40, 'a zigzag stays mostly as it was, ${kept.segments().length} of 100 segments');
    ramp.dispose(); merged.dispose(); limited.dispose(); saw.dispose(); kept.dispose();
  }


  /**
    Input shaping: the shaped motion is the weighted sum of the original delayed by each impulse, held at rest beyond its
    ends, so it ends the longest delay later at the same place, never goes faster or harder than the original, and a joint
    given only a delay is the original that much later. The shapers' impulses add up to one and cancel their own mode.
  **/
  public function testShapedTrajectoryIsTheWeightedSumOfDelays():Void {
    // A rest-to-rest move of 50 ms accelerating, 100 ms cruising, 50 ms decelerating at 8 m/s^2 (joint 0) and its twin (joint 1).
    var a = 8.0, t1 = 0.05, t2 = 0.1;
    var v = a * t1, d1 = 0.5 * a * t1 * t1, d2 = v * t2;
    var segments = [
      {timeFromStartNs: Int64.ofInt(0), durationNs: Int64.ofInt(50000000), coefficients: [[0.0, 0.0, 0.5 * a], [0.0, 0.0, a]]},
      {timeFromStartNs: Int64.ofInt(50000000), durationNs: Int64.ofInt(100000000), coefficients: [[d1, v, 0.0], [2.0 * d1, 2.0 * v, 0.0]]},
      {timeFromStartNs: Int64.ofInt(150000000), durationNs: Int64.ofInt(50000000), coefficients: [[d1 + d2, v, -0.5 * a], [2.0 * (d1 + d2), 2.0 * v, -a]]}];
    var original = motionkit.trajectory.Trajectory.fromSegments(segments);
    var shaper = new motionkit.trajectory.InputShaper(motionkit.trajectory.InputShaper.InputShaperKind.MZV, 50.0, 0.1);
    var impulses = shaper.impulses;
    var delayOnly = motionkit.trajectory.ShaperImpulses.delay(impulses.meanDelay());
    var shaped = original.shape([impulses.delays, delayOnly.delays], [impulses.amplitudes, delayOnly.amplitudes]);
    near(shaped.durationSeconds(), original.durationSeconds() + impulses.tail(), "shaping ends the shaper's tail later", 2e-9);
    function held(trajectory:motionkit.trajectory.Trajectory, joint:Int, time:Float):Float
      return trajectory.evaluate(Math.min(Math.max(time, 0.0), trajectory.durationSeconds())).positions[joint];
    var worst = 0.0, fastest = 0.0, hardest = 0.0, original_fastest = 0.0, original_hardest = 0.0;
    for (step in 0...2000) {
      var t = shaped.durationSeconds() * (step + 0.5) / 2000;
      var sum = 0.0;
      for (i in 0...impulses.delays.length) sum += impulses.amplitudes[i] * held(original, 0, t - impulses.delays[i]);
      var state = shaped.evaluate(t);
      worst = Math.max(worst, Math.abs(state.positions[0] - sum));
      worst = Math.max(worst, Math.abs(state.positions[1] - held(original, 1, t - delayOnly.delays[0])));
      fastest = Math.max(fastest, Math.abs(state.velocities[0])); hardest = Math.max(hardest, Math.abs(state.accelerations[0]));
      var before = original.evaluate(Math.min(t, original.durationSeconds()));
      original_fastest = Math.max(original_fastest, Math.abs(before.velocities[0])); original_hardest = Math.max(original_hardest, Math.abs(before.accelerations[0]));
    }
    check(worst < 1e-9, 'the shaped joints are the weighted sums of the delayed originals, off by $worst m');
    check(fastest <= original_fastest * (1.0 + 1e-9) && hardest <= original_hardest * (1.0 + 1e-9),
      'shaping raises no velocity or acceleration: $fastest of $original_fastest m/s, $hardest of $original_hardest m/s^2');
    var end = shaped.evaluate(shaped.durationSeconds());
    near(end.positions[0], d1 * 2.0 + d2, "the shaped move ends where the original does", 1e-9);
    near(end.velocities[0], 0.0, "at rest", 1e-9);
    near(shaped.evaluate(0.0).positions[0], 0.0, "and starts there", 1e-12);
    // Rejected: amplitudes that do not add up to one, a negative delay, no impulse, too many impulses.
    function rejects(delays:Array<Float>, amplitudes:Array<Float>):Bool {
      try { new motionkit.trajectory.ShaperImpulses(delays, amplitudes); return false; } catch (_:Dynamic) return true;
    }
    check(rejects([0.0, 0.01], [0.5, -0.5]) && rejects([-0.01], [1.0]) && rejects([], []) && rejects([for (i in 0...9) i * 0.001], [for (i in 0...9) 1.0]),
      "impulses with a non-positive amplitude, a negative delay, none or nine are refused");
    var refused = false;
    try original.shape([impulses.delays], [impulses.amplitudes]) catch (_:Dynamic) refused = true;
    check(refused, "shaping needs impulses for every joint");
    // The shapers' impulses add up to one and cancel their mode (ZV exactly, MZV to 3 % at a damping of 0.1, EI to 5 %), and MZV stays low in a band around it.
    for (kind in [motionkit.trajectory.InputShaper.InputShaperKind.ZV, motionkit.trajectory.InputShaper.InputShaperKind.MZV, motionkit.trajectory.InputShaper.InputShaperKind.EI]) {
      var each = new motionkit.trajectory.InputShaper(kind, 50.0, 0.1);
      var total = 0.0;
      for (amplitude in each.impulses.amplitudes) total += amplitude;
      near(total, 1.0, 'the $kind amplitudes add up to one', 1e-12);
      var residual = each.impulses.residual(50.0, 0.1);
      check(residual <= (kind == motionkit.trajectory.InputShaper.InputShaperKind.EI ? 0.0501 : kind == motionkit.trajectory.InputShaper.InputShaperKind.MZV ? 0.03 : 1e-9),
        'the $kind shaper leaves $residual of its own mode');
      if (kind == motionkit.trajectory.InputShaper.InputShaperKind.MZV)
        check(each.impulses.residual(45.0, 0.1) < 0.1 && each.impulses.residual(55.0, 0.1) < 0.1, "MZV keeps under 10 % for a frequency 10 % off");
    }
    original.dispose(); shaped.dispose();
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
    var blueprint = MachineKitRobotCompiler.compileGantry(
      new machinekit.gantry.Gantry(new machinekit.gantry.GantrySpec(80, 80, 80)),
      0.2, 2.0);
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("linear-planner-limits", runtime,
      blueprint.model.name, [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    homeGantryFixture(blueprint, runtime, simulationHarness, robot, machine);
    var options = new MotionOptions(0.2, 2.0);
    var gentle = planned(machine.moveLinear(Pose.xyz(0.02, 0.0, 0.0),
      Feed.metresPerSecond(0.2), new MotionOptions(0.2, 0.4)));
    var gentleAcceleration = peakChordAcceleration(gentle, 0);
    check(gentleAcceleration <= 0.4 + 1e-6, "moveLinear obeys the lower authored acceleration");
    runMotion(machine, simulationHarness);
    machine.home();
    runMotion(machine, simulationHarness);
    var xOnly = planned(machine.moveLinear(Pose.xyz(0.02, 0.0, 0.0),
      Feed.metresPerSecond(0.2), options));
    var peakAcceleration = peakChordAcceleration(xOnly, 0);
    // At the motor's 43.7 mm/s cap, the retiming grid need not reach 2 m/s².
    // Raising the authored cap must still raise acceleration, within that cap.
    check(peakAcceleration > 2.0 * gentleAcceleration && peakAcceleration <= 2.0 + 1e-6,
      'moveLinear uses its authored acceleration cap ($gentleAcceleration -> $peakAcceleration)');

    runMotion(machine, simulationHarness);

    var diagonal = planned(machine.moveLinear(Pose.xyz(0.03, 0.03, 0.03),
      Feed.metresPerSecond(0.2), options));
    for (joint in 0...3)
      check(peakChordAcceleration(diagonal, joint) <= 2.0 + 1e-6,
        "diagonal moveLinear chords stay within per-axis acceleration caps");
    throws(function() {
      var outside = machine.moveLinear(Pose.xyz(0.080000001, 0.0, 0.0),
        Feed.metresPerSecond(0.2), options);
    }, "path limits reject a 1 nm excursion while accepting homing roundoff");
    simulationHarness.dispose();
  }

  public function testToleranceBlend():Void {
    var tolerance = 0.0005;
    var blueprint = gantryBlueprint();
    function home(id:String):Float {
      for (axis in blueprint.axes) if (axis.id == id) return axis.homePosition;
      throw 'Gantry blueprint has no "$id" axis';
    }
    var origin = new PathPoint(home("x"), home("y"), home("z"));
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
    // The tightest worthwhile arc comes from the machine: (half the speed)^2 over the acceleration it gives.
    near(CornerBlender.minimumRadius(0.1, 12.5), 0.0025 / 12.5, "a corner is worth rounding if the machine keeps half the speed");
    near(CornerBlender.minimumRadius(0.1, 0.0), 0.0, "an unknown acceleration sets no minimum");
    // Lines of 0.1 mm meeting at a right angle can only be rounded by an arc of 45 um, under half of the 121 um that a
    // 50 um tolerance wanted, and tighter than the machine takes at 0.1 m/s (200 um).
    var shortCorner = GeometricPath.lines([new PathPoint(), new PathPoint(0.0001, 0.0), new PathPoint(0.0001, 0.0001)]);
    var untried = CornerBlender.blend(shortCorner, 0.00005, Math.PI * 0.9);
    var judged = CornerBlender.blend(shortCorner, 0.00005, Math.PI * 0.9, CornerBlender.minimumRadius(0.1, 12.5));
    check(untried.path.primitives.length == 3 && untried.diagnostics.length == 0, "without a minimum a short corner is rounded");
    check(judged.path.primitives.length == 2 && judged.diagnostics.length == 1,
      "a corner the machine cannot take at a good speed is stopped at: " + judged.diagnostics);
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
    var mixedAtHome = new GeometricPath([
      new LineSegment(origin, new PathPoint(origin.x + 0.05, origin.y, origin.z)),
      new ArcSegment(new PathPoint(origin.x + 0.07, origin.y, origin.z),
        0.02, Math.PI, -Math.PI * 0.5)
    ]);
    var mixedRig = gantryRig(true);
    var mixedTimed = mixedRig.machine.movePath(mixedAtHome,
      PathPlanningOptions.blend(0.0005), new MotionOptions(0.08, 0.4));
    var mixedReport = mixedRig.machine.lastPathValidationReport;
    if (mixedReport == null) throw "Mixed blend did not record validation";
    check(mixedTimed.durationSeconds() > 0.0 &&
      mixedReport.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE].status ==
        TrajectoryCoreConstants.MK_CHECK_PASSED,
      "mixed blend times and validates through the runtime plan");
    runMotion(mixedRig.machine, mixedRig.harness);
    near(mixedRig.robot.snapshot().positions.get(0), origin.x + 0.07,
      "mixed blend executes to its X endpoint", 1e-5);
    near(mixedRig.robot.snapshot().positions.get(1), origin.y + 0.02,
      "mixed blend executes to its Y endpoint", 1e-5);
    mixedRig.harness.dispose();
    var east = new PathPoint(origin.x + 0.05, origin.y, origin.z);
    var northeast = new PathPoint(origin.x + 0.05, origin.y + 0.05, origin.z);
    var path = GeometricPath.lines([origin, east, northeast]);
    var exactRig = gantryRig(true);
    var exact = exactRig.machine.movePath(path, PathPlanningOptions.exactStopMode(),
      new MotionOptions(0.08, 0.4));
    var blendedRig = gantryRig(true);
    var blended = blendedRig.machine.movePath(path, PathPlanningOptions.blend(0.0005),
      new MotionOptions(0.08, 0.4));
    // A tiny fillet can cost time under conservative junction limits. A
    // larger permitted deviation demonstrates the speed/tolerance tradeoff.
    var widerRig = gantryRig(true);
    var wider = widerRig.machine.movePath(path, PathPlanningOptions.blend(0.002),
      new MotionOptions(0.08, 0.4));
    check(wider.durationSeconds() < exact.durationSeconds(),
      '2 mm fillet ${wider.durationSeconds()} is faster than exact stop ${exact.durationSeconds()}');
    Sys.println('cartesian blends: exact ${exact.durationSeconds()} s, 0.5 mm ${blended.durationSeconds()} s, 2 mm ${wider.durationSeconds()} s');
    widerRig.harness.dispose();
    var closestCorner = 1.0;
    var cornerSpeed = 0.0;
    for (index in 0...501) {
      var state = blended.evaluate(blended.durationSeconds() * index / 500.0);
      var x = state.positions[0], y = state.positions[1];
      var deviation = Math.min(segmentDistance(x, y, origin.x, origin.y, east.x, east.y),
        segmentDistance(x, y, east.x, east.y, northeast.x, northeast.y));
      check(deviation <= 0.0005 + 1e-5,
        "0.5 mm fillet stays within its authored path tolerance");
      var cornerDistance = Math.sqrt((x - east.x) * (x - east.x) +
        (y - east.y) * (y - east.y));
      if (cornerDistance < closestCorner) {
        closestCorner = cornerDistance;
        cornerSpeed = Math.sqrt(state.velocities[0] * state.velocities[0] +
          state.velocities[1] * state.velocities[1]);
      }
    }
    check(cornerSpeed > 1e-3, "0.5 mm fillet carries speed through the corner");
    var blendReport = blendedRig.machine.lastPathValidationReport;
    if (blendReport == null) throw "Blend path did not record validation";
    var taskCheck = blendReport.checks[TrajectoryCoreConstants.MK_CHECK_TASK_SPACE];
    check(taskCheck.status == TrajectoryCoreConstants.MK_CHECK_PASSED &&
      Math.abs(taskCheck.limit - 0.0005) < 1e-12,
      "blend validation checks the authored 0.5 mm tolerance");
    runMotion(blendedRig.machine, blendedRig.harness);
    near(blendedRig.robot.snapshot().positions.get(0), northeast.x,
      "blended path executes to its X endpoint", 1e-5);
    near(blendedRig.robot.snapshot().positions.get(1), northeast.y,
      "blended path executes to its Y endpoint", 1e-5);
    exactRig.harness.dispose();
    blendedRig.harness.dispose();

    var reverseCorner = new PathPoint(origin.x + 0.05, origin.y, origin.z);
    var reverse = GeometricPath.lines([origin, reverseCorner,
      new PathPoint(origin.x + 0.05 + 0.05 * Math.cos(Math.PI * 170.0 / 180.0),
        origin.y + 0.05 * Math.sin(Math.PI * 170.0 / 180.0), origin.z)]);
    var reverseRig = gantryRig(true);
    var fallback = reverseRig.machine.movePath(reverse, PathPlanningOptions.blend(0.0005),
      new MotionOptions(0.08, 0.4));
    check(reverseRig.machine.lastPathPlanningDiagnostics.length > 0 &&
      reverseRig.machine.lastPathPlanningDiagnostics[0].indexOf("turn angle") >= 0,
      "near reversal reports an exact-stop fallback");
    var stoppedAtCorner = false;
    for (segment in fallback.segments()) {
      var state = fallback.evaluate(Int64.toFloat(segment.timeFromStartNs) * 1e-9);
      if (Math.abs(state.positions[0] - reverseCorner.x) < 1e-8 &&
          Math.abs(state.positions[1] - reverseCorner.y) < 1e-8 &&
          Math.abs(state.velocities[0]) < 1e-7 && Math.abs(state.velocities[1]) < 1e-7)
        stoppedAtCorner = true;
    }
    check(stoppedAtCorner, "near reversal stops at the authored corner");
    reverseRig.harness.dispose();
  }

  public function testCircularSegments():Void {
    var cases = [CircularPlane.XY, CircularPlane.XZ, CircularPlane.YZ];
    for (plane in cases) {
      var rig = gantryRig(true);
      var measured = rig.robot.snapshot().positions.toArray();
      var xAxis = rig.machine.axis("x"), yAxis = rig.machine.axis("y"), zAxis = rig.machine.axis("z");
      if (xAxis == null) throw "Homed gantry has no X axis";
      if (yAxis == null) throw "Homed gantry has no Y axis";
      if (zAxis == null) throw "Homed gantry has no Z axis";
      var origin = new PathPoint(xAxis.logicalPosition(measured), yAxis.logicalPosition(measured),
        zAxis.logicalPosition(measured));
      var center = switch plane {
        case XY: new PathPoint(origin.x + 0.02, origin.y, origin.z);
        case XZ: new PathPoint(origin.x + 0.02, origin.y, origin.z);
        case YZ: new PathPoint(origin.x, origin.y + 0.02, origin.z);
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
      var blueprint = MachineKitRobotCompiler.compileGantry(
      new machinekit.gantry.Gantry(new machinekit.gantry.GantrySpec(200, 200, 200)),
      0.1, 0.4);
      var solver = new AxisKinematics(blueprint, "x", "y", "z");
      var count = solver.jointCount();
      var limits = new ValidationLimits(count,
        Int64.ofInt(blueprint.runtime.revision),
        Int64.ofInt(blueprint.runtime.calibrationRevision));
      var ids = [for (joint in blueprint.model.joints) joint.id];
      var scales = [for (_ in ids) 0.0];
      for (axis in blueprint.axes) for (slot in 0...axis.jointIds.length)
        scales[ids.indexOf(axis.jointIds[slot])] = Math.abs(axis.jointScales[slot]);
      var velocity = [for (joint in blueprint.model.joints) joint.limits.requireVelocity()];
      var acceleration = [for (joint in blueprint.model.joints) joint.limits.requireAcceleration()];
      var jerk = [for (scale in scales) 10.0 * scale];
      for (joint in 0...count) {
        var bounds = blueprint.model.joints[joint].limits;
        if (bounds.lower < bounds.upper)
          limits.position(joint, bounds.lower, bounds.upper);
        limits.velocity(joint, velocity[joint]);
        limits.acceleration(joint, acceleration[joint]);
        limits.jerk(joint, jerk[joint]);
      }
      var compiler = new ProgramCompiler(solver, limits, "work", velocity,
        acceleration, jerk, new StartTolerances([for (scale in scales) 0.0005 * scale],
          [for (scale in scales) 0.004 * scale], [for (scale in scales) 0.01 * scale]),
        null, 0.01, 0.5, 0.005, 0.02, null, null,
        [for (scale in scales) 0.5 * scale], ids, blueprint.model.couplings);
      var primitive = new TestCircularPosePrimitive(circular, 0.05);
      var path = new PosePath("work", [primitive]).withAuthoredGeometry(authored, 0.001);
      var compiled = compiler.compile(new MotionProgram([
        MotionOp.FollowPath(path, "work", 0.05, [])]),
        [for (_ in blueprint.model.joints) 0.0], Int64.ofInt(901));
      check(compiled.blocks.length == 1, 'helical $plane compiles through TOPP-RA');
      compiled.dispose();
    }
  }


}
