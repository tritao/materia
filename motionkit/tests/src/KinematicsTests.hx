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
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import kinematicskit.LinearAlgebra;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.ManipulatorServo;
import motionkit.robot.ServoSession;
import motionkit.robot.ServoPlan.ServoPlanOptions;
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
import robotkit.world.JointTarget;
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

class KinematicsTests extends MotionKitTestSupport {
  public function new() { super(); }

  public function testPathConfigurationSelector():Void {
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

  /** One group serves several threads at once: each evaluates in data of its own. */
  public function testSharedGroupAcrossThreads():Void {
    var arm = buildContractArmFixture().arm;
    var configurations = [for (k in 0...64) [for (j in 0...6) 0.3 * Math.sin(0.7 * k + j)]];
    var expected = [for (q in configurations) arm.tcpPose(q).translation];
    var mismatches = [0, 0, 0, 0], done = new sys.thread.Lock();
    for (t in 0...4)
      sys.thread.Thread.create(function() {
        for (round in 0...200)
          for (k in 0...configurations.length) {
            var reached = arm.tcpPose(configurations[(k + t * 7) % configurations.length]).translation;
            var wanted = expected[(k + t * 7) % configurations.length];
            if (Math.abs(reached.x - wanted.x) + Math.abs(reached.y - wanted.y) + Math.abs(reached.z - wanted.z) > 1e-12)
              mismatches[t]++;
          }
        done.release();
      });
    for (_ in 0...4) done.wait();
    for (t in 0...4) check(mismatches[t] == 0, 'thread $t evaluates the shared arm as one thread does (${mismatches[t]} off)');
    var own = arm.newData();
    var withOwn = arm.tcpPose(configurations[5], own).translation;
    near(withOwn.x, expected[5].x, "a caller's own data evaluates the same", 1e-12);
  }

  public function testKinematicsContract():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(fixture.arm, 1e-8);
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
    var jacobian = fixture.arm.jacobian(q);
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

  public function testOpwKinematics():Void {
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
    var arm = new Manipulator(model, links[0].id, flange.id);
    var manipulator = arm;
    var solver = new OpwKinematics(model, manipulator);
    near(solver.parameters.a1, 0.1, "OPW extracts a1", 1e-9);
    near(solver.parameters.a2, -0.135, "OPW extracts a2", 1e-9);
    near(solver.parameters.c1, 0.615, "OPW extracts c1", 1e-9);
    // Through the interface: type tests work, and the solver runs its own (analytic) path search.
    {
      var general:KinematicsSolver = solver;
      check(Std.isOfType(general, OpwKinematics) && !Std.isOfType(general, ManipulatorKinematics),
        "an OPW solver is recognised through the KinematicsSolver interface");
      var q0 = [0.2, -0.3, 0.4, 0.5, -0.6, 0.7];
      var path = general.solvePath(new PathRequest([0.0], [general.forward(q0)], q0, new IkTolerance(),
        [for (_ in 0...6) 0.5], [for (_ in 0...6) 1.0]));
      check(path.length == 1, "an OPW solver searches its own paths through the interface");
    }
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
    var chosen = solver.solvePath(new PathRequest([0.0, 0.04], [solver.forward(q), solver.forward(next)], q,
      new IkTolerance(), [for (_ in 0...6) 0.5], [for (_ in 0...6) 1.0]));
    var second:Array<Float> = chosen[1];
    near(second[0], next[0], "Descartes samples OPW branches natively across a path", 1e-6);
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(0));
    var compiler = new ProgramCompiler(solver, limits, "work",
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 4.0],
      [for (_ in 0...6) 20.0], StartTolerances.uniform(6, 0.02, 0.02, 0.02));
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
      var arm = new Manipulator(fixture, parts[0].id, tool.id);
      var robot = arm;
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
    try new OpwKinematics(bad.model, bad.arm)
    catch (error:Dynamic) diagnostic = Std.string(error);
    check(diagnostic.indexOf("joint-3") >= 0,
      "non-spherical UR5 wrist names its violating joint");
  }

  public function testManipulatorServo():Void {
    var fixture = buildContractArmFixture();
    var arm = fixture.arm;
    var servo = new ManipulatorServo(arm, 1e-3);
    var q = [0.3, -0.8, 1.1, -0.5, 0.4, 0.2];
    var twist = new Twist6(0.05, -0.02, 0.03, 0.1, -0.05, 0.08);
    var unlimited = [for (_ in 0...6) Math.POSITIVE_INFINITY];

    // Far from every bound it is the damped least-squares step at the same damping.
    var free = servo.step(q, twist, 0.01, unlimited);
    var expected = LinearAlgebra.dampedStep(arm.tcpJacobian(q), 6, 6, [for (i in 0...6) i],
      [for (value in twist.toArray()) value * 0.01], 1e-3);
    check(!free.fallback && free.limited.length == 0, "an unconstrained servo tick solves on the QP");
    for (joint in 0...6) near(free.velocity[joint], expected[joint] / 0.01,
      "an unconstrained servo tick is the damped least-squares step", 1e-5);

    // Velocity limits hold exactly.
    var fast = new Twist6(2.0, -1.0, 1.5, 3.0, -2.0, 2.5);
    var capped = servo.step(q, fast, 0.01, [for (_ in 0...6) 0.5]);
    for (joint in 0...6) check(Math.abs(capped.velocity[joint]) <= 0.5 + 1e-12, "servo velocities stay within their limits");
    check(capped.limited.length > 0, "a too-fast twist reports the joints held by their velocity limit");

    // Turning the whole arm about the base axis (the tool twist v = ω × p, ω = z) is the base joint's
    // own motion; a steady turn drives it into its +2π stop and holds it there.
    var state = q.copy();
    var crossed = false;
    for (_ in 0...1200) {
      var tcp = arm.tcpPose(state).translation;
      var yaw = new Twist6(-tcp.y, tcp.x, 0.0, 0.0, 0.0, 1.0);
      var tick = servo.step(state, yaw, 0.01, [for (_ in 0...6) 2.0]);
      for (joint in 0...6) {
        state[joint] += tick.velocity[joint] * 0.01;
        var limits = arm.group.limitsOf(joint);
        if (state[joint] > limits.upper + 1e-12 || state[joint] < limits.lower - 1e-12) crossed = true;
      }
    }
    check(!crossed, "integrating servo ticks never leaves the joint range");
    near(state[0], 2.0 * Math.PI, "the base joint ends on its stop", 1e-9);

    // With a limit gain the base slows into its stop instead of arriving in one tick.
    var soft = q.copy();
    for (_ in 0...1200) {
      var tcp = arm.tcpPose(soft).translation;
      var tick = servo.step(soft, new Twist6(-tcp.y, tcp.x, 0.0, 0.0, 0.0, 1.0), 0.01, [for (_ in 0...6) 2.0], 1000, 0.3);
      for (joint in 0...6) soft[joint] += tick.velocity[joint] * 0.01;
    }
    check(soft[0] < 2.0 * Math.PI && 2.0 * Math.PI - soft[0] < 1e-6, "a limit gain slows the base into its stop without touching it");

    // A QP that runs out of iterations falls back to the clamped damped step, still within limits.
    var starved = servo.step(q, fast, 0.01, [for (_ in 0...6) 0.5], 1);
    if (starved.fallback) for (joint in 0...6)
      check(Math.abs(starved.velocity[joint]) <= 0.5 + 1e-12, "the fallback step also respects the limits");
    servo.dispose();
  }

  /**
   * Live servoing through the simulated runtime: a streamed twist is followed within the joints'
   * velocity and acceleration limits, stale and expired commands are rejected, a lost operator brakes the
   * arm to rest within its acceleration limits, and driving into a joint stop ends exactly on it.
   */
  /** Servoing through short plans, on the plan-executing runtime and on the virtual device. */
  public function testServoPlans():Void {
    for (virtual in [false, true]) servoPlanTrial(virtual);
  }

  function servoPlanTrial(virtual:Bool):Void {
    var label = virtual ? "virtual device" : "plan runtime";
    var fixture = buildContractArmFixture();
    for (joint in fixture.model.joints) { joint.limits.velocity = 2.0; joint.limits.maxAcceleration = 4.0; }
    fixture.model.joints[0].limits.upper = 1.2;
    var blueprint = RobotRuntimeCompiler.compile(fixture.model);
    var harness = new SimulationHarness(0.01);
    var options:Null<VirtualDeviceOptions> = null;
    if (virtual) {
      options = new VirtualDeviceOptions();
      // 1e-4 rad steps: up to 2 rad/s fits the default 40 kHz step clock.
      options.stepsPerUnit = [for (_ in 0...6) 10000.0];
    }
    var runtime = harness.simulation.addRobot(blueprint, null, options);
    var tick = 0;
    for (_ in 0...20) harness.step(Int64.ofInt(++tick));
    var robot = new SimulatedRobot('servo-plans-$virtual', runtime, fixture.model.name,
      [for (link in fixture.model.links) link.name], [for (joint in fixture.model.joints) joint.name]);
    var arm = fixture.arm;
    function positions():Array<Float> return [for (i in 0...6) robot.snapshot().positions.get(i)];
    var model = Int64.ofInt(blueprint.revision), calibration = Int64.ofInt(blueprint.calibrationRevision);

    var quantumStart = virtual ? 1e-4 : 1e-9;
    // Reach the start with one rest-to-rest quintic (the virtual device takes plans only).
    var from = positions();
    var start = [0.3, -0.8, 1.1, -0.5, 0.4, 0.2];
    var span = 2.0;
    var coefficients = [for (j in 0...6) {
      var d = start[j] - from[j];
      [from[j], 0.0, 0.0, 10.0 * d / Math.pow(span, 3), -15.0 * d / Math.pow(span, 4), 6.0 * d / Math.pow(span, 5)];
    }];
    var zero = [for (_ in 0...6) 0.0];
    robot.submit(RobotCommand.ExecutionPlan(new ExecutionPlanSubmission(Int64.ofInt(1), model, calibration, 1, from,
      zero, zero, [new TrajectorySegment(Int64.ofInt(0), Int64.fromFloat(span * 1e9), coefficients)])));
    for (_ in 0...260) harness.step(Int64.ofInt(++tick));
    for (j in 0...6) near(positions()[j], start[j], '$label: the arm reaches its start (joint $j)', 2.0 * quantumStart);

    var lead = 0.04;
    var session = new ServoSession(robot, arm, 0.01, 1e-3, 1e-4, new ServoPlanOptions(model, calibration, lead));
    var ms = Int64.ofInt(1000000);
    var accel = 4.0, dt = 0.01;
    // Joint motion is judged from the measured positions: second differences bound the acceleration.
    var quantum = virtual ? 2e-4 : 1e-9;
    var trace:Array<Array<Float>> = [positions()];
    // Source time of each traced sample: a device may report the same sample on two ticks.
    var stamps:Array<Float> = [Int64.toFloat(robot.snapshot().sourceTimestampNs) * 1e-9];
    var sequence = 0;
    function advance():Void {
      harness.step(Int64.ofInt(++tick));
      trace.push(positions());
      stamps.push(Int64.toFloat(robot.snapshot().sourceTimestampNs) * 1e-9);
    }
    function tickOnce(refresh:Null<motionkit.kinematics.Twist6>) {
      if (refresh != null) check(session.command(refresh, ++sequence, session.nowNs() + ms * 100) == null, '$label: a fresh command is accepted');
      var result = session.update();
      advance();
      return result;
    }
    // Second differences over distinct samples, on the samples' own clock.
    function worstAcceleration(from:Int):Float {
      var distinct = [for (i in from...trace.length) if (i == from || stamps[i] > stamps[i - 1]) i];
      var worst = 0.0;
      for (k in 2...distinct.length) {
        var a = distinct[k - 2], b = distinct[k - 1], c = distinct[k];
        var h1 = stamps[b] - stamps[a], h2 = stamps[c] - stamps[b];
        for (j in 0...6)
          worst = Math.max(worst, Math.abs(2.0 * ((trace[c][j] - trace[b][j]) / h2 - (trace[b][j] - trace[a][j]) / h1) /
            (h1 + h2)));
      }
      return worst;
    }
    function worstSpeed(from:Int):Float {
      var worst = 0.0;
      for (i in (from + 1)...trace.length) for (j in 0...6) worst = Math.max(worst, Math.abs(trace[i][j] - trace[i - 1][j]) / dt);
      return worst;
    }
    function still(count:Int):Bool {
      var last = trace.length - 1;
      for (i in (last - count)...last) for (j in 0...6) if (Math.abs(trace[i + 1][j] - trace[i][j]) > 1e-9) return false;
      return true;
    }
    var slack = accel + 2.0 * quantum / (dt * dt);

    // Stream 5 cm/s along x for 1.5 s.
    var pull = new motionkit.kinematics.Twist6(0.05, 0.0, 0.0, 0.0, 0.0, 0.0);
    var first = trace.length - 1;
    var samples:Array<{t:Float, x:Float, y:Float}> = [];
    for (i in 0...150) {
      tickOnce(pull);
      if (i >= 100) {
        var tool = arm.tcpPose(positions()).translation;
        samples.push({t: i * dt, x: tool.x, y: tool.y});
      }
    }
    var a = samples[0], b = samples[samples.length - 1];
    near((b.x - a.x) / (b.t - a.t), 0.05, '$label: the tool moves at the streamed 5 cm/s', 0.0025);
    near((b.y - a.y) / (b.t - a.t), 0.0, '$label: the tool does not drift sideways', 0.0025);
    check(worstAcceleration(first) <= slack, '$label: streaming stays within the acceleration limits (${worstAcceleration(first)})');
    check(worstSpeed(first) <= 2.0 + quantum / dt, '$label: streaming stays within the velocity limits');

    // The operator stops (the host keeps updating): the arm brakes to rest.
    var peak = worstSpeed(trace.length - 2);
    session.stop();
    first = trace.length - 1;
    var rested = false, ticks = 0;
    while (!rested && ticks < 200) {
      rested = session.update().atRest;
      advance();
      ticks++;
    }
    check(rested, '$label: a stopped servo comes to rest');
    // Braking starts at the end of the queue, one lead ahead. A device's queue is done once the
    // runtime's copy of it has run out too, an owner period after the device finishes.
    check(ticks * dt <= peak / accel + lead + dt + 0.03, '$label: braking takes about v/a (${ticks * dt} s for $peak rad/s)');
    check(worstAcceleration(first) <= slack, '$label: braking stays within the acceleration limits (${worstAcceleration(first)})');
    for (_ in 0...10) advance();
    check(still(9), '$label: the arm stays at rest');

    // Turn about the base into the base joint's 1.2 rad stop.
    first = trace.length - 1;
    var overshoot = 0.0;
    for (_ in 0...400) {
      var tool = arm.tcpPose(positions()).translation;
      tickOnce(new motionkit.kinematics.Twist6(-tool.y * 1.5, tool.x * 1.5, 0.0, 0.0, 0.0, 1.5));
      overshoot = Math.max(overshoot, positions()[0] - 1.2);
    }
    check(overshoot <= quantum, '$label: the base joint never passes its stop ($overshoot)');
    near(positions()[0], 1.2, '$label: the base joint ends on its stop', 2e-3);
    check(worstAcceleration(first) <= slack, '$label: approaching the stop stays within the acceleration limits (${worstAcceleration(first)})');
    // Last, as a device latches its stop: the host stalls mid-stream, the queue runs dry within the lead,
    // and the robot brakes every joint by itself.
    first = trace.length - 1;
    var back = new motionkit.kinematics.Twist6(0.0, 0.0, -0.08, 0.0, 0.0, 0.0);
    for (_ in 0...60) tickOnce(back);
    var moving = worstSpeed(trace.length - 2);
    check(moving > 0.05, '$label: the arm is moving when the host stalls ($moving rad/s)');
    var stalledAt = trace.length - 1, stoppedAt = -1;
    for (_ in 0...100) {
      advance();
      if (stoppedAt < 0 && still(1)) stoppedAt = trace.length - 2;
    }
    var stopSeconds = (stoppedAt - stalledAt) * dt;
    check(stoppedAt >= 0 && stopSeconds <= lead + moving / accel + 0.04,
      '$label: a stalled host leaves the arm stopping within the lead plus v/a ($stopSeconds s)');
    check(still(40), '$label: the stalled arm stays at rest');
    // The runtime reports the underflow and stays ready; a device latches its stop as a fault.
    if (virtual) check(robot.snapshot().safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT, '$label: the device latches the underflow stop');
    else check(robot.snapshot().safety == RobotKitRuntimeConstants.RK_SAFETY_READY &&
      robot.snapshot().faultCode == RobotKitRuntimeConstants.RK_FAULT_TRAJECTORY_UNDERFLOW, '$label: the runtime reports the underflow');
    check(worstAcceleration(first) <= slack, '$label: the stalled stop stays within the acceleration limits (${worstAcceleration(first)})');

    session.dispose();
    harness.dispose();
  }

  public function testServoSession():Void {
    var fixture = buildContractArmFixture();
    for (joint in fixture.model.joints) { joint.limits.velocity = 2.0; joint.limits.maxAcceleration = 4.0; }
    fixture.model.joints[0].limits.upper = 1.2;
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(RobotRuntimeCompiler.compile(fixture.model));
    var robot = new SimulatedRobot("servo-arm", runtime, fixture.model.name,
      [for (link in fixture.model.links) link.name], [for (joint in fixture.model.joints) joint.name]);
    var arm = fixture.arm;
    var tick = 0;
    function positions():Array<Float> return [for (i in 0...6) robot.snapshot().positions.get(i)];
    // Start away from singularities.
    var start = [0.3, -0.8, 1.1, -0.5, 0.4, 0.2];
    robot.submit(RobotCommand.JointTargets([for (i in 0...6) JointTarget.position(i, start[i])], null));
    for (_ in 0...300) harness.step(Int64.ofInt(tick++));
    var session = new ServoSession(robot, arm);
    session.update();
    var ms = Int64.ofInt(1000000);
    var accel = 4.0, dt = 0.01;
    var worstJump = 0.0, fastest = 0.0, sequence = 0;
    var previous = [for (_ in 0...6) 0.0];
    function tickOnce(refresh:Null<motionkit.kinematics.Twist6>) {
      if (refresh != null) check(session.command(refresh, ++sequence, session.nowNs() + ms * 100) == null, "a fresh command is accepted");
      harness.step(Int64.ofInt(tick++));
      var result = session.update();
      for (i in 0...6) {
        worstJump = Math.max(worstJump, Math.abs(result.velocity[i] - previous[i]));
        fastest = Math.max(fastest, Math.abs(result.velocity[i]));
      }
      previous = result.velocity;
      return result;
    }

    // Stream 5 cm/s along x for 1.5 s; the tool should settle at that speed.
    var pull = new motionkit.kinematics.Twist6(0.05, 0.0, 0.0, 0.0, 0.0, 0.0);
    var samples:Array<{t:Float, x:Float, y:Float, z:Float}> = [];
    for (i in 0...150) {
      tickOnce(pull);
      if (i >= 100) {
        var tool = arm.tcpPose(positions()).translation;
        samples.push({t: i * dt, x: tool.x, y: tool.y, z: tool.z});
      }
    }
    var first = samples[0], last = samples[samples.length - 1];
    var span = last.t - first.t;
    near((last.x - first.x) / span, 0.05, "the tool moves at the streamed 5 cm/s", 0.0025);
    near((last.y - first.y) / span, 0.0, "the tool does not drift sideways", 0.0025);
    check(worstJump <= accel * dt + 1e-9, 'joint velocities change by at most a·dt per tick ($worstJump)');
    check(fastest <= 2.0 + 1e-9, "joint velocities stay within their limits");

    check(session.command(pull, sequence, session.nowNs() + ms * 100) == Stale, "a repeated sequence is stale");
    check(session.command(pull, sequence + 1, session.nowNs()) == Expired, "a deadline already passed is rejected");

    // The operator goes quiet: once the deadline passes the arm brakes to rest within its limits.
    var peak = 0.0;
    for (v in previous) peak = Math.max(peak, Math.abs(v));
    var braking = 0, rested = false;
    worstJump = 0.0;
    while (braking < 200 && !rested) {
      var result = tickOnce(null);
      if (result.braking) braking++;
      rested = result.atRest;
    }
    check(rested && !session.following(), "a lost operator leaves the arm at rest");
    check(worstJump <= accel * dt + 1e-9, 'braking stays within the acceleration limits ($worstJump)');
    check(braking * dt <= peak / accel + 0.15, 'braking takes about v/a (${braking * dt} s for ${peak} rad/s)');
    for (_ in 0...20) harness.step(Int64.ofInt(tick++));
    for (i in 0...6) near(robot.snapshot().velocities.get(i), 0.0, "the simulated joints are at rest", 1e-6);

    // Turn the arm about its base (v = ω × p) into the base joint's 1.2 rad stop.
    worstJump = 0.0;
    var overshoot = 0.0;
    for (_ in 0...400) {
      var tool = arm.tcpPose(positions()).translation;
      tickOnce(new motionkit.kinematics.Twist6(-tool.y * 1.5, tool.x * 1.5, 0.0, 0.0, 0.0, 1.5));
      overshoot = Math.max(overshoot, positions()[0] - 1.2);
    }
    check(overshoot <= 1e-9, 'the base joint never passes its stop ($overshoot)');
    near(positions()[0], 1.2, "the base joint ends on its stop", 1e-3);
    check(worstJump <= accel * dt + 1e-9, 'approaching the stop stays within the acceleration limits ($worstJump)');

    // The host stalls mid-stream: the runtime enforces the deadline itself and brakes every joint.
    var back = new motionkit.kinematics.Twist6(0.0, 0.0, -0.08, 0.0, 0.0, 0.0);
    for (_ in 0...60) tickOnce(back);
    // The last targets sent carry the last command's 100 ms deadline.
    var stalledAt = session.nowNs();
    var seen = [for (i in 0...6) robot.snapshot().velocities.get(i)];
    var moving = 0.0;
    for (v in seen) moving = Math.max(moving, Math.abs(v));
    check(moving > 0.05, 'the arm is moving when the host stalls ($moving rad/s)');
    var stalledJump = 0.0, stoppedAfter = -1.0;
    for (step in 0...200) {
      harness.step(Int64.ofInt(tick++));
      var now = [for (i in 0...6) robot.snapshot().velocities.get(i)];
      var fastestNow = 0.0;
      for (i in 0...6) {
        stalledJump = Math.max(stalledJump, Math.abs(now[i] - seen[i]));
        fastestNow = Math.max(fastestNow, Math.abs(now[i]));
      }
      seen = now;
      if (fastestNow <= 1e-9 && stoppedAfter < 0.0) stoppedAfter = Int64.toInt(robot.snapshot().sourceTimestampNs - stalledAt) * 1e-9;
    }
    check(stoppedAfter >= 0.1, 'the runtime keeps the stream running until its deadline ($stoppedAfter s)');
    check(stoppedAfter <= 0.1 + moving / accel + 0.02,
      'the runtime stops a stalled stream within v/a of its deadline ($stoppedAfter s)');
    check(stalledJump <= accel * dt * 1.01, 'the runtime brakes within the acceleration limits ($stalledJump)');
    check(robot.snapshot().faultCode == 7, "the snapshot reports the lapsed command");
    session.dispose();
    harness.dispose();
  }

  public function testTransmissionDerivedAxisMapping():Void {
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

  public function testAxisKinematics():Void {
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

  public function testDualMotorAxisRunsThroughSimulation():Void {
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
    var simulationHarness = new SimulationHarness(0.01);
    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("dual-motor-x", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    var logicalAxis = machine.axis("x");
    check(logicalAxis != null && logicalAxis.jointIndices.length == 2,
      "one logical axis exposes both dual-motor joints");

    machine.home();
    runMotion(machine, simulationHarness);
    machine.moveAxes([new AxisTarget("x", 0.035)], new MotionOptions(0.08, 0.4));
    runMotion(machine, simulationHarness);
    var snapshot = robot.snapshot();
    near(snapshot.positions.get(0), 0.035, "dual-motor axis reaches its logical target on motor one", 1e-5);
    near(snapshot.positions.get(1), 0.035, "dual-motor axis reaches its logical target on motor two", 1e-5);
    simulationHarness.dispose();
  }

}
