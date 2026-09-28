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

  public function testKinematicsContract():Void {
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
      [for (_ in 0...6) 20.0], StartTolerances.uniform(6, 0.02, 0.02, 0.02));
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

}
