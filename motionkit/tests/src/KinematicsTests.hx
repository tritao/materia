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
import robotkit.core.JointTarget;
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

class KinematicsTests extends MotionKitTestSupport {
  public function new() { super(); }

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
    check(arm.numericSolveCount() == 0 && arm.numericSolveCount(own) == 0,
      "forward queries do not count as numeric IK");
    arm.solve(arm.tcpPose(configurations[5], own), configurations[5], null, own);
    check(arm.numericSolveCount(own) == 1 && arm.numericSolveCount() == 0,
      "numeric IK diagnostics stay with their caller-owned context");
    var countDone = new sys.thread.Lock();
    var workerCount = [0];
    sys.thread.Thread.create(function() {
      arm.solve(arm.tcpPose(configurations[5]), configurations[5]);
      workerCount[0] = arm.numericSolveCount();
      countDone.release();
    });
    countDone.wait();
    check(workerCount[0] == 1 && arm.numericSolveCount() == 0,
      "numeric IK diagnostics stay isolated between planning threads");
  }

  public function testKinematicsContract():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(fixture.arm, 1e-8);
    check(solver.jointCount() == 6, "kinematics adapter reports the manipulator joint count");

    var q = [0.3, -0.5, 0.8, -0.2, 0.6, -0.4];
    var target = solver.forward(q);
    var seed = [for (value in q) value + 0.03];
    var tolerance = new IkTolerance(1e-5, 1e-4, 200, 0.02, 1e-3);
    var solved = solver.solvePose(target, seed, tolerance, null);
    check(solved != null, "kinematics adapter solves a reachable TCP pose");
    var achieved = solver.forward(cast solved);
    near(achieved.x, target.x, "forward/solve round trip preserves TCP x", 1e-5);
    near(achieved.y, target.y, "forward/solve round trip preserves TCP y", 1e-5);
    near(achieved.z, target.z, "forward/solve round trip preserves TCP z", 1e-5);
    near(Math.abs(achieved.qx * target.qx + achieved.qy * target.qy +
      achieved.qz * target.qz + achieved.qw * target.qw), 1.0,
      "forward/solve round trip preserves TCP orientation", 1e-4);

    var zeroTarget = solver.forward([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    var firstCandidates = solver.sampleCandidates(zeroTarget, 4, tolerance, null);
    var secondCandidates = solver.sampleCandidates(zeroTarget, 4, tolerance, null);
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
      requested[3], requested[4], requested[5]), null, null);
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

  public function testExternalOpwIk():Void {
    var model = new RobotModel("opw-track-positioner");
    var root = model.addLink(new Link("cell")), carriage = model.addLink(new Link("carriage"));
    var track = model.addJoint(new Joint("track", JointType.Prismatic, root, carriage));
    track.axis = [0.8, 0.6, 0.0]; track.limits.lower = -1; track.limits.upper = 1;
    var yaw = Quat.fromAxisAngle(new Vec3(0, 0, 1), 0.37);
    track.parentFrameRotation = [yaw.x, yaw.y, yaw.z, yaw.w];
    var positions = [[0.0, 0.0, 0.0], [0.1, 0.0, 0.615], [0.0, 0.0, 0.705],
      [-0.135, 0.0, 0.755], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
    var axes = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
      [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    var parent = carriage;
    for (index in 0...6) {
      var child = model.addLink(new Link('arm-link-$index'));
      var joint = model.addJoint(new Joint('arm-joint-$index', JointType.Revolute, parent, child));
      joint.axis = axes[index]; joint.parentFramePosition = positions[index];
      joint.limits.lower = -2 * Math.PI; joint.limits.upper = 2 * Math.PI;
      parent = child;
    }
    var flange = model.addFrame(new Frame("arm-flange", parent)); flange.position = [0, 0, 0.085];
    var work = model.addLink(new Link("positioner-work"));
    var positioner = model.addJoint(new Joint("positioner", JointType.Revolute, root, work));
    positioner.axis = [0, 0, 1]; positioner.parentFramePosition = [0.25, -0.3, 0.1];
    positioner.limits.lower = -2 * Math.PI; positioner.limits.upper = 2 * Math.PI;
    var workFrame = model.addFrame(new Frame("work-frame", work)); workFrame.position = [0.07, 0.02, 0.15];
    var tcp = new Transform3(new Vec3(0.04, 0.02, 0.13), Quat.fromAxisAngle(new Vec3(0, 1, 0), 0.2));
    for (withPositioner in [false, true]) {
      var group = new robotkit.manipulation.KinematicGroup(model, root.id, flange.id,
        withPositioner ? workFrame.id : null, tcp, null, ["track"]);
      var analytic = new OpwKinematics(model, group);
      var numeric = new ManipulatorKinematics(group);
      var nativeSampler = new motionkit.robot.SerialCandidateSampler(group);
      check(analytic.jointCount() == (withPositioner ? 8 : 7), "OPW group preserves arm and external DOFs");
      var before = group.numericSolveCount();
      var nativeSeed = [for(i in 0...group.group.count()) 0.3*Math.sin(i+0.4)];
      var nativeTarget = numeric.forward(nativeSeed);
      var nativeBefore = group.numericSolveCount();
      var grid = [new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(0,nativeSeed[0]-0.02,nativeSeed[0]+0.02,3)];
      if(withPositioner) grid.push(new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(nativeSeed.length-1,nativeSeed[nativeSeed.length-1]-0.03,nativeSeed[nativeSeed.length-1]+0.03,3));
      var coneAxis = group.tcpPose(nativeSeed).transformVector(new Vec3(0,0,1)).toArray();
      for(policy in [motionkit.path.OrientationPolicy.Fixed,motionkit.path.OrientationPolicy.FreeAboutTool,
          motionkit.path.OrientationPolicy.Cone(coneAxis,0.1)]) {
        var activeGrid = switch policy {
          case Cone(_, _): [for(joint in 0...nativeSeed.length)if(group.external[joint])
            new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(joint,nativeSeed[joint],nativeSeed[joint],1)];
          default: grid;
        };
        var candidates = nativeSampler.sample(nativeTarget,nativeSeed,policy,activeGrid,4,1,4);
        var repeated = nativeSampler.sample(nativeTarget,nativeSeed,policy,activeGrid,4,1,4);
        check(candidates.length == repeated.length,"serial candidate counts are deterministic");
        check(candidates.length>0,"combined native serial sampler has candidates");
        var original = false;
        for(index in 0...candidates.length) {
          var candidate = candidates[index],again = repeated[index];
          check(candidate.branch == again.branch && candidate.singular == again.singular && candidate.roll == again.roll &&
            candidate.tilt == again.tilt && candidate.azimuth == again.azimuth,"serial branch and orientation coordinates are deterministic");
          for(joint in 0...candidate.q.length) {
            near(candidate.q[joint],again.q[joint],"serial candidate joints are deterministic",1e-12);
            check(candidate.wraps[joint] == again.wraps[joint],"serial candidate wraps are deterministic");
            var bound = group.group.limitsOf(joint);
            check(candidate.q[joint] >= bound.lower-1e-9 && candidate.q[joint] <= bound.upper+1e-9,"combined serial lifts satisfy all compiled bounds");
            if(group.external[joint])check(candidate.wraps[joint]==0,"external lattice coordinates are never independently lifted");
          }
          for(axis in 0...candidate.external.length) {
            check(candidate.external[axis]==again.external[axis],"serial external coordinates are deterministic");
            var range = activeGrid[axis];
            var fraction = range.points == 1 ? 0.0 : candidate.external[axis]/(range.points-1);
            near(candidate.q[range.joint],range.lower+(range.upper-range.lower)*fraction,"serial external coordinates match their values",1e-12);
          }
          var same = true;
          for (joint in 0...nativeSeed.length) if (Math.abs(candidate.q[joint]-nativeSeed[joint])>1e-5) same = false;
          original = original || same;
          var actual=numeric.forward(candidate.q);
          near(motionkit.path.PoseMath.distance(actual,nativeTarget),0,"exported serial native candidate task position",1e-6);
          near(motionkit.robot.ToolFreedom.orientationError(actual,nativeTarget,policy),0,"exported serial native candidate orientation",1e-6);
        }
        switch policy { case Cone(_, _): default: check(original,"combined native serial sampling retains the original centre-cell branch"); }
      }
      check(group.numericSolveCount()==nativeBefore,"combined serial native sampler uses no numeric IK");
      var ruleEnd = nativeSeed.copy();ruleEnd[0] += 0.005;
      if(withPositioner)ruleEnd[ruleEnd.length-1] += 0.005;
      var pathRequest = new PathRequest([0.0,0.01],[nativeTarget,numeric.forward(ruleEnd)],nativeSeed,new IkTolerance(1e-6,1e-6),
        [for(_ in nativeSeed)0.5],[for(_ in nativeSeed)1.0]);
      var options = new motionkit.robot.CandidateProblem.CandidateSamplingOptions(4,1,4,true,null,
        (index,distance,target) -> {
          var q = index == 0 ? nativeSeed : ruleEnd;
          return [for(joint in 0...q.length)if(group.external[joint])new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(joint,q[joint],q[joint],1)];
        });
      var pathProblem = new motionkit.robot.CandidateProblem(group,pathRequest,options);
      check(pathProblem.samples[0].candidates.length == 1 && pathProblem.samples[1].candidates.length > 0,
        "per-sample external rules build a populated path ladder");
      for(candidate in pathProblem.samples[1].candidates)for(joint in 0...ruleEnd.length)if(group.external[joint])
        near(candidate.q[joint],ruleEnd[joint],"path ladder holds the per-sample external rule",1e-12);

      checkMovingExternalRefinement(group,pathProblem,nativeSeed);

      for (sample in 0...30) {
        var q = [for (joint in 0...group.group.count()) (joint == 0 ? 0.8 : 1.4) * Math.sin((sample + 1) * (joint + 1) * 1.618)];
        var target = numeric.forward(q), original = false;
        near(motionkit.path.PoseMath.distance(analytic.forward(q), target), 0,
          "External OPW analytic FK matches the compiled full-group model", 1e-6);
        if (sample == 0) {
          var solved = analytic.solvePose(target, q, new IkTolerance(1e-6, 1e-6));
          check(solved != null, "External OPW KinematicsSolver interface returns a branch");
          var answer:Array<Float> = cast solved;
          for (joint in 0...q.length) near(answer[joint], q[joint],
            "External OPW seeded solve selects the original complete configuration", 1e-5);
        }
        for (branch in analytic.branches(target, q)) {
          var same = true;
          for (joint in 0...q.length) {
            same = same && Math.abs(branch.q[joint] - q[joint]) < 1e-5;
            if (group.external[joint]) near(branch.q[joint], q[joint], "OPW holds the requested external lattice cell", 1e-12);
          }
          original = original || same;
          var actual = numeric.forward(branch.q);
          near(motionkit.path.PoseMath.distance(actual, target), 0, "External OPW preserves TCP position in its reference", 1e-6);
          near(motionkit.robot.ToolFreedom.orientationError(actual, target, motionkit.path.OrientationPolicy.Fixed),
            0, "External OPW preserves work-frame orientation", 1e-6);
        }
        check(original, "External OPW includes the original arm branch");
      }
      check(group.numericSolveCount() == before, "External OPW uses no numeric pose queries");
    }
  }

  public function testExternalUrIk():Void {
    var model = new RobotModel("ur-track-positioner");
    var root = model.addLink(new Link("cell")), carriage = model.addLink(new Link("carriage"));
    var track = model.addJoint(new Joint("track", JointType.Prismatic, root, carriage));
    track.axis = [0.8, 0.6, 0.0]; track.limits.lower = -1; track.limits.upper = 1;
    var yaw = Quat.fromAxisAngle(new Vec3(0, 0, 1), 0.37);
    track.parentFrameRotation = [yaw.x, yaw.y, yaw.z, yaw.w];
    var positions = [[0.0,0.0,0.089159],[0.0,0.13585,0.0],[0.0,-0.1197,0.425],
      [0.0,0.0,0.39225],[0.0,0.10915,0.0],[0.0,0.0,0.09465]];
    var axes = [[0.0,0.0,1.0],[0.0,1.0,0.0],[0.0,1.0,0.0],
      [0.0,1.0,0.0],[0.0,0.0,1.0],[0.0,1.0,0.0]];
    var parent = carriage;
    for (index in 0...6) {
      var child = model.addLink(new Link('arm-link-$index'));
      var joint = model.addJoint(new Joint('arm-joint-$index', JointType.Revolute, parent, child));
      joint.axis = axes[index]; joint.parentFramePosition = positions[index];
      joint.limits.lower = -2 * Math.PI; joint.limits.upper = 2 * Math.PI;
      parent = child;
    }
    var flange = model.addFrame(new Frame("arm-flange", parent)); flange.position = [0, 0.0823, 0];
    var work = model.addLink(new Link("positioner-work"));
    var positioner = model.addJoint(new Joint("positioner", JointType.Revolute, root, work));
    positioner.axis = [0, 0, 1]; positioner.parentFramePosition = [0.25, -0.3, 0.1];
    positioner.limits.lower = -2 * Math.PI; positioner.limits.upper = 2 * Math.PI;
    var workFrame = model.addFrame(new Frame("work-frame", work)); workFrame.position = [0.07, 0.02, 0.15];
    var tcp = new Transform3(new Vec3(0.04, 0.02, 0.13), Quat.fromAxisAngle(new Vec3(0, 1, 0), 0.2));
    for (withPositioner in [false, true]) {
      var group = new robotkit.manipulation.KinematicGroup(model, root.id, flange.id,
        withPositioner ? workFrame.id : null, tcp, null, ["track"]);
      var analytic = new motionkit.robot.UrAnalyticIk(group);
      var numeric = new ManipulatorKinematics(group);
      var nativeSampler = new motionkit.robot.SerialCandidateSampler(group);
      check(analytic.jointCount() == (withPositioner ? 8 : 7), "UR group preserves arm and external DOFs");
      var before = group.numericSolveCount();
      var seed = [for (i in 0...group.group.count()) 0.3*Math.sin(i+0.4)];
      var ranges = [new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(0,-0.2,0.2,3)];
      if (withPositioner) ranges.push(new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(seed.length-1,-0.4,0.4,2));
      var cells = motionkit.robot.ExternalAxisGrid.sample(group,seed,ranges);
      check(cells.length == (withPositioner ? 6 : 3),"native external grid Cartesian-product count");
      for (i in 0...cells.length) {
        var cell = cells[i];
        check(cell.coordinates[0] == (withPositioner ? Std.int(i/2) : i),"track lattice coordinate follows group ordering");
        if (withPositioner) check(cell.coordinates[1] == i%2,"positioner lattice coordinate varies fastest");
        for (joint in 0...seed.length) if (!group.external[joint])
          near(cell.q[joint],seed[joint],"native external grid retains arm joints",1e-12);
        var target = numeric.forward(cell.q),found = false;
        for (branch in analytic.branches(target,cell.q)) {
          var same = true;
          for (joint in 0...seed.length) if (Math.abs(branch.q[joint]-cell.q[joint]) > 1e-5) same = false;
          found = found || same;
        }
        check(found,"native rotated track/positioner cell retains original UR branch");
      }
      var heldCells = motionkit.robot.ExternalAxisGrid.sample(group,seed,[]);
      check(heldCells.length == 1,"unspecified external axes become one-point rules");
      for (joint in 0...seed.length) near(heldCells[0].q[joint],seed[joint],"held external cell copies complete seed",1e-12);
      var nativeSeed = [for(i in 0...group.group.count()) 0.3*Math.sin(i+0.4)];
      var nativeTarget = numeric.forward(nativeSeed);
      var nativeBefore = group.numericSolveCount();
      var grid = [new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(0,nativeSeed[0]-0.02,nativeSeed[0]+0.02,3)];
      if(withPositioner) grid.push(new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(nativeSeed.length-1,nativeSeed[nativeSeed.length-1]-0.03,nativeSeed[nativeSeed.length-1]+0.03,3));
      var coneAxis = group.tcpPose(nativeSeed).transformVector(new Vec3(0,0,1)).toArray();
      for(policy in [motionkit.path.OrientationPolicy.Fixed,motionkit.path.OrientationPolicy.FreeAboutTool,
          motionkit.path.OrientationPolicy.Cone(coneAxis,0.1)]) {
        var activeGrid = switch policy {
          case Cone(_, _): [for(joint in 0...nativeSeed.length)if(group.external[joint])
            new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(joint,nativeSeed[joint],nativeSeed[joint],1)];
          default: grid;
        };
        var candidates = nativeSampler.sample(nativeTarget,nativeSeed,policy,activeGrid,4,1,4);
        var repeated = nativeSampler.sample(nativeTarget,nativeSeed,policy,activeGrid,4,1,4);
        check(candidates.length == repeated.length,"serial candidate counts are deterministic");
        check(candidates.length>0,"combined native serial sampler has candidates");
        var original = false;
        for(index in 0...candidates.length) {
          var candidate = candidates[index],again = repeated[index];
          check(candidate.branch == again.branch && candidate.singular == again.singular && candidate.roll == again.roll &&
            candidate.tilt == again.tilt && candidate.azimuth == again.azimuth,"serial branch and orientation coordinates are deterministic");
          for(joint in 0...candidate.q.length) {
            near(candidate.q[joint],again.q[joint],"serial candidate joints are deterministic",1e-12);
            check(candidate.wraps[joint] == again.wraps[joint],"serial candidate wraps are deterministic");
            var bound = group.group.limitsOf(joint);
            check(candidate.q[joint] >= bound.lower-1e-9 && candidate.q[joint] <= bound.upper+1e-9,"combined serial lifts satisfy all compiled bounds");
            if(group.external[joint])check(candidate.wraps[joint]==0,"external lattice coordinates are never independently lifted");
          }
          for(axis in 0...candidate.external.length) {
            check(candidate.external[axis]==again.external[axis],"serial external coordinates are deterministic");
            var range = activeGrid[axis];
            var fraction = range.points == 1 ? 0.0 : candidate.external[axis]/(range.points-1);
            near(candidate.q[range.joint],range.lower+(range.upper-range.lower)*fraction,"serial external coordinates match their values",1e-12);
          }
          var same = true;
          for (joint in 0...nativeSeed.length) if (Math.abs(candidate.q[joint]-nativeSeed[joint])>1e-5) same = false;
          original = original || same;
          var actual=numeric.forward(candidate.q);
          near(motionkit.path.PoseMath.distance(actual,nativeTarget),0,"exported serial native candidate task position",1e-6);
          near(motionkit.robot.ToolFreedom.orientationError(actual,nativeTarget,policy),0,"exported serial native candidate orientation",1e-6);
        }
        switch policy { case Cone(_, _): default: check(original,"combined native serial sampling retains the original centre-cell branch"); }
      }
      check(group.numericSolveCount()==nativeBefore,"combined serial native sampler uses no numeric IK");
      var ruleEnd = nativeSeed.copy();ruleEnd[0] += 0.005;
      if(withPositioner)ruleEnd[ruleEnd.length-1] += 0.005;
      var pathRequest = new PathRequest([0.0,0.01],[nativeTarget,numeric.forward(ruleEnd)],nativeSeed,new IkTolerance(1e-6,1e-6),
        [for(_ in nativeSeed)0.5],[for(_ in nativeSeed)1.0]);
      var options = new motionkit.robot.CandidateProblem.CandidateSamplingOptions(4,1,4,true,null,
        (index,distance,target) -> {
          var q = index == 0 ? nativeSeed : ruleEnd;
          return [for(joint in 0...q.length)if(group.external[joint])new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(joint,q[joint],q[joint],1)];
        });
      var pathProblem = new motionkit.robot.CandidateProblem(group,pathRequest,options);
      check(pathProblem.samples[0].candidates.length == 1 && pathProblem.samples[1].candidates.length > 0,
        "per-sample external rules build a populated path ladder");
      for(candidate in pathProblem.samples[1].candidates)for(joint in 0...ruleEnd.length)if(group.external[joint])
        near(candidate.q[joint],ruleEnd[joint],"path ladder holds the per-sample external rule",1e-12);

      checkMovingExternalRefinement(group,pathProblem,nativeSeed);

      for (sample in 0...30) {
        var q = [for (joint in 0...group.group.count()) (joint == 0 ? 0.8 : 1.4) * Math.sin((sample + 1) * (joint + 1) * 1.618)];
        var target = numeric.forward(q), original = false;
        near(motionkit.path.PoseMath.distance(analytic.forward(q), target), 0,
          "External UR analytic FK matches the compiled full-group model", 1e-6);
        for (branch in analytic.branches(target, q)) {
          var same = true;
          for (joint in 0...q.length) {
            same = same && Math.abs(branch.q[joint] - q[joint]) < 1e-5;
            if (group.external[joint]) near(branch.q[joint], q[joint], "UR holds the requested external lattice cell", 1e-12);
          }
          original = original || same;
          var actual = numeric.forward(branch.q);
          near(motionkit.path.PoseMath.distance(actual, target), 0, "External UR preserves TCP position in its reference", 1e-6);
          near(motionkit.robot.ToolFreedom.orientationError(actual, target, motionkit.path.OrientationPolicy.Fixed),
            0, "External UR preserves work-frame orientation", 1e-6);
        }
        check(original, "External UR includes the original arm branch");
      }
      check(group.numericSolveCount() == before, "External UR uses no numeric pose queries");
    }
  }

  function checkMovingExternalRefinement(group:robotkit.manipulation.KinematicGroup,
      pathProblem:motionkit.robot.CandidateProblem,nativeSeed:Array<Float>):Void {
    var numeric = new ManipulatorKinematics(group);
    var selected = motionkit.robot.StructuredLadder.search(pathProblem);
    check(selected.diagnostic == null,"moving external axes have a connected analytic route");
    var refiner = new motionkit.robot.AnalyticPathRefiner(group,pathProblem,selected);
    // Independent full-model FK differences supply task rates. Arm joints
    // remain constant while the track and optional work frame move together.
    function externalPose(s:Float):Pose3 {
      var q = nativeSeed.copy();
      for(j in 0...q.length)if(group.external[j])q[j] += 0.5*s;
      return numeric.forward(q);
    }
    var refinedSeed = nativeSeed.copy(),h = 1e-4;
    for(i in 0...11) {
      var s = 0.001*i,p = externalPose(s),minus = externalPose(s-h),plus = externalPose(s+h);
      var omega = poseRotationDelta(minus,plus,1/(2*h));
      var omegaBefore = poseRotationDelta(externalPose(s-2*h),p,1/(2*h));
      var omegaAfter = poseRotationDelta(p,externalPose(s+2*h),1/(2*h));
      var velocity = [(plus.x-minus.x)/(2*h),(plus.y-minus.y)/(2*h),(plus.z-minus.z)/(2*h)].concat(omega);
      var acceleration = [(plus.x-2*p.x+minus.x)/(h*h),(plus.y-2*p.y+minus.y)/(h*h),
        (plus.z-2*p.z+minus.z)/(h*h)].concat([for(k in 0...3)(omegaAfter[k]-omegaBefore[k])/(2*h)]);
      refinedSeed = refiner.sample(s,p,motionkit.path.OrientationPolicy.Fixed,refinedSeed).q;
      var derivatives = refiner.refinedDerivatives(s,refinedSeed,p,motionkit.path.OrientationPolicy.Fixed,velocity,acceleration);
      for(j in 0...refinedSeed.length) {
        near(refinedSeed[j],nativeSeed[j]+(group.external[j]?0.5*s:0),"moving-frame refinement preserves the selected arm configuration",1e-6);
        near(derivatives.first[j],group.external[j]?0.5:0,"moving-frame differential rates separate arm and external motion",1e-6);
        near(derivatives.second[j],0,"moving-frame differential acceleration cancels reference curvature",1e-4);
      }
    }
  }

  public function testNumericBranchFallback():Void {
    var fixture = buildSevenAxisArmFixture();
    var fallback = new motionkit.robot.NumericBranchIk(fixture.arm,"seven-axis test arm",new IkTolerance(1e-6,1e-6,60));
    check(fallback.family() == "numeric-fallback" && fallback.diagnostic.indexOf("seven-axis test arm") >= 0,
      "unsupported geometry has an explicit numeric fallback diagnostic");
    var q = [for (i in 0...7) 0.3*Math.sin(i+0.4)];
    check(motionkit.robot.BranchIk.of(fixture.arm).family() == "numeric-fallback","unsupported model selects the numeric fallback");
    var target = fallback.forward(q);
    var first = fallback.branchesFromNeighbours(target,[q,q],q), second = fallback.branchesFromNeighbours(target,[q,q],q);
    check(first.length > 0 && first.length == second.length,"numeric fallback is deterministic and keeps the neighbour solution");
    for (i in 0...first.length) {
      check(first[i].branch == second[i].branch,"numeric fallback seed identity is deterministic");
      check(!first[i].singularityKnown,"numeric fallback reports singularity classification as unknown");
      near(motionkit.path.PoseMath.distance(fallback.forward(first[i].q),target),0,"numeric fallback satisfies task position",1e-6);
      near(motionkit.robot.ToolFreedom.orientationError(fallback.forward(first[i].q),target,motionkit.path.OrientationPolicy.Fixed),0,
        "numeric fallback satisfies task orientation",1e-6);
      for (j in 0...7) near(first[i].q[j],second[i].q[j],"numeric fallback deterministic joints",1e-9);
      for (j in 0...i) {
        var distance = 0.0;
        for (k in 0...7) distance += Math.pow(first[i].q[k]-first[j].q[k],2);
        check(Math.sqrt(distance) >= 1e-3,"numeric fallback deduplicates neighbour seeds");
      }
    }
    var sevenStart=[0.2,-0.8,0.6,-0.4,0.5,0.3,-0.2],sevenEnd=sevenStart.copy();sevenEnd[0]+=0.01;
    var sevenSolver=new ManipulatorKinematics(fixture.arm);
    var sevenLine=new PoseLine(new PoseWaypoint(sevenSolver.forward(sevenStart),1e-6,1e-6),
      new PoseWaypoint(sevenSolver.forward(sevenEnd),1e-6,1e-6),motionkit.path.OrientationPolicy.Interpolated,0.1,0.1);
    var sevenPath=new motionkit.path.PosePath("task",[sevenLine]),sevenDistances=[for(i in 0...5)sevenPath.length()*i/4];
    var sevenRequest=new PathRequest(sevenDistances,[for(s in sevenDistances)sevenPath.poseAt(s)],sevenStart,
      new IkTolerance(1e-6,1e-6),[for(_ in sevenStart)0.5],[for(_ in sevenStart)1.0]);
    var sevenPlanner=new motionkit.robot.StructuredJointPathPlanner(fixture.arm);
    var sevenSamples=sevenPlanner.plan(sevenPath,sevenRequest);
    check(sevenSamples.jointCount==7,"numeric refinement prescribes internal redundancy for a seven-axis arm");
    for(i in 0...sevenDistances.length)near(motionkit.path.PoseMath.distance(sevenSolver.forward(sevenSamples.q[i]),sevenPath.poseAt(sevenDistances[i])),0,
      "seven-axis differential refinement preserves authored task geometry",1e-6);
    var sevenMid=2,sevenQ=sevenSamples.q[sevenMid],sevenRate=sevenSamples.qPrime[sevenMid],sevenCurve=sevenSamples.qDoublePrime[sevenMid];
    var sevenStep=1e-5;
    function sevenDerivativePose(offset:Float):Pose3 return sevenSolver.forward([for(j in 0...7)
      sevenQ[j]+offset*sevenRate[j]+0.5*offset*offset*sevenCurve[j]]);
    var sevenCentre=sevenDerivativePose(0),sevenMinus=sevenDerivativePose(-sevenStep),sevenPlus=sevenDerivativePose(sevenStep);
    var sevenTask=sevenLine.derivativesAt(sevenDistances[sevenMid]);
    var sevenLinear=[(sevenPlus.x-sevenMinus.x)/(2*sevenStep),(sevenPlus.y-sevenMinus.y)/(2*sevenStep),(sevenPlus.z-sevenMinus.z)/(2*sevenStep)];
    var sevenSecond=[(sevenPlus.x-2*sevenCentre.x+sevenMinus.x)/(sevenStep*sevenStep),
      (sevenPlus.y-2*sevenCentre.y+sevenMinus.y)/(sevenStep*sevenStep),(sevenPlus.z-2*sevenCentre.z+sevenMinus.z)/(sevenStep*sevenStep)];
    var sevenAngular=poseRotationDelta(sevenMinus,sevenPlus,1/(2*sevenStep));
    var sevenAngularBefore=poseRotationDelta(sevenDerivativePose(-2*sevenStep),sevenCentre,1/(2*sevenStep));
    var sevenAngularAfter=poseRotationDelta(sevenCentre,sevenDerivativePose(2*sevenStep),1/(2*sevenStep));
    for(axis in 0...3){
      near(sevenLinear[axis],sevenTask.linear[axis],"seven-axis rates recover task velocity through independent FK",1e-5);
      near(sevenSecond[axis],sevenTask.linearSecond[axis],"seven-axis curvature recovers task acceleration through independent FK",1e-3);
      near(sevenAngular[axis],sevenTask.angular[axis],"seven-axis rates recover task angular velocity",1e-5);
      near((sevenAngularAfter[axis]-sevenAngularBefore[axis])/(2*sevenStep),sevenTask.angularSecond[axis],
        "seven-axis curvature recovers task angular acceleration",1e-3);
    }
    var sevenTimed=new ToppraPathTiming().time(sevenSamples,new PathTimingLimits([for(_ in sevenStart)1.0],[for(_ in sevenStart)2.0]));
    try {
      var sevenDuration=sevenTimed.trajectory.durationSeconds();check(sevenDuration>0,"internally redundant refinement succeeds in TOPP-RA");
      var sevenFinal=sevenTimed.trajectory.evaluate(sevenDuration);
      for(j in 0...7)near(sevenFinal.positions[j],sevenSamples.q[sevenSamples.q.length-1][j],"timed seven-axis path reaches its refined endpoint",1e-6);
      var sevenBounds=new ValidationLimits(7,Int64.ofInt(0),Int64.ofInt(0));
      for(j in 0...7){var bound=fixture.arm.group.limitsOf(j);sevenBounds.position(j,bound.lower,bound.upper);
        sevenBounds.velocity(j,1.00001);sevenBounds.acceleration(j,2.00001);}
      check(!sevenTimed.trajectory.validate(sevenBounds).hasFailure(),"timed seven-axis path passes native extrema limits");
    }catch(error:Dynamic){sevenTimed.releaseDistanceMap();sevenTimed.trajectory.dispose();throw error;}
    sevenTimed.releaseDistanceMap();sevenTimed.trajectory.dispose();
    var unsupportedFixture=buildContractArmFixture();
    unsupportedFixture.arm.robot.joints[4].axis=[0.1,0.0,Math.sqrt(0.99)];
    var unsupported=new robotkit.manipulation.KinematicGroup(unsupportedFixture.arm.robot,
      unsupportedFixture.arm.rootLink,unsupportedFixture.arm.flangeFrame);
    check(motionkit.robot.BranchIk.of(unsupported).family()=="numeric-fallback","skew wrist retains explicit numeric fallback");
    var unsupportedSolver=new ManipulatorKinematics(unsupported),seed=[0.2,-0.8,0.6,-0.4,0.5,0.3],goal=seed.copy();goal[0]+=0.01;
    var line=new PoseLine(new PoseWaypoint(unsupportedSolver.forward(seed),1e-6,1e-6),
      new PoseWaypoint(unsupportedSolver.forward(goal),1e-6,1e-6),motionkit.path.OrientationPolicy.Interpolated,0.1,0.1);
    var path=new motionkit.path.PosePath("task",[line]),distances=[for(i in 0...5)path.length()*i/4];
    var numericRequest=new PathRequest(distances,[for(s in distances)path.poseAt(s)],seed,new IkTolerance(1e-6,1e-6),
      [for(_ in seed)0.5],[for(_ in seed)1.0]);
    var numericPlanner=new motionkit.robot.StructuredJointPathPlanner(unsupported);
    var numericPath=numericPlanner.plan(path,numericRequest);
    check(numericPlanner.fallbackDiagnostic!=null,"numeric path planner retains the unsupported-geometry diagnostic");
    for(i in 0...distances.length)near(motionkit.path.PoseMath.distance(unsupportedSolver.forward(numericPath.q[i]),path.poseAt(distances[i])),0,
      "numeric continuation refinement preserves authored geometry",1e-6);
    var cell = buildWorkcellFixture();
    var held = new motionkit.robot.NumericBranchIk(cell.group,"test external arm",new IkTolerance(1e-6,1e-6,30));
    var complete = [for (i in 0...cell.group.group.count()) 0.1*Math.sin(i+0.3)];
    var candidates = held.branches(held.forward(complete),complete);
    check(candidates.length > 0,"numeric fallback accepts an external lattice cell");
    for (candidate in candidates) for (i in 0...complete.length) if (cell.group.external[i])
      near(candidate.q[i],complete[i],"numeric fallback holds external coordinates",1e-12);
  }

  public function testCandidateProblem():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(fixture.arm);
    var start = [0.2,-0.8,0.6,-0.4,0.5,0.3],end = start.copy();end[0] += 0.03;
    var request = new PathRequest([0.0,0.04],[solver.forward(start),solver.forward(end)],start,new IkTolerance(1e-6,1e-6),
      [for(_ in start)0.5],[for(_ in start)1.0],1);
    var before = fixture.arm.numericSolveCount();
    var problem = new motionkit.robot.CandidateProblem(fixture.arm,request);
    check(problem.family == "UR6R" && problem.diagnostic == null,"path request selects the native UR family");
    check(problem.samples.length == 2 && problem.samples[0].candidates.length == 1,"path problem pins the first complete configuration");
    check(problem.samples[1].candidates.length > request.maxCandidates,"native candidate problem retains all branches beyond the legacy cap");
    for(joint in 0...6)near(problem.samples[0].candidates[0].q[joint],start[joint],"pinned start stays exact",1e-12);
    var repeated = new motionkit.robot.CandidateProblem(fixture.arm,request);
    for(sample in 0...2) {
      check(problem.samples[sample].candidates.length == repeated.samples[sample].candidates.length,"candidate problem counts are deterministic");
      for(i in 0...problem.samples[sample].candidates.length) {
        var candidate = problem.samples[sample].candidates[i],again = repeated.samples[sample].candidates[i];
        check(candidate.branch == again.branch,"candidate problem branch order is deterministic");
        for(joint in 0...6)near(candidate.q[joint],again.q[joint],"candidate problem joints are deterministic",1e-12);
      }
    }
    check(fixture.arm.numericSolveCount() == before,"native PathRequest candidate construction makes no numeric IK calls");
    var selected=motionkit.robot.StructuredLadder.search(problem);
    check(selected.diagnostic==null && selected.candidates.length==2,"Haxe structured ladder selects a complete native route");
    var selectedAgain=motionkit.robot.StructuredLadder.search(problem);
    var coarseSelected=motionkit.robot.StructuredLadder.search(problem,null,0,null,
      new motionkit.robot.StructuredLadder.CoarseSearchOptions(1,1,24,0));
    check(coarseSelected.diagnostic==null && coarseSelected.candidates.length==selected.candidates.length,
      "Haxe coarse ladder selects a complete path");
    near(coarseSelected.cost,selected.cost,"full-corridor coarse route agrees with exact native search",1e-12);
    near(selected.cost,selectedAgain.cost,"structured native route cost is deterministic",1e-12);
    for(sample in 0...2){var actual=solver.forward(selected.candidates[sample].q);
      near(motionkit.path.PoseMath.distance(actual,request.poses[sample]),0,"selected native route retains task position",1e-6);
      near(motionkit.path.PoseMath.angle(actual,request.poses[sample]),0,"selected native route retains task orientation",1e-6);
      for(j in 0...6)near(selected.candidates[sample].q[j],selectedAgain.candidates[sample].q[j],"structured route is deterministic",1e-12);
    }
    var collisionChecks=0;
    var collisionRoute=motionkit.robot.LazyCollisionLadder.selectWithChecks(problem,q -> {collisionChecks++;return null;});
    near(collisionRoute.cost,selected.cost,"clear route retains native ladder cost",1e-12);
    check(collisionChecks==problem.samples.length,"lazy collision checks only selected samples");
    throws(function() motionkit.robot.LazyCollisionLadder.selectWithChecks(problem,q ->
      ({a:"arm",b:"fixture",distance:0.0,required:0.01})),"impossible pinned start reports collision blockage");
    throws(function() motionkit.robot.LazyCollisionLadder.selectWithChecks(problem,q -> null,8,
      (from,to) -> ({a:"tool",b:"post",distance:0.0,required:0.01})),"sweep blockage cannot return a clear route");
    var rerouteRequest=new PathRequest(request.distances,request.poses,start,request.tolerance,
      [for(_ in start)10.0],request.velocity);
    var rerouteProblem=new motionkit.robot.CandidateProblem(fixture.arm,rerouteRequest);
    var initialRoute=motionkit.robot.StructuredLadder.search(rerouteProblem);
    var preference = (sample:Int, candidate:motionkit.robot.CartesianCandidateSampler.LatticeCandidate) ->
      sample == 1 && candidate == initialRoute.candidates[1] ? 1000.0 : 0.0;
    var preferredRoute = motionkit.robot.StructuredLadder.search(rerouteProblem, null, 0, preference);
    var preferredClearRoute = motionkit.robot.LazyCollisionLadder.selectWithChecks(rerouteProblem,
      q -> null, 2, null, null, null, preference);
    check(preferredClearRoute.candidates[1] == preferredRoute.candidates[1] &&
      preferredClearRoute.candidates[1] != initialRoute.candidates[1],
      "lazy clearance retains process state preferences");
    near(preferredClearRoute.cost, preferredRoute.cost, "lazy clearance retains preference cost", 1e-12);
    var preferredRetry = motionkit.robot.LazyCollisionLadder.selectWithChecks(rerouteProblem,
      q -> q == preferredRoute.candidates[1].q ? {a:"tool", b:"post", distance:0.0, required:0.01} : null,
      2, null, null, null, preference);
    check(preferredRetry.candidates[1] != preferredRoute.candidates[1] &&
      preferredRetry.candidates[1] != initialRoute.candidates[1],
      "collision retries retain process preferences while excluding blocked states");
    var blockedEndpoint=initialRoute.candidates[1].q,rerouteChecks=0;
    var rerouted=motionkit.robot.LazyCollisionLadder.selectWithChecks(rerouteProblem,q -> {
      rerouteChecks++;
      return q==blockedEndpoint ? {a:"wrist",b:"obstacle",distance:0.0,required:0.01} : null;
    },2);
    check(rerouted.diagnostic==null && rerouted.candidates[1].q!=blockedEndpoint,
      "lazy collision retries select a different legal candidate");
    check(rerouteChecks==4,"one rejected sample causes exactly two bounded search rounds");
    near(motionkit.path.PoseMath.distance(solver.forward(rerouted.candidates[1].q),request.poses[1]),0,
      "collision rerouting retains the geometric task",1e-6);
    throws(function() motionkit.robot.LazyCollisionLadder.selectWithChecks(rerouteProblem,q ->
      q==blockedEndpoint ? {a:"wrist",b:"obstacle",distance:0.0,required:0.01} : null,1),
      "collision retry budget is enforced before accepting an unchecked alternate");
    var sweepChecks=0;
    var sweepRoute=motionkit.robot.LazyCollisionLadder.selectWithChecks(rerouteProblem,q -> null,2,
      (from,to) -> {sweepChecks++;return to==blockedEndpoint ? {a:"tool",b:"post",distance:0.0,required:0.01} : null;});
    check(sweepRoute.candidates[1].q!=blockedEndpoint && sweepChecks==2,
      "lazy sweep exclusions reroute the blocked transition in two rounds");
    var blockedEdge=new motionkit.robot.StructuredLadder.BlockedLadderEdge(1,
      rerouteProblem.samples[0].candidates.indexOf(initialRoute.candidates[0]),
      rerouteProblem.samples[1].candidates.indexOf(initialRoute.candidates[1]));
    var filteredRoute=motionkit.robot.StructuredLadder.search(rerouteProblem,null,0,null,null,[blockedEdge]);
    check(filteredRoute.diagnostic==null && filteredRoute.candidates[1].q!=blockedEndpoint,
      "Haxe edge records reach filtered native search");
    throws(function() motionkit.robot.StructuredLadder.search(rerouteProblem,null,0,null,null,
      [new motionkit.robot.StructuredLadder.BlockedLadderEdge(0,0,0)]),"Haxe rejects a nonadjacent blocked edge");
    var refinementChecks=0;
    var refinedReroute=motionkit.robot.LazyCollisionLadder.selectWithChecks(rerouteProblem,q -> null,2,null,null,route -> {
      refinementChecks++;
      return route.candidates[1].q==blockedEndpoint ? new motionkit.robot.LazyCollisionLadder.RefinedCollision(1,true,
        {a:"refined-tool",b:"post",distance:0.0,required:0.01}) : null;
    });
    check(refinementChecks==2 && refinedReroute.candidates[1].q!=blockedEndpoint,
      "refined sweep failure reroutes within the shared collision budget");
    var originalRoute=motionkit.robot.StructuredLadder.search(rerouteProblem);
    check(originalRoute.candidates[1].q==blockedEndpoint,"lazy collision filtering leaves the source candidate problem unchanged");
    var refiner=new motionkit.robot.AnalyticPathRefiner(fixture.arm,problem,selected);
    var previous=start.copy();
    for(i in 0...11){var q=start.copy();q[0]+=0.003*i;var target=solver.forward(q);
      var refined=refiner.sample(0.004*i,target,motionkit.path.OrientationPolicy.Fixed,previous);
      check(refined.branch==selected.candidates[0].branch,"refinement retains the selected geometric branch");
      near(motionkit.path.PoseMath.distance(solver.forward(refined.q),target),0,"refined analytic TCP preserves the task",1e-6);
      for(j in 0...q.length)near(refined.q[j],q[j],"fixed-orientation refinement retains the continuous joint lift",1e-6);
      previous=refined.q;
    }
    var h=1e-4,at=start.copy(),minus=start.copy(),plus=start.copy();
    at[0]+=0.015;minus[0]+=0.015-0.75*h;plus[0]+=0.015+0.75*h;
    var pa=solver.forward(at),pm=solver.forward(minus),pp=solver.forward(plus);
    var relative=new Quat(pp.qx,pp.qy,pp.qz,pp.qw).multiply(new Quat(pm.qx,pm.qy,pm.qz,pm.qw).conjugate());
    var length=Math.sqrt(relative.x*relative.x+relative.y*relative.y+relative.z*relative.z);
    var angularScale=2*Math.atan2(length,relative.w)/(2*h*length);
    var taskFirst=[(pp.x-pm.x)/(2*h),(pp.y-pm.y)/(2*h),(pp.z-pm.z)/(2*h),
      relative.x*angularScale,relative.y*angularScale,relative.z*angularScale];
    var taskSecond=[(pp.x-2*pa.x+pm.x)/(h*h),(pp.y-2*pa.y+pm.y)/(h*h),(pp.z-2*pa.z+pm.z)/(h*h),0.0,0.0,0.0];
    var derivatives=refiner.refinedDerivatives(0.02,at,pa,motionkit.path.OrientationPolicy.Fixed,taskFirst,taskSecond);
    for(j in 0...at.length){near(derivatives.first[j],j==0?0.75:0,"differential rates match the independently sampled FK path",1e-5);
      near(derivatives.second[j],0,"differential acceleration cancels Jacobian variation",1e-4);}
    var refinedPath=refiner.refinePath([for(i in 0...21)0.002*i],(distance)->{
      var q=start.copy();q[0]+=0.75*distance;var p=solver.forward(q);
      return new motionkit.robot.AnalyticPathRefiner.RefinementTarget(p,motionkit.path.OrientationPolicy.Fixed,
        [-0.75*p.y,0.75*p.x,0.0,0.0,0.0,0.75],[-0.5625*p.x,-0.5625*p.y,0.0,0.0,0.0,0.0]);
    });
    for(i in 0...refinedPath.s.length)for(j in 0...start.length){
      near(refinedPath.q[i][j],start[j]+(j==0?0.75*refinedPath.s[i]:0),"refined joint path retains the exact analytic curve",1e-6);
      near(refinedPath.qPrime[i][j],j==0?0.75:0,"refined joint path carries differential velocity",1e-6);
      near(refinedPath.qDoublePrime[i][j],0,"refined joint path carries differential curvature",1e-5);
    }
    var timed=new ToppraPathTiming().time(refinedPath,new PathTimingLimits([for(_ in start)1.0],[for(_ in start)2.0]));
    try {
      var duration=timed.trajectory.durationSeconds();check(duration>0 && Math.isFinite(duration),"refined path succeeds in one TOPP-RA timing pass");
      near(timed.distanceToTime(0.04),duration,"refined path end maps to timing end",1e-6);
      for(i in 0...101){var state=timed.trajectory.evaluate(duration*i/100);
        for(j in 0...start.length){check(Math.abs(state.velocities[j])<=1.00001,"timed refinement respects joint speed");
          check(Math.abs(state.accelerations[j])<=2.00001,"timed refinement respects joint acceleration");}}
      var exactLimits=new ValidationLimits(start.length,Int64.ofInt(0),Int64.ofInt(0));
      for(j in 0...start.length){var bound=fixture.arm.group.limitsOf(j);exactLimits.position(j,bound.lower,bound.upper);
        exactLimits.velocity(j,1.00001);exactLimits.acceleration(j,2.00001);}
      check(!timed.trajectory.validate(exactLimits).hasFailure(),"native extrema validation proves refined trajectory bounds");
      var finalState=timed.trajectory.evaluate(duration);
      for(j in 0...start.length)near(finalState.positions[j],end[j],"timed refinement reaches the complete path endpoint",1e-6);
    } catch(error:Dynamic){timed.releaseDistanceMap();timed.trajectory.dispose();throw error;}
    timed.releaseDistanceMap();timed.trajectory.dispose();
    var nativeCompileBefore=fixture.arm.numericSolveCount();
    var serialLimits=new ValidationLimits(start.length,Int64.ofInt(1),Int64.ofInt(0));
    var serialCompiler=new ProgramCompiler(solver,serialLimits,"task",
      [for(_ in start)1.0],[for(_ in start)2.0],[for(_ in start)20.0],
      StartTolerances.uniform(start.length,0.02,0.02,0.02),null,0.005,0.5,0.005,0.02,
      null,new motionkit.robot.StructuredJointPathPlanner(fixture.arm));
    var serialProgram=new MotionProgram([MotionOp.MoveL(solver.forward(end),"task",0.1,Blend.ExactStop)]);
    var serialCompiled=serialCompiler.compile(serialProgram,start,Int64.ofInt(920));
    check(serialCompiled.blocks[0].plans.length==1,"UR MoveL compiles through structured selection and differential refinement");
    serialCompiled.dispose();
    check(fixture.arm.numericSolveCount()==nativeCompileBefore,"structured UR compilation uses no numeric pose IK");
    var serialWorker=serialCompiler.forWorker();
    var serialWorkerPlan=serialWorker.compile(serialProgram,start,Int64.ofInt(921));
    check(serialWorkerPlan.blocks[0].plans.length==1,"UR worker compiles through independent structured planning");
    serialWorkerPlan.dispose();
    var coneStartPose=solver.forward(start),coneEndPose=solver.forward(end);
    var coneAxis=fixture.arm.tcpPose(start).transformVector(new Vec3(0,0,1)).toArray();
    var authoredCone=new motionkit.path.PosePath("task",[new PoseLine(
      new PoseWaypoint(coneStartPose,1e-6,1e-6),new PoseWaypoint(coneEndPose,1e-6,1e-6),
      motionkit.path.OrientationPolicy.Cone(coneAxis,0.1),0.1,0.1)]);
    var coneCompiler=new ProgramCompiler(solver,serialLimits,"task",
      [for(_ in start)1.0],[for(_ in start)2.0],[for(_ in start)20.0],
      StartTolerances.uniform(start.length,0.02,0.02,0.02),null,0.005,0.5,0.005,0.02,
      null,new motionkit.robot.StructuredJointPathPlanner(fixture.arm,
        new motionkit.robot.CandidateProblem.CandidateSamplingOptions(4,1,4)));
    var coneCompiled=coneCompiler.compile(new MotionProgram([MotionOp.FollowPath(authoredCone,"task",0.1,[])]),
      start,Int64.ofInt(922));
    check(coneCompiled.blocks[0].plans.length==1,"UR cone path compiles through projected-centre refinement and timing");
    coneCompiled.dispose();
    check(fixture.arm.numericSolveCount()==nativeCompileBefore,"structured cone compilation uses no numeric pose IK");
    function cornerQ(distance:Float):Array<Float> {
      var q=start.copy();q[0]+=0.75*distance-0.1*0.02*0.02+(distance<0.02?0.1*(distance-0.02)*(distance-0.02):0);return q;
    }
    var cornerRequest=new PathRequest([0.0,0.02,0.04],[for(s in [0.0,0.02,0.04])solver.forward(cornerQ(s))],start,
      new IkTolerance(1e-6,1e-6),request.maxJump,request.velocity);
    var cornerProblem=new motionkit.robot.CandidateProblem(fixture.arm,cornerRequest);
    var cornerRefiner=new motionkit.robot.AnalyticPathRefiner(fixture.arm,cornerProblem,motionkit.robot.StructuredLadder.search(cornerProblem));
    var cornerPath=cornerRefiner.refinePath([for(i in 0...21)0.002*i],(distance)->{
      var p=solver.forward(cornerQ(distance)),rate=0.75+(distance<0.02?0.2*(distance-0.02):0);
      var outgoing=distance<0.02?0.2:0.0,incoming=distance<=0.02?0.2:0.0;
      function acceleration(a:Float):Array<Float> return [-rate*rate*p.x-a*p.y,-rate*rate*p.y+a*p.x,0.0,0.0,0.0,a];
      return new motionkit.robot.AnalyticPathRefiner.RefinementTarget(p,motionkit.path.OrientationPolicy.Fixed,
        [-rate*p.y,rate*p.x,0.0,0.0,0.0,rate],acceleration(outgoing),acceleration(incoming));
    });
    near(cornerPath.qDoublePrime[10][0],0,"refined path preserves outgoing curvature",1e-5);
    near(cornerPath.qDoublePrimeBefore[10][0],0.2,"refined path preserves incoming curvature at a C1 knot",1e-5);
    near(cornerPath.qPrime[10][0],0.75,"curvature break retains its continuous tangent",1e-6);
    var spinTargets=[for(p in request.poses){var rotation=new Quat(p.qx,p.qy,p.qz,p.qw).multiply(Quat.fromAxisAngle(new Vec3(0,0,1),0.17));
      new Pose3(p.x,p.y,p.z,rotation.x,rotation.y,rotation.z,rotation.w);}];
    var spinRequest=new PathRequest(request.distances,spinTargets,start,new IkTolerance(1e-6,1e-6),request.maxJump,request.velocity,1,
      [for(_ in spinTargets)motionkit.path.OrientationPolicy.FreeAboutTool]);
    var spinProblem=new motionkit.robot.CandidateProblem(fixture.arm,spinRequest);
    var spinSelected=motionkit.robot.StructuredLadder.search(spinProblem);
    check(spinSelected.diagnostic==null,"free-spin refinement fixture has a selected path");
    var spinRefiner=new motionkit.robot.AnalyticPathRefiner(fixture.arm,spinProblem,spinSelected);
    previous=start.copy();
    for(i in 0...11){var q=start.copy();q[0]+=0.003*i;var p=solver.forward(q);
      var rotation=new Quat(p.qx,p.qy,p.qz,p.qw).multiply(Quat.fromAxisAngle(new Vec3(0,0,1),0.17));
      var target=new Pose3(p.x,p.y,p.z,rotation.x,rotation.y,rotation.z,rotation.w);
      var refined=spinRefiner.sample(0.004*i,target,motionkit.path.OrientationPolicy.FreeAboutTool,previous);
      near(motionkit.robot.ToolFreedom.orientationError(solver.forward(refined.q),target,motionkit.path.OrientationPolicy.FreeAboutTool),0,
        "smoothed free roll remains within task freedom",1e-6);
      if(i==0)for(j in 0...q.length)near(refined.q[j],start[j],"refinement recovers pinned spin from actual FK",1e-6);
      for(j in 0...q.length)check(Math.abs(refined.q[j]-previous[j])<request.maxJump[j],"free-roll refinement stays continuous");
      previous=refined.q;
    }
    var excluded=motionkit.robot.StructuredLadder.search(problem,null,0,(sample,candidate)->sample==1 ? Math.POSITIVE_INFINITY : 0.0);
    check(excluded.failedSample==1 && excluded.failedDistance==request.distances[1] && excluded.diagnostic!=null,
      "Haxe native ladder reports disabled-state disconnection at its sample distance");
    check(fixture.arm.numericSolveCount()==before,"native structured selection makes no numeric IK calls");
    var free = new motionkit.robot.CandidateProblem(fixture.arm,request,
      new motionkit.robot.CandidateProblem.CandidateSamplingOptions(4,1,4,false));
    check(free.samples[0].candidates.length > 1,"free-start candidate problem retains the full first layer");
    var toolAxis=fixture.arm.tcpPose(start).transformVector(new Vec3(0,0,1)).toArray();
    var coneRequest=new PathRequest([0.0],[solver.forward(start)],start,new IkTolerance(1e-6,1e-6),
      [for(_ in start)0.5],[for(_ in start)1.0],1,[motionkit.path.OrientationPolicy.Cone(toolAxis,0.1)]);
    var coneProblem=new motionkit.robot.CandidateProblem(fixture.arm,coneRequest,
      new motionkit.robot.CandidateProblem.CandidateSamplingOptions(1,1,1));
    check(coneProblem.samples[0].candidates.length==1,"pinned cone start preserves its exact spin beyond the orientation grid");
    for(joint in 0...6)near(coneProblem.samples[0].candidates[0].q[joint],start[joint],"pinned cone start remains exact",1e-12);
    problem.samples[1].candidates.resize(0);
    var empty=motionkit.robot.StructuredLadder.search(problem);
    check(empty.failedSample==1 && empty.diagnostic=="no candidates (unreachable or joint limits)",
      "Haxe native ladder distinguishes an empty candidate layer");


  }

  public function testOrientationDifferential():Void {
    for(angle in [0.01,0.06,0.1,1.2,3.0]) {
      var initial=Quat.fromAxisAngle(new Vec3(1,0,0),0.7);
      var finalRotation=Quat.fromAxisAngle(new Vec3(0,0,1),angle).multiply(initial);
      var a=new PoseWaypoint(new Pose3(0,0,0,initial.x,initial.y,initial.z,initial.w),1e-6,1e-6);
      var b=new PoseWaypoint(new Pose3(1,0,0,finalRotation.x,finalRotation.y,finalRotation.z,finalRotation.w),1e-6,1e-6);
      var line=new PoseLine(a,b,motionkit.path.OrientationPolicy.Interpolated,0.1,0.1),h=1e-4;
      for(distance in [0.1,0.3,0.5,0.7,0.9]) {
        var rates=line.derivativesAt(distance),p=line.waypointAt(distance).pose;
        var omega=poseRotationDelta(line.waypointAt(distance-h).pose,line.waypointAt(distance+h).pose,1/(2*h));
        var before=poseRotationDelta(line.waypointAt(distance-2*h).pose,p,1/(2*h));
        var after=poseRotationDelta(p,line.waypointAt(distance+2*h).pose,1/(2*h));
        var conePolicy=motionkit.path.OrientationPolicy.Cone([0.2,0.3,1.0],0.2);
        var projected=motionkit.robot.OrientationDifferential.cone(new Quat(p.qx,p.qy,p.qz,p.qw),rates.angular,rates.angularSecond,[0.2,0.3,1.0]);
        function projectedPose(s:Float):Pose3 return motionkit.robot.OrientationLattice.centre(line.waypointAt(s).pose,conePolicy);
        var centre=projectedPose(distance),centreMinus=projectedPose(distance-h),centrePlus=projectedPose(distance+h);
        var centreOmega=poseRotationDelta(centreMinus,centrePlus,1/(2*h));
        var centreBefore=poseRotationDelta(projectedPose(distance-2*h),centre,1/(2*h));
        var centreAfter=poseRotationDelta(centre,projectedPose(distance+2*h),1/(2*h));
        near(motionkit.path.PoseMath.angle(new Pose3(centre.x,centre.y,centre.z,projected.rotation.x,projected.rotation.y,
          projected.rotation.z,projected.rotation.w),centre),0,"cone differential uses the lattice centre rotation",1e-7);
        for(axis in 0...3){near(projected.velocity[axis],centreOmega[axis],"cone centre velocity matches independent quaternion differences",1e-7);
          near(projected.acceleration[axis],(centreAfter[axis]-centreBefore[axis])/(2*h),
            "cone centre acceleration matches independent quaternion differences",1e-5);}
        for(axis in 0...3){near(rates.angular[axis],omega[axis],"primitive angular velocity matches quaternion differences",1e-7);
          near(rates.angularSecond[axis],(after[axis]-before[axis])/(2*h),"primitive angular acceleration matches quaternion differences",1e-6);}
      }
    }
    function rotationAt(s:Float):Quat {
      var x=0.3+0.2*s-0.03*s*s,y=-0.2+0.1*s+0.04*s*s,tilt=Math.sqrt(x*x+y*y);
      var swing=new Quat(-y*Math.sin(tilt/2)/tilt,x*Math.sin(tilt/2)/tilt,0,Math.cos(tilt/2));
      return Quat.fromAxisAngle(new Vec3(0,0,1),0.2+0.4*s+0.05*s*s).multiply(swing)
        .multiply(Quat.fromAxisAngle(new Vec3(0,0,1),0.7-0.3*s+0.06*s*s));
    }
    function finiteOmega(s:Float):Array<Float> {
      var h=1e-5,relative=rotationAt(s+h).multiply(rotationAt(s-h).conjugate());
      var length=Math.sqrt(relative.x*relative.x+relative.y*relative.y+relative.z*relative.z);
      var factor=2*Math.atan2(length,relative.w)/(2*h*length);
      return [relative.x*factor,relative.y*factor,relative.z*factor];
    }
    for(i in 0...21){var s=i/20.0,centre=Quat.fromAxisAngle(new Vec3(0,0,1),0.2+0.4*s+0.05*s*s);
      var motion=motionkit.robot.OrientationDifferential.refine(centre,[0.0,0.0,0.4+0.1*s],[0.0,0.0,0.1],
        new motionkit.robot.RedundancySpline.SplineSample(0.3+0.2*s-0.03*s*s,0.2-0.06*s,-0.06),
        new motionkit.robot.RedundancySpline.SplineSample(-0.2+0.1*s+0.04*s*s,0.1+0.08*s,0.08),
        new motionkit.robot.RedundancySpline.SplineSample(0.7-0.3*s+0.06*s*s,-0.3+0.12*s,0.12));
      var expected=rotationAt(s),actual=motion.rotation;
      near(Math.abs(actual.x*expected.x+actual.y*expected.y+actual.z*expected.z+actual.w*expected.w),1,
        "orientation jet reproduces direct quaternion composition",1e-10);
      var omega=finiteOmega(s),h=1e-4,before=finiteOmega(s-h),after=finiteOmega(s+h);
      for(axis in 0...3){near(motion.velocity[axis],omega[axis],"analytic angular velocity matches quaternion differences",1e-7);
        near(motion.acceleration[axis],(after[axis]-before[axis])/(2*h),"analytic angular acceleration matches quaternion differences",1e-5);}
    }
    var zero=motionkit.robot.OrientationDifferential.refine(Quat.identity(),[0.0,0.0,0.0],[0.0,0.0,0.0],
      new motionkit.robot.RedundancySpline.SplineSample(0,0.2,0.1),new motionkit.robot.RedundancySpline.SplineSample(0,0.3,-0.2),
      new motionkit.robot.RedundancySpline.SplineSample(0,0.4,0.5));
    for(axis in 0...3){near(zero.velocity[axis],[-0.3,0.2,0.4][axis],"zero swing has finite exact angular rates",1e-12);
      near(zero.acceleration[axis],[0.28,0.22,0.5][axis],"zero swing includes swing-roll acceleration coupling",1e-12);}
  }

  public function testRedundancySpline():Void {
    var knots=[0.0,0.2,0.7,1.0],affine=new motionkit.robot.RedundancySpline(knots,[for(x in knots)0.3+0.7*x]);
    var periodic=new motionkit.robot.RedundancySpline(knots,[for(x in knots){var angle=3.0+0.4*x;angle>Math.PI ? angle-2*Math.PI : angle;}],2*Math.PI);
    for(i in 0...101){var x=i/100.0,a=affine.evaluate(x),r=periodic.evaluate(x);
      near(a.value,0.3+0.7*x,"redundancy spline reproduces affine external motion",1e-10);
      near(a.first,0.7,"affine external derivative",1e-10);near(a.second,0,"affine external curvature",1e-10);
      near(r.value,3.0+0.4*x,"period-aware roll stays continuous across its seam",1e-10);
      near(r.first,0.4,"period-aware roll derivative",1e-10);near(r.second,0,"period-aware roll curvature",1e-10);
    }
    var nonlinear=new motionkit.robot.RedundancySpline(knots,[0.0,0.4,-0.1,0.2]);
    for(i in 0...knots.length)near(nonlinear.evaluate(knots[i]).value,[0.0,0.4,-0.1,0.2][i],"spline interpolates each selected knot",1e-10);
    for(i in 1...100){var x=i/100.0,step=1e-5,at=nonlinear.evaluate(x),before=nonlinear.evaluate(x-step),after=nonlinear.evaluate(x+step);
      near(at.first,(after.value-before.value)/(2*step),"spline first derivative matches finite differences",1e-6);
      near(at.second,(after.first-before.first)/(2*step),"spline second derivative matches finite differences",0.002);
    }
    for(knot in [0.2,0.7]){var before=nonlinear.evaluate(knot-1e-9),after=nonlinear.evaluate(knot+1e-9);
      near(before.first,after.first,"redundancy spline is C1 at knots",1e-6);
      near(before.second,after.second,"redundancy spline is C2 at knots",1e-6);}
    var rejected=false;try new motionkit.robot.RedundancySpline([0.0,0.0],[0.0,1.0]) catch(_:Dynamic)rejected=true;
    check(rejected,"spline rejects coincident distances");
    rejected=false;try nonlinear.evaluate(-0.1) catch(_:Dynamic)rejected=true;check(rejected,"spline rejects extrapolation");
    rejected=false;try new motionkit.robot.RedundancySpline([-1e308,1e308],[0.0,0.0]) catch(_:Dynamic)rejected=true;
    check(rejected,"spline rejects an overflowing interval");
    var two=new motionkit.robot.RedundancySpline([0.0,1.0],[0.2,0.6]);
    near(two.evaluate(0.3).value,0.32,"two-knot spline remains linear",1e-12);
    near(two.evaluate(0.3).first,0.4,"two-knot spline has the exact derivative",1e-12);

  }

  public function testOrientationLattice():Void {
    var rotation = Quat.fromAxisAngle(new Vec3(1,2,3).normalized(),0.73);
    var target = new Pose3(0.4,-0.1,0.8,rotation.x,rotation.y,rotation.z,rotation.w);
    var ownAxis=rotation.rotate(new Vec3(0,0,1)).toArray();
    var kept=motionkit.robot.OrientationLattice.sample(target,motionkit.path.OrientationPolicy.Cone(ownAxis,0.2),1,1,1);
    near(motionkit.path.PoseMath.angle(kept[0].pose,target),0,"cone centre preserves the target spin when its axis already matches",1e-7);
    var almostOpposite=rotation.rotate(new Vec3(1e-5,0,-1).normalized()).toArray();
    var almostFlipped=motionkit.robot.OrientationLattice.centre(target,motionkit.path.OrientationPolicy.Cone(almostOpposite,0.0));
    near(motionkit.robot.ToolFreedom.orientationError(almostFlipped,target,motionkit.path.OrientationPolicy.Cone(almostOpposite,0.0)),0,
      "cone axis alignment remains accurate near the antipodal case",1e-7);
    var opposite=rotation.rotate(new Vec3(0,0,-1)).toArray();
    var flipped=motionkit.robot.OrientationLattice.centre(target,motionkit.path.OrientationPolicy.Cone(opposite,0.2));
    near(motionkit.robot.ToolFreedom.orientationError(flipped,target,motionkit.path.OrientationPolicy.Cone(opposite,0.2)),0,
      "cone centre handles the antipodal axis",1e-7);

    for (freedom in [motionkit.path.OrientationPolicy.Fixed,motionkit.path.OrientationPolicy.FreeAboutTool,
        motionkit.path.OrientationPolicy.Cone([0.3,0.4,0.5],0.4)]) {
      var cells = motionkit.robot.OrientationLattice.sample(target,freedom);
      var expected = switch freedom {case Fixed: 1; case FreeAboutTool: 12; default: 300;};
      check(cells.length == expected,"native orientation lattice count matches policy");
      var repeated = motionkit.robot.OrientationLattice.sample(target,freedom);
      for (i in 0...cells.length) {
        near(motionkit.robot.ToolFreedom.orientationError(cells[i].pose,target,freedom),0,
          "native orientation lattice satisfies ToolFreedom",1e-7);
        near(motionkit.path.PoseMath.distance(cells[i].pose,target),0,"orientation lattice preserves position",1e-12);
        check(cells[i].roll == repeated[i].roll && cells[i].tilt == repeated[i].tilt && cells[i].azimuth == repeated[i].azimuth,
          "orientation lattice coordinates are deterministic");
        near(motionkit.path.PoseMath.angle(cells[i].pose,repeated[i].pose),0,"orientation lattice pose is deterministic",1e-7);
      }
    }
  }

  public function testCartesianAnalyticIk():Void {
    var rollModel=new RobotModel("physical-roll-reroute");
    var rollBase=rollModel.addLink(new Link("base")),rollParent=rollBase;
    for(j in 0...4){var child=rollModel.addLink(new Link('roll-link-$j'));
      var joint=rollModel.addJoint(new Joint('roll-joint-$j',j<3?JointType.Prismatic:JointType.Revolute,rollParent,child));
      joint.axis=j<3?[for(k in 0...3)k==j?1.0:0.0]:[0.0,0.0,1.0];
      joint.limits.lower=j<3?-1:-Math.PI;joint.limits.upper=j<3?1:Math.PI;rollParent=child;}
    var rollTip=rollModel.addFrame(new Frame("tip",rollParent));
    var rollGroup=new robotkit.manipulation.KinematicGroup(rollModel,rollBase.id,rollTip.id);
    var rollStart=[0.0,0.0,0.0,0.0],rollEnd=[0.2,0.0,0.0,0.0];
    var rollSolver=new ManipulatorKinematics(rollGroup);
    var rollRequest=new PathRequest([0.0,0.2],[rollSolver.forward(rollStart),rollSolver.forward(rollEnd)],rollStart,
      new IkTolerance(1e-6,1e-6),[1.0,1.0,1.0,2.0],[1.0,1.0,1.0,1.0],32,
      [motionkit.path.OrientationPolicy.FreeAboutTool,motionkit.path.OrientationPolicy.FreeAboutTool]);
    var rollProblem=new motionkit.robot.CandidateProblem(rollGroup,rollRequest,
      new motionkit.robot.CandidateProblem.CandidateSamplingOptions(4,1,1));
    function rollHull(x:Float):Array<Float>{var vertices:Array<Float> = [];
      for(dx in [-0.01,0.01])for(y in [-0.01,0.01])for(z in [-0.01,0.01]){
        vertices.push(x+dx);vertices.push(y);vertices.push(z);}
      return vertices;}
    var rollWorld=new robotkit.manipulation.ArmClearance(rollGroup,[
      {name:"offset-tool",link:rollParent.id,vertices:rollHull(0.1),tool:true},
      {name:"post",link:rollBase.id,vertices:rollHull(0.3),tool:false}],rollStart);
    var unobstructed=motionkit.robot.StructuredLadder.search(rollProblem);
    check(rollWorld.violation(unobstructed.candidates[1].q)!=null,"physical post blocks the cheapest roll route");
    var avoided=motionkit.robot.LazyCollisionLadder.select(rollProblem,rollWorld,3);
    check(avoided.diagnostic==null && Math.abs(avoided.candidates[1].q[3])>1,
      "physical obstacle forces a native tool-roll change within the round budget");
    check(rollWorld.sweep(avoided.candidates[0].q,avoided.candidates[1].q)==null,
      "physical rerouted roll transition has no sampled clearance violation");
    near(motionkit.path.PoseMath.distance(rollSolver.forward(avoided.candidates[1].q),rollRequest.poses[1]),0,
      "physical roll rerouting retains the TCP position",1e-8);
    check(avoided.closestClearance!=null,"physical reroute returns its closest sampled clearance");
    if(avoided.closestClearance!=null)check(avoided.closestClearance.distance>=avoided.closestClearance.required,
      "physical reroute clearance exceeds the required margin");
    var rollPath=new motionkit.path.PosePath("task",[new PoseLine(
      new PoseWaypoint(rollRequest.poses[0],1e-6,1e-6),new PoseWaypoint(rollRequest.poses[1],1e-6,1e-6),
      motionkit.path.OrientationPolicy.FreeAboutTool,0.1,0.1)]);
    var physicalPlanner=new motionkit.robot.StructuredJointPathPlanner(rollGroup,
      new motionkit.robot.CandidateProblem.CandidateSamplingOptions(4,1,1),null,rollWorld,3);
    var physicalCurve=physicalPlanner.plan(rollPath,rollRequest);
    check(Math.abs(physicalCurve.q[1][3])>1,"integrated planner refines the physical obstacle-avoiding roll");
    check(rollWorld.sweep(physicalCurve.q[0],physicalCurve.q[1])==null,"integrated refined route has a clear sampled sweep");
    var blockedWorld=new robotkit.manipulation.ArmClearance(rollGroup,[
      {name:"offset-tool",link:rollParent.id,vertices:rollHull(0.1),tool:true},
      {name:"blocking-post",link:rollBase.id,vertices:rollHull(0.1),tool:false}],rollStart);
    var blockage="";
    try {motionkit.robot.LazyCollisionLadder.select(rollProblem,blockedWorld,3);}catch(error:Dynamic){blockage=Std.string(error);}
    check(blockage.indexOf("sample 0")>=0 && blockage.indexOf("offset-tool")>=0 && blockage.indexOf("blocking-post")>=0,
      "impossible physical pinned start reports both blocking bodies and sample");
    var clearanceModel=new RobotModel("closest-clearance");
    var fixedLink=clearanceModel.addLink(new Link("fixed")),movingLink=clearanceModel.addLink(new Link("moving"));
    var slide=clearanceModel.addJoint(new Joint("slide",JointType.Prismatic,fixedLink,movingLink));
    slide.axis=[1.0,0.0,0.0];slide.limits.lower=-2;slide.limits.upper=2;
    var tip=clearanceModel.addFrame(new Frame("tip",movingLink));
    var clearanceGroup=new robotkit.manipulation.KinematicGroup(clearanceModel,fixedLink.id,tip.id);
    function cube(x:Float):Array<Float> {
      var vertices:Array<Float> = [];
      for(dx in [-0.01,0.01])for(y in [-0.01,0.01])for(z in [-0.01,0.01]){vertices.push(x+dx);vertices.push(y);vertices.push(z);}
      return vertices;
    }
    var world=new robotkit.manipulation.ArmClearance(clearanceGroup,[
      {name:"tool",link:movingLink.id,vertices:cube(0),tool:true},
      {name:"far",link:fixedLink.id,vertices:cube(1.5),tool:false},
      {name:"near",link:fixedLink.id,vertices:cube(1),tool:false}],[0.0]);
    var closest=world.closest([0.0]);check(closest!=null,"closest clearance reports clear pairs");
    if(closest!=null){near(closest.distance,0.98,"closest clearance measures the nearest hull gap",1e-8);
      check(closest.a=="near" || closest.b=="near","closest clearance retains the nearest body pair");}
    var moved=world.closest([0.5],true);if(moved!=null){near(moved.distance,0.48,"closest clearance follows moving geometry",1e-8);
      near(moved.required,robotkit.manipulation.ArmClearance.CONTACT_MARGIN,"closest clearance retains contact margin",1e-12);}
    var sweepClosest=world.closestSweep([0.0],[1.2],false,0.02);
    check(sweepClosest!=null,"closest sweep reports a checked hull pair");
    if(sweepClosest!=null)near(sweepClosest.distance,0,"closest sweep finds an interior collision between clear endpoints",1e-8);
    var clearSweep=world.closestSweep([0.0],[0.5]);
    if(clearSweep!=null)near(clearSweep.distance,0.48,"closest sweep aggregates clear route distances",1e-8);
    throws(function() world.closestSweep([0.0],[0.5],false,Math.POSITIVE_INFINITY),"closest sweep rejects an infinite sampling step");
    var emptyWorld=new robotkit.manipulation.ArmClearance(clearanceGroup,[],[0.0]);
    check(emptyWorld.closest([0.0])==null,"empty clearance world has no closest pair");
    function waypoint(x:Float,y:Float):PoseWaypoint return new PoseWaypoint(new Pose3(x,y,0),1e-6,1e-6);
    var line=new PoseLine(waypoint(-1,0),waypoint(0,0),motionkit.path.OrientationPolicy.Fixed,0.1,0.1);
    var arc=new motionkit.path.PoseArc(waypoint(0,0),waypoint(Math.sqrt(0.5),1-Math.sqrt(0.5)),
      waypoint(1,1),motionkit.path.OrientationPolicy.Fixed,0.1);
    var joined=new motionkit.robot.PosePathRefinement(new motionkit.path.PosePath("task",[line,arc]));
    var knot=joined.at(line.length());
    near(knot.velocity[0],1,"line/arc refinement retains the shared tangent",1e-6);
    near(knot.acceleration[1],1,"line/arc refinement selects outgoing curvature",1e-4);
    near(knot.accelerationBefore[1],0,"line/arc refinement retains incoming line curvature",1e-6);
    var corner=new PoseLine(waypoint(0,0),waypoint(0,1),motionkit.path.OrientationPolicy.Fixed,0.1,0.1);
    var discontinuous=new motionkit.robot.PosePathRefinement(new motionkit.path.PosePath("task",[line,corner]));
    throws(function() discontinuous.at(line.length()),"refinement rejects an unblended task-velocity corner");
    throws(function() joined.at(-0.01),"refinement rejects distance before the authored path");
    for (count in 3...6) {
      var model = new RobotModel('analytic-cartesian-$count');
      var base = model.addLink(new Link("cartesian-base")), parent = base;
      for (index in 0...count) {
        var child = model.addLink(new Link('cartesian-link-$index'));
        var joint = model.addJoint(new Joint('cartesian-joint-$index',
          index < 3 ? JointType.Prismatic : JointType.Revolute, parent, child));
        joint.axis = index < 3 ? [for (axis in 0...3) axis == index ? 1.0 : 0.0]
          : index == 3 ? [0.0, 0.0, 1.0] : [0.0, 1.0, 0.0];
        if (index == 0) {
          var yaw = Quat.fromAxisAngle(new Vec3(0, 0, 1), 0.37);
          joint.parentFrameRotation = [yaw.x, yaw.y, yaw.z, yaw.w];
        }
        joint.parentFramePosition = [0.03 * index, -0.02 * index, 0.04 * index];
        joint.limits.lower = index < 3 ? -2 : -2 * Math.PI;
        joint.limits.upper = index < 3 ? 2 : 2 * Math.PI;
        parent = child;
      }
      var flange = model.addFrame(new Frame("cartesian-flange", parent));
      flange.position = [0.04, 0.02, 0.07];
      var tool = new Transform3(new Vec3(0.08, -0.03, 0.14), Quat.fromAxisAngle(new Vec3(0, 1, 0), 0.4));
      var group = new robotkit.manipulation.KinematicGroup(model, base.id, flange.id, null, tool);
      var analytic:motionkit.robot.AnalyticIk = new motionkit.robot.CartesianAnalyticIk(group);
      var numeric = new ManipulatorKinematics(group);
      var sampler = new motionkit.robot.CartesianCandidateSampler(group);
      var before = group.numericSolveCount();
      check(analytic.jointCount() == count, "Cartesian family keeps model DOF count");
      var marked = new robotkit.manipulation.KinematicGroup(model,base.id,flange.id,null,tool,null,["cartesian-joint-0"]);
      check(marked.external[0],"Cartesian regression fixture marks a leading axis external");
      var markedQ = [for(i in 0...count)0.2*Math.sin(i+0.3)];
      var markedTarget = new ManipulatorKinematics(marked).forward(markedQ);
      var markedSampler = new motionkit.robot.CartesianCandidateSampler(marked);
      var markedCandidates = markedSampler.sample(markedTarget,markedQ,motionkit.path.OrientationPolicy.Fixed);
      check(markedCandidates.length>0,"complete Cartesian chain exports despite external markers");
      for(candidate in markedCandidates)near(motionkit.path.PoseMath.distance(new ManipulatorKinematics(marked).forward(candidate.q),markedTarget),0,
        "marked Cartesian native candidates satisfy compiled FK",1e-7);
      var markedRequest = new PathRequest([0.0],[markedTarget],markedQ,new IkTolerance(1e-6,1e-6),
        [for(_ in markedQ)0.5],[for(_ in markedQ)1.0]);
      var markedProblem = new motionkit.robot.CandidateProblem(marked,markedRequest);
      check(markedProblem.externalJoints.length==0 && markedProblem.samples[0].candidates.length==1,
        "Cartesian task axes are solved geometrically rather than sampled independently");
      var refinedEnd = markedQ.copy();refinedEnd[0] += 0.02;
      var refineRequest = new PathRequest([0.0,0.04],[markedTarget,new ManipulatorKinematics(marked).forward(refinedEnd)],
        markedQ,new IkTolerance(1e-6,1e-6),[for(_ in markedQ)0.5],[for(_ in markedQ)1.0]);
      var refineProblem = new motionkit.robot.CandidateProblem(marked,refineRequest);
      var route = motionkit.robot.StructuredLadder.search(refineProblem);
      check(route.diagnostic==null,"Cartesian refinement has a complete selected route");
      var refiner = new motionkit.robot.AnalyticPathRefiner(marked,refineProblem,route);
      var delta = new ManipulatorKinematics(marked).forward(refinedEnd);
      var velocity = [(delta.x-markedTarget.x)/0.04,(delta.y-markedTarget.y)/0.04,(delta.z-markedTarget.z)/0.04,0.0,0.0,0.0];
      var refined = refiner.refinePath([for(i in 0...11)0.004*i],distance -> {
        var q=markedQ.copy();q[0]+=0.5*distance;
        return new motionkit.robot.AnalyticPathRefiner.RefinementTarget(new ManipulatorKinematics(marked).forward(q),
          motionkit.path.OrientationPolicy.Fixed,velocity,[for(_ in 0...6)0.0]);
      });
      for(i in 0...11)for(j in 0...count) {
        near(refined.q[i][j],markedQ[j]+(j==0?0.002*i:0),"Cartesian refinement preserves geometric task axes",1e-7);
        near(refined.qPrime[i][j],j==0?0.5:0,"Cartesian refinement solves task derivatives",1e-7);
        near(refined.qDoublePrime[i][j],0,"Cartesian straight refinement has zero curvature",1e-6);
      }
      var authoredLine=new PoseLine(new PoseWaypoint(markedTarget,1e-6,1e-6),new PoseWaypoint(delta,1e-6,1e-6),
        motionkit.path.OrientationPolicy.Fixed,0.1,0.1);
      var authoredPath=new motionkit.path.PosePath("task",[authoredLine]);
      var authoredRequest=new PathRequest([0.0,authoredPath.length()],[markedTarget,delta],markedQ,
        new IkTolerance(1e-6,1e-6),[for(_ in markedQ)0.5],[for(_ in markedQ)1.0]);
      var authoredProblem=new motionkit.robot.CandidateProblem(marked,authoredRequest);
      var authoredRefiner=new motionkit.robot.AnalyticPathRefiner(marked,authoredProblem,motionkit.robot.StructuredLadder.search(authoredProblem));
      var provider=new motionkit.robot.PosePathRefinement(authoredPath);
      var authoredSamples=authoredRefiner.refinePath([for(i in 0...11)authoredPath.length()*i/10],provider.at);
      var plannedDistances=[for(i in 0...11)authoredPath.length()*i/10];
      var plannedRequest=new PathRequest(plannedDistances,[for(s in plannedDistances)authoredPath.poseAt(s)],markedQ,
        new IkTolerance(1e-6,1e-6),[for(_ in markedQ)0.5],[for(_ in markedQ)1.0]);
      var planner:motionkit.robot.JointPathPlanner=new motionkit.robot.StructuredJointPathPlanner(marked);
      var plannedSamples=planner.plan(authoredPath,plannedRequest);
      for(i in 0...11)for(j in 0...count) {
        near(plannedSamples.q[i][j],authoredSamples.q[i][j],"joint planner composes selection and refinement",1e-7);
        near(plannedSamples.qPrime[i][j],authoredSamples.qPrime[i][j],"joint planner returns timing-ready derivatives",1e-7);
      }
      for(i in 0...11)for(j in 0...count) {
        near(authoredSamples.q[i][j],markedQ[j]+(j==0?0.002*i:0),"authored pose path drives analytic refinement",1e-7);
        near(authoredSamples.qPrime[i][j],j==0?1.0:0,"authored primitive supplies task rates",1e-6);
      }
      var plannerSolver=new ManipulatorKinematics(marked);
      var plannerLimits=new ValidationLimits(count,Int64.ofInt(1),Int64.ofInt(0));
      var plannerCompiler=new ProgramCompiler(plannerSolver,plannerLimits,"task",
        [for(_ in markedQ)1.0],[for(_ in markedQ)2.0],[for(_ in markedQ)20.0],
        StartTolerances.uniform(count,0.02,0.02,0.02),null,0.01,0.5,0.005,0.02,
        null,new motionkit.robot.StructuredJointPathPlanner(marked));
      var plannerProgram=new MotionProgram([MotionOp.MoveL(delta,"task",0.1,Blend.ExactStop)]);
      var compiledPlanner=plannerCompiler.compile(plannerProgram,markedQ,Int64.ofInt(910));
      check(compiledPlanner.blocks[0].plans.length==1,"compiler consumes structured timing-ready joint path");
      compiledPlanner.dispose();
      var workerCompiler=plannerCompiler.forWorker();
      check(workerCompiler.jointPathPlanner!=plannerCompiler.jointPathPlanner,"worker compiler rebuilds its structured planner");
      var workerPlan=workerCompiler.compile(plannerProgram,markedQ,Int64.ofInt(911));
      check(workerPlan.blocks[0].plans.length==1,"worker compiler plans with independent analytic kinematics");
      workerPlan.dispose();
      if(count>3) {
        var opposite=markedQ.copy();opposite[3]+=Math.PI;
        var oppositePose=new ManipulatorKinematics(marked).forward(opposite);
        var centreX=(markedTarget.x+oppositePose.x)/2,centreY=(markedTarget.y+oppositePose.y)/2;
        var rotaryEnd=markedQ.copy();rotaryEnd[3]+=0.5*0.04+0.2*0.04*0.04;
        var rotaryRequest=new PathRequest([0.0,0.04],[markedTarget,new ManipulatorKinematics(marked).forward(rotaryEnd)],
          markedQ,new IkTolerance(1e-6,1e-6),[for(_ in markedQ)0.5],[for(_ in markedQ)1.0]);
        var rotaryProblem=new motionkit.robot.CandidateProblem(marked,rotaryRequest);
        var rotaryRoute=motionkit.robot.StructuredLadder.search(rotaryProblem);
        check(rotaryRoute.diagnostic==null,"Cartesian rotary refinement has a connected route");
        var rotaryRefiner=new motionkit.robot.AnalyticPathRefiner(marked,rotaryProblem,rotaryRoute);
        var rotaryPath=rotaryRefiner.refinePath([for(i in 0...11)0.004*i],distance -> {
          var q=markedQ.copy();q[3]+=0.5*distance+0.2*distance*distance;
          var p=new ManipulatorKinematics(marked).forward(q),rate=0.5+0.4*distance;
          var x=p.x-centreX,y=p.y-centreY;
          return new motionkit.robot.AnalyticPathRefiner.RefinementTarget(p,motionkit.path.OrientationPolicy.Fixed,
            [-rate*y,rate*x,0.0,0.0,0.0,rate],
            [-rate*rate*x-0.4*y,-rate*rate*y+0.4*x,0.0,0.0,0.0,0.4]);
        });
        for(i in 0...11)for(j in 0...count) {
          var distance=rotaryPath.s[i];
          near(rotaryPath.q[i][j],markedQ[j]+(j==3?0.5*distance+0.2*distance*distance:0),
            "Cartesian rotary refinement preserves the offset-tool curve",1e-7);
          near(rotaryPath.qPrime[i][j],j==3?0.5+0.4*distance:0,"Cartesian rotary differential velocity",1e-7);
          near(rotaryPath.qDoublePrime[i][j],j==3?0.4:0,"Cartesian rotary differential curvature",1e-6);
        }
      }
      for (sample in 0...40) {
        var q = [for (joint in 0...count) 1.4 * Math.sin((sample + 1) * (joint + 1) * 1.618)];
        var target = numeric.forward(q);
        var axis = group.tcpPose(q).transformVector(new Vec3(0,0,1)).toArray();
        for (policy in [motionkit.path.OrientationPolicy.Fixed,motionkit.path.OrientationPolicy.FreeAboutTool,
            motionkit.path.OrientationPolicy.Cone(axis,0.1)]) {
          var candidates = sampler.sample(target,q,policy,4,1,4), found = false;
          for (candidate in candidates) {
            var same = true;
            for (joint in 0...count) if (Math.abs(candidate.q[joint]-q[joint]) > 1e-6) same = false;
            found = found || same;
            var actual = numeric.forward(candidate.q);
            near(motionkit.path.PoseMath.distance(actual,target),0,"combined native Cartesian sampler task position",1e-7);
            near(motionkit.robot.ToolFreedom.orientationError(actual,target,policy),0,"combined native Cartesian sampler freedom",1e-7);
            check(candidate.wraps.length == count && candidate.external.length == 0,"native candidate retains wrap coordinates");
          }
          check(found,"combined native Cartesian sampler retains original legal configuration");
        }
        check(group.numericSolveCount() == before,"combined Cartesian sampling uses no numeric IK");
        near(motionkit.path.PoseMath.distance(analytic.forward(q), target), 0, "Cartesian model-derived FK", 1e-7);
        for (freedom in [motionkit.path.OrientationPolicy.Fixed, motionkit.path.OrientationPolicy.FreeAboutTool]) {
          var answers = analytic.branches(target, q, freedom), original = false;
          check(answers.length > 0, "Cartesian analytic inverse returns a legal branch");
          for (answer in answers) {
            var same = true;
            for (joint in 0...count) {
              same = same && Math.abs(answer.q[joint] - q[joint]) < 1e-6;
              var bounds = group.group.limitsOf(joint);
              check(answer.q[joint] >= bounds.lower - 1e-9 && answer.q[joint] <= bounds.upper + 1e-9,
                "Every Cartesian periodic lift respects compiled limits");
            }
            original = original || same;
            var actual = numeric.forward(answer.q);
            near(motionkit.path.PoseMath.distance(actual, target), 0, "Cartesian branch preserves TCP position", 1e-7);
            near(motionkit.robot.ToolFreedom.orientationError(actual, target, freedom), 0,
              "Cartesian branch preserves permitted orientation", 1e-7);
          }
          check(original, "Cartesian analytic branches contain original model configuration");
        }
      }
    }
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
    for (pole in [0.0, Math.PI]) {
      var seed = [0.2,-0.3,0.4,0.5,pole,0.7];
      var before = arm.numericSolveCount();
      var request = new PathRequest([0.0],[solver.forward(seed)],seed,new IkTolerance(1e-6,1e-6),
        [for (_ in seed)0.5],[for (_ in seed)1.0]);
      var problem = new motionkit.robot.CandidateProblem(arm,request);
      check(problem.family == "OPW" && problem.samples[0].candidates.length == 1,
        "OPW wrist-pole path retains its pinned start");
      if (problem.samples[0].candidates.length == 1) {
        var candidate = problem.samples[0].candidates[0];
        check(candidate.singular != 0,"OPW pinned wrist-pole candidate reports singularity");
        for (joint in 0...6)near(candidate.q[joint],seed[joint],"OPW singular pinned joints remain exact",1e-12);
      }
      check(arm.numericSolveCount() == before,"OPW wrist-pole pinning uses no numeric IK");
    }
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
    var candidates = solver.sampleCandidates(reference, 8, new IkTolerance(), null);
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
    near(second[0], next[0], "structured ladder selects OPW branches natively across a path", 1e-6);
    var disconnected = "";
    try solver.solvePath(new PathRequest([0.0, 0.04], [solver.forward(q), solver.forward(next)], q,
      new IkTolerance(), [for (_ in 0...6) 1e-6], [for (_ in 0...6) 1.0]))
    catch (error:Dynamic) disconnected = Std.string(error);
    check(disconnected.indexOf("sample 1") >= 0 && disconnected.indexOf("0.04") >= 0,
      "OPW structured selection reports the first disconnected sample and distance");
    var limits = new ValidationLimits(6, Int64.ofInt(1), Int64.ofInt(0));
    var compiler = new ProgramCompiler(solver, limits, "work",
      [for (_ in 0...6) 2.0], [for (_ in 0...6) 4.0],
      [for (_ in 0...6) 20.0], StartTolerances.uniform(6, 0.02, 0.02, 0.02),
      null, 0.01, 0.5, 0.005, 0.02);
    check(Std.isOfType(compiler.jointPathPlanner, motionkit.robot.StructuredJointPathPlanner),
      "standalone OPW compiler selects the structured planner by default");
    var program = new MotionProgram([MotionOp.MoveL(solver.forward(next),
      "work", 0.1, Blend.ExactStop)]);
    var compiled = compiler.compile(program, q, Int64.ofInt(901));
    check(compiled.blocks[0].plans.length == 1,
      "OPW arm path compiles through the shared ProgramCompiler");
    compiled.dispose();
    var workerCompiler = compiler.forWorker();
    check(workerCompiler.jointPathPlanner != compiler.jointPathPlanner,
      "OPW worker receives its own structured planner");
    var workerPlan = workerCompiler.compile(program, q, Int64.ofInt(902));
    check(workerPlan.blocks[0].plans.length == 1,
      "OPW worker compiles through structured path refinement");
    workerPlan.dispose();
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
    // Equivalent joint frames with displaced axis origins and a placed q=0:
    // extraction must use axis lines rather than a particular frame convention.
    var placedOffsets = [0.2, 0.4, -0.7, 0.3, 0.6, -0.2];
    for (index in 0...6) {
      var joint = model.joints[index];
      var axis = Vec3.fromArray(joint.axis);
      var shift = axis.scale(0.013);
      joint.parentFramePosition = Vec3.fromArray(joint.parentFramePosition).add(shift).toArray();
      joint.childFramePosition = shift.toArray();
      var rotation = Quat.fromAxisAngle(axis, placedOffsets[index]);
      joint.parentFrameRotation = [rotation.x, rotation.y, rotation.z, rotation.w];
    }
    var placedArm = new Manipulator(model, links[0].id, flange.id);
    var placed = new OpwKinematics(model, placedArm);
    for (sample in 0...30) {
      var q = [for (joint in 0...6) 1.5 * Math.sin((sample + 1) * (joint + 1))];
      var target = new ManipulatorKinematics(placedArm).forward(q);
      var solved = placed.solvePose(target, q, new IkTolerance(1e-6, 1e-6));
      check(solved != null, "placed OPW frame convention has an inverse branch");
      var answer:Array<Float> = cast solved;
      for (joint in 0...6) near(answer[joint], q[joint], "placed OPW retains original branch", 1e-5);
    }
    var bad = buildContractArmFixture();
    model.joints[5].limits.lower = -4 * Math.PI;
    model.joints[5].limits.upper = 4 * Math.PI;
    var wrappedArm = new Manipulator(model, links[0].id, flange.id);
    var wrappedAnalytic = new OpwKinematics(model, wrappedArm);
    var shared:motionkit.robot.AnalyticIk = wrappedAnalytic;
    var wrappedQ = [0.2, -0.3, 0.4, -0.5, 0.6, 3 * Math.PI + 0.2];
    var wrappedTarget = new ManipulatorKinematics(wrappedArm).forward(wrappedQ);
    var foundOriginal = false;
    for (branch in shared.branches(wrappedTarget, wrappedQ)) {
      check(branch.branch >= 0 && branch.branch < 8, "OPW shared interface preserves native branch identity");
      var same = true;
      for (joint in 0...6) same = same && Math.abs(branch.q[joint] - wrappedQ[joint]) < 1e-5;
      foundOriginal = foundOriginal || same;
    }
    check(foundOriginal, "OPW enumerates legal periodic lifts beyond the nearest plus/minus turn");
    var singularQ = wrappedQ.copy();
    singularQ[4] = wrappedAnalytic.parameters.offsets[4] / wrappedAnalytic.parameters.signCorrections[4];
    var singularBranches = shared.branches(wrappedAnalytic.forward(singularQ), singularQ);
    check([for (branch in singularBranches) if (branch.singular) branch].length > 0,
      "OPW shared interface explicitly reports wrist singularity");
    var ur = new motionkit.robot.UrAnalyticIk(bad.arm);
    check(motionkit.robot.BranchIk.of(bad.arm).family() == "UR6R","model-derived family selection recognizes UR geometry");
    for (sample in 0...30) {
      var q = [for (joint in 0...6) 0.7 * Math.sin(sample * 0.37 + joint * 0.61)];
      var actual = bad.arm.tcpPose(q), predicted = ur.forward(q);
      near(actual.translation.sub(new Vec3(predicted.x,predicted.y,predicted.z)).norm(),0.0,"UR model-derived FK",1e-6);
      var candidates = ur.branches(predicted,q);
      check(candidates.length > 0,"UR model-derived inverse branches");
      var found = false;
      for (candidate in candidates) {
        var same = true;
        for (joint in 0...6) if (Math.abs(candidate.q[joint]-q[joint]) > 1e-5) same = false;
        found = found || same;
      }
      check(found,"UR inverse contains the original legal lift");
    }
    var placedUrFixture = buildContractArmFixture();
    for (index in 0...6) {
      var joint = placedUrFixture.model.joints[index];
      var axis = Vec3.fromArray(joint.axis), shift = axis.scale(0.013*(index+1));
      joint.parentFramePosition = Vec3.fromArray(joint.parentFramePosition).add(shift).toArray();
      joint.childFramePosition = shift.toArray();
      joint.parentFrameRotation = Quat.fromAxisAngle(axis,0.17*(index+1)).toArray();
    }
    var placedUrArm = new Manipulator(placedUrFixture.model,"base","flange");
    var placedUr = new motionkit.robot.UrAnalyticIk(placedUrArm);
    for (sample in 0...30) {
      var q = [for (joint in 0...6) 0.8*Math.sin(sample*0.39+joint*0.73)];
      var target = new ManipulatorKinematics(placedUrArm).forward(q);
      var found = false;
      for (candidate in placedUr.branches(target,q)) {
        var same = true;
        for (joint in 0...6) if (Math.abs(candidate.q[joint]-q[joint]) > 1e-5) same = false;
        found = found || same;
      }
      check(found,"UR extraction recovers placed joint references");
    }
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
    var blueprint = RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile());
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
    var runtime = harness.simulation.addRobot(RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile()));
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
    model.addActuator(new Actuator("left-motor", null, 1.0,
      Transmission.SimpleTransmission(first.id, 1.0, 0.0)));
    model.addActuator(new Actuator("right-motor", null, 1.0,
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
    var q = solver.solvePose(target, [0.0, 0.02, 0.0, 0.0], new IkTolerance(), null);
    check(q != null, "axis IK reaches a pose within logical limits");
    if (q == null) throw "axis IK returned no solution";
    near(q[0], 0.01, "axis IK maps the X leader");
    near(q[1], 0.005, "axis IK keeps the coupled follower in proportion");
    near(solver.forward(q).y, 0.02, "axis FK maps Y to the tool");
    check(solver.sampleCandidates(target, 8, new IkTolerance(), null).length == 1,
      "axis IK exposes one exact candidate");
    var velocity = solver.solveDifferential(q,
      new Twist6(0.02, 0.01, -0.01, 0.0, 0.0, 0.0), null, null);
    check(velocity != null, "axis differential IK maps a linear twist");
    if (velocity == null) throw "axis differential IK returned no solution";
    near(velocity[0], 0.02, "axis differential IK maps X velocity");
    near(velocity[1], -0.03,
      "axis differential IK keeps follower velocity in proportion");
    var path = new PosePath("work", [new PoseLine(new PoseWaypoint(target, 1e-6, 1e-6),
      new PoseWaypoint(new Pose3(0.03, 0.02, -0.03), 1e-6, 1e-6), OrientationPolicy.Fixed, 0.1, 0.1)]);
    var planner = new motionkit.robot.AxisJointPathPlanner(solver);
    var curve = planner.plan(path, new PathRequest([0.0, path.length()],
      [target, path.poseAt(path.length())], q, new IkTolerance(),
      [for (_ in 0...4) 0.1], [for (_ in 0...4) 0.1], 32,
      [OrientationPolicy.Fixed, OrientationPolicy.Fixed]));
    near(curve.q[1][1], -0.025, "axis path retains follower offset");
    near(curve.qPrime[1][0], 1.0, "axis path has exact leader derivative");
    near(curve.qPrime[1][1], -1.5, "axis path has exact follower derivative");
    near(curve.qDoublePrime[1][1], 0.0, "axis line has zero follower curvature");
    var freePath = new PosePath("work", [new PoseLine(new PoseWaypoint(target, 1e-6, 1e-6),
      new PoseWaypoint(new Pose3(0.03, 0.02, -0.03, 0.0, 0.0, Math.sin(0.2), Math.cos(0.2)), 1e-6, 1e-6),
      OrientationPolicy.Free, 0.1, 0.1)]);
    var freeCurve = planner.plan(freePath, new PathRequest([0.0, freePath.length()],
      [target, freePath.poseAt(freePath.length())], q, new IkTolerance(),
      [for (_ in 0...4) 0.1], [for (_ in 0...4) 0.1], 32,
      [OrientationPolicy.Free, OrientationPolicy.Free]));
    near(freeCurve.q[1][0], 0.03, "axis free-orientation path ignores authored rotation");
    var axisCompiler = new ProgramCompiler(solver, new ValidationLimits(4, Int64.ofInt(1), Int64.ofInt(0)), "work",
      [for (_ in 0...4) 0.1], [for (_ in 0...4) 0.4], [for (_ in 0...4) 4.0],
      StartTolerances.uniform(4, 0.001, 0.001, 0.001));
    check(Std.isOfType(axisCompiler.jointPathPlanner, motionkit.robot.AxisJointPathPlanner),
      "compiler defaults to the exact logical-axis planner");
    check(axisCompiler.forWorker().jointPathPlanner != axisCompiler.jointPathPlanner,
      "axis worker owns its path planner");
    var arcStart = new Pose3(0.02, 0.0, 0.0);
    var joined = new PosePath("work", [
      new PoseLine(new PoseWaypoint(new Pose3(0.02, -0.01, 0.0), 1e-6, 1e-6),
        new PoseWaypoint(arcStart, 1e-6, 1e-6), OrientationPolicy.Fixed, 0.1, 0.1),
      new motionkit.path.PoseArc(new PoseWaypoint(arcStart, 1e-6, 1e-6),
        new PoseWaypoint(new Pose3(0.02 / Math.sqrt(2.0), 0.02 / Math.sqrt(2.0), 0.0), 1e-6, 1e-6),
        new PoseWaypoint(new Pose3(0.0, 0.02, 0.0), 1e-6, 1e-6), OrientationPolicy.Fixed, 0.1)]);
    var distances = [0.0, joined.primitives[0].length(), joined.length()];
    var start = solver.solvePose(joined.poseAt(0.0), q, new IkTolerance(), OrientationPolicy.Fixed);
    var joinedCurve = planner.plan(joined, new PathRequest(distances,
      [for (s in distances) joined.poseAt(s)], start, new IkTolerance(),
      [for (_ in 0...4) 0.1], [for (_ in 0...4) 0.1], 32,
      [for (_ in distances) OrientationPolicy.Fixed]));
    near(joinedCurve.qDoublePrime[1][0], -50.0, "axis arc preserves exact outgoing curvature", 1e-6);
    near(joinedCurve.qDoublePrime[1][1], 75.0, "axis arc scales follower curvature", 1e-6);
    near(joinedCurve.qDoublePrimeBefore[1][1], 0.0, "axis join retains incoming line curvature");
    var invalidStart = q.copy(); invalidStart[1] += 0.001;
    var failure = "";
    try planner.plan(path, new PathRequest([0.0, path.length()],
      [target, path.poseAt(path.length())], invalidStart, new IkTolerance(),
      [for (_ in 0...4) 0.1], [for (_ in 0...4) 0.1], 32,
      [OrientationPolicy.Fixed, OrientationPolicy.Fixed]))
    catch (error:Dynamic) failure = Std.string(error);
    check(failure.indexOf("joint mapping") >= 0, "axis planner rejects an inconsistent pinned follower");
    check(solver.solvePose(new Pose3(0.06), q, new IkTolerance(), null) == null,
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
