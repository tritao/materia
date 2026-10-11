package app;

import robotkit.collision.CollisionClearance;

import robotkit.world.RobotWorld;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.EaikKinematics;
import motionkit.kinematics.Pose3;
import motionkit.path.PoseMath;

@:access(app.ProjectSourceTests)
@:access(app.MissionPlayer)
class OverheadArmTests {
  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
  }

  public static function run(root:String):Void {
    var manifest = sys.FileSystem.fullPath(root + "/machinekit/examples/overhead-arm/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    // Like G17, handling needs MuJoCo's compound-body contacts: the table is
    // part of the fixed gantry body, rather than an independent box obstacle.
    for (backend in [ApplicationSimulation.MUJOCO]) {
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        check(simulation.rebuild(session.sensors, session.scene, session), "Overhead arm builds: " + simulation.error);
        var mission = simulation.missionPlayer();
        if (mission == null) throw "Overhead arm has no mission";
        var channel = ProjectSourceTests.singleHandlingChannel(mission);
        var runner = ProjectSourceTests.handlingRunner(mission, channel);
        var solver:ManipulatorKinematics = cast runner.motion.compiler.solver;
        var indices = runner.motion.jointIndices;
        check(indices.length == 9, "XYZ gantry and six-axis arm share one logical motion group");
        function axis(name:String):Int {
          for (index in 0...indices.length)
            if (mission.robot.model.joints[indices[index]].name == name) return index;
          throw 'Overhead arm lacks external axis $name';
        }
        var xyz = [axis("x"), axis("y"), axis("z")];
        check([for (external in solver.manipulator.external) if (external) external].length == 3,
          "The planner derives all three external axes");
        for (index in xyz) check(solver.manipulator.external[index], "Gantry translations are external coordinates");
        // Check the entire tabletop, including its 100 mm border beyond the
        // gantry travel. Each station holds its external axes during handling.
        var definition = generated.assemblyDefinition;
        if (definition == null) throw "Overhead arm lacks its mechanical definition";
        var state = new cadkit.modeling.AssemblyState(definition);
        var pick = state.worldConnector("padPick", "top");
        var workY = pick.y / 1000;
        var inverse = new EaikKinematics(solver.manipulator);
        var reference = solver.forward(runner.home);
        var reachChecks = 0;
        for (x in [-0.1, 0.75, 1.5, 2.25, 3.1]) for (y in [-0.1, 0.75, 1.6]) {
          var seed = runner.home.copy();
          seed[xyz[0]] = Math.max(0.0, Math.min(3.0, x));
          seed[xyz[1]] = Math.max(0.0, Math.min(1.5, y));
          seed[xyz[2]] = 0.15;
          var target = new Pose3(x, workY + y, pick.z / 1000 + 0.05,
            reference.qx, reference.qy, reference.qz, reference.qw);
          var reached = inverse.solvePose(target, seed, runner.motion.compiler.ikTolerance);
          check(reached != null, 'The inverted arm reaches tabletop station ($x,$y)');
          if (reached == null) throw "Missing table reach witness";
          check(PoseMath.distance(solver.forward(reached), target) < 1e-8, "Analytic reach agrees with compiled FK");
          for (index in 0...reached.length) {
            var limits = solver.manipulator.group.limitsOf(index);
            check(reached[index] >= limits.lower && reached[index] <= limits.upper, "Table reach respects physical joint limits");
          }
          reachChecks++;
        }
        var lastDone = 0;
        while (!mission.finished && simulation.activeSession().simulationTime() < 600) {
          simulation.step();
          check(mission.failure == null, "Overhead positioning mission: " + mission.failure);
          if (mission.completed > lastDone) {
            lastDone = mission.completed;
            Sys.println('overhead arm ($backend): positioning step $lastDone at ${simulation.activeSession().simulationTime()} s');
          }
        }
        check(mission.finished && mission.completed == 6, "Both overhead table stations complete");
        var work = [for (part in simulation.capturePresentationSnapshot().environment) if (part.id == "project:workpiece") part];
        check(work.length == 1 && Math.abs(work[0].position[0] - 2.8) < 0.005 &&
          Math.abs(work[0].position[1] - workY - 1.5) < 0.005, "The carried part reaches the far table corner");
        var make = mission.newHandlingByChannel.get(channel);
        if (make == null) throw "Overhead arm cannot create coordinated motion";
        coordinated(simulation, mission, make().motion, xyz, channel, backend);
        Sys.println('overhead arm ($backend): $reachChecks table reach witnesses, two stations, checked coordinated XYZ motion');
      } catch (error:Dynamic) { simulation.clear(); session.dispose(); throw error; }
      simulation.clear(); session.dispose();
    }
  }

  static function coordinated(simulation:ApplicationSimulation, mission:MissionPlayer,
      motion:motionkit.robot.ManipulatorMotion, xyz:Array<Int>, channel:String, backend:Int):Void {
    var solver:ManipulatorKinematics = cast motion.compiler.solver;
    var entry = [for (_ in 0...motion.jointIndices.length) 0.0];
    entry[xyz[0]] = 0.2; entry[xyz[1]] = 0.1; entry[xyz[2]] = 0.05;
    motion.run(new motionkit.program.MotionProgram([
      motionkit.program.MotionOp.MoveJ(motionkit.program.MoveTarget.JointTarget(entry),
        new motionkit.MotionOptions(), motionkit.program.Blend.ExactStop)]));
    var deadline = simulation.activeSession().simulationTime() + 180;
    while (!motion.completed && motion.failure == null && simulation.activeSession().simulationTime() < deadline) {
      motion.update(simulation.timestep); simulation.step();
    }
    check(motion.completed && motion.failure == null, "Overhead coordinated entry: " + motion.failure);
    var start = solver.forward(entry);
    var delta = [2.6, 1.2, -0.15];
    var stop = new Pose3(start.x + delta[0], start.y + delta[1], start.z + delta[2],
      start.qx, start.qy, start.qz, start.qw);
    var path = new motionkit.path.PosePath(motionkit.robot.HandlingPlanRunner.FRAME, [new motionkit.path.PoseLine(
      new motionkit.path.PoseWaypoint(start, 0.002, 0.02), new motionkit.path.PoseWaypoint(stop, 0.002, 0.02),
      motionkit.path.OrientationPolicy.FreeAboutTool, 0.1, 0.08)]);
    check(path.length() > 2 * 1.2, "Coordinated travel exceeds twice the arm's conservative reach");
    solver.preferredPosture = entry;
    // A narrow moving external corridor exercises all three axes without a
    // Cartesian product over the whole table at every 2 mm task sample.
    var rule:(Int,Float,Pose3)->Array<motionkit.robot.ExternalAxisGrid.ExternalAxisRange> = (sample,distance,pose) -> {
      var fraction = distance / path.length();
      var widths = [0.02, 0.01, 0.005];
      return [for (axis in 0...3) {
        var centre = entry[xyz[axis]] + (axis == 2 ? -delta[axis] : delta[axis]) * fraction;
        new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(xyz[axis], centre - widths[axis], centre + widths[axis], 3);
      }];
    };
    var sampling = new motionkit.robot.CandidateProblem.CandidateSamplingOptions(8,3,8,true,[],rule);
    // Fitted gantry hardware has a nominal 3 mm gap. A stated 2 mm planning
    // margin leaves positive room for the continuous displacement certificate.
    var clearance = new robotkit.collision.CollisionClearance(solver.manipulator,
      mission.weldCollisionBodies(cast solver.manipulator,"arm/tool/cup",[]),entry,
      () -> new collisionkit.native.NativeCollisionWorld(),0.002,0.0005);
    var closest = clearance.closest(entry);
    if (closest != null) Sys.println('overhead entry clearance: ${closest.a} / ${closest.b}, ${closest.distance} m, requires ${closest.required} m');
    var compiler = motion.compiler.withJointPathPlanner(new motionkit.robot.StructuredJointPathPlanner(
      solver.manipulator,sampling,null,clearance,8,false,null,null,null,0,solver));
    motion = new motionkit.robot.ManipulatorMotion(motion.robot,compiler,_ -> null,
      () -> ({events:[],overflow:false}),motion.jointIndices);
    var numericBefore = solver.manipulator.numericSolveCount();
    var analytic = new EaikKinematics(solver.manipulator);
    function branch(q:Array<Float>):Int return analytic.analytic.configuration.branch(
      [for (index in analytic.analytic.chain.armIndices) q[index]]);
    var initialBranch = branch(entry);
    try {
      motion.run(new motionkit.program.MotionProgram([
        motionkit.program.MotionOp.FollowPath(path,motionkit.robot.HandlingPlanRunner.FRAME,0.08,[])]));
      var previous = entry.copy(), last = entry.copy(), worst = 0.0, margin = Math.POSITIVE_INFINITY, jump = 0.0;
      deadline = simulation.activeSession().simulationTime() + 180;
      var lengthSquared = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2];
      while (!motion.completed && motion.failure == null && simulation.activeSession().simulationTime() < deadline) {
        motion.update(simulation.timestep); simulation.step();
        var feedback = mission.robot.robot.snapshot().positions;
        last = [for (index in motion.jointIndices) feedback.get(index)];
        check(branch(last) == initialBranch, "Inverted motion preserves its geometric IK branch");
        check(clearance.violation(last) == null, "Coordinated feedback clears the arm, gantry and table");
        var tip = mission.toolContact(channel);
        var t = Math.max(0.0,Math.min(1.0,((tip.x-start.x)*delta[0]+(tip.y-start.y)*delta[1]+(tip.z-start.z)*delta[2])/lengthSquared));
        worst = Math.max(worst,Math.sqrt(Math.pow(tip.x-start.x-t*delta[0],2)+Math.pow(tip.y-start.y-t*delta[1],2)+Math.pow(tip.z-start.z-t*delta[2],2)));
        for (index in 0...last.length) if (!solver.manipulator.external[index]) {
          var limits = solver.manipulator.group.limitsOf(index);
          margin = Math.min(margin,Math.min(last[index]-limits.lower,limits.upper-last[index]));
          jump = Math.max(jump,Math.abs(last[index]-previous[index]));
        }
        previous = last.copy();
      }
      check(motion.completed && motion.failure == null, "Overhead coordinated path: " + motion.failure);
      check(PoseMath.distance(solver.forward(last),stop) < 0.002 && worst < 0.002, "Coordinated TCP remains within 2 mm");
      for (axis in 0...3) check(Math.abs(last[xyz[axis]]-entry[xyz[axis]]) > 0.8*Math.abs(delta[axis]),
        "Each external axis supplies most of its corresponding task travel");
      check(margin > 0.15 && jump < 0.05, 'Inverted arm retains posture margin and avoids branch jumps ($margin margin, $jump jump)');
      var wrist = analytic.analytic.chain.armIndices[5];
      check(Math.abs(last[wrist] - entry[wrist]) < 1.0, "Coordinated motion retains its wrist winding");
      check(solver.manipulator.numericSolveCount() == numericBefore, "Supported inverted motion uses analytic IK");
      Sys.println('overhead arm ($backend): ${path.length()} m coordinated path, ${worst*1000} mm error, $margin rad margin, $jump rad arm step');
    } catch (error:Dynamic) { clearance.world.dispose(); throw error; }
    clearance.world.dispose();
  }
}
