package tests;

import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.Manipulator;
import processkit.manipulation.ReachabilityChecker;
import processkit.manipulation.WorkPatchPlanner;
import robotkit.model.CollisionShape;
import robotkit.collision.CollisionClearance;
import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPose;
import collisionkit.native.NativeCollisionWorld;
import robotkit.tool.ToolGeometry;
import processkit.path.ToolpathPoint;
import processkit.path.Toolpath;
import processkit.work.WorkSurface;
import processkit.work.Polygon2;
import processkit.work.Point2;

/** M8 acceptance tests for robotkit.manipulation: ReachabilityChecker, WorkPatchPlanner. */
class PlacementTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testReachabilityCheckerReportsFractionAndFirstFailure();
    testWallSplitIntoReachablePatches();
    testObstacleForcesADifferentBasePose();
    testConvexToolClearancePreservesEmptyCorner();
    testToolClearanceChecksJointMotion();
    testApproachPoseFarFromTheSeedIsReachable();
    testFirstPointDoesNotCheckSeedApproach();
    testFiveMillimetreObstacleBetweenPoints();
    testWorkPatchPlannerUsesToolObstacles();
    Sys.println('RobotKit placement tests passed ($assertions assertions)');
    return assertions;
  }

  /**
   * The pose a path starts from can be far from the seed posture: tracking IK, which steps from a nearby seed, does not reach
   * this one from it in 100 iterations. The planner's reachability check and a plan runner both solve that pose with
   * `solveApproach`, so a path the planner calls reachable is one the runner can start.
   */
  static function testApproachPoseFarFromTheSeedIsReachable():Void {
    var fixture = buildUR5Fixture();
    var manipulator = fixture.arm;
    var seed = [0.3, -0.9, 1.2, -0.3, 0.5, 0.0];
    var target = manipulator.tcpPose([3.0, -1.0, 1.0, 2.0, 1.0, -2.0]);
    var approach = ReachabilityChecker.solveApproach(manipulator, target, seed);
    check(approach.converged, 'The approach solve reaches a pose far from the seed (${approach.status} after ${approach.iterations} iterations)');
    var toolpath = new Toolpath("base", [new ToolpathPoint(target, 0.05, true)]);
    var result = ReachabilityChecker.check(manipulator, toolpath, Transform3.identity(), seed);
    check(result.fullyReachable(), "The reachability check solves the first point of a path the way the runner does");
  }

  static function testReachabilityCheckerReportsFractionAndFirstFailure():Void {
    var fixture = buildUR5Fixture();
    var manipulator = fixture.arm;
    var referenceQ = [0.3, -0.9, 1.2, -0.3, 0.5, 0.0];
    var referencePose = manipulator.tcpPose(referenceQ);

    var reachable = new ToolpathPoint(referencePose, 0.05, true);
    var unreachable = new ToolpathPoint(new Transform3(new Vec3(100.0, 100.0, 100.0), referencePose.rotation), 0.05, true);
    var toolpath = new Toolpath("base", [reachable, unreachable, reachable]);

    var result = ReachabilityChecker.check(manipulator, toolpath, Transform3.identity(), referenceQ);
    check(approx(result.reachableFraction, 2.0 / 3.0, 1e-9),
      'Reachability checker reports the correct reachable fraction (got ${result.reachableFraction})');
    check(result.firstFailureIndex == 1, "Reachability checker reports the index of the first unreachable point");
  }

  static function testWallSplitIntoReachablePatches():Void {
    var scenario = buildWallScenario();
    var plan = WorkPatchPlanner.plan(scenario.design, scenario.map_T_surface, scenario.manipulator,
      0.2, 0.08, 0.0, 0.02, 0.05, 0.0, 0.46, 0.5, scenario.seed, 2, 1, 0.05);

    check(plan.patches.length >= 2, 'A 6m wall splits into at least two patches (got ${plan.patches.length})');
    check(plan.fullyPlanned, "Every patch is fully reachable from its chosen base pose");
    for (patch in plan.patches)
      check(patch.reachableFraction == 1.0, 'Patch reachable fraction is 1.0 (got ${patch.reachableFraction})');
  }

  static function testObstacleForcesADifferentBasePose():Void {
    var scenario = buildWallScenario();
    var withoutObstacle = WorkPatchPlanner.plan(scenario.design, scenario.map_T_surface, scenario.manipulator,
      0.2, 0.08, 0.0, 0.02, 0.05, 0.0, 0.46, 0.5, scenario.seed, 2, 1, 0.02);
    var originalPose = withoutObstacle.patches[0].basePose;

    var obstacles = obstacle(new Transform3(new Vec3(originalPose.x, originalPose.y, 0), Quat.identity()), new Vec3(0.01,0.01,1));
    var withObstacle = WorkPatchPlanner.plan(scenario.design, scenario.map_T_surface, scenario.manipulator,
      0.2, 0.08, 0.0, 0.02, 0.05, 0.0, 0.46, 0.5, scenario.seed, 2, 1, 0.02, obstacles, 1e-4, 1e-3, null, null, 0, null, 0.005, () -> new NativeCollisionWorld());
    var movedPose = withObstacle.patches[0].basePose;

    var moved = Math.abs(movedPose.x - originalPose.x) > 1e-6 || Math.abs(movedPose.y - originalPose.y) > 1e-6;
    check(moved, "An obstacle at the original candidate forces the planner to choose a different base pose");
    check(withObstacle.patches[0].reachableFraction == 1.0, "The alternate base pose still fully reaches the patch");
  }

  static function obstacle(pose:Transform3, half:Vec3):CollisionDescription {
    var scene = new CollisionDescription(), p = pose.translation, q = pose.rotation;
    scene.addFixed("obstacle", new CollisionPose(p.x,p.y,p.z,q.x,q.y,q.z,q.w), Box(half.x,half.y,half.z), 0);
    return scene;
  }

  static function toolClearance(manipulator:Manipulator, shapes:Array<CollisionShape>,
      pose:Transform3, half:Vec3):CollisionClearance {
    var scene = CollisionClearance.shapes(manipulator.flangeLink, shapes, true);
    scene.append(obstacle(pose, half));
    scene.setGroupMargins(collisionkit.CollisionMarginClass.Tool, collisionkit.CollisionMarginClass.Air, 0.0, 0.0);
    return new CollisionClearance(manipulator, scene, [0.0], () -> new NativeCollisionWorld());
  }

  static function testConvexToolClearancePreservesEmptyCorner():Void {
    var shape = ToolGeometry.convex([
      boxVertices(new Vec3(0.0, 0.0, 0.5), new Vec3(0.05, 0.05, 0.5)),
      boxVertices(new Vec3(0.45, 0.0, 0.95), new Vec3(0.45, 0.05, 0.05))
    ], 0.0);
    var manipulator = slidingManipulator();
    var gap = toolClearance(manipulator, shape,
      new Transform3(new Vec3(0.45, 0.0, 0.45), Quat.identity()), new Vec3(0.04, 0.04, 0.04));
    check(gap.violation([0.0]) == null, "Tool hulls leave the L-shaped empty corner clear");
    var cup = toolClearance(manipulator, shape,
      new Transform3(new Vec3(0.85, 0.0, 0.95), Quat.identity()), new Vec3(0.04, 0.04, 0.04));
    check(cup.violation([0.0]) != null, "Tool hull detects an obstacle touching the offset cup");
  }

  static function testToolClearanceChecksJointMotion():Void {
    var model = new RobotModel("tool-motion");
    var base = model.addLink(new Link("base"));
    var moving = model.addLink(new Link("moving"));
    var joint = model.addJoint(new Joint("slide", JointType.Prismatic, base, moving));
    joint.axis = [1.0, 0.0, 0.0];
    joint.limits.lower = -1.0;
    joint.limits.upper = 1.0;
    var flange = model.addFrame(new Frame("flange", moving));
    var manipulator = new Manipulator(model, base.id, flange.id);
    var checker = toolClearance(manipulator, ToolGeometry.convex([
      boxVertices(Vec3.zero(), new Vec3(0.05, 0.05, 0.05))], 0.0),
      Transform3.identity(), new Vec3(0.1, 0.1, 0.1));
    check(checker.violation([-1.0]) == null, "Motion starts clear");
    check(checker.violation([1.0]) == null, "Motion ends clear");
    check(checker.sweep([-1.0], [1.0], false, checker.jointStep([-1.0], [1.0], 0.005)) != null,
      "Joint motion checks the obstacle between clear endpoints");

    var path = new Toolpath("base", [
      new ToolpathPoint(manipulator.tcpPose([-1.0]), 0.05, true),
      new ToolpathPoint(manipulator.tcpPose([1.0]), 0.05, true)
    ]);
    var result = ReachabilityChecker.check(manipulator, path, Transform3.identity(), [-1.0],
      1e-4, 1e-3, 100, 0.02, checker);
    check(result.firstFailureIndex == 1, "Reachability rejects a collision along the joint segment");
  }

  static function testWorkPatchPlannerUsesToolObstacles():Void {
    var scenario = buildWallScenario();
    var small = new WorkSurface("small-wall", "map", Transform3.identity(), new Polygon2([
      new Point2(-0.1, 0.0), new Point2(0.1, 0.0),
      new Point2(0.1, 0.3), new Point2(-0.1, 0.3)
    ]));
    var shape = ToolGeometry.box(new Vec3(0.05, 0.05, 0.05));
    var obstacles = obstacle(Transform3.identity(), new Vec3(10.0, 10.0, 10.0));
    var worlds = 0;
    var plan = WorkPatchPlanner.plan(small, scenario.map_T_surface, scenario.manipulator,
      0.2, 0.08, 0.0, 0.02, 0.05, 0.0, 0.46, 0.5, scenario.seed,
      1, 1, 0.02, null, 1e-4, 1e-3, shape, obstacles, 0, null, 0.005, () -> { worlds++; return new NativeCollisionWorld(); });
    check(!plan.fullyPlanned, "Work patch planner rejects a path blocked by a 3D tool obstacle");
    check(worlds == 1,
      "Work patch planner builds one collision world for its one candidate");
  }

  static function testFirstPointDoesNotCheckSeedApproach():Void {
    var manipulator = slidingManipulator();
    var checker = toolClearance(manipulator, ToolGeometry.box(new Vec3(0.001, 0.001, 0.001)),
      Transform3.identity(), new Vec3(0.01, 0.01, 0.01));
    var path = new Toolpath("base", [
      new ToolpathPoint(manipulator.tcpPose([0.1]), 0.05, true),
      new ToolpathPoint(manipulator.tcpPose([0.2]), 0.05, true)
    ]);
    var result = ReachabilityChecker.check(manipulator, path, Transform3.identity(), [-0.1],
      1e-4, 1e-3, 100, 0.02, checker);
    check(result.fullyReachable(), "Seed approach does not count against the first process point");
    var approach = ReachabilityChecker.check(manipulator, path, Transform3.identity(), [-0.1],
      1e-4, 1e-3, 100, 0.02, checker, null, 0.005, true);
    check(approach.firstFailureIndex == 0, "Explicit approach checking catches the seed obstacle");
  }

  static function testFiveMillimetreObstacleBetweenPoints():Void {
    var manipulator = slidingManipulator();
    var checker = toolClearance(manipulator, ToolGeometry.box(new Vec3(0.001, 0.001, 0.001)),
      Transform3.identity(), new Vec3(0.0025, 0.0025, 0.0025));
    var path = new Toolpath("base", [
      new ToolpathPoint(manipulator.tcpPose([-0.01]), 0.05, true),
      new ToolpathPoint(manipulator.tcpPose([0.01]), 0.05, true)
    ]);
    var result = ReachabilityChecker.check(manipulator, path, Transform3.identity(), [-0.01],
      1e-4, 1e-3, 100, 0.02, checker);
    check(result.firstFailureIndex == 1,
      "Five millimetre obstacle between process points is detected");
  }

  static function slidingManipulator():Manipulator {
    var model = new RobotModel("tool-step");
    var base = model.addLink(new Link("base"));
    var moving = model.addLink(new Link("moving"));
    var joint = model.addJoint(new Joint("slide", JointType.Prismatic, base, moving));
    joint.axis = [1.0, 0.0, 0.0];
    joint.limits.lower = -1.0;
    joint.limits.upper = 1.0;
    var flange = model.addFrame(new Frame("flange", moving));
    return new Manipulator(model, base.id, flange.id);
  }

  static function boxVertices(centre:Vec3, half:Vec3):Array<Float> {
    var vertices = [];
    for (x in [-1.0, 1.0]) for (y in [-1.0, 1.0]) for (z in [-1.0, 1.0]) {
      vertices.push(centre.x + x * half.x);
      vertices.push(centre.y + y * half.y);
      vertices.push(centre.z + z * half.z);
    }
    return vertices;
  }

  // -- fixtures and helpers ---------------------------------------------

  static function buildWallScenario():{design:WorkSurface, map_T_surface:Transform3, manipulator:Manipulator, seed:Array<Float>} {
    var boundary = new Polygon2([
      new Point2(-3.0, 0.0), new Point2(3.0, 0.0), new Point2(3.0, 0.3), new Point2(-3.0, 0.3)
    ]);
    var design = new WorkSurface("finish-wall", "map", Transform3.identity(), boundary);
    // Rotation whose columns send the surface's local X/Y/Z (along-wall,
    // vertical, outward normal) to the map's Y/Z/X axes respectively (a
    // proper rotation: a cyclic axis permutation, determinant +1).
    var rotation = Quat.fromRotationMatrix([0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]);
    var map_T_surface = new Transform3(new Vec3(-0.47, 0.0, -0.15), rotation);

    var fixture = buildUR5Fixture();
    var manipulator = fixture.arm;
    var seed = [0.2, -1.0, 1.3, -0.3, 0.5, 0.0];
    return { design: design, map_T_surface: map_T_surface, manipulator: manipulator, seed: seed };
  }

  static function buildUR5Fixture():{model:RobotModel, arm:Manipulator} {
    var d1 = 0.089159, shoulderOffset = 0.13585, elbowOffset = -0.1197,
      a2 = 0.425, a3 = 0.39225, d4 = 0.10915, d5 = 0.09465, d6 = 0.0823;
    var model = new RobotModel("ur5-fixture");
    var linkNames = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
      "wrist_1_link", "wrist_2_link", "wrist_3_link"];
    var links = [for (name in linkNames) model.addLink(new Link(name))];
    var offsets = [
      new Vec3(0.0, 0.0, d1),
      new Vec3(0.0, shoulderOffset, 0.0),
      new Vec3(0.0, elbowOffset, a2),
      new Vec3(0.0, 0.0, a3),
      new Vec3(0.0, d4, 0.0),
      new Vec3(0.0, 0.0, d5)
    ];
    var axes = [
      [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
      [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]
    ];
    var jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"];
    for (i in 0...6) {
      var joint = model.addJoint(new Joint(jointNames[i], JointType.Revolute, links[i], links[i + 1]));
      joint.parentFramePosition = offsets[i].toArray();
      joint.axis = axes[i];
      joint.limits.lower = -2.0 * Math.PI;
      joint.limits.upper = 2.0 * Math.PI;
      joint.limits.velocity = null;
    }
    var flangeOffset = new Vec3(0.0, d6, 0.0);
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = flangeOffset.toArray();
    var arm = new Manipulator(model, links[0].id, flange.id);
    return { model: model, arm: arm };
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
