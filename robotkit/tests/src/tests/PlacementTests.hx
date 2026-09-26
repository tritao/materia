package tests;

import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.ChainTip;
import robotkit.manipulation.KinematicChain;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ReachabilityChecker;
import robotkit.manipulation.WorkPatchPlanner;
import robotkit.manipulation.BaseObstacle;
import robotkit.process.ToolpathPoint;
import robotkit.process.Toolpath;
import robotkit.work.WorkSurface;
import robotkit.work.Polygon2;
import robotkit.work.Point2;

/** M8 acceptance tests for robotkit.manipulation: ReachabilityChecker, WorkPatchPlanner. */
class PlacementTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testReachabilityCheckerReportsFractionAndFirstFailure();
    testWallSplitIntoReachablePatches();
    testObstacleForcesADifferentBasePose();
    Sys.println('RobotKit placement tests passed ($assertions assertions)');
    return assertions;
  }

  static function testReachabilityCheckerReportsFractionAndFirstFailure():Void {
    var fixture = buildUR5Fixture();
    var manipulator = new Manipulator(fixture.model, fixture.chain);
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

    var obstacles = [new BaseObstacle(originalPose.x, originalPose.y, 0.01)];
    var withObstacle = WorkPatchPlanner.plan(scenario.design, scenario.map_T_surface, scenario.manipulator,
      0.2, 0.08, 0.0, 0.02, 0.05, 0.0, 0.46, 0.5, scenario.seed, 2, 1, 0.02, obstacles);
    var movedPose = withObstacle.patches[0].basePose;

    var moved = Math.abs(movedPose.x - originalPose.x) > 1e-6 || Math.abs(movedPose.y - originalPose.y) > 1e-6;
    check(moved, "An obstacle at the original candidate forces the planner to choose a different base pose");
    check(withObstacle.patches[0].reachableFraction == 1.0, "The alternate base pose still fully reaches the patch");
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
    var manipulator = new Manipulator(fixture.model, fixture.chain);
    var seed = [0.2, -1.0, 1.3, -0.3, 0.5, 0.0];
    return { design: design, map_T_surface: map_T_surface, manipulator: manipulator, seed: seed };
  }

  static function buildUR5Fixture():{model:RobotModel, chain:KinematicChain} {
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
      joint.limits.velocity = 0.0;
    }
    var flangeOffset = new Vec3(0.0, d6, 0.0);
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = flangeOffset.toArray();
    var chain = new KinematicChain(model, links[0].id, ChainTip.Frame(flange.id));
    return { model: model, chain: chain };
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
