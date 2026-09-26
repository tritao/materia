package tests;

import haxe.Int64;
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
import robotkit.manipulation.InverseKinematics;
import robotkit.manipulation.JointGroup;
import robotkit.manipulation.Manipulator;
import robotkit.tool.Tool;
import robotkit.tool.ToolCollisionShape;
import robotkit.process.ToolpathExecutionResult;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.skill.Skill;
import robotkit.skill.SkillRunner;
import robotkit.skill.SkillStatus;
import robotkit.skill.DigTrench;
import robotkit.skill.GradeRegion;
import robotkit.skill.DumpAt;
import robotkit.work.HeightMap;
import robotkit.work.EarthworkRegion;
import robotkit.work.DigCyclePlanner;
import robotkit.work.DigCyclePlan;
import robotkit.work.Point2;
import robotkit.world.SimulatedRobot;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;

/**
 * M12 acceptance tests for the simulated excavator: the 4-DOF
 * slew/boom/stick/bucket kinematic chain, `DigCyclePlanner`, and the
 * `DigTrench`/`GradeRegion`/`DumpAt` skills, run through `SkillRunner`
 * against a `SimulatedRobot` on the default backend.
 */
class ExcavatorTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testZeroPoseFK();
    testFourDofIkRecoversManifoldTargets();
    testDigCyclePlannerStages();
    testDigTrenchScenario();
    testGradeRegionScenario();
    testDumpAtMovesToTarget();
    Sys.println('RobotKit excavator tests passed ($assertions assertions)');
    return assertions;
  }

  // -- M12 kinematics -----------------------------------------------------

  static function testZeroPoseFK():Void {
    var fixture = buildExcavatorFixture();
    var zero = [0.0, 0.0, 0.0, 0.0];
    var tcp = fixture.manipulator.tcpPose(zero);
    check(approx(tcp.translation.x, 6.0, 1e-9) && approx(tcp.translation.y, 0.0, 1e-9) &&
      approx(tcp.translation.z, 0.9, 1e-9),
      "Excavator zero-pose TCP position matches the summed fixture offsets");
    check(tcp.rotation.angularDistance(Quat.identity()) < 1e-9,
      "Excavator zero-pose TCP has no net rotation");
    check(fixture.chain.dofCount() == 4, "Excavator fixture exposes four degrees of freedom");
  }

  static function testFourDofIkRecoversManifoldTargets():Void {
    var fixture = buildExcavatorFixture();
    var group = JointGroup.fromChain(fixture.chain);
    var seed = [0.0, 0.3, -1.0, -1.0];
    var samples = [
      { x: 3.2, y: 0.0, z: -0.4, pitch: -0.4 },
      { x: 3.6, y: 0.4, z: 0.1, pitch: -1.6 },
      { x: 2.6, y: -0.3, z: 0.6, pitch: 0.5 }
    ];
    var current = seed;
    for (sample in samples) {
      var target = DigCyclePlanner.poseAt(sample.x, sample.y, sample.z, sample.pitch);
      var result = InverseKinematics.solve(fixture.chain, group, target, current, 1e-4, 1e-3, 300, 0.02);
      check(result.converged, 'Excavator IK converges for manifold target ($sample.x, $sample.y, $sample.z, pitch=$sample.pitch)');
      var achieved = fixture.chain.forwardKinematics(result.q);
      check(approx(achieved.translation.x, target.translation.x, 1e-3) &&
        approx(achieved.translation.y, target.translation.y, 1e-3) &&
        approx(achieved.translation.z, target.translation.z, 1e-3),
        "Excavator IK solution FK matches the requested position");
      check(achieved.rotation.angularDistance(target.rotation) < 1e-2,
        "Excavator IK solution FK matches the requested orientation");
      current = result.q;
    }
  }

  static function testDigCyclePlannerStages():Void {
    var entry = new Point2(3.0, 0.0);
    var exit = new Point2(4.0, 0.0);
    var dump = new Point2(1.5, 3.0);
    var plan = DigCyclePlanner.planCycle("excavator", entry, exit, 0.0, 0.3,
      0.6, dump, 0.4, 0.4, -0.4, -1.8, 0.6, 0.4, 8);
    var points = plan.toolpath.points;
    check(points.length == 14,
      "Dig cycle toolpath has entry + cut + curl + lift + 8 swing steps + descend + open");
    check(!points[0].processOn, "Entry point is not engaged with the ground");
    check(points[1].processOn, "Cut point is engaged with the ground");
    check(points[2].processOn, "Curl point is engaged with the ground");
    check(!points[3].processOn, "Lift point is not engaged with the ground");
    check(approx(points[1].work_T_tcp.translation.x, exit.x - 0.4, 1e-9) &&
      approx(points[1].work_T_tcp.translation.z, -0.3, 1e-9),
      "Cut point reaches the inward offset from the exit wall at target depth");
    var descend = points[points.length - 2];
    var open = points[points.length - 1];
    check(approx(descend.work_T_tcp.translation.x, dump.x, 1e-9) &&
      approx(descend.work_T_tcp.translation.y, dump.y, 1e-9) &&
      approx(descend.work_T_tcp.translation.z, 0.4, 1e-9),
      "Descend point reaches the dump position and elevation, still curled");
    check(approx(open.work_T_tcp.translation.x, dump.x, 1e-9) &&
      approx(open.work_T_tcp.translation.y, dump.y, 1e-9) &&
      approx(open.work_T_tcp.translation.z, 0.4, 1e-9),
      "Open point stays at the dump position and elevation");
    check(descend.work_T_tcp.rotation.angularDistance(open.work_T_tcp.rotation) > 0.1,
      "Open point's pitch differs from the descend point's curled pitch");
    check(plan.sweepFrom == entry && plan.sweepTo == exit,
      "Dig cycle plan retains the full authored sweep segment for material accounting");
    check(approx(plan.cuttingEdgeFrom.x, entry.x + 0.4, 1e-9) &&
      approx(plan.cuttingEdgeTo.x, exit.x - 0.4, 1e-9),
      "Dig cycle plan insets the cutting edge by half the bucket width at both walls");
    check(approx(plan.sweepHalfWidth, 0.4, 1e-9), "Dig cycle plan carries the sweep half-width");
    check(approx(plan.sweepEdgeHeight, -0.3, 1e-9), "Dig cycle plan carries the cut depth as the sweep edge height");
  }

  // -- M12 scenario ---------------------------------------------------------

  static function testDigTrenchScenario():Void {
    var fixture = buildExcavatorFixture();
    var blueprint = RobotRuntimeCompiler.compile(fixture.model);
    var simulation = new Simulation(0.02);
    var linkNames = [for (link in fixture.model.links) link.name];
    var jointNames = [for (joint in fixture.model.joints) joint.name];
    var robot = new SimulatedRobot("excavator", simulation.addRobot(blueprint), fixture.model.name, linkNames, jointNames);

    var groundZ = 0.0;
    var heightMap = new HeightMap("excavator", 0.0, -2.0, 0.1, 61, 41);
    for (row in 0...41) for (col in 0...61) heightMap.setElevation(col, row, groundZ);

    var lineFrom = new Point2(2.5, 0.0);
    var lineTo = new Point2(4.0, 0.0);
    var width = 0.8;
    var depth = 0.5;
    var gradeTolerance = 0.03;
    var spec:DigTrenchSpec = {
      clearanceZ: 0.6, dumpX: 2.0, dumpY: 2.5, dumpZ: 0.4,
      digPitch: -0.4, curlPitch: -1.8, dumpPitch: 0.6,
      feedRate: 0.4, maxAcceleration: 0.6, sampleInterval: 0.05,
      maxCutPerPass: 0.2, maxCycles: 10, maxJointStep: 6.5,
      positionTolerance: 2e-3, orientationTolerance: 5e-3, ikMaxIterations: 300, ikDamping: 0.03,
      progressSamples: 4
    };
    var seed = [0.0, 0.3, -1.0, -1.0];
    var dig = new DigTrench(fixture.manipulator, robot, heightMap, "excavator",
      lineFrom, lineTo, width, depth, gradeTolerance, spec, seed);

    var runner = new SkillRunner();
    var tick = 0;
    var lastCyclesReported = 0;
    var status = runner.start(dig);
    var steps = 0;
    while (status == SkillStatus.Running && steps < 200000) {
      simulation.step(Int64.ofInt(tick));
      tick++;
      var snapshot = robot.snapshot();
      status = runner.update(snapshot, 0.02);
      if (dig.cyclesCompleted != lastCyclesReported) {
        lastCyclesReported = dig.cyclesCompleted;
        Sys.println('M12 trench dig cycle $lastCyclesReported: remainingDepthError=${dig.remainingDepthError()} totalRemovedVolume=${dig.totalRemovedVolume}');
      }
      steps++;
    }

    check(switch status { case SkillStatus.Succeeded: true; case _: false; },
      'DigTrench reaches Succeeded (got $status)');
    check(dig.cyclesCompleted >= 1 && dig.cyclesCompleted <= spec.maxCycles,
      'DigTrench completes within the cycle budget (${dig.cyclesCompleted} cycles)');
    check(dig.remainingDepthError() <= gradeTolerance,
      'DigTrench reaches the design depth within grade tolerance (remaining=${dig.remainingDepthError()})');
    var expectedVolume = (lineTo.x - lineFrom.x) * width * depth;
    check(dig.totalRemovedVolume >= expectedVolume * 0.9 && dig.totalRemovedVolume <= expectedVolume * 1.1,
      'DigTrench removed volume (${dig.totalRemovedVolume} m^3) stays within 10% of the design volume ($expectedVolume m^3)');
    var design = dig.designMap;
    var trenchFootprint = dig.footprint;
    if (design == null || trenchFootprint == null) throw "DigTrench did not retain its generated design region";
    var belowDesign = false;
    var outsideFootprint = false;
    for (row in 0...heightMap.rows) for (col in 0...heightMap.columns) {
      var point = new Point2(heightMap.worldX(col), heightMap.worldY(row));
      var actual = heightMap.elevationAt(col, row);
      if (actual < design.elevationAt(col, row) - gradeTolerance - 1e-9) belowDesign = true;
      if (!trenchFootprint.containsOrWithin(point, heightMap.cellSize * 0.5) && actual < groundZ - 1e-9)
        outsideFootprint = true;
    }
    check(!belowDesign, "DigTrench does not cut below the design surface beyond grade tolerance");
    check(!outsideFootprint, "DigTrench does not lower vertices outside the footprint plus half a cell");
    Sys.println('M12 trench scenario: cycles=${dig.cyclesCompleted} finalDepthError=${dig.remainingDepthError()} removedVolume=${dig.totalRemovedVolume}');
    simulation.dispose();
  }

  static function testGradeRegionScenario():Void {
    var fixture = buildExcavatorFixture();
    var blueprint = RobotRuntimeCompiler.compile(fixture.model);
    var simulation = new Simulation(0.02);
    var linkNames = [for (link in fixture.model.links) link.name];
    var jointNames = [for (joint in fixture.model.joints) joint.name];
    var robot = new SimulatedRobot("excavator-grade", simulation.addRobot(blueprint), fixture.model.name, linkNames, jointNames);

    var columns = 9, rows = 9;
    var cellSize = 0.15;
    var originX = 2.8, originY = -0.6;
    var design = new HeightMap("excavator", originX, originY, cellSize, columns, rows);
    var existingElevation:Array<Float> = [for (_ in 0...(columns * rows)) 0.0];
    var existing = new HeightMap("excavator", originX, originY, cellSize, columns, rows, existingElevation);
    // One high spot needs grading down to design (0.0); everything else already at grade.
    existing.setElevation(4, 4, 0.12);
    var region = new EarthworkRegion("pad", existing, design, [], 0.01);

    var spec:GradeRegionSpec = {
      clearanceZ: 0.6, dumpX: 2.0, dumpY: 2.5, dumpZ: 0.4,
      digPitch: -0.4, curlPitch: -1.8, dumpPitch: 0.6,
      feedRate: 0.4, maxAcceleration: 0.6, sampleInterval: 0.05,
      maxCutPerPass: 0.2, maxCycles: 10, maxJointStep: 6.5,
      positionTolerance: 2e-3, orientationTolerance: 5e-3, ikMaxIterations: 300, ikDamping: 0.03,
      bucketHalfWidth: 0.3
    };
    var seed = [0.0, 0.3, -1.0, -1.0];
    var grade = new GradeRegion(fixture.manipulator, robot, region, "excavator", spec, seed);

    var runner = new SkillRunner();
    var tick = 0;
    var status = runner.start(grade);
    var steps = 0;
    while (status == SkillStatus.Running && steps < 200000) {
      simulation.step(Int64.ofInt(tick));
      tick++;
      status = runner.update(robot.snapshot(), 0.02);
      steps++;
    }

    check(switch status { case SkillStatus.Succeeded: true; case _: false; },
      'GradeRegion reaches Succeeded (got $status)');
    check(approx(region.gradeFraction(), 1.0, 1e-9), "GradeRegion brings every vertex to grade");
    check(grade.cyclesCompleted >= 1, "GradeRegion runs at least one cycle");
    simulation.dispose();
  }

  static function testDumpAtMovesToTarget():Void {
    var fixture = buildExcavatorFixture();
    var blueprint = RobotRuntimeCompiler.compile(fixture.model);
    var simulation = new Simulation(0.02);
    var linkNames = [for (link in fixture.model.links) link.name];
    var jointNames = [for (joint in fixture.model.joints) joint.name];
    var robot = new SimulatedRobot("excavator-dump", simulation.addRobot(blueprint), fixture.model.name, linkNames, jointNames);

    // Seed already at the dump position, curled (as if just swung in after a
    // dig); DumpAt only needs to open the bucket in place. A cold seed far
    // across the workspace would ask CartesianTrajectory's straight-line
    // position lerp / rotation slerp to track this chain's reachable
    // orientation manifold (see DigCyclePlanner's own doc comment) over a
    // large combined swing, which it is not guaranteed to do -- the same
    // reason DigCyclePlanner densifies its own swing with manifold-consistent
    // waypoints. DumpAt does not have manifold knowledge for an arbitrary
    // target pose, so its realistic use (relocate a *little* and release) is
    // what this test exercises; see ARCHITECTURE.md.
    var seed = [0.8961, 0.994, -1.609, -1.185];
    var targetPose = DigCyclePlanner.poseAt(2.0, 2.5, 0.4, 0.6);
    var dumpAt = new DumpAt(fixture.manipulator, robot, "excavator", targetPose, seed,
      0.4, 0.6, 0.05, 6.5, 2e-3, 8e-3, 500, 0.03);

    var runner = new SkillRunner();
    var tick = 0;
    var status = runner.start(dumpAt);
    var steps = 0;
    var lastQ = seed;
    while (status == SkillStatus.Running && steps < 20000) {
      simulation.step(Int64.ofInt(tick));
      tick++;
      var snapshot = robot.snapshot();
      lastQ = [for (i in 0...4) snapshot.positions.get(i)];
      status = runner.update(snapshot, 0.02);
      steps++;
    }

    check(switch status { case SkillStatus.Succeeded: true; case _: false; },
      'DumpAt reaches Succeeded (got $status)');
    var achieved = fixture.manipulator.tcpPose(lastQ);
    check(approx(achieved.translation.x, targetPose.translation.x, 5e-3) &&
      approx(achieved.translation.y, targetPose.translation.y, 5e-3) &&
      approx(achieved.translation.z, targetPose.translation.z, 5e-3),
      "DumpAt drives the TCP to the requested position");
    simulation.dispose();
  }

  // -- fixture --------------------------------------------------------

  static function buildExcavatorFixture():{model:RobotModel, chain:KinematicChain, manipulator:Manipulator, tool:Tool} {
    var model = new RobotModel("excavator-fixture");
    var undercarriage = model.addLink(new Link("undercarriage"));
    var upperStructure = model.addLink(new Link("upper_structure"));
    var boomLink = model.addLink(new Link("boom_link"));
    var stickLink = model.addLink(new Link("stick_link"));
    var bucketLink = model.addLink(new Link("bucket_link"));

    var slew = model.addJoint(new Joint("slew_joint", JointType.Revolute, undercarriage, upperStructure));
    slew.axis = [0.0, 0.0, 1.0];
    slew.limits.lower = -Math.PI * 2.0;
    slew.limits.upper = Math.PI * 2.0;

    var boom = model.addJoint(new Joint("boom_joint", JointType.Revolute, upperStructure, boomLink));
    boom.parentFramePosition = [0.3, 0.0, 1.0];
    boom.axis = [0.0, 1.0, 0.0];
    boom.limits.lower = -2.2;
    boom.limits.upper = 2.2;

    var stick = model.addJoint(new Joint("stick_joint", JointType.Revolute, boomLink, stickLink));
    stick.parentFramePosition = [3.0, 0.0, 0.0];
    stick.axis = [0.0, 1.0, 0.0];
    stick.limits.lower = -3.0;
    stick.limits.upper = 3.0;

    var bucket = model.addJoint(new Joint("bucket_joint", JointType.Revolute, stickLink, bucketLink));
    bucket.parentFramePosition = [1.8, 0.0, 0.0];
    bucket.axis = [0.0, 1.0, 0.0];
    bucket.limits.lower = -3.0;
    bucket.limits.upper = 3.0;

    var flange = model.addFrame(new Frame("bucket_flange", bucketLink));

    var chain = new KinematicChain(model, undercarriage.id, ChainTip.Frame(flange.id));
    var tool = new Tool("bucket", "bucket", new Transform3(new Vec3(0.9, 0.0, -0.1), Quat.identity()),
      ToolCollisionShape.Box(new Vec3(0.5, 0.6, 0.4)), 400.0);
    var manipulator = new Manipulator(model, chain, tool.flangeTTcp);
    return { model: model, chain: chain, manipulator: manipulator, tool: tool };
  }

  // -- small local helpers ---------------------------------------------

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
