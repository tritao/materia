package tests;

import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.Manipulator;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.tool.ConvexDistance;
import robotkit.tool.ConvexSolid;

/**
 * `ArmClearance` on a planar two-link arm in a cell with a post and a table: clear poses, a pose that hits the post, a
 * motion whose end poses are clear but whose path is not, the margin, the tool's contact margin, bodies that touch by
 * design, and hulls whose edges cross with no corner of either inside the other.
 */
class ClearanceTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testPosesAndPath();
    testMargins();
    testTouchingByDesign();
    testCoupledDriveContact();
    testMovingRackContact();
    testCrossingEdges();
    testBoundsPreserveHullChecks();
    testSolidEdges();
    Sys.println('RobotKit clearance tests passed ($assertions assertions)');
    return assertions;
  }

  static function testCoupledDriveContact():Void {
    var model = new RobotModel("screw-carriage-clearance");
    var base = model.addLink(new Link("base"));
    var carriage = model.addLink(new Link("carriage"));
    var shaft = model.addLink(new Link("shaft"));
    var slide = model.addJoint(new Joint("x", JointType.Prismatic, base, carriage));
    slide.axis = [1.0, 0, 0]; slide.limits.lower = -0.05; slide.limits.upper = 0.4;
    var screw = model.addJoint(new Joint("screw", JointType.Continuous, base, shaft));
    screw.axis = [1.0, 0, 0];
    model.addCoupling(new robotkit.model.JointCoupling("thread", slide.id, screw.id, 100, 0));
    var tip = model.addFrame(new Frame("tip", carriage));
    var group = new Manipulator(model, base.id, tip.id);
    var clearance = new ArmClearance(group, [
      {name: "nut", link: carriage.id, vertices: box(-0.02, 0.02, -0.04, 0.04, -0.04, 0.04), tool: false},
      {name: "screw", link: shaft.id, vertices: box(-0.1, 0.5, -0.006, 0.006, -0.006, 0.006), tool: false},
      {name: "work", link: base.id, vertices: box(0.28, 0.32, 0.025, 0.04, -0.025, 0.025), tool: false}
    ], [0.0]);
    check(clearance.violation([0.0]) == null && clearance.violation([0.1]) == null,
      "a driven nut's designed engagement stays clear at rest and through travel");
    var hit = clearance.violation([0.3]);
    check(hit != null && (hit.a == "work" || hit.b == "work"),
      "drive contact does not exempt the carriage from a workpiece collision");
  }

  static function testMovingRackContact():Void {
    var model = new RobotModel("rack-pinion-clearance");
    var base = model.addLink(new Link("base"));
    var carriage = model.addLink(new Link("carriage"));
    var pinion = model.addLink(new Link("pinion"));
    var slide = model.addJoint(new Joint("track", JointType.Prismatic, base, carriage));
    slide.axis = [1.0, 0, 0]; slide.parentFramePosition = [0.0, 0, 0.02];
    slide.limits.lower = 0.0; slide.limits.upper = 0.4;
    var shaft = model.addJoint(new Joint("pinion", JointType.Continuous, carriage, pinion));
    shaft.axis = [0.0, 1, 0];
    model.addCoupling(new robotkit.model.JointCoupling("rack", slide.id, shaft.id, -50, 0));
    var tip = model.addFrame(new Frame("tip", carriage));
    var group = new Manipulator(model, base.id, tip.id);
    var clearance = new ArmClearance(group, [
      {name: "carriage", link: carriage.id, vertices: box(-0.02, 0.02, -0.04, 0.04, 0.08, 0.1), tool: false},
      {name: "pinion", link: pinion.id, vertices: box(-0.03, 0.03, -0.01, 0.01, -0.03, 0.03), tool: false},
      {name: "rack", link: base.id, vertices: box(-0.1, 0.6, -0.01, 0.01, -0.02, 0), tool: false},
      {name: "work", link: base.id, vertices: box(0.28, 0.32, -0.02, 0.02, 0.01, 0.04), tool: false}
    ], [0.0]);
    check(clearance.violation([0.0]) == null && clearance.violation([0.1]) == null,
      "a moving pinion retains its designed engagement with the fixed rack");
    var hit = clearance.violation([0.3]);
    check(hit != null && (hit.a == "work" || hit.b == "work") && (hit.a == "pinion" || hit.b == "pinion"),
      "rack engagement does not exempt the pinion from another body on the same fixed link");
  }

  /** The vertices of the box spanning the given ranges. */
  static function box(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Array<Float> {
    var result:Array<Float> = [];
    for (x in [x0, x1]) for (y in [y0, y1]) for (z in [z0, z1]) {
      result.push(x);
      result.push(y);
      result.push(z);
    }
    return result;
  }

  /**
   * A 2R arm in the XY plane, links 0.3 m long (bars of 5 cm), with `fixed` bodies on the base link. The second link is
   * the tool.
   */
  static function rig(fixed:Array<{name:String, vertices:Array<Float>}>, ?linkWidth:Float = 0.025, ?reference:Array<Float>):{arm:Manipulator, clearance:ArmClearance} {
    var model = new RobotModel("clearance-arm");
    var base = model.addLink(new Link("base"));
    var link1 = model.addLink(new Link("link1"));
    var link2 = model.addLink(new Link("link2"));
    var joint1 = model.addJoint(new Joint("joint1", JointType.Revolute, base, link1));
    joint1.axis = [0.0, 0.0, 1.0];
    var joint2 = model.addJoint(new Joint("joint2", JointType.Revolute, link1, link2));
    joint2.parentFramePosition = [0.3, 0.0, 0.0];
    joint2.axis = [0.0, 0.0, 1.0];
    var tip = model.addFrame(new Frame("tip", link2));
    tip.position = [0.3, 0.0, 0.0];
    var arm = new Manipulator(model, base.id, tip.id);
    var bodies = [
      {name: "upper", link: link1.id, vertices: box(0.0, 0.3, -linkWidth, linkWidth, -linkWidth, linkWidth), tool: false},
      {name: "forearm", link: link2.id, vertices: box(0.0, 0.3, -linkWidth, linkWidth, -linkWidth, linkWidth), tool: true}
    ];
    for (body in fixed) bodies.push({name: body.name, link: base.id, vertices: body.vertices, tool: false});
    return {arm: arm, clearance: new ArmClearance(arm, bodies, reference == null ? [0.0, 0.0] : reference)};
  }

  static function testPosesAndPath():Void {
    // A post beside the straight arm, 0.1 m to its left, between 0.2 and 0.25 m out.
    var post = {name: "post", vertices: box(0.2, 0.25, 0.1, 0.2, -0.05, 0.05)};
    var cell = rig([post]);
    check(cell.clearance.violation([0.0, 0.0]) == null, "the straight arm is clear of the post");
    check(cell.clearance.violation([0.0, -1.0]) == null, "the forearm folded away from the post is clear");
    var hit = cell.clearance.violation([0.6, 0.0]);
    check(hit != null, "the arm swung toward the post hits it");
    var names = hit == null ? "" : hit.a + "," + hit.b;
    check(names.indexOf("post") >= 0 && names.indexOf("upper") >= 0, 'the violation names the parts, got $names');
    check(hit != null && hit.distance <= 0.0, "the arm overlaps the post");
    // A motion whose end poses are both clear but whose path goes through the post: the arm swings from along X to nearly along Y.
    check(cell.clearance.violation([1.4, 0.0]) == null, "the arm swung well past the post is clear");
    check(cell.clearance.sweep([0.0, 0.0], [1.4, 0.0]) != null, "the arm swinging round through the post is found along the way");
    check(cell.clearance.sweep([0.0, 0.0], [-1.4, 0.0]) == null, "the same swing away from the post is clear");
    check(cell.clearance.pairCount() > 0 && cell.clearance.bodyCount() == 3, "the arm's bodies are checked against the post");
  }

  static function testMargins():Void {
    // A table whose top is 3 mm below the arm's bars: closer than the 5 mm margin, but not touching.
    var table = {name: "table", vertices: box(-0.2, 0.8, -0.5, 0.5, -0.2, -0.025 - 0.003)};
    var cell = rig([table]);
    var tight = cell.clearance.violation([0.0, 0.0]);
    check(tight != null && tight.distance > 0.0 && tight.distance < ArmClearance.MARGIN, "3 mm above the table is a violation of the margin, not a collision");
    check(tight != null && Math.abs(tight.distance - 0.003) < 0.0005, 'its distance is 3 mm, got ${tight == null ? -1 : tight.distance * 1000} mm');
    // The tool in the contact zone may come as close as the contact margin: 3 mm is clear for the forearm but not the upper arm.
    var contact = cell.clearance.violation([0.0, 0.0], true);
    check(contact != null && contact.a != "forearm" && contact.b != "forearm", 'only the arm itself, not the tool, is held to the margin there, got ${contact == null ? "" : contact.a + "," + contact.b}');
    var lifted = rig([{name: "table", vertices: box(-0.2, 0.8, -0.5, 0.5, -0.2, -0.025 - 0.00025)}]);
    check(lifted.clearance.violation([0.0, 0.0], true) != null, "0.25 mm is closer than even the contact margin");
    var roomy = rig([{name: "table", vertices: box(-0.2, 0.8, -0.5, 0.5, -0.2, -0.025 - 0.02)}]);
    check(roomy.clearance.violation([0.0, 0.0]) == null, "20 mm above the table is clear");
    // With a margin wider than the gap the violation reports the distance between the faces.
    var strict = new ArmClearance(cell.arm, [
      {name: "forearm", link: "link2", vertices: box(0.0, 0.3, -0.025, 0.025, -0.025, 0.025), tool: true},
      {name: "table", link: "base", vertices: box(-0.2, 0.8, -0.5, 0.5, -0.2, -0.025 - 0.0123), tool: false}], [0.0, 0.0], 0.05, 0.05);
    var found = strict.violation([0.0, 0.0]);
    check(found != null && Math.abs(found.distance - 0.0123) < 1e-6, 'the distance between parallel faces is measured, got ${found == null ? -1 : found.distance}');
  }

  static function testTouchingByDesign():Void {
    // The upper arm and forearm bars meet at the elbow in the reference pose: neighbours that touch there are not counted,
    // however the arm folds. A body on a farther link that touches in the reference is a violation.
    var cell = rig([]);
    check(cell.clearance.violation([0.0, 0.0]) == null, "bars that meet at the elbow are not a collision");
    check(cell.clearance.violation([0.0, 2.0]) == null, "the elbow turning does not make its own bars collide");
    check(cell.clearance.pairCount() == 0, "no pair is checked in an arm of two neighbouring links");
  }

  static function testCrossingEdges():Void {
    // A thin bar across the forearm's path, at the forearm's height: no corner of either lies inside the other, but they cross.
    var bar = {name: "bar", vertices: box(0.45, 0.46, -0.2, 0.2, -0.006, 0.006)};
    var cell = rig([bar], 0.004);
    var crossing = cell.clearance.violation([0.0, 0.0]);
    check(crossing != null, "a bar across the arm is a collision although no corner of either is inside the other");
    check(crossing != null && crossing.distance <= 0.0, "they overlap");
    var below = rig([{name: "bar", vertices: box(0.45, 0.46, -0.2, 0.2, -0.02, -0.0121)}], 0.004);
    check(below.clearance.violation([0.0, 0.0]) == null, "the same bar 8 mm under the arm is clear");
  }

  static function testBoundsPreserveHullChecks():Void {
    // Compare the broad-phase result with direct hull distances throughout a
    // rotation. Long thin obstacles defeat spheres; rotated bars and near
    // contact ensure a box cannot replace the exact hull test.
    var obstacles = [
      box(-2.0, 2.0, 0.07, 0.09, -0.01, 0.01),
      box(0.44, 0.46, -2.0, 2.0, -0.01, 0.01),
      box(-2.0, 2.0, -2.0, 2.0, -0.04, -0.026)
    ];
    for (vertices in obstacles) {
      var model = new RobotModel("bounds-clearance");
      var base = model.addLink(new Link("base"));
      var moving = model.addLink(new Link("moving"));
      var obstacle = model.addLink(new Link("obstacle"));
      model.addJoint(new Joint("fixture", JointType.Fixed, base, obstacle));
      var joint = model.addJoint(new Joint("turn", JointType.Revolute, base, moving));
      joint.axis = [0.0, 0.0, 1.0];
      var flange = model.addFrame(new Frame("flange", moving));
      var group = new Manipulator(model, base.id, flange.id);
      var tool = box(0.3, 0.6, -0.025, 0.025, -0.025, 0.025);
      var clearance = new ArmClearance(group, [
        {name: "tool", link: moving.id, vertices: tool, tool: true},
        {name: "obstacle", link: obstacle.id, vertices: vertices, tool: false}
      ], [0.0]);
      for (sample in 0...61) {
        var q = [-Math.PI + sample * Math.PI / 30];
        var poses = group.linkPoses(q, [moving.id, obstacle.id]);
        function placed(points:Array<Float>, pose:Transform3):Array<Float> {
          var result:Array<Float> = [];
          for (i in 0...Std.int(points.length / 3)) {
            var point = pose.transformPoint(new Vec3(points[3*i], points[3*i+1], points[3*i+2]));
            result.push(point.x); result.push(point.y); result.push(point.z);
          }
          return result;
        }
        for (contact in [false, true]) {
          var margin = contact ? ArmClearance.CONTACT_MARGIN : ArmClearance.MARGIN;
          var exact = ConvexDistance.between(placed(tool, poses[0]), placed(vertices, poses[1]), margin);
          check((clearance.violation(q, contact) != null) == (exact < margin),
            "bounds preserve exact rotated hull clearance and contact margins");
        }
      }
    }
  }

  static function testSolidEdges():Void {
    var solid = new ConvexSolid(box(0.0, 0.1, 0.0, 0.2, 0.0, 0.3));
    check(solid.cornerPoints().length == 24, "a box has eight corners");
    check(solid.distanceBelow(0.2, 0.3, 0.4, 0.005) >= 0.005,
      "a separating face proves a distant point clear");
    var corner = solid.distanceBelow(0.103, 0.203, 0.303, 0.005);
    check(Math.abs(corner - Math.sqrt(3.0) * 0.003) < 1e-9,
      "below-threshold face distances still solve the true corner distance");
    check(Math.abs(solid.distanceBelow(0.05, 0.1, 0.15, 0.005) - solid.distance(0.05, 0.1, 0.15)) < 1e-12,
      "clearance cutoff preserves penetration depth");

    // The distance between hulls: two boxes 0.05 apart face to face, and 0.05 apart edge to edge (a diagonal gap of 0.05 * sqrt(2)).
    var a = box(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
    near(ConvexDistance.between(a, box(1.05, 2.0, 0.0, 1.0, 0.0, 1.0), 1.0), 0.05, 1e-9, "boxes apart face to face");
    near(ConvexDistance.between(a, box(1.05, 2.0, 1.05, 2.0, 0.0, 1.0), 1.0), 0.05 * Math.sqrt(2), 1e-9, "boxes apart edge to edge");
    near(ConvexDistance.between(a, box(1.05, 2.0, 1.05, 2.0, 1.05, 2.0), 1.0), 0.05 * Math.sqrt(3), 1e-9, "boxes apart corner to corner");
    check(ConvexDistance.between(a, box(0.5, 1.5, 0.5, 1.5, 0.5, 1.5), 1.0) == 0.0, "overlapping boxes are at no distance");
    check(ConvexDistance.between(a, box(1.0, 2.0, 0.0, 1.0, 0.0, 1.0), 1.0) == 0.0, "touching boxes are at no distance");
    // Two bars that cross, edges against faces, with no corner of either inside the other.
    check(ConvexDistance.between(box(0.0, 1.0, 0.4, 0.6, -0.01, 0.01), box(0.4, 0.6, 0.0, 1.0, -0.02, 0.02), 1.0) == 0.0, "crossing bars overlap");
    near(ConvexDistance.between(box(0.0, 1.0, 0.4, 0.6, -0.01, 0.01), box(0.4, 0.6, 0.0, 1.0, 0.03, 0.05), 1.0), 0.02, 1e-9, "crossing bars one above the other");
    // A cylinder (a 24-sided prism, as hulls of round parts are) against a box face, upright and tilted 45 degrees: the nearest
    // corner to the face is the distance. Prisms are symmetric and give flat simplices, which a naive search takes for overlap.
    for (tilt in [0.0, Math.PI / 4, 1.0]) {
      var prism:Array<Float> = [];
      for (end in [0.0, 0.05]) for (side in 0...24) {
        var angle = 2 * Math.PI * side / 24;
        var x = 0.013 * Math.cos(angle), y = 0.013 * Math.sin(angle), z = end;
        prism.push(x * Math.cos(tilt) + z * Math.sin(tilt) + 0.05);
        prism.push(y);
        prism.push(-x * Math.sin(tilt) + z * Math.cos(tilt) + 0.2);
      }
      var lowest = Math.POSITIVE_INFINITY;
      for (i in 0...48) lowest = Math.min(lowest, prism[3 * i + 2]);
      near(ConvexDistance.between(prism, box(-1.0, 1.0, -1.0, 1.0, lowest - 0.5, lowest - 0.003), 1.0), 0.003, 1e-9, 'a cylinder tilted $tilt rad over a face');
    }
    // Far apart: the search stops at a separating plane's gap, which is at least `enough`.
    check(ConvexDistance.between(a, box(5.0, 6.0, 0.0, 1.0, 0.0, 1.0), 0.1) >= 0.1, "far boxes are reported farther than the margin asked");
  }

  static function near(actual:Float, expected:Float, tolerance:Float, message:String):Void {
    assertions++;
    if (!(Math.abs(actual - expected) <= tolerance)) throw 'Assertion failed: $message (expected $expected, got $actual)';

  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'Assertion failed: $message';
  }
}
