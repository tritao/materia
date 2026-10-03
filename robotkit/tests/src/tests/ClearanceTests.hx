package tests;

import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.Manipulator;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
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
    testCrossingEdges();
    testSolidEdges();
    Sys.println('RobotKit clearance tests passed ($assertions assertions)');
    return assertions;
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
    check(hit != null && hit.distance < 0.0, "the arm overlaps the post");
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
    var lifted = rig([{name: "table", vertices: box(-0.2, 0.8, -0.5, 0.5, -0.2, -0.025 - 0.0005)}]);
    check(lifted.clearance.violation([0.0, 0.0], true) != null, "0.5 mm is closer than even the contact margin");
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

  static function testSolidEdges():Void {
    var solid = new ConvexSolid(box(0.0, 0.1, 0.0, 0.2, 0.0, 0.3));
    check(solid.cornerPoints().length == 24, "a box has eight corners");
    var points = solid.edgePoints(0.07);
    // 8 corners, and along the four edges of 0.1 (1 more each), of 0.2 (2) and of 0.3 (4) at that spacing.
    check(Std.int(points.length / 3) == 8 + 4 * (1 + 2 + 4), 'a box\'s edges are sampled, got ${Std.int(points.length / 3)} points');
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'Assertion failed: $message';
  }
}
