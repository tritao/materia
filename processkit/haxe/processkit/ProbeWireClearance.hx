package processkit;

import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ArmClearance.ClearanceBodyData;
import robotkit.manipulation.ArmClearance.ClearanceViolation;
import robotkit.model.LinkId;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.ConvexSolid;
import robotkit.tool.ConvexDistance;

private class WireObstacle {
  public final name:String;
  public final link:Int;
  public final corners:Array<Float>;
  public final centre:Vec3;
  public final radius:Float;
  public function new(body:ClearanceBodyData, link:Int) {
    name = body.name; this.link = link;
    corners = new ConvexSolid(body.vertices).cornerPoints();
    var sum = new Vec3();
    var count = Std.int(corners.length / 3);
    for (index in 0...count) sum = sum.add(new Vec3(corners[3 * index], corners[3 * index + 1], corners[3 * index + 2]));
    centre = sum.scale(1.0 / count);
    var far = 0.0;
    for (index in 0...count)
      far = Math.max(far, new Vec3(corners[3 * index], corners[3 * index + 1], corners[3 * index + 2]).sub(centre).norm());
    radius = far;
  }
}

/** The unconsumed CAD wire must stay clear in air and cannot penetrate solids during touch sensing. */
class ProbeWireClearance {
  public final arm:Manipulator;
  /** Extra radial reach of the conservative box beyond the CAD wire cylinder. */
  public final envelopeExcess:Float;
  final links:Array<LinkId>;
  final obstacles:Array<WireObstacle> = [];
  final corners:Array<Float> = [];
  final centre:Vec3;
  final radius:Float;
  final airMargin:Float;

  public function new(arm:Manipulator, link:LinkId, tip:Transform3, diameter:Float, stickout:Float,
      bodies:Array<ClearanceBodyData>, airMargin:Float = WeldPathPlanner.AIR_MARGIN) {
    if (arm == null || link == null || tip == null || bodies == null || !Math.isFinite(diameter) || !(diameter > 0) ||
        !Math.isFinite(stickout) || !(stickout > 0) || !Math.isFinite(airMargin) || !(airMargin > 0))
      throw "Probe wire clearance needs CAD wire dimensions, its link-local tip and finite air clearance";
    this.arm = arm; this.airMargin = airMargin;
    links = [link];
    var half = diameter / 2;
    envelopeExcess = (Math.sqrt(2) - 1) * half;
    // A circumscribed box is conservative for the cylindrical wire, including its end cap.
    for (x in [-half, half]) for (y in [-half, half]) for (z in [-stickout, 0.0]) {
      var point = tip.transformPoint(new Vec3(x, y, z));
      corners.push(point.x); corners.push(point.y); corners.push(point.z);
    }
    centre = tip.transformPoint(new Vec3(0, 0, -stickout / 2));
    radius = Math.sqrt(2 * half * half + stickout * stickout / 4);
    for (body in bodies) {
      // Tool skins are the wire's own rigid assembly; their clearance remains checked by ArmClearance.
      if (body.tool || body.link == link) continue;
      var index = links.indexOf(body.link);
      if (index < 0) { index = links.length; links.push(body.link); }
      obstacles.push(new WireObstacle(body, index));
    }
  }

  static function placed(points:Array<Float>, pose:Transform3):Array<Float> {
    var result:Array<Float> = [];
    for (index in 0...Std.int(points.length / 3)) {
      var point = pose.transformPoint(new Vec3(points[3 * index], points[3 * index + 1], points[3 * index + 2]));
      result.push(point.x); result.push(point.y); result.push(point.z);
    }
    return result;
  }

  /** A posture-independent distal lever bound is built from the rigid wire's extent at zero joints. */
  public function extentFrom(q:Array<Float>, origin:Vec3):Float {
    var pose = arm.linkPoses(q, [links[0]])[0];
    var far = 0.0;
    for (index in 0...Std.int(corners.length / 3)) {
      var at = pose.transformPoint(new Vec3(corners[3 * index], corners[3 * index + 1], corners[3 * index + 2]));
      far = Math.max(far, at.sub(origin).norm());
    }
    return far;
  }

  public function violation(q:Array<Float>, contact:Bool = false):Null<ClearanceViolation> {
    var poses = arm.linkPoses(q, links);
    var at = poses[0].transformPoint(centre);
    var wire:Null<Array<Float>> = null;
    var required = contact ? 0.0 : airMargin;
    for (body in obstacles) {
      var pose = poses[body.link];
      if (at.sub(pose.transformPoint(body.centre)).norm() - radius - body.radius > required) continue;
      if (wire == null) wire = placed(corners, poses[0]);
      var distance = ConvexDistance.between(cast wire, placed(body.corners, pose), required);
      if (distance < required || contact && distance <= 0)
        return {a: "contact sensing wire", b: body.name, distance: distance, required: required};
    }
    return null;
  }

  public function sweep(from:Array<Float>, to:Array<Float>, contact:Bool, maxJointStep:Float):Null<ClearanceViolation> {
    if (from == null || to == null || from.length != arm.group.count() || to.length != from.length ||
        !Math.isFinite(maxJointStep) || !(maxJointStep > 0))
      throw "Probe wire sweep needs matching arm configurations and a finite positive joint step";
    var steps = 1;
    for (joint in 0...from.length)
      steps = Std.int(Math.max(steps, Math.ceil(Math.abs(to[joint] - from[joint]) / maxJointStep)));
    for (step in 0...steps + 1) {
      var t = step / steps;
      var hit = violation([for (joint in 0...from.length) from[joint] + (to[joint] - from[joint]) * t], contact);
      if (hit != null) return hit;
    }
    return null;
  }
}
