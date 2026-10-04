package robotkit.manipulation;

import robotkit.model.LinkId;
import robotkit.spatial.Transform3;
import robotkit.tool.ConvexDistance;
import robotkit.tool.ConvexSolid;

/** A convex collision body of the robot's cell: the hull of a part (x, y, z triples in metres, in the frame of the link that carries it). */
typedef ClearanceBodyData = {
  var name:String;
  var link:LinkId;
  var vertices:Array<Float>;
  /** Part of the tool: it may come closer to the work than the arm may, within the contact zone (see `ArmClearance.violation`). */
  var tool:Bool;
}

/** Two bodies closer than they may be. `distance` is zero when they touch or overlap. */
typedef ClearanceViolation = {
  var a:String;
  var b:String;
  var distance:Float;
  var required:Float;
}

private class ClearanceBody {
  public final name:String;
  public final link:LinkId;
  public final linkIndex:Int;
  /** The hull's corners as x, y, z triples, in the link frame. */
  public final corners:Array<Float>;
  public final centre:Array<Float>;
  public final radius:Float;
  public final moving:Bool;
  public final tool:Bool;

  public function new(name:String, link:LinkId, linkIndex:Int, corners:Array<Float>, moving:Bool, tool:Bool) {
    this.name = name;
    this.link = link;
    this.linkIndex = linkIndex;
    this.corners = corners;
    this.moving = moving;
    this.tool = tool;
    var count = Std.int(corners.length / 3);
    var sum = [0.0, 0.0, 0.0];
    for (i in 0...count) for (axis in 0...3) sum[axis] += corners[3 * i + axis] / count;
    var far = 0.0;
    for (i in 0...count) {
      var dx = corners[3 * i] - sum[0], dy = corners[3 * i + 1] - sum[1], dz = corners[3 * i + 2] - sum[2];
      far = Math.max(far, Math.sqrt(dx * dx + dy * dy + dz * dz));
    }
    centre = sum;
    radius = far;
  }

  /** The corners in the frame `pose` leads to. */
  public function placed(pose:Transform3):Array<Float> {
    var q = pose.rotation, t = pose.translation;
    var xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    var xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z, wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    var r00 = 1 - 2 * (yy + zz), r01 = 2 * (xy - wz), r02 = 2 * (xz + wy);
    var r10 = 2 * (xy + wz), r11 = 1 - 2 * (xx + zz), r12 = 2 * (yz - wx);
    var r20 = 2 * (xz - wy), r21 = 2 * (yz + wx), r22 = 1 - 2 * (xx + yy);
    var result:Array<Float> = [];
    for (i in 0...Std.int(corners.length / 3)) {
      var x = corners[3 * i], y = corners[3 * i + 1], z = corners[3 * i + 2];
      result.push(r00 * x + r01 * y + r02 * z + t.x);
      result.push(r10 * x + r11 * y + r12 * z + t.y);
      result.push(r20 * x + r21 * y + r22 * z + t.z);
    }
    return result;
  }

  /** The hull's centre in the frame `pose` leads to. */
  public function centreIn(pose:Transform3):robotkit.spatial.Vec3
    return pose.transformPoint(new robotkit.spatial.Vec3(centre[0], centre[1], centre[2]));
}

/**
 * Whether an arm, with its tool, is clear of its surroundings at joint values, and along a motion: the check a planner
 * runs over the poses a program will take, until a collision world does it natively.
 *
 * Every collision body is a convex hull on a link (the hulls the simulation collides with). A body is *moving* when its
 * link is carried by one of the arm's joints, *fixed* otherwise (the pedestal, the table, the workpiece). Moving bodies
 * are checked against fixed bodies and against other moving bodies on links that are not neighbours. Bodies on one link,
 * and bodies of neighbouring links that touch (within `TOUCH`) in the reference configuration, are one rigid assembly or
 * a joint and never counted: they touch by design.
 *
 * The distance between two hulls is the exact distance between their convex hulls (`ConvexDistance`, from the hulls' corners;
 * zero when they touch or overlap), after a first rejection of pairs whose bounding spheres are far apart.
 *
 * Margins: a pair must be at least `margin` apart. A tool body against fixed bodies in the contact zone (the caller says
 * so: the wire tip is working a seam) need only keep `contactMargin`, since the nozzle sits a few
 * millimetres from the faces it welds. The wire itself is no body (it is millimetres of metal that is consumed).
 */
class ArmClearance {
  /** The distance bodies must keep, in metres. */
  public static inline var MARGIN:Float = 0.005;
  /** The distance a tool body must keep from fixed bodies where it works, in metres. */
  public static inline var CONTACT_MARGIN:Float = 0.0005;
  /** Neighbouring links whose bodies are closer than this in the reference configuration touch by design, in metres. */
  public static inline var TOUCH:Float = 0.002;
  public final arm:KinematicGroup;
  public final margin:Float;
  public final contactMargin:Float;

  final bodies:Array<ClearanceBody> = [];
  final links:Array<LinkId> = [];
  /** The pairs checked, as indices into `bodies`. */
  final pairs:Array<Array<Int>> = [];

  /**
   * `reference` is a configuration the arm stands in and nothing is meant to collide: neighbouring links whose bodies
   * touch there (a joint's housing against the next link) are not checked against one another.
   */
  public function new(arm:KinematicGroup, data:Array<ClearanceBodyData>, reference:Array<Float>, ?margin:Float = MARGIN,
      ?contactMargin:Float = CONTACT_MARGIN) {
    if (arm == null || data == null || reference == null) throw "Arm clearance needs an arm, its bodies and a reference configuration";
    if (!(margin >= 0.0) || !(contactMargin >= 0.0) || contactMargin > margin) throw "Clearance margins must not be negative, the contact margin at most the margin";
    this.arm = arm;
    this.margin = margin;
    this.contactMargin = contactMargin;
    var movingLinks = new Map<LinkId, Bool>();
    for (body in data) {
      if (body.vertices == null || body.vertices.length < 12) throw 'Clearance body "${body.name}" needs a hull';
      var linkIndex = links.indexOf(body.link);
      if (linkIndex < 0) {
        linkIndex = links.length;
        links.push(body.link);
        movingLinks.set(body.link, arm.moves(body.link));
      }
      bodies.push(new ClearanceBody(body.name, body.link, linkIndex, new ConvexSolid(body.vertices).cornerPoints(),
        movingLinks.get(body.link) == true, body.tool));
    }
    var neighbours = new Map<String, Bool>();
    for (joint in arm.robot.joints) if (joint != null && joint.parent != null && joint.child != null) {
      neighbours.set(joint.parent.id + "\n" + joint.child.id, true);
      neighbours.set(joint.child.id + "\n" + joint.parent.id, true);
    }
    var poses = arm.linkPoses(reference, links);
    for (i in 0...bodies.length) for (j in i + 1...bodies.length) {
      var a = bodies[i], b = bodies[j];
      if (!a.moving && !b.moving) continue;
      if (a.link == b.link) continue;
      if (neighbours.exists(a.link + "\n" + b.link) && distance(a, a.placed(poses[a.linkIndex]), poses[a.linkIndex], b, b.placed(poses[b.linkIndex]),
          poses[b.linkIndex], TOUCH) < TOUCH) continue;
      pairs.push([i, j]);
    }
  }

  /** How many pairs of bodies are checked. */
  public function pairCount():Int return pairs.length;

  /** How many bodies there are. */
  public function bodyCount():Int return bodies.length;

  /** The names of the bodies that move with the arm. */
  public function movingNames():Array<String> return [for (body in bodies) if (body.moving) body.name];

  /**
   * The first pair closer than it may be with the arm at `q`, or null. With `contact` the tool's bodies may come as close to
   * the fixed bodies as `contactMargin`. `wanted`, when given, is a smaller margin than the default for the others.
   */
  public function violation(q:Array<Float>, contact:Bool = false, ?wanted:Float):Null<ClearanceViolation> {
    var poses = arm.linkPoses(q, links);
    var placed:Array<Null<Array<Float>>> = [for (_ in bodies) null];
    function corners(index:Int):Array<Float> {
      var found = placed[index];
      if (found != null) return found;
      var made = bodies[index].placed(poses[bodies[index].linkIndex]);
      placed[index] = made;
      return made;
    }
    for (pair in pairs) {
      var a = bodies[pair[0]], b = bodies[pair[1]];
      var required = contact && (a.tool && !b.moving || b.tool && !a.moving) ? contactMargin : (wanted == null ? margin : Math.min(margin, wanted));
      // Whole bodies clear by more than the margin: bounding spheres.
      var gap = a.centreIn(poses[a.linkIndex]).sub(b.centreIn(poses[b.linkIndex])).norm() - a.radius - b.radius;
      if (gap > required) continue;
      var apart = ConvexDistance.between(corners(pair[0]), corners(pair[1]), required);
      if (apart < required) return {a: a.name, b: b.name, distance: apart, required: required};
    }
    return null;
  }

  /**
   * The first violation along the straight joint-space motion from `from` to `to`, sampled so that no joint turns more than
   * `maxJointStep` radians between samples, or null.
   */
  public function sweep(from:Array<Float>, to:Array<Float>, ?contact:Bool = false, ?maxJointStep:Float = 0.02, ?wanted:Float):Null<ClearanceViolation> {
    if (from == null || to == null || from.length != to.length || !(maxJointStep > 0.0)) throw "A clearance sweep needs matching joint values and a positive step";
    var steps = 1;
    for (joint in 0...from.length) steps = Std.int(Math.max(steps, Math.ceil(Math.abs(to[joint] - from[joint]) / maxJointStep)));
    for (step in 0...steps + 1) {
      var t = step / steps;
      var found = violation([for (joint in 0...from.length) from[joint] + (to[joint] - from[joint]) * t], contact, wanted);
      if (found != null) return found;
    }
    return null;
  }


  /** The distance between two bodies whose corners are `cornersA` and `cornersB` in the arm's frame (`enough`: see `ConvexDistance`). */
  static function distance(a:ClearanceBody, cornersA:Array<Float>, ta:Transform3, b:ClearanceBody, cornersB:Array<Float>, tb:Transform3,
      enough:Float):Float {
    var gap = a.centreIn(ta).sub(b.centreIn(tb)).norm() - a.radius - b.radius;
    if (gap > enough) return gap;
    return ConvexDistance.between(cornersA, cornersB, enough);

  }
}
