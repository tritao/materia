package robotkit.collision;

import collisionkit.CollisionGeometry;
import collisionkit.CollisionMargins;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionViolation;
import collisionkit.CollisionWorld;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.ArmClearance.ClearanceBodyData;
import robotkit.manipulation.ClearanceMotionBounds;
import robotkit.manipulation.ClearanceMotionEnvelope;
import robotkit.manipulation.ClearanceViolation;
import robotkit.manipulation.ClearanceWorld;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.LinkId;
import robotkit.spatial.Transform3;
import robotkit.tool.ConvexDistance;
import robotkit.tool.ConvexSolid;

/**
 * `ArmClearance`'s queries on a collisionkit world (COLLISION.md CL-D12,
 * CL4a). The bodies, margins and checked pairs are `ArmClearance`'s, decided
 * the same way (same link, neighbouring or drive-coupled links touching at
 * the reference, fixed pairs); only the geometry engine differs: each hull
 * is a convex object on its own collisionkit body, posed at its link, and
 * queries go to the world, with a violation's per-body displacement as the
 * world's per-body inflation and sweeps as batched pose sets.
 *
 * Groups carry the margins: moving bodies, moving tool bodies, fixed bodies
 * and fixed tool bodies; with `contact`, a tool group against a fixed group
 * keeps `contactMargin`. Distances are reported as `ArmClearance` does: zero
 * when bodies touch or overlap.
 */
class CollisionClearance implements ClearanceWorld {
  static inline var MOVING = 0;
  static inline var MOVING_TOOL = 1;
  static inline var FIXED = 2;
  static inline var FIXED_TOOL = 3;

  public final arm:KinematicGroup;
  public final margin:Float;
  public final contactMargin:Float;
  public final world:CollisionWorld;
  final makeWorld:Void->CollisionWorld;
  final data:Array<ClearanceBodyData>;
  final reference:Array<Float>;
  final names:Array<String> = [];
  final bodyLinks:Array<LinkId> = [];
  final corners:Array<Array<Float>> = [];
  final moving:Array<Bool> = [];
  final tool:Array<Bool> = [];
  final links:Array<LinkId> = [];
  final linkOf:Array<Int> = [];
  final bodies:Array<Int> = [];
  final objects:Array<Int> = [];
  var checkedPairs = 0;
  final poses:Array<Float> = [];

  /** As `ArmClearance`'s constructor, with `makeWorld` giving an empty world (and another per `withGroup`). */
  public function new(arm:KinematicGroup, data:Array<ClearanceBodyData>, reference:Array<Float>, makeWorld:Void->CollisionWorld,
      ?margin:Float = ArmClearance.MARGIN, ?contactMargin:Float = ArmClearance.CONTACT_MARGIN) {
    if (arm == null || data == null || reference == null || makeWorld == null)
      throw "Collision clearance needs an arm, its bodies, a reference configuration and a world";
    if (!(margin >= 0.0) || !(contactMargin >= 0.0) || contactMargin > margin)
      throw "Clearance margins must not be negative, the contact margin at most the margin";
    this.arm = arm;
    this.data = data;
    this.reference = reference.copy();
    this.makeWorld = makeWorld;
    this.margin = margin;
    this.contactMargin = contactMargin;
    world = makeWorld();
    var movingLinks = new Map<LinkId, Bool>();
    for (body in data) {
      if (body.vertices == null || body.vertices.length < 12) throw 'Clearance body "${body.name}" needs a hull';
      var linkIndex = links.indexOf(body.link);
      if (linkIndex < 0) {
        linkIndex = links.length;
        links.push(body.link);
        movingLinks.set(body.link, arm.moves(body.link));
      }
      var isMoving = movingLinks.get(body.link) == true;
      names.push(body.name);
      bodyLinks.push(body.link);
      linkOf.push(linkIndex);
      corners.push(new ConvexSolid(body.vertices).cornerPoints());
      moving.push(isMoving);
      tool.push(body.tool);
      var id = world.addBody(isMoving ? (body.tool ? MOVING_TOOL : MOVING) : (body.tool ? FIXED_TOOL : FIXED));
      if (!isMoving) world.setBodyStatic(id, true);
      bodies.push(id);
      objects.push(world.add(id, CollisionPose.identity(), CollisionGeometry.Convex(corners[corners.length - 1])));
    }
    for (_ in 0...7 * bodies.length) poses.push(0.0);
    declarePairs();
  }

  /** `ArmClearance`'s pair rules: one link, both fixed, or neighbours touching at the reference are not checked. */
  function declarePairs():Void {
    var neighbours = new Map<String, Bool>();
    for (joint in arm.robot.joints) if (joint != null && joint.parent != null && joint.child != null) {
      neighbours.set(joint.parent.id + "\n" + joint.child.id, true);
      neighbours.set(joint.child.id + "\n" + joint.parent.id, true);
    }
    for (coupling in arm.robot.couplings) {
      var leader:Null<robotkit.model.Joint> = null, follower:Null<robotkit.model.Joint> = null;
      for (joint in arm.robot.joints) {
        if (joint.id == coupling.leader) leader = joint;
        if (joint.id == coupling.follower) follower = joint;
      }
      if (leader == null || follower == null) continue;
      neighbours.set(leader.child.id + "\n" + follower.child.id, true);
      neighbours.set(follower.child.id + "\n" + leader.child.id, true);
      if (follower.parent.id == leader.child.id) {
        neighbours.set(leader.parent.id + "\n" + follower.child.id, true);
        neighbours.set(follower.child.id + "\n" + leader.parent.id, true);
      }
    }
    var linkPoses = arm.linkPoses(reference, links);
    for (i in 0...bodies.length) for (j in i + 1...bodies.length) {
      if (!moving[i] && !moving[j]) continue;
      if (bodyLinks[i] == bodyLinks[j]) {
        world.setBodyRule(bodies[i], bodies[j], CollisionPairRule.Allow, CollisionPairStatus.Rigid);
        continue;
      }
      if (neighbours.exists(bodyLinks[i] + "\n" + bodyLinks[j])
          && touching(placed(corners[i], linkPoses[linkOf[i]]), placed(corners[j], linkPoses[linkOf[j]]))) {
        world.setBodyRule(bodies[i], bodies[j], CollisionPairRule.Allow, CollisionPairStatus.Adjacent);
        continue;
      }
      checkedPairs++;
    }
  }

  public function group():KinematicGroup return arm;

  public function withGroup(group:KinematicGroup):ClearanceWorld
    return new CollisionClearance(group, data, reference, makeWorld, margin, contactMargin);

  public function pairCount():Int return checkedPairs;

  public function bodyCount():Int return bodies.length;

  public function movingNames():Array<String> return [for (i in 0...names.length) if (moving[i]) names[i]];

  public function violation(q:Array<Float>, ?contactFlag:Bool, ?wanted:Float, ?displacements:Array<Float>):Null<ClearanceViolation> {
    // Defaults are applied here: through `ClearanceWorld` an omitted argument arrives as null.
    var contact = contactFlag == true;
    if (displacements != null && displacements.length != bodies.length) throw "Clearance displacement count differs from bodies";
    if (displacements != null) for (value in displacements)
      if (!Math.isFinite(value) || value < 0) throw "Clearance displacements must be finite and nonnegative";
    place(q);
    return named(world.violation(margins(contact, wanted), displacements));
  }

  public function closest(q:Array<Float>, ?contactFlag:Bool, ?wanted:Float):Null<ClearanceViolation> {
    var contact = contactFlag == true;
    place(q);
    return named(world.closest(margins(contact, wanted)));
  }

  public function sweep(from:Array<Float>, to:Array<Float>, ?contactFlag:Bool, ?step:Float, ?wanted:Float,
      ?contactAt:Array<Float>->Bool, ?endpoints:Bool):Null<ClearanceViolation> {
    // Defaults are applied here: through `ClearanceWorld` an omitted argument arrives as null.
    var contact = contactFlag == true, maxJointStep:Float = step == null ? 0.02 : step, endpointsChecked = endpoints == true;
    if (from == null || to == null || from.length != to.length || !(maxJointStep > 0.0))
      throw "A clearance sweep needs matching joint values and a positive step";
    var steps = 1;
    for (joint in 0...from.length) steps = Std.int(Math.max(steps, Math.ceil(Math.abs(to[joint] - from[joint]) / maxJointStep)));
    var samples = [for (step in (endpointsChecked ? 1 : 0)...(endpointsChecked ? steps : steps + 1)) {
      var t = step / steps;
      [for (joint in 0...from.length) from[joint] + (to[joint] - from[joint]) * t];
    }];
    if (contactAt != null) {
      // The margins can change at every sample: one query each.
      for (q in samples) {
        var found = violation(q, contactAt(q), wanted);
        if (found != null) return found;
      }
      return null;
    }
    if (samples.length == 0) return null;
    // Every sample's poses in one batched call.
    var sets:Array<Float> = [];
    for (q in samples) {
      fill(q);
      for (value in poses) sets.push(value);
    }
    return named(world.firstViolation(sets, margins(contact, wanted)));
  }

  public function closestSweep(from:Array<Float>, to:Array<Float>, ?contactFlag:Bool, ?step:Float,
      ?wanted:Float):Null<ClearanceViolation> {
    var contact = contactFlag == true, maxJointStep:Float = step == null ? 0.02 : step;
    if (from == null || to == null || from.length != to.length || !Math.isFinite(maxJointStep) || maxJointStep <= 0)
      throw "A closest-clearance sweep needs matching joints and a finite positive step";
    var steps = 1;
    for (joint in 0...from.length) {
      if (!Math.isFinite(from[joint]) || !Math.isFinite(to[joint])) throw "Clearance sweep joints must be finite";
      steps = Std.int(Math.max(steps, Math.ceil(Math.abs(to[joint] - from[joint]) / maxJointStep)));
    }
    var best:Null<ClearanceViolation> = null;
    for (step in 0...steps + 1) {
      var t = step / steps;
      var found = closest([for (joint in 0...from.length) from[joint] + (to[joint] - from[joint]) * t], contact, wanted);
      if (found != null && (best == null || found.distance < best.distance)) best = found;
    }
    return best;
  }

  public function displacementBounds(q:Array<Float>, errors:Array<Float>):Array<Float> {
    var bounds = new ClearanceMotionBounds(arm, q, errors);
    return [for (i in 0...bodies.length) bounds.point(bodyLinks[i], originRadius(corners[i]))];
  }

  public function motionEnvelope(lower:Array<Float>, upper:Array<Float>):ClearanceMotionEnvelope
    return new ClearanceMotionEnvelope(arm, lower, upper, bodyLinks.copy(), corners);

  public function tcpDisplacementBound(q:Array<Float>, errors:Array<Float>):Float
    return new ClearanceMotionBounds(arm, q, errors).tcp();

  public function auditDisplacement(q:Array<Float>, perturbed:Array<Float>, errors:Array<Float>, ?envelope:ClearanceMotionEnvelope):Float {
    var bounds = envelope == null ? displacementBounds(q, errors) : envelope.bounds(errors);
    var a = arm.linkPoses(q, links), b = arm.linkPoses(perturbed, links), rootA = arm.workPose(q), rootB = arm.workPose(perturbed);
    var ratio = 0.0;
    for (i in 0...bodies.length) {
      var pa = placed(corners[i], rootA.compose(a[linkOf[i]])), pb = placed(corners[i], rootB.compose(b[linkOf[i]]));
      for (k in 0...Std.int(pa.length / 3)) {
        var dx = pa[3 * k] - pb[3 * k], dy = pa[3 * k + 1] - pb[3 * k + 1], dz = pa[3 * k + 2] - pb[3 * k + 2];
        var moved = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (moved > bounds[i] + 1e-10) throw 'Hull displacement exceeds CL-D5 bound for ${names[i]}: $moved > ${bounds[i]}';
        if (bounds[i] > 0) ratio = Math.max(ratio, moved / bounds[i]);
      }
    }
    var moved = arm.tcpPose(q).translation.sub(arm.tcpPose(perturbed).translation).norm();
    var tcp = envelope == null ? tcpDisplacementBound(q, errors) : envelope.tcp(errors);
    if (moved > tcp + 1e-10) throw 'TCP displacement exceeds contact neighborhood bound: $moved > $tcp';
    return ratio;
  }

  function margins(contact:Bool, wanted:Null<Float>):CollisionMargins {
    var m = wanted == null ? margin : Math.min(margin, wanted);
    var table = new CollisionMargins(4, m);
    if (contact) for (t in [MOVING_TOOL, FIXED_TOOL]) for (f in [FIXED, FIXED_TOOL]) table.set(t, f, contactMargin);
    return table;
  }

  /** Every body's pose (its link's, in the arm's reference frame) for `q`, into `poses`. */
  function fill(q:Array<Float>):Void {
    var linkPoses = arm.linkPoses(q, links);
    for (i in 0...bodies.length) {
      var pose = linkPoses[linkOf[i]], t = pose.translation, r = pose.rotation, o = 7 * i;
      poses[o] = t.x; poses[o + 1] = t.y; poses[o + 2] = t.z;
      poses[o + 3] = r.x; poses[o + 4] = r.y; poses[o + 5] = r.z; poses[o + 6] = r.w;
    }
  }

  function place(q:Array<Float>):Void {
    fill(q);
    world.setBodyPoses(0, poses);
  }

  function named(found:Null<CollisionViolation>):Null<ClearanceViolation> {
    if (found == null) return null;
    var a = objects.indexOf(found.a), b = objects.indexOf(found.b);
    return {a: names[a], b: names[b], distance: Math.max(0.0, found.distance), required: found.required};
  }

  static function touching(a:Array<Float>, b:Array<Float>):Bool
    return ConvexDistance.between(a, b, ArmClearance.TOUCH) < ArmClearance.TOUCH;

  static function placed(points:Array<Float>, pose:Transform3):Array<Float> {
    var out:Array<Float> = [];
    for (i in 0...Std.int(points.length / 3)) {
      var p = pose.transformPoint(new robotkit.spatial.Vec3(points[3 * i], points[3 * i + 1], points[3 * i + 2]));
      out.push(p.x);
      out.push(p.y);
      out.push(p.z);
    }
    return out;
  }

  static function originRadius(points:Array<Float>):Float {
    var radius = 0.0;
    for (i in 0...Std.int(points.length / 3))
      radius = Math.max(radius, Math.sqrt(points[3 * i] * points[3 * i] + points[3 * i + 1] * points[3 * i + 1] + points[3 * i + 2] * points[3 * i + 2]));
    return radius;
  }
}
