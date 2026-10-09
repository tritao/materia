package robotkit.manipulation;

import robotkit.model.LinkId;
import robotkit.model.JointType;
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

private typedef ClearancePair = {
  var first:Int;
  var second:Int;
  var interfaceMargin:Null<Float>;
}

/** A named pair of part hulls with a mechanical interface clearance. A zero
    margin allows swept intervals to end at contact; an ambiguous stationary
    zero-distance interval fails closed because GJK cannot distinguish touch
    from overlap. Positive margins apply only to close reference interfaces. */
typedef ClearanceInterfaceData = {
	var first:String;
	var second:String;
	var margin:Float;
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
 * and bodies of neighbouring or drive-coupled links that touch (within `TOUCH`) in the reference configuration, are one rigid assembly or
 * a joint and never counted: they touch by design.
 *
 * The distance between two hulls is the exact distance between their convex hulls (`ConvexDistance`, from the hulls' corners;
 * zero when they touch or overlap), after rejecting pairs whose bounding spheres or axis-aligned bounds are far apart.
 *
 * Margins: a pair must be at least `margin` apart. A tool body against fixed bodies in the contact zone (the caller says
 * so: the wire tip is working a seam) need only keep `contactMargin`, since the nozzle sits a few
 * millimetres from the faces it welds. A caller may name the fixed bodies that permit contact,
 * so a vise grasp does not relax clearance at the enclosure or trays. The wire itself is no body.
 */
class ArmClearance {
  /** The distance bodies must keep, in metres. */
  public static inline var MARGIN:Float = 0.005;
  /** The distance a tool body must keep from fixed bodies where it works, in metres. */
  public static inline var CONTACT_MARGIN:Float = 0.0005;
  /** Clearance required at named interfaces between mechanical parts, in metres. */
  public static inline var INTERFACE_MARGIN:Float = 0.001;
  /** CAD mating surfaces can differ by a few micrometres after frame transforms. */
  public static inline var INTERFACE_TOUCH_TOLERANCE:Float = 0.00001;
  /** Numerical tolerance for conservative zero-margin interval contact, in metres. */
  public static inline var ZERO_MARGIN_INTERVAL_TOLERANCE:Float = 1e-9;
  /** Neighbouring links whose bodies are closer than this in the reference configuration touch by design, in metres. */
  public static inline var TOUCH:Float = 0.002;
  public final arm:KinematicGroup;
  public final margin:Float;
  public final contactMargin:Float;
  /** Null keeps the traditional contact policy; otherwise only named fixed bodies get it. */
  final contactBodies:Null<Map<String, Bool>>;
  final allowedContactBodyNames:Null<Array<String>>;
  final interfaceDefinitions:Array<ClearanceInterfaceData> = [];

  final reference:Array<Float>;
  final bodies:Array<ClearanceBody> = [];
  final links:Array<LinkId> = [];
  /** The checked body pairs and their resolved reference interface policy. */
  final pairs:Array<ClearancePair> = [];

  /**
   * `reference` is a configuration the arm stands in and nothing is meant to collide: neighbouring links whose bodies
   * touch there (a joint's housing against the next link) are not checked against one another.
   */
  public function new(arm:KinematicGroup, data:Array<ClearanceBodyData>, reference:Array<Float>, ?margin:Float = MARGIN,
      ?contactMargin:Float = CONTACT_MARGIN, ?allowedContactBodies:Array<String>,
      ?interfaces:Array<ClearanceInterfaceData>) {
    if (arm == null || data == null || reference == null) throw "Arm clearance needs an arm, its bodies and a reference configuration";
    if (!(margin >= 0.0) || !(contactMargin >= 0.0) || contactMargin > margin) throw "Clearance margins must not be negative, the contact margin at most the margin";
    this.arm = arm;this.reference=reference.copy();
    this.margin = margin;
    this.contactMargin = contactMargin;
    allowedContactBodyNames = allowedContactBodies == null ? null : allowedContactBodies.copy();
    var permitted:Null<Map<String, Bool>> = null;
    if (allowedContactBodies != null) {
      var names = new Map<String, Bool>();
      for (name in allowedContactBodies) names.set(name, true);
      permitted = names;
    }
    contactBodies = permitted;
    var declaredInterfaces = new Map<String, Float>();
    if (interfaces != null) for (entry in interfaces) {
      if (entry == null || entry.first == null || entry.second == null || entry.first == entry.second ||
          !Math.isFinite(entry.margin) || entry.margin < 0.0 || entry.margin > margin)
        throw "Clearance interfaces need distinct part names and a finite margin within the global margin";
      var key = interfaceKey(entry.first, entry.second);
      if (declaredInterfaces.exists(key)) throw 'Clearance interface "${entry.first}/${entry.second}" is duplicated';
      declaredInterfaces.set(key, entry.margin);
      interfaceDefinitions.push({first: entry.first, second: entry.second, margin: entry.margin});
    }
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
    // Drive followers also meet their logical carriage by design (for example
    // a lead screw inside its travelling nut). Only bodies touching at the
    // reference pose get the same exemption as an ordinary joint interface.
    for (coupling in arm.robot.couplings) {
      var leader:Null<robotkit.model.Joint> = null, follower:Null<robotkit.model.Joint> = null;
      for (joint in arm.robot.joints) {
        if (joint.id == coupling.leader) leader = joint;
        if (joint.id == coupling.follower) follower = joint;
      }
      if (leader == null || follower == null) continue;
      neighbours.set(leader.child.id + "\n" + follower.child.id, true);
      neighbours.set(follower.child.id + "\n" + leader.child.id, true);
      // A follower carried by the translating body meets a fixed drive member
      // (the rack). The reference-touch test below still exempts individual
      // body pairs, so another obstacle on the fixed link remains checked.
      if (follower.parent.id == leader.child.id) {
        neighbours.set(leader.parent.id + "\n" + follower.child.id, true);
        neighbours.set(follower.child.id + "\n" + leader.parent.id, true);
      }
    }
    var poses = arm.linkPoses(reference, links);
    for (i in 0...bodies.length) for (j in i + 1...bodies.length) {
      var a = bodies[i], b = bodies[j];
      if (!a.moving && !b.moving) continue;
      if (a.link == b.link) continue;
      var key = interfaceKey(a.name, b.name);
      var namedInterface = declaredInterfaces.get(key);
      var resolvedInterface:Null<Float> = null;
      var jointNeighbour = neighbours.exists(a.link + "\n" + b.link);
      if (jointNeighbour || namedInterface != null) {
        var referenceGap = distance(a, a.placed(poses[a.linkIndex]), poses[a.linkIndex],
          b, b.placed(poses[b.linkIndex]), poses[b.linkIndex], namedInterface == null ? TOUCH : margin);
        // A zero-margin interface explicitly permits contact throughout the
        // plan (for example, a carried workpiece against its gripper). Positive
        // mechanical fits get their smaller margin only while they are close at
        // the planning reference; a distant pair remains an ordinary obstacle.
        if (namedInterface != null && (namedInterface == 0.0 || referenceGap < margin))
          resolvedInterface = namedInterface;
        // A joint mating face may carry a few micrometres of transform-rounding gap.
        // Named interfaces only waive their margin within that CAD tolerance;
        // A close named pair with a real design gap keeps the interface margin.
        if ((namedInterface == null && jointNeighbour && referenceGap < TOUCH) ||
            (namedInterface != null && jointNeighbour && referenceGap <= INTERFACE_TOUCH_TOLERANCE)) continue;
      }
      pairs.push({first: i, second: j, interfaceMargin: resolvedInterface});
    }
  }

  /** How many pairs of bodies are checked. */
  /** Same collision geometry on a worker's independent compiled group. */
  public function withGroup(group:KinematicGroup):ArmClearance {
    return new ArmClearance(group,[for(body in bodies){name:body.name,link:body.link,
      vertices:body.corners.copy(),tool:body.tool}],reference,margin,contactMargin,allowedContactBodyNames,interfaceDefinitions);
  }

  public function pairCount():Int return pairs.length;

  /** How many bodies there are. */
  public function bodyCount():Int return bodies.length;

  /** The names of the bodies that move with the arm. */
  public function movingNames():Array<String> return [for (body in bodies) if (body.moving) body.name];

  function canContact(name:String):Bool {
    var allowed = contactBodies;
    return allowed == null || allowed.exists(name);
  }

  static function interfaceKey(first:String, second:String):String
    return Reflect.compare(first, second) <= 0 ? first + "\n" + second : second + "\n" + first;

  /** Certify a whole joint-coordinate box around q. Excursions bound every
   * coordinate's distance from q throughout the interval, including polynomial
   * interiors. Triangle bounds on link lengths and coupled coordinates bound
   * every hull point's travel; no obstacle can hide between sample poses. */
  public function intervalClear(q:Array<Float>, excursions:Array<Float>):Bool {
    if (q == null || excursions == null || q.length != arm.dofCount() || excursions.length != q.length)
      throw "Clearance interval needs one position and excursion per group coordinate";
    for (i in 0...q.length) if (!Math.isFinite(q[i]) || !Math.isFinite(excursions[i]) || excursions[i] < 0)
      throw "Clearance interval positions and nonnegative excursions must be finite";
    var ids = arm.jointIds(), model = arm.model;
    var state = arm.stateOf(q);
    var dofExcursions = [for (_ in 0...model.dofCount()) 0.0];
    for (i in 0...ids.length) dofExcursions[model.dofIndex(ids[i])] = excursions[i];
    var spans:Array<Array<Float>> = [];
    for (joint in 0...model.jointCount()) {
      var value = model.jointConstant[joint], excursion = 0.0;
      for (term in model.jointTermStart[joint]...model.jointTermStart[joint + 1]) {
        var dof = model.jointTermDof[term], scale = model.jointTermScale[term];
        value += scale * state.q[dof];
        excursion += Math.abs(scale) * dofExcursions[dof];
      }
      spans.push([value, excursion]);
    }
    var movement:Array<Float> = [];
    for (body in bodies) {
      var reach = 0.0;
      for (i in 0...Std.int(body.corners.length / 3)) {
        var x = body.corners[3 * i], y = body.corners[3 * i + 1], z = body.corners[3 * i + 2];
        reach = Math.max(reach, Math.sqrt(x * x + y * y + z * z));
      }
      var travel = 0.0, link = body.link;
      var visited = new Map<String, Bool>();
      while (true) {
        if (visited.exists(link)) throw "Clearance cannot bound a cyclic link tree";
        visited.set(link, true);
        var parents = [for (joint in arm.robot.joints) if (joint.child.id == link) joint];
        if (parents.length == 0) break;
        if (parents.length != 1) throw "Clearance needs one parent per link";
        var joint = parents[0], range = spans[model.jointIndex(joint.id)];
        reach += vectorLength(joint.childFramePosition);
        switch (joint.type) {
          case JointType.Prismatic:
            travel += range[1];
            reach += Math.abs(range[0]) + range[1];
          case JointType.Revolute, JointType.Continuous:
            travel += Math.min(2.0, range[1]) * reach;
          case JointType.Fixed:
          case JointType.Floating: throw "Clearance interval cannot certify a floating joint";
        }
        reach += vectorLength(joint.parentFramePosition);
        link = joint.parent.id;
      }
      movement.push(travel);
    }
    var poses = arm.linkPoses(q, links);
    var placed = [for (body in bodies) body.placed(poses[body.linkIndex])];
    for (pair in pairs) {
      var a = bodies[pair.first], b = bodies[pair.second];
      var jointInterface = pair.interfaceMargin;
      var required = (jointInterface == null ? margin : jointInterface) + movement[pair.first] + movement[pair.second];
      var lowerBound = distance(a, placed[pair.first], poses[a.linkIndex],
        b, placed[pair.second], poses[b.linkIndex], required);
      if (jointInterface == 0.0) {
        // GJK reports both contact and overlap as zero. With nonzero motion,
        // the distance lower bound must cover the complete relative travel;
        // with no motion, zero is ambiguous and cannot certify the interface.
        if (lowerBound + ZERO_MARGIN_INTERVAL_TOLERANCE < required ||
            required == 0.0 && lowerBound == 0.0) return false;
      } else if (lowerBound <= required) return false;
    }
    return true;
  }

  static function vectorLength(v:Array<Float>):Float
    return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

  /**
   * The first pair closer than it may be with the arm at `q`, or null. With `contact` the tool's bodies may come as close to
   * the fixed bodies as `contactMargin`. `wanted`, when given, is a smaller margin than the default for the others.
   */
  public function violation(q:Array<Float>, contact:Bool = false, ?wanted:Float, ?displacements:Array<Float>):Null<ClearanceViolation> {
    if(displacements!=null && displacements.length!=bodies.length)throw "Clearance displacement count differs from bodies";
    if(displacements!=null)for(value in displacements)if(!Math.isFinite(value) || value<0)throw "Clearance displacements must be finite and nonnegative";
    var poses = arm.linkPoses(q, links);
    var centres = [for (body in bodies) body.centreIn(poses[body.linkIndex])];
    var placed:Array<Null<Array<Float>>> = [for (_ in bodies) null];
    var bounds:Array<Null<Array<Float>>> = [for (_ in bodies) null];
    function corners(index:Int):Array<Float> {
      var found = placed[index];
      if (found != null) return found;
      var made = bodies[index].placed(poses[bodies[index].linkIndex]);
      placed[index] = made;
      return made;
    }
    function box(index:Int):Array<Float> {
      var found = bounds[index];
      if (found != null) return found;
      var points = corners(index);
      var made = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY,
        Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
      for (point in 0...Std.int(points.length / 3)) for (axis in 0...3) {
        var value = points[3 * point + axis];
        made[axis] = Math.min(made[axis], value);
        made[axis + 3] = Math.max(made[axis + 3], value);
      }
      bounds[index] = made;
      return made;
    }
    for (pair in pairs) {
      var a = bodies[pair.first], b = bodies[pair.second];
      var workingFace = (a.tool && !b.moving && canContact(b.name)) ||
        (b.tool && !a.moving && canContact(a.name));
      var requested = wanted == null ? margin : Math.min(margin, wanted);
      var jointInterface = pair.interfaceMargin;
      if (jointInterface != null) requested = Math.min(requested, jointInterface);
      var required = contact && workingFace ? contactMargin : requested;
      if(displacements!=null)required+=displacements[pair.first]+displacements[pair.second];
      // Whole bodies clear by more than the margin: bounding spheres.
      var gap = centres[pair.first].sub(centres[pair.second]).norm() - a.radius - b.radius;
      if (gap > required) continue;
      // Long rails and tables have large spheres. Their world bounds give
      // another conservative lower bound before the exact hull query.
      var boxA = box(pair.first), boxB = box(pair.second);
      var gap2 = 0.0;
      for (axis in 0...3) {
        var separation = Math.max(0.0, Math.max(boxA[axis] - boxB[axis + 3], boxB[axis] - boxA[axis + 3]));
        gap2 += separation * separation;
      }
      if (gap2 > required * required) continue;
      var apart = ConvexDistance.between(corners(pair.first), corners(pair.second), required);
      if (apart < required) return {a: a.name, b: b.name, distance: apart, required: required};
    }
    return null;
  }

  /** CL-D5 displacement bound for the same hull corners used by GJK.
   * Bounds are in fixed root coordinates; pair distances are unchanged by the
   * common task-frame transform. Every coupled term contributes its absolute
   * gain. Prismatic reach includes the complete supplied joint neighborhood. */
  public function displacementBounds(q:Array<Float>, errors:Array<Float>):Array<Float> {
    var bounds=new ClearanceMotionBounds(arm,q,errors);
    return [for(body in bodies)bounds.point(body.link,originRadius(body.corners))];
  }

  public function motionEnvelope(lower:Array<Float>,upper:Array<Float>):ClearanceMotionEnvelope
    return new ClearanceMotionEnvelope(arm,lower,upper,[for(body in bodies)body.link],[for(body in bodies)body.corners]);

  public function tcpDisplacementBound(q:Array<Float>,errors:Array<Float>):Float
    return new ClearanceMotionBounds(arm,q,errors).tcp();

  /** Acceptance audit of the analytical bound using these exact hull corners.
   * Both placements are brought to the fixed group root before comparison. */
  public function auditDisplacement(q:Array<Float>,perturbed:Array<Float>,errors:Array<Float>,?envelope:ClearanceMotionEnvelope):Float {
    var bounds=envelope==null ? displacementBounds(q,errors) : envelope.bounds(errors),a=arm.linkPoses(q,links),b=arm.linkPoses(perturbed,links),
      rootA=arm.workPose(q),rootB=arm.workPose(perturbed),ratio=0.0;
    for(i in 0...bodies.length){var body=bodies[i],pa=body.placed(rootA.compose(a[body.linkIndex])),pb=body.placed(rootB.compose(b[body.linkIndex]));
      for(k in 0...Std.int(pa.length/3)){var dx=pa[3*k]-pb[3*k],dy=pa[3*k+1]-pb[3*k+1],dz=pa[3*k+2]-pb[3*k+2],
        moved=Math.sqrt(dx*dx+dy*dy+dz*dz);
        if(moved>bounds[i]+1e-10)throw 'Hull displacement exceeds CL-D5 bound for ${body.name}: $moved > ${bounds[i]}';
        if(bounds[i]>0)ratio=Math.max(ratio,moved/bounds[i]);
      }
    }
    var moved=arm.tcpPose(q).translation.sub(arm.tcpPose(perturbed).translation).norm(),tcp=envelope==null ? tcpDisplacementBound(q,errors) : envelope.tcp(errors);
    if(moved>tcp+1e-10)throw 'TCP displacement exceeds contact neighborhood bound: $moved > $tcp';
    return ratio;
  }

  static function originRadius(corners:Array<Float>):Float {
    var radius=0.0;
    for(i in 0...Std.int(corners.length/3))radius=Math.max(radius,
      Math.sqrt(corners[3*i]*corners[3*i]+corners[3*i+1]*corners[3*i+1]+corners[3*i+2]*corners[3*i+2]));
    return radius;
  }

  /** Closest checked pair, including clear pairs, or null for an empty world.
   * Uses full hull queries; at the GJK iteration limit the distance remains
   * a conservative lower bound, as in ConvexDistance.between. */
  public function closest(q:Array<Float>,contact:Bool=false,?wanted:Float):Null<ClearanceViolation> {
    var poses=arm.linkPoses(q,links);
    var placed:Array<Null<Array<Float>>> = [for(_ in bodies)null];
    function corners(index:Int):Array<Float> {
      var found=placed[index];if(found!=null)return found;
      var made=bodies[index].placed(poses[bodies[index].linkIndex]);placed[index]=made;return made;
    }
    var best:Null<ClearanceViolation> = null;
    for(pair in pairs){var a=bodies[pair.first],b=bodies[pair.second];
      var required=contact && (a.tool && !b.moving || b.tool && !a.moving) ? contactMargin
        : (wanted==null ? margin : Math.min(margin,wanted));
      var apart=ConvexDistance.between(corners(pair.first),corners(pair.second),Math.POSITIVE_INFINITY);
      if(best==null || apart<best.distance)best={a:a.name,b:b.name,distance:apart,required:required};
    }
    return best;
  }

  /**
   * The first violation along the straight joint-space motion from `from` to `to`, sampled so that no joint turns more than
   * `maxJointStep` radians between samples, or null. `contactAt`, when supplied,
   * selects the margin independently at every sampled configuration.
   */
  public function sweep(from:Array<Float>, to:Array<Float>, ?contact:Bool = false, ?maxJointStep:Float = 0.02, ?wanted:Float,
      ?contactAt:Array<Float>->Bool,endpointsChecked:Bool=false):Null<ClearanceViolation> {
    if (from == null || to == null || from.length != to.length || !(maxJointStep > 0.0)) throw "A clearance sweep needs matching joint values and a positive step";
    var steps = 1;
    for (joint in 0...from.length) steps = Std.int(Math.max(steps, Math.ceil(Math.abs(to[joint] - from[joint]) / maxJointStep)));
    // Reuse endpoint checks only when the caller has just checked both under
    // the same world and contact policy; every interior sample is retained.
    for (step in (endpointsChecked ? 1 : 0)...(endpointsChecked ? steps : steps+1)) {
      var t = step / steps;
      var q = [for (joint in 0...from.length) from[joint] + (to[joint] - from[joint]) * t];
      var found = violation(q, contactAt == null ? contact : contactAt(q), wanted);
      if (found != null) return found;
    }
    return null;
  }

  /** Certify the complete straight joint-coordinate segment, subdividing until
   * the analytical hull-displacement bounds prove each interval clear. Unlike
   * `sweep`, this does not rely on a fixed sample spacing. */
  public function certifySegment(from:Array<Float>, to:Array<Float>, contact:Bool = false):Null<ClearanceViolation> {
    if (from == null || to == null || from.length != to.length || from.length != arm.dofCount())
      throw "Clearance segment needs matching group coordinates";
    var delta:Array<Float> = [];
    for (joint in 0...from.length) {
      if (!Math.isFinite(from[joint]) || !Math.isFinite(to[joint]))
        throw "Clearance segment coordinates must be finite";
      delta.push(to[joint] - from[joint]);
    }
    var endpointHit = violation(from, contact);
    if (endpointHit != null) return endpointHit;
    endpointHit = violation(to, contact);
    if (endpointHit != null) return endpointHit;
    var pending = [{start: 0.0, end: 1.0}], checked = 0;
    while (pending.length > 0) {
      if (++checked > 100000) throw "Clearance segment certification budget exceeded; split the motion";
      var interval = pending.pop();
      if (interval == null) throw "Clearance segment interval stack is empty";
      var middle = interval.start + (interval.end - interval.start) / 2;
      var q = [for (joint in 0...from.length) from[joint] + delta[joint] * middle];
      var hit = violation(q, contact);
      if (hit != null) return hit;
      var radius = (interval.end - interval.start) / 2;
      var excursions = [for (joint in 0...from.length) Math.abs(delta[joint]) * radius];
      if (intervalClear(q, excursions)) continue;
      if (middle == interval.start || middle == interval.end)
        throw "Clearance segment cannot certify the required margin";
      pending.push({start: middle, end: interval.end});
      pending.push({start: interval.start, end: middle});
    }
    return null;
  }


  /** Minimum hull clearance over the same sampled straight joint sweep as
   * sweep(). This is a sampled result, not a continuous collision bound. */
  public function closestSweep(from:Array<Float>,to:Array<Float>,contact:Bool=false,
      maxJointStep:Float=0.02,?wanted:Float):Null<ClearanceViolation> {
    if(from==null || to==null || from.length!=to.length || !Math.isFinite(maxJointStep) || maxJointStep<=0)
      throw "A closest-clearance sweep needs matching joints and a finite positive step";
    var steps=1;
    for(joint in 0...from.length){
      if(!Math.isFinite(from[joint]) || !Math.isFinite(to[joint]))throw "Clearance sweep joints must be finite";
      steps=Std.int(Math.max(steps,Math.ceil(Math.abs(to[joint]-from[joint])/maxJointStep)));
    }
    var best:Null<ClearanceViolation> = null;
    for(step in 0...steps+1){var t=step/steps;
      var found=closest([for(joint in 0...from.length)from[joint]+(to[joint]-from[joint])*t],contact,wanted);
      if(found!=null && (best==null || found.distance<best.distance))best=found;
    }
    return best;
  }

  /** The distance between two bodies whose corners are `cornersA` and `cornersB` in the arm's frame (`enough`: see `ConvexDistance`). */
  static function distance(a:ClearanceBody, cornersA:Array<Float>, ta:Transform3, b:ClearanceBody, cornersB:Array<Float>, tb:Transform3,
      enough:Float):Float {
    var gap = a.centreIn(ta).sub(b.centreIn(tb)).norm() - a.radius - b.radius;
    if (gap > enough) return gap;
    return ConvexDistance.between(cornersA, cornersB, enough);

  }
}
