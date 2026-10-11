package robotkit.collision;

import collisionkit.CollisionGeometry;
import collisionkit.CollisionMargins;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionViolation;
import collisionkit.CollisionWorld;
import collisionkit.CollisionDescription;
import collisionkit.DescribedObject;
import robotkit.model.CollisionShape;
import robotkit.manipulation.ClearanceChange;
import robotkit.manipulation.ClearanceScene;
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
 * Whether an arm, with its tool, is clear of its surroundings at joint
 * values and along a motion, on a collisionkit world (COLLISION.md CL-D12,
 * CL4a; PP10 retired the Haxe GJK `ArmClearance` it replaced, with equal
 * answers on the weld, rail and G17 cells).
 *
 * Each described shape keeps its geometry, pose and inflation on its link
 * or tool body. Authored fixed obstacles retain their world poses.
 * A body is *moving* when its link is carried by one of the arm's joints,
 * *fixed* otherwise. Bodies on one link, both fixed, or on neighbouring or
 * drive-coupled links touching (within `TOUCH`) at the reference are never
 * checked: they touch by design. Queries ask the world in batches.
 *
 * Groups carry the margins: moving bodies, moving tool bodies, fixed bodies
 * and fixed tool bodies; with `contact`, a tool group against a fixed group
 * keeps `contactMargin`. Distances are zero when bodies touch or overlap.
 *
 * As a `ClearanceScene` it replays a program's changes (CL-D9): a hull can
 * be attached to a link where it is (a grasp) and detached to the arm's root
 * (a place), and a pair can get a margin of its own or be allowed to touch
 * while a contact window is open. Each window pair gets two groups of its
 * own, so the margins stay per group pair.
 */
class CollisionClearance implements ClearanceWorld implements ClearanceScene {
  /** The distance bodies must keep, in metres. */
  public static inline var MARGIN:Float = 0.005;
  /** The distance a tool body must keep from fixed bodies where it works, in metres. */
  public static inline var CONTACT_MARGIN:Float = 0.0005;
  /** Neighbouring links whose bodies are closer than this in the reference configuration touch by design, in metres. */
  public static inline var TOUCH:Float = 0.002;
  static inline var MOVING = 0;
  static inline var MOVING_TOOL = 1;
  static inline var FIXED = 2;
  static inline var FIXED_TOOL = 3;

  public final arm:KinematicGroup;
  public final margin:Float;
  public final contactMargin:Float;
  public var world(default, null):CollisionWorld;
  final makeWorld:Void->CollisionWorld;
  final data:CollisionDescription;
  final reference:Array<Float>;
  final geometries:Array<CollisionGeometry> = [];
  final offsets:Array<CollisionPose> = [];
  final inflations:Array<Float> = [];
  final names:Array<String> = [];
  final bodyLinks:Array<LinkId> = [];
  final corners:Array<Array<Float>> = [];
  final moving:Array<Bool> = [];
  final tool:Array<Bool> = [];
  final environment:Array<Bool> = [];
  final links:Array<LinkId> = [];
  final linkOf:Array<Int> = [];
  final bodies:Array<Int> = [];
  final objects:Array<Int> = [];
  var checkedPairs = 0;
  final poses:Array<Float> = [];
  /** The rule each checked-or-not pair was built with ("i\nj" -> reason, absent when checked). */
  final baseRules = new Map<String, CollisionPairStatus>();
  final baseChecks = new Map<String, Bool>();
  /** Open windows: their two hulls and margin (null: contact allowed). */
  final windows:Array<{a:Int, b:Int, margin:Null<Float>}> = [];

  /** Exact placement clearance against an authored obstacle description. */
  public static function placementClear(description:CollisionDescription, position:robotkit.spatial.Vec3,
      clearance:Float, makeWorld:Void->CollisionWorld):Bool {
    var scene = description.copy();
    var body = scene.addBody("placement probe", 1, false,
      CollisionPose.translation(position.x,position.y,position.z));
    scene.addObject("placement probe", body, CollisionPose.identity(), Sphere(1e-12));
    var world = makeWorld();
    try {
      scene.build(world);
      var groups = 1;
      for (body in scene.bodies) groups = Std.int(Math.max(groups, body.group + 1));
      var clear = world.violation(new collisionkit.CollisionMargins(groups, clearance)) == null;
      world.dispose();
      return clear;
    } catch (error:Dynamic) { world.dispose(); throw error; }
  }

  /** Combines scene fragments; geometry and role information stay in the description. */
  public static function combine(parts:Array<CollisionDescription>):CollisionDescription {
    var result = new CollisionDescription();
    for (part in parts) result.append(part);
    return result;
  }

  /** Shapes on a named link, or on a tool body following that link (group 1). */
  public static function shapes(link:String, shapes:Array<CollisionShape>, isTool:Bool = false):CollisionDescription {
    var result = new CollisionDescription();
    var body = result.addBody(link, isTool ? 1 : 0, false);
    for (i in 0...shapes.length) {
      var shape = shapes[i];
      var invalid = shape.validate(); if (invalid != null) throw invalid;
      var p = shape.position, q = shape.rotation;
      result.addObject(shape.name == "" ? link + "/shape/" + i : shape.name, body, new CollisionPose(p[0],p[1],p[2],q[0],q[1],q[2],q[3]),
        shape.geometry, shape.inflation, "authored shape");
    }
    return result;
  }

  public static function convex(name:String, link:String, vertices:Array<Float>, isTool:Bool = false):CollisionDescription {
    var shape = new CollisionShape(Convex(vertices));
    shape.name = name;
    return shapes(link, [shape], isTool);
  }

  public static function link(data:CollisionDescription, object:DescribedObject):String {
    if (object.body < 0) throw "A carried collision shape requires a body";
    return data.bodies[object.body].name;
  }

  public static function isTool(data:CollisionDescription, object:DescribedObject):Bool
    return object.body >= 0 && data.bodies[object.body].group == 1;

  static function transform(p:CollisionPose):Transform3
    return new Transform3(new robotkit.spatial.Vec3(p.x,p.y,p.z), new robotkit.spatial.Quat(p.qx,p.qy,p.qz,p.qw));

  /** Finite, link-local points for motion bounds; queries retain exact geometry. */
  public static function vertices(object:DescribedObject):Array<Float> {
    switch object.geometry {
      case Convex(points) if (object.inflation == 0):
        return placed(new ConvexSolid(points).cornerPoints(), transform(object.offset));
      case _:
        var bounds = collisionkit.CollisionBounds.of(object.geometry, object.offset, object.inflation);
        var c = bounds.centre, h = bounds.halfExtents;
        return [for (i in 0...8) for (a in 0...3)
          (a == 0 ? c.x : a == 1 ? c.y : c.z) +
          ((i & (1 << a)) == 0 ? -1 : 1) * (a == 0 ? h.x : a == 1 ? h.y : h.z)];
    }
  }

  /** The arm's bodies (`data`) and the configuration whose touching neighbours are allowed (`reference`); `makeWorld` gives an empty world (and another per `withGroup`). */
  public function new(arm:KinematicGroup, data:CollisionDescription, reference:Array<Float>, makeWorld:Void->CollisionWorld,
      ?margin:Float = MARGIN, ?contactMargin:Float = CONTACT_MARGIN) {
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
    build();
  }

  /** The world as described by `data`, with no change applied. */
  function build():Void {
    world = makeWorld();
    names.resize(0);
    bodyLinks.resize(0);
    links.resize(0);
    corners.resize(0);
    moving.resize(0);
    tool.resize(0);
    environment.resize(0);
    linkOf.resize(0);
    bodies.resize(0);
    objects.resize(0);
    poses.resize(0);
    windows.resize(0);
    geometries.resize(0);
    offsets.resize(0);
    inflations.resize(0);
    baseRules.clear();
    baseChecks.clear();
    checkedPairs = 0;
    var movingLinks = new Map<LinkId, Bool>();
    for (object in data.objects) {
      var source = object.body < 0 ? null : data.bodies[object.body];
      var on = source == null ? arm.rootLink : source.name;
      var offset = object.offset;
      if (source != null && source.fixed) {
        if (arm.model.bodyIndex(on) < 0) on = arm.rootLink;
        var at = transform(source.pose).compose(transform(offset));
        var p = at.translation, q = at.rotation;
        offset = new CollisionPose(p.x,p.y,p.z,q.x,q.y,q.z,q.w);
      }
      if (arm.model.bodyIndex(on) < 0) throw 'Clearance shape "${object.name}" names unknown link "$on"';
      var linkIndex = links.indexOf(on);
      if (linkIndex < 0) { linkIndex = links.length; links.push(on); movingLinks.set(on, arm.moves(on)); }
      names.push(object.name);
      bodyLinks.push(on);
      linkOf.push(linkIndex);
      var posed = new DescribedObject(object.name, object.body, offset, object.geometry, object.inflation, object.source);
      var points = vertices(posed);
      corners.push(points);
      moving.push(movingLinks.get(on) == true);
      tool.push(isTool(data, object));
      environment.push(source == null || source.fixed);
      // Keep the existing convex support ordering while general shapes stay exact.
      var geometry = object.geometry;
      switch geometry {
        case Convex(_) if (object.inflation == 0): geometry = Convex(points); offset = CollisionPose.identity();
        case _:
      }
      geometries.push(geometry); offsets.push(offset); inflations.push(object.inflation);
      var id = world.addBody(roleOf(bodies.length));
      if (!moving[moving.length - 1]) world.setBodyStatic(id, true);
      bodies.push(id);
      var added = world.add(id, offset, geometry);
      if (object.inflation > 0) world.setInflation(added, object.inflation);
      objects.push(added);
    }
    for (_ in 0...7 * bodies.length) poses.push(0.0);
    declarePairs();
    declaredRules();
  }

  /** Explicit description rules override structural defaults, as in CollisionDescription.build. */
  function declaredRules():Void {
    function apply(i:Int, j:Int, rule:CollisionPairRule, reason:CollisionPairStatus):Void {
      if (i == j || rule == CollisionPairRule.Default) return;
      var key = i < j ? '$i\n$j' : '$j\n$i';
      if (rule == CollisionPairRule.Check) { baseRules.remove(key); baseChecks.set(key, true); }
      else { baseChecks.remove(key); baseRules.set(key, reason); }
      world.setBodyRule(bodies[i], bodies[j], rule, reason);
    }
    for (rule in data.bodyRules)
      for (i in 0...data.objects.length) for (j in i + 1...data.objects.length)
        if ((data.objects[i].body == rule.a && data.objects[j].body == rule.b) ||
            (data.objects[i].body == rule.b && data.objects[j].body == rule.a))
          apply(i,j,rule.rule,rule.reason);
    for (rule in data.objectRules) apply(rule.a,rule.b,rule.rule,rule.reason);
    if (data.bodyRules.length > 0 || data.objectRules.length > 0) {
      checkedPairs = 0;
      for (i in 0...objects.length) for (j in i + 1...objects.length)
        if (world.pairStatus(objects[i],objects[j]) == CollisionPairStatus.Checked) checkedPairs++;
    }
  }

  public function reset():Void {
    world.dispose();
    build();
  }

  public function apply(change:ClearanceChange, q:Array<Float>):Void {
    switch change {
      case Attach(body, link):
        move(hull(body), link, q);
      case Detach(body):
        move(hull(body), arm.rootLink, q);
      case OpenWindow(a, b, margin):
        var i = hull(a), j = hull(b);
        for (window in windows) if (window.a == i || window.b == i || window.a == j || window.b == j)
          throw 'Clearance window $a / $b overlaps an open window';
        if (margin != null && !(margin >= 0.0)) throw 'Clearance window $a / $b needs a nonnegative margin';
        windows.push({a: i, b: j, margin: margin});
        regroup();
        if (margin == null) world.setBodyRule(bodies[i], bodies[j], CollisionPairRule.Allow, CollisionPairStatus.ProcessContact);
      case CloseWindow(a, b):
        var i = hull(a), j = hull(b);
        var found = -1;
        for (k in 0...windows.length) if ((windows[k].a == i && windows[k].b == j) || (windows[k].a == j && windows[k].b == i)) found = k;
        if (found < 0) throw 'No clearance window is open for $a / $b';
        windows.splice(found, 1);
        restoreRule(i, j);
        regroup();
    }
  }

  function hull(name:String):Int {
    var index = names.indexOf(name);
    if (index < 0) throw 'Clearance world has no body "$name"';
    return index;
  }

  /** Moves hull `i` onto `link` where it is at `q`. */
  function move(i:Int, link:LinkId, q:Array<Float>):Void {
    var target = links.indexOf(link);
    if (target < 0) {
      target = links.length;
      links.push(link);
    }
    var at = arm.linkPoses(q, links);
    var inCell = placed(corners[i], at[linkOf[i]]);
    corners[i] = placed(inCell, at[target].inverse());
    bodyLinks[i] = link;
    moving[i] = arm.moves(link);
    var offset = at[target].inverse().compose(at[linkOf[i]]).compose(transform(offsets[i]));
    var p = offset.translation, r = offset.rotation;
    offsets[i] = new CollisionPose(p.x,p.y,p.z,r.x,r.y,r.z,r.w);
    switch geometries[i] {
      case Convex(_) if (inflations[i] == 0): geometries[i] = Convex(corners[i]); offsets[i] = CollisionPose.identity();
      case _:
    }
    world.remove(objects[i]);
    objects[i] = world.add(bodies[i], offsets[i], geometries[i]);
    if (inflations[i] > 0) world.setInflation(objects[i], inflations[i]);
    linkOf[i] = target;
    world.setBodyStatic(bodies[i], !moving[i]);
    // A moved hull is rigid with the hulls on its new link and checked against everything else.
    for (j in 0...bodies.length) if (j != i) {
      var rule = bodyLinks[j] == link ? CollisionPairStatus.Rigid : null;
      var key = i < j ? '$i\n$j' : '$j\n$i';
      if (rule == null) baseRules.remove(key) else baseRules.set(key, rule);
      restoreRule(i, j);
    }
    regroup();
  }

  function restoreRule(i:Int, j:Int):Void {
    var key = i < j ? '$i\n$j' : '$j\n$i';
    if (baseChecks.exists(key)) {
      world.setBodyRule(bodies[i], bodies[j], CollisionPairRule.Check, CollisionPairStatus.Checked);
      return;
    }
    var base = baseRules.get(key);
    if (base == null) world.setBodyRule(bodies[i], bodies[j], CollisionPairRule.Default, CollisionPairStatus.Checked);
    else world.setBodyRule(bodies[i], bodies[j], CollisionPairRule.Allow, base);
  }

  /** Every hull's group: its role, or its window's own. */
  function regroup():Void {
    for (i in 0...bodies.length) world.setBodyGroup(bodies[i], groupOf(i));
  }

  function roleOf(i:Int):Int return moving[i] ? (tool[i] ? MOVING_TOOL : MOVING) : (tool[i] ? FIXED_TOOL : FIXED);

  function groupOf(i:Int):Int {
    for (k in 0...windows.length) {
      if (windows[k].a == i) return 4 + 2 * k;
      if (windows[k].b == i) return 5 + 2 * k;
    }
    return roleOf(i);
  }

  /** The pair rules: one link, both fixed, or neighbours touching at the reference are not checked. */
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
      if (bodyLinks[i] == bodyLinks[j] && !environment[i] && !environment[j]) {
        world.setBodyRule(bodies[i], bodies[j], CollisionPairRule.Allow, CollisionPairStatus.Rigid);
        baseRules.set('$i\n$j', CollisionPairStatus.Rigid);
        continue;
      }
      if (!environment[i] && !environment[j] && neighbours.exists(bodyLinks[i] + "\n" + bodyLinks[j])
          && touching(placed(corners[i], linkPoses[linkOf[i]]), placed(corners[j], linkPoses[linkOf[j]]))) {
        world.setBodyRule(bodies[i], bodies[j], CollisionPairRule.Allow, CollisionPairStatus.Adjacent);
        baseRules.set('$i\n$j', CollisionPairStatus.Adjacent);
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
    if (samples.length == 0 || checkedPairs == 0) return null;
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

  /** Sampling step from a finite-box bound on all carried geometry. */
  public function jointStep(from:Array<Float>, to:Array<Float>, maxToolStep:Float):Float {
    if (!Math.isFinite(maxToolStep) || maxToolStep <= 0) throw "Tool sampling distance must be positive";
    var errors = [for (i in 0...from.length) Math.abs(to[i] - from[i])];
    var lower = [for (i in 0...from.length) Math.min(from[i],to[i])];
    var upper = [for (i in 0...from.length) Math.max(from[i],to[i])];
    var motion = 0.0, delta = 0.0;
    for (bound in motionEnvelope(lower, upper).bounds(errors)) motion = Math.max(motion, bound);
    for (value in errors) delta = Math.max(delta,value);
    return motion > 0 && delta > 0 ? maxToolStep * delta / motion : maxToolStep;
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
    var groups = 4 + 2 * windows.length;
    var roles = [for (g in 0...4) g];
    for (window in windows) {
      roles.push(roleOf(window.a));
      roles.push(roleOf(window.b));
    }
    var table = new CollisionMargins(groups, m);
    if (contact) for (g in 0...groups) for (h in 0...groups) {
      var toolG = roles[g] == MOVING_TOOL || roles[g] == FIXED_TOOL, toolH = roles[h] == MOVING_TOOL || roles[h] == FIXED_TOOL;
      var fixedG = roles[g] == FIXED || roles[g] == FIXED_TOOL, fixedH = roles[h] == FIXED || roles[h] == FIXED_TOOL;
      if ((toolG && fixedH) || (toolH && fixedG)) table.set(g, h, contactMargin);
    }
    for (k in 0...windows.length) {
      var window = windows[k];
      // An allowed pair is not checked; its groups keep the default for others.
      if (window.margin != null) table.set(4 + 2 * k, 5 + 2 * k, window.margin);
    }
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
    return ConvexDistance.between(a, b, TOUCH) < TOUCH;

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
