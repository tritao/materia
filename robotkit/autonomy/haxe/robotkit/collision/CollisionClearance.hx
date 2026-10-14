package robotkit.collision;

import collisionkit.CollisionMargins;
import collisionkit.CollisionDescription;
import collisionkit.DescribedObject;
import collisionkit.CollisionGeometry;
import robotkit.model.CollisionShape;
import robotkit.tool.ConvexSolid;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionViolation;
import collisionkit.CollisionWorld;
import robotkit.manipulation.ClearanceChange;
import robotkit.manipulation.ClearanceScene;
import kinematicskit.ClearanceMotionBounds;
import kinematicskit.ClearanceMotionEnvelope;
import robotkit.manipulation.ClearanceViolation;
import robotkit.manipulation.ClearanceWorld;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.LinkId;
import robotkit.spatial.Transform3;

/**
 * Point and motion clearance against one CollisionDescription. Standalone
 * hull fixtures use the shared builder before entering this adapter.
 * Kinematics supply body poses and conservative motion bounds; CollisionKit
 * owns geometry, pair rules, margins, and transfers of held bodies.
 * Temporary contact windows scope a margin or permission to one named pair.
 */
class CollisionClearance implements ClearanceWorld implements ClearanceScene {
  /** The distance bodies must keep, in metres. */
  public static inline var MARGIN:Float = 0.005;
  /** The distance a tool body must keep from fixed bodies where it works, in metres. */
  public static inline var CONTACT_MARGIN:Float = 0.0005;

  public final arm:KinematicGroup;
  public var world(default, null):CollisionWorld;
  final makeWorld:Void->CollisionWorld;
  final reference:Array<Float>;
  final data:collisionkit.CollisionDescription;
  final mapping:collisionkit.kinematics.ModelBodies;
  var described:Null<collisionkit.CollisionBuild> = null;
  var snapshot:Null<kinematicskit.KinematicSnapshot> = null;
  var mechanics:Null<collisionkit.kinematics.ModelBodies> = null;
  final names:Array<String> = [];
  final bodyLinks:Array<LinkId> = [];
  final corners:Array<Array<Float>> = [];
  final moving:Array<Bool> = [];
  final bodies:Array<Int> = [];
  final objects:Array<Int> = [];
  var checkedPairs = 0;
  final poses:Array<Float> = [];
  /** Open windows: their two hulls and margin (null: contact allowed). */
  final windows:Array<{a:Int, b:Int, margin:Null<Float>}> = [];
  var airMarginCache:Null<{wanted:Null<Float>, table:CollisionMargins}> = null;
  var contactMarginCache:Null<{wanted:Null<Float>, table:CollisionMargins}> = null;

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
    var result = new CollisionDescription(MARGIN);
    for (part in parts) result.append(part);
    return result;
  }

  /** Shapes on a named link, with explicit tool identity and margin classes. */
  public static function shapes(link:String, shapes:Array<CollisionShape>, isTool:Bool = false):CollisionDescription {
    var result = new CollisionDescription(MARGIN);
    result.setGroupMargins(collisionkit.CollisionMarginClass.Tool, collisionkit.CollisionMarginClass.Air, MARGIN, CONTACT_MARGIN);
    var body = result.addBody(link, isTool ? collisionkit.CollisionMarginClass.Tool : collisionkit.CollisionMarginClass.Installed, false);
    for (i in 0...shapes.length) {
      var shape = shapes[i];
      var invalid = shape.validate(); if (invalid != null) throw invalid;
      var p = shape.position, q = shape.rotation;
      result.addObject(shape.name == "" ? link + "/shape/" + i : shape.name, body, new CollisionPose(p[0],p[1],p[2],q[0],q[1],q[2],q[3]),
        shape.geometry, shape.inflation, "authored shape", null, isTool);
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

  public static function isTool(object:DescribedObject):Bool
    return object.tool;

  static function transform(p:CollisionPose):Transform3
    return new Transform3(new robotkit.spatial.Vec3(p.x,p.y,p.z), new robotkit.spatial.Quat(p.qx,p.qy,p.qz,p.qw));

  /** Finite, link-local points for motion bounds; queries retain exact geometry. */
  public static function vertices(object:DescribedObject):Array<Float> {
    switch object.geometry {
      case Convex(points) if (object.inflation == 0):
        return placed(new ConvexSolid(points).cornerPoints(), transform(object.offset));
      case _:
        var bounds = collisionkit.CollisionBounds.of(object.geometry, object.offset, object.inflation);
        return bounds.corners();
    }
  }

  /** The complete described cell and its kinematic mapping; every fixture builds a description first. */
  public function new(arm:KinematicGroup, data:collisionkit.CollisionDescription, reference:Array<Float>, makeWorld:Void->CollisionWorld,
      ?mapping:collisionkit.kinematics.ModelBodies) {
    if (arm == null || data == null || reference == null || makeWorld == null)
      throw "Collision clearance needs an arm, a description, a reference configuration and a world";
    this.arm = arm;
    this.reference = reference.copy();
    this.makeWorld = makeWorld;
    this.data = data;
    this.mapping = mapping == null ? collisionkit.kinematics.ModelBodies.bind(data, arm.model, arm.stateOf(reference)) : mapping;
    build();
  }

  /** Rebuild the original description, with no transfer or window applied. */
  function build():Void {
    world = makeWorld();
    names.resize(0);
    bodyLinks.resize(0);
    corners.resize(0);
    moving.resize(0);
    bodies.resize(0);
    objects.resize(0);
    poses.resize(0);
    windows.resize(0);
    airMarginCache = null;
    contactMarginCache = null;
    checkedPairs = 0;
    buildDescription();
  }

  function buildDescription():Void {
    var source = mapping;
    var built = data.build(world);
    described = built;
    mechanics = source.copyForDescription(built.description);
    snapshot = new kinematicskit.KinematicSnapshot(source.model);
    for (i in 0...data.bodies.length) {
      if (built.bodies[i] != i) throw "Described clearance requires an initially empty collision world";
      var carrier = descriptionMechanics().carrier(i);
      var link = carrier < 0 ? arm.rootLink : source.model.bodyIds[carrier];
      names.push(data.bodies[i].name);
      bodyLinks.push(link);
      corners.push(bodyCorners(data, i)); moving.push(carrier >= 0 && arm.moves(link));
      bodies.push(built.bodies[i]);
    }
    for (object in built.objects) objects.push(object);
    for (_ in 0...7 * bodies.length) poses.push(0.0);
    for (i in 0...objects.length) for (j in i + 1...objects.length)
      if (world.pairStatus(objects[i], objects[j]) == CollisionPairStatus.Checked) checkedPairs++;
  }

  public function reset():Void {
    world.dispose();
    build();
  }

  public function apply(change:ClearanceChange, q:Array<Float>):Void {
    switch change {
      case Attach(body, link, contacts):
        var named:Array<Int> = [];
        for (name in contacts) {
          var matching = [for (i in 0...data.objects.length)
            if (data.objects[i].name == name || data.objects[i].member == name) i];
          if (matching.length == 0) throw 'Clearance attachment has unknown contact "$name"';
          for (index in matching) if (named.indexOf(index) < 0) named.push(index);
        }
        move(hull(body), link, q, named, false);
      case Detach(body):
        move(hull(body), arm.rootLink, q, [], true);
      case OpenWindow(a, b, margin):
        var i = hull(a), j = hull(b);
        for (window in windows) if (window.a == i || window.b == i || window.a == j || window.b == j)
          throw 'Clearance window $a / $b overlaps an open window';
        if (margin != null && !(margin >= 0.0)) throw 'Clearance window $a / $b needs a nonnegative margin';
        windows.push({a: i, b: j, margin: margin});
        invalidateMarginCaches();
        descriptionBuild().openContactWindow(i, j, margin);
      case CloseWindow(a, b):
        var i = hull(a), j = hull(b);
        var found = -1;
        for (k in 0...windows.length) if ((windows[k].a == i && windows[k].b == j) || (windows[k].a == j && windows[k].b == i)) found = k;
        if (found < 0) throw 'No clearance window is open for $a / $b';
        windows.splice(found, 1);
        restoreRule(i, j);
        invalidateMarginCaches();
    }
  }

  function hull(name:String):Int {
    for (object in data.objects) if (object.name == name || object.member == name) return object.body;
    throw 'Clearance world has no body "$name"';
  }

  function move(i:Int, link:LinkId, q:Array<Float>, contacts:Array<Int>, detached:Bool):Void {
    for (window in windows) if (window.a == i || window.b == i)
      throw "Close contact windows before transferring their body";
    var source = mapping;
    var carrier = source.model.bodyIndex(link);
    if (carrier < 0) throw 'Clearance attachment has unknown link "$link"';
    // A carrier's mechanical body cannot be transferred like a workpiece.
    if (source.bodies.indexOf(i) >= 0) throw "A clearance attachment needs an independent part body";
    fill(q);
    var at = 7 * i;
    var current = Transform3.fromArrays(poses.slice(at, at + 3), poses.slice(at + 3, at + 7));
    var target = detached ? new kinematicskit.Transform(current.translation.x, current.translation.y,
      current.translation.z, current.rotation.x, current.rotation.y, current.rotation.z, current.rotation.w) :
      descriptionSnapshot().bodyPose(carrier);
    var inverse = robotkit.kinematics.RobotKinematics.toTransform3(target).inverse();
    var delta = inverse.compose(current);
    corners[i] = placed(corners[i], delta);
    descriptionMechanics().transfer(descriptionBuild(), i, new CollisionPose(current.translation.x, current.translation.y,
      current.translation.z, current.rotation.x, current.rotation.y, current.rotation.z, current.rotation.w),
      new CollisionPose(target.x, target.y, target.z, target.qx, target.qy, target.qz, target.qw),
      detached || !arm.moves(link), detached ? -1 : carrier, contacts, detached ? data : null);
    bodyLinks[i] = link; moving[i] = !detached && arm.moves(link);
    invalidateMarginCaches();
  }

  function restoreRule(i:Int, j:Int):Void {
    descriptionBuild().closeContactWindow(i, j);
  }

  /** Windows use only sparse object overrides; only the cached query tables change. */
  function invalidateMarginCaches():Void {
    airMarginCache = null;
    contactMarginCache = null;
  }

  public function group():KinematicGroup return arm;

  public function withGroup(group:KinematicGroup):ClearanceWorld
    return copyForGroup(group);

  /** A concrete owned copy, for callers responsible for the native world's lifetime. */
  public function copyForGroup(group:KinematicGroup):CollisionClearance
    return new CollisionClearance(group, data, reference, makeWorld, mapping);

  public function pairCount():Int return checkedPairs;

  public function bodyCount():Int return bodies.length;

  /** The described body index used by displacement/envelope arrays for a named object. */
  public function bodyIndex(name:String):Int return hull(name);

  public function movingNames():Array<String>
    return [for (object in data.objects) if (object.body >= 0 && moving[object.body]) object.name];

  public function distanceBetween(q:Array<Float>, first:String, second:String):Float {
    var build = descriptionBuild(), objects = build.description.objects;
    var a = [for (i in 0...objects.length) if (objects[i].name == first || objects[i].member == first) i];
    var b = [for (i in 0...objects.length) if (objects[i].name == second || objects[i].member == second) i];
    if (a.length == 0 || b.length == 0) throw 'Clearance distance needs known objects "$first" and "$second"';
    var pairs:Array<collisionkit.CollisionPair> = [];
    for (i in a) for (j in b) if (i != j)
      pairs.push(new collisionkit.CollisionPair(build.objects[i], build.objects[j],
        build.bodies[objects[i].body], build.bodies[objects[j].body]));
    if (pairs.length == 0) throw "Clearance distance needs distinct geometry";
    place(q);
    var distance = Math.POSITIVE_INFINITY;
    for (found in world.pairDistances(pairs)) distance = Math.min(distance, found.distance);
    return distance;
  }

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
    var bounds = new ClearanceMotionBounds(arm.model, [for(id in arm.group.jointIds)arm.model.dofIndex(id)], arm.stateOf(q, mapping.reference.q), errors, arm.model.frameIndex(arm.flangeFrame), robotkit.kinematics.RobotKinematics.toTransform(arm.flangeTTcp), arm.workFrame == null ? -1 : arm.model.frameIndex(arm.workFrame));
    return [for (i in 0...bodies.length) bounds.point(arm.model.bodyIndex(bodyLinks[i]), originRadius(corners[i]))];
  }

  public function motionEnvelope(lower:Array<Float>, upper:Array<Float>):ClearanceMotionEnvelope
    return new ClearanceMotionEnvelope(arm.model, [for(id in arm.group.jointIds)arm.model.dofIndex(id)],
      arm.stateOf([for(i in 0...lower.length)lower[i]+(upper[i]-lower[i])*0.5], mapping.reference.q),
      lower, upper, [for(link in bodyLinks)arm.model.bodyIndex(link)], corners,
      arm.model.frameIndex(arm.flangeFrame), robotkit.kinematics.RobotKinematics.toTransform(arm.flangeTTcp),
      arm.workFrame == null ? -1 : arm.model.frameIndex(arm.workFrame));

  public function tcpDisplacementBound(q:Array<Float>, errors:Array<Float>):Float
    return new ClearanceMotionBounds(arm.model, [for(id in arm.group.jointIds)arm.model.dofIndex(id)], arm.stateOf(q, mapping.reference.q), errors, arm.model.frameIndex(arm.flangeFrame), robotkit.kinematics.RobotKinematics.toTransform(arm.flangeTTcp), arm.workFrame == null ? -1 : arm.model.frameIndex(arm.workFrame)).tcp();

  public function auditDisplacement(q:Array<Float>, perturbed:Array<Float>, errors:Array<Float>, ?envelope:ClearanceMotionEnvelope):Float {
    var bounds = envelope == null ? displacementBounds(q, errors) : envelope.bounds(errors);
    fill(q); var first = poses.copy(); fill(perturbed); var ratio = 0.0;
    for (i in 0...bodies.length) {
      var at = 7 * i;
      var a = placed(corners[i], Transform3.fromArrays(first.slice(at, at + 3), first.slice(at + 3, at + 7)));
      var b = placed(corners[i], Transform3.fromArrays(poses.slice(at, at + 3), poses.slice(at + 3, at + 7)));
      for (j in 0...Std.int(a.length / 3)) {
        var dx = a[3 * j] - b[3 * j], dy = a[3 * j + 1] - b[3 * j + 1], dz = a[3 * j + 2] - b[3 * j + 2];
        var moved = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (moved > bounds[i] + 1e-10) throw 'Hull displacement exceeds described bound for ${names[i]}';
        if (bounds[i] > 0) ratio = Math.max(ratio, moved / bounds[i]);
      }
    }
    var moved = arm.tcpPose(q).translation.sub(arm.tcpPose(perturbed).translation).norm();
    var tcp = envelope == null ? tcpDisplacementBound(q, errors) : envelope.tcp(errors);
    if (moved > tcp + 1e-10) throw "TCP displacement exceeds described contact neighborhood bound";
    return ratio;
  }

  function margins(contact:Bool, wanted:Null<Float>):CollisionMargins {
    if (wanted != null && (!Math.isFinite(wanted) || wanted < 0))
      throw "Clearance margin must be finite and nonnegative";
    var original = contact ? data.contactMargins : data.margins;
    if (windows.length == 0 && wanted == null) return original;
    var cached = contact ? contactMarginCache : airMarginCache;
    if (cached != null && cached.wanted == wanted) return cached.table;
    var table = new CollisionMargins(original.groups, 0.0, contact, wanted);
    for (g in 0...original.groups) for (h in g...original.groups) {
      var value = original.between(g, h);
      table.set(g, h, wanted == null ? value : Math.min(value, wanted));
    }
    if (contact) contactMarginCache = {wanted: wanted, table: table};
    else airMarginCache = {wanted: wanted, table: table};
    return table;
  }

  /** Every body's pose (its link's, in the arm's reference frame) for `q`, into `poses`. */
  function fill(q:Array<Float>):Void {
    var source = mapping, snapshot = descriptionSnapshot();
    var observed = arm.stateOf(q, source.reference.q).q;
    snapshot.evaluate(source.state(observed));
    for (i in 0...bodies.length) {
      var carrier = descriptionMechanics().carrier(i);
      if (carrier >= 0) snapshot.bodyPoseInto(carrier, poses, 7 * i);
      else {
        var at = descriptionBuild().description.bodies[i].pose, o = 7 * i;
        poses[o] = at.x; poses[o + 1] = at.y; poses[o + 2] = at.z;
        poses[o + 3] = at.qx; poses[o + 4] = at.qy; poses[o + 5] = at.qz; poses[o + 6] = at.qw;
      }
    }
  }

  function place(q:Array<Float>):Void {
    fill(q);
    world.setBodyPoses(0, poses);
  }

  function named(found:Null<CollisionViolation>):Null<ClearanceViolation> {
    if (found == null) return null;
    var built = descriptionBuild();
    var a = built.objectOf(found.a), b = built.objectOf(found.b);
    if (a < 0 || b < 0) throw "Clearance violation lost its described object";
    return {a: data.objects[a].name, b: data.objects[b].name,
      distance: Math.max(0.0, found.distance), required: found.required};
  }


  function descriptionMechanics():collisionkit.kinematics.ModelBodies {
    var value = mechanics;
    if (value == null) throw "Collision clearance has no live mechanical mapping";
    return value;
  }

  function descriptionBuild():collisionkit.CollisionBuild {
    var built = described;
    if (built == null) throw "Clearance cell description has not been built";
    return built;
  }

  function descriptionSnapshot():kinematicskit.KinematicSnapshot {
    var built = snapshot;
    if (built == null) throw "Clearance cell kinematics have not been initialized";
    return built;
  }

  public static function placed(points:Array<Float>, pose:Transform3):Array<Float> {
    var out:Array<Float> = [];
    for (i in 0...Std.int(points.length / 3)) {
      var p = pose.transformPoint(new robotkit.spatial.Vec3(points[3 * i], points[3 * i + 1], points[3 * i + 2]));
      out.push(p.x);
      out.push(p.y);
      out.push(p.z);
    }
    return out;
  }

  static function bodyCorners(description:CollisionDescription, body:Int):Array<Float> {
    var result:Array<Float> = [];
    for (object in description.objects) if (object.body == body)
      for (value in vertices(object)) result.push(value);
    return result;
  }

  static function originRadius(points:Array<Float>):Float {
    var radius = 0.0;
    for (i in 0...Std.int(points.length / 3))
      radius = Math.max(radius, Math.sqrt(points[3 * i] * points[3 * i] + points[3 * i + 1] * points[3 * i + 1] + points[3 * i + 2] * points[3 * i + 2]));
    return radius;
  }
}
