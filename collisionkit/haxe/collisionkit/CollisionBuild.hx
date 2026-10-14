package collisionkit;

/**
 * A `CollisionDescription` built into a world: the world's id of every
 * described body and object, how many pairs overlapped at their
 * articulation's reference, and the layout errors: pairs colliding at the
 * reference between an articulation and anything outside it (CL-D3).
 */
class CollisionBuild {
  public final world:CollisionWorld;
  /** Live geometry and policy after transfers; the original build input remains reusable. */
  public final description:CollisionDescription;
  public final bodies:Array<Int>;
  public final objects:Array<Int>;
  public final overlapsAtReference:Int;
  public final layoutErrors:Array<CollisionPair>;

  public function new(world:CollisionWorld, bodies:Array<Int>, objects:Array<Int>, overlapsAtReference:Int,
      layoutErrors:Array<CollisionPair>, description:CollisionDescription) {
    this.description = description.copy();
    this.world = world;
    this.bodies = bodies;
    this.objects = objects;
    this.overlapsAtReference = overlapsAtReference;
    this.layoutErrors = layoutErrors;
  }

  /**
   * Change a carried body's frame without moving its shapes in the world.
   * Geometry, inflation and object ids remain unchanged. This is the same
   * operation for a grasp, a placement, and a transfer between carriers.
   */
  public function transferBody(body:Int, current:CollisionPose, target:CollisionPose, fixed:Bool,
      ?contacts:Array<Int>, ?releasedPolicy:CollisionDescription):Void {
    if (body < 0 || body >= bodies.length) throw "Attachment needs a described body";
    var namedContacts = contacts == null ? [] : contacts;
    // Validate and author policy before mutating the physical world.
    if (releasedPolicy == null) description.setHeldContacts(body, namedContacts);
    else description.restoreBodyPolicy(body, releasedPolicy);
    var from = kinematicskit.Transform.checked(new kinematicskit.Transform(current.x, current.y, current.z, current.qx, current.qy, current.qz, current.qw), "Attachment source pose");
    var to = kinematicskit.Transform.checked(new kinematicskit.Transform(target.x, target.y, target.z, target.qx, target.qy, target.qz, target.qw), "Attachment target pose");
    var delta = to.inverse().compose(from);
    for (i in 0...description.objects.length) {
      var shape = description.objects[i];
      if (shape.body != body) continue;
      var old = shape.offset;
      var at = delta.compose(new kinematicskit.Transform(old.x, old.y, old.z, old.qx, old.qy, old.qz, old.qw));
      var offset = new CollisionPose(at.x, at.y, at.z, at.qx, at.qy, at.qz, at.qw);
      world.attach(objects[i], bodies[body], offset);
      description.objects[i] = new DescribedObject(shape.name, body, offset, shape.geometry, shape.inflation, shape.source, shape.member, shape.tool);
      for (other in 0...description.objects.length) if (description.objects[other].body != body)
        description.applyObjectMargins(world, objects, i, other);
    }
    var previous = description.bodies[body];
    description.bodies[body] = new DescribedBody(previous.name, previous.group, fixed, target);
    world.setBodyPose(bodies[body], target);
    world.setBodyStatic(bodies[body], fixed);
    for (other in 0...bodies.length) if (other != body) restoreBodyRule(body, other);
    for (rule in description.objectRules)
      if (description.objects[rule.a].body == body || description.objects[rule.b].body == body)
        world.setObjectRule(objects[rule.a], objects[rule.b], rule.rule, rule.reason);
  }

  /** Restore an authored rule after a temporary process window. */
  public function restoreBodyRule(first:Int, second:Int):Void {
    var rule = description.bodyRule(first, second);
    world.setBodyRule(bodies[first], bodies[second], rule == null ? CollisionPairRule.Default : rule.rule,
      rule == null ? CollisionPairStatus.Checked : rule.reason);
  }

  /** A temporary contact window overrides object rules as well as body rules. */
  public function openContactWindow(first:Int, second:Int, ?margin:Float):Void {
    if (margin == null) world.setBodyRule(bodies[first], bodies[second], CollisionPairRule.Allow, CollisionPairStatus.ProcessContact);
    openObjectWindow(objectsOfBody(first), objectsOfBody(second), margin);
  }

  /** Restore the live description's policy after a contact window closes. */
  public function closeContactWindow(first:Int, second:Int):Void {
    restoreBodyRule(first, second);
    closeObjectWindow(objectsOfBody(first), objectsOfBody(second));
  }

  /** A window between two sets of described objects: contact allowed (no margin) or the margin between them. */
  public function openObjectWindow(first:Array<Int>, second:Array<Int>, ?margin:Float):Void {
    for (a in first) for (b in second) if (a != b) {
      if (margin == null) world.setObjectRule(objects[a], objects[b], CollisionPairRule.Allow, CollisionPairStatus.ProcessContact);
      else world.setObjectMargins(objects[a], objects[b], margin, margin);
    }
  }

  /** Restore the described rules and margins between two sets of objects. */
  public function closeObjectWindow(first:Array<Int>, second:Array<Int>):Void {
    for (a in first) for (b in second) if (a != b) {
      var rule = description.objectRule(a, b);
      world.setObjectRule(objects[a], objects[b], rule == null ? CollisionPairRule.Default : rule.rule,
        rule == null ? CollisionPairStatus.Checked : rule.reason);
      description.applyObjectMargins(world, objects, a, b);
    }
  }

  function objectsOfBody(body:Int):Array<Int>
    return [for (i in 0...description.objects.length) if (description.objects[i].body == body) i];

  /** The described object a world object came from, or -1. */
  public function objectOf(worldObject:Int):Int return objects.indexOf(worldObject);

  /** The described body a world body came from, or -1 (the world). */
  public function bodyOf(worldBody:Int):Int return worldBody < 0 ? -1 : bodies.indexOf(worldBody);
}
