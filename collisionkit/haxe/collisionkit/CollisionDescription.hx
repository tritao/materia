package collisionkit;

/**
 * A cell's collision description (COLLISION.md CL3): bodies with their
 * groups and poses, shapes on them, rules with their reasons, the
 * articulations and their reference configurations, and the assumptions the
 * geometry makes (CL-D8). Geometry sources (RobotKit, CadKit through
 * cadbridge, terrain) add to it; `build` turns it into a `CollisionWorld`.
 * It is plain data, so the same description can feed other consumers (the
 * simulation, CL3b).
 */
class CollisionDescription {
  public final bodies:Array<DescribedBody> = [];
  public final objects:Array<DescribedObject> = [];
  public final bodyRules:Array<DescribedRule> = [];
  public final objectRules:Array<DescribedRule> = [];
  public final articulations:Array<DescribedArticulation> = [];
  /** What the geometry approximates or ignores, once each, in the order noted. */
  public final assumptions:Array<String> = [];

  public function new() {}

  /** Adds a body; `pose` is where it starts (an articulation's bodies start at its reference). */
  public function addBody(name:String, group:Int, fixed:Bool, ?pose:CollisionPose):Int {
    if (group < 0) throw 'Collision body "$name" needs a group from 0';
    bodies.push(new DescribedBody(name, group, fixed, pose == null ? CollisionPose.identity() : pose));
    return bodies.length - 1;
  }

  /** Adds a shape on `body` (-1: the world) at `offset`; `source` says what it approximates. */
  public function addObject(name:String, body:Int, offset:CollisionPose, geometry:CollisionGeometry,
      inflation:Float = 0.0, source:String = "primitive"):Int {
    if (body < -1 || body >= bodies.length) throw 'Collision object "$name" is on an unknown body $body';
    if (!Math.isFinite(inflation) || inflation < 0) throw 'Collision object "$name" needs a finite, nonnegative inflation';
    objects.push(new DescribedObject(name, body, offset, geometry, inflation, source));
    return objects.length - 1;
  }

  /** Adds a fixed body at `pose` with one shape on it; returns the object. */
  public function addFixed(name:String, pose:CollisionPose, geometry:CollisionGeometry, group:Int,
      inflation:Float = 0.0, source:String = "primitive"):Int {
    var body = addBody(name, group, true, pose);
    return addObject(name, body, CollisionPose.identity(), geometry, inflation, source);
  }

  public function setBodyRule(a:Int, b:Int, rule:CollisionPairRule, reason:CollisionPairStatus):Void {
    if (a == b || a < -1 || b < -1 || a >= bodies.length || b >= bodies.length) throw 'Bad body rule $a-$b';
    bodyRules.push(new DescribedRule(a, b, rule, reason));
  }

  public function setObjectRule(a:Int, b:Int, rule:CollisionPairRule, reason:CollisionPairStatus):Void {
    if (a == b || a < 0 || b < 0 || a >= objects.length || b >= objects.length) throw 'Bad object rule $a-$b';
    objectRules.push(new DescribedRule(a, b, rule, reason));
  }

  /** Declares an articulation: its bodies, posed at the named reference configuration. */
  public function addArticulation(name:String, bodies:Array<Int>, reference:String):Int {
    for (body in bodies) if (body < 0 || body >= this.bodies.length) throw 'Articulation "$name" has an unknown body $body';
    articulations.push(new DescribedArticulation(name, bodies.copy(), reference));
    note('Articulation "$name" checks its own overlaps at its reference configuration ($reference)');
    return articulations.length - 1;
  }

  /**
   * The radius of a body's objects about its frame: no point of them is
   * farther from the body's origin (inflation included). Motion bounds use
   * it (CL-D5). Height fields and half-spaces have no finite radius.
   */
  public function bodyRadius(body:Int):Float {
    var radius = 0.0;
    for (object in objects) if (object.body == body) {
      var o = object.offset;
      var centre = Math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
      // Rotations keep distances from the offset's origin, so local extents add to the offset's length.
      var extent = switch object.geometry {
        case Box(x, y, z): Math.sqrt(x * x + y * y + z * z);
        case Sphere(r): r;
        case Capsule(r, h): r + h;
        case Cylinder(r, h): Math.sqrt(r * r + h * h);
        case Convex(points) | Mesh(points, _):
          var far = 0.0;
          for (i in 0...Std.int(points.length / 3))
            far = Math.max(far, Math.sqrt(points[3 * i] * points[3 * i] + points[3 * i + 1] * points[3 * i + 1] + points[3 * i + 2] * points[3 * i + 2]));
          far;
        case HalfSpace(_, _, _, _) | HeightField(_, _, _, _, _): Math.POSITIVE_INFINITY;
      }
      radius = Math.max(radius, centre + extent + object.inflation);
    }
    return radius;
  }

  /** Records an assumption once. */
  public function note(text:String):Void {
    if (assumptions.indexOf(text) < 0) assumptions.push(text);
  }

  /**
   * Adds everything to `world`, poses each body where the description has
   * it, marks each articulation's overlaps at its reference, and reports as
   * layout errors the pairs colliding there between an articulation's body
   * and anything outside it. The world is left posed at the references.
   */
  public function build(world:CollisionWorld):CollisionBuild {
    var bodyIds = [for (body in bodies) world.addBody(body.group)];
    for (i in 0...bodies.length) {
      if (bodies[i].fixed) world.setBodyStatic(bodyIds[i], true);
      world.setBodyPose(bodyIds[i], bodies[i].pose);
    }
    function bodyId(body:Int):Int return body < 0 ? -1 : bodyIds[body];
    var objectIds:Array<Int> = [];
    for (object in objects) {
      var id = world.add(bodyId(object.body), object.offset, object.geometry);
      if (object.inflation > 0) world.setInflation(id, object.inflation);
      objectIds.push(id);
    }
    for (rule in bodyRules) world.setBodyRule(bodyId(rule.a), bodyId(rule.b), rule.rule, rule.reason);
    for (rule in objectRules) world.setObjectRule(objectIds[rule.a], objectIds[rule.b], rule.rule, rule.reason);
    var overlaps = 0;
    var articulated = new Map<Int, Int>();
    for (index in 0...articulations.length) {
      var articulation = articulations[index];
      for (body in articulation.bodies) articulated.set(bodyIds[body], index);
      overlaps += world.allowOverlapping([for (body in articulation.bodies) bodyIds[body]]);
    }
    var errors:Array<CollisionPair> = [];
    for (pair in world.colliding(0.0)) {
      var a = articulated.get(pair.bodyA), b = articulated.get(pair.bodyB);
      if (a == null && b == null) continue;
      errors.push(pair);
    }
    return new CollisionBuild(world, bodyIds, objectIds, overlaps, errors);
  }
}
