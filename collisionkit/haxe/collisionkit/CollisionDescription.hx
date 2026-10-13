package collisionkit;

private typedef DescriptionPairMargin = {
  var first:Int;
  var second:Int;
  var air:Float;
  var contact:Float;
}

private typedef HeldMarginBase = {
  var body:Int;
  var first:Int;
  var second:Int;
  var authored:Null<DescriptionPairMargin>;
}

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
  /** Air and process margins belong to the same description as its pair rules. */
  public var margins(get, never):CollisionMargins;
  public var contactMargins(get, never):CollisionMargins;
  public final defaultMargin:Float;
  var groups:Int = 1;
  final pairMargins:Map<String, DescriptionPairMargin> = new Map();
  final objectMargins:Map<String, DescriptionPairMargin> = new Map();
  final heldMarginBases:Map<String, HeldMarginBase> = new Map();
  var airTable:Null<CollisionMargins> = null;
  var contactTable:Null<CollisionMargins> = null;
  public final bodies:Array<DescribedBody> = [];
  public final objects:Array<DescribedObject> = [];
  final bodyRuleIndex:Map<String, Int> = new Map();
  final objectRuleIndex:Map<String, Int> = new Map();
  public final bodyRules:Array<DescribedRule> = [];
  public final objectRules:Array<DescribedRule> = [];
  public final articulations:Array<DescribedArticulation> = [];
  /** What the geometry approximates or ignores, once each, in the order noted. */
  public final assumptions:Array<String> = [];

  public function new(margin:Float = 0.0) {
    defaultMargin = margin;
    if (!Math.isFinite(margin) || margin < 0) throw "Collision description needs a finite nonnegative margin";
  }

  /** Appends another scene, remapping its body/object references and rules. */
  public function append(other:CollisionDescription):Void {
    if (other == this) throw "Cannot append a collision description to itself";
    var bodyIds = [for (body in other.bodies) addBody(body.name, body.group, body.fixed, body.pose)];
    var objectIds = [for (object in other.objects) addObject(object.name,
      object.body < 0 ? -1 : bodyIds[object.body], object.offset, object.geometry, object.inflation, object.source, object.member, object.tool)];
    for (rule in other.bodyRules) setBodyRule(rule.a < 0 ? -1 : bodyIds[rule.a],
      rule.b < 0 ? -1 : bodyIds[rule.b], rule.rule, rule.reason);
    for (rule in other.objectRules) setObjectRule(objectIds[rule.a], objectIds[rule.b], rule.rule, rule.reason);
    for (articulation in other.articulations)
      addArticulation(articulation.name, [for (body in articulation.bodies) bodyIds[body]], articulation.reference);
    for (margin in other.pairMargins) setGroupMargins(margin.first, margin.second, margin.air, margin.contact);
    for (margin in other.objectMargins) setObjectMargins(objectIds[margin.first], objectIds[margin.second], margin.air, margin.contact);
    for (base in other.heldMarginBases) {
      var a = objectIds[base.first], b = objectIds[base.second], original = base.authored;
      heldMarginBases.set(pairKey(a, b), {body: bodyIds[base.body], first: a, second: b,
        authored: original == null ? null : {first: a, second: b, air: original.air, contact: original.contact}});
    }
    for (assumption in other.assumptions) note(assumption);
  }

  public function copy():CollisionDescription {
    var result = new CollisionDescription(defaultMargin);
    result.append(this);
    return result;
  }

  /** Adds a body; `pose` is where it starts (an articulation's bodies start at its reference). */
  public function addBody(name:String, group:Int, fixed:Bool, ?pose:CollisionPose):Int {
    if (group < 0) throw 'Collision body "$name" needs a group from 0';
    if (group >= groups) { groups = group + 1; airTable = null; contactTable = null; }
    bodies.push(new DescribedBody(name, group, fixed, pose == null ? CollisionPose.identity() : pose));
    return bodies.length - 1;
  }

  /** Adds a shape on `body` (-1: the world) at `offset`; `source` says what it approximates. */
  public function addObject(name:String, body:Int, offset:CollisionPose, geometry:CollisionGeometry,
      inflation:Float = 0.0, source:String = "primitive", ?member:String, tool:Bool = false):Int {
    if (body < -1 || body >= bodies.length) throw 'Collision object "$name" is on an unknown body $body';
    if (!Math.isFinite(inflation) || inflation < 0) throw 'Collision object "$name" needs a finite, nonnegative inflation';
    objects.push(new DescribedObject(name, body, offset, geometry, inflation, source, member, tool));
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
    var key = pairKey(a, b), index = bodyRuleIndex.get(key);
    var value = new DescribedRule(a, b, rule, reason);
    if (index != null) bodyRules[index] = value;
    else { bodyRuleIndex.set(key, bodyRules.length); bodyRules.push(value); }
  }

  public function setObjectRule(a:Int, b:Int, rule:CollisionPairRule, reason:CollisionPairStatus):Void {
    if (a == b || a < 0 || b < 0 || a >= objects.length || b >= objects.length) throw 'Bad object rule $a-$b';
    var key = pairKey(a, b), index = objectRuleIndex.get(key);
    var value = new DescribedRule(a, b, rule, reason);
    if (index != null) objectRules[index] = value;
    else { objectRuleIndex.set(key, objectRules.length); objectRules.push(value); }
  }

  static function pairKey(a:Int, b:Int):String return a < b ? '$a\n$b' : '$b\n$a';

  public function bodyRule(a:Int, b:Int):Null<DescribedRule> {
    var index = bodyRuleIndex.get(pairKey(a, b));
    return index == null ? null : bodyRules[index];
  }

  public function objectRule(a:Int, b:Int):Null<DescribedRule> {
    var index = objectRuleIndex.get(pairKey(a, b));
    return index == null ? null : objectRules[index];
  }

  /** Release restores authored seat/contact policy instead of a carried part's exclusions. */
  public function restoreBodyPolicy(body:Int, authored:CollisionDescription):Void {
    if (authored == null || authored.bodies.length != bodies.length || authored.objects.length != objects.length)
      throw "Release needs the original description with stable geometry ids";
    restoreHeldMargins(body);
    for (other in -1...bodies.length) if (other != body) {
      var rule = authored.bodyRule(body, other);
      setBodyRule(body, other, rule == null ? CollisionPairRule.Default : rule.rule,
        rule == null ? CollisionPairStatus.Checked : rule.reason);
    }
    for (a in 0...objects.length) if (objects[a].body == body)
      for (b in 0...objects.length) if (objects[b].body != body) {
        var rule = authored.objectRule(a, b);
        setObjectRule(a, b, rule == null ? CollisionPairRule.Default : rule.rule,
          rule == null ? CollisionPairStatus.Checked : rule.reason);
        var key = pairKey(a, b), margin = authored.objectMargins.get(key);
        if (margin == null) objectMargins.remove(key); else objectMargins.set(key, margin);
      }
  }

  /** Carried stock checks every outside shape except its explicitly named contact surfaces. */
  public function setHeldContacts(body:Int, contacts:Array<Int>,
      contactRule:CollisionPairRule = CollisionPairRule.Allow,
      contactReason:CollisionPairStatus = CollisionPairStatus.ProcessContact, ?margin:Float):Void {
    if (body < 0 || body >= bodies.length) throw "Held stock needs a described body";
    for (contact in contacts)
      if (contact < 0 || contact >= objects.length || objects[contact].body == body)
        throw "Held contacts must name outside described objects";
    if (margin != null && (!Math.isFinite(margin) || margin < 0)) throw "Held contact margin must be finite and nonnegative";
    restoreHeldMargins(body);
    var contactIds:Map<Int, Bool> = new Map();
    for (contact in contacts) contactIds.set(contact, true);
    for (other in -1...bodies.length) if (other != body)
      setBodyRule(body, other, CollisionPairRule.Check, CollisionPairStatus.Checked);
    for (i in 0...objects.length) if (objects[i].body == body)
      for (j in 0...objects.length) if (objects[j].body != body) {
        var contact = contactIds.exists(j);
        if (contact && margin != null) {
          var key = pairKey(i, j);
          heldMarginBases.set(key, {body: body, first: i, second: j, authored: objectMargins.get(key)});
          setObjectMargins(i, j, margin, margin);
        }
        setObjectRule(i, j, contact ? contactRule : CollisionPairRule.Check,
          contact ? contactReason : CollisionPairStatus.Checked);
      }
  }

  /** Replacing grasp contacts restores only margins introduced by that grasp. */
  function restoreHeldMargins(body:Int):Void {
    var keys = [for (key in heldMarginBases.keys()) key];
    for (key in keys) {
      var base = heldMarginBases.get(key);
      if (base == null || base.body != body) continue;
      if (base.authored == null) objectMargins.remove(key); else objectMargins.set(key, base.authored);
      heldMarginBases.remove(key);
    }
  }

  /** Exact object-pair margins do not alter any other object in the same margin class. */
  public function setObjectMargins(a:Int, b:Int, ?air:Float, ?contact:Float):Void {
    if (a < 0 || b < 0 || a >= objects.length || b >= objects.length)
      throw "Collision margins need described objects";
    if ((air != null && (!Math.isFinite(air) || air < 0)) ||
        (contact != null && (!Math.isFinite(contact) || contact < 0))) throw "Object margins must be finite and nonnegative";
    if (air == null && contact == null) return;
    var key = a < b ? '$a\n$b' : '$b\n$a';
    var previous = objectMargins.get(key);
    objectMargins.set(key, {first: a, second: b,
      air: air == null ? (previous == null ? -1.0 : previous.air) : air,
      contact: contact == null ? (air == null ? (previous == null ? -1.0 : previous.contact) : air) : contact});
  }

  /** Restore only this described pair's sparse margins, without rewriting the class table. */
  public function applyObjectMargins(world:CollisionWorld, ids:Array<Int>, a:Int, b:Int):Void {
    var value = objectMargins.get(a < b ? '$a\n$b' : '$b\n$a');
    world.setObjectMargins(ids[a], ids[b], value == null || value.air < 0 ? null : value.air,
      value == null || value.contact < 0 ? null : value.contact);
  }

  public function marginBetweenObjects(a:Int, b:Int, contact:Bool = false):Float {
    if (a < 0 || b < 0 || a >= objects.length || b >= objects.length) throw "Margins need described object ids";
    var pair = objectMargins.get(a < b ? '$a\n$b' : '$b\n$a');
    var exact = pair == null ? -1.0 : contact ? pair.contact : pair.air;
    if (exact >= 0) return exact;
    var first = objects[a].body, second = objects[b].body;
    return (contact ? contactMargins : margins).between(first < 0 ? 0 : bodies[first].group,
      second < 0 ? 0 : bodies[second].group);
  }

  /** Margin classes are explicit groups, shared by as many objects as belong to that class. */
  public function setGroupMargins(g:Int, h:Int, ?air:Float, ?contact:Float):Void {
    if (g < 0 || h < 0) throw "Collision margin classes must be nonnegative";
    if (g >= groups || h >= groups) groups = Std.int(Math.max(g, h)) + 1;
    if ((air != null && (!Math.isFinite(air) || air < 0)) || (contact != null && (!Math.isFinite(contact) || contact < 0)))
      throw "Collision pair margins must be finite and nonnegative";
    var key = g < h ? '$g\n$h' : '$h\n$g';
    var previous = pairMargins.get(key);
    var normal = air == null ? (previous == null ? defaultMargin : previous.air) : air;
    var process = contact == null ? (air != null ? air : previous == null ? defaultMargin : previous.contact) : contact;
    pairMargins.set(key, {first: g, second: h, air: normal, contact: process});
    airTable = null; contactTable = null;
  }

  public function groupCount():Int return groups;

  function marginTable(contact:Bool):CollisionMargins {
    var cached = contact ? contactTable : airTable;
    if (cached != null) return cached;
    var result = new CollisionMargins(groups, defaultMargin, contact);
    for (pair in pairMargins) result.set(pair.first, pair.second, contact ? pair.contact : pair.air);
    if (contact) contactTable = result; else airTable = result;
    return result;
  }

  function get_margins():CollisionMargins return marginTable(false);
  function get_contactMargins():CollisionMargins return marginTable(true);

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
    for (pair in objectMargins) applyObjectMargins(world, objectIds, pair.first, pair.second);
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
    return new CollisionBuild(world, bodyIds, objectIds, overlaps, errors, this);
  }
}
