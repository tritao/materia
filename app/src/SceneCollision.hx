package app;

import cadbridge.AssemblySimulationBridge;
import cadkit.modeling.AssemblyState;
import collisionkit.CollisionBuild;
import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionWorld;
import materia.assembly.AssemblyBodies;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;

/** Two scene objects closer than the near distance: overlapping when `distance` is zero or less. */
class SceneCollisionPair {
  /** Scene object ids, in a stable order. */
  public final a:String;
  public final b:String;
  /** Signed distance in metres: negative is penetration depth. */
  public final distance:Float;
  /** The closest points, in world metres (on `a`, then on `b`). */
  public final pointA:Array<Float>;
  public final pointB:Array<Float>;

  public function new(a:String, b:String, distance:Float, pointA:Array<Float>, pointB:Array<Float>) {
    this.a = a;
    this.b = b;
    this.distance = distance;
    this.pointA = pointA;
    this.pointB = pointB;
  }

  public function colliding():Bool return distance <= 0;
}

/**
 * The editor's collision world (COLLISION.md CL-D14, CL8a): the scene's collision-enabled objects,
 * and a generated project's assembly occurrences with their collision pieces, in one collisionkit
 * world. The world is rebuilt only when what collides changes (objects, shapes, the collision flag,
 * the assembly); a moved object or a jogged joint only re-poses its body. `query` answers the pairs
 * closer than the near distance at the scene's current poses, once per scene revision.
 *
 * Within an assembly, the rules are CL-D3's: occurrences of one rigid body never collide, nor do
 * the bodies on either side of a moving joint, and the assembly is an articulation whose reference
 * is its as-designed state (default joint values), so parts that touch by design are allowed.
 * Every other object is a movable box (CAD parts by their collision bounds), so laying two objects
 * into each other shows.
 */
class SceneCollision {
  /** How many times the world was built; a pose-only change must not add one. */
  public var builds(default, null):Int = 0;

  final makeWorld:Void->CollisionWorld;
  var world:Null<CollisionWorld> = null;
  var build:Null<CollisionBuild> = null;
  var contentKey:Null<String> = null;
  /** The scene object behind each described body, and how to pose that body. */
  var bodyObjects:Array<String> = [];
  var bodyCentres:Array<Null<Array<Float>>> = [];
  var queriedRevision = -1;
  var queriedNear = -1.0;
  var queried:Array<SceneCollisionPair> = [];

  /** `makeWorld` gives an empty world (a `NativeCollisionWorld`). */
  public function new(makeWorld:Void->CollisionWorld) {
    if (makeWorld == null) throw "Scene collision needs a world";
    this.makeWorld = makeWorld;
  }

  /**
   * The pairs closer than `near` metres at the scene's current poses, closest first, one per pair of
   * scene objects. `project` supplies the assembly of a generated project, when one is open.
   */
  public function query(scene:EditorScene, project:Null<ProjectDocumentSession>, near:Float):Array<SceneCollisionPair> {
    if (!Math.isFinite(near) || near < 0) throw "The near distance must be finite and nonnegative";
    if (scene.revision == queriedRevision && near == queriedNear) return queried;
    var key = keyOf(scene, project);
    if (key != contentKey) rebuild(scene, project, key);
    queried = pairs(scene, near);
    queriedRevision = scene.revision;
    queriedNear = near;
    return queried;
  }

  public function dispose():Void {
    if (world != null) world.dispose();
    world = null;
    build = null;
    contentKey = null;
    queriedRevision = -1;
  }

  /** What the world is built from; poses are left out on purpose. */
  static function keyOf(scene:EditorScene, project:Null<ProjectDocumentSession>):String {
    var parts:Array<String> = [];
    if (project != null) {
      var open:ProjectDocumentSession = project;
      var assembly = open.projectAssemblyDefinition;
      if (assembly != null && open.projectPhysical != null)
        parts.push('assembly:${open.generation}:${assembly.occurrences.length}');
    }
    for (item in scene.items()) if (item.collisionEnabled) {
      var bounds = scene.isCadPart(item.id) ? scene.cadSession(item.id).collisionBounds : null;
      parts.push(item.id + "|" + item.kind + "|" + item.width + "|" + item.height + "|" + item.depth +
        (bounds == null ? "" : "|" + bounds.center.x + "," + bounds.center.y + "," + bounds.center.z + "," +
          bounds.halfExtents.x + "," + bounds.halfExtents.y + "," + bounds.halfExtents.z));
    }
    return parts.join(";");
  }

  function rebuild(scene:EditorScene, project:Null<ProjectDocumentSession>, key:String):Void {
    if (world != null) world.dispose();
    var description = new CollisionDescription();
    bodyObjects = [];
    bodyCentres = [];
    var assemblyObjects = describeAssembly(description, scene, project);
    for (item in scene.items()) if (item.collisionEnabled && !assemblyObjects.exists(item.id)) {
      var box = boxOf(scene, item);
      var body = description.addBody(item.id, 0, false, posed(item, null));
      description.addObject(item.id, body, CollisionPose.translation(box.offset[0], box.offset[1], box.offset[2]),
        CollisionGeometry.Box(box.half[0], box.half[1], box.half[2]), 0.0, "box");
      bodyObjects.push(item.id);
      bodyCentres.push(null);
    }
    var created = makeWorld();
    build = description.build(created);
    world = created;
    contentKey = key;
    builds++;
  }

  /**
   * The open project's assembly occurrences, posed at the as-designed state for the articulation's
   * reference; returns the scene objects it described.
   */
  function describeAssembly(description:CollisionDescription, scene:EditorScene,
      project:Null<ProjectDocumentSession>):Map<String, Bool> {
    var described = new Map<String, Bool>();
    if (project == null) return described;
    var open:ProjectDocumentSession = project;
    var assembly = open.projectAssemblyDefinition, physical = open.projectPhysical;
    if (assembly == null || physical == null) return described;
    var flat = AssemblyDefinitionFlattener.flatten(assembly);
    var scale = physical.metresPerUnit;
    var designed = new AssemblyState(assembly);
    var components = new Map<String, AssemblyComponentDefinition>();
    for (component in flat.definitions) components.set(component.id, component);
    var parts = new Map<String, cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart>();
    for (part in physical.parts) parts.set(part.id, part);
    var bodyOf = new Map<String, Int>();
    for (occurrence in flat.occurrences) {
      var id = "project:" + occurrence.id;
      var item = scene.object(id);
      if (item == null || !item.collisionEnabled) continue;
      var pieces = AssemblySimulationBridge.collisionPieces(occurrence.id, components.get(occurrence.definition),
        parts.get(occurrence.definition));
      if (pieces.length == 0) continue;
      var frame = designed.worldPose(occurrence.id);
      var body = description.addBody(id, 0, false, new CollisionPose(frame.x * scale, frame.y * scale, frame.z * scale,
        frame.qx, frame.qy, frame.qz, frame.qw));
      for (index in 0...pieces.length)
        description.addObject('$id#$index', body, CollisionPose.identity(),
          CollisionGeometry.Convex([for (value in pieces[index]) value * scale]), 0.0, "assembly hull");
      var centre = open.assemblyPreviewCenter(occurrence.definition);
      if (centre == null) throw 'Assembly component "${occurrence.definition}" has no preview centre';
      bodyObjects.push(id);
      bodyCentres.push([for (value in centre) value * scale]);
      bodyOf.set(occurrence.id, body);
      described.set(id, true);
    }
    // CL-D3: one rigid body never collides with itself, nor with the bodies one moving joint away.
    var rigid = AssemblyBodies.of(flat);
    var members = new Map<String, Array<Int>>();
    for (group in rigid) members.set(group.id, [for (id in group.occurrences) if (bodyOf.exists(id)) bodyOf.get(id)]);
    function membersOf(group:String):Array<Int> {
      var found = members.get(group);
      return found == null ? [] : found;
    }
    for (group in rigid) {
      var mine = membersOf(group.id);
      for (i in 0...mine.length) for (j in i + 1...mine.length)
        description.setBodyRule(mine[i], mine[j], CollisionPairRule.Allow, CollisionPairStatus.Rigid);
      // A body's parent is named by its root occurrence, which is the parent group's id.
      var parent = group.parent;
      if (parent != null) for (x in mine) for (y in membersOf(parent))
        description.setBodyRule(x, y, CollisionPairRule.Allow, CollisionPairStatus.Adjacent);
    }
    var all = [for (id in bodyOf) id];
    if (all.length > 1) description.addArticulation("assembly", all, "as designed");
    return described;
  }

  /** The pairs within `near`, with every body posed where the scene has its object now. */
  function pairs(scene:EditorScene, near:Float):Array<SceneCollisionPair> {
    var current = world, built = build;
    if (current == null || built == null || bodyObjects.length == 0) return [];
    var items = new Map<String, EditorSceneObject>();
    for (item in scene.items()) items.set(item.id, item);
    for (index in 0...bodyObjects.length) {
      var item = items.get(bodyObjects[index]);
      if (item == null) throw 'Scene object "${bodyObjects[index]}" vanished without a rebuild';
      current.setBodyPose(built.bodies[index], posed(item, bodyCentres[index]));
    }
    var closest = new Map<String, SceneCollisionPair>();
    for (found in current.distances(near)) {
      var a = bodyObjects[built.bodyOf(found.bodyA)], b = bodyObjects[built.bodyOf(found.bodyB)];
      var swap = a > b;
      var key = swap ? b + "\n" + a : a + "\n" + b;
      var known = closest.get(key);
      if (known != null && known.distance <= found.distance) continue;
      closest.set(key, swap ? new SceneCollisionPair(b, a, found.distance, found.pointB, found.pointA)
        : new SceneCollisionPair(a, b, found.distance, found.pointA, found.pointB));
    }
    var result = [for (pair in closest) pair];
    result.sort((x, y) -> x.distance < y.distance ? -1 : x.distance > y.distance ? 1 : Reflect.compare(x.a + x.b, y.a + y.b));
    return result;
  }

  /**
   * The body's world pose: an object's centre and rotation, or, for an assembly occurrence, its
   * part frame (the centre less the preview centre, rotated).
   */
  static function posed(item:EditorSceneObject, centre:Null<Array<Float>>):CollisionPose {
    var r = item.rotation == null ? [0.0, 0.0, 0.0, 1.0] : item.rotation;
    if (centre == null) return new CollisionPose(item.x, item.y, item.z, r[0], r[1], r[2], r[3]);
    var offset = ApplicationSimulation.rotateOffset(centre[0], centre[1], centre[2], item.rotation);
    return new CollisionPose(item.x - offset[0], item.y - offset[1], item.z - offset[2], r[0], r[1], r[2], r[3]);
  }

  /** The object's box, as the simulation builds it: its extents, or a CAD part's collision bounds. */
  static function boxOf(scene:EditorScene, item:EditorSceneObject):{offset:Array<Float>, half:Array<Float>} {
    if (!scene.isCadPart(item.id))
      return {offset: [0.0, 0.0, 0.0], half: [item.width / 2.0, item.height / 2.0, item.depth / 2.0]};
    var bounds = scene.cadSession(item.id).collisionBounds;
    if (bounds == null) throw "CAD collision bounds are unavailable for: " + item.id;
    return {offset: [bounds.center.x, bounds.center.y, bounds.center.z],
      half: [bounds.halfExtents.x, bounds.halfExtents.y, bounds.halfExtents.z]};
  }
}
