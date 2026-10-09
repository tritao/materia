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

/** Two scene objects closer than the near distance, colliding when they overlap by more than the contact tolerance. */
class SceneCollisionPair {
  /** Scene object ids, in a stable order. */
  public final a:String;
  public final b:String;
  /** Signed distance in metres: negative is penetration depth. */
  public final distance:Float;
  /** The closest points, in world metres (on `a`, then on `b`). */
  public final pointA:Array<Float>;
  public final pointB:Array<Float>;
  final overlapping:Bool;

  public function new(a:String, b:String, distance:Float, pointA:Array<Float>, pointB:Array<Float>, overlapping:Bool) {
    this.a = a;
    this.b = b;
    this.distance = distance;
    this.pointA = pointA;
    this.pointB = pointB;
    this.overlapping = overlapping;
  }

  /** Overlapping by more than the contact tolerance; resting or flush objects are only touching. */
  public function colliding():Bool return overlapping;
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
  /** Objects laid flush (a box on a table) touch rather than collide: 0.1 mm while editing. */
  public static inline var EDITING_CONTACT:Float = 1e-4;
  /**
   * Physics lets resting bodies sink in a little: 5 mm while simulating. The generated arm's
   * workpiece rests 2.04 mm into the table under MuJoCo and its suction cup grips 0.95 mm in.
   */
  public static inline var SIMULATION_CONTACT:Float = 5e-3;

  /** How many times the world was built; a pose-only change must not add one. */
  public var builds(default, null):Int = 0;

  final makeWorld:Void->CollisionWorld;
  var world:Null<CollisionWorld> = null;
  var build:Null<CollisionBuild> = null;
  var contentKey:Null<String> = null;
  /** The scene object behind each described body, and how to pose that body. */
  var bodyObjects:Array<String> = [];
  var bodyCentres:Array<Null<Array<Float>>> = [];
  var queriedRevision = "";
  var queriedNear = -1.0;
  var queried:Array<SceneCollisionPair> = [];

  /** `makeWorld` gives an empty world (a `NativeCollisionWorld`). */
  public function new(makeWorld:Void->CollisionWorld) {
    if (makeWorld == null) throw "Scene collision needs a world";
    this.makeWorld = makeWorld;
  }

  /**
   * The pairs closer than `near` metres, closest first, one per pair of scene objects; a pair
   * overlapping by more than `contact` metres collides. Bodies are where the scene has them or, while
   * a simulation runs, where `poses` (its presentation's geometry-centred poses, by scene id, at
   * `posesRevision`) puts them. `project` supplies the assembly of a generated project, when one is open.
   */
  public function query(scene:EditorScene, project:Null<ProjectDocumentSession>, near:Float,
      contact:Float = EDITING_CONTACT, ?poses:Array<ApplicationSimulation.SimulationPoseVisual>,
      posesRevision:Int = 0):Array<SceneCollisionPair> {
    if (!Math.isFinite(near) || near < 0) throw "The near distance must be finite and nonnegative";
    if (!Math.isFinite(contact) || contact < 0) throw "The contact tolerance must be finite and nonnegative";
    var revision = scene.revision + ":" + contact + (poses == null ? "" : ":sim" + posesRevision);
    if (revision == queriedRevision && near == queriedNear) return queried;
    var key = keyOf(scene, project);
    if (key != contentKey) rebuild(scene, project, key);
    queried = pairs(scene, near, contact, poses);
    queriedRevision = revision;
    queriedNear = near;
    return queried;
  }

  /**
   * The toolbar's line for `pairs`: how many collide and how many are near, and the closest pair by
   * name with its depth or gap in millimetres; null when there are none.
   */
  public static function summary(pairs:Array<SceneCollisionPair>, name:String->String):Null<String> {
    if (pairs.length == 0) return null;
    var colliding = [for (pair in pairs) if (pair.colliding()) pair].length, near = pairs.length - colliding;
    var counts:Array<String> = [];
    if (colliding > 0) counts.push(colliding + (colliding == 1 ? " collision" : " collisions"));
    if (near > 0) counts.push(near + " near");
    var closest = pairs[0];
    var millimetres = Math.round(Math.abs(closest.distance) * 10000.0) / 10.0;
    return counts.join(", ") + " · " + name(closest.a) + " / " + name(closest.b) + " " + millimetres +
      (closest.colliding() ? " mm deep" : " mm apart");
  }

  public function dispose():Void {
    if (world != null) world.dispose();
    world = null;
    build = null;
    contentKey = null;
    queriedRevision = "";
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
  function pairs(scene:EditorScene, near:Float, contact:Float,
      poses:Null<Array<ApplicationSimulation.SimulationPoseVisual>>):Array<SceneCollisionPair> {
    var current = world, built = build;
    if (current == null || built == null || bodyObjects.length == 0) return [];
    var items = new Map<String, EditorSceneObject>();
    for (item in scene.items()) items.set(item.id, item);
    var simulated = new Map<String, ApplicationSimulation.SimulationPoseVisual>();
    if (poses != null) for (pose in poses) simulated.set(pose.id, pose);
    for (index in 0...bodyObjects.length) {
      var item = items.get(bodyObjects[index]);
      if (item == null) throw 'Scene object "${bodyObjects[index]}" vanished without a rebuild';
      var live = simulated.get(item.id);
      current.setBodyPose(built.bodies[index], live == null ? posed(item, bodyCentres[index])
        : placed(live.position, live.rotation, bodyCentres[index]));
    }
    var closest = new Map<String, SceneCollisionPair>();
    for (found in current.distances(near)) {
      var a = bodyObjects[built.bodyOf(found.bodyA)], b = bodyObjects[built.bodyOf(found.bodyB)];
      var swap = a > b;
      var key = swap ? b + "\n" + a : a + "\n" + b;
      var known = closest.get(key);
      if (known != null && known.distance <= found.distance) continue;
      var overlapping = found.distance < -contact;
      closest.set(key, swap ? new SceneCollisionPair(b, a, found.distance, found.pointB, found.pointA, overlapping)
        : new SceneCollisionPair(a, b, found.distance, found.pointA, found.pointB, overlapping));
    }
    var result = [for (pair in closest) pair];
    result.sort((x, y) -> x.distance < y.distance ? -1 : x.distance > y.distance ? 1 : Reflect.compare(x.a + x.b, y.a + y.b));
    return result;
  }

  /**
   * The body's world pose: an object's centre and rotation, or, for an assembly occurrence, its
   * part frame (the centre less the preview centre, rotated).
   */
  static function posed(item:EditorSceneObject, centre:Null<Array<Float>>):CollisionPose
    return placed([item.x, item.y, item.z], item.rotation, centre);

  /** A geometry-centred pose as the body's pose (less the preview centre, for an assembly occurrence). */
  static function placed(position:Array<Float>, rotation:Null<Array<Float>>, centre:Null<Array<Float>>):CollisionPose {
    var r = rotation == null ? [0.0, 0.0, 0.0, 1.0] : rotation;
    if (centre == null) return new CollisionPose(position[0], position[1], position[2], r[0], r[1], r[2], r[3]);
    var offset = ApplicationSimulation.rotateOffset(centre[0], centre[1], centre[2], rotation);
    return new CollisionPose(position[0] - offset[0], position[1] - offset[1], position[2] - offset[2], r[0], r[1], r[2], r[3]);
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
