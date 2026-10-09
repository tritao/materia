import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblySimulationBridge.AssemblySimulationModel;
import cadbridge.CadCollision;
import cadbridge.CadCollision.CadCollisionMesh;
import cadbridge.CadCollision.CadDecompositionCache;
import cadbridge.CadCollision.CadPartCollision;
import cadkit.Shape;
import cadkit.modeling.AssemblyModel;
import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPair;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.native.NativeCollisionWorld;
import collisionkit.native.NativeConvexDecomposer;
import haxe.io.Bytes;
import materia.assembly.AssemblyFrames;
import processkit.collision.HeightMapCollision;
import processkit.work.HeightMap;
import robotkit.collision.RobotCollision;
import robotkit.collision.RobotCollision.RobotCollisionOptions;
import robotkit.collision.RobotCollision.RobotLinkHull;
import robotkit.model.CollisionShape;
import robotkit.model.CollisionShape.ShapeContact;
import robotkit.model.ContactPair;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.Tool;
import robotkit.tool.ToolCollisionShape;

/**
 * A cell's collision geometry (COLLISION.md CL3a): a robot with primitives,
 * a hull, finger pads with a contact pair and a tool; a CAD mechanism posed
 * from its own model; terrain from a height map; a fixture through the
 * robot reported as a layout error; the CL-D10 mapping; and a decomposed
 * non-convex CAD part whose pieces enclose its tessellation.
 */
class CellCollisionTests {
  static var assertions = 0;

  public static function main():Void {
    testRobotMechanismAndTerrain();
    testFixtureThroughTheRobotIsALayoutError();
    testSimulationContactSettings();
    testDecomposedPartEnclosesItsTessellation();
    testArmClearanceBodies();
    Sys.println('Cell collision tests passed ($assertions assertions)');
  }

  /** Base -> shoulder (z) -> upper -> elbow (z, x = 1) -> fore; a flange frame at the forearm's tip (x = 1). */
  static function arm():RobotModel {
    var model = new RobotModel("arm");
    var base = model.addLink(new Link("base"));
    var upper = model.addLink(new Link("upper"));
    var fore = model.addLink(new Link("fore"));
    var shoulder = model.addJoint(new Joint("shoulder", JointType.Revolute, base, upper));
    shoulder.axis = [0.0, 0, 1]; shoulder.limits.lower = -3; shoulder.limits.upper = 3;
    var elbow = model.addJoint(new Joint("elbow", JointType.Revolute, upper, fore));
    elbow.axis = [0.0, 0, 1]; elbow.limits.lower = -3; elbow.limits.upper = 3;
    elbow.parentFramePosition = [1.0, 0, 0];
    var flange = model.addFrame(new Frame("flange", fore));
    flange.position = [1.0, 0, 0];
    base.collisionShapes.push(new CollisionShape(Box(0.1, 0.1, 0.1)));
    upper.collisionShapes.push(new CollisionShape(Box(0.4, 0.05, 0.05), [0.5, 0, 0]));
    fore.collisionShapes.push(new CollisionShape(Box(0.4, 0.05, 0.05), [0.5, 0, 0]));
    // Two finger pads meant to touch each other: simulation-only contact settings and a contact pair.
    var left = new CollisionShape(Box(0.02, 0.02, 0.02), [0.95, 0.015, 0.1]);
    left.contact = ShapeContact.PairsOnly;
    var right = new CollisionShape(Box(0.02, 0.02, 0.02), [0.95, -0.015, 0.1]);
    right.contact = ShapeContact.PairsOnly;
    fore.collisionShapes.push(left);
    fore.collisionShapes.push(right);
    model.contactPairs.push(new ContactPair(fore.id, 1, fore.id, 2));
    return model;
  }

  static function armOptions(name:String):RobotCollisionOptions {
    var options = new RobotCollisionOptions(name);
    options.tool = new Tool("probe", "probe", new Transform3(new Vec3(0.1, 0, 0), new Quat(0, 0, 0, 1)),
      ToolCollisionShape.Box(new Vec3(0.05, 0.05, 0.05), new Vec3(0.05, 0, 0)));
    options.toolFrame = "flange";
    options.hulls = [new RobotLinkHull(1, "upper-hull", cube(0.2, 0, 0, 0.04))];
    return options;
  }

  /** A slider mechanism from CAD, its two parts carrying hulls (mm). */
  static function slider():AssemblySimulationModel {
    var assembly = new AssemblyModel();
    assembly.add("base");
    assembly.add("slider");
    assembly.connector("base", "mount", AssemblyFrames.identity());
    assembly.connector("slider", "mount", AssemblyFrames.identity());
    assembly.mateOnAxis("slide", "prismatic", "base", "mount", "slider", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: 0, upper: 100, velocity: 20, effort: 50});
    var points = cube(0, 0, 0, 50);
    var vertices = Bytes.alloc(points.length * 8);
    for (index in 0...points.length) vertices.setDouble(index * 8, points[index]);
    var parts = AssemblyPhysicalPartView.fromSceneArtifact({metresPerUnit: 0.001,
      parts: [for (id in ["base", "slider"]) {
        id: id, name: id, red: 0.5, green: 0.5, blue: 0.5,
        materialId: "machined-steel", materialDensity: 7850.0,
        volume: 1000000.0, centerOfMass: [0.0, 0.0, 0.0],
        inertia: [10000000000.0, 0, 0, 0, 10000000000.0, 0, 0, 0, 10000000000.0],
        vertexCount: 8, indexCount: 0, vertices: vertices,
        normals: Bytes.alloc(0), indices: Bytes.alloc(0), faceRanges: []
      }]});
    return AssemblySimulationBridge.toRobotModel(assembly.definition("slider"), parts);
  }

  static function testRobotMechanismAndTerrain():Void {
    var description = new CollisionDescription();
    var robot = RobotCollision.describe(description, arm(), armOptions("arm"));
    var mechanismOptions = new RobotCollisionOptions("slider");
    mechanismOptions.linkGroup = 2;
    mechanismOptions.rootPose = new Transform3(new Vec3(0, 3, 0), new Quat(0, 0, 0, 1));
    var mechanism = CadCollision.describeMechanism(description, slider(), mechanismOptions);
    var map = new HeightMap("cell", -2, -2, 0.5, 9, 9);
    var terrain = HeightMapCollision.describe(description, "terrain", map, -3, CollisionPose.translation(0, 0, -0.5), 3);
    check(description.articulations.length == 2, "the robot and the mechanism are articulations");
    check(robot.toolBody >= 0 && robot.toolObjects.length == 1 && robot.hulls.length == 1, "tool and hull described");
    check(mechanism.hulls.length == 2, 'the mechanism carries its CAD hulls (${mechanism.hulls.length})');

    var world = new NativeCollisionWorld();
    var build = description.build(world);
    check(build.layoutErrors.length == 0, 'a clean layout has no errors (${build.layoutErrors.length})');
    var objectId = (described:Int) -> build.objects[described];
    var foreBox = objectId(robot.shapes[2][0]), toolBox = objectId(robot.toolObjects[0]);
    check(world.pairStatus(foreBox, toolBox) == CollisionPairStatus.Rigid, "the tool is rigid with its flange link");
    check(world.pairStatus(objectId(robot.shapes[1][0]), toolBox) == CollisionPairStatus.Adjacent,
      "the tool takes its link's adjacency");
    check(world.pairStatus(objectId(robot.shapes[0][0]), objectId(mechanism.hulls[0])) == CollisionPairStatus.Static,
      "two fixed bases are static");
    check(world.pairStatus(toolBox, objectId(terrain)) == CollisionPairStatus.Checked, "the tool is checked against terrain");
    check(world.pairStatus(toolBox, objectId(mechanism.hulls[1])) == CollisionPairStatus.Checked,
      "the tool is checked against the mechanism's moving part");

    // Turn the arm toward the mechanism (shoulder 90 degrees): its tool box reaches y = 2.1; the slider's
    // cube (50 mm half side) spans y 2.95..3.05 at rest.
    robot.bodies.place(build, robot.bodies.state([Math.PI / 2, 0.0]));
    var gap = world.pairDistances([new CollisionPair(toolBox, objectId(mechanism.hulls[1]), -1, -1)])[0].distance;
    check(Math.abs(gap - (2.95 - 2.1)) < 1e-6, 'the tool against the slider ($gap)');
    // The slider moves out of the mechanism's own model: 0.1 m along y.
    mechanism.bodies.place(build, mechanism.bodies.state([0.1]));
    gap = world.pairDistances([new CollisionPair(toolBox, objectId(mechanism.hulls[1]), -1, -1)])[0].distance;
    check(Math.abs(gap - (3.05 - 2.1)) < 1e-6, 'the slider follows its own model ($gap)');
    // Digging is a height update in the field's layout; the terrain stays clear of the arm.
    map.lowerTo(4, 4, -1.0);
    world.setHeights(build.objects[terrain], HeightMapCollision.heights(map));
    var near = world.distances(0.6).filter(d -> d.a == build.objects[terrain] || d.b == build.objects[terrain]);
    check(near.length > 0 && near[0].distance > 0.3, "the terrain lies under the arm");
    check(description.assumptions.length > 0, "the description records its assumptions");
    world.dispose();
  }

  /** A fixture through the robot at its reference is a layout error, not an allowed overlap (CL-D3). */
  static function testFixtureThroughTheRobotIsALayoutError():Void {
    var description = new CollisionDescription();
    var robot = RobotCollision.describe(description, arm(), armOptions("arm"));
    var fixture = description.addFixed("fixture", CollisionPose.translation(1.5, 0, 0), CollisionGeometry.Box(0.1, 0.1, 0.1),
      3, 0.0, "primitive");
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    var foreBox = build.objects[robot.shapes[2][0]];
    var errors = build.layoutErrors.filter(p -> (p.a == foreBox || p.b == foreBox)
      && (p.a == build.objects[fixture] || p.b == build.objects[fixture]));
    check(errors.length == 1, 'the fixture through the forearm is a layout error (${build.layoutErrors.length})');
    check(world.pairStatus(foreBox, build.objects[fixture]) == CollisionPairStatus.Checked, "and stays checked");
    world.dispose();
  }

  /** CL-D10: a PairsOnly shape is still checked against a fixture, and the contact pair is allowed with its reason. */
  static function testSimulationContactSettings():Void {
    var description = new CollisionDescription();
    var robot = RobotCollision.describe(description, arm(), armOptions("arm"));
    var fixture = description.addFixed("fixture", CollisionPose.translation(1.95, 0.0, 0.3), CollisionGeometry.Box(0.05, 0.05, 0.05),
      3, 0.0, "primitive");
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    var left = build.objects[robot.shapes[2][1]], right = build.objects[robot.shapes[2][2]];
    check(world.pairStatus(left, right) == CollisionPairStatus.DeclaredContact, "a contact pair is allowed as declared contact");
    check(world.pairStatus(left, build.objects[fixture]) == CollisionPairStatus.Checked,
      "a PairsOnly shape is still checked against a fixture");
    var pads = world.distances(1.0).filter(d -> d.b == build.objects[fixture] && (d.a == left || d.a == right));
    check(pads.length == 2 && Math.abs(pads[0].distance - (0.25 - 0.12)) < 1e-6, 'the pads are measured (${pads.length})');
    var ignored = description.assumptions.filter(a -> a.indexOf("simulation-only") >= 0);
    check(ignored.length == 2, 'the ignored settings are recorded (${ignored.length})');
    var flagged = new RobotCollisionOptions("arm-2");
    flagged.selfCollision = false;
    flagged.rootPose = new Transform3(new Vec3(0, -5, 0), new Quat(0, 0, 0, 1));
    RobotCollision.describe(description, arm(), flagged);
    check(description.assumptions.filter(a -> a.indexOf("self-collision") >= 0).length == 1,
      "the self-collision opt-out is recorded as ignored");
    world.dispose();
  }

  /** A non-convex L-shaped part decomposes into pieces that, inflated, enclose its tessellation; the cache reuses them. */
  static function testDecomposedPartEnclosesItsTessellation():Void {
    var a = Shape.box(100, 25, 20), b = Shape.box(25, 100, 20);
    var part = a.fuse(b);
    var decomposer = new NativeConvexDecomposer(50000);
    var cache = new CadDecompositionCache();
    var description = new CollisionDescription();
    var pieces = CadCollision.describePart(description, "bracket", AssemblyFrames.identity(), 0.001,
      CadPartCollision.Decomposed(part, 0.1, 8), 3, decomposer, cache);
    check(pieces.length >= 2, 'the L needs several pieces (${pieces.length})');
    var inflation = description.objects[pieces[0]].inflation;
    check(inflation > 0 && inflation < 0.005, 'the inflation is small against the part ($inflation)');
    var recorded = description.assumptions.filter(t -> t.indexOf("bracket") >= 0 && t.indexOf("inflated") >= 0);
    check(recorded.length == 1, "the inflation is recorded");
    if (recorded.length == 1) Sys.println(recorded[0]);
    // Again: the cached pieces are reused.
    CadCollision.describePart(description, "bracket", AssemblyFrames.identity(), 0.001,
      CadPartCollision.Decomposed(part, 0.1, 8), 3, decomposer, cache);
    check(cache.decompositions == 1, "a part is decomposed once");

    // Every tessellation vertex, and points across its triangles, lies inside the inflated pieces.
    var mesh = CadCollisionMesh.of(part.tessellate(0.1), 0.001);
    var world = new NativeCollisionWorld();
    var probeBody = world.addBody(0);
    var objects = [for (i in 0...pieces.length) {
      var object = description.objects[pieces[i]];
      var id = world.add(-1, CollisionPose.identity(), object.geometry);
      world.setInflation(id, object.inflation);
      id;
    }];
    var probe = world.add(probeBody, CollisionPose.identity(), CollisionGeometry.Sphere(1e-7));
    var worst = Math.NEGATIVE_INFINITY;
    var t = 0;
    while (t + 2 < mesh.indices.length) {
      for (weights in [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0], [1 / 3, 1 / 3, 1 / 3], [0.5, 0.5, 0], [0.2, 0.1, 0.7]]) {
        var p = [for (k in 0...3) weights[0] * mesh.vertices[3 * mesh.indices[t] + k]
          + weights[1] * mesh.vertices[3 * mesh.indices[t + 1] + k] + weights[2] * mesh.vertices[3 * mesh.indices[t + 2] + k]];
        world.setBodyPose(probeBody, CollisionPose.translation(p[0], p[1], p[2]));
        var nearest = Math.POSITIVE_INFINITY;
        for (d in world.pairDistances([for (o in objects) new CollisionPair(probe, o, probeBody, -1)]))
          nearest = Math.min(nearest, d.distance + 1e-7);
        worst = Math.max(worst, nearest);
      }
      t += 3;
    }
    check(worst <= 1e-9, 'the inflated pieces enclose the tessellation (worst $worst)');
    world.dispose();
    part.close();
    a.close();
    b.close();
  }

  /** The hulls ArmClearance is given (link hulls, a tool hull, parts and welds on the base link) land on the same bodies. */
  static function testArmClearanceBodies():Void {
    var description = new CollisionDescription();
    var robot = RobotCollision.describe(description, arm(), armOptions("arm"));
    var objects = RobotCollision.describeClearanceBodies(description, robot, [
      {name: "upper-hull", link: "upper", vertices: cube(0.5, 0, 0, 0.05), tool: false},
      {name: "torch", link: "fore", vertices: cube(1.05, 0, 0, 0.03), tool: true},
      {name: "workpiece", link: "base", vertices: cube(1.5, 1.0, 0, 0.1), tool: false}
    ]);
    check(description.objects[objects[0]].body == robot.linkBody("upper"), "a link hull is on its link");
    check(description.objects[objects[1]].body == robot.toolBody, "a tool hull is on the tool body");
    check(description.objects[objects[2]].body == robot.linkBody("base"), "a part carried by the base is on the base");
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    check(build.layoutErrors.length == 0, "the clearance scene lays out cleanly");
    check(world.pairStatus(build.objects[objects[1]], build.objects[objects[2]]) == CollisionPairStatus.Checked,
      "the torch is checked against the workpiece");
    world.dispose();
  }

  /** A cube's corners (x, y, z triples) centred at (x, y, z) with half side `half`. */
  static function cube(x:Float, y:Float, z:Float, half:Float):Array<Float>
    return [for (i in 0...8) for (axis in 0...3) (axis == 0 ? x : axis == 1 ? y : z) + (((i >> axis) & 1) == 1 ? half : -half)];

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
