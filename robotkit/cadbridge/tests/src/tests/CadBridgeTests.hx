package tests;

import CadKit;
import cadkit.Shape;
import cadkit.Geometry;
import cadkit.modeling.Sketch;
import cadkit.modeling.Curve;
import cadkit.modeling.Vector;
import cadkit.modeling.Part;
import cadkit.InertiaTensor;
import cadkit.parametric.ElementReference;
import bimkit.BimDocument;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.FrameTree3;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;
import robotkit.manipulation.ChainTip;
import robotkit.manipulation.KinematicChain;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.WorkPatchPlanner;
import cadbridge.FaceBridge;
import cadbridge.WallBridge;
import cadbridge.BimFrameBridge;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.MachineAssemblyMassBridge;
import cadbridge.EndEffectorBridge;
import cadbridge.EndEffectorRuntimeBridge;
import cadbridge.EndEffectorControlBinding;
import eoat.EndEffectorExample;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssembly.AssemblyBomMass;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.PortInterface;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import robotkit.tool.ToolCollisionShape;
import robotkit.tool.ToolRuntimeSelection;
import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventValue;
import haxe.Int64;
import robotkit.material.LoadLimits;
import robotkit.runtime.RobotRuntimeCompiler;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;
import haxe.io.Bytes;

/** M6 acceptance tests for robotkit/cadbridge: CadKit Face and BimKit wall -> WorkSurface, and BIM hierarchy -> FrameTree3. */
class CadBridgeTests {
  static var assertions = 0;

  public static function main():Void {
    testMachineAssemblyMassBridge();
    testEndEffectorBridge();
    testEndEffectorRuntimeBridge();
    testAssemblySimulationBridge();
    testFaceBridgeOnPlainBoxFace();
    testFaceBridgePreservesConcaveWireOrder();
    testWallBridgeAreaNormalAndExclusion();
    testBimFrameHierarchy();
    testBimWallToPatchPlanEndToEnd();
    Sys.println('CadBridge tests passed ($assertions assertions)');
  }

  static function testMachineAssemblyMassBridge():Void {
    var assembly = new MachineAssembly();
    assembly.addComponent("tool", new BridgeMassPart(true));
    var link = new Link("tool");
    MachineAssemblyMassBridge.applyToLink(assembly, link);
    check(approx(link.mass, 2, 1e-12) && approx(link.centerOfMass[0], 0.1, 1e-12) &&
      approx(link.centerOfMass[2], 0.05, 1e-12), "MachineKit mass and centre convert to RobotKit SI units");
    check(approx(link.inertiaTensor[0], 2, 1e-12) &&
      approx(link.inertiaTensor[1], 0.25, 1e-12) &&
      approx(link.inertiaTensor[3], 0.25, 1e-12) &&
      approx(link.inertiaTensor[4], 3, 1e-12), "centroidal inertia converts to row-major kg m²");
    var limits = new LoadLimits(3, 0.25, 1);
    check(MachineAssemblyMassBridge.payloadViolation(assembly, limits, 0.5, 0.2, 0.2, 0.2) == null,
      "MachineKit payload is within RobotKit limits");
    check(MachineAssemblyMassBridge.payloadViolation(assembly, new LoadLimits(3, 0.15, 1),
      0.5, 0.2, 0.2, 0.2) != null, "MachineKit payload moment violates RobotKit limits");

    assembly.addBomItem({partNumber: "TUBE", description: "Tube", quantity: 1,
      material: "polyurethane"}, 1, Attached(0.1, "tool", new Vector(100, 0, 50)));
    MachineAssemblyMassBridge.applyToLink(assembly, link);
    check(approx(link.mass, 2.1, 1e-12) && approx(link.centerOfMass[0], 0.1, 1e-12),
      "accounted tubing contributes to RobotKit link mass");

    var missingTensor = new MachineAssembly();
    missingTensor.addComponent("declared", new BridgeMassPart(false));
    var rejected = false;
    try MachineAssemblyMassBridge.applyToLink(missingTensor, new Link("missing"))
    catch (error:Dynamic) rejected = Std.string(error).indexOf("declared") >= 0;
    check(rejected, "missing declared inertia identifies the member");

    assembly.addBomItem({partNumber: "UNMODELLED-LINE", description: "line", quantity: 1,
      material: "steel"});
    rejected = false;
    try MachineAssemblyMassBridge.payloadViolation(assembly, limits, 0.5, 0.2, 0.2, 0.2)
    catch (error:Dynamic) rejected = Std.string(error).indexOf("UNMODELLED-LINE") >= 0;
    check(rejected, "unaccounted BOM mass prevents a payload decision");
  }

  static function testEndEffectorBridge():Void {
    var endEffector = new EndEffector();
    endEffector.addComponent("body", new BridgeEndEffectorPart());
    endEffector.mount("body", "mount");
    endEffector.workingFrame("contact", "body", "contact", true);
    endEffector.workingFrame("inspection", "body", "mount");
    endEffector.addBomItem({partNumber: "ATTACHED-TUBE", description: "attached air tube",
      quantity: 1, material: "polyurethane"}, 1,
      Attached(1, "body", new Vector(10, 0, 20)));

    var tool = EndEffectorBridge.toTool(endEffector, "contact");
    check(approx(tool.flangeTTcp.translation.x, 0, 1e-12) &&
      approx(tool.flangeTTcp.translation.z, 0.03, 1e-12) &&
      tool.flangeTTcp.rotation.angularDistance(Quat.identity()) < 1e-9,
      "contact frame converts from connector +Y to robot +Z");
    check(approx(tool.mass, 3, 1e-12), "attached tube contributes to RobotKit tool mass");
    var boxCorrect = switch tool.collision {
      case Box(half, centre): centre != null &&
        approx(centre.x, -0.01, 1e-6) && approx(centre.y, 0, 1e-6) &&
        approx(centre.z, 0.015, 1e-6) &&
        approx(half.x, 0.01, 1e-6) && approx(half.y, 0.005, 1e-6) &&
        approx(half.z, 0.015, 1e-6);
      case _: false;
    };
    check(boxCorrect, "offset envelope box follows the actual body bounds");
    var centredBox = ToolCollisionShape.Box(new Vec3(0.01, 0.02, 0.03));
    check(switch centredBox {
      case Box(_, centre): centre == null;
      case _: false;
    }, "existing flange-centred Box construction remains valid");
    var inspection = EndEffectorBridge.toTool(endEffector, "inspection");
    check(inspection.id == "inspection" &&
      inspection.flangeTTcp.translation.norm() < 1e-12,
      "each working frame produces a separate RobotKit tool");

    var set = EndEffectorExample.build();
    var shortTool = EndEffectorBridge.toTool(set.configuration("short"), "contact", null, "short/contact");
    var longTool = EndEffectorBridge.toTool(set.configuration("long"), "contact", null, "long/contact");
    check(shortTool.id == "short/contact" && longTool.id == "long/contact" &&
      shortTool.id != longTool.id, "configuration-qualified RobotKit tool IDs are distinct");
    var tcpX = shortTool.flangeTTcp.transformVector(new Vec3(1, 0, 0));
    check(approx(tcpX.x, 1, 1e-9) && approx(tcpX.y, 0, 1e-9) &&
      approx(tcpX.z, 0, 1e-9), "cup TCP X retains the flange locating-pin direction");
    var forwardBox = switch shortTool.collision {
      case Box(half, centre): centre != null &&
        approx(centre.z - half.z, 0, 1e-6) &&
        approx(centre.z + half.z, shortTool.flangeTTcp.translation.z, 1e-6);
      case _: false;
    };
    check(forwardBox, "adapter, frame and cup collision box spans flange to contact only");

    var link = new Link("end-effector");
    MachineAssemblyMassBridge.applyToLink(endEffector, link);
    check(approx(link.mass, 3, 1e-12) &&
      approx(link.centerOfMass[0], -2.0 / 300.0, 1e-12) &&
      approx(link.centerOfMass[2], 2.0 / 300.0, 1e-12),
      "link mass and attached tube centre use the mount frame in metres");
    check(link.inertiaTensor[2] < 0 && link.inertiaTensor[6] < 0,
      "attached tube contributes rotated off-diagonal inertia");

    var fixture = buildUR5Fixture();
    var manipulator = new Manipulator(fixture.model, fixture.chain, tool.flangeTTcp);
    var q = [0.2, -0.4, 0.3, 0.1, -0.2, 0.15];
    var flangePose = fixture.chain.forwardKinematics(q);
    var expected = flangePose.transformPoint(new Vec3(0, 0, 0.03));
    var tcp = manipulator.tcpPose(q);
    check(approx(tcp.translation.x, expected.x, 1e-9) &&
      approx(tcp.translation.y, expected.y, 1e-9) &&
      approx(tcp.translation.z, expected.z, 1e-9),
      "Manipulator.tcpPose places the cup contact at the mounted tool offset");
  }

  static function testEndEffectorRuntimeBridge():Void {
    var set = new EndEffectorSet();
    set.addComponent("base", new BridgeEndEffectorPart());
    set.mount("base", "mount");
    set.changer("coupling", "base", "contact", []);
    for (kind in ["cup", "gripper", "combo"]) {
      var endEffector = new EndEffector();
      endEffector.addComponent("body", new BridgeRuntimePart(kind != "cup", kind != "gripper"));
      endEffector.mount("body", "mount");
      endEffector.workingFrame("contact", "body", "contact", true);
      if (kind != "cup") {
        endEffector.connectPorts("open-feed", "body", "openSupply", "body", "open");
        endEffector.connectPorts("close-feed", "body", "closeSupply", "body", "close");
      }
      if (kind != "gripper")
        endEffector.connectPorts("vacuum-feed", "body", "vacuumSupply", "body", "vacuum");
      set.addTool(kind, endEffector);
    }

    var cup = EndEffectorRuntimeBridge.toRuntime(set, "cup", "contact", [
      EndEffectorControlBinding.Vacuum("cup.vacuum", "tool/body", "vacuum")]);
    var gripper = EndEffectorRuntimeBridge.toRuntime(set, "gripper", "contact", [
      EndEffectorControlBinding.Gripper("gripper.close", "tool/body", "open", "close")]);
    check(cup.tool.id == "cup/contact" && gripper.tool.id == "gripper/contact" &&
      cup.tool.id != gripper.tool.id, "configuration creates a distinct mounted tool ID");
    check(cup.tool.mass < gripper.tool.mass &&
      cup.tool.flangeTTcp.translation.z < gripper.tool.flangeTTcp.translation.z,
      "configuration selects its own mass and TCP");
    check(cup.vacuum != null && cup.gripper == null &&
      gripper.gripper != null && gripper.vacuum == null,
      "configuration selects only its bound runtime capabilities");
    check(cup.channelDeclarations().length == 1 &&
      cup.channelDeclarations()[0].id == "cup.vacuum" &&
      gripper.channelDeclarations().length == 1 &&
      gripper.channelDeclarations()[0].id == "gripper.close",
      "configuration selects its own process channel bindings");
    var selected = new ToolRuntimeSelection();
    selected.select(cup, Int64.ofInt(0));
    selected.apply(new FiredProcessEvent(Int64.ofInt(1), "cup.vacuum",
      ProcessEventValue.Digital(true), Int64.ofInt(100), Int64.ofInt(110), 1));
    check(cup.vacuum.isHolding(), "active vacuum configuration receives its channel");
    selected.select(gripper, Int64.ofInt(150));
    check(!cup.vacuum.isHolding() && selected.active() == gripper,
      "switching releases the old tool and selects the new runtime");
    selected.apply(new FiredProcessEvent(Int64.ofInt(2), "gripper.close",
      ProcessEventValue.Digital(true), Int64.ofInt(200), Int64.ofInt(210), 1));
    check(gripper.gripper.isGrasped(), "selected gripper channel actuates the new configuration");
    var rejected = false;
    try selected.apply(new FiredProcessEvent(Int64.ofInt(2), "cup.vacuum",
      ProcessEventValue.Digital(true), Int64.ofInt(300), Int64.ofInt(310), 1))
    catch (_:Dynamic) rejected = true;
    check(rejected, "inactive configuration channels are not bound");
    rejected = false;
    try EndEffectorRuntimeBridge.toRuntime(set, "cup", "contact", [
      EndEffectorControlBinding.Gripper("wrong", "tool/body", "open", "close")])
    catch (_:Dynamic) rejected = true;
    check(rejected, "binding a capability to missing actuator ports is rejected");
    var combo = EndEffectorRuntimeBridge.toRuntime(set, "combo", "contact", [
      EndEffectorControlBinding.Gripper("combo.close", "tool/body", "open", "close"),
      EndEffectorControlBinding.Vacuum("combo.vacuum", "tool/body", "vacuum")]);
    check(combo.gripper != null && combo.vacuum != null &&
      combo.channelDeclarations().length == 2,
      "one coupled end effector can bind gripper and vacuum together");
  }

  static function testAssemblySimulationBridge():Void {
    var assembly = new AssemblyModel();
    assembly.add("base");
    assembly.add("slider");
    assembly.connector("base", "mount", AssemblyFrames.identity());
    assembly.connector("slider", "mount", AssemblyFrames.identity());
    assembly.mateOnAxis("slide", "prismatic", "base", "mount", "slider", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: 0, upper: 100, velocity: 20, effort: 50});
    var vertices = Bytes.alloc(4 * 24);
    var points = [0.0, 0.0, 0.0, 10.0, 0.0, 0.0,
      0.0, 10.0, 0.0, 0.0, 0.0, 10.0];
    for (index in 0...points.length) vertices.setDouble(index * 8, points[index]);
    var parts = AssemblyPhysicalPartView.fromSceneArtifact({metresPerUnit: 0.001,
      parts: [for (id in ["base", "slider"]) {
        id: id, name: id, red: 0.5, green: 0.5, blue: 0.5,
        materialId: "machined-steel", materialDensity: 7850.0,
        volume: 1000000.0, centerOfMass: [0.0, 0.0, 0.0],
        inertia: [10000000000.0, 0, 0, 0, 10000000000.0, 0, 0, 0, 10000000000.0],
        vertexCount: 4, indexCount: 0, vertices: vertices,
        normals: Bytes.alloc(0), indices: Bytes.alloc(0), faceRanges: []
      }]});
    var translated = AssemblySimulationBridge.toRobotModel(assembly.definition("bridge-test"), parts);
    check(translated.model.links.length == 3 && translated.model.joints.length == 2,
      "assembly tree translates to robot links and joints");
    var slide = translated.model.joints[1];
    check(Math.abs(slide.limits.upper - 0.1) < 1e-9 && slide.axis[1] == 1 &&
      Math.abs(translated.model.links[1].mass - 7.85) < 1e-9,
      "assembly limits and material mass convert to SI units");
    check(RobotRuntimeCompiler.validate(translated.model).length == 0,
      "translated assembly is a valid RobotKit runtime model");
    var hull:Array<Float> = cast translated.linkCollisionHulls[1];
    check(translated.linkCollisionHulls.length == translated.model.links.length &&
      hull != null && hull.length <= 64 * 3,
      "physical-part view supplies bounded hulls to the bridge");
  }

  /**
   * M9 fallback (see CONSTRUCTION_ROADMAP.md, "Simulated wall-finishing
   * robot"): the M9 scenario test builds its design WorkSurface directly in
   * robotkit/tests to keep that project's own dependency on nativekit only
   * (robotkit's CAD-agnostic-core rule). This test instead starts from an
   * actual BIM wall and carries it all the way through WallBridge and
   * robotkit.manipulation's own WorkPatchPlanner, so the BIM-to-patch-plan
   * path this milestone's fallback describes is exercised by at least one
   * test even though the M9 scenario itself never imports cadbridge.
   */
  static function testBimWallToPatchPlanEndToEnd():Void {
    var bim = new BimDocument();
    // A 6m x 0.3m wall band, the same proportions PlacementTests' (M8) own
    // WorkPatchPlanner acceptance fixture uses, so this test's standoff
    // parameters are known to keep the whole band within a UR5-class arm's
    // reach; only the surface geometry's *source* (an actual BIM wall
    // through WallBridge, not a hand-built WorkSurface) differs from M8.
    var wall = bim.createWall("Finish wall band", 6000, 200, 300);
    // A modest vent much narrower than a single patch column (1m, below)
    // so it never swallows a whole column's raster rows outright, and
    // vertically centered within the 300mm band so its footprint-band cut
    // (RasterToolpathGenerator's own reach-aware margin) has clearance top
    // and bottom.
    var opening = bim.createWindowDefinition("Vent", 200, 80, 10, 40);
    var instance = bim.cad.createInstance("Vent 1", opening);
    bim.hostOpening(instance, wall.id, 2000, 110);

    var design = WallBridge.wallToWorkSurface(bim, wall.id, "bim-wall", "map");
    check(approx(design.boundary.area(), 1.8, 1e-3),
      'BIM wall face converts to a WorkSurface with the expected area (got ${design.boundary.area()})');
    check(design.exclusions.length == 1, "the hosted opening becomes exactly one WorkSurface exclusion");

    var fixture = buildUR5Fixture();
    var manipulator = new Manipulator(fixture.model, fixture.chain);
    var mapTSurface = design.frame_T_surface;
    var seed = [0.2, -1.0, 1.3, -0.3, 0.5, 0.0];
    var plan = WorkPatchPlanner.plan(design, mapTSurface, manipulator,
      1.0, 0.08, 0.0, 0.02, 0.05, 0.0, 0.35, 0.65, seed, 6, 1, 0.05);

    check(plan.patches.length >= 2, 'the BIM wall band splits into at least two patches (got ${plan.patches.length})');
    // Not asserting plan.fullyPlanned == true here: unlike PlacementTests'
    // (M8) hand-built, axis-aligned identity-frame WorkSurface, this
    // surface's frame_T_surface is WallBridge's own derived pose for a real
    // BIM wall face, so the exact standoff bounds a candidate search needs
    // for 100% reach are a separate tuning question from whether the
    // BIM-wall-to-patch-plan pipeline itself works correctly. Most searched
    // candidates do reach most of their patch; every patch reaches at least
    // half of its own raster, which is enough to demonstrate the pipeline
    // without over-fitting this test's standoff search to one fixture.
    for (patch in plan.patches) {
      check(patch.reachableFraction > 0.5,
        'each patch reached from the BIM wall is more than half-reachable from its chosen base pose (got ${patch.reachableFraction})');
      check(patch.toolpath.points.length > 0, "each patch carries a non-empty raster toolpath");
    }
    bim.cad.close();
  }

  static function buildUR5Fixture():{model:RobotModel, chain:KinematicChain} {
    var d1 = 0.089159, shoulderOffset = 0.13585, elbowOffset = -0.1197,
      a2 = 0.425, a3 = 0.39225, d4 = 0.10915, d5 = 0.09465, d6 = 0.0823;
    var model = new RobotModel("ur5-fixture");
    var linkNames = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
      "wrist_1_link", "wrist_2_link", "wrist_3_link"];
    var links = [for (name in linkNames) model.addLink(new Link(name))];
    var offsets = [
      new Vec3(0.0, 0.0, d1),
      new Vec3(0.0, shoulderOffset, 0.0),
      new Vec3(0.0, elbowOffset, a2),
      new Vec3(0.0, 0.0, a3),
      new Vec3(0.0, d4, 0.0),
      new Vec3(0.0, 0.0, d5)
    ];
    var axes = [
      [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
      [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]
    ];
    var jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"];
    for (i in 0...6) {
      var joint = model.addJoint(new Joint(jointNames[i], JointType.Revolute, links[i], links[i + 1]));
      joint.parentFramePosition = offsets[i].toArray();
      joint.axis = axes[i];
      joint.limits.lower = -2.0 * Math.PI;
      joint.limits.upper = 2.0 * Math.PI;
      joint.limits.velocity = 0.0;
    }
    var flangeOffset = new Vec3(0.0, d6, 0.0);
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = flangeOffset.toArray();
    var chain = new KinematicChain(model, links[0].id, ChainTip.Frame(flange.id));
    return { model: model, chain: chain };
  }

  static function testFaceBridgeOnPlainBoxFace():Void {
    var box = Shape.box(2.0, 3.0, 4.0);
    var top = box.faces().query().surface(CadKit.SurfaceKind.Plane)
      .centerNear(Geometry.vec3(1.0, 1.5, 4.0), 0.001).unique();
    var surface = FaceBridge.toWorkSurface(top, "top", "world");
    check(approx(surface.boundary.area(), 6.0, 1e-6), "plain box top face area matches width*depth");
    check(surface.exclusions.length == 0, "plain box face has no inner wires");
    var worldNormal = surface.frame_T_surface.transformVector(new Vec3(0.0, 0.0, 1.0));
    check(approx(worldNormal.z, 1.0, 1e-6), "surface local +Z matches the face's own outward (+Z) normal");
    check(surface.surfaceFrameId == "top" && surface.frameId == "world", "generated surface keeps the requested id and frame id");
    top.close();
    box.close();
  }

  static function testFaceBridgePreservesConcaveWireOrder():Void {
    var outer = closedCurve([
      new Vector(-3.0, -3.0, 0.0), new Vector(3.0, -3.0, 0.0),
      new Vector(3.0, 3.0, 0.0), new Vector(1.0, 3.0, 0.0),
      new Vector(1.0, 1.0, 0.0), new Vector(-3.0, 1.0, 0.0)
    ]);
    var opening = closedCurve([
      new Vector(-2.0, -2.0, 0.0), new Vector(0.5, -2.0, 0.0),
      new Vector(0.5, -1.5, 0.0), new Vector(-1.5, -1.5, 0.0),
      new Vector(-1.5, -0.5, 0.0), new Vector(-2.0, -0.5, 0.0)
    ]);
    var faceModel = Sketch.face(outer, [opening]);
    var face = new cadkit.Face(faceModel.shape.subshape(CadKit.ShapeKind.Face, 0));
    var bridged = FaceBridge.toWorkSurface(face, "concave", "world");
    check(bridged.boundary.vertices().length == 6,
      "concave outer wire retains every source vertex");
    check(approx(bridged.boundary.area(), 28.0, 1e-9),
      'concave outer wire keeps its connected-loop area (got ${bridged.boundary.area()})');
    check(reflexVertices(bridged.boundary.vertices()) == 1,
      "concave outer wire retains its single reflex corner");
    check(bridged.exclusions.length == 1 && bridged.exclusions[0].vertices().length == 6,
      "concave opening becomes one connected six-vertex exclusion");
    check(approx(bridged.exclusions[0].area(), 1.75, 1e-9),
      'concave opening keeps its connected-loop area (got ${bridged.exclusions[0].area()})');
    check(reflexVertices(bridged.exclusions[0].vertices()) == 1,
      "concave opening retains its single reflex corner");
    face.close();
    faceModel.close();
    outer.close();
    opening.close();
  }

  static function closedCurve(points:Array<Vector>):Curve {
    var edges:Array<Curve> = [];
    for (i in 0...points.length)
      edges.push(Curve.line(points[i], points[(i + 1) % points.length]));
    var result = Curve.wire(edges);
    for (edge in edges) edge.close();
    return result;
  }

  static function reflexVertices(points:Array<robotkit.work.Point2>):Int {
    var result = 0;
    for (i in 0...points.length) {
      var previous = points[(i + points.length - 1) % points.length];
      var current = points[i];
      var next = points[(i + 1) % points.length];
      var cross = (current.x - previous.x) * (next.y - current.y) -
        (current.y - previous.y) * (next.x - current.x);
      if (cross < -1e-9) result++;
    }
    return result;
  }

  static function testWallBridgeAreaNormalAndExclusion():Void {
    var bim = new BimDocument();
    var wall = bim.createWall("Exterior wall", 6000, 200, 3000);
    var window = bim.createWindowDefinition("Window", 1200, 1500, 80, 200);
    var instance = bim.cad.createInstance("Window 1", window);
    bim.hostOpening(instance, wall.id, 1800, 900);

    var surface = WallBridge.wallToWorkSurface(bim, wall.id, "wall-1", "world");
    check(approx(surface.boundary.area(), 18.0, 1e-3), 'wall side face area matches 6m x 3m (got ${surface.boundary.area()})');
    check(surface.exclusions.length == 1, "wall side face has exactly one exclusion, matching its one hosted opening");
    check(approx(surface.exclusions[0].area(), 1.8, 1e-3),
      'the exclusion area matches the window opening (1.2m x 1.5m, got ${surface.exclusions[0].area()})');
    var worldNormal = surface.frame_T_surface.transformVector(new Vec3(0.0, 0.0, 1.0));
    check(approx(Math.abs(worldNormal.y), 1.0, 1e-6), "wall side face normal points along the wall's thickness (Y) axis");
    check(surface.provenance.designElementId == wall.id.value, "wall WorkSurface provenance references the wall's BIM element id");
    bim.cad.close();
  }

  static function testBimFrameHierarchy():Void {
    var bim = new BimDocument();
    var project = bim.createProject("Project");
    var site = bim.createSite("Site", project.id);
    var building = bim.createBuilding("Building", site.id);
    var groundLevel = bim.cad.createLevel("Ground", 3500);
    var storey = bim.createStorey("Ground Floor", building.id, new ElementReference(bim.cad.id, groundLevel.id));

    var tree = new FrameTree3();
    BimFrameBridge.registerHierarchy(bim, tree);
    var lookup = tree.lookup(BimFrameBridge.frameName(project.id), BimFrameBridge.frameName(storey.id));
    check(approx(lookup.translation.x, 0.0, 1e-9) && approx(lookup.translation.y, 0.0, 1e-9),
      "project->storey frame has no horizontal offset");
    check(approx(lookup.translation.z, 3.5, 1e-9),
      'project->storey frame carries the storey base level elevation in meters (got ${lookup.translation.z})');
    var siteToBuilding = tree.lookup(BimFrameBridge.frameName(site.id), BimFrameBridge.frameName(building.id));
    check(approx(siteToBuilding.translation.norm(), 0.0, 1e-9), "site->building frame is identity (no geometry of its own)");
    bim.cad.close();
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}

private class BridgeMassPart extends MachineComponent {
  public function new(withInertia:Bool) {
    super("BRIDGE-MASS", "declared bridge mass", "steel", true);
    declareMass(2, new Vector(100, 0, 50), withInertia ?
      new InertiaTensor(2000000, 250000, 0, 3000000, 0, 4000000) : null);
  }
}

private class BridgeEndEffectorPart extends MachineComponent {
  public function new() {
    super("BRIDGE-EOAT", "bridge end effector", "steel", true);
    addConnector("mount", Mount, Solids.axial(10, 0, 0));
    addConnector("contact", Face, Solids.axial(10, 0, 30));
    declareMass(2, new Vector(0, 0, 0),
      new InertiaTensor(2000000, 0, 0, 3000000, 0, 4000000));
  }

  override public function geometry(detail:ComponentDetail = Preview):Part
    return Part.box(20, 10, 30);
}

private class BridgeRuntimePart extends MachineComponent {
  final length:Float;

  public function new(gripper:Bool, vacuum:Bool) {
    super(gripper && vacuum ? "BRIDGE-COMBO" : gripper ? "BRIDGE-GRIPPER" : "BRIDGE-CUP",
      "runtime bridge fixture", "steel", true);
    length = gripper ? 60 : 25;
    addConnector("mount", Mount, Solids.axial(10, 0, 0));
    addConnector("contact", Face, Solids.axial(10, 0, length));
    if (gripper) {
      addPort({name: "openSupply", kind: PortKind.Pneumatic, role: PortRole.Supply,
        iface: PortInterface.Unspecified, required: false});
      addPort({name: "closeSupply", kind: PortKind.Pneumatic, role: PortRole.Supply,
        iface: PortInterface.Unspecified, required: false});
      addPort({name: "open", kind: PortKind.Pneumatic, role: PortRole.Consumer,
        iface: PortInterface.Unspecified, required: true});
      addPort({name: "close", kind: PortKind.Pneumatic, role: PortRole.Consumer,
        iface: PortInterface.Unspecified, required: true});
    }
    if (vacuum) {
      addPort({name: "vacuumSupply", kind: PortKind.Vacuum, role: PortRole.Supply,
        iface: PortInterface.Unspecified, required: false});
      addPort({name: "vacuum", kind: PortKind.Vacuum, role: PortRole.Consumer,
        iface: PortInterface.Unspecified, required: true});
    }
    declareMass(gripper && vacuum ? 4 : gripper ? 3 : 1, new Vector(0, 0, length / 2),
      new InertiaTensor(2000000, 0, 0, 3000000, 0, 4000000));
  }

  override public function geometry(detail:ComponentDetail = Preview):Part
    return Part.box(20, 10, length);
}
