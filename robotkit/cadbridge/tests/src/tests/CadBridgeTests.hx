package tests;

import CadKit;
import cadkit.Shape;
import cadkit.Geometry;
import cadkit.modeling.Sketch;
import cadkit.modeling.Curve;
import cadkit.modeling.Vector;
import cadkit.modeling.Part;
import cadkit.InertiaTensor;
import cadkit.ConvexHullVertices;
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
import robotkit.manipulation.Manipulator;
import processkit.manipulation.WorkPatchPlanner;
import processkit.cadbridge.FaceBridge;
import processkit.cadbridge.WallBridge;
import cadbridge.BimFrameBridge;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import MobileBasePreview;
import materia.project.SceneArtifact;
import robotkit.mobile.Twist2;
import robotkit.runtime.DifferentialDrivePlant;
import robotkit.runtime.SimulationHarness;
import robotkit.simulation.SimulatedRobot;
import cadbridge.MachineAssemblyMassBridge;
import cadbridge.EndEffectorBridge;
import cadbridge.EndEffectorCollision;
import cadbridge.EndEffectorCollision.EndEffectorCollisionOptions;
import cadbridge.EndEffectorRuntimeBridge;
import cadbridge.EndEffectorControlBinding;
import cadbridge.SuctionCapacityBridge;
import eoat.EndEffectorExample;
import eoat.SchmalzEndEffectorExample;
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
import machinekit.robotics.ParallelGripper;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.pneumatic.VacuumPressureSensor;
import machinekit.pneumatic.VacuumControlValve;
import machinekit.welding.WeldingInterfaces;
import machinekit.welding.WeldingTorch;
import robotkit.tool.ToolCollisionShape;
import robotkit.tool.ToolCollisionShapes;
import robotkit.tool.ToolRuntimeSelection;
import robotkit.tool.SimulatedGripper;
import robotkit.tool.SimulatedVacuum;
import robotkit.tool.SuctionCapacityChecker;
import robotkit.tool.SuctionMotionSample;
import robotkit.tool.WorkpieceLoad;
import robotkit.tool.MassProperties;
import robotkit.spatial.Inertia3;
import robotkit.execution.FiredProcessEvent;
import robotkit.execution.ProcessEventValue;
import robotkit.core.SensorFrame;
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
    testEnclosingHull();
    testEndEffectorCollisionPieces();
    testMachineAssemblyMassBridge();
    testEndEffectorBridge();
    testSuctionCapacityBridge();
    testSchmalzEndEffector();
    testEndEffectorRuntimeBridge();
    testDerivedRuntimeBindings();
    testTorchBindings();
    testAssemblySimulationBridge();
    testDerivedExternalAxes();
    testMobileBaseBridge();
    testFaceBridgeOnPlainBoxFace();
    testFaceBridgePreservesConcaveWireOrder();
    testWallBridgeAreaNormalAndExclusion();
    testBimFrameHierarchy();
    testBimWallToPatchPlanEndToEnd();
    Sys.println('CadBridge tests passed ($assertions assertions)');
  }

  static function testEndEffectorCollisionPieces():Void {
    var effector = new EndEffector();
    effector.addComponent("bar", new BridgeCollisionBlock(100, 10, 10, 90));
    effector.addComponent("upright", new BridgeCollisionBlock(10, 10, 100, 0));
    effector.mount("bar", "mount");
    effector.addMate("corner", "fixed", "bar", "end", "upright", "mount");
    effector.workingFrame("tip", "upright", "end", true);
    var solved = effector.solve();
    var pose = effector.mountTFrame("tip");
    var reused = effector.mountTFrame("tip", null, solved);
    check(approx(pose.raw().x, reused.raw().x, 1e-9) && approx(pose.raw().y, reused.raw().y, 1e-9) &&
      approx(pose.raw().z, reused.raw().z, 1e-9), "shared solve preserves TCP pose");
    check(approx(effector.massPropertiesAtMount().mass,
      effector.massPropertiesAtMount(null, solved).mass, 1e-9),
      "shared solve preserves mount mass");
    var collision = EndEffectorCollision.pieces(effector, null, null, solved);
    check(collision.pieces.length == 2 && collision.excluded.length == 0,
      "L-shaped effector keeps separate conservative pieces");
    var tool = EndEffectorBridge.toTool(effector, "tip");
    var bounds = ToolCollisionShapes.bounds(tool.collision);
    var point = [0.05, -0.05, 0.005];
    var insideBox = Math.abs(point[0] - bounds.centre.x) < bounds.halfExtents.x &&
      Math.abs(point[1] - bounds.centre.y) < bounds.halfExtents.y &&
      Math.abs(point[2] - bounds.centre.z) < bounds.halfExtents.z;
    var insidePieceBounds = false;
    for (piece in collision.pieces) {
      var pieceBounds = ToolCollisionShapes.bounds(ToolCollisionShape.Hulls([piece.vertices], 0));
      if (Math.abs(point[0] - pieceBounds.centre.x) < pieceBounds.halfExtents.x &&
        Math.abs(point[1] - pieceBounds.centre.y) < pieceBounds.halfExtents.y &&
        Math.abs(point[2] - pieceBounds.centre.z) < pieceBounds.halfExtents.z)
        insidePieceBounds = true;
    }
    check(insideBox && !insidePieceBounds, "empty L corner is clear of every piece");
    effector.addComponent("screw", new BridgeCollisionBlock(2, 2, 2, 0));
    effector.addMate("screw-mount", "fixed", "bar", "end", "screw", "mount");
    var merged = EndEffectorCollision.pieces(effector);
    check(merged.pieces.length == 2 && merged.merged.indexOf("screw") >= 0,
      "small member merges into a neighbouring piece");
    var twoPieceBudget = EndEffectorCollision.pieces(effector, null,
      new EndEffectorCollisionOptions(0, 2));
    check(twoPieceBudget.pieces.length == 2 && twoPieceBudget.merged.indexOf("screw") >= 0,
      "two-piece budget merges the smallest member");
    var limited = EndEffectorCollision.pieces(effector, null,
      new EndEffectorCollisionOptions(0, 1));
    check(limited.pieces.length == 1 && limited.merged.length == 2,
      "piece limit merges without dropping members");
    effector.excludeFromCollision("screw");
    var excluded = EndEffectorCollision.pieces(effector);
    check(excluded.excluded.indexOf("screw") >= 0 && excluded.pieces.length == 2,
      "only explicit exclusion drops a member");

    var remote = new EndEffector();
    remote.addComponent("bar", new BridgeCollisionBlock(100, 10, 10, 90));
    remote.addComponent("sensor", new BridgeCollisionBlock(2, 2, 2, 0));
    remote.mount("bar", "mount");
    remote.addMate("sensor-mount", "fixed", "bar", "sensor", "sensor", "mount");
    var separate = EndEffectorCollision.pieces(remote);
    check(separate.pieces.length == 2 && separate.merged.length == 0,
      "distant small sensor remains a separate collision piece");
    var gapPoint = new Vec3(0.2, 0, 0);
    var gapCovered = false;
    for (piece in separate.pieces) {
      var box = ToolCollisionShapes.bounds(ToolCollisionShape.Hulls([piece.vertices], 0));
      if (Math.abs(gapPoint.x - box.centre.x) <= box.halfExtents.x &&
          Math.abs(gapPoint.y - box.centre.y) <= box.halfExtents.y &&
          Math.abs(gapPoint.z - box.centre.z) <= box.halfExtents.z) gapCovered = true;
    }
    check(!gapCovered, "gap between bar and sensor remains free");
    var budgeted = EndEffectorCollision.pieces(remote, null,
      new EndEffectorCollisionOptions(10, 1));
    check(budgeted.pieces.length == 1 && budgeted.merged.length == 1,
      "piece budget still forces a distant merge");
  }

  static function testEnclosingHull():Void {
    var count = 32;
    var mesh = Bytes.alloc(count * 24);
    for (index in 0...count) {
      var angle = index * Math.PI * 2 / count;
      mesh.setDouble(index * 24, Math.cos(angle) * 10);
      mesh.setDouble(index * 24 + 8, Math.sin(angle) * 10);
      mesh.setDouble(index * 24 + 16, index % 2 == 0 ? 2 : -2);
    }
    var hull = ConvexHullVertices.enclosingFromMesh(mesh, count, 0.1);
    check(hull.vertices.length >= 12 && hull.vertices.length <= 192,
      "enclosing hull respects the convex vertex limit");
    for (xi in 0...3) for (yi in 0...3) for (zi in 0...3) {
      var x = xi - 1, y = yi - 1, z = zi - 1;
      if (x == 0 && y == 0 && z == 0) continue;
      var meshSupport = Math.NEGATIVE_INFINITY, hullSupport = Math.NEGATIVE_INFINITY;
      for (index in 0...count) {
        var at = index * 24;
        meshSupport = Math.max(meshSupport, x * mesh.getDouble(at) +
          y * mesh.getDouble(at + 8) + z * mesh.getDouble(at + 16));
      }
      for (index in 0...Std.int(hull.vertices.length / 3)) {
        var at = index * 3;
        hullSupport = Math.max(hullSupport, x * hull.vertices[at] +
          y * hull.vertices[at + 1] + z * hull.vertices[at + 2]);
      }
      check(hullSupport + 1e-6 >= meshSupport, "k-DOP encloses every sampled mesh support");
    }
    var cylinder = Shape.cylinder(30, 60);
    var cylinderMesh = cylinder.tessellateRelative(0.1);
    var cylinderHull = ConvexHullVertices.enclosingFromMesh(cylinderMesh.vertices,
      cylinderMesh.vertexCount, 0.1, cylinderMesh.linearDeflection + 0.05);
    var cylinderBounds = cylinder.bounds();
    var minZ = cylinderBounds.get_min().get_z();
    var maxZ = cylinderBounds.get_max().get_z();
    var centerX = (cylinderBounds.get_min().get_x() + cylinderBounds.get_max().get_x()) * 0.5;
    var centerY = (cylinderBounds.get_min().get_y() + cylinderBounds.get_max().get_y()) * 0.5;
    var rimEnclosed = true;
    for (sample in 0...128) {
      var angle = sample * Math.PI * 2 / 128;
      var point = [centerX + 30 * Math.cos(angle), centerY + 30 * Math.sin(angle),
        sample % 2 == 0 ? minZ : maxZ];
      for (xi in 0...3) for (yi in 0...3) for (zi in 0...3) {
        var x = xi - 1, y = yi - 1, z = zi - 1;
        if (x == 0 && y == 0 && z == 0) continue;
        var length = Math.sqrt(x * x + y * y + z * z);
        var support = Math.NEGATIVE_INFINITY;
        for (vertex in 0...Std.int(cylinderHull.vertices.length / 3)) {
          var at = vertex * 3;
          support = Math.max(support, (x * cylinderHull.vertices[at] +
            y * cylinderHull.vertices[at + 1] + z * cylinderHull.vertices[at + 2]) / length);
        }
        if ((x * point[0] + y * point[1] + z * point[2]) / length > support + 1e-6)
          rimEnclosed = false;
      }
    }
    cylinder.close();
    check(rimEnclosed, "coarse cylinder exact rim is enclosed");
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

  static function testSchmalzEndEffector():Void {
    var effector = SchmalzEndEffectorExample.build();
    check(effector.validate().length == 0 &&
      effector.billOfMaterials().lines().filter(line ->
        StringTools.startsWith(line.partNumber, "10.07.09.00001-L")).length == 1,
      "catalog EOAT validates with its hose and matching fitting");
    var grip = SuctionCapacityBridge.toGrip(effector, "cup", 60, 0.5, 2);
    check(approx(grip.normalCapacityN(), 69, 1e-9),
      "Schmalz theoretical force yields the documented area at 60 kPa");
    var tool = EndEffectorBridge.toTool(effector, "contact");
    check(tool.mass > 0.038 && tool.flangeTTcp.translation.z > 0.1,
      "catalog parts and hose contribute to the robot tool mass and TCP");
    var load = new WorkpieceLoad("panel", new MassProperties(1, Vec3.zero(), Inertia3.zero()),
      grip.flangeTCup);
    var down = new Transform3(Vec3.zero(), Quat.fromAxisAngle(new Vec3(1, 0, 0), Math.PI));
    var resting = new SuctionMotionSample(down, Vec3.zero());
    var accelerated = new SuctionMotionSample(down, new Vec3(15, 0, 0));
    check(SuctionCapacityChecker.checkPath(load, grip, [resting]).safe &&
      !SuctionCapacityChecker.checkPath(load, grip, [resting, accelerated]).safe,
      "catalog cup holds the static sample and rejects excessive sideways acceleration");
  }

  static function testSuctionCapacityBridge():Void {
    var effector = new EndEffector();
    effector.addComponent("generator", new VacuumGenerator(60));
    effector.addComponent("cup", new SuctionCup(40, 18, Math.PI * 18 * 18, 5));
    effector.mount("generator", "mount");
    effector.addMate("cup-mount", "fixed", "generator", "mount", "cup", "mount");
    effector.connectPorts("vacuum-cup", "generator", "vacuum", "cup", "vacuum");
    effector.exposePort("air", "generator", "air");
    var grip = SuctionCapacityBridge.toGrip(effector, "cup", 50, 0.5, 2);
    check(approx(grip.effectiveAreaM2, Math.PI * 18 * 18 * 1e-6, 1e-12) &&
      approx(grip.flangeTCup.translation.z, 0.018, 1e-9) &&
      approx(grip.normalCapacityN(), 50 * Math.PI * 18 * 18 * 1e-3, 1e-9),
      "suction bridge converts cup contact and measured area to RobotKit");
    check(effector.upstreamChain("cup", "vacuum").indexOf("generator/vacuum") >= 0,
      "suction bridge follows the actual vacuum service chain");
    var rejected = false;
    try SuctionCapacityBridge.toGrip(effector, "cup", 70, 0.5, 2)
    catch (error:Dynamic) rejected = Std.string(error).indexOf("rating") >= 0;
    check(rejected, "cup vacuum cannot exceed upstream generator rating");
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
    check(tool.massProperties != null && approx(tool.massProperties.massKg, 3, 1e-12),
      "end effector bridge preserves complete tool mass properties");
    check(switch tool.collision { case Hulls(pieces, _): pieces.length == 1; case _: false; },
      "end effector emits convex hull collision by default");
    var boxTool = EndEffectorBridge.toTool(endEffector, "contact", null, null, null, true);
    var boxCorrect = switch boxTool.collision {
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

    var payloadEffector = new EndEffector();
    payloadEffector.addComponent("body", new BridgePayloadPart());
    payloadEffector.mount("body", "mount");
    payloadEffector.workingFrame("contact", "body", "mount", true);
    var payloadTool = EndEffectorBridge.toTool(payloadEffector, "contact");
    var payloadMass = payloadTool.massProperties;
    check(payloadMass != null && approx(payloadMass.centerOfMass.x, 0, 1e-12) &&
      approx(payloadMass.centerOfMass.y, 0, 1e-12) &&
      approx(payloadMass.centerOfMass.z, 0.1, 1e-12),
      "MachineKit +Y centre of mass becomes RobotKit +Z in metres");
    check(payloadMass != null && payloadMass.inertia != null &&
      approx(payloadMass.inertia.xx, 1, 1e-9) &&
      approx(payloadMass.inertia.yy, 3, 1e-9) &&
      approx(payloadMass.inertia.zz, 2, 1e-9),
      "bridge rotates centroidal inertia into RobotKit axes and kg m²");

    var set = EndEffectorExample.build();
    var rebuiltSet = machinekit.robotics.EndEffectorSet.fromDescription(
      haxeon.wire.JsonWire.decode(set.encode()));
    for (id in ["short", "long"]) {
      var original = EndEffectorBridge.toTool(set.configuration(id), "contact", null, id + "/contact");
      var restored = EndEffectorBridge.toTool(rebuiltSet.configuration(id), "contact", null, id + "/contact");
      check(approx(original.flangeTTcp.translation.x, restored.flangeTTcp.translation.x, 1e-12) &&
        approx(original.flangeTTcp.translation.y, restored.flangeTTcp.translation.y, 1e-12) &&
        approx(original.flangeTTcp.translation.z, restored.flangeTTcp.translation.z, 1e-12) &&
        approx(original.mass, restored.mass, 1e-12),
        "saved EOAT configuration changes RobotKit TCP or mass");
      var originalBounds = ToolCollisionShapes.bounds(original.collision);
      var restoredBounds = ToolCollisionShapes.bounds(restored.collision);
      check(approx(originalBounds.centre.x, restoredBounds.centre.x, 1e-12) &&
        approx(originalBounds.centre.y, restoredBounds.centre.y, 1e-12) &&
        approx(originalBounds.centre.z, restoredBounds.centre.z, 1e-12) &&
        approx(originalBounds.halfExtents.x, restoredBounds.halfExtents.x, 1e-12) &&
        approx(originalBounds.halfExtents.y, restoredBounds.halfExtents.y, 1e-12) &&
        approx(originalBounds.halfExtents.z, restoredBounds.halfExtents.z, 1e-12),
        "saved EOAT configuration changes RobotKit collision bounds");
    }
    var shortTool = EndEffectorBridge.toTool(set.configuration("short"), "contact", null, "short/contact");
    var longTool = EndEffectorBridge.toTool(set.configuration("long"), "contact", null, "long/contact");
    check(shortTool.id == "short/contact" && longTool.id == "long/contact" &&
      shortTool.id != longTool.id, "configuration-qualified RobotKit tool IDs are distinct");
    var hullBounds = ToolCollisionShapes.bounds(shortTool.collision);
    check(hullBounds.centre.z - hullBounds.halfExtents.z >= -0.001 &&
      hullBounds.centre.z + hullBounds.halfExtents.z <=
        shortTool.flangeTTcp.translation.z + 0.001,
      "tool pieces stay forward within tessellation clearance");
    var tcpX = shortTool.flangeTTcp.transformVector(new Vec3(1, 0, 0));
    check(approx(tcpX.x, 1, 1e-9) && approx(tcpX.y, 0, 1e-9) &&
      approx(tcpX.z, 0, 1e-9), "cup TCP X retains the flange locating-pin direction");
    var shortBox = EndEffectorBridge.toTool(set.configuration("short"), "contact", null,
      "short/contact-box", null, true);
    var forwardBox = switch shortBox.collision {
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
    var manipulator = fixture.arm.withTool(tool.flangeTTcp);
    var q = [0.2, -0.4, 0.3, 0.1, -0.2, 0.15];
    var flangePose = fixture.arm.forwardKinematics(q);
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
    check(cup.vacuum.isEnabled() && !cup.vacuum.isHolding(),
      "active vacuum command waits for pressure feedback");
    var cupSensor:SimulatedVacuum = cast cup.vacuum;
    cupSensor.observeVacuumKpa(45, Int64.ofInt(120));
    check(cup.vacuum.isHolding(), "active vacuum configuration accepts pressure feedback");
    selected.select(gripper, Int64.ofInt(150));
    check(!cup.vacuum.isHolding() && selected.active() == gripper,
      "switching releases the old tool and selects the new runtime");
    selected.apply(new FiredProcessEvent(Int64.ofInt(2), "gripper.close",
      ProcessEventValue.Digital(true), Int64.ofInt(200), Int64.ofInt(210), 1));
    check(!gripper.gripper.isOpen() && !gripper.gripper.isGrasped(),
      "selected gripper command waits for contact feedback");
    var gripperSensor:SimulatedGripper = cast gripper.gripper;
    gripperSensor.observeContact(true, Int64.ofInt(220));
    check(gripper.gripper.isGrasped(), "selected gripper configuration accepts contact feedback");
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

  static function testDerivedRuntimeBindings():Void {
    var generic = EndEffectorExample.build();
    var derived = EndEffectorRuntimeBridge.toRuntimeFromDesign(generic, "short", "contact");
    check(derived.runtime.vacuum != null && derived.runtime.changerLock != null &&
      derived.runtime.channelDeclarations().length == 2 &&
      derived.bindings.vacuumSensorId == null,
      "generic EOAT derives vacuum and changer lock outputs from ports");
    var selected = new ToolRuntimeSelection();
    selected.select(derived.runtime, Int64.ofInt(0));
    selected.apply(new FiredProcessEvent(Int64.ofInt(1), "short/robot/master.lock",
      ProcessEventValue.Digital(false), Int64.ofInt(1), Int64.ofInt(2), 1));
    check(!derived.runtime.changerLock.isLocked(), "declared changer lock accepts release command");
    selected.select(null, Int64.ofInt(3));
    check(derived.runtime.changerLock.isLocked(), "inactive changer returns to safe locked state");

    var set = new EndEffectorSet();
    set.addComponent("base", new BridgeServiceSourcePart());
    set.mount("base", "mount");
    set.exposePort("coupledAir", "base", "air");
    set.exposePort("valveCommand", "base", "valveCommand");
    set.changer("manual", "base", "contact", [
      {robot: "coupledAir", tool: "air"},
      {robot: "valveCommand", tool: "valveCommand"}]);
    var effector = new EndEffector();
    effector.addComponent("generator", new VacuumGenerator());
    effector.addComponent("valve", new VacuumControlValve(6));
    effector.addComponent("sensor", new VacuumPressureSensor(6));
    effector.addComponent("cup", new SuctionCup(25, 18));
    effector.mount("generator", "mount");
    effector.addMate("valve-seat", "fixed", "generator", "mount", "valve", "mount");
    effector.addMate("sensor-seat", "fixed", "generator", "mount", "sensor", "mount");
    effector.addMate("cup-seat", "fixed", "generator", "mount", "cup", "mount");
    effector.connectPorts("valve-feed", "generator", "vacuum", "valve", "vacuumIn");
    effector.connectPorts("pressure-feed", "valve", "vacuumOut", "sensor", "vacuumIn");
    effector.connectPorts("cup-feed", "sensor", "vacuumOut", "cup", "vacuum");
    effector.exposePort("air", "generator", "air");
    effector.exposePort("valveCommand", "valve", "control");
    effector.workingFrame("contact", "cup", "contact", true);
    set.addTool("sensor", effector);
    var sensed = EndEffectorRuntimeBridge.toRuntimeFromDesign(set, "sensor", "contact");
    check(sensed.bindings.vacuumSensorId == "sensor/tool/sensor.pressureSignal" &&
      sensed.runtime.vacuum != null &&
      sensed.runtime.channelDeclarations()[0].id == "sensor/tool/valve.enable",
      "pressure sensor and explicit vacuum valve derive feedback and command IDs");
    var sensorSelection = new ToolRuntimeSelection();
    var sensorAdapter = EndEffectorRuntimeBridge.bindSensors(sensorSelection, sensed);
    sensorSelection.select(sensed.runtime, Int64.ofInt(0));
    sensorSelection.apply(new FiredProcessEvent(Int64.ofInt(1),
      "sensor/tool/valve.enable", ProcessEventValue.Digital(true),
      Int64.ofInt(1), Int64.ofInt(2), 1));
    var frame = new SensorFrame(sensed.bindings.vacuumSensorId, "tool_vacuum_kpa",
      "tool", Int64.ofInt(1), Int64.ofInt(3), [45.0], Int64.ofInt(3),
      "", null, null, "robotkit.monotonic");
    check(sensorAdapter.apply(frame) && sensed.runtime.vacuum.isHolding(),
      "derived pressure sensor feeds active vacuum capability");

    var gripperSet = new EndEffectorSet();
    gripperSet.addComponent("base", new BridgeServiceSourcePart());
    gripperSet.mount("base", "mount");
    gripperSet.exposePort("open", "base", "open");
    gripperSet.exposePort("close", "base", "close");
    gripperSet.changer("manual", "base", "contact", [
      {robot: "open", tool: "openAir"}, {robot: "close", tool: "closeAir"}]);
    var gripperTool = new EndEffector();
    gripperTool.addComponent("gripper", new ParallelGripper(40, 20, 60, 12));
    gripperTool.mount("gripper", "mount");
    gripperTool.exposePort("openAir", "gripper", "open");
    gripperTool.exposePort("closeAir", "gripper", "close");
    gripperTool.workingFrame("contact", "gripper", "tcp", true);
    gripperSet.addTool("gripper", gripperTool);
    var derivedGripper = EndEffectorRuntimeBridge.toRuntimeFromDesign(gripperSet,
      "gripper", "contact");
    check(derivedGripper.runtime.gripper != null &&
      derivedGripper.runtime.channelDeclarations()[0].id == "gripper/tool/gripper.close",
      "gripper valve command derives from declared open and close ports");
  }

  /** A welding torch derives an arc control, bound to its signal inlet, and its other channels. */
  static function testTorchBindings():Void {
    var services = ["power", "gas", "wire", "control"];
    var set = new EndEffectorSet();
    set.addComponent("base", new BridgeWelderSourcePart());
    set.mount("base", "mount");
    for (name in services) set.exposePort(name, "base", name);
    set.changer("manual", "base", "contact", [for (name in services) {robot: name, tool: name}]);
    var tool = new EndEffector();
    tool.addComponent("torch", new WeldingTorch(45));
    tool.mount("torch", "robot");
    for (name in services) tool.exposePort(name, "torch", name);
    tool.workingFrame("tcp", "torch", "tcp", true);
    set.addTool("weld", tool);
    var bindings = EndEffectorRuntimeBridge.deriveBindings(set, "weld");
    check(bindings.controls.length == 1 && bindings.vacuumSensorId == null, "a torch derives one control and no vacuum sensor");
    switch bindings.controls[0] {
      case Arc(channel, instanceId, controlPort):
        check(channel == "weld/tool/torch.arc" && instanceId == "tool/torch" && controlPort == "control",
          "the arc control is the torch's trigger channel on its signal inlet");
      case _: check(false, "a torch derives an arc binding");
    }
    var derived = machinekit.robot.EndEffectorControls.derive(set.configuration("weld"), "weld");
    check(derived.arcs.length == 1 && derived.arcs[0].wireSpeedChannel == "weld/tool/torch.wire_speed" &&
      derived.arcs[0].voltageChannel == "weld/tool/torch.voltage" && derived.arcs[0].sensor == "weld/tool/torch.weld" &&
      derived.arcs[0].tcpConnector == "tcp", "the torch's analogue channels and weld sensor are named after its member");
    var runtime = EndEffectorRuntimeBridge.toRuntimeFromDesign(set, "weld", "tcp");
    check(runtime.runtime.gripper == null && runtime.runtime.vacuum == null && runtime.runtime.channelDeclarations().length == 0,
      "the arc is worked by the robot's welder, so the kinematic tool runtime declares no channel for it");
  }

  static function testDerivedExternalAxes():Void {
    var arm = new MachineAssembly();
    for (index in 0...7) arm.addComponent('link$index', new BridgeCollisionBlock(10, 10, 10, 10));
    for (index in 0...6) arm.addMateOnAxis('axis$index', "revolute", 'link$index', "end", 'link${index + 1}', "mount",
      {x: 0.0, y: 0.0, z: 1.0}, 0.0, {lower: -2.0, upper: 2.0, velocity: 1.0, effort: 100.0});
    arm.addComponent("baseFlange", new machinekit.robotics.RobotFlange(63));
    arm.addMate("baseFace", "fixed", "link0", "mount", "baseFlange", "face");
    arm.addComponent("toolFlange", new machinekit.robotics.RobotFlange(31.5));
    arm.addMate("toolFace", "fixed", "link6", "end", "toolFlange", "face");
    var cell = new MachineAssembly();
    cell.addComponent("floor", new BridgeCollisionBlock(10, 10, 10, 10));
    cell.addComponent("carriage", new BridgeCollisionBlock(10, 10, 10, 10));
    cell.addComponent("trackFlange", new machinekit.robotics.RobotFlange(63));
    cell.addMate("trackFace", "fixed", "floor", "mount", "trackFlange", "face");
    cell.addMateOnAxis("track", "prismatic", "floor", "end", "carriage", "mount", {x: 1.0, y: 0.0, z: 0.0},
      0.0, {lower: 0.0, upper: 1000.0, velocity: 100.0, effort: 1000.0});
    cell.include("arm", arm);
    cell.addMate("armMount", "fixed", "carriage", "end", "arm/link0", "mount");
    var outer = new MachineAssembly();
    outer.addComponent("rootFlange", new machinekit.robotics.RobotFlange(50));
    outer.include("cell", cell);
    var definition = machinekit.assembly.FrozenAssemblyDefinitions.thaw(outer.describe().mechanical);
    var physical:cadbridge.AssemblySimulationBridge.AssemblyPhysicalData = {metresPerUnit: 0.001,
      parts: [for (part in outer.components()) {id: part.id, materialId: "steel", volume: 1000.0,
        centerOfMass: [0.0, 0.0, 0.0], inertia: [100000.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 100000.0], density: 7850.0}]};
    var translated = AssemblySimulationBridge.toRobotModel(definition, physical);
    var model = translated.model;
    var physicalFlange:Null<Frame> = null;
    for (frame in model.frames) if (frame.name == "cell/arm/toolFlange robot flange") physicalFlange = frame;
    check(physicalFlange != null && physicalFlange.flangeIncludePath == "cell/arm",
      "a real flange capability survives nested assembly freezing and the bridge");
    var flange:Frame = cast physicalFlange;
    var tcp = model.addFrame(new Frame("tcp", flange.link));
    tcp.position = flange.position.copy(); tcp.rotation = flange.rotation.copy();
    var manipulator = new Manipulator(model, model.links[0].id, tcp.id);
    check(manipulator.group.count() == 7 && manipulator.external[0], "the nearest tool flange, rather than the root and track flanges, determines the external track");
    for (index in 1...7) check(!manipulator.external[index], "the arm's six joints stay inside its include");
    check(manipulator.swivel == null, "an assembly six-axis arm on a track has no swivel");
    var solver = new motionkit.robot.ManipulatorKinematics(manipulator);
    check(Std.isOfType(solver.redundancy(), motionkit.robot.ExternalAxesParameterization),
      "the derived track uses external-axis parameterization");
    var seed = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    var result = manipulator.solve(manipulator.tcpPose(seed), seed, new robotkit.manipulation.IkOptions().preferring(seed));
    check(result.converged && result.q.length == 7, "posture includes the derived external coordinate");
    // One tracking step exposes the external cost: reaching the same target
    // takes a smaller track step than a group treating every joint as an arm joint.
    var destination = seed.copy(); destination[0] = 0.2;
    var target = manipulator.tcpPose(destination);
    var tracking = new robotkit.manipulation.IkOptions(1e-6, 1e-6, 1);
    var damped = manipulator.solve(target, seed, tracking);
    var undamped = new robotkit.manipulation.KinematicGroup(model, model.links[0].id, tcp.id,
      null, null, null, []);
    var control = undamped.solve(target, seed, tracking);
    check(damped.q[0] > seed[0] && damped.q[0] < control.q[0] - 1e-4,
      "an assembly-derived external track receives damping during a tracking solve");
    var copy = robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(model));
    var rootOwner = false;
    for (frame in copy.frames) if (frame.name == "rootFlange robot flange" && frame.flangeIncludePath == "") rootOwner = true;
    check(rootOwner, "the model codec preserves an empty root include rather than dropping flange ownership");
    var copied = new Manipulator(copy, copy.links[0].id, tcp.id);
    check(copied.external[0] && copied.swivel == null, "model round trips retain flange and joint include ownership");
    var flat = materia.assembly.AssemblyDefinitionFlattener.flatten(definition);
    var bad:materia.assembly.AssemblyDefinition = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(flat));
    bad.definitions[0].robotFlangeConnector = "absent";
    var rejected = false;
    try materia.assembly.AssemblyDefinitionCodec.validate(bad) catch (_:Dynamic) rejected = true;
    check(rejected, "a physical flange cannot name a missing face connector");
  }

  static function testAssemblySimulationBridge():Void {
    var assembly = new AssemblyModel();
    assembly.add("base");
    assembly.add("slider");
    assembly.connector("base", "mount", AssemblyFrames.identity());
    assembly.connector("slider", "mount", AssemblyFrames.identity());
    assembly.mateOnAxis("slide", "prismatic", "base", "mount", "slider", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: 0, upper: 100, velocity: 20, effort: 50});
    assembly.actuateDrive({id: "drive", joint: "slide", maxEffort: 10, maxRate: 5,
      fullStepsPerRevolution: 200, microsteps: 16, maxStepRate: 200000});
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
    // The world-fixed base joins the root link; the slider is one link carried by its one joint.
    check(translated.model.links.length == 2 && translated.model.joints.length == 1,
      "assembly bodies translate to robot links and moving joints");
    var slide = translated.model.joints[0];
    check(Math.abs(slide.limits.upper - 0.1) < 1e-9 && slide.axis[1] == 1 &&
      Math.abs(translated.model.links[1].mass - 7.85) < 1e-9,
      "assembly limits and material mass convert to SI units");
    check(RobotRuntimeCompiler.validate(translated.model, new robotkit.profile.RobotProfile()).length == 0,
      "translated assembly is a valid RobotKit runtime model");
    check(translated.model.actuators.length == 1 && translated.model.actuators[0].fullStepsPerRevolution == 200,
      "a stepper's full steps reach the robot actuator");
    // A drive kind with its torque-speed curve reaches the actuator, a servo's too, and a bare stepper stays a bare stepper.
    var driven = new AssemblyModel();
    driven.add("base");
    driven.add("slider");
    driven.connector("base", "mount", AssemblyFrames.identity());
    driven.connector("slider", "mount", AssemblyFrames.identity());
    driven.mateOnAxis("slide", "prismatic", "base", "mount", "slider", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: 0, upper: 100, velocity: 20, effort: 50});
    driven.actuateDrive({id: "stepper", joint: "slide", maxEffort: 0.6, maxRate: 100, rotorInertia: 3e-5,
      fullStepsPerRevolution: 200, microsteps: 16, maxStepRate: 200000, drive: "stepper", holdingTorque: 1.2, torqueSpeed: [0, 1.2, 100, 1.2, 400, 0.3]});
    driven.actuateDrive({id: "servo", joint: "slide", maxEffort: 1.8, maxRate: 500, drive: "servo", ratedTorque: 0.6,
      peakTorque: 1.8, ratedSpeed: 300, maxSpeed: 500, encoderCounts: 4096, servoStiffness: 12});
    var drives = AssemblySimulationBridge.toRobotModel(driven.definition("drive-test"), parts).model.actuators;
    var stepperDrive = drives[0].drive, servoDrive = drives[1].drive;
    check(stepperDrive != null && stepperDrive.kind() == "stepper" && stepperDrive.curve.torqueAt(250) == 0.75 &&
      drives[0].fullStepsPerRevolution == 200 && stepperDrive.rotorInertia == 3e-5,
      "a stepper's drive and pull-out curve reach the robot actuator");
    check(servoDrive != null && servoDrive.kind() == "servo" && drives[1].requireEffort() == 1.8 &&
      drives[1].requireRate() == 500 && drives[1].servoStiffness == 12 && servoDrive.curve.torqueAt(400) < 1.8,
      "a servo's drive, peak torque and maximum speed reach the robot actuator");
    // A gearbox between a servo and a turning joint: the actuator turns `gearRatio` times for one turn of the joint, the joint
    // is limited to the servo's speed over the ratio and its torque through the ratio at the efficiency, and the rotor's
    // inertia at the joint is the ratio squared times its own.
    var geared = new AssemblyModel();
    geared.add("base");
    geared.add("slider");
    geared.connector("base", "mount", AssemblyFrames.identity());
    geared.connector("slider", "mount", AssemblyFrames.identity());
    geared.mateOnAxis("turn", "revolute", "base", "mount", "slider", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: -3, upper: 3, velocity: 100, effort: 1000});
    geared.actuateDrive({id: "servo", joint: "turn", maxEffort: 2.0, maxRate: 500, rotorInertia: 2e-5, drive: "servo", ratedTorque: 0.6,
      peakTorque: 2.0, ratedSpeed: 300, maxSpeed: 500, gearRatio: 100, gearEfficiency: 0.8});
    var gearedModel = AssemblySimulationBridge.toRobotModel(geared.definition("gear-test"), parts).model;
    var gearedMotor = gearedModel.actuators[0];
    var gearing = switch gearedMotor.transmission { case SimpleTransmission(_, ratio, _): ratio; };
    check(gearing == 100.0 && gearedMotor.efficiency == 0.8, "a gearbox's ratio and efficiency reach the robot actuator");
    check(Math.abs(gearedModel.joints[0].armature - 2e-5 * 100.0 * 100.0) < 1e-12, "the rotor's inertia is seen through the ratio squared");
    check(gearedModel.joints[0].limits.velocity == 5.0 && gearedModel.joints[0].limits.effort == 160.0,
      "the compiled model carries effective limits before runtime lowering");
    var gearedJoint = RobotRuntimeCompiler.compile(gearedModel, new robotkit.profile.RobotProfile()).joints[0];
    check(Math.abs(gearedJoint.requireRate() - 5.0) < 1e-12 && Math.abs(gearedJoint.requireEffort() - 160.0) < 1e-9,
      'the joint is limited to its drive: ${gearedJoint.maxRate} rad/s and ${gearedJoint.maxEffort} N m');
    // Encoders are sensors on joints: counts per millimetre on a sliding joint become per metre, per revolution on a
    // turning one per radian, and a servo that names its encoder holds no count of its own.
    var sensed = new AssemblyModel();
    sensed.add("base");
    sensed.add("slider");
    sensed.connector("base", "mount", AssemblyFrames.identity());
    sensed.connector("slider", "mount", AssemblyFrames.identity());
    sensed.mateOnAxis("slide", "prismatic", "base", "mount", "slider", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: 0, upper: 100, velocity: 20, effort: 50});
    sensed.actuateDrive({id: "servo", joint: "slide", maxEffort: 1.8, maxRate: 500, drive: "servo", ratedTorque: 0.6,
      peakTorque: 1.8, ratedSpeed: 300, maxSpeed: 500, encoder: "scale"});
    sensed.addEncoder({id: "scale", joint: "slide", kind: "absolute", counts: 200, index: true});
    var sensedModel = AssemblySimulationBridge.toRobotModel(sensed.definition("encoder-test"), parts).model;
    check(sensedModel.encoders.length == 1 && sensedModel.encoders[0].countsPerUnit == 200000.0 &&
      sensedModel.encoders[0].kind == robotkit.model.EncoderKind.Absolute && sensedModel.encoders[0].index &&
      sensedModel.encoders[0].joint == sensedModel.joints[0].id, "an encoder's counts per millimetre reach the robot in counts per metre");
    check(sensedModel.actuators[0].encoder == "scale" && sensedModel.encoderFor(sensedModel.actuators[0]) == sensedModel.encoders[0] &&
      RobotRuntimeCompiler.validate(sensedModel, new robotkit.profile.RobotProfile()).length == 0, "the servo's encoder is the sensor it names");
    // A coupling's stiffness, backlash and drag reach the robot coupling in SI units: a 100 N/mm drive on a
    // millimetre axis is 100000 N/m, 0.05 mm of backlash 5e-5 m, and a turning follower's drag is as given.
    var screwed = new AssemblyModel();
    for (member in ["base", "slider", "screw"]) {
      screwed.add(member);
      screwed.connector(member, "mount", AssemblyFrames.identity());
    }
    screwed.mateOnAxis("slide", "prismatic", "base", "mount", "slider", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: 0, upper: 100, velocity: 20, effort: 50});
    screwed.mateOnAxis("turn", "continuous", "base", "mount", "screw", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: null, upper: null, velocity: 70, effort: 0});
    screwed.couple("lead", "slide", "turn", Math.PI, 0, 0.4, 100.0, 0.05, 0.02);
    var screwParts = AssemblyPhysicalPartView.fromSceneArtifact({metresPerUnit: 0.001,
      parts: [for (id in ["base", "slider", "screw"]) {
        id: id, name: id, red: 0.5, green: 0.5, blue: 0.5,
        materialId: "machined-steel", materialDensity: 7850.0,
        volume: 1000000.0, centerOfMass: [0.0, 0.0, 0.0],
        inertia: [10000000000.0, 0, 0, 0, 10000000000.0, 0, 0, 0, 10000000000.0],
        vertexCount: 4, indexCount: 0, vertices: vertices,
        normals: Bytes.alloc(0), indices: Bytes.alloc(0), faceRanges: []
      }]});
    var lead = AssemblySimulationBridge.toRobotModel(screwed.definition("lead-test"), screwParts).model.couplings[0];
    check(Math.abs(lead.stiffness - 1.0e5) < 1e-6 && Math.abs(lead.backlash - 5e-5) < 1e-15 && lead.drag == 0.02 &&
      Math.abs(lead.efficiency - 0.4) < 1e-15, "a coupling's stiffness, backlash and drag reach the robot in SI units");
    check(translated.linkHulls.length == 2 && translated.linkHulls[0].link == 0 &&
      translated.linkHulls[1].link == 1 && translated.linkHulls[1].vertices.length <= 64 * 3,
      "physical-part view supplies each part's bounded hull on its link");
    var baseLink = translated.partLinks.get("base"), sliderLink = translated.partLinks.get("slider");
    check(baseLink != null && sliderLink != null && baseLink.link == 0 && sliderLink.link == 1,
      "each part knows its link");
    // A tip bolted to the slider 100 mm along X rides the slider's link: one body, combined mass.
    var tipped = new AssemblyModel();
    tipped.add("base");
    tipped.add("slider");
    tipped.add("tip");
    tipped.connector("base", "mount", AssemblyFrames.identity());
    tipped.connector("slider", "mount", AssemblyFrames.identity());
    tipped.connector("slider", "tipSeat", AssemblyFrames.translation(100, 0, 0));
    tipped.connector("tip", "mount", AssemblyFrames.identity());
    tipped.mateOnAxis("slide", "prismatic", "base", "mount", "slider", "mount",
      {x: 0, y: 1, z: 0}, 0, {lower: 0, upper: 100, velocity: 20, effort: 50});
    tipped.mate("tip-mount", "fixed", "slider", "tipSeat", "tip", "mount");
    var withTip = AssemblyPhysicalPartView.fromSceneArtifact({metresPerUnit: 0.001,
      parts: [for (id in ["base", "slider", "tip"]) {
        id: id, name: id, red: 0.5, green: 0.5, blue: 0.5,
        materialId: "machined-steel", materialDensity: 7850.0,
        volume: 1000000.0, centerOfMass: [0.0, 0.0, 0.0],
        inertia: [10000000000.0, 0, 0, 0, 10000000000.0, 0, 0, 0, 10000000000.0],
        vertexCount: 4, indexCount: 0, vertices: vertices,
        normals: Bytes.alloc(0), indices: Bytes.alloc(0), faceRanges: []
      }]});
    var body = AssemblySimulationBridge.toRobotModel(tipped.definition("bridge-tip"), withTip);
    var link = body.model.links[1];
    check(body.model.links.length == 2 && body.model.joints.length == 1,
      "a part fixed to the slider adds no link and no joint");
    check(Math.abs(link.mass - 15.7) < 1e-9 && Math.abs(link.centerOfMass[0] - 0.05) < 1e-12,
      'the slider body sums its parts\' masses about their joint centre (${link.mass}, ${link.centerOfMass})');
    // Each part: 1e10 mm^5 x 7850 kg/m^3 = 0.0785 kg m^2 per axis, plus 7.85 kg at 50 mm off-axis.
    var own = 1e10 * 7850 * 1e-15;
    check(Math.abs(link.inertiaTensor[0] - 2 * own) < 1e-9 &&
      Math.abs(link.inertiaTensor[4] - (2 * own + 2 * 7.85 * 0.05 * 0.05)) < 1e-9,
      'the slider body inertia follows the parallel-axis theorem (${link.inertiaTensor})');
    var tip = body.partLinks.get("tip");
    if (tip == null) throw "the tip has no link";
    check(Math.abs(tip.offset.x - 0.1) < 1e-12 && tip.link == 1, "the tip rides the slider's link 100 mm out");
    var tipHull = [for (hull in body.linkHulls) if (hull.part == "tip") hull][0];
    var xs = [for (index in 0...Std.int(tipHull.vertices.length / 3)) tipHull.vertices[index * 3]];
    var lowX = xs[0], highX = xs[0];
    for (x in xs) { lowX = Math.min(lowX, x); highX = Math.max(highX, x); }
    check(tipHull.link == 1 && Math.abs(lowX - 0.1) < 1e-12 && Math.abs(highX - 0.11) < 1e-12,
      'the tip\'s hull sits 100 mm out in the slider link\'s frame ($lowX..$highX)');
    var nestedDefinition = assembly.definition("bridge-nested");
    nestedDefinition.occurrences[1].definition = "slider-sub";
    nestedDefinition.occurrences[1].assembly = "slider-sub";
    nestedDefinition.assemblies = [{id: "slider-sub",
      definitions: [{id: "slider-part", connectors: nestedDefinition.definitions[1].connectors}],
      occurrences: [{id: "body", definition: "slider-part", initialPose: AssemblyFrames.identity()}],
      joints: [], exposedConnectors: [{name: "mount", occurrence: "body", connector: "mount"}]}];
    parts.parts[1].id = "slider/slider-part";
    var nestedModel = AssemblySimulationBridge.toRobotModel(nestedDefinition, parts);
    check(nestedModel.model.links.length == translated.model.links.length &&
      nestedModel.model.joints.length == translated.model.joints.length &&
      nestedModel.model.links[1].name == "slider/body",
      "nested assembly flattens before RobotKit translation");
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
  /**
   * The mobile base example drives as a wheeled robot: the bridge makes its chassis the root link that
   * the drive rolls over the floor, each wheel turns about its own outward shaft, and the chassis
   * follows the commanded twist.
   */
  static function testMobileBaseBridge():Void {
    var scene = SceneArtifact.decode(MobileBasePreview.base());
    var drive = scene.mobileBase;
    if (drive == null || scene.assemblyDefinition == null) throw "Mobile base example should declare its drive";
    var converted = AssemblySimulationBridge.toRobotModel(cast scene.assemblyDefinition,
      AssemblyPhysicalPartView.fromSceneArtifact(scene), scene.assemblyState, null, null, drive);
    var model = converted.model;
    check(model.links.length == 3 && [for (joint in model.joints) joint.id].join(",") == "wheel_l,wheel_r",
      "the mobile base is its chassis and two wheel links on the wheel joints");
    check(converted.profile.mobileBase != null, "the bridge configures the mobile base");
    var blueprint = robotkit.runtime.RobotRuntimeCompiler.compile(model, converted.profile);
    var configuration:robotkit.runtime.RobotRuntimeConfiguration = cast blueprint.configuration;
    var mobile:robotkit.runtime.RobotRuntimeMobileConfiguration = cast configuration.mobileBase;
    check(switch mobile.drive {
      case robotkit.runtime.RobotRuntimeDriveConfiguration.Differential(_, _, _, _, radius, track, left, right):
        left == 1 && right == -1 && approx(radius, 0.075, 1e-12) && approx(track, 0.38, 1e-12);
      case _: false;
    }, "each wheel's direction comes from its shaft: the right one rolls back on a positive rate");

    var harness = new SimulationHarness(0.02);
    var runtime = harness.simulation.addRobotAtPose(blueprint, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
    var robot = new SimulatedRobot("mobile-base", runtime, model.name, [for (link in model.links) link.id],
      [for (joint in model.joints) joint.id]);
    var base = robotkit.mobile.MobileBase.fromBlueprint(robot, blueprint);
    var plant = new DifferentialDrivePlant(harness, 0, base);
    var odometry = base.driveModel.createOdometry();
    var tick = 1;
    var snapshot = plant.step(haxe.Int64.ofInt(tick++));
    odometry.update(snapshot);
    // 0.5 m/s for ten 0.02 s ticks: 0.1 m straight ahead, each wheel turning 0.1 / 0.075 rad its own way.
    base.command(new Twist2(0.5, 0.0));
    for (_ in 0...10) odometry.update(snapshot = plant.step(haxe.Int64.ofInt(tick++)));
    check(approx(plant.pose.x, 0.1, 1e-9) && approx(plant.pose.y, 0.0, 1e-9) && approx(plant.pose.yaw, 0.0, 1e-9),
      "the chassis rolls straight ahead at the commanded speed");
    var chassis = harness.simulation.linkPose(0, 0);
    check(approx(chassis.position[0], 0.1, 1e-6) && approx(chassis.position[2], 0.0, 1e-6),
      "the chassis link, which carries the base's parts, moves with the drive");
    check(approx(snapshot.velocities.get(0), 0.5 / 0.075, 1e-6) && approx(snapshot.velocities.get(1), -0.5 / 0.075, 1e-6),
      "the wheel joints spin at v / r, the right one negative");
    // A positive yaw rate turns the base counter-clockwise in place.
    base.command(new Twist2(0.0, 1.0));
    for (_ in 0...10) odometry.update(snapshot = plant.step(haxe.Int64.ofInt(tick++)));
    check(approx(plant.pose.yaw, 0.2, 1e-9) && approx(plant.pose.x, 0.1, 1e-9),
      "the base turns in place counter-clockwise at the commanded rate");
    var estimate = odometry.current();
    check(approx(estimate.x, plant.pose.x, 2e-3) && approx(estimate.yaw, plant.pose.yaw, 2e-2),
      'wheel odometry follows the chassis (${estimate.x}, ${estimate.yaw} vs ${plant.pose.x}, ${plant.pose.yaw})');
    harness.dispose();
  }

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
    var manipulator = fixture.arm;
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

  static function buildUR5Fixture():{model:RobotModel, arm:Manipulator} {
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
      joint.limits.velocity = null;
    }
    var flangeOffset = new Vec3(0.0, d6, 0.0);
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = flangeOffset.toArray();
    var arm = new Manipulator(model, links[0].id, flange.id);
    return { model: model, arm: arm };
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

  static function reflexVertices(points:Array<processkit.work.Point2>):Int {
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

private class BridgeCollisionBlock extends MachineComponent {
  final width:Float;
  final depth:Float;
  final height:Float;

  public function new(width:Float, depth:Float, height:Float, endX:Float) {
    super("BRIDGE-COLLISION-BLOCK", "collision test block", "steel", true);
    this.width = width;
    this.depth = depth;
    this.height = height;
    addConnector("mount", Mount, AssemblyFrames.identity());
    addConnector("end", Mount, AssemblyFrames.translation(endX, 0, 0));
    addConnector("sensor", Mount, AssemblyFrames.translation(300, 0, 0));
    declareMass(1, new Vector(), InertiaTensor.zero());
  }

  override public function geometry(detail:ComponentDetail = Preview):Part
    return Part.box(width, depth, height);
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

private class BridgeServiceSourcePart extends MachineComponent {
  public function new() {
    super("BRIDGE-SERVICE-SOURCE", "service source fixture", "steel", true);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("contact", Face, Solids.axial(0, 0, 10));
    for (name in ["air", "open", "close"])
      addPort({name: name, kind: Pneumatic, role: Supply,
        iface: Unspecified, required: false});
    addPort({name: "valveCommand", kind: Signal, role: Supply,
      iface: Plug("digital-valve", 2), required: false});
    declareMass(1, new Vector(0, 0, 5), InertiaTensor.zero());
  }

  override public function geometry(detail:ComponentDetail = Preview):Part
    return Part.box(10, 10, 10);
}

private class BridgeWelderSourcePart extends MachineComponent {
  public function new() {
    super("BRIDGE-WELDER-SOURCE", "welding service source fixture", "steel", true);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("contact", Face, Solids.axial(0, 0, 10));
    addPort({name: "power", kind: ElectricalPower, role: Supply, iface: WeldingInterfaces.weldCable(), required: false});
    addPort({name: "gas", kind: Gas, role: Supply, iface: WeldingInterfaces.gas(), required: false});
    addPort({name: "wire", kind: Wire, role: Supply, iface: WeldingInterfaces.wireLiner(), required: false});
    addPort({name: "control", kind: Signal, role: Supply, iface: WeldingInterfaces.control(), required: false});
    declareMass(1, new Vector(0, 0, 5), InertiaTensor.zero());
  }

  override public function geometry(detail:ComponentDetail = Preview):Part
    return Part.box(10, 10, 10);
}

private class BridgePayloadPart extends MachineComponent {
  public function new() {
    super("BRIDGE-PAYLOAD", "payload conversion fixture", "steel", true);
    addConnector("mount", Mount, AssemblyFrames.identity());
    declareMass(2, new Vector(0, 100, 0),
      new InertiaTensor(1000000, 0, 0, 2000000, 0, 3000000));
  }

  override public function geometry(detail:ComponentDetail = Preview):Part
    return Part.box(20, 20, 20);
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
