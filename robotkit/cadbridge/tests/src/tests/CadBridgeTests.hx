package tests;

import CadKit;
import cadkit.Shape;
import cadkit.Geometry;
import cadkit.parametric.ElementReference;
import bimkit.BimDocument;
import robotkit.spatial.Vec3;
import robotkit.spatial.FrameTree3;
import cadbridge.FaceBridge;
import cadbridge.WallBridge;
import cadbridge.BimFrameBridge;

/** M6 acceptance tests for robotkit/cadbridge: CadKit Face and BimKit wall -> WorkSurface, and BIM hierarchy -> FrameTree3. */
class CadBridgeTests {
  static var assertions = 0;

  public static function main():Void {
    testFaceBridgeOnPlainBoxFace();
    testWallBridgeAreaNormalAndExclusion();
    testBimFrameHierarchy();
    Sys.println('CadBridge tests passed ($assertions assertions)');
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
