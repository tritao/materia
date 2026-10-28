import cadkit.modeling.Part;
import cadkit.modeling.Scope;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ReleasePlan;
import materia.tooling.ReleaseSystem;
import machinekit.tooling.ToolingReleaseExport;
import machinekit.tooling.ToolingReleaseExport.ReleaseGeometryPart;

class ToolingReleaseTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function box(x:Float,
    y:Float, z:Float, width:Float, depth:Float, height:Float):Part return machinekit.tooling.ToolingBlock.placedBox({
      x: x,
      y: y,
      z: z,
      width: width,
      depth: depth,
      height: height
    });
  static function near(a:Float, b:Float, message:String):Void check(
    Math.abs(a - b) < 1e-6,
    message + ": " + a + " vs " + b
  );
  static function main():Void {
    var scope = new Scope();
    try {
      var blank = scope.own(box(0, 0, 0, 10, 10, 10));
      var insert = scope.own(box(4, 4, -1, 2, 2, 5));
      var cylinder = scope.own(Part.cylinder(0.5, 4));
      var anchor = scope.own(cylinder.translated(new Vector(5, 5, 4)));
      var eps = scope.own(box(1, 1, 4, 2, 2, 2));
      var resin = scope.own(blank.subtractAll([insert, anchor, eps]));
      var bedBlank = scope.own(box(0, 0, -2, 10, 10, 2)), bed = scope.own(bedBlank.subtractAll([insert]));
      var fixture = scope.own(box(0, 0, 10, 2, 2, 2));
      var stock = materia.tooling.panel.PanelJoints.stock(2);
      var parts:Array<ReleaseGeometryPart> = [
        {
          id: "bed",
          sourceIds: ["casting"],
          part: bed,
          substrate: Board,
          stock: {stock: stock, pose: F.translation(0, 0, -2)}
        },
        {id: "datum", sourceIds: ["source-insert"], part: fixture, substrate: ToolingMetal, stock: null},
        {id: "core", sourceIds: ["source-pocket"], part: eps, substrate: EPSCovering, stock: null},
        {id: "insert", sourceIds: ["source-insert"], part: insert, substrate: BondingSteel, stock: null},
        {
          id: "anchor",
          sourceIds: ["source-insert", "source-anchor"],
          part: anchor,
          substrate: BondingSteel,
          stock: null
        }
      ];
      var system = ReleaseSystems.unresolved(2, 5, 1, EPSCoveringKind.Tape);
      system.epsSafety = EPSReleaseSafety.DeclaredSafe;
      var initialVolume = resin.volume();
      var plan = ToolingReleaseExport.fromGeometry("casting", system, 0.1, resin, parts);
      var checks = ReleasePlans.validate(plan);
      for (rule in ["coverage", "bonding", "assembly", "eps-safety"]) check(
        checks.filter(row -> row.rule == "fabrication.release." + rule && row.status == "pass").length == 1,
        "Native release geometry passes " + rule
      );
      var boardWet = plan.faces.filter(face -> face.partId == "bed" && face.wettedAreaMm2 > 0);
      check(boardWet.length == 1, "A pocketed board has one exact wetted face, not its full bounding face");
      near(boardWet[0].wettedAreaMm2, 96, "Steel contact is subtracted from the board's wetted area");
      check(
        !boardWet[0].needsSealing,
        "A preserved melamine stock face is released without being called a machined edge"
      );
      check(
        plan.faces.filter(face -> face.partId == "bed" && face.needsSealing).length > 0,
        "The same board's actual cut edges and pocket surfaces are listed for sealing"
      );
      var metalWet = plan.faces.filter(face -> face.partId == "datum" && face.wettedAreaMm2 > 0);
      check(
        metalWet.length == 1 && !metalWet[0].needsSealing,
        "A resin-contacting metal fixture gets release, without board sealer"
      );
      near(metalWet[0].wettedAreaMm2, 4, "Fixture contact area comes from its exact face intersection");
      var epsFaces = plan.faces.filter(face -> face.partId == "core" && face.wettedAreaMm2 > 0);
      check(epsFaces.length == 6, "All six faces of an enclosed EPS void touch resin");
      var epsArea = 0.0;
      for (face in epsFaces) epsArea += face.wettedAreaMm2;
      near(epsArea, 24, "EPS release area is the exact cavity boundary");
      check(
        plan.faces.filter(face -> face.partId == "insert" && face.adjacentCoatedFaceIds.length > 0).length > 0,
        "Exact shared steel/board edges generate protective masks"
      );
      for (face in plan.faces) if (face.substrate == BondingSteel) {
        check(plan.mustNotCoatFaceIds.indexOf(face.id) >= 0, "Every source steel face stays protected: " + face.id);
        check(plan.coatedFaceIds.indexOf(face.id) < 0, "Neither insert nor anchor receives release: " + face.id);
      }
      near(resin.volume(), initialVolume, "Surface extraction leaves borrowed resin geometry unchanged");
      check(bed.valid() && insert.valid() && anchor.valid(), "Borrowed component geometry survives extraction");
      plan.system.epsSafety = EPSReleaseSafety.Unsafe;
      var unsafe = ReleasePlans.validate(plan).filter(row -> row.rule == "fabrication.release.eps-safety")[0];
      check(
        unsafe.status == "fail" && unsafe.affectedIds.length == 6,
        "Unsafe agent fails against every actual EPS-contact face"
      );
      for (face in epsFaces) check(
        unsafe.affectedIds.indexOf(face.id) >= 0,
        "Unsafe diagnostics identify the physical EPS face"
      );
      scope.close();
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
    var document = SinglePocketSetup.document();
    var mold = materia.tooling.casting.CastingMoldPlanner.planSinglePocket(document);
    var shop = machinekit.tooling.FormerShopExport.build(mold);
    var release = ToolingReleaseExport.build(mold, shop);
    check(release.coatedFaceIds.length > 0, "The real single-pocket mold uses the same surface query");
    check(release.mustNotCoatFaceIds.length > 0, "Its real source insert has must-not-coat faces");
    var dryEPS = release.faces.filter(face -> face.substrate == EPSCovering && face.wettedAreaMm2 == 0);
    check(dryEPS.length > 0, "The fill-side carrier joint is a dry EPS face");
    for (face in dryEPS) check(
      release.steps.filter(step -> step.action == CoverEPS && step.faceIds.indexOf(face.id) >= 0).length == 0,
      "A real carrier bond face is not taped or released"
    );
    check(
      ReleasePlans.validate(release).filter(row -> row.status == "fail").length == 0,
      "Actual panel/fixture/hardware geometry yields a complete release plan"
    );
    Sys.println('ToolingReleaseTests: $assertions assertions passed');
  }
}
