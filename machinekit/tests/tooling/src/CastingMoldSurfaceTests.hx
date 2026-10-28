import cadkit.modeling.Model;
import machinekit.tooling.CastingMoldCad;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.tooling.casting.MoldRegions;
import materia.tooling.panel.PanelContours;

/** A dry relieved neck may split the resin face, but must not remain a selectable patch. */
@:access(machinekit.tooling.CastingMoldCad) class CastingMoldSurfaceTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  public static function main():Void {
    var plan = materia.tooling.casting.CastingMoldPlanner.planSinglePocket(SinglePocketSetup.document());
    plan.construction = SharedBridges;
    plan.cavity = {x: 0, y: 0, z: 0, width: 100, depth: 100, height: 30};
    plan.formers = [];
    plan.fixtures = [];
    plan.document.input.inserts = [];
    var neck:CastingRegion = Placed({
      solid: Prism(PanelContours.relievedRectangle(50, 50, 40, 40, 3), 10),
      pose: F.translation(0, 0, 30)
    });
    plan.cores[0].cavity = Union([MoldRegions.box({x: 30, y: 30, z: 10, width: 40, depth: 40, height: 20}), neck]);
    var scope = new cadkit.modeling.Scope();
    try {
      var raw = scope.own(CastingMoldCad.unrefinedReconstruction(plan));
      var finished = scope.own(CastingMoldCad.reconstructed(plan));
      check(
        raw.shape.subshapeCount(CadKit.ShapeKind.Face) > finished.shape.subshapeCount(CadKit.ShapeKind.Face),
        "Tangent corner reliefs really create coplanar patches, and reconstruction merges them"
      );
      check(finished.valid() && finished.solidCount() == 1, "Refined resin remains one valid solid");
      var lost = scope.own(raw.subtract(finished)), added = scope.own(finished.subtract(raw));
      check(lost.volume() + added.volume() < 1e-6, "Refinement has zero symmetric difference");
      check(Math.abs(raw.volume() - finished.volume()) < 1e-6, "Refinement preserves volume");
      check(Math.abs(raw.shape.area() - finished.shape.area()) < 1e-6, "Refinement preserves surface area");
      var topFaces = 0;
      for (i in 0...finished.shape.subshapeCount(CadKit.ShapeKind.Face)) {
        var face = scope.own(new Model(finished.shape.subshape(CadKit.ShapeKind.Face, i)));
        var bounds = face.shape.bounds();
        if (Math.abs(bounds.get_min().get_z() - 30) < 1e-6
          && Math.abs(bounds.get_max().get_z() - 30) < 1e-6) topFaces++;
      }
      check(topFaces == 1, "The face around the opening highlights as one face, without four dark discs");
      scope.close();
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
    Sys.println('Casting mold surfaces: $assertions assertions passed');
  }
}
