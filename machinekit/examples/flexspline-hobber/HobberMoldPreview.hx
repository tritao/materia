import hobber.*;
import HobberMoldSetup.HobberMoldCasting;

/** Same native planner, scene and fabrication exporter as the mill kits. */
class HobberMoldPreview {
  static function foundation(casting:HobberMoldCasting):machinekit.structural.MineralFoundation return switch casting {
    case HobberMoldCasting.Plinth:
      new hobber.MineralCastBed();
    case HobberMoldCasting.Column:
      new hobber.MineralCastColumn();
    default:
      throw "Unknown hobber mold casting: " + casting;
  };

  public static function plinthScene():haxe.io.Bytes return linkedScene(HobberMoldCasting.Plinth);
  public static function columnScene():haxe.io.Bytes return linkedScene(HobberMoldCasting.Column);
  static function linkedScene(casting:HobberMoldCasting):haxe.io.Bytes {
    var data = materia.project.SceneArtifact.decode(scene(casting));
    var project:materia.project.SceneArtifact.SceneArtifactProject = cast data.project;
    project.navigation = [{
      occurrence: machinekit.tooling.CastingMoldView.REFERENCE_ID,
      label: "Back to Machine",
      variant: "machine",
      target: casting == HobberMoldCasting.Plinth ? "machine/bed/casting" : "machine/column/casting"
    }];
    return materia.project.SceneArtifact.encode(data);
  }

  public static function scene(casting:HobberMoldCasting = HobberMoldCasting.Plinth):haxe.io.Bytes {
    var foundation = foundation(casting);
    var document = HobberMoldSetup.document(foundation, casting);
    var plan = machinekit.tooling.CastingMoldCad.plan(document, HobberMoldSetup.options(document.input));
    return machinekit.tooling.CastingMoldExport.artifact(plan);
  }

  public static function main():Void {
    var casting:HobberMoldCasting = Sys.args().indexOf("--column")
      >= 0 ? HobberMoldCasting.Column : HobberMoldCasting.Plinth;
    var foundation = foundation(casting);
    var document = HobberMoldSetup.document(foundation, casting), cwd = Sys.getCwd();
    var root = sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/examples/flexspline-hobber" : cwd;
    CastingShopExport.write(
      document,
      HobberMoldSetup.options(document.input),
      foundation,
      casting,
      "hobber-" + casting + "-process-review",
      root,
      HobberMoldChecks.run,
      p -> HobberMoldChecks.liftingReview(p.document, p.handling)
    );
  }
}
