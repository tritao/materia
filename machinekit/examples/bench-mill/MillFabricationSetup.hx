import materia.tooling.export.FabricationSpec;
import camkit.CamToolingPanels.PanelCamSettings;
import materia.tooling.panel.PanelPlan;

/** Mill identity only; workshop choices live in the shared shop declaration. */
class MillFabricationSetup {
  public static function spec():FabricationSpec return CastingShopRecipe.fabrication();
  public static function process(recipe:materia
    .tooling.ToolingRecipe):materia.tooling.process.CastingProcessSpec return CastingShopRecipe.process(
      recipe,
      "mill-base-process-review"
    );
  public static function cam(panels:Array<PanelPlan>):PanelCamSettings return CastingShopRecipe.cam(panels);
}
