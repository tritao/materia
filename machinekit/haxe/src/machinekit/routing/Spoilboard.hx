package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class bleeder spoilboard. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class Spoilboard extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("width", 1600),
    P.length("length", 2600),
    P.length("thickness", 25),
    P.quantity("permeability", 0.000001, "mm2")
  ];
  public function new(?v:ComponentValues) {
    super("spoilboard", "bleeder spoilboard", "MDF", inputs(), v);
    positive(["width", "length", "thickness", "permeability"]);
    addConnector("bottom", Mount, F.translation(0, 0, 0));
    addConnector("top", Face, F.translation(0, 0, n("thickness")));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.spoilboard", inputs(), v -> new Spoilboard(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /** Convert the assumed permeability to SI for the later bleed-flow model. */
  public function permeabilityM2():Float return n("permeability") * 1e-6;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.named(Part.box(n("width"), n("length"), n("thickness")), "bleeder-board");
  }

}
