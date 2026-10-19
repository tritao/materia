package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class ER32 collet. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class Collet extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("diameter", 33),
    P.length("length", 40),
    P.length("toolDiameter", 8)
  ];
  public function new(?v:ComponentValues) {
    super("collet", "ER32 collet", "steel", inputs(), v);
    positive(["diameter", "length", "toolDiameter"]);
    require(n("toolDiameter") < n("diameter"), "tool bore exceeds collet body");
    addConnector("holder", Mount, Solids.axial(0, 0, n("length")));
    addConnector("tool", Axis, Solids.axial(0, 0, 0));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.collet", inputs(), v -> new Collet(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.cut(
      Solids.named(Part.cylinder(n("diameter") / 2, n("length")), "collet-envelope"),
      [Solids.named(
        Part.cylinderSpan(
          n("toolDiameter") / 2,
          -1,
          n("length") + 1
        ),
        "tool-bore"
      )]
    );
  }

}
