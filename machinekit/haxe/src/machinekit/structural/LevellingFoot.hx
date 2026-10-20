package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class load-rated levelling foot. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class LevellingFoot extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("padDiameter", 100),
    P.length("padHeight", 20),
    P.length("stemDiameter", 20),
    P.length("stemLength", 100),
    P.quantity("ratedLoad", 10000, "N"),
    P.length("adjustment", 40)
  ];
  public function new(?v:ComponentValues) {
    super("levelling-foot", "load-rated levelling foot", "steel", inputs(), v);
    positive(["padDiameter", "padHeight", "stemDiameter", "stemLength", "ratedLoad", "adjustment"]);
    require(n("stemDiameter") < n("padDiameter") && n("adjustment") < n("stemLength"), "invalid foot adjustment/stem");
    addConnector("floor", Mount, Solids.axial(0, 0, 0));
    addConnector("frame", Mount, Solids.axial(0, 0, n("padHeight") + n("stemLength") - n("adjustment") / 2));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.structural.levelling-foot",
      inputs(),
      v -> new LevellingFoot(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.union([
      Solids.named(Part.cylinder(n("padDiameter") / 2, n("padHeight")), "pad"),
      Solids.named(
        Part.cylinderSpan(
          n("stemDiameter") / 2,
          n("padHeight"),
          n("padHeight") + n("stemLength")
        ),
        "stem"
      )
    ]);
  }

}
