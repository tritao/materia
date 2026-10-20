package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class open ISO30 rack fork. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class ToolFork extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("width", 80),
    P.length("depth", 100),
    P.length("thickness", 12),
    P.length("gripDiameter", 40),
    P.length("clearance", 0.5)
  ];
  public function new(?v:ComponentValues) {
    super("tool-fork", "open ISO30 rack fork", "PA12 nylon", inputs(), v);
    positive(["width", "depth", "thickness", "gripDiameter", "clearance"]);
    require(n("gripDiameter") + 2 * n("clearance") < Math.min(n("width"),
      n("depth")), "fork opening leaves no support");
    addConnector("mount", Mount, F.translation(0, -n("depth") / 2, 0));
    addConnector("pocket", Mount, Solids.axial(0, 0, n("thickness")));
    addConnector("approach", Mount, F.translation(0, n("depth"), n("thickness")));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.tool-fork", inputs(), v -> new ToolFork(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.cut(
      Solids.named(Part.box(n("width"), n("depth"), n("thickness")), "fork"),
      [
        Solids.named(
          Part.cylinderSpan(
            n("gripDiameter") / 2 + n("clearance"),
            -1,
            n("thickness") + 1
          ),
          "grip"
        ),
        Solids.named(
          Part.box(
            n("gripDiameter") + 2 * n("clearance"),
            n("depth"),
            n("thickness") + 2,
            Center,
            Min,
            Min
          ),
          "entry"
        )
      ]
    );
  }

}
