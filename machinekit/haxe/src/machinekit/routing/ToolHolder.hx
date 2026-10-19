package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class ISO30 ER32 tool holder. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class ToolHolder extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("gaugeDiameter", 31.75),
    P.length("taperLength", 45),
    P.length("flangeDiameter", 50),
    P.length("flangeThickness", 8),
    P.length("noseDiameter", 40),
    P.length("gaugeLength", 60),
    P.length("colletBore", 33.1),
    P.length("colletSeatLength",40)
  ];
  public function new(?v:ComponentValues) {
    super("tool-holder", "ISO30 ER32 tool holder", "steel", inputs(), v);
    positive([
      "gaugeDiameter",
      "taperLength",
      "flangeDiameter",
      "flangeThickness",
      "noseDiameter",
      "gaugeLength",
      "colletBore",
      "colletSeatLength"
    ]);
    require(
      n("gaugeDiameter") > n("taperLength") * 7 / 24
      && n("colletBore") < n("noseDiameter") && n("colletSeatLength") < n("gaugeLength")-n("flangeThickness"),
      "invalid holder sections"
    );
    addConnector("gauge", Axis, Solids.axial(0, 0, 0));
    addConnector("fork", Mount, Solids.axial(0, 0, -n("flangeThickness")));
    addConnector("collet", Mount, Solids.axial(0, 0, -n("gaugeLength")+n("colletSeatLength")));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.tool-holder", inputs(), v -> new ToolHolder(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.named(Part.revolve([{r: n("colletBore") / 2, z: -n("gaugeLength")}, {
      r: n("noseDiameter") / 2,
      z: -n("gaugeLength")
    }, {
      r: n("noseDiameter") / 2,
      z: -n("flangeThickness")
    }, {
      r: n("flangeDiameter") / 2,
      z: -n("flangeThickness")
    }, {
      r: n("flangeDiameter") / 2,
      z: 0
    }, {
      r: n("gaugeDiameter") / 2,
      z: 0
    }, {
      r:(n("gaugeDiameter") - n("taperLength") * 7 / 24) / 2,
      z: n("taperLength")
    }, {
      r: 0,
      z: n("taperLength")
    }, {
      r: 0,
      z: -n("gaugeLength")+n("colletSeatLength")
    }, {
      r: n("colletBore") / 2,
      z: -n("gaugeLength")+n("colletSeatLength")
    }
    ]), "holder");
  }

}
