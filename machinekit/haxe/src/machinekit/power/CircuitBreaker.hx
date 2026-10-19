package machinekit.power;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class three-pole circuit breaker. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class CircuitBreaker extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.quantity("ratedCurrent", 32, "A"),
    P.quantity("ratedVoltage", 400, "V"),
    P.quantity("breakingCapacity", 6000, "A"),
    P.count("poles", 3),
    P.length("poleWidth", 18),
    P.length("depth", 75),
    P.length("height", 90)
  ];
  public function new(?v:ComponentValues) {
    super("circuit-breaker", "three-pole circuit breaker", "steel", inputs(), v);
    positive(["ratedCurrent", "ratedVoltage", "breakingCapacity", "poleWidth", "depth", "height"]);
    require(parameterCount("poles") > 0, "breaker needs poles");
    addConnector("din", Mount, F.translation(0, 0, 0));
    for (name in["line", "load"]) addPort({
      name: name,
      kind: ElectricalPower,
      role: name == "line" ? Consumer : Supply,
      iface : Plug(
        "power",
        parameterCount("poles")
      ),
      required : true
    }
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.power.circuit-breaker",
      inputs(),
      v -> new CircuitBreaker(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.named(Part.box(n("poleWidth") * parameterCount("poles"),
      n("depth"), n("height")), "breaker-envelope");
  }

}
