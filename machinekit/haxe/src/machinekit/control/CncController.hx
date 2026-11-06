package machinekit.control;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** CNC cabinet with drive references, digital inputs, general outputs and valve coils. */
class CncController extends ControllerCabinet {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public function new(axes:Int = 4, inputs:Int = 16, outputs:Int = 16, valves:Int = 8) super(
    "cnc",
    axes,
    inputs,
    outputs,
    valves
  );

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.control.cnc-controller",
      [
        ComponentRecipeSupport.count("axes", 4),
        ComponentRecipeSupport.count("inputs", 16),
        ComponentRecipeSupport.count("outputs", 16),
        ComponentRecipeSupport.count("valves", 8)
      ],
      v -> new CncController(v.integer("axes"), v.integer("inputs"), v.integer("outputs"), v.integer("valves"))
    ).numbered(v -> 'CNC-CTRL-ASSUMED-${v.integer("axes")}A-${v.integer("inputs")}I-${v.integer("outputs")}O-${v.integer("valves")}V')
      .procured(_ -> Buy(null, null, null)).requiringInputs([], ["axes", "inputs", "outputs", "valves"]));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setInteger("axes", axes).setInteger(
    "inputs",
    inputs
  ).setInteger(
    "outputs",
    outputs
  ).setInteger(
    "valves",
    valves
  ).setToken(
    "material",
    materialSpec()
  );
}
