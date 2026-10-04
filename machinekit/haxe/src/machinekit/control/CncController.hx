package machinekit.control;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** CNC cabinet with drive references, digital inputs, general outputs and valve coils. */
class CncController extends ControllerCabinet {
	static var recipe:Null<ComponentType>;
	public function new(axes:Int = 4, inputs:Int = 16, outputs:Int = 16, valves:Int = 8)
		super("cnc", axes, inputs, outputs, valves);

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.control.cnc-controller", [
			ComponentRecipeSupport.count("axes", 4), ComponentRecipeSupport.count("inputs", 16),
			ComponentRecipeSupport.count("outputs", 16), ComponentRecipeSupport.count("valves", 8)
		], v -> new CncController(v.integer("axes"), v.integer("inputs"), v.integer("outputs"), v.integer("valves")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setInteger("axes", axes).setInteger("inputs", inputs).setInteger("outputs", outputs)
		.setInteger("valves", valves).setToken("material", materialSpec());
}
