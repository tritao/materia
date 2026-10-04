package machinekit.control;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** Robot cabinet with one drive reference per axis and separately wired digital I/O. */
class RobotController extends ControllerCabinet {
	static var recipe:Null<ComponentType>;
	public function new(axes:Int = 6, inputs:Int = 16, outputs:Int = 16, valves:Int = 4)
		super("robot", axes, inputs, outputs, valves);

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.control.robot-controller", [
			ComponentRecipeSupport.count("axes", 6), ComponentRecipeSupport.count("inputs", 16),
			ComponentRecipeSupport.count("outputs", 16), ComponentRecipeSupport.count("valves", 4)
		], v -> new RobotController(v.integer("axes"), v.integer("inputs"), v.integer("outputs"), v.integer("valves")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setInteger("axes", axes).setInteger("inputs", inputs).setInteger("outputs", outputs)
		.setInteger("valves", valves).setToken("material", materialSpec());
}
