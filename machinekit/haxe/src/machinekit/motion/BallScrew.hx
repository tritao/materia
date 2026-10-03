package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** Generic SFU1605 screw. Race root and efficiency are assumed reference values.
 * The shaft, support critical speed and transmission use the same screw model as sliding screws.
 */
class BallScrew extends LeadScrew {
	public function new(length:Float) {
		super(new LeadScrewThread(Ball, 16, 5), length);
	}

	public static function sfu1605(length:Float):BallScrew return new BallScrew(length);

	static var ballRecipe:Null<ComponentType>;
	public static function ballRecipeType():ComponentType {
		if (ballRecipe == null) ballRecipe = new ComponentType("machinekit.motion.ball-screw",
			[ComponentRecipeSupport.length("length", 400)], v -> new BallScrew(v.number("length")));
		return ballRecipe;
	}
	override public function componentType():Null<ComponentType> return ballRecipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setNumber("length", totalLength).setToken("material", materialSpec());
}
