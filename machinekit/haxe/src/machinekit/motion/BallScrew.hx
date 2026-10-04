package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** Generic SFU1605 screw. Race root and efficiency are assumed reference values.
 * The shaft, support critical speed and transmission use the same screw model as sliding screws.
 */
class BallScrew extends LeadScrew {
	/** Assumed machined bearing journals, shared by support and coupling placement. */
	public static inline var JOURNAL_DIAMETER:Float = 12;
	public static inline var JOURNAL_LENGTH:Float = 25;
	public static inline var INPUT_JOURNAL_LENGTH:Float = 50;
	public function new(length:Float) {
		super(new LeadScrewThread(Ball, 16, 5), length);
		if (length <= INPUT_JOURNAL_LENGTH + JOURNAL_LENGTH) throw "Ball screw needs a race between its journals";
	}

	override public function geometry(detail:machinekit.component.ComponentDetail = Preview):cadkit.modeling.Part {
		return machinekit.component.Solids.union([
			machinekit.component.Solids.named(cadkit.modeling.Part.cylinderSpan(JOURNAL_DIAMETER / 2, 0, INPUT_JOURNAL_LENGTH), "input-journal"),
			machinekit.component.Solids.named(cadkit.modeling.Part.cylinderSpan(thread.screwDiameter / 2, INPUT_JOURNAL_LENGTH, totalLength - JOURNAL_LENGTH), "race"),
			machinekit.component.Solids.named(cadkit.modeling.Part.cylinderSpan(JOURNAL_DIAMETER / 2, totalLength - JOURNAL_LENGTH, totalLength), "output-journal")]);
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
