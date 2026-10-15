package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** Generic SFU1605 screw. Race root and efficiency are assumed reference values.
 * The shaft, support critical speed and transmission use the same screw model as sliding screws.
 */
class BallScrew extends LeadScrew {
	/** Assumed machined bearing journals, shared by support and coupling placement. */
	/** Assumed SFU1605 race root; not a formula for other nominal sizes. */
	public static inline var RACE_ROOT_DIAMETER:Float = 13;
	public static function race():LeadScrewThread return new LeadScrewThread(Ball, 16, 5, 1, RightHand, RACE_ROOT_DIAMETER);
	public static inline var JOURNAL_DIAMETER:Float = 12;
	public static inline var JOURNAL_LENGTH:Float = 25;
	public static inline var INPUT_JOURNAL_LENGTH:Float = 50;
	public function new(length:Float) {
		super(race(), length);
		if (length <= INPUT_JOURNAL_LENGTH + JOURNAL_LENGTH) throw "Ball screw needs a race between its journals";
	}

	override public function axialRigidityAt(position:Float):Float {
		if (position>=INPUT_JOURNAL_LENGTH && position<=totalLength-JOURNAL_LENGTH) return super.axialRigidityAt(position);
		return YOUNGS_MODULUS/1e6*Math.PI*JOURNAL_DIAMETER*JOURNAL_DIAMETER/4.0;
	}

	override public function axialComplianceBetween(from:Float, to:Float):Float {
		if (!Math.isFinite(from) || !Math.isFinite(to) || Math.min(from,to)<0 || Math.max(from,to)>totalLength)
			throw "Axial load path must lie on the ball screw shaft";
		var low = Math.min(from,to), high = Math.max(from,to), compliance = 0.0;
		var boundaries = [0.0,INPUT_JOURNAL_LENGTH,totalLength-JOURNAL_LENGTH,totalLength];
		for (index in 0...3) {
			var a = Math.max(low,boundaries[index]), b = Math.min(high,boundaries[index+1]);
			if (b>a) compliance += (b-a)/axialRigidityAt((a+b)*0.5);
		}
		return compliance;
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
