package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;

/** Generic SFU1605 flanged nut. Dimensions, preload drag and zero reversal clearance are assumed;
 * they describe a preloaded nut, not an accuracy guarantee for an unpreloaded vendor assembly.
 */
class BallNut extends LeadScrewNut {
	public final preloaded:Bool;

	public function new(preloaded:Bool = true) {
		super(new LeadScrewThread(Ball, 16, 5), 6, true, preloaded ? PreloadedBallNut : BallNut,
			{bodyDiameter: 28, bodyLength: 35, flangeDiameter: 48, flangeThickness: 10,
				boltCircleDiameter: 38, mountScrew: "M5"});
		this.preloaded = preloaded;
		setMaterial("steel");
	}

	static var ballRecipe:Null<ComponentType>;
	public static function ballRecipeType():ComponentType {
		if (ballRecipe == null) ballRecipe = new ComponentType("machinekit.motion.ball-nut",
			[ComponentRecipeSupport.flag("preloaded", true)], v -> new BallNut(v.boolean("preloaded")));
		return ballRecipe;
	}
	override public function componentType():Null<ComponentType> return ballRecipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setBoolean("preloaded", preloaded).setToken("material", materialSpec());
}
