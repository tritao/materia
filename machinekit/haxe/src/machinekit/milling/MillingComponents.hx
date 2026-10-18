package machinekit.milling;

import machinekit.component.ComponentRegistry;

/** The milling package's component recipes. */
class MillingComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(MillPanel.recipeType());
		registry.register(MillLevelingFoot.recipeType());
		registry.register(MillMotorCover.recipeType());
		registry.register(MillBeltMotorPlate.recipeType());
		registry.register(MillSupportTray.recipeType());
		registry.register(MillCasting.recipeType());
		registry.register(MillCrossSaddle.recipeType());
registry.register(MillCrossTable.recipeType());
registry.register(MillBallNutBracket.recipeType());
registry.register(MillScrewPedestal.recipeType());
registry.register(MillServoMount.recipeType());
		registry.register(MillZCarriage.recipeType());
		registry.register(MillZHead.recipeType());
		registry.register(MillZBearingPlate.recipeType());
		registry.register(MillZMotorStand.recipeType());
		registry.register(SpindleCartridge.recipeType());
		registry.register(SpindleCartridge.Er20Holder.recipeType());
		registry.register(SpindleMotor.recipeType());
	}
}
