package machinekit.milling;

import machinekit.component.ComponentRegistry;

/** The milling package's component recipes. */
class MillingComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(MillPanel.recipeType());
		registry.register(Rke92VisePart.recipeType());
		registry.register(ZppComponent.recipeType());
		registry.register(MillFixtureAdapter.recipeType());
		registry.register(MillLevelingFoot.recipeType());
		registry.register(MillMotorCover.recipeType());
		registry.register(MillDriveHatch.recipeType());
		registry.register(MillYNutBridge.recipeType());
		registry.register(MillYNutMount.recipeType());
		registry.register(MillBeltMotorPlate.recipeType());
		registry.register(MillSupportTray.recipeType());
		registry.register(MillCasting.recipeType());
		registry.register(MillCrossSaddle.recipeType());
		registry.register(MillCrossTable.recipeType());
		registry.register(MillBallNutBracket.recipeType());
		registry.register(MillScrewPedestal.recipeType());
		registry.register(MillYDriveSeat.recipeType());
		registry.register(MillServoMount.recipeType());
		registry.register(MillZCarriage.recipeType());
		registry.register(MillZHead.recipeType());
		registry.register(MillSpindleMotorSupport.recipeType());
		registry.register(MillZBearingPlate.recipeType());
		registry.register(MillZDriveAdapter.recipeType());
		registry.register(MillZMotorStand.recipeType());
		registry.register(MillZCouplingGuard.recipeType());
		registry.register(MillSpindleBeltGuard.recipeType());
		registry.register(FlatEndMill.recipeType());
		registry.register(Bt30Cartridge.recipeType());
		registry.register(Bt30SpindleMotor.recipeType());
		registry.register(MillPowerDrawbarPart.recipeType());
		registry.register(MillSpindleMotorMount.recipeType());
		registry.register(MillSpindleDrivePart.recipeType());
		registry.register(MillSpindleDrivePart.recipeType("guard"));
		registry.register(SpindleCartridge.recipeType());
		registry.register(SpindleCartridge.Er20Holder.recipeType());
		registry.register(SpindleMotor.recipeType());
	}
}
