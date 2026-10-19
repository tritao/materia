package machinekit.motion;

import machinekit.component.ComponentRegistry;

/** The motion package's component recipes. */
class MotionComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(LinearBearing.recipeType());
		registry.register(PillowBlock.recipeType());
		registry.register(LinearRail.recipeType());
		registry.register(LinearRailBlock.recipeType());
		registry.register(NemaStepper.namedRecipeType());
		registry.register(NemaStepper.genericRecipeType());
		registry.register(ServoMotor.namedRecipeType());
		registry.register(LeadshineServo.recipeType());
		registry.register(ShaftBrakePart.recipeType());
		registry.register(MotorDriver.recipeType());
		registry.register(PowerSupply.recipeType());
		registry.register(machinekit.power.ThreePhaseRectifier.recipeType());
		registry.register(Gearbox.recipeType());
		registry.register(FlangeBearingHousing.recipeType());
		registry.register(LeadScrew.recipeType());
		registry.register(BallScrew.ballRecipeType());
		registry.register(BallScrewRetainer.recipeType());
		registry.register(BallNut.ballRecipeType());
		registry.register(VallderSfu1605Screw.drawingRecipeType());
		registry.register(HzMotionDrawingScrew.recipeType());
		registry.register(VallderSfu1605Nut.drawingRecipeType());
		registry.register(VallderBf12Support.drawingRecipeType());
		registry.register(ScrewSupportUnit.recipeType());
		registry.register(LeadScrewNut.recipeType());
		registry.register(DriveWheel.recipeType());
		registry.register(CasterWheel.recipeType());
		registry.register(SteppedShaft.recipeType());
		registry.register(ShaftCoupling.recipeType());
		registry.register(ShaftEncoder.recipeType());
		registry.register(LinearScale.recipeType());
		registry.register(LimitSwitch.recipeType());
		registry.register(ProximitySwitch.recipeType());
	}
}
