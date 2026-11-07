package machinekit.motion;

import machinekit.component.ComponentRegistry;

/** The motion package's component recipes. */
class MotionComponents {
  public static function register(registry:ComponentRegistry):Void {
    registry.register(TorqueMotor.recipeType());
    registry.register(AngularContactBearing.recipeType());
    registry.register(SpindleBearing.recipeType());
    registry.register(CrossedRollerBearing.recipeType());
    registry.register(RotaryEncoderRing.recipeType());
    registry.register(PlanetaryReducer.recipeType());
    registry.register(CableChain.recipeType());
    registry.register(machinekit.gantry.GantryCableChain.recipeType());
    registry.register(BrakeServo.recipeType());

    registry.register(LinearBearing.recipeType());
    registry.register(PillowBlock.recipeType());
    registry.register(PurchasedRotaryPart.recipeType());
    registry.register(ScrewLinearModulePart.recipeType());
    registry.register(ScrewLinearModuleCover.recipeType());
    registry.register(LinearRail.recipeType());
    registry.register(LinearRailBlock.recipeType());
    registry.register(SfuNut.familyRecipeType());
    registry.register(SfuDrawingScrew.drawingRecipeType());
    registry.register(DsgNutHousing.familyRecipeType());
    registry.register(NemaStepper.namedRecipeType());
    registry.register(NemaStepper.genericRecipeType());
    registry.register(ServoMotor.namedRecipeType());
    registry.register(AxisServoMotor.recipeType());
    registry.register(ShaftBrakePart.recipeType());
    registry.register(MotorDriver.recipeType());
    registry.register(PowerSupply.recipeType());
    registry.register(machinekit.power.ThreePhaseRectifier.recipeType());
    registry.register(machinekit.power.ThreePhaseDcSupply.recipeType());
    registry.register(Gearbox.recipeType());
    registry.register(FlangeBearingHousing.recipeType());
    registry.register(LeadScrew.recipeType());
    registry.register(BallScrew.ballRecipeType());
    registry.register(BallScrewRetainer.recipeType());
    registry.register(BallNut.ballRecipeType());
    registry.register(HzMotionDrawingScrew.recipeType());
    registry.register(ScrewSupportUnit.recipeType());
    registry.register(LeadScrewNut.recipeType());
    registry.register(NutBlock.recipeType());
    registry.register(DriveWheel.recipeType());
    registry.register(CasterWheel.recipeType());
    registry.register(SteppedShaft.recipeType());
    registry.register(ShaftCoupling.recipeType());
    registry.register(ShaftEncoder.recipeType());
    registry.register(LinearScale.recipeType());
    registry.register(LimitSwitch.recipeType());
    registry.register(ProximitySwitch.recipeType());
    registry.register(ErobActuatorPart.recipeType());
    registry.register(SealedScalePart.recipeType());
  }
}
