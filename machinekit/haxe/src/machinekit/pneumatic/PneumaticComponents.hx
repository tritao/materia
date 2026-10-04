package machinekit.pneumatic;

import machinekit.component.ComponentRegistry;

/** The pneumatic package's component recipes. */
class PneumaticComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(PneumaticManifold.recipeType());
		registry.register(SuctionCup.recipeType());
		registry.register(VacuumGenerator.recipeType());
		registry.register(VacuumControlValve.recipeType());
		registry.register(VacuumPressureSensor.recipeType());
		registry.register(RoutedHose.recipeType());
		registry.register(PneumaticCylinder.recipeType());
		registry.register(PneumaticCylinderRod.recipeType());
		registry.register(AirSupply.recipeType());
		registry.register(SolenoidValve.recipeType());
		registry.register(machinekit.pneumatic.schmalz.SchmalzPushInFitting.recipeType());
		registry.register(machinekit.pneumatic.schmalz.SchmalzSuctionCup.recipeType());
		registry.register(machinekit.pneumatic.schmalz.SchmalzVacuumGenerator.recipeType());
		registry.register(machinekit.pneumatic.schmalz.SchmalzVacuumHose.recipeType());
	}
}
