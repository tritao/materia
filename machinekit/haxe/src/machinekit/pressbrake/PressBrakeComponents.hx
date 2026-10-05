package machinekit.pressbrake;

import machinekit.component.ComponentRegistry;

/** The press-brake package's component recipes. */
class PressBrakeComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(ToolSegment.recipeType());
		registry.register(ToolClamp.recipeType());
		registry.register(HydraulicCylinder.recipeType());
		registry.register(HydraulicCylinderRod.recipeType());
		registry.register(HydraulicSupply.recipeType());
		registry.register(HydraulicValve.recipeType());
		registry.register(HydraulicManifold.recipeType());
	}
}
