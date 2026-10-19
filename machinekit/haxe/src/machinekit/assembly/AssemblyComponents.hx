package machinekit.assembly;

import machinekit.component.ComponentRegistry;

/** The assembly package's component recipes. */
class AssemblyComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(NemaMountPlate.recipeType());
        registry.register(ScrewAxisPart.recipeType());
        registry.register(Sfu20AxisPart.recipeType());
        registry.register(ScrewAxisCoupling.axisRecipeType());
		registry.register(RbkMotorMount.recipeType());
		registry.register(HzMotionComponent.recipeType());
		registry.register(HzMotionComponent.cadRecipeType());
		registry.register(machinekit.assembly.LinearAxis.Carriage.recipeType());
	}
}
