package machinekit.assembly;

import machinekit.component.ComponentRegistry;

/** The assembly package's component recipes. */
class AssemblyComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(MotorPlate.recipeType());
        registry.register(ScrewAxisPart.recipeType());
        registry.register(Sfu20AxisPart.recipeType());
        registry.register(ScrewAxisCoupling.axisRecipeType());
		registry.register(machinekit.motion.MotorMount.recipeType());
		registry.register(machinekit.motion.MotorAdapterPlate.recipeType());
		registry.register(machinekit.assembly.LinearAxis.Carriage.recipeType());
	}
}
