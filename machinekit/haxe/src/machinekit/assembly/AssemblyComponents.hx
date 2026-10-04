package machinekit.assembly;

import machinekit.component.ComponentRegistry;

/** The assembly package's component recipes. */
class AssemblyComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(machinekit.assembly.LinearAxis.Carriage.recipeType());
	}
}
