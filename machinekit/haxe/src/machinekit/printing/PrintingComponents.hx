package machinekit.printing;

import machinekit.component.ComponentRegistry;

/** The printing package's component recipes. */
class PrintingComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(Thermistor.recipeType());
		registry.register(HeaterCartridge.recipeType());
		registry.register(Nozzle.recipeType());
		registry.register(Hotend.recipeType());
		registry.register(Extruder.recipeType());
		registry.register(HeatedBed.recipeType());
		registry.register(Fan.recipeType());
	}
}
