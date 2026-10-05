package machinekit.control;

import machinekit.component.ComponentRegistry;

/** The control package's component recipes. */
class ControlComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(CncController.recipeType());
		registry.register(RobotController.recipeType());
		registry.register(DigitalSensor.recipeType());
		registry.register(ControllerCabinet.Support.recipeType());
	}
}
