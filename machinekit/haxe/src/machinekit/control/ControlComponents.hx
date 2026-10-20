package machinekit.control;

import machinekit.component.ComponentRegistry;

/** The control package's component recipes. */
class ControlComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(SafetyRelay.recipeType());
		registry.register(EmergencyStopButton.recipeType());
		registry.register(ControlCabinet.recipeType());

		registry.register(CncController.recipeType());
		registry.register(RobotController.recipeType());
		registry.register(DigitalSensor.recipeType());
		registry.register(ThermistorInput.recipeType());
	}
}
