package machinekit.pressbrake;

import machinekit.component.ComponentRegistry;

/** The press-brake package's component recipes. */
class PressBrakeComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(ToolSegment.recipeType());
		registry.register(ToolClamp.recipeType());
	}
}
