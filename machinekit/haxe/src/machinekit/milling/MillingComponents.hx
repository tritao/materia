package machinekit.milling;

import machinekit.component.ComponentRegistry;

/** The milling package's component recipes. */
class MillingComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(MillPanel.recipeType());
		registry.register(MillCasting.recipeType());
		registry.register(SpindleCartridge.recipeType());
		registry.register(SpindleCartridge.Er20Holder.recipeType());
		registry.register(SpindleMotor.recipeType());
	}
}
