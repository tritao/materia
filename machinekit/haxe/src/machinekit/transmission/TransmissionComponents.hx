package machinekit.transmission;

import machinekit.component.ComponentRegistry;

/** The transmission package's component recipes. */
class TransmissionComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(Sprocket.chainRecipeType());
		registry.register(Sprocket.genericRecipeType());
		registry.register(SpurGear.recipeType());
		registry.register(Rack.recipeType());
		registry.register(TimingPulley.standardRecipeType());
		registry.register(TimingPulley.customRecipeType());
		registry.register(TimingBelt.pairRecipeType());
	}
}
