package machinekit.standard;

import machinekit.component.ComponentRegistry;

/** The standard package's component recipes. */
class StandardComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(DeepGrooveBearing.recipeType());
		registry.register(SocketHeadCapScrew.recipeType());
		registry.register(HexBolt.recipeType());
		registry.register(HexNut.recipeType());
		registry.register(FlatWasher.recipeType());
		registry.register(ParallelKey.recipeType());
		registry.register(RetainingRing.recipeType());
		registry.register(ShaftCollar.recipeType());
		registry.register(Bushing.recipeType());
	}
}
