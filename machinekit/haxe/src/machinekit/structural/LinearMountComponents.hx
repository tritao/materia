package machinekit.structural;

import machinekit.component.ComponentRegistry;
import machinekit.component.ComponentKind;

/** Shared mount recipes declare their classification beside their registration. */
class LinearMountComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(LinearMountSteel.recipeType());
		registry.classify("machinekit.structural.linear-mount-steel","Precision mounts",ComponentKind.MachinePart);
	}
}
