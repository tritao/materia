package machinekit.structural;

import machinekit.component.ComponentRegistry;
import machinekit.component.ComponentKind;

/** Shared mount recipes declare their classification beside their registration. */
class LinearMountComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(LinearMountSteel.recipeType());
        registry.register(LinearMountCleat.recipeType());
        registry.register(MountDrilledPlate.recipeType());
		registry.register(LinearMountReplication.recipeType());
		registry.classify("machinekit.structural.linear-mount-replication","Precision mounts",ComponentKind.MachinePart);
		registry.register(LinearMountNutCassette.recipeType());
		registry.classify("machinekit.structural.linear-mount-nut-cassette","Precision mounts",ComponentKind.MachinePart);
		registry.classify("machinekit.structural.linear-mount-cleat","Precision mounts",ComponentKind.MachinePart);
        registry.classify("machinekit.structural.mount-drilled-plate","Precision mounts",ComponentKind.MachinePart);
        registry.classify("machinekit.structural.linear-mount-steel","Precision mounts",ComponentKind.MachinePart);
	}
}
