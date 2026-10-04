package machinekit.component;

/** MachineKit's built-in component recipes, registered package by package. */
class MachineKitComponents {
	static var shared:Null<ComponentRegistry>;

	/** A new registry holding only the built-in recipes. */
	public static function standard():ComponentRegistry {
		var registry = new ComponentRegistry();
		machinekit.standard.StandardComponents.register(registry);
		machinekit.motion.MotionComponents.register(registry);
		machinekit.transmission.TransmissionComponents.register(registry);
		machinekit.robotics.RoboticsComponents.register(registry);
		machinekit.pneumatic.PneumaticComponents.register(registry);
		machinekit.milling.MillingComponents.register(registry);
		machinekit.assembly.AssemblyComponents.register(registry);
		return registry;
	}

	/**
	 * The registry saved assemblies load through unless the caller passes one: the built-ins plus
	 * whatever an application registers into it before loading.
	 */
	public static function defaultRegistry():ComponentRegistry {
		if (shared == null) shared = standard();
		return shared;
	}
}
