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
		machinekit.hobbing.HobbingComponents.register(registry);
		machinekit.pressbrake.PressBrakeComponents.register(registry);
		machinekit.printing.PrintingComponents.register(registry);
		machinekit.assembly.AssemblyComponents.register(registry);
		machinekit.control.ControlComponents.register(registry);
		registry.register(machinekit.structural.LevelingFoot.recipeType());
		registry.register(machinekit.structural.LevelingSeat.recipeType());
		registry.register(machinekit.structural.RectTubeMember.recipeType());
		registry.register(machinekit.structural.ProfileMember.tslotRecipeType());
		registry.register(machinekit.structural.ProfileMember.drawingRecipeType());
		registry.register(machinekit.structural.SlideInSlotNut.recipeType());
		registry.register(machinekit.structural.MineralCasting.recipeType());
		registry.register(machinekit.structural.CastInsert.recipeType());
		registry.register(machinekit.tooling.ToolingPanel.recipeType());
		registry.register(machinekit.tooling.ToolingPin.recipeType());
		registry.register(machinekit.tooling.ToolingBlock.recipeType());
		registry.register(machinekit.tooling.ToolingVolume.recipeType());
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
