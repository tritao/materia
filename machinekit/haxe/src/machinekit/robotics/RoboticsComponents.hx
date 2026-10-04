package machinekit.robotics;

import machinekit.component.ComponentRegistry;

/** The robotics package's component recipes. */
class RoboticsComponents {
	public static function register(registry:ComponentRegistry):Void {
		registry.register(GearedArmJoint.recipeType());
		registry.register(CobotJoint.moduleRecipeType());
		registry.register(RobotFlange.recipeType());
		registry.register(EndEffectorPlate.recipeType());
		registry.register(Pedestal.recipeType());
		registry.register(FrameBar.recipeType());
		registry.register(ArmJoint.recipeType());
		registry.register(ArmLink.recipeType());
		registry.register(ParallelGripper.recipeType());
		registry.register(ToolChangerMaster.recipeType());
		registry.register(ToolChangerTool.recipeType());
		registry.register(machinekit.robotics.schmalz.SchmalzSxtMaster.recipeType());
		registry.register(machinekit.robotics.schmalz.SchmalzSxtTool.recipeType());
	}
}
