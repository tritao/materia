package robotkit.runtime;

import RobotKitSimKit;
import RobotKitRuntime;
import haxe.Int64;

/** Owned, immutable native presentation copied at one simulation revision. */
class SimulationPresentationSnapshot {
	public static inline var ROBOT_BASE:Int = 1;
	public static inline var ROBOT_LINK:Int = 2;
	public static inline var ENVIRONMENT:Int = 3;

	public final stepIndex:Int64;
	public final simulationTime:Float;
	public final poses:Array<SimulationPresentationPose>;
	final owner:Ownedrk_simulation_presentation;
	var disposed:Bool = false;

	@:allow(robotkit.runtime.Simulation)
	private function new(owner:Ownedrk_simulation_presentation) {
		this.owner = owner;
		var info = new rk_simulation_presentation_info();
		info.set_struct_size(rk_simulation_presentation_info.size());
		check(RobotKitSimKit.rk_simulation_presentation_get_info(owner.borrow(), info).status,
			"simulation.presentation.info");
		stepIndex = info.get_step_index();
		simulationTime = info.get_simulation_time();
		poses = [];
		for (index in 0...info.get_pose_count()) {
			var value = new rk_simulation_presentation_pose();
			value.set_struct_size(rk_simulation_presentation_pose.size());
			check(RobotKitSimKit.rk_simulation_presentation_get_pose(owner.borrow(), index, value).status,
				"simulation.presentation.pose");
			poses.push({kind:value.get_kind(), robotIndex:value.get_robot_index(),
				linkIndex:value.get_link_index(), objectId:value.get_object_id(),
				position:[for (component in 0...3) value.get_position(component)],
				rotation:[for (component in 0...4) value.get_rotation(component)]});
		}
	}

	public function dispose():Void {
		if (disposed) return;
		owner.close();
		disposed = true;
	}

	static function check(status:Int, operation:String):Void
		if (status != RobotKitRuntimeConstants.RK_OK)
			throw '$operation failed with RobotKit status $status';
}
