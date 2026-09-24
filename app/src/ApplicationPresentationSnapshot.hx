package app;

import robotkit.runtime.SimulationPresentationSnapshot;
import robotkit.world.WorldSnapshot;

/** Physics poses and RobotWorld publications captured together for one UI frame. */
class ApplicationPresentationSnapshot {
	public final revision:Int;
	public final simulationTime:Float;
	public final world:WorldSnapshot;
	public final robots:Array<SimulationRobotVisual>;
	public final environment:Array<SimulationPoseVisual>;

	public function new(world:WorldSnapshot, nativeSnapshot:Null<SimulationPresentationSnapshot>,
		robots:Array<SimulationRobotVisual>, environment:Array<SimulationPoseVisual>) {
		this.world = world;
		this.revision = nativeSnapshot == null ? 0 : haxe.Int64.toInt(nativeSnapshot.stepIndex);
		this.simulationTime = nativeSnapshot == null ? 0.0 : nativeSnapshot.simulationTime;
		this.robots = robots;
		this.environment = environment;
		if (nativeSnapshot != null) nativeSnapshot.dispose();
	}

}
