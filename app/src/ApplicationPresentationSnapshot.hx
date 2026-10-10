package app;

import robotkit.runtime.SimulationPresentationSnapshot;

/** Physics poses and robot stream observations captured together for one UI frame. */
class ApplicationPresentationSnapshot {
	public final revision:Int;
	public final simulationTime:Float;
	public final robots:Array<SimulationRobotVisual>;
	public final environment:Array<SimulationPoseVisual>;

	public function new(nativeSnapshot:Null<SimulationPresentationSnapshot>,
		robots:Array<SimulationRobotVisual>, environment:Array<SimulationPoseVisual>,
		presentationEpoch:Int = 0) {
		this.revision = (nativeSnapshot == null ? 0 : haxe.Int64.toInt(nativeSnapshot.stepIndex)) +
			presentationEpoch * 1000000;
		this.simulationTime = nativeSnapshot == null ? 0.0 : nativeSnapshot.simulationTime;
		this.robots = robots;
		this.environment = environment;
		if (nativeSnapshot != null) nativeSnapshot.dispose();
	}

}
