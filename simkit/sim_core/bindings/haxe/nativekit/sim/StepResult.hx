package nativekit.sim;

import NativeKitSim;

/** Result of one deterministic fixed-timestep simulation tick. */
class StepResult {
    public final stepIndex:haxe.Int64;
    public final simulationTime:Float;
    public final physicsSubsteps:Int;
    public final sceneChanges:SceneChanges;

    @:allow(SimWorld)
    private function new(value:nksim_step_result) {
        stepIndex = value.get_step_index();
        simulationTime = value.get_simulation_time();
        physicsSubsteps = value.get_physics_substeps();
        sceneChanges = new SceneChanges(value.get_scene_changes());
    }
}
