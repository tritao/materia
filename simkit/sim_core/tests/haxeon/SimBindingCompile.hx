import nativekit.scene.Scene;
import nativekit.sim.SimWorld;

/** Compile-only smoke for the thin typed Haxeon simulation façade. */
class SimBindingCompile {
    static function main():Int {
        var scene:Scene = null;
        var options:SimWorldOptions = {timestep: 0.001, physicsSubsteps: 2,
            gravity: [0.0, 0.0, -9.81]};
        // Keep the API reachable to the Haxeon compiler without constructing native state.
        if (scene == null && options.timestep == 0.0)
            return 1;
        return 42;
    }
}
