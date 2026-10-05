import nativekit.scene.Scene;
import nativekit.sim.SimWorld;
import nativekit.sim.SimWorldOptions;
import nativekit.sim.Joint;

/** Compile-only smoke for the thin typed Haxeon simulation façade. */
class SimBindingCompile {
    static function brakeApi(joint:Joint):Bool {
        joint.configureBrake(4.0, 0.05, 0.03);
        joint.setBrakeRelease(true);
        return joint.brakeReleased();
    }

    static function main():Int {
        var scene:Scene = null;
        var options:SimWorldOptions = {timestep: 0.001, physicsSubsteps: 2,
            gravity: [0.0, 0.0, -9.81]};
        // Keep the API reachable to the Haxeon compiler without constructing native state.
        if (scene == null && options.timestep == 0.0)
            return brakeApi(null) ? 1 : 0;
        return 42;
    }
}
