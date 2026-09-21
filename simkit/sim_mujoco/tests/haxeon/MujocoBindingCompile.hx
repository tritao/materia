import nativekit.scene.Scene;
import nativekit.sim.Joint;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimWorld;

/** Compile-only smoke for the engine-neutral MuJoCo Haxeon façade. */
class MujocoBindingCompile {
    static function main():Int {
        var scene:Scene = null;
        var options:SimWorldOptions = {timestep: 0.001, physicsSubsteps: 2,
            gravity: [0.0, 0.0, -9.81]};
        var world:MujocoSimWorld = null;
        var joint:Joint = null;
        if (scene == null && options.timestep == 0.0 && world == null && joint == null)
            return 1;
        return 42;
    }
}
