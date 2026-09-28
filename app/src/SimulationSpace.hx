package app;

import nativekit.scene.Scene;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimSession;
import nativekit.sim.SimWorld;

/**
 * The scene, physics world, and SimKit session one application simulation
 * runs in. Robots, environment props, and people all join the session.
 */
class SimulationSpace {
	public final session:SimSession;
	final scene:Scene;
	final releaseWorld:Void->Void;
	var disposed:Bool = false;

	function new(scene:Scene, session:SimSession, releaseWorld:Void->Void) {
		this.scene = scene;
		this.session = session;
		this.releaseWorld = releaseWorld;
	}

	/** Creates a space on the deterministic test backend or on MuJoCo. */
	public static function create(backend:Int, timestep:Float):SimulationSpace {
		var scene = Scene.create();
		try {
			if (backend == ApplicationSimulation.MUJOCO) {
				var world = MujocoSimWorld.create(scene, {timestep: timestep, physicsSubsteps: 1});
				return new SimulationSpace(scene, new SimSession(scene, world.nativeHandle()), world.dispose);
			}
			var world = new SimWorld(scene, {timestep: timestep, physicsSubsteps: 1});
			return new SimulationSpace(scene, new SimSession(scene, world.nativeHandle()), world.dispose);
		} catch (error:Dynamic) {
			scene.dispose();
			throw error;
		}
	}

	/** Releases the session, then the world it borrowed, then the scene. */
	public function dispose():Void {
		if (disposed)
			return;
		disposed = true;
		session.dispose();
		releaseWorld();
		scene.dispose();
	}
}
