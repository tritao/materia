package robotkit.runtime;

import nativekit.scene.Scene;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimSession;
import nativekit.sim.SimWorld;

/**
 * The scene, physics world, and SimKit session one simulation runs in.
 * Robots (through Simulation.inSession()), environment props, and people all
 * join the session. Shared by the app and by anything else that needs to own
 * a session end to end, such as a test.
 */
class SimulationSpace {
	/** The deterministic, dependency-free test backend. */
	public static inline var DETERMINISTIC:Int = 0;
	/** The MuJoCo backend; must be built in. */
	public static inline var MUJOCO:Int = 1;

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
			if (backend == MUJOCO) {
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
