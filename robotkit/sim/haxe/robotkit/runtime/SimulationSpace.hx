package robotkit.runtime;

import NativeKitSim;
import nativekit.scene.Scene;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimSession;
import nativekit.sim.SimWorld;
import nativekit.sim.SimWorldOptions;

/**
 * The scene, physics world, and SimKit session one simulation runs in.
 * Robots (through Simulation.inSpace()), environment props, and people all
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
	final configureWorld:(Int, Int)->Void;
	var requiredSubsteps:Int;
	final timestep:Float;
	var disposed:Bool = false;

	function new(scene:Scene, session:SimSession, releaseWorld:Void->Void,
		configureWorld:(Int, Int)->Void, timestep:Float, substeps:Int) {
		this.scene = scene;
		this.session = session;
		this.releaseWorld = releaseWorld;
		this.configureWorld = configureWorld;
		this.timestep = timestep;
		this.requiredSubsteps = substeps;
	}

	/**
	 * Creates a space on the deterministic test backend or on MuJoCo. Physics
	 * advances `physicsSubsteps` times per tick; `solver` picks the integrator,
	 * friction cone, and solver iteration limits (unset keeps backend defaults).
	 */
	public static function create(backend:Int, timestep:Float, physicsSubsteps:Int = 1,
			?solver:SimWorldOptions):SimulationSpace {
		var options:SimWorldOptions = {timestep: timestep, physicsSubsteps: physicsSubsteps};
		if (solver != null) {
			options.integrator = solver.integrator;
			options.frictionCone = solver.frictionCone;
			options.solverIterations = solver.solverIterations;
			options.lineSearchIterations = solver.lineSearchIterations;
		}
		var scene = Scene.create();
		try {
			if (backend == MUJOCO) {
				var world = MujocoSimWorld.create(scene, options);
				return new SimulationSpace(scene, new SimSession(scene, world.nativeHandle()), world.dispose,
					world.configureIntegration, timestep, physicsSubsteps);
			}
			var world = new SimWorld(scene, options);
			return new SimulationSpace(scene, new SimSession(scene, world.nativeHandle()), world.dispose,
					world.configureIntegration, timestep, physicsSubsteps);
		} catch (error:Dynamic) {
			scene.dispose();
			throw error;
		}
	}

	/** Derives integration from the drive loops and their mechanical loads. */
	public function requireDrives(blueprint:RobotRuntimeBlueprint):Void {
		var rate = blueprint.fastestPositionLoopRate();
		if (rate <= 0) return;
		var interval = Math.min(1.0 / rate, blueprint.servoStabilityInterval());
		requiredSubsteps = Std.int(Math.max(requiredSubsteps, Math.ceil(timestep / interval)));
		configureWorld(requiredSubsteps, NativeKitSimConstants.NKSIM_INTEGRATOR_IMPLICIT_FAST);
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
