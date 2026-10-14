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
	/** Backend selected when this space was created; sensor contracts use it to reject unsupported observations. */
	public final backend:Int;
	final scene:Scene;
	final releaseWorld:Void->Void;
	final configureWorld:(Int, Int)->Void;
	var requiredSubsteps:Int;
	var integrator:Int;
	final timestep:Float;
	var disposed:Bool = false;

	function new(scene:Scene, session:SimSession, releaseWorld:Void->Void,
		configureWorld:(Int, Int)->Void, timestep:Float, substeps:Int, backend:Int, integrator:Int) {
		this.scene = scene;
		this.session = session;
		this.releaseWorld = releaseWorld;
		this.configureWorld = configureWorld;
		this.timestep = timestep;
		this.requiredSubsteps = substeps;
		this.backend = backend;
		this.integrator = integrator;
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
		var integrator = options.integrator == null ? NativeKitSimConstants.NKSIM_INTEGRATOR_DEFAULT : options.integrator;
		try {
			if (backend == MUJOCO) {
				var world = MujocoSimWorld.create(scene, options);
				return new SimulationSpace(scene, new SimSession(scene, world.nativeHandle()), world.dispose,
					world.configureIntegration, timestep, physicsSubsteps, backend, integrator);
			}
			var world = new SimWorld(scene, options);
			return new SimulationSpace(scene, new SimSession(scene, world.nativeHandle()), world.dispose,
					world.configureIntegration, timestep, physicsSubsteps, backend, integrator);
		} catch (error:Dynamic) {
			scene.dispose();
			throw error;
		}
	}

	/** Derives integration from the drive loops and their mechanical loads. */
	public function requireDrives(blueprint:RobotRuntimeBlueprint):Void {
		var rate = blueprint.fastestPositionLoopRate();
		// A servo that drives a compliant coupling integrates its spring against its own inertia, whatever position loop
		// the drive states.
		var interval = Math.min(blueprint.pneumaticStabilityInterval(), blueprint.servoStabilityInterval(true));
		if (rate > 0.0) interval = Math.min(interval,
			Math.min(1.0 / rate, blueprint.servoStabilityInterval()));
		requireInterval(interval);
	}

	/** Steps the world at least this finely (a spring integrated explicitly, a stiff cushion), and implicitly. */
	public function requireInterval(interval:Float):Void {
		if (!Math.isFinite(interval) || !(interval > 0.0)) return;
		requiredSubsteps = Std.int(Math.max(requiredSubsteps, Math.ceil(timestep / interval)));
		integrator = NativeKitSimConstants.NKSIM_INTEGRATOR_IMPLICIT_FAST;
		configureWorld(requiredSubsteps, integrator);
	}

	/** Resolves authored positive contact time constants without MuJoCo's two-step safety floor changing them. */
	public function requireContact(surface:robotkit.model.CollisionShape.ContactSurface):Void {
		var issue = surface.validate();
		if (issue != null) throw 'Simulation contact: $issue';
		requireConstraintInterval(surface.contactTimeConstant);
	}

	/** Joint stops have the same two-step resolution requirement as shape contacts. */
	public function requireStops(blueprint:RobotRuntimeBlueprint):Void {
		for (joint in blueprint.joints) requireConstraintInterval(joint.limitTimeConstant);
	}

	function requireConstraintInterval(timeConstant:Float):Void {
		if (!Math.isFinite(timeConstant) || timeConstant < 0)
			throw "Simulation constraint time constant must be finite and non-negative";
		if (timeConstant == 0) return;
		var requested = 2.0 * timestep / timeConstant;
		if (!Math.isFinite(requested) || requested > 2147483647.0)
			throw "Simulation constraint time constant needs an unrepresentable number of physics substeps";
		var substeps = Std.int(Math.ceil(requested));
		if (substeps <= requiredSubsteps) return;
		requiredSubsteps = substeps;
		configureWorld(requiredSubsteps, integrator);
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
