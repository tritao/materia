package cadkit.modeling;

import kinematicskit.FrameOrientation;
import kinematicskit.FrameTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.LevenbergMarquardt;
import kinematicskit.SolverWorkspace;
import kinematicskit.Transform;
import kinematicskit.Vector3;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadkit.modeling.AssemblyMateSolver.AssemblyMateSetup;

/** One update of a mate drag: whether the grabbed point reached its target, and if not how far off it stayed. */
class AssemblyMateDragResult {
	/** The grabbed point is on its target (within the assembly's position tolerance, scaled to the drag). */
	public final following:Bool;
	/** "Following", or what holds the part back, for the user. */
	public final message:String;
	/** Distance of the grabbed point from the target, in the assembly's length unit. */
	public final positionError:Float;

	public function new(following:Bool, message:String, positionError:Float) {
		this.following = following;
		this.message = message;
		this.positionError = positionError;
	}
}

/**
	Drags a point of a mated part towards world targets while every mate and
	closure holds (plan C4.5d), for the editor: a pin in a bore turns about its
	axis when pulled sideways, a part on a plane slides on it.

	Each update solves twice from the previous preview, both with
	Levenberg-Marquardt's minimal steps: first the mates and closures at their
	full (tolerance-scaled) weight with a light pull of the grabbed point
	towards the target, which finds the nearest placement the mates allow;
	then the mates and closures alone, which puts the preview exactly on them.
	The movable coordinates are the mate solver's (`AssemblyMateSolver.setup`):
	free roots and the joints between mated parts and their roots.
*/
class AssemblyMateDrag {
	public final occurrence:String;
	final definition:AssemblyDefinition;
	final start:AssemblyStateRecord;
	final setup:AssemblyMateSetup;
	final pull:KinematicProblem;
	final task:FrameTask;
	final snapshot:KinematicSnapshot;
	final workspace = new SolverWorkspace();
	var preview:KinematicState;

	/**
		`definition` with its mates (nested definitions are flattened, and `occurrence` is then a flattened id),
		`state` the configuration to start from, `grabPoint` the point that follows the targets, in the occurrence's
		own frame and the assembly's length unit. Throws when the mates cannot move the occurrence.
	*/
	public function new(definition:AssemblyDefinition, state:AssemblyStateRecord, occurrence:String, grabPoint:Vector3) {
		this.definition = definition;
		this.start = state;
		this.occurrence = occurrence;
		setup = AssemblyMateSolver.setup(definition, state);
		var kinematics = setup.kinematics, model = kinematics.model;
		var body = kinematics.body(occurrence);
		if (!moves(body)) throw 'The mates do not move "$occurrence"';
		preview = setup.seed.copy();
		snapshot = new KinematicSnapshot(model);
		snapshot.evaluate(preview);
		var offset = Transform.translation(grabPoint.x, grabPoint.y, grabPoint.z);
		task = new FrameTask(model, body, offset, snapshot.bodyPose(body).compose(offset), setup.settings.positionTolerance,
			setup.settings.angularTolerance, FrameTask.ALL_AXES, FrameOrientation.Free, occurrence);
		// A wish, not a constraint: the pull's rows are the distance (length unit), the mates' are divided by their
		// tolerance (a micrometre), so the weighted optimum bends a mate by about a tolerance-squared share of the
		// distance in any unit, and the projection that follows removes even that.
		task.positionWeight = 1.0;
		pull = new KinematicProblem(model).setActiveDofs(setup.dofs);
		for (id in setup.freeRoots) pull.setRootMotion(kinematics.body(id), kinematicskit.RootMotion.Floating);
		for (mateTask in setup.problem.tasks) pull.add(mateTask);
		pull.add(task);
	}

	/** Where the grabbed point is in the current preview (world, assembly length unit). */
	public function grabbedPoint():Vector3 {
		var pose = task.currentPose(snapshot);
		return new Vector3(pose.x, pose.y, pose.z);
	}

	/** An occurrence's world pose in the current preview (flattened id). */
	public function previewPose(occurrenceId:String):AssemblyFrame
		return AssemblyKinematics.toFrame(snapshot.bodyPose(setup.kinematics.body(occurrenceId)));

	/** Moves the grabbed point towards `target` (world, assembly length unit) as far as the mates let it. */
	public function drag(target:Vector3, ?maxIterations:Int = 40):AssemblyMateDragResult {
		task.setTarget(Transform.translation(target.x, target.y, target.z));
		var settings = setup.settings;
		var pulled = LevenbergMarquardt.solve(pull, preview, maxIterations, settings.initialDamping, settings.rankTolerance,
			setup.scale, workspace, true);
		var held = LevenbergMarquardt.solve(setup.problem, pulled.state, maxIterations, settings.initialDamping,
			settings.rankTolerance, setup.scale, workspace, true);
		if (held.converged()) {
			preview = held.state.copy();
			snapshot.evaluate(preview);
		}
		var point = grabbedPoint();
		var dx = point.x - target.x, dy = point.y - target.y, dz = point.z - target.z;
		var distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
		// On target to a thousandth of the assembly's size: the cursor's own precision, not the solver's.
		if (distance <= 1e-3 * setup.scale) return new AssemblyMateDragResult(true, "Following", distance);
		var unit = setup.flat.lengthUnit == null ? "mm" : setup.flat.lengthUnit;
		return new AssemblyMateDragResult(false, held.converged() ? 'Held by its mates ${Math.round(distance * 10) / 10} $unit from the cursor'
			: "The mates cannot hold here", distance);
	}

	/** The configuration with the preview applied (every coordinate the mates do not move kept): the record to store. */
	public function commit():AssemblyStateRecord {
		var placed = setup.placement(preview);
		return AssemblyMateSolver.merge(definition, start, placed.rootPoses, placed.jointCoordinates);
	}

	/** Whether a free root or a movable joint carries `body`. */
	function moves(body:Int):Bool {
		var kinematics = setup.kinematics, model = kinematics.model;
		for (id in setup.freeRoots) if (model.bodyRoot[body] == kinematics.body(id)) return true;
		for (joint in model.bodyChain[body])
			for (term in model.jointTermStart[joint]...model.jointTermStart[joint + 1])
				if (setup.dofs.indexOf(model.jointTermDof[term]) >= 0) return true;
		return false;
	}
}
