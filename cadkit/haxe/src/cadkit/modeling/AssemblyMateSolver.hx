package cadkit.modeling;

import cadkit.solve.ConstraintDiagnosis;
import kinematicskit.ClosureTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicState;
import kinematicskit.KinematicStatus;
import kinematicskit.LevenbergMarquardt;
import kinematicskit.RootMotion;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyRootPose;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.units.LengthUnit;

typedef AssemblyMateSolveOptions = {
	@:optional var positionTolerance:Float;
	@:optional var angularTolerance:Float;
	@:optional var maxIterations:Int;
}

/** Outcome of placing an assembly's parts by its mates (see `AssemblyMateSolver`). */
class AssemblyMateSolveResult {
	public final status:String;
	public final converged:Bool;
	public final iterations:Int;
	/** Mates and closures outside tolerance; empty on success. */
	public final unsatisfied:Array<String>;
	public final message:String;
	/** The mates' and closures' diagnosis: what is still free, redundant or conflicting (owners are their IDs). */
	public final report:DiagnosisReport;
	/** Occurrences the solve could move as whole parts, in definition order. */
	public final freeRoots:Array<String>;
	/** Solved poses of the free roots and coordinates of the joints the mates reached. */
	public final rootPoses:Array<AssemblyRootPose>;
	public final jointCoordinates:Array<AssemblyJointCoordinate>;

	public function new(status:String, converged:Bool, iterations:Int, unsatisfied:Array<String>, message:String,
			report:DiagnosisReport, freeRoots:Array<String>, rootPoses:Array<AssemblyRootPose>,
			jointCoordinates:Array<AssemblyJointCoordinate>) {
		this.status = status;
		this.converged = converged;
		this.iterations = iterations;
		this.unsatisfied = unsatisfied;
		this.message = message;
		this.report = report;
		this.freeRoots = freeRoots;
		this.rootPoses = rootPoses;
		this.jointCoordinates = jointCoordinates;
	}

	/** The solved placement as a configuration of `definition` (its other coordinates stay at their defaults). */
	public function state(definition:AssemblyDefinition):AssemblyStateRecord {
		var record:AssemblyStateRecord = {schemaVersion: AssemblyDefinitionCodec.VERSION, definition: definition.id,
			jointCoordinates: jointCoordinates.copy(), rootPoses: rootPoses.copy()};
		AssemblyDefinitionCodec.validateState(definition, record);
		return record;
	}
}

/**
	Places parts by their mates (plan C4.3). Every root occurrence except the
	grounded ones (or, when none is grounded, the first root) is a free rigid
	body; the movable joints between a mated occurrence and its root that are
	neither driven nor coupled may move too. Mates and joint closures are all
	equalities, solved by Levenberg-Marquardt in its Levenberg mode (the
	problem is usually under-determined: mates rarely fix everything), then
	diagnosed: the report's degrees of freedom are what the mates leave free.
*/
class AssemblyMateSolver {
	static inline var DEFAULT_POSITION_TOLERANCE:Float = 1e-3; // millimetres
	static inline var DEFAULT_ANGULAR_TOLERANCE:Float = 1e-6;
	static inline var DEFAULT_MAX_ITERATIONS:Int = 200;

	public static function solve(definition:AssemblyDefinition, ?state:AssemblyStateRecord,
			?options:AssemblyMateSolveOptions):AssemblyMateSolveResult {
		AssemblyDefinitionCodec.validate(definition);
		if (state != null) AssemblyDefinitionCodec.validateState(definition, state);
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var flatState = state == null ? null : AssemblyDefinitionFlattener.flattenState(definition, state);
		var kinematics = AssemblyKinematics.compile(flat, true), model = kinematics.model;
		var metresPerUnit = LengthUnit.metresPerUnit(flat.lengthUnit == null ? "mm" : flat.lengthUnit);
		var positionTolerance = options != null && options.positionTolerance != null ? options.positionTolerance
			: DEFAULT_POSITION_TOLERANCE * 0.001 / metresPerUnit;
		var angularTolerance = options != null && options.angularTolerance != null ? options.angularTolerance : DEFAULT_ANGULAR_TOLERANCE;
		var maxIterations = options != null && options.maxIterations != null ? options.maxIterations : DEFAULT_MAX_ITERATIONS;

		// What may move: free roots as rigid bodies, and the joints between mated occurrences and their roots.
		var roots = AssemblyDefinitionCodec.rootOccurrences(flat);
		var rootIds = [for (occurrence in flat.occurrences) if (roots.exists(occurrence.id)) occurrence];
		var grounded = [for (occurrence in rootIds) if (occurrence.grounded == true) occurrence.id];
		if (grounded.length == 0 && rootIds.length > 0) grounded.push(rootIds[0].id);
		var freeRoots = [for (occurrence in rootIds) if (grounded.indexOf(occurrence.id) < 0) occurrence.id];
		var dofs = reachedDofs(flat, model);

		var seed = new KinematicState(model);
		if (flatState != null) {
			for (coordinate in flatState.jointCoordinates) {
				var dof = model.dofIndex(coordinate.joint);
				if (dof >= 0) seed.q[dof] = coordinate.value;
			}
			for (root in flatState.rootPoses) seed.setRootPose(kinematics.body(root.occurrence), AssemblyKinematics.fromFrame(root.pose));
		}
		var problem = new KinematicProblem(model).setActiveDofs(dofs);
		for (id in freeRoots) problem.setRootMotion(kinematics.body(id), RootMotion.Floating);
		for (closure in 0...model.closureCount())
			problem.add(new ClosureTask(model, closure, positionTolerance, angularTolerance));

		var scale = assemblyScale(flat);
		var solution = LevenbergMarquardt.solve(problem, seed, maxIterations, 1e-3, 1e-8, scale, null, true);
		var report = AssemblyClosureDiagnosis.diagnose(problem, solution.state);
		var status = solution.converged() ? "converged" : switch solution.status {
			case KinematicStatus.LimitBlocked: "limit-blocked";
			case KinematicStatus.Conflicting: "conflicting";
			default: "nonconvergent";
		};
		var message = switch status {
			case "converged": report.degreesOfFreedom == 0 ? "the mates place every part" : 'the mates leave ${report.degreesOfFreedom} degrees of freedom';
			case "conflicting": "the mates cannot all hold; see the report's conflicting groups";
			case "limit-blocked": "joint limits stop the mates from closing";
			default: "the mate solve ran out of iterations while still improving";
		};
		var rootPoses = [for (id in freeRoots) {occurrence: id, pose: AssemblyKinematics.toFrame(solution.state.rootPose(kinematics.body(id)))}];
		var coordinates = [for (dof in dofs) {joint: model.dofId(dof), value: solution.state.q[dof]}];
		return new AssemblyMateSolveResult(status, solution.converged(), solution.iterations, solution.unsatisfied(), message,
			report, freeRoots, rootPoses, coordinates);
	}

	/**
		`definition` with each solved free root's initial pose and each reached
		joint's default value replaced by the solve: the design-time placement.
		Only for definitions without nested assemblies (a nested member's pose
		is not an occurrence of the root definition).
	*/
	public static function place(definition:AssemblyDefinition, result:AssemblyMateSolveResult):AssemblyDefinition {
		if (definition.assemblies != null && definition.assemblies.length > 0)
			throw "Placing mates into a definition with nested assemblies is not supported; use the solve's state instead";
		var copy = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(definition));
		for (root in result.rootPoses)
			for (occurrence in copy.occurrences) if (occurrence.id == root.occurrence) occurrence.initialPose = root.pose;
		for (coordinate in result.jointCoordinates)
			for (joint in copy.joints) if (joint.id == coordinate.joint) joint.defaultValue = coordinate.value;
		AssemblyDefinitionCodec.validate(copy);
		return copy;
	}

	/** Movable tree joints between each mate's occurrences and their roots that are neither driven nor coupled. */
	static function reachedDofs(flat:AssemblyDefinition, model:kinematicskit.KinematicModel):Array<Int> {
		var parentJoint = new Map<String, KinematicJoint>();
		for (joint in flat.joints) if (joint.role == AssemblyJointRole.Tree) parentJoint.set(joint.child, joint);
		var coupled = new Map<String, Bool>();
		if (flat.couplings != null) for (coupling in flat.couplings) coupled.set(coupling.target, true);
		var reached = new Map<String, Bool>();
		if (flat.mates != null) for (mate in flat.mates)
			for (start in [mate.first, mate.second]) {
				var current:Null<String> = start;
				while (current != null) {
					var joint = parentJoint.get(current);
					if (joint == null) break;
					reached.set(joint.id, true);
					current = joint.parent;
				}
			}
		var dofs:Array<Int> = [];
		for (joint in flat.joints)
			if (reached.exists(joint.id) && AssemblyDefinitionCodec.hasCoordinate(joint.type) && joint.driven != true &&
				!coupled.exists(joint.id) && model.dofIndex(joint.id) >= 0)
				dofs.push(model.dofIndex(joint.id));
		return dofs;
	}

	/** The extent of occurrence origins and connectors: prismatic and root translations are solved in this unit. */
	static function assemblyScale(flat:AssemblyDefinition):Float {
		var extent = 0.0;
		var components = new Map<String, AssemblyComponentDefinition>();
		for (component in flat.definitions) components.set(component.id, component);
		for (occurrence in flat.occurrences) {
			extent = Math.max(extent, Math.sqrt(occurrence.initialPose.x * occurrence.initialPose.x +
				occurrence.initialPose.y * occurrence.initialPose.y + occurrence.initialPose.z * occurrence.initialPose.z));
			var component = components.get(occurrence.definition);
			if (component != null) for (connector in component.connectors)
				extent = Math.max(extent, Math.sqrt(connector.frame.x * connector.frame.x + connector.frame.y * connector.frame.y +
					connector.frame.z * connector.frame.z));
		}
		return Math.max(1, extent);
	}
}
