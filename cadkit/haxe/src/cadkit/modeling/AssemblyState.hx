package cadkit.modeling;

import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyRootPose;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.units.LengthUnit;
import cadkit.modeling.AssemblyLoopSolver.AssemblyLoopSolveOptions;
import cadkit.modeling.AssemblyLoopSolver.AssemblyLoopSolveResult;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;

typedef AssemblyClosureResidual = {
	var joint:String;
	var position:Float;
	var axis:Float;
	var rotation:Float;
}

/** Mutable configuration and derived world poses for an immutable assembly definition. */
class AssemblyState {
	public final definition:AssemblyDefinition;
	final coordinates:Map<String, Float> = [];
	final rootPoses:Map<String, AssemblyFrame> = [];
	final occurrences:Map<String, AssemblyComponentOccurrence> = [];
	final components:Map<String, AssemblyComponentDefinition> = [];
	final joints:Map<String, KinematicJoint> = [];
	final poses:Map<String, AssemblyFrame> = [];
	final kinematics:AssemblyKinematics;
	final kinematicState:KinematicState;
	final snapshot:KinematicSnapshot;
	var dirty:Bool = true;

	public function new(definition:AssemblyDefinition, ?state:AssemblyStateRecord) {
		AssemblyDefinitionCodec.validate(definition);
		if (state != null) state = AssemblyDefinitionFlattener.flattenState(definition, state);
		definition = AssemblyDefinitionFlattener.flatten(definition);
		this.definition = definition;
		for (component in definition.definitions) components.set(component.id, component);
		for (occurrence in definition.occurrences) occurrences.set(occurrence.id, occurrence);
		for (joint in definition.joints) {
			joints.set(joint.id, joint);
			if (joint.role == AssemblyJointRole.Tree) {
				if (AssemblyDefinitionCodec.hasCoordinate(joint.type))
					coordinates.set(joint.id, joint.defaultValue);
			}
		}
		kinematics = AssemblyKinematics.compile(definition);
		kinematicState = new KinematicState(kinematics.model);
		snapshot = new KinematicSnapshot(kinematics.model);
		if (state != null) {
			AssemblyDefinitionCodec.validateState(definition, state);
			for (coordinate in state.jointCoordinates) coordinates.set(coordinate.joint, coordinate.value);
			for (root in state.rootPoses) rootPoses.set(root.occurrence, root.pose);
		}
	}

	public function setJoint(id:String, value:Float):Void {
		var joint = joints.get(id);
		if (joint == null || joint.role != AssemblyJointRole.Tree ||
			!AssemblyDefinitionCodec.hasCoordinate(joint.type))
			throw 'Joint "$id" is not a driving tree joint';
		if (!Math.isFinite(value) || (joint.limits.lower != null && value < joint.limits.lower) ||
			(joint.limits.upper != null && value > joint.limits.upper))
			throw 'Joint "$id" coordinate is outside its limits';
		var couplings = definition.couplings == null ? [] : definition.couplings;
		for (coupling in couplings) if (coupling.target == id)
			throw 'Joint "$id" is driven by coupled joint "${coupling.source}"';
		var updates = new Map<String, Float>();
		collectCoupledValues(id, value, updates);
		for (jointId in updates.keys()) coordinates.set(jointId, updates.get(jointId));
		dirty = true;
	}

	function collectCoupledValues(id:String, value:Float, updates:Map<String, Float>):Void {
		var joint = joints.get(id);
		if (joint == null || !Math.isFinite(value) ||
			(joint.limits.lower != null && value < joint.limits.lower) ||
			(joint.limits.upper != null && value > joint.limits.upper))
			throw 'Coupled joint "$id" coordinate is outside its limits';
		updates.set(id, value);
		if (definition.couplings != null) for (coupling in definition.couplings)
			if (coupling.source == id)
				collectCoupledValues(coupling.target, value * coupling.ratio + coupling.offset, updates);
	}

	public function joint(id:String):Float {
		var joint = joints.get(id);
		if (joint == null || joint.role != AssemblyJointRole.Tree ||
			!AssemblyDefinitionCodec.hasCoordinate(joint.type))
			throw 'Joint "$id" is not a driving tree joint';
		var value = coordinates.get(id);
		if (value == null) throw 'Missing coordinate for joint "$id"';
		return value;
	}

	public function setRootPose(id:String, pose:AssemblyFrame):Void {
		if (!AssemblyDefinitionCodec.rootOccurrences(definition).exists(id))
			throw 'Occurrence "$id" is not a kinematic root';
		AssemblyCodec.validateFrame(pose);
		rootPoses.set(id, pose);
		dirty = true;
	}

	/**
		Recomputes all occurrence poses from root placements and tree-joint
		coordinates. A coupled joint follows its source through the compiled
		coupling, so its pose never depends on a stale stored coordinate.
	*/
	public function forwardKinematics():Void {
		poses.clear();
		syncKinematicState();
		snapshot.evaluate(kinematicState);
		dirty = false;
	}

	/** The compiled model and a copy of this configuration in its terms, for kinematics solvers. */
	@:allow(cadkit.modeling.AssemblyLoopSolver)
	@:allow(cadkit.modeling.AssemblyDrag)
	function kinematicModel():AssemblyKinematics return kinematics;

	@:allow(cadkit.modeling.AssemblyLoopSolver)
	@:allow(cadkit.modeling.AssemblyDrag)
	function kinematicSeed():KinematicState {
		syncKinematicState();
		return kinematicState.copy();
	}

	function syncKinematicState():Void {
		var model = kinematics.model;
		for (dof in 0...model.dofCount()) {
			var value = coordinates.get(model.dofId(dof));
			kinematicState.q[dof] = value == null ? model.jointDefault[model.dofJoint[dof]] : value;
		}
		for (id in rootPoses.keys()) kinematicState.setRootPose(kinematics.body(id), AssemblyKinematics.fromFrame(rootPoses.get(id)));
	}

	public function worldPose(id:String):AssemblyFrame {
		if (!occurrences.exists(id)) throw 'Missing assembly occurrence "$id"';
		if (dirty) forwardKinematics();
		var pose = poses.get(id);
		if (pose == null) {
			pose = AssemblyKinematics.toFrame(snapshot.bodyPose(kinematics.body(id)));
			poses.set(id, pose);
		}
		return pose;
	}

	public function worldConnector(id:String, name:String):AssemblyFrame {
		var occurrence = occurrences.get(id);
		if (occurrence == null) throw 'Missing assembly occurrence "$id"';
		var component = components.get(occurrence.definition);
		if (component == null) throw 'Missing assembly component "${occurrence.definition}"';
		for (connector in component.connectors) if (connector.name == name)
			return AssemblyFrames.compose(worldPose(id), connector.frame);
		throw 'Missing assembly connector "$id/$name"';
	}

	/** Closure checks are diagnostic only; dependent coordinates need a loop solver. */
	public function closureResiduals():Array<AssemblyClosureResidual> {
		if (dirty) forwardKinematics();
		var result:Array<AssemblyClosureResidual> = [];
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Closure) {
			var first = worldConnector(joint.parent, joint.parentConnector);
			var second = worldConnector(joint.child, joint.childConnector);
			var dx = second.x - first.x, dy = second.y - first.y, dz = second.z - first.z;
			var axisFirst = AssemblyFrames.transformVector(first, joint.axis.x, joint.axis.y, joint.axis.z);
			var axisSecond = AssemblyFrames.transformVector(second, joint.axis.x, joint.axis.y, joint.axis.z);
			var axisDot = Math.abs(axisFirst.x * axisSecond.x + axisFirst.y * axisSecond.y + axisFirst.z * axisSecond.z);
			var position:Float;
			if (joint.type == AssemblyJointType.Prismatic) {
				var along = dx * axisFirst.x + dy * axisFirst.y + dz * axisFirst.z;
				var px = dx - along * axisFirst.x, py = dy - along * axisFirst.y, pz = dz - along * axisFirst.z;
				position = Math.sqrt(px * px + py * py + pz * pz);
			} else position = Math.sqrt(dx * dx + dy * dy + dz * dz);
			var rotation = 0.0;
			if (joint.type == AssemblyJointType.Fixed) {
				var dot = first.qx * second.qx + first.qy * second.qy + first.qz * second.qz + first.qw * second.qw;
				rotation = 1 - Math.abs(dot);
			}
			result.push({joint: joint.id, position: position, axis: 1 - axisDot, rotation: rotation});
		}
		return result;
	}

	/** Check closures using a one-micrometre default in the definition's length unit. */
	public function checkClosures():Void {
		for (residual in closureResiduals()) {
			var joint = joints.get(residual.joint);
			if (joint == null) throw 'Missing assembly joint "${residual.joint}"';
			var tolerance = joint.closureTolerance;
			if (tolerance == null)
				tolerance = 1e-6 / LengthUnit.metresPerUnit(definition.lengthUnit == null ? "mm" : definition.lengthUnit);
			if (residual.position > tolerance)
				throw 'Assembly joint "${residual.joint}" has separated connectors';
			if (joint.type == AssemblyJointType.Fixed) {
				if (residual.rotation > 1e-5)
					throw 'Assembly fixed joint "${residual.joint}" has misaligned frames';
			} else if (residual.axis > 1e-5)
				throw 'Assembly joint "${residual.joint}" has misaligned axes';
		}
	}

	/**
		Adjusts tree-joint coordinates until the assembly closures are satisfied: the named ones, or by default
		`dependentJoints()`.
	*/
	public function solveClosures(?dependentJointIds:Array<String>,
		?options:AssemblyLoopSolveOptions):AssemblyLoopSolveResult
		return AssemblyLoopSolver.solve(this, dependentJointIds == null ? dependentJoints() : dependentJointIds, options);

	/**
		The coordinates closures depend on: every movable tree joint on the tree path between a closure's two
		occurrences that is neither driven nor a coupling target (a coupled joint follows its source). In
		definition order.
	*/
	public function dependentJoints():Array<String> {
		var parentJoint = new Map<String, KinematicJoint>();
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree) parentJoint.set(joint.child, joint);
		var coupled = new Map<String, Bool>();
		if (definition.couplings != null) for (coupling in definition.couplings) coupled.set(coupling.target, true);
		var onLoop = new Map<String, Bool>();
		for (closure in definition.joints) if (closure.role == AssemblyJointRole.Closure) {
			// Occurrences from each end up to its root, then the joints below their lowest common ancestor.
			var fromParent = pathToRoot(closure.parent, parentJoint), fromChild = pathToRoot(closure.child, parentJoint);
			var common = new Map<String, Bool>();
			for (step in fromParent) common.set(step.occurrence, true);
			var meet:Null<String> = null;
			for (step in fromChild) if (common.exists(step.occurrence)) { meet = step.occurrence; break; }
			for (path in [fromParent, fromChild])
				for (step in path) {
					if (step.occurrence == meet) break;
					if (step.joint != null) onLoop.set(step.joint.id, true);
				}
		}
		return [for (joint in definition.joints)
			if (onLoop.exists(joint.id) && AssemblyDefinitionCodec.hasCoordinate(joint.type) && joint.driven != true && !coupled.exists(joint.id))
				joint.id];
	}

	/** Each occurrence from `start` to its root, with the tree joint that attaches it to the next (null at the root). */
	static function pathToRoot(start:String, parentJoint:Map<String, KinematicJoint>):Array<{occurrence:String, joint:Null<KinematicJoint>}> {
		var path:Array<{occurrence:String, joint:Null<KinematicJoint>}> = [];
		var current:Null<String> = start;
		while (current != null) {
			var joint = parentJoint.get(current);
			path.push({occurrence: current, joint: joint});
			current = joint == null ? null : joint.parent;
		}
		return path;
	}

	public function record():AssemblyStateRecord {
		var values:Array<AssemblyJointCoordinate> = [];
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree &&
			AssemblyDefinitionCodec.hasCoordinate(joint.type)) {
			var value = coordinates.get(joint.id);
			if (value == null) throw 'Missing coordinate for joint "${joint.id}"';
			values.push({joint: joint.id, value: value});
		}
		var roots:Array<AssemblyRootPose> = [];
		for (occurrence in definition.occurrences) if (rootPoses.exists(occurrence.id))
			roots.push({occurrence: occurrence.id, pose: rootPoses.get(occurrence.id)});
		var result:AssemblyStateRecord = {schemaVersion: AssemblyDefinitionCodec.VERSION,
			definition: definition.id, jointCoordinates: values, rootPoses: roots};
		AssemblyDefinitionCodec.validateState(definition, result);
		return result;
	}
}
