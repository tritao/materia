package cadkit.modeling;

import materia.assembly.AssemblyRecord;
import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyDefinitionCodec;
import materia.units.LengthUnit;
import haxeon.wire.JsonWire;

/** Records an assembly definition and solves its poses lazily through AssemblyState. */
class AssemblyModel {
	final data:AssemblyDefinition;
	final byId:Map<String, AssemblyComponentOccurrence> = [];
	final components:Map<String, AssemblyComponentDefinition> = [];
	final attached:Map<String, Bool> = [];
	var solved:Null<AssemblyState> = null;
	public final lengthUnit:String;
	public final metresPerUnit:Float;

	public function new(lengthUnit:String = "mm") {
		this.lengthUnit = lengthUnit;
		this.metresPerUnit = LengthUnit.metresPerUnit(lengthUnit);
		data = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "assembly", lengthUnit: lengthUnit,
			definitions: [], occurrences: [], joints: [], couplings: []};
	}

	public function add(id:String, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || byId.exists(id)) throw 'Duplicate assembly instance "$id"';
		var initialPose = pose == null ? AssemblyFrames.identity() : pose;
		AssemblyCodec.validateFrame(initialPose);
		var component:AssemblyComponentDefinition = {id: id, connectors: []};
		var occurrence:AssemblyComponentOccurrence = {id: id, definition: id, initialPose: initialPose};
		data.definitions.push(component);
		data.occurrences.push(occurrence);
		components.set(id, component);
		byId.set(id, occurrence);
		solved = null;
	}

	public function connector(id:String, name:String, frame:AssemblyFrame):Void {
		var component = requireComponent(id);
		if (name == null || name.length == 0) throw "Assembly connector needs a name";
		for (existing in component.connectors) if (existing.name == name)
			throw 'Duplicate connector "$id/$name"';
		AssemblyCodec.validateFrame(frame);
		component.connectors.push({name: name, frame: frame});
		solved = null;
	}

	public function pose(id:String):AssemblyFrame {
		require(id);
		return solvedState().worldPose(id);
	}

	public function worldPoint(id:String, connectorName:String):{x:Float, y:Float, z:Float} {
		var frame = requireConnector(id, connectorName);
		return AssemblyFrames.transformPoint(pose(id), frame.x, frame.y, frame.z);
	}

	public function place(id:String, pose:AssemblyFrame):Void {
		if (attached.exists(id)) throw 'Joint-owned instance "$id" cannot be placed independently';
		AssemblyCodec.validateFrame(pose);
		require(id).initialPose = pose;
		solved = null;
	}

	/**
		The child pose makes its named connector coincide with the parent joint frame. A joint that closes a loop
		becomes a closure instead (see `mateOnAxis`).
	*/
	public function mate(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0):Void {
		mateOnAxis(id, kind, parent, parentConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, value);
	}

	/**
		Creates a joint with an explicit unit axis in the parent connector frame: a tree joint, or a closure when
		the child already hangs from a joint or is an ancestor of the parent (the joint closes a loop). A closure
		has no coordinate of its own, so its value must be 0.
	*/
	public function mateOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, value:Float = 0,
			?limits:AssemblyJointLimits):Void {
		require(parent);
		require(child);
		if (attached.exists(child) || isAncestor(child, parent) || AssemblyDefinitionCodec.closureOnlyType(cast kind)) {
			if (value != 0) throw 'Joint "$id" closes a loop, so it has no coordinate of its own; its value must be 0';
			constrainOnAxis(id, kind, parent, parentConnector, child, childConnector, axis, null, limits);
			return;
		}
		if (!validJointKind(kind))
			throw 'Unsupported assembly joint "$kind"';
		if (!Math.isFinite(value)) throw "Assembly joint value must be finite";
		if (kind == "fixed" && value != 0) throw "Fixed assembly joints do not have a coordinate";
		validateAxis(axis);
		validateLimits(limits, value);
		requireConnector(parent, parentConnector);
		requireConnector(child, childConnector);
		attached.set(child, true);
		data.joints.push({id: id, type: cast kind, role: AssemblyJointRole.Tree,
			parent: parent, parentConnector: parentConnector, child: child,
			childConnector: childConnector, defaultValue: value, axis: axis,
			limits: limits == null ? noLimits() : limits});
		solved = null;
	}

	/**
		Marks a movable tree joint as an input of the mechanism (a motor, a cylinder): closure solves never move it,
		and the other movable joints on its loops follow it.
	*/
	public function drive(jointId:String, driven:Bool = true):Void {
		for (joint in data.joints)
			if (joint.id == jointId) {
				if (joint.role != AssemblyJointRole.Tree || !AssemblyDefinitionCodec.hasCoordinate(joint.type))
					throw 'Assembly joint "$jointId" is not a movable tree joint';
				if (driven) joint.driven = true; else Reflect.deleteField(joint, "driven");
				solved = null;
				return;
			}
		throw 'Missing assembly joint "$jointId"';
	}

	/** Records a closure; its residual is checked against solved poses on read. */
	public function constrain(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, ?tolerance:Float):Void {
		constrainOnAxis(id, kind, parent, parentConnector, child, childConnector,
			{x: 0, y: 1, z: 0}, tolerance);
	}

	/** Records a closure using its explicit unit axis in the parent connector frame. */
	public function constrainOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, ?tolerance:Float,
			?limits:AssemblyJointLimits):Void {
		if (!validJointKind(kind))
			throw 'Unsupported assembly joint "$kind"';
		validateAxis(axis);
		if (tolerance != null && (!Math.isFinite(tolerance) || tolerance < 0)) throw "Assembly tolerance must be finite and nonnegative";
		validateLimits(limits, 0);
		requireConnector(parent, parentConnector);
		requireConnector(child, childConnector);
		var joint:materia.assembly.AssemblyDefinition.KinematicJoint = {id: id, type: cast kind, role: AssemblyJointRole.Closure,
			parent: parent, parentConnector: parentConnector, child: child,
			childConnector: childConnector, defaultValue: 0, axis: axis,
			limits: limits == null ? noLimits() : limits};
		if (tolerance != null) joint.closureTolerance = tolerance;
		data.joints.push(joint);
		solved = null;
	}

	public function record():AssemblyRecord {
		var state = solvedState();
		var result:AssemblyRecord = {instances: [for (occurrence in data.occurrences) {
			id: occurrence.id, pose: state.worldPose(occurrence.id),
			connectors: requireComponent(occurrence.id).connectors.copy()
		}], joints: [for (joint in data.joints) {
			id: joint.id, kind: cast joint.type, parent: joint.parent,
			parentConnector: joint.parentConnector, child: joint.child,
			childConnector: joint.childConnector, value: joint.defaultValue
		}]};
		AssemblyCodec.validate(result);
		return result;
	}

	/**
	 * Couples a target coordinate to a source using target = source × ratio + offset. `efficiency`
	 * is the share of power the coupling passes on, such as a lead screw's; null means lossless.
	 * `stiffness` (force per unit of source travel), `backlash` (lost motion in the source's units)
	 * and `drag` (constant resisting effort at the target) describe a compliant or loose drive; null
	 * means rigid, tight and free-running.
	 */
	public function couple(id:String, source:String, target:String, ratio:Float, offset:Float = 0,
			?efficiency:Float, ?stiffness:Float, ?backlash:Float, ?drag:Float):Void {
		var coupling:materia.assembly.AssemblyDefinition.AssemblyJointCoupling = {id: id, source: source,
			target: target, ratio: ratio, offset: offset};
		if (efficiency != null) coupling.efficiency = efficiency;
		if (stiffness != null) coupling.stiffness = stiffness;
		if (backlash != null) coupling.backlash = backlash;
		if (drag != null) coupling.drag = drag;
		data.couplings.push(coupling);
		solved = null;
	}

	/** Puts a motor on joint `joint`: its usable effort and rate, in the joint's units, and rotor inertia. */
	public function actuate(id:String, joint:String, maxEffort:Float, maxRate:Float, ?rotorInertia:Float, ?fullStepsPerRevolution:Float):Void {
		if (data.actuators == null) data.actuators = [];
		var actuator:materia.assembly.AssemblyDefinition.AssemblyActuator = {id: id, joint: joint,
			maxEffort: maxEffort, maxRate: maxRate};
		if (rotorInertia != null) actuator.rotorInertia = rotorInertia;
		if (fullStepsPerRevolution != null) actuator.fullStepsPerRevolution = fullStepsPerRevolution;
		data.actuators.push(actuator);
	}

	/**
	 * Puts a motor on a joint with everything a drive records: its kind (stepper or servo), torque-speed
	 * curve, torques, speeds and encoder counts, as `AssemblyActuator` names them.
	 */
	public function actuateDrive(actuator:materia.assembly.AssemblyDefinition.AssemblyActuator):Void {
		if (data.actuators == null) data.actuators = [];
		data.actuators.push(materia.assembly.AssemblyDefinitionFlattener.copyActuator(actuator, actuator.id, actuator.joint));
	}

	/** Exports reusable definitions and explicit tree/closure semantics. */
	public function definition(id:String = "assembly"):AssemblyDefinition {
		var copy:AssemblyDefinition = JsonWire.decode(JsonWire.encode(data));
		copy.id = id;
		AssemblyDefinitionCodec.validate(copy);
		return copy;
	}

	public function initialState(?definitionId:String):AssemblyState
		return new AssemblyState(definition(definitionId == null ? data.id : definitionId));

	/** Returns an independent default solve after checking closure tolerances. */
	public function solve():AssemblyState {
		var state = solvedState();
		return new AssemblyState(state.definition, state.record());
	}

	function solvedState():AssemblyState {
		if (solved != null) return solved;
		var state = initialState();
		state.checkClosures();
		solved = state;
		return state;
	}

	/** Whether `ancestor` lies on `occurrence`'s tree path to its root (or is it). */
	function isAncestor(ancestor:String, occurrence:String):Bool {
		var current:Null<String> = occurrence;
		while (current != null) {
			if (current == ancestor) return true;
			var next:Null<String> = null;
			for (joint in data.joints) if (joint.role == AssemblyJointRole.Tree && joint.child == current) { next = joint.parent; break; }
			current = next;
		}
		return false;
	}

	function require(id:String):AssemblyComponentOccurrence {
		var instance = byId.get(id);
		if (instance == null) throw 'Missing assembly instance "$id"';
		return instance;
	}
	function requireComponent(id:String):AssemblyComponentDefinition {
		require(id);
		return components.get(id);
	}
	function requireConnector(id:String, name:String):AssemblyFrame {
		for (connector in requireComponent(id).connectors) if (connector.name == name) return connector.frame;
		throw 'Missing assembly connector "$id/$name"';
	}

	static function validJointKind(kind:String):Bool
		return kind == "fixed" || kind == "revolute" || kind == "continuous" || kind == "prismatic" ||
			AssemblyDefinitionCodec.closureOnlyType(cast kind);

	static function validateAxis(axis:AssemblyVector):Void {
		if (axis == null || !Math.isFinite(axis.x) || !Math.isFinite(axis.y) || !Math.isFinite(axis.z) ||
			Math.abs(Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z) - 1) > 1e-5)
			throw "Assembly joint axis must be a finite unit vector";
	}

	static function validateLimits(limits:Null<AssemblyJointLimits>, value:Float):Void {
		if (limits == null) return;
		if ((limits.lower != null && !Math.isFinite(limits.lower)) ||
			(limits.upper != null && !Math.isFinite(limits.upper)) ||
			(limits.velocity != null && (!Math.isFinite(limits.velocity) || limits.velocity < 0)) ||
			(limits.effort != null && (!Math.isFinite(limits.effort) || limits.effort < 0)) ||
			(limits.overtravel != null && (!Math.isFinite(limits.overtravel) || limits.overtravel < 0)) ||
			(limits.acceleration != null && (!Math.isFinite(limits.acceleration) || limits.acceleration < 0)) ||
			(limits.lower != null && limits.upper != null && limits.lower > limits.upper) ||
			(limits.lower != null && value < limits.lower) || (limits.upper != null && value > limits.upper))
			throw "Assembly joint limits or default value are invalid";
	}

	static function noLimits():AssemblyJointLimits
		return {lower: null, upper: null, velocity: null, effort: null};
}
