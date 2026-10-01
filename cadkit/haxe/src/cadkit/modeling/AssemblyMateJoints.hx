package cadkit.modeling;

import kinematicskit.ClosureTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.RootMotion;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyConnector;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** The joint a part's mates amount to (see `AssemblyMateJoints.infer`), or why they amount to none. */
class AssemblyMateJoint {
	/** Revolute or Prismatic; null when the mates are not one such joint (`reason` says why). */
	public final type:Null<AssemblyJointType>;
	public final reason:String;
	/** The part the mates hold it to (the joint's parent), and the part itself (its child). */
	public final parent:String;
	public final child:String;
	/** The mates the joint replaces. */
	public final mates:Array<String>;
	/** A point on the joint's axis and its unit direction (world, assembly length unit). */
	public final point:Array<Float>;
	public final axis:Array<Float>;

	public function new(type:Null<AssemblyJointType>, reason:String, parent:String, child:String, mates:Array<String>,
			point:Array<Float>, axis:Array<Float>) {
		this.type = type;
		this.reason = reason;
		this.parent = parent;
		this.child = child;
		this.mates = mates;
		this.point = point;
		this.axis = axis;
	}
}

/** A joint made from mates: the joint, and the connectors it needs on its parent's and child's components. */
class AssemblyMateJointConversion {
	public final joint:KinematicJoint;
	public final parentComponent:String;
	public final parentConnector:AssemblyConnector;
	public final childComponent:String;
	public final childConnector:AssemblyConnector;
	public final mates:Array<String>;

	public function new(joint:KinematicJoint, parentComponent:String, parentConnector:AssemblyConnector, childComponent:String,
			childConnector:AssemblyConnector, mates:Array<String>) {
		this.joint = joint;
		this.parentComponent = parentComponent;
		this.parentConnector = parentConnector;
		this.childComponent = childComponent;
		this.childConnector = childConnector;
		this.mates = mates;
	}
}

/**
	Turns a free part's mates into the joint they amount to (plan C4.5e): when
	the mates hold the part to one other part and leave it exactly one motion,
	a pure turn makes a revolute joint about that axis and a pure slide a
	prismatic one. The motion is the null space of the mates' rows over the
	part's six rigid-body columns (the other parts held still).
*/
class AssemblyMateJoints {
	/** Relative size below which an eigenvalue of the scaled JᵀJ counts as a free motion (a singular value ratio of 1e-6). */
	static inline var NULL_RATIO:Float = 1e-12;
	/** A turn with a slide along its axis under this share of the assembly scale per radian is a pure turn. */
	static inline var PITCH_RATIO:Float = 1e-6;

	/** The joint the mates of free root `occurrence` amount to, in `state` (which must satisfy them). */
	public static function infer(definition:AssemblyDefinition, state:AssemblyStateRecord, occurrence:String):AssemblyMateJoint {
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var none = (reason:String) -> new AssemblyMateJoint(null, reason, "", occurrence, [], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
		var roots = AssemblyDefinitionCodec.rootOccurrences(flat);
		if (!roots.exists(occurrence)) return none('"$occurrence" hangs from a joint already');
		var mates = flat.mates == null ? [] : [for (mate in flat.mates) if (mate.first == occurrence || mate.second == occurrence) mate];
		if (mates.length == 0) return none('"$occurrence" has no mates');
		var parent = mates[0].first == occurrence ? mates[0].second : mates[0].first;
		for (mate in mates)
			if ((mate.first == occurrence ? mate.second : mate.first) != parent)
				return none('"$occurrence" is mated to more than one part');

		var setup = AssemblyMateSolver.setup(definition, state);
		var kinematics = setup.kinematics, model = kinematics.model;
		var problem = new KinematicProblem(model).setActiveDofs([]);
		problem.setRootMotion(kinematics.body(occurrence), RootMotion.Floating);
		for (mate in mates)
			problem.add(new ClosureTask(model, model.closureIndex(mate.id), setup.settings.positionTolerance, setup.settings.angularTolerance));
		var rows = problem.rowCount(), width = problem.layout().width;
		var residual = [for (_ in 0...rows) 0.0], jacobian = [for (_ in 0...rows * width) 0.0];
		problem.evaluate(setup.seed, new KinematicSnapshot(model), residual, jacobian);

		// JᵀJ over (v / scale, ω): a unit of each column moves the part about as far.
		var scale = setup.scale;
		var normal = [for (_ in 0...36) 0.0];
		for (i in 0...6) for (j in 0...6) {
			var sum = 0.0;
			for (r in 0...rows) sum += jacobian[r * width + i] * jacobian[r * width + j];
			normal[i * 6 + j] = sum * (i < 3 ? scale : 1.0) * (j < 3 ? scale : 1.0);
		}
		var eigen = symmetricEigen(normal);
		var largest = 0.0;
		for (value in eigen.values) largest = Math.max(largest, value);
		var free = [for (k in 0...6) if (eigen.values[k] <= NULL_RATIO * Math.max(largest, 1e-300)) k];
		var names = [for (mate in mates) mate.id];
		if (free.length != 1)
			return none(free.length == 0 ? 'the mates fix "$occurrence" completely' : 'the mates leave "$occurrence" ${free.length} motions, not one');
		var k = free[0];
		var v = [for (i in 0...3) eigen.vectors[i * 6 + k] * scale], w = [for (i in 3...6) eigen.vectors[i * 6 + k]];
		var origin = AssemblyKinematics.toFrame(setup.seed.rootPose(kinematics.body(occurrence)));
		var turn = Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]), slide = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		if (turn <= 1e-9 * Math.max(1, slide / scale))
			return new AssemblyMateJoint(AssemblyJointType.Prismatic, "", parent, occurrence, names, [origin.x, origin.y, origin.z],
				[v[0] / slide, v[1] / slide, v[2] / slide]);
		var axis = [w[0] / turn, w[1] / turn, w[2] / turn];
		var pitch = (v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2]) / turn;
		if (Math.abs(pitch) > PITCH_RATIO * scale) return none('the mates leave "$occurrence" a screw motion, not a turn or a slide');
		// The point of the axis nearest the origin: x − o = (ω × v) / |ω|².
		var cross = [w[1] * v[2] - w[2] * v[1], w[2] * v[0] - w[0] * v[2], w[0] * v[1] - w[1] * v[0]];
		var squared = turn * turn;
		return new AssemblyMateJoint(AssemblyJointType.Revolute, "", parent, occurrence, names,
			[origin.x + cross[0] / squared, origin.y + cross[1] / squared, origin.z + cross[2] / squared], axis);
	}

	/**
		The joint for `inferred` in `state`: a tree joint from its parent to its child about (or along) the axis, at
		coordinate 0 in the current placement, and the connectors it needs ("<joint>-parent" and "<joint>-child"
		on the two parts' components, z along the axis).
	*/
	public static function convert(definition:AssemblyDefinition, state:AssemblyStateRecord, inferred:AssemblyMateJoint,
			jointId:String):AssemblyMateJointConversion {
		var type = inferred.type;
		if (type == null) throw 'The mates make no joint: ${inferred.reason}';
		var current = new AssemblyState(definition, state);
		var world = frameAlong(inferred.point, inferred.axis);
		var parentPose = current.worldPose(inferred.parent), childPose = current.worldPose(inferred.child);
		var parentComponent = "", childComponent = "";
		for (occurrence in definition.occurrences) {
			if (occurrence.id == inferred.parent) parentComponent = occurrence.definition;
			if (occurrence.id == inferred.child) childComponent = occurrence.definition;
		}
		var parentConnector:AssemblyConnector = {name: jointId + "-parent", frame: AssemblyFrames.compose(AssemblyFrames.inverse(parentPose), world)};
		var childConnector:AssemblyConnector = {name: jointId + "-child", frame: AssemblyFrames.compose(AssemblyFrames.inverse(childPose), world)};
		var joint:KinematicJoint = {id: jointId, type: type, role: AssemblyJointRole.Tree, parent: inferred.parent,
			parentConnector: parentConnector.name, child: inferred.child, childConnector: childConnector.name,
			axis: {x: 0, y: 0, z: 1}, limits: {lower: null, upper: null, velocity: null, effort: null}, defaultValue: 0};
		return new AssemblyMateJointConversion(joint, parentComponent, parentConnector, childComponent, childConnector, inferred.mates);
	}

	/** A frame at `point` with z along unit `axis` (x any direction across it). */
	static function frameAlong(point:Array<Float>, axis:Array<Float>):AssemblyFrame {
		var ax = Math.abs(axis[0]), ay = Math.abs(axis[1]), az = Math.abs(axis[2]);
		var hint = ax <= ay && ax <= az ? [1.0, 0.0, 0.0] : ay <= az ? [0.0, 1.0, 0.0] : [0.0, 0.0, 1.0];
		var along = hint[0] * axis[0] + hint[1] * axis[1] + hint[2] * axis[2];
		var x = [hint[0] - along * axis[0], hint[1] - along * axis[1], hint[2] - along * axis[2]];
		var length = Math.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
		x = [x[0] / length, x[1] / length, x[2] / length];
		var y = [axis[1] * x[2] - axis[2] * x[1], axis[2] * x[0] - axis[0] * x[2], axis[0] * x[1] - axis[1] * x[0]];
		return AssemblyFrames.fromRotationMatrix(point[0], point[1], point[2], [x[0], y[0], axis[0], x[1], y[1], axis[1], x[2], y[2], axis[2]]);
	}

	/** Eigenvalues and eigenvectors (columns of a row-major 6 × 6) of a symmetric 6 × 6 matrix, by cyclic Jacobi rotations. */
	static function symmetricEigen(matrix:Array<Float>):{values:Array<Float>, vectors:Array<Float>} {
		var n = 6, a = matrix.copy();
		var vectors = [for (i in 0...n * n) i % (n + 1) == 0 ? 1.0 : 0.0];
		for (_ in 0...100) {
			var off = 0.0;
			for (p in 0...n) for (q in p + 1...n) off += a[p * n + q] * a[p * n + q];
			if (off <= 1e-30) break;
			for (p in 0...n) for (q in p + 1...n) {
				var apq = a[p * n + q];
				if (Math.abs(apq) <= 1e-300) continue;
				var theta = (a[q * n + q] - a[p * n + p]) / (2 * apq);
				var t = (theta >= 0 ? 1.0 : -1.0) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
				var c = 1 / Math.sqrt(t * t + 1), s = t * c;
				for (k in 0...n) {
					var akp = a[k * n + p], akq = a[k * n + q];
					a[k * n + p] = c * akp - s * akq;
					a[k * n + q] = s * akp + c * akq;
				}
				for (k in 0...n) {
					var apk = a[p * n + k], aqk = a[q * n + k];
					a[p * n + k] = c * apk - s * aqk;
					a[q * n + k] = s * apk + c * aqk;
				}
				for (k in 0...n) {
					var vkp = vectors[k * n + p], vkq = vectors[k * n + q];
					vectors[k * n + p] = c * vkp - s * vkq;
					vectors[k * n + q] = s * vkp + c * vkq;
				}
			}
		}
		return {values: [for (i in 0...n) a[i * n + i]], vectors: vectors};
	}
}
