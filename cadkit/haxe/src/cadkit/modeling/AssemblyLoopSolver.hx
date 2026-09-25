package cadkit.modeling;

import materia.project.AssemblyDefinition.AssemblyJointRole;
import materia.project.AssemblyDefinition.AssemblyJointType;
import materia.project.AssemblyDefinition.AssemblyVector;
import materia.project.AssemblyDefinition.KinematicJoint;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Tolerances and iteration settings for joint-coordinate loop solving. */
typedef AssemblyLoopSolveOptions = {
	@:optional var positionTolerance:Float;
	@:optional var angularTolerance:Float;
	@:optional var maxIterations:Int;
	@:optional var initialDamping:Float;
	@:optional var rankTolerance:Float;
	@:optional var finiteDifferenceStep:Float;
}

/** Outcome of solving a set of assembly closure joints. */
class AssemblyLoopSolveResult {
	public final status:String;
	public final converged:Bool;
	/** Euclidean norm of all closure residuals after scaling by the requested tolerances. */
	public final residual:Float;
	public final positionResidual:Float;
	public final angularResidual:Float;
	public final degreesOfFreedom:Int;
	public final iterations:Int;
	/** Closure joint IDs that remain outside tolerance; empty on success. */
	public final closureIds:Array<String>;
	public final message:String;

	public function new(status:String, converged:Bool, residual:Float, positionResidual:Float,
		angularResidual:Float, degreesOfFreedom:Int, iterations:Int, closureIds:Array<String>, message:String) {
		this.status = status;
		this.converged = converged;
		this.residual = residual;
		this.positionResidual = positionResidual;
		this.angularResidual = angularResidual;
		this.degreesOfFreedom = degreesOfFreedom;
		this.iterations = iterations;
		this.closureIds = closureIds.copy();
		this.message = message;
	}
}

private typedef ResidualEvaluation = {
	var values:Array<Float>;
	var positionResidual:Float;
	var angularResidual:Float;
	var failingIds:Array<String>;
	var converged:Bool;
}

/**
	Solves closed mechanisms by varying selected tree-joint coordinates. Other
	tree coordinates remain fixed and closure joints contribute geometric
	residuals; closure joints are never treated as additional FK edges.
*/
class AssemblyLoopSolver {
	static inline var DEFAULT_POSITION_TOLERANCE:Float = 1e-3;
	static inline var DEFAULT_ANGULAR_TOLERANCE:Float = 1e-5;
	static inline var DEFAULT_MAX_ITERATIONS:Int = 100;
	static inline var DEFAULT_DAMPING:Float = 1e-3;
	static inline var DEFAULT_RANK_TOLERANCE:Float = 1e-8;
	static inline var DEFAULT_FINITE_DIFFERENCE_STEP:Float = 1e-6;

	/**
		Adjusts only the named tree-joint coordinates. The supplied state is
		updated when all closure residuals meet tolerance; on failure it is left
		at its original configuration.
	*/
	public static function solve(state:AssemblyState, dependentJointIds:Array<String>,
		?options:AssemblyLoopSolveOptions):AssemblyLoopSolveResult {
		if (state == null) throw "Assembly loop solve needs a state";
		if (dependentJointIds == null || dependentJointIds.length == 0)
			throw "Assembly loop solve needs at least one dependent tree joint";
		var positionTolerance = option(options, "positionTolerance", DEFAULT_POSITION_TOLERANCE);
		var angularTolerance = option(options, "angularTolerance", DEFAULT_ANGULAR_TOLERANCE);
		var maxIterations = intOption(options, "maxIterations", DEFAULT_MAX_ITERATIONS);
		var initialDamping = option(options, "initialDamping", DEFAULT_DAMPING);
		var rankTolerance = option(options, "rankTolerance", DEFAULT_RANK_TOLERANCE);
		var finiteDifferenceStep = option(options, "finiteDifferenceStep", DEFAULT_FINITE_DIFFERENCE_STEP);
		validateOptions(positionTolerance, angularTolerance, maxIterations, initialDamping,
			rankTolerance, finiteDifferenceStep);

		var closureJoints:Array<KinematicJoint> = [];
		for (joint in state.definition.joints)
			if (joint.role == AssemblyJointRole.Closure) closureJoints.push(joint);
		if (closureJoints.length == 0) throw "Assembly loop solve needs at least one closure joint";

		var jointById = new Map<String, KinematicJoint>();
		for (joint in state.definition.joints) jointById.set(joint.id, joint);
		var selected:Array<KinematicJoint> = [];
		var seen = new Map<String, Bool>();
		for (id in dependentJointIds) {
			var joint = jointById.get(id);
			if (joint == null || joint.role != AssemblyJointRole.Tree ||
				(joint.type != AssemblyJointType.Revolute && joint.type != AssemblyJointType.Continuous &&
				joint.type != AssemblyJointType.Prismatic))
				throw 'Dependent assembly coordinate "$id" is not a movable tree joint';
			if (seen.exists(id)) throw 'Dependent assembly coordinate "$id" is listed more than once';
			seen.set(id, true);
			selected.push(joint);
		}

		var characteristicLength = assemblyScale(state);
		var scales:Array<Float> = [];
		var originalValues:Array<Float> = [];
		for (joint in selected) {
			scales.push(joint.type == AssemblyJointType.Prismatic ? characteristicLength : 1.0);
			originalValues.push(state.joint(joint.id));
		}

		// Work on a private configuration so numerical failure cannot partially
		// mutate the caller's state.
		var candidate = new AssemblyState(state.definition, state.record());
		var current = evaluate(candidate, closureJoints, positionTolerance, angularTolerance);
		var currentNorm = norm(current.values);
		var damping = initialDamping;
		var iterations = 0;
		var jacobian:Array<Array<Float>> = [];
		var exhausted = false;
		var limitStalled = false;
		while (!current.converged && iterations < maxIterations) {
			limitStalled = false;
			iterations++;
			jacobian = numericalJacobian(candidate, selected, scales, closureJoints, current.values,
				positionTolerance, angularTolerance, finiteDifferenceStep);
			var normal = normalMatrix(jacobian, selected.length);
			var gradient = gradientVector(jacobian, current.values, selected.length);
			var rhs:Array<Float> = [];
			for (value in gradient) rhs.push(-value);
			for (column in 0...selected.length)
				normal[column][column] += damping * Math.max(normal[column][column], 1e-12);
			var delta = linearSolve(normal, rhs);
			if (delta == null) {
				damping = Math.min(1e16, damping * 10);
				if (damping >= 1e16) exhausted = true;
				if (exhausted) break;
				continue;
			}

			var maxDelta = 0.0;
			for (value in delta) maxDelta = Math.max(maxDelta, Math.abs(value));
			var stepFactor = maxDelta > 1 ? 1 / maxDelta : 1.0;
			var moved = false;
			var outwardAtLimit = false;
			for (index in 0...selected.length) {
				var joint = selected[index];
				var currentValue = candidate.joint(joint.id);
				var requestedValue = currentValue + delta[index] * stepFactor * scales[index];
				var value = requestedValue;
				if (joint.limits.lower != null) value = Math.max(value, joint.limits.lower);
				if (joint.limits.upper != null) value = Math.min(value, joint.limits.upper);
				if (Math.abs(value - currentValue) > 1e-14) moved = true;
				else if (Math.abs(requestedValue - currentValue) > 1e-14 &&
					((joint.limits.lower != null && currentValue <= joint.limits.lower && requestedValue < currentValue) ||
					(joint.limits.upper != null && currentValue >= joint.limits.upper && requestedValue > currentValue)))
					outwardAtLimit = true;
				candidate.setJoint(joint.id, value);
			}
			if (!moved) {
				limitStalled = outwardAtLimit;
				damping = Math.min(1e16, damping * 10);
				if (damping >= 1e16) exhausted = true;
				if (exhausted) break;
				continue;
			}

			var trial = evaluate(candidate, closureJoints, positionTolerance, angularTolerance);
			var trialNorm = norm(trial.values);
			if (trial.converged || trialNorm < currentNorm) {
				current = trial;
				currentNorm = trialNorm;
				limitStalled = false;
				damping = Math.max(1e-12, damping * 0.3);
			} else {
				// Restore the last accepted coordinates before trying a more
				// conservative step.
				for (index in 0...selected.length)
					candidate.setJoint(selected[index].id, originalValues[index]);
				// `originalValues` is updated after every accepted step below.
				damping = Math.min(1e16, damping * 10);
				if (damping >= 1e16) exhausted = true;
				if (exhausted) break;
			}
			if (current.converged) break;
			// Keep an accepted-coordinate snapshot for rollback on the next trial.
			for (index in 0...selected.length) originalValues[index] = candidate.joint(selected[index].id);
		}

		// Re-evaluate once because the final accepted step may have reached the
		// requested tolerances at the iteration boundary.
		current = evaluate(candidate, closureJoints, positionTolerance, angularTolerance);
		jacobian = numericalJacobian(candidate, selected, scales, closureJoints, current.values,
			positionTolerance, angularTolerance, finiteDifferenceStep);
		var rankValue = matrixRank(jacobian, rankTolerance);
		var dof = selected.length - rankValue;
		if (current.converged) {
			for (joint in selected) state.setJoint(joint.id, candidate.joint(joint.id));
			return new AssemblyLoopSolveResult("converged", true, norm(current.values),
				current.positionResidual, current.angularResidual, dof, iterations, [],
				"assembly closures converged");
		}

		var gradient = gradientVector(jacobian, current.values, selected.length);
		var gradientNorm = norm(gradient);
		var jacobianNorm = matrixNorm(jacobian);
		var stationary = gradientNorm <= 1e-10 * (1 + jacobianNorm * norm(current.values));
		var status = limitStalled ? "limit-blocked" : (stationary ? "conflicting" : "nonconvergent");
		var message = limitStalled
			? "joint limits blocked further motion before the closure tolerances were met"
			: stationary
			? "closure residuals stopped at a locally stationary configuration; this does not prove global inconsistency"
			: "assembly loop solve exhausted its iteration limit while a local descent direction remained";
		return new AssemblyLoopSolveResult(status, false, norm(current.values), current.positionResidual,
			current.angularResidual, dof, iterations, current.failingIds, message);
	}

	static function evaluate(state:AssemblyState, closures:Array<KinematicJoint>, positionTolerance:Float,
		angularTolerance:Float):ResidualEvaluation {
		state.forwardKinematics();
		var values:Array<Float> = [];
		var positionResidual = 0.0;
		var angularResidual = 0.0;
		var failingIds:Array<String> = [];
		var converged = true;
		for (joint in closures) {
			var first = state.worldConnector(joint.parent, joint.parentConnector);
			var second = state.worldConnector(joint.child, joint.childConnector);
			var firstAxis = normalized(AssemblyFrames.transformVector(first, joint.axis.x, joint.axis.y, joint.axis.z));
			var secondAxis = normalized(AssemblyFrames.transformVector(second, joint.axis.x, joint.axis.y, joint.axis.z));
			var dx = second.x - first.x, dy = second.y - first.y, dz = second.z - first.z;
			var position = 0.0;
			var angle = 0.0;
			switch (joint.type) {
				case AssemblyJointType.Fixed:
					position = Math.sqrt(dx * dx + dy * dy + dz * dz);
					push(values, [dx / positionTolerance, dy / positionTolerance, dz / positionTolerance]);
					var rotation = rotationVector(first, second);
					angle = Math.sqrt(rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z);
					push(values, [rotation.x / angularTolerance, rotation.y / angularTolerance,
						rotation.z / angularTolerance]);
				case AssemblyJointType.Revolute | AssemblyJointType.Continuous:
					position = Math.sqrt(dx * dx + dy * dy + dz * dz);
					push(values, [dx / positionTolerance, dy / positionTolerance, dz / positionTolerance]);
					angle = axisAngle(firstAxis, secondAxis);
					appendAxisResidual(values, firstAxis, secondAxis, angularTolerance);
				case AssemblyJointType.Prismatic:
					var along = dx * firstAxis.x + dy * firstAxis.y + dz * firstAxis.z;
					var px = dx - along * firstAxis.x, py = dy - along * firstAxis.y, pz = dz - along * firstAxis.z;
					var basis = perpendicularBasis(firstAxis);
					var transverseA = px * basis.u.x + py * basis.u.y + pz * basis.u.z;
					var transverseB = px * basis.v.x + py * basis.v.y + pz * basis.v.z;
					position = Math.sqrt(transverseA * transverseA + transverseB * transverseB);
					push(values, [transverseA / positionTolerance, transverseB / positionTolerance]);
					angle = axisAngle(firstAxis, secondAxis);
					appendAxisResidual(values, firstAxis, secondAxis, angularTolerance);
				default:
					throw 'Unsupported assembly closure type "${joint.type}"';
			}
			positionResidual = Math.max(positionResidual, position);
			angularResidual = Math.max(angularResidual, angle);
			if (position > positionTolerance || angle > angularTolerance) {
				converged = false;
				failingIds.push(joint.id);
			}
		}
		return {values: values, positionResidual: positionResidual,
			angularResidual: angularResidual, failingIds: failingIds, converged: converged};
	}

	static function numericalJacobian(state:AssemblyState, joints:Array<KinematicJoint>, scales:Array<Float>,
		closures:Array<KinematicJoint>, base:Array<Float>, positionTolerance:Float,
		angularTolerance:Float, stepSize:Float):Array<Array<Float>> {
		var result:Array<Array<Float>> = [];
		for (_ in 0...base.length) {
			var row:Array<Float> = [];
			for (_ in 0...joints.length) row.push(0);
			result.push(row);
		}
		for (column in 0...joints.length) {
			var joint = joints[column];
			var current = state.joint(joint.id);
			var normalizedCurrent = current / scales[column];
			var step = stepSize * Math.max(1, Math.abs(normalizedCurrent)) * scales[column];
			var plus = joint.limits.upper == null ? step : Math.min(step, Math.max(0, joint.limits.upper - current));
			var minus = joint.limits.lower == null ? step : Math.min(step, Math.max(0, current - joint.limits.lower));
			var plusValues:Null<Array<Float>> = null;
			var minusValues:Null<Array<Float>> = null;
			if (plus > 0) {
				state.setJoint(joint.id, current + plus);
				plusValues = evaluate(state, closures, positionTolerance, angularTolerance).values;
				state.setJoint(joint.id, current);
			}
			if (minus > 0) {
				state.setJoint(joint.id, current - minus);
				minusValues = evaluate(state, closures, positionTolerance, angularTolerance).values;
				state.setJoint(joint.id, current);
			}
			for (row in 0...base.length) {
				var derivative:Float;
				if (plusValues != null && minusValues != null)
					derivative = (plusValues[row] - minusValues[row]) / (plus + minus) * scales[column];
				else if (plusValues != null)
					derivative = (plusValues[row] - base[row]) / plus * scales[column];
				else if (minusValues != null)
					derivative = (base[row] - minusValues[row]) / minus * scales[column];
				else derivative = 0;
				result[row][column] = derivative;
			}
		}
		return result;
	}

	static function normalMatrix(jacobian:Array<Array<Float>>, columnCount:Int):Array<Array<Float>> {
		var result:Array<Array<Float>> = [];
		for (_ in 0...columnCount) {
			var row:Array<Float> = [];
			for (_ in 0...columnCount) row.push(0);
			result.push(row);
		}
		for (row in jacobian)
			for (a in 0...columnCount)
				for (b in 0...columnCount) result[a][b] += row[a] * row[b];
		return result;
	}

	static function gradientVector(jacobian:Array<Array<Float>>, residual:Array<Float>, columnCount:Int):Array<Float> {
		var result:Array<Float> = [];
		for (_ in 0...columnCount) result.push(0);
		for (row in 0...jacobian.length)
			for (column in 0...columnCount) result[column] += jacobian[row][column] * residual[row];
		return result;
	}

	static function linearSolve(input:Array<Array<Float>>, values:Array<Float>):Null<Array<Float>> {
		var count = values.length;
		var matrix:Array<Array<Float>> = [];
		for (row in 0...count) {
			matrix.push(input[row].copy());
			matrix[row].push(values[row]);
		}
		for (column in 0...count) {
			var pivot = column;
			for (row in column...count)
				if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) pivot = row;
			if (!Math.isFinite(matrix[pivot][column]) || Math.abs(matrix[pivot][column]) < 1e-30) return null;
			var swap = matrix[column]; matrix[column] = matrix[pivot]; matrix[pivot] = swap;
			for (row in (column + 1)...count) {
				var factor = matrix[row][column] / matrix[column][column];
				for (entry in column...(count + 1)) matrix[row][entry] -= factor * matrix[column][entry];
			}
		}
		var result:Array<Float> = [];
		for (_ in 0...count) result.push(0);
		var row = count - 1;
		while (row >= 0) {
			var value = matrix[row][count];
			for (column in (row + 1)...count) value -= matrix[row][column] * result[column];
			result[row] = value / matrix[row][row];
			if (!Math.isFinite(result[row])) return null;
			row--;
		}
		return result;
	}

	static function matrixRank(input:Array<Array<Float>>, tolerance:Float):Int {
		if (input.length == 0 || input[0].length == 0) return 0;
		var matrix:Array<Array<Float>> = [];
		var maxValue = 0.0;
		for (row in input) {
			matrix.push(row.copy());
			for (value in row) maxValue = Math.max(maxValue, Math.abs(value));
		}
		if (maxValue == 0) return 0;
		var threshold = maxValue * tolerance;
		var row = 0;
		var column = 0;
		while (row < matrix.length && column < matrix[0].length) {
			var pivot = row;
			for (candidate in row...matrix.length)
				if (Math.abs(matrix[candidate][column]) > Math.abs(matrix[pivot][column])) pivot = candidate;
			if (Math.abs(matrix[pivot][column]) <= threshold) { column++; continue; }
			var swap = matrix[row]; matrix[row] = matrix[pivot]; matrix[pivot] = swap;
			var divisor = matrix[row][column];
			for (entry in column...matrix[row].length) matrix[row][entry] /= divisor;
			for (candidate in 0...matrix.length) if (candidate != row) {
				var factor = matrix[candidate][column];
				for (entry in column...matrix[candidate].length)
					matrix[candidate][entry] -= factor * matrix[row][entry];
			}
			row++;
			column++;
		}
		return row;
	}

	static function rotationVector(first:AssemblyFrame, second:AssemblyFrame):AssemblyVector {
		// Quaternion of inverse(first) * second gives the shortest local rotation.
		var x = first.qw * second.qx - first.qx * second.qw - first.qy * second.qz + first.qz * second.qy;
		var y = first.qw * second.qy + first.qx * second.qz - first.qy * second.qw - first.qz * second.qx;
		var z = first.qw * second.qz - first.qx * second.qy + first.qy * second.qx - first.qz * second.qw;
		var w = first.qw * second.qw + first.qx * second.qx + first.qy * second.qy + first.qz * second.qz;
		if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
		var sine = Math.sqrt(x * x + y * y + z * z);
		if (sine < 1e-12) return {x: 2 * x, y: 2 * y, z: 2 * z};
		var angle = 2 * Math.atan2(sine, w), scale = angle / sine;
		return {x: x * scale, y: y * scale, z: z * scale};
	}

	static function appendAxisResidual(values:Array<Float>, first:AssemblyVector, second:AssemblyVector,
		angularTolerance:Float):Void {
		var basis = perpendicularBasis(first);
		var sign = dot(first, second) < 0 ? -1.0 : 1.0;
		values.push(sign * dot(second, basis.u) / angularTolerance);
		values.push(sign * dot(second, basis.v) / angularTolerance);
	}

	static function perpendicularBasis(axis:AssemblyVector):{u:AssemblyVector, v:AssemblyVector} {
		var reference:AssemblyVector = Math.abs(axis.x) < 0.8 ? {x: 1, y: 0, z: 0} : {x: 0, y: 1, z: 0};
		var u = normalized(cross(axis, reference));
		return {u: u, v: normalized(cross(axis, u))};
	}

	static function axisAngle(first:AssemblyVector, second:AssemblyVector):Float {
		var cosine = Math.min(1, Math.max(-1, Math.abs(dot(first, second))));
		return Math.acos(cosine);
	}

	static function normalized(vector:AssemblyVector):AssemblyVector {
		var length = Math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
		if (length < 1e-12) throw "Assembly loop solve encountered a degenerate frame axis";
		return {x: vector.x / length, y: vector.y / length, z: vector.z / length};
	}

	static function cross(a:AssemblyVector, b:AssemblyVector):AssemblyVector
		return {x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x};

	static function dot(a:AssemblyVector, b:AssemblyVector):Float return a.x * b.x + a.y * b.y + a.z * b.z;

	static function push(target:Array<Float>, values:Array<Float>):Void
		for (value in values) target.push(value);

	static function assemblyScale(state:AssemblyState):Float {
		var minX = 1e300, minY = 1e300, minZ = 1e300;
		var maxX = -1e300, maxY = -1e300, maxZ = -1e300;
		for (occurrence in state.definition.occurrences) {
			var pose = state.worldPose(occurrence.id);
			minX = Math.min(minX, pose.x); minY = Math.min(minY, pose.y); minZ = Math.min(minZ, pose.z);
			maxX = Math.max(maxX, pose.x); maxY = Math.max(maxY, pose.y); maxZ = Math.max(maxZ, pose.z);
		}
		return Math.max(1, Math.sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY) +
			(maxZ - minZ) * (maxZ - minZ)));
	}

	static function option(options:Null<AssemblyLoopSolveOptions>, name:String, fallback:Float):Float {
		if (options == null) return fallback;
		var value:Dynamic = Reflect.field(options, name);
		return value == null ? fallback : cast value;
	}

	static function intOption(options:Null<AssemblyLoopSolveOptions>, name:String, fallback:Int):Int {
		if (options == null) return fallback;
		var value:Dynamic = Reflect.field(options, name);
		return value == null ? fallback : cast value;
	}

	static function validateOptions(positionTolerance:Float, angularTolerance:Float, maxIterations:Int,
		initialDamping:Float, rankTolerance:Float, finiteDifferenceStep:Float):Void {
		if (!Math.isFinite(positionTolerance) || positionTolerance <= 0 || !Math.isFinite(angularTolerance) ||
			angularTolerance <= 0 || maxIterations <= 0 || !Math.isFinite(initialDamping) || initialDamping <= 0 ||
			!Math.isFinite(rankTolerance) || rankTolerance <= 0 || !Math.isFinite(finiteDifferenceStep) ||
			finiteDifferenceStep <= 0)
			throw "Assembly loop solver settings must be finite and positive";
	}

	static function norm(values:Array<Float>):Float {
		var sum = 0.0;
		for (value in values) sum += value * value;
		return Math.sqrt(sum);
	}

	static function matrixNorm(matrix:Array<Array<Float>>):Float {
		var sum = 0.0;
		for (row in matrix) for (value in row) sum += value * value;
		return Math.sqrt(sum);
	}
}
