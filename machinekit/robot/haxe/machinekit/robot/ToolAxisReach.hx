package machinekit.robot;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
/** Design-only reach check for a named TCP and axis, with joint limits and free roll. */
class ToolAxisReach {
	/** Weight of one unit of direction error against one millimetre of position error. */
	static inline var DIRECTION_WEIGHT:Float = 200;

	/** The joint values that put the tip on `target` with the wire along `direction`, or null. */
	public static function solve(state:AssemblyState, joints:Array<String>, limits:Array<{lower:Float, upper:Float}>,
			target:{x:Float, y:Float, z:Float}, direction:{x:Float, y:Float, z:Float}, seed:Array<Float>, occurrence:String, connector:String):Null<Array<Float>> {
		var q = [for (j in 0...joints.length) Math.min(limits[j].upper, Math.max(limits[j].lower, seed[j]))];
		var residual = evaluate(state, joints, q, target, direction, occurrence, connector);
		var damping = 1.0;
		for (iteration in 0...120) {
			if (norm(residual) < 0.05) return q;
			var jacobian:Array<Array<Float>> = [];
			for (j in 0...joints.length) {
				var step = q.copy();
				var h = q[j] + 1e-4 <= limits[j].upper ? 1e-4 : -1e-4;
				step[j] += h;
				var moved = evaluate(state, joints, step, target, direction, occurrence, connector);
				jacobian.push([for (k in 0...6) (moved[k] - residual[k]) / h]);
			}
			// (J^T J + damping I) delta = -J^T r, with jacobian[j][k] = d residual k / d joint j.
			var normal:Array<Array<Float>> = [];
			for (a in 0...joints.length) {
				var row:Array<Float> = [];
				for (b in 0...joints.length) {
					var sum = a == b ? damping : 0.0;
					for (k in 0...6) sum += jacobian[a][k] * jacobian[b][k];
					row.push(sum);
				}
				var rhs = 0.0;
				for (k in 0...6) rhs -= jacobian[a][k] * residual[k];
				row.push(rhs);
				normal.push(row);
			}
			var delta = gauss(normal);
			var next = [for (j in 0...joints.length) Math.min(limits[j].upper, Math.max(limits[j].lower, q[j] + delta[j]))];
			var nextResidual = evaluate(state, joints, next, target, direction, occurrence, connector);
			if (norm(nextResidual) < norm(residual)) {
				q = next;
				residual = nextResidual;
				damping = Math.max(1e-6, damping * 0.5);
			} else damping *= 4;
		}
		return norm(residual) < 0.5 ? q : null;
	}

	static function evaluate(state:AssemblyState, joints:Array<String>, q:Array<Float>, target:{x:Float, y:Float, z:Float},
			direction:{x:Float, y:Float, z:Float}, occurrence:String, connector:String):Array<Float> {
		for (i in 0...joints.length) state.setJoint(joints[i], q[i]);
		state.forwardKinematics();
		var tcp = state.worldConnector(occurrence, connector);
		var wire = AssemblyFrames.transformVector(tcp, 0, 0, 1);
		return [tcp.x - target.x, tcp.y - target.y, tcp.z - target.z, DIRECTION_WEIGHT * (wire.x - direction.x),
			DIRECTION_WEIGHT * (wire.y - direction.y), DIRECTION_WEIGHT * (wire.z - direction.z)];
	}

	static function norm(values:Array<Float>):Float {
		var sum = 0.0;
		for (value in values) sum += value * value;
		return Math.sqrt(sum);
	}

	/** Solves an augmented n x (n+1) system by elimination with partial pivoting. */
	static function gauss(matrix:Array<Array<Float>>):Array<Float> {
		var n = matrix.length;
		for (column in 0...n) {
			var pivot = column;
			for (row in column + 1...n) if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) pivot = row;
			var swap = matrix[column];
			matrix[column] = matrix[pivot];
			matrix[pivot] = swap;
			for (row in column + 1...n) {
				var factor = matrix[row][column] / matrix[column][column];
				for (k in column...n + 1) matrix[row][k] -= factor * matrix[column][k];
			}
		}
		var result = [for (i in 0...n) 0.0];
		var i = n - 1;
		while (i >= 0) {
			var sum = matrix[i][n];
			for (k in i + 1...n) sum -= matrix[i][k] * result[k];
			result[i] = sum / matrix[i][i];
			i--;
		}
		return result;
	}
}
