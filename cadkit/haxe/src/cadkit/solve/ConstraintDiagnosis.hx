package cadkit.solve;

/** Constraints that share one linear dependency; see `ConstraintDiagnosis`. */
class DependencyGroup {
	/** Constraint (owner) IDs, sorted. */
	public final owners:Array<String>;
	/** How many rows could be dropped without losing rank. */
	public final deficiency:Int;
	/** Every row of the group is within tolerance: redundant, not conflicting. */
	public final satisfied:Bool;

	public function new(owners:Array<String>, deficiency:Int, satisfied:Bool) {
		this.owners = owners;
		this.deficiency = deficiency;
		this.satisfied = satisfied;
	}

	public function toString():String
		return (satisfied ? "redundant" : "conflicting") + "(" + owners.join(",") + ")-" + deficiency;
}

/** One independent part of the system: owners that share variables, directly or through each other. */
class DiagnosisSubsystem {
	public final owners:Array<String>;
	public final variables:Int;
	public final rank:Int;

	public function new(owners:Array<String>, variables:Int, rank:Int) {
		this.owners = owners;
		this.variables = variables;
		this.rank = rank;
	}
}

/** What `ConstraintDiagnosis.diagnose` found. Every list is sorted, so equal systems give equal reports. */
class DiagnosisReport {
	public final variables:Int;
	/** Numerical rank of the Jacobian at the evaluated point. */
	public final rank:Int;
	public final degreesOfFreedom:Int;
	public final subsystems:Array<DiagnosisSubsystem>;
	public final dependencyGroups:Array<DependencyGroup>;
	/** Owners with a row outside tolerance, whether or not they are in a group. */
	public final unsatisfied:Array<String>;
	/** A kept pivot is within `nearDegenerateRatio` of the rank threshold: rank may change nearby. */
	public final nearDegenerate:Bool;
	/** A repair suggestion, not part of the diagnosis: owners whose removal leaves no dependency. */
	public final suggestedRemovals:Array<String>;

	public function new(variables:Int, rank:Int, subsystems:Array<DiagnosisSubsystem>, dependencyGroups:Array<DependencyGroup>,
			unsatisfied:Array<String>, nearDegenerate:Bool, suggestedRemovals:Array<String>) {
		this.variables = variables;
		this.rank = rank;
		this.degreesOfFreedom = variables - rank;
		this.subsystems = subsystems;
		this.dependencyGroups = dependencyGroups;
		this.unsatisfied = unsatisfied;
		this.nearDegenerate = nearDegenerate;
		this.suggestedRemovals = suggestedRemovals;
	}

	public function redundantOwners():Array<String>
		return ownersOf(true);

	public function conflictingOwners():Array<String>
		return ownersOf(false);

	function ownersOf(satisfied:Bool):Array<String> {
		var result:Array<String> = [];
		for (group in dependencyGroups)
			if (group.satisfied == satisfied)
				for (owner in group.owners)
					if (result.indexOf(owner) < 0)
						result.push(owner);
		result.sort(Reflect.compare);
		return result;
	}
}

/** Inputs to `ConstraintDiagnosis.diagnose`. */
typedef DiagnosisInput = {
	/** Row-major, `owners.length` rows by `variables` columns. Rows are residuals divided by their tolerance. */
	var jacobian:Array<Float>;
	var variables:Int;
	/** The constraint each row belongs to; a constraint may own several rows. */
	var owners:Array<String>;
	/** Residuals in the same scaling as the rows: |r| <= 1 is within tolerance. */
	var residuals:Array<Float>;
	/** Owners never suggested for removal (structural rows such as arc endpoint rules). */
	@:optional var protectedOwners:Array<String>;
	/** Pivots below this fraction of the largest are rank-deficient. Default 1e-9. */
	@:optional var rankTolerance:Float;
}

/**
	Constraint diagnosis over a Jacobian, independent of what produced it
	(`cadkit/plans/CONSTRAINT_SOLVING.md`, C1):
	1. rows are split into subsystems that share no variable;
	2. each subsystem is equilibrated (rows, then columns, to unit norm) and
	   factored by Householder QR with column pivoting of its transpose, so
	   pivots pick independent rows; rank uses a threshold relative to the
	   largest pivot;
	3. every dependent row yields its fundamental circuit (the independent
	   rows it is a combination of); circuits sharing an owner merge into one
	   `DependencyGroup`, which does not depend on the pivot order;
	4. a group is redundant when all its rows are within tolerance, otherwise
	   conflicting.
*/
class ConstraintDiagnosis {
	public static inline var DEFAULT_RANK_TOLERANCE:Float = 1e-9;
	/** Share of a circuit's largest coefficient below which an owner is not a member. */
	static inline var MEMBER_TOLERANCE:Float = 1e-7;
	/** How far above the rank threshold a pivot must be not to count as near-degenerate. */
	static inline var NEAR_DEGENERATE_RATIO:Float = 1e3;

	public static function diagnose(input:DiagnosisInput):DiagnosisReport {
		var rows = input.owners.length, columns = input.variables;
		if (columns < 0 || input.jacobian.length != rows * columns)
			throw 'Diagnosis Jacobian has ${input.jacobian.length} entries, expected $rows x $columns';
		if (input.residuals.length != rows)
			throw 'Diagnosis has ${input.residuals.length} residuals for $rows rows';
		var tolerance = input.rankTolerance == null ? DEFAULT_RANK_TOLERANCE : input.rankTolerance;
		if (!(tolerance > 0) || tolerance >= 1)
			throw "Diagnosis rank tolerance must be in (0, 1)";

		var unsatisfiedRow = [for (row in 0...rows) !(Math.abs(input.residuals[row]) <= 1)];
		var subsystems:Array<DiagnosisSubsystem> = [];
		var circuits:Array<Array<Int>> = [];
		var rank = 0, nearDegenerate = false;
		for (component in components(input, rows, columns)) {
			var result = factor(input, component.rows, component.columns, tolerance);
			rank += result.rank;
			nearDegenerate = nearDegenerate || result.nearDegenerate;
			for (circuit in result.circuits)
				circuits.push(circuit);
			subsystems.push(new DiagnosisSubsystem(uniqueOwners(input.owners, component.rows), component.columns.length, result.rank));
		}
		subsystems.sort((a, b) -> Reflect.compare(a.owners.join(","), b.owners.join(",")));

		var groups = mergeCircuits(input.owners, circuits, unsatisfiedRow);
		var unsatisfied = uniqueOwners(input.owners, [for (row in 0...rows) if (unsatisfiedRow[row]) row]);
		var protectedOwners = input.protectedOwners == null ? [] : input.protectedOwners;
		return new DiagnosisReport(columns, rank, subsystems, groups, unsatisfied, nearDegenerate,
			suggestRemovals(input.owners, groups, protectedOwners));
	}

	/** Rows joined through shared variables and shared owners; columns no row touches are left out (free). */
	static function components(input:DiagnosisInput, rows:Int, columns:Int):Array<{rows:Array<Int>, columns:Array<Int>}> {
		var parent = [for (i in 0...rows) i];
		var firstRowOfColumn = [for (_ in 0...columns) -1];
		var firstRowOfOwner = new Map<String, Int>();
		for (row in 0...rows) {
			var owner = input.owners[row];
			if (firstRowOfOwner.exists(owner))
				join(parent, row, firstRowOfOwner.get(owner));
			else
				firstRowOfOwner.set(owner, row);
			for (column in 0...columns)
				if (input.jacobian[row * columns + column] != 0) {
					if (firstRowOfColumn[column] < 0)
						firstRowOfColumn[column] = row;
					else
						join(parent, row, firstRowOfColumn[column]);
				}
		}
		// Roots are row indices; `slot[root]` is the root's component in order of first appearance.
		var slot = [for (_ in 0...rows) -1];
		var result:Array<{rows:Array<Int>, columns:Array<Int>}> = [];
		for (row in 0...rows) {
			var root = find(parent, row);
			if (slot[root] < 0) {
				slot[root] = result.length;
				result.push({rows: [], columns: []});
			}
			result[slot[root]].rows.push(row);
		}
		for (column in 0...columns)
			if (firstRowOfColumn[column] >= 0)
				result[slot[find(parent, firstRowOfColumn[column])]].columns.push(column);
		return result;
	}

	/**
		Rank and fundamental circuits of one component. Works on A = Jᵀ
		(variables × rows), so a column pivot is a constraint row.
	*/
	static function factor(input:DiagnosisInput, rowIds:Array<Int>, columnIds:Array<Int>,
			tolerance:Float):{rank:Int, circuits:Array<Array<Int>>, nearDegenerate:Bool} {
		var m = columnIds.length, n = rowIds.length;
		if (m == 0 || n == 0)
			return {rank: 0, circuits: [], nearDegenerate: false};
		// a[i * n + j] = J[row j][variable i], equilibrated.
		var a = [for (i in 0...m) for (j in 0...n) input.jacobian[rowIds[j] * input.variables + columnIds[i]]];
		for (pass in 0...2) {
			for (j in 0...n) {
				var norm = 0.0;
				for (i in 0...m)
					norm += a[i * n + j] * a[i * n + j];
				norm = Math.sqrt(norm);
				if (norm > 0)
					for (i in 0...m)
						a[i * n + j] /= norm;
			}
			if (pass == 1)
				break;
			for (i in 0...m) {
				var norm = 0.0;
				for (j in 0...n)
					norm += a[i * n + j] * a[i * n + j];
				norm = Math.sqrt(norm);
				if (norm > 0)
					for (j in 0...n)
						a[i * n + j] /= norm;
			}
		}

		// Householder QR with column pivoting; `permutation[k]` is the local row pivoted to position k.
		var permutation = [for (j in 0...n) j];
		var norms = [for (j in 0...n) columnNorm(a, m, n, j, 0)];
		var steps = m < n ? m : n;
		var rank = 0, largest = 0.0, smallestKept = Math.POSITIVE_INFINITY;
		for (k in 0...steps) {
			var best = k;
			for (j in (k + 1)...n)
				if (norms[j] > norms[best])
					best = j;
			if (best != k) {
				for (i in 0...m) {
					var t = a[i * n + k];
					a[i * n + k] = a[i * n + best];
					a[i * n + best] = t;
				}
				var tp = permutation[k];
				permutation[k] = permutation[best];
				permutation[best] = tp;
				var tn = norms[k];
				norms[k] = norms[best];
				norms[best] = tn;
			}
			var alpha = columnNorm(a, m, n, k, k);
			if (k == 0)
				largest = alpha;
			if (largest == 0 || alpha <= tolerance * largest)
				break;
			// Reflect rows k..m-1 so column k becomes (−sign·alpha, 0, …).
			var sign = a[k * n + k] >= 0 ? 1.0 : -1.0;
			var v = [for (i in k...m) a[i * n + k]];
			v[0] += sign * alpha;
			var vv = 0.0;
			for (x in v)
				vv += x * x;
			if (vv > 0)
				for (j in k...n) {
					var dot = 0.0;
					for (i in k...m)
						dot += v[i - k] * a[i * n + j];
					var f = 2 * dot / vv;
					for (i in k...m)
						a[i * n + j] -= f * v[i - k];
				}
			rank++;
			smallestKept = Math.min(smallestKept, alpha);
			for (j in (k + 1)...n)
				norms[j] = columnNorm(a, m, n, j, k + 1);
		}

		// Each dependent row j >= rank: solve R11 x = R12[:, j]; its circuit is j plus the pivots with weight.
		var circuits:Array<Array<Int>> = [];
		for (j in rank...n) {
			var x = [for (_ in 0...rank) 0.0];
			var i = rank - 1;
			while (i >= 0) {
				var sum = a[i * n + j];
				for (c in (i + 1)...rank)
					sum -= a[i * n + c] * x[c];
				x[i] = sum / a[i * n + i];
				i--;
			}
			var biggest = 0.0;
			for (value in x)
				biggest = Math.max(biggest, Math.abs(value));
			var circuit = [rowIds[permutation[j]]];
			for (c in 0...rank)
				if (biggest > 0 && Math.abs(x[c]) > MEMBER_TOLERANCE * biggest)
					circuit.push(rowIds[permutation[c]]);
			circuits.push(circuit);
		}
		var nearDegenerate = rank > 0 && smallestKept < NEAR_DEGENERATE_RATIO * tolerance * largest;
		return {rank: rank, circuits: circuits, nearDegenerate: nearDegenerate};
	}

	static function columnNorm(a:Array<Float>, m:Int, n:Int, column:Int, from:Int):Float {
		var sum = 0.0;
		for (i in from...m)
			sum += a[i * n + column] * a[i * n + column];
		return Math.sqrt(sum);
	}

	/** Circuits sharing an owner merge; a group's deficiency is its number of circuits. */
	static function mergeCircuits(owners:Array<String>, circuits:Array<Array<Int>>, unsatisfiedRow:Array<Bool>):Array<DependencyGroup> {
		var parent = [for (i in 0...circuits.length) i];
		var circuitOfOwner = new Map<String, Int>();
		for (index in 0...circuits.length)
			for (row in circuits[index]) {
				var seen = circuitOfOwner.get(owners[row]);
				if (seen == null)
					circuitOfOwner.set(owners[row], index);
				else
					join(parent, index, seen);
			}
		// Roots are circuit indices; `slot[root]` is the root's group in order of first appearance.
		var slot = [for (_ in 0...circuits.length) -1];
		var members:Array<Array<String>> = [], counts:Array<Int> = [], satisfied:Array<Bool> = [];
		for (index in 0...circuits.length) {
			var root = find(parent, index);
			if (slot[root] < 0) {
				slot[root] = members.length;
				members.push([]);
				counts.push(0);
				satisfied.push(true);
			}
			var group = slot[root];
			counts[group]++;
			for (row in circuits[index]) {
				if (members[group].indexOf(owners[row]) < 0)
					members[group].push(owners[row]);
				if (unsatisfiedRow[row])
					satisfied[group] = false;
			}
		}
		var groups = [for (group in 0...members.length) {
			members[group].sort(Reflect.compare);
			new DependencyGroup(members[group], counts[group], satisfied[group]);
		}];
		groups.sort((a, b) -> Reflect.compare(a.owners.join(","), b.owners.join(",")));
		return groups;
	}

	static function count(counts:Map<String, Int>, key:String):Int {
		var value = counts.get(key);
		return value == null ? 0 : value;
	}

	/**
		FreeCAD's heuristic, as a suggestion only: within each group, pick the
		unprotected owner in the most groups, then the one with fewer rows,
		then the one declared last, until the group's deficiency is used up.
	*/
	static function suggestRemovals(owners:Array<String>, groups:Array<DependencyGroup>, protectedOwners:Array<String>):Array<String> {
		var rowCount = new Map<String, Int>(), lastIndex = new Map<String, Int>(), groupCount = new Map<String, Int>();
		for (index in 0...owners.length) {
			rowCount.set(owners[index], count(rowCount, owners[index]) + 1);
			lastIndex.set(owners[index], index);
		}
		for (group in groups)
			for (owner in group.owners)
				groupCount.set(owner, count(groupCount, owner) + 1);
		var result:Array<String> = [];
		for (group in groups) {
			var candidates = [for (owner in group.owners) if (protectedOwners.indexOf(owner) < 0 && result.indexOf(owner) < 0) owner];
			candidates.sort((a, b) -> {
				if (count(groupCount, a) != count(groupCount, b))
					return count(groupCount, b) - count(groupCount, a);
				if (count(rowCount, a) != count(rowCount, b))
					return count(rowCount, a) - count(rowCount, b);
				return count(lastIndex, b) - count(lastIndex, a);
			});
			var needed = group.deficiency;
			for (owner in group.owners)
				if (result.indexOf(owner) >= 0)
					needed -= count(rowCount, owner);
			var index = 0;
			while (needed > 0 && index < candidates.length) {
				result.push(candidates[index]);
				needed -= count(rowCount, candidates[index]);
				index++;
			}
		}
		result.sort(Reflect.compare);
		return result;
	}

	/** Union-find root, halving the path on the way. */
	static function find(parent:Array<Int>, i:Int):Int {
		while (parent[i] != i) {
			parent[i] = parent[parent[i]];
			i = parent[i];
		}
		return i;
	}

	/** Joins two sets under the smaller root, so roots follow first appearance. */
	static function join(parent:Array<Int>, a:Int, b:Int):Void {
		var ra = find(parent, a), rb = find(parent, b);
		if (ra < rb)
			parent[rb] = ra;
		else if (rb < ra)
			parent[ra] = rb;
	}

	static function uniqueOwners(owners:Array<String>, rows:Array<Int>):Array<String> {
		var result:Array<String> = [];
		for (row in rows)
			if (result.indexOf(owners[row]) < 0)
				result.push(owners[row]);
		result.sort(Reflect.compare);
		return result;
	}
}
