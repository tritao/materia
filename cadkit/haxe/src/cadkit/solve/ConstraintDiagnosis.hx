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

/**
	A row graph for `ConstraintDiagnosis.diagnoseSparse` and its envelope
	ordering, computed once by a caller whose structure does not change
	between diagnoses (a drag). `neighbours` must include every pair of rows
	that can share a variable, so no entry ever falls outside the envelope.
*/
typedef RowStructure = {
	var neighbours:Array<Array<Int>>;
	var position:Array<Int>;
	var first:Array<Int>;
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

	/**
		`diagnose` for sparse rows (`index`/`value` pairs, each variable once
		per row), without a dense QR in the usual cases. JJᵀ (equilibrated) is
		factored by Cholesky in reverse Cuthill-McKee order. A Gram pivot is
		about the square of the row's distance from the earlier rows, and is
		accurate to about 1e-16, so with a rank tolerance t of at least
		`SPARSE_TOLERANCE` a pivot of at most t² drops the row as dependent
		(its fundamental circuit read from the factor) and any other keeps it.
		With a smaller t the sparse path can only prove clear independence
		(pivot ≥ `INDEPENDENT_PIVOT`); anything else goes to the dense QR.
	*/
	public static function diagnoseSparse(rows:Array<{index:Array<Int>, value:Array<Float>}>, variables:Int, owners:Array<String>,
			residuals:Array<Float>, ?rankTolerance:Float, ?protectedOwners:Array<String>, ?structure:RowStructure):DiagnosisReport {
		if (rows.length != owners.length || rows.length != residuals.length)
			throw 'Diagnosis has ${rows.length} rows, ${owners.length} owners and ${residuals.length} residuals';
		var tolerance = rankTolerance == null ? DEFAULT_RANK_TOLERANCE : rankTolerance;
		var factored = sparseFactor(rows, tolerance, structure);
		if (factored != null) {
			var unsatisfiedRow = [for (row in 0...rows.length) !(Math.abs(residuals[row]) <= 1)];
			var groups = mergeCircuits(owners, factored.circuits, unsatisfiedRow);
			var unsatisfied = uniqueOwners(owners, [for (row in 0...rows.length) if (unsatisfiedRow[row]) row]);
			return new DiagnosisReport(variables, rows.length - factored.circuits.length,
				sparseSubsystems(rows, owners, variables, factored.dropped), groups, unsatisfied, factored.nearDegenerate,
				suggestRemovals(owners, groups, protectedOwners == null ? [] : protectedOwners));
		}
		var dense = [for (_ in 0...rows.length * variables) 0.0];
		for (row in 0...rows.length)
			for (e in 0...rows[row].index.length)
				dense[row * variables + rows[row].index[e]] += rows[row].value[e];
		return diagnose({jacobian: dense, variables: variables, owners: owners, residuals: residuals,
			rankTolerance: rankTolerance, protectedOwners: protectedOwners});
	}

	/** Below this rank tolerance a Gram pivot cannot be told from rounding, so only clear independence is decided here. */
	public static inline var SPARSE_TOLERANCE:Float = 1e-6;
	/** A pivot that proves independence whatever the tolerance. */
	static inline var INDEPENDENT_PIVOT:Float = 1e-8;

	/**
		Cholesky of JJᵀ with dependent rows dropped. Returns each dropped row's
		fundamental circuit (the row and the kept rows its coefficients reach)
		and which rows were dropped; null when a pivot is ambiguous.
	*/
	static function sparseFactor(source:Array<{index:Array<Int>, value:Array<Float>}>, tolerance:Float,
			?structure:RowStructure):Null<{circuits:Array<Array<Int>>, dropped:Array<Bool>, nearDegenerate:Bool}> {
		// Dependent at or below t², independent above it; near-degenerate below (1e3 t)², as the dense flag.
		var decides = tolerance >= SPARSE_TOLERANCE;
		var dependentPivot = decides ? tolerance * tolerance : -1.0;
		var keptPivot = decides ? tolerance * tolerance : INDEPENDENT_PIVOT;
		var nearPivot = decides ? 1e6 * tolerance * tolerance : 0.0;
		var nearDegenerate = false;
		var m = source.length;
		// Equilibrate as `factor` does (rows, columns, rows), so a variable in small units is not a dependency.
		var rows = [for (row in source) {index: row.index, value: row.value.copy()}];
		normalizeRows(rows);
		var columnNorm = new Map<Int, Float>();
		for (row in rows)
			for (e in 0...row.index.length) {
				var old = columnNorm.get(row.index[e]);
				columnNorm.set(row.index[e], (old == null ? 0 : old) + row.value[e] * row.value[e]);
			}
		for (row in rows)
			for (e in 0...row.index.length) {
				var norm = columnNorm.get(row.index[e]);
				if (norm != null && norm > 0) row.value[e] /= Math.sqrt(norm);
			}
		var scale = normalizeRows(rows);
		if (structure == null) structure = rowGraph(rows);
		var neighbours = structure.neighbours, ordering = structure, first = structure.first;
		var original = [for (_ in 0...m) 0];
		for (row in 0...m) original[ordering.position[row]] = row;
		var envelope = EnvelopeCholesky.zero(first);
		for (a in 0...m) {
			var pa = ordering.position[a];
			envelope[pa][pa - first[pa]] = scale[a] > 0 ? 1 : 0;
			if (scale[a] > 0)
				for (b in neighbours[a]) {
					var pb = ordering.position[b];
					if (pb < pa && scale[b] > 0) envelope[pa][pb - first[pa]] = dot(rows[a], rows[b]);
				}
		}
		var dropped = [for (_ in 0...m) false], droppedRows:Array<Int> = [], droppedFactors:Array<Array<Float>> = [];
		for (p in 0...m) {
			var rowFirst = first[p];
			for (q in rowFirst...p) {
				if (dropped[q]) { envelope[p][q - rowFirst] = 0; continue; }
				var sum = envelope[p][q - rowFirst];
				var from = rowFirst > first[q] ? rowFirst : first[q];
				for (k in from...q) sum -= envelope[p][k - rowFirst] * envelope[q][k - first[q]];
				envelope[p][q - rowFirst] = sum / envelope[q][q - first[q]];
			}
			var pivot = envelope[p][p - rowFirst];
			for (k in rowFirst...p) pivot -= envelope[p][k - rowFirst] * envelope[p][k - rowFirst];
			if (pivot > keptPivot) {
				envelope[p][p - rowFirst] = Math.sqrt(pivot);
				if (pivot < nearPivot) nearDegenerate = true;
			} else if (pivot <= dependentPivot) {
				dropped[p] = true;
				droppedRows.push(p);
				droppedFactors.push(envelope[p].copy());
				for (k in 0...envelope[p].length) envelope[p][k] = 0;
				envelope[p][p - rowFirst] = 1;
			} else
				return null;
		}
		// A dropped row's factor row y solves L y = G[kept, row]; Lᵀ c = y gives its coefficients over earlier kept rows.
		var circuits:Array<Array<Int>> = [];
		for (index in 0...droppedRows.length) {
			var p = droppedRows[index], y = droppedFactors[index], rowFirst = first[p];
			var c = [for (_ in 0...p) 0.0];
			for (q in rowFirst...p) c[q] = y[q - rowFirst];
			// y is zero before first[p], but the coefficients can reach further back through earlier rows.
			var r = p - 1;
			while (r >= 0) {
				if (dropped[r])
					c[r] = 0;
				else if (c[r] != 0) {
					c[r] /= envelope[r][r - first[r]];
					for (q in first[r]...r) c[q] -= envelope[r][q - first[r]] * c[r];
				}
				r--;
			}
			var biggest = 0.0;
			for (value in c) biggest = Math.max(biggest, Math.abs(value));
			var circuit = [original[p]];
			for (q in 0...p)
				if (biggest > 0 && Math.abs(c[q]) > MEMBER_TOLERANCE * biggest)
					circuit.push(original[q]);
			circuits.push(circuit);
		}
		return {circuits: circuits, dropped: [for (row in 0...m) dropped[ordering.position[row]]], nearDegenerate: nearDegenerate};
	}

	/** Rows are adjacent when they share a variable; ordered by reverse Cuthill-McKee. */
	public static function rowGraph(rows:Array<{index:Array<Int>, value:Array<Float>}>):RowStructure {
		var rowsOf = new Map<Int, Array<Int>>();
		for (row in 0...rows.length)
			for (variable in rows[row].index) {
				var list = rowsOf.get(variable);
				if (list == null) { list = []; rowsOf.set(variable, list); }
				list.push(row);
			}
		var neighbours:Array<Array<Int>> = [for (_ in 0...rows.length) []];
		for (list in rowsOf)
			for (a in list) for (b in list)
				if (a != b && neighbours[a].indexOf(b) < 0) neighbours[a].push(b);
		var ordering = EnvelopeCholesky.order(neighbours);
		return {neighbours: neighbours, position: ordering.position, first: ordering.first};
	}

	/** Scales each row to unit norm in place; returns the norms before scaling (0 for an empty row, left as is). */
	static function normalizeRows(rows:Array<{index:Array<Int>, value:Array<Float>}>):Array<Float> {
		return [for (row in rows) {
			var sum = 0.0;
			for (value in row.value) sum += value * value;
			var norm = Math.sqrt(sum);
			if (norm > 0) for (e in 0...row.value.length) row.value[e] /= norm;
			norm;
		}];
	}

	static function dot(a:{index:Array<Int>, value:Array<Float>}, b:{index:Array<Int>, value:Array<Float>}):Float {
		var sum = 0.0;
		for (i in 0...a.index.length)
			for (j in 0...b.index.length)
				if (a.index[i] == b.index[j]) sum += a.value[i] * b.value[j];
		return sum;
	}

	/** Subsystems as `components` finds them (shared nonzero variable or owner), from sparse rows. */
	static function sparseSubsystems(rows:Array<{index:Array<Int>, value:Array<Float>}>, owners:Array<String>, variables:Int,
			dropped:Array<Bool>):Array<DiagnosisSubsystem> {
		var m = rows.length, parent = [for (i in 0...m) i];
		var firstRowOfColumn = [for (_ in 0...variables) -1], firstRowOfOwner = new Map<String, Int>();
		for (row in 0...m) {
			var seen = firstRowOfOwner.get(owners[row]);
			if (seen == null) firstRowOfOwner.set(owners[row], row); else join(parent, row, seen);
			for (e in 0...rows[row].index.length)
				if (rows[row].value[e] != 0) {
					var column = rows[row].index[e];
					if (firstRowOfColumn[column] < 0) firstRowOfColumn[column] = row; else join(parent, row, firstRowOfColumn[column]);
				}
		}
		var slot = [for (_ in 0...m) -1];
		var members:Array<Array<Int>> = [], columnCounts:Array<Int> = [];
		for (row in 0...m) {
			var root = find(parent, row);
			if (slot[root] < 0) { slot[root] = members.length; members.push([]); columnCounts.push(0); }
			members[slot[root]].push(row);
		}
		for (column in 0...variables)
			if (firstRowOfColumn[column] >= 0) columnCounts[slot[find(parent, firstRowOfColumn[column])]]++;
		var result = [for (i in 0...members.length) new DiagnosisSubsystem(uniqueOwners(owners, members[i]), columnCounts[i],
			[for (row in members[i]) if (!dropped[row]) row].length)];
		result.sort((a, b) -> Reflect.compare(a.owners.join(","), b.owners.join(",")));
		return result;
	}

	/**
		Combines reports of disjoint parts of one system with `variables` in
		all: ranks add, lists concatenate (sorted), and variables no part
		touches count as free.
	*/
	public static function merge(reports:Array<DiagnosisReport>, variables:Int):DiagnosisReport {
		var rank = 0, nearDegenerate = false;
		var subsystems:Array<DiagnosisSubsystem> = [], groups:Array<DependencyGroup> = [];
		var unsatisfied:Array<String> = [], suggestions:Array<String> = [];
		for (report in reports) {
			rank += report.rank;
			nearDegenerate = nearDegenerate || report.nearDegenerate;
			for (subsystem in report.subsystems) subsystems.push(subsystem);
			for (group in report.dependencyGroups) groups.push(group);
			for (owner in report.unsatisfied) if (unsatisfied.indexOf(owner) < 0) unsatisfied.push(owner);
			for (owner in report.suggestedRemovals) if (suggestions.indexOf(owner) < 0) suggestions.push(owner);
		}
		// Sort by precomputed keys: joining owner lists inside the comparator dominated merging many parts.
		var subsystemKeys = [for (subsystem in subsystems) {key: subsystem.owners.join(","), value: subsystem}];
		subsystemKeys.sort((a, b) -> Reflect.compare(a.key, b.key));
		subsystems = [for (entry in subsystemKeys) entry.value];
		var groupKeys = [for (group in groups) {key: group.owners.join(","), value: group}];
		groupKeys.sort((a, b) -> Reflect.compare(a.key, b.key));
		groups = [for (entry in groupKeys) entry.value];
		unsatisfied.sort(Reflect.compare);
		suggestions.sort(Reflect.compare);
		return new DiagnosisReport(variables, rank, subsystems, groups, unsatisfied, nearDegenerate, suggestions);
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
