import cadkit.solve.ConstraintDiagnosis;

/** Rank, dependency groups and feasibility on hand-built Jacobians whose answers are known. */
class ConstraintDiagnosisSmoke {
	public static function run():Void {
		var independent = diagnose([1, 0, 0, 1], 2, ["a", "b"], [0, 0]);
		check(independent.rank == 2 && independent.degreesOfFreedom == 0 && independent.dependencyGroups.length == 0,
			"two independent rows fix two variables");

		var free = diagnose([1, 0, 0], 3, ["a"], [0]);
		check(free.degreesOfFreedom == 2 && free.subsystems.length == 1 && free.subsystems[0].variables == 1,
			"untouched variables are free and belong to no subsystem");

		var duplicate = diagnose([1, 0, 1, 0, 0, 1], 2, ["a", "b", "c"], [0, 0.5, 0]);
		check(groups(duplicate) == "redundant(a,b)-1" && duplicate.degreesOfFreedom == 0, 'a repeated row is redundant: ${groups(duplicate)}');
		check(duplicate.suggestedRemovals.join(",") == "b", "the later of two equal candidates is suggested");

		var conflict = diagnose([1, 0, 1, 0, 0, 1], 2, ["a", "b", "c"], [-2, 2, 0]);
		check(groups(conflict) == "conflicting(a,b)-1", 'rows that cannot both hold conflict: ${groups(conflict)}');
		check(conflict.unsatisfied.join(",") == "a,b", "both sides of the conflict are unsatisfied");

		// x0 + x1, x1 + x2, x0 + 2 x1 + x2: the third is the sum of the first two.
		var triangle = diagnose([1, 1, 0, 0, 1, 1, 1, 2, 1], 3, ["a", "b", "c"], [0, 0, 0]);
		check(groups(triangle) == "redundant(a,b,c)-1" && triangle.degreesOfFreedom == 1, 'a sum is one group of three: ${groups(triangle)}');

		var separate = diagnose([1, 0, 1, 0, 0, 1, 0, 1], 2, ["a", "b", "c", "d"], [0, 0, 0, 0]);
		check(groups(separate) == "redundant(a,b)-1 redundant(c,d)-1" && separate.subsystems.length == 2,
			'independent dependencies stay separate groups: ${groups(separate)}');

		// The same system with one row a million times larger and one variable a million times smaller.
		var scaled = diagnose([1e6, 0, 1, 0, 0, 1e-6], 2, ["a", "b", "c"], [0, 0, 0]);
		check(groups(scaled) == "redundant(a,b)-1" && scaled.rank == 2, 'scaling rows and variables changes nothing: ${groups(scaled)}');

		// Pivot order must not matter: reversing the rows gives the same report.
		var reversed = diagnose([1, 2, 1, 0, 1, 1, 1, 1, 0], 3, ["c", "b", "a"], [0, 0, 0]);
		check(groups(reversed) == groups(triangle) && reversed.rank == triangle.rank, "row order does not change the groups");

		// A variable seen only with a tiny coefficient is just in small units: equilibration rescues it.
		var smallUnits = diagnose([1, 0, 1, 1e-6], 2, ["a", "b"], [0, 0]);
		check(smallUnits.rank == 2 && !smallUnits.nearDegenerate, "a variable in small units is not a degeneracy");
		var nearly = diagnose([1, 1, 1, 1 + 1e-7], 2, ["a", "b"], [0, 0]);
		check(nearly.rank == 2 && nearly.nearDegenerate, "rows nearly parallel are independent but flagged near-degenerate");
		var equalish = diagnose([1, 1, 1, 1 + 1e-12], 2, ["a", "b"], [0, 0]);
		check(equalish.rank == 1 && groups(equalish) == "redundant(a,b)-1", "rows 1e-12 apart are dependent at the default tolerance");

		var multiRow = diagnose([1, 0, 0, 1, 1, 0], 2, ["fix", "fix", "x"], [0, 0, 0], ["fix"]);
		check(multiRow.suggestedRemovals.join(",") == "x", "protected owners are never suggested");

		var threw = false;
		try diagnose([1, 0], 3, ["a"], [0]) catch (_:Dynamic) threw = true;
		check(threw, "a Jacobian of the wrong size is refused");
	}

	/** The dense diagnosis, checked against the sparse entry point, which must agree whichever path it takes. */
	static function diagnose(jacobian:Array<Float>, variables:Int, owners:Array<String>, residuals:Array<Float>,
			?protectedOwners:Array<String>):DiagnosisReport {
		var dense = ConstraintDiagnosis.diagnose({jacobian: jacobian, variables: variables, owners: owners, residuals: residuals,
			protectedOwners: protectedOwners});
		var rows = [for (row in 0...owners.length) {
			var index:Array<Int> = [], value:Array<Float> = [];
			for (column in 0...variables)
				if (jacobian[row * variables + column] != 0) { index.push(column); value.push(jacobian[row * variables + column]); }
			{index: index, value: value};
		}];
		// At the default tolerance the sparse path mostly defers to the QR; at 1e-6 it decides dependencies itself.
		for (tolerance in [ConstraintDiagnosis.DEFAULT_RANK_TOLERANCE, ConstraintDiagnosis.SPARSE_TOLERANCE]) {
			var reference = ConstraintDiagnosis.diagnose({jacobian: jacobian, variables: variables, owners: owners, residuals: residuals,
				protectedOwners: protectedOwners, rankTolerance: tolerance});
			var sparse = ConstraintDiagnosis.diagnoseSparse(rows, variables, owners, residuals, tolerance, protectedOwners);
			check(sparse.rank == reference.rank && groups(sparse) == groups(reference) && sparse.unsatisfied.join(",") == reference.unsatisfied.join(",")
				&& sparse.subsystems.length == reference.subsystems.length && sparse.suggestedRemovals.join(",") == reference.suggestedRemovals.join(","),
				'sparse and dense diagnoses agree at $tolerance: rank ${sparse.rank}/${reference.rank}, ${groups(sparse)} / ${groups(reference)}');
		}
		return dense;
	}

	static function groups(report:DiagnosisReport):String
		return [for (group in report.dependencyGroups) group.toString()].join(" ");

	static function check(value:Bool, message:String):Void {
		if (!value) throw message;
	}
}
