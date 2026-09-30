package cadkit.solve;

/** The worst disagreement `JacobianCheck.compare` found between an analytic and a numeric Jacobian. */
class JacobianCheckResult {
	public final passed:Bool;
	public final row:Int;
	public final column:Int;
	public final analytic:Float;
	public final numeric:Float;
	/** `|analytic − numeric|` at (`row`, `column`) divided by the largest entry of that row. */
	public final error:Float;

	public function new(passed:Bool, row:Int, column:Int, analytic:Float, numeric:Float, error:Float) {
		this.passed = passed;
		this.row = row;
		this.column = column;
		this.analytic = analytic;
		this.numeric = numeric;
		this.error = error;
	}

	public function toString():String
		return (passed ? "ok" : "mismatch") + ' at ($row, $column): analytic $analytic, numeric $numeric, error $error';
}

/**
	Test oracle for analytic Jacobians: compares one against central
	differences. Jacobians are row-major, one row per residual and one column
	per variable. Not solver code: it evaluates the residual 2n times. The
	step for each variable is 1e-6 of its magnitude (at least 1e-3 of the
	largest variable's).
*/
class JacobianCheck {
	public static inline var DEFAULT_TOLERANCE:Float = 1e-6;

	public static function compare(residual:Array<Float>->Array<Float>, jacobian:Array<Float>->Array<Float>,
			x:Array<Float>, tolerance:Float = DEFAULT_TOLERANCE):JacobianCheckResult {
		if (x == null || x.length == 0) throw "Jacobian check needs at least one variable";
		if (!(tolerance > 0)) throw "Jacobian check tolerance must be positive";
		var base = residual(x.copy());
		var rows = base.length, columns = x.length;
		var analytic = jacobian(x.copy());
		if (analytic.length != rows * columns)
			throw 'Jacobian has ${analytic.length} entries, expected $rows x $columns';
		// Steps follow the variables' own magnitude, so a sketch a millionth the size is probed as finely.
		var reference = 0.0;
		for (value in x) reference = Math.max(reference, Math.abs(value));
		if (reference == 0) reference = 1;
		var numeric = [for (_ in 0...rows * columns) 0.0];
		for (column in 0...columns) {
			var step = 1e-6 * Math.max(Math.abs(x[column]), 1e-3 * reference);
			var plus = x.copy(), minus = x.copy();
			plus[column] += step;
			minus[column] -= step;
			var high = residual(plus), low = residual(minus);
			if (high.length != rows || low.length != rows) throw "Residual length changed between evaluations";
			for (row in 0...rows)
				numeric[row * columns + column] = (high[row] - low[row]) / (2 * step);
		}
		// Errors are relative to the row's largest entry, so the check is as strict at any scale.
		var worst:Null<JacobianCheckResult> = null;
		for (row in 0...rows) {
			var rowScale = 0.0;
			for (column in 0...columns)
				rowScale = Math.max(rowScale, Math.max(Math.abs(numeric[row * columns + column]), Math.abs(analytic[row * columns + column])));
			for (column in 0...columns) {
				var value = analytic[row * columns + column], expected = numeric[row * columns + column];
				var error = rowScale == 0 ? 0 : Math.abs(value - expected) / rowScale;
				if (!Math.isFinite(error)) error = Math.POSITIVE_INFINITY;
				if (worst == null || error > worst.error)
					worst = new JacobianCheckResult(error <= tolerance, row, column, value, expected, error);
			}
		}
		if (worst == null) return new JacobianCheckResult(true, -1, -1, 0, 0, 0);
		return worst;
	}
}
