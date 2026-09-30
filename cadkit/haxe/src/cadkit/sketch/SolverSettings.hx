package cadkit.sketch;

/** Settings are dimensionless: the solver divides lengths by the sketch's characteristic size. */
class SolverSettings {
	/** Largest acceptable residual norm, in scaled units. */
	public final tolerance:Float;
	/** Pivots smaller than this fraction of the largest count as rank-deficient (see `ConstraintDiagnosis`). */
	public final rankTolerance:Float;
	public final maxIterations:Int;
	public final initialDamping:Float;

	public function new(tolerance:Float = 1e-8, rankTolerance:Float = 1e-7, maxIterations:Int = 80, initialDamping:Float = 1e-3) {
		this.tolerance = tolerance;
		this.rankTolerance = rankTolerance;
		this.maxIterations = maxIterations;
		this.initialDamping = initialDamping;
	}
}
