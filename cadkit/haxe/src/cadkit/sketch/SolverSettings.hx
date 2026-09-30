package cadkit.sketch;

/** Settings are dimensionless: the solver divides lengths by the sketch's characteristic size. */
class SolverSettings {
	/** Largest acceptable residual norm, in scaled units. */
	public final tolerance:Float;
	/**
		Rows closer than this (relative) count as dependent (see `ConstraintDiagnosis`). At 1e-6 or more the
		diagnosis stays sparse even when it finds dependencies; below, those go through a dense QR.
	*/
	public final rankTolerance:Float;
	public final maxIterations:Int;
	public final initialDamping:Float;

	public function new(tolerance:Float = 1e-8, rankTolerance:Float = 1e-6, maxIterations:Int = 80, initialDamping:Float = 1e-3) {
		this.tolerance = tolerance;
		this.rankTolerance = rankTolerance;
		this.maxIterations = maxIterations;
		this.initialDamping = initialDamping;
	}
}
