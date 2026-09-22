package cadkit.sketch;

class SolverSettings {
	public final tolerance:Float;
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
