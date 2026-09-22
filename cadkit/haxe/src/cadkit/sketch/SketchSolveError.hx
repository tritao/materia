package cadkit.sketch;

class SketchSolveError extends haxe.Exception {
	public final diagnostic:SolveDiagnostic;
	public function new(diagnostic:SolveDiagnostic) {
		this.diagnostic = diagnostic;
		super(diagnostic.message);
	}
}
