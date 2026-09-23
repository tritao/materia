package cadkit.sketch;

import cadkit.modeling.Sketch;

/**
	 * An editable sketch draft whose solve state is independent of profile construction.
	 * Open and under-constrained sketches can be solved and inspected before they are
	 * ready to drive a solid feature.
	 */
class SketchSession {
	private var authored:ConstrainedSketch;
	public var solution(default, null):Null<SolvedSketch>;
	public var lastValidSolution(default, null):Null<SolvedSketch>;
	public var diagnostic(default, null):Null<SolveDiagnostic>;
	public var degreesOfFreedom(get, never):Int;
	public var conflictingConstraintIds(get, never):Array<String>;
	public var isSolved(get, never):Bool;

	public function new(sketch:ConstrainedSketch) {
		if (sketch == null)
			throw "sketch sessions require an authored sketch";
		authored = sketch.copy();
		solution = null;
		lastValidSolution = null;
		diagnostic = null;
		solveDraft();
	}

	/** Return an isolated copy of the current authored draft. */
	public function snapshot():ConstrainedSketch
		return authored.copy();

	/**
	 * Apply one edit to a private copy, then solve it. Solver conflicts remain in
	 * the draft for repair; malformed edits throw before replacing the current draft.
	 */
	public function edit(change:ConstrainedSketch->Void):Bool {
		if (change == null)
			throw "sketch edit callback must not be null";
		var candidate = authored.copy();
		change(candidate);
		authored = candidate;
		return solveDraft();
	}

	/** Build a profile only when the current draft has a valid current solution. */
	public function buildProfile():Sketch {
		if (solution == null)
			throw new SketchSolveError(diagnostic == null
				? new SolveDiagnostic("invalid", false, 0, 0, 0, [], "sketch has no valid current solution")
				: diagnostic);
		return SketchProfile.build(authored, solution);
	}

	function solveDraft():Bool {
		try {
			var candidate = authored.solve(lastValidSolution);
			solution = candidate;
			lastValidSolution = candidate;
			diagnostic = candidate.diagnostic;
			return true;
		} catch (error:SketchSolveError) {
			solution = null;
			diagnostic = error.diagnostic;
			return false;
		}
	}

	function get_degreesOfFreedom():Int
		return diagnostic == null ? 0 : diagnostic.degreesOfFreedom;

	function get_conflictingConstraintIds():Array<String>
		return diagnostic == null ? [] : diagnostic.constraintIds.copy();

	function get_isSolved():Bool
		return solution != null && solution.diagnostic.converged;
}
