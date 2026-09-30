package cadkit.sketch;

import cadkit.solve.ConstraintDiagnosis;

/**
	One solved part of a sketch, kept in its `SolvedSketch` so a later solve
	seeded from it can skip the part when nothing it depends on changed. The
	key (in `SolvedSketch.partCache`) is the part's structure; `values` holds
	the numbers (settings, dimensions, fixed positions), compared exactly.
*/
typedef CachedPart = {
	var values:Array<Float>;
	var report:DiagnosisReport;
	var degenerate:Bool;
}
