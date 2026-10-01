package cadkit.sketch;

import cadkit.solve.ConstraintDiagnosis;

/**
	What a sketch part's structure fixes, kept in its `SolvedSketch` so a
	later solve of the same structure (a drag) reuses it even when dimension
	values change: the solve's envelope ordering (`position`/`first`), the
	diagnosis row graph and ordering, and the last diagnosis, which a solve
	without diagnosis reports as is.
*/
typedef PartStructure = {
	var position:Array<Int>;
	var first:Array<Int>;
	var rows:Null<RowStructure>;
	var report:DiagnosisReport;
	var degenerate:Bool;
	/** The part's variables (sketch-wide indices) that its constraints left free at the last diagnosis. */
	var free:Array<Int>;
}
