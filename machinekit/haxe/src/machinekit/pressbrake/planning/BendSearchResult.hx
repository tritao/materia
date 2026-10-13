package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;

/** A bounded search never confuses budget exhaustion with proving the candidate domain infeasible. */
enum BendSearchResult {
	Planned(program:BendProgram);
	NoSequence(failures:Array<BendSequenceFailure>);
	SearchLimit(failures:Array<BendSequenceFailure>, reason:SheetDiagnosis);
}
