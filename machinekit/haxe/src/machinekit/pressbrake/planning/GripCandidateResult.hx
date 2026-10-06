package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;

/** One authored grip's outcome from a shared setup-geometry clearance pass. */
enum GripCandidateResult {
	GripAccepted(grip:SheetGrip);
	GripRejected(grip:SheetGrip, diagnosis:SheetDiagnosis);
}
