package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Sheet;

/** Phase B checks geometric grasp clearance; Phase C adds robot reach and IK through this boundary. */
interface GripCheck {
	function check(sheet:Sheet, setup:BendSetupResult, grip:SheetGrip, gauge:BackgaugeStroke):Void;
	/** Return one ordered typed outcome per candidate, sharing setup geometry where possible. */
	function checkCandidates(sheet:Sheet, setup:BendSetupResult, grips:Array<SheetGrip>, gauge:BackgaugeStroke):Array<GripCandidateResult>;
}
