package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetState;

/** Derived executable bend data; authoring edits require planning again. */
class BendProgram {
	final ordered:Array<BendProgramStep>;
	public var expectedTime(get,never):Float;
	public function new(steps:Array<BendProgramStep>) { ordered = steps.copy(); }
	public function steps():Array<BendProgramStep> return ordered.copy();
	function get_expectedTime():Float { var seconds = 0.0; for (step in ordered) seconds += step.time.total; return seconds; }
	public function finalState():SheetState {
		var state = new SheetState();
		for (step in ordered) { var applied = state.set(step.setup.bend,step.setup.process.targetAngle); }
		return state;
	}
}
