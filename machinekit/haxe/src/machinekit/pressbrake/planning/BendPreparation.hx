package machinekit.pressbrake.planning;

/** One action's preparation and timing, shared across its checked grip candidates. */
class BendPreparation {
	public final setup:BendSetupResult;
	public final gauge:BackgaugeStroke;
	public final flip:Bool;
	final previousGrip:Null<SheetGrip>;
	final heldTime:BendTimeBreakdown;
	final changedTime:BendTimeBreakdown;
	public function new(previous:Null<BendProgramStep>,setup:BendSetupResult,gauge:BackgaugeStroke,timing:BendTimeModel) {
		this.setup = setup; this.gauge = gauge;
		flip = previous == null ? !setup.orientation.faceAUp : previous.setup.orientation.faceAUp != setup.orientation.faceAUp;
		previousGrip = previous == null ? null : previous.grip;
		heldTime = timing.measure(previous,setup,gauge,flip,false);
		changedTime = previous == null ? heldTime : timing.measure(previous,setup,gauge,flip,true);
	}
	public function regrip(grip:SheetGrip):Bool {
		var held = previousGrip;
		return held != null && !BendTimeModel.sameGrip(held,grip);
	}
	public function time(grip:SheetGrip):BendTimeBreakdown return regrip(grip) ? changedTime : heldTime;
}
