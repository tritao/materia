package machinekit.pressbrake.planning;

/** One checked bend with its preparation markers, registered/withdrawn gauge and nominal time. */
class BendProgramStep {
	public final setup:BendSetupResult;
	public final gauge:BackgaugeStroke;
	public final grip:SheetGrip;
	public final flip:Bool;
	public final regrip:Bool;
	public final time:BendTimeBreakdown;
	public function new(preparation:BendPreparation,grip:SheetGrip) {
		setup = preparation.setup; gauge = preparation.gauge; this.grip = grip;
		flip = preparation.flip; regrip = preparation.regrip(grip); time = preparation.time(grip);
	}
}
