package machinekit.pressbrake.planning;

/** Feasibility depends on completed bends; preparation costs depend on the preceding search state. */
class BendSearchAction {
	public final id:Int;
	public final setup:BendSetupResult;
	public final gauge:BackgaugeStroke;
	public final grips:Array<SheetGrip>;
	public function new(id:Int,setup:BendSetupResult,gauge:BackgaugeStroke,grips:Array<SheetGrip>) {
		this.id = id; this.setup = setup; this.gauge = gauge; this.grips = grips;
	}
}
