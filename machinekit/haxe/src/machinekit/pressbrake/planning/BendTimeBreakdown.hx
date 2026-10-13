package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;

/** Nominal action seconds are separate from simulation wall time. */
class BendTimeBreakdown {
	public final handling:Float;
	public final flip:Float;
	public final regrip:Float;
	public final station:Float;
	public final backgauge:Float;
	public final bend:Float;
	public var total(get,never):Float;
	public function new(handling:Float,flip:Float,regrip:Float,station:Float,backgauge:Float,bend:Float) {
		for (value in [handling,flip,regrip,station,backgauge,bend])
			if (!Math.isFinite(value) || value < 0) throw new SheetDiagnosis("press.time-model-range","Action times must be finite nonnegative seconds");
		if (!Math.isFinite(handling+flip+regrip+station+backgauge+bend))
			throw new SheetDiagnosis("press.time-model-range","Total action time exceeds the finite range");
		this.handling = handling; this.flip = flip; this.regrip = regrip;
		this.station = station; this.backgauge = backgauge; this.bend = bend;
	}
	function get_total():Float return handling+flip+regrip+station+backgauge+bend;
}
