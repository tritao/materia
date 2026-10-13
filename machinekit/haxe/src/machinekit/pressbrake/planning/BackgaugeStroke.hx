package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;

/** Register first, then withdraw X while the press holds the flat stock before starting the bend. */
class BackgaugeStroke {
	public final registeredX:Float;
	public final strokeX:Float;
	public final r:Float;
	public function new(registration:BendRegistration, retract:Float) {
		if (registration == null || !Math.isFinite(retract) || retract < 0)
			throw new SheetDiagnosis("press.gauge-retract", "Backgauge withdrawal must be finite nonnegative mm");
		registeredX = registration.x; strokeX = registration.x + retract; r = registration.r;
		if (!Math.isFinite(strokeX))
			throw new SheetDiagnosis("press.gauge-retract", "Backgauge withdrawal exceeds the finite range");
	}
	public function withdrawal(samples:Int = 20):Array<Float> {
		if (samples < 1 || samples > 3600)
			throw new SheetDiagnosis("press.gauge-retract", "Backgauge withdrawal needs between 1 and 3600 sample intervals");
		return [for (i in 0...(samples + 1)) registeredX + (strokeX - registeredX) * i / samples];
	}
	public function validate(registration:BendRegistration):Void {
		if (registeredX != registration.x || r != registration.r)
			throw new SheetDiagnosis("press.gauge-retract", "Backgauge stroke must start at the setup registration");
	}
}
