package cadkit.sheetmetal;

/** Explicit per-bend angles. Missing, unknown or out-of-range entries are diagnosed by evaluation. */
class SheetState implements SheetAngles {
	final angles:Map<Int, Float>;
	public function new() {
		angles = new Map();
	}
	public function set(bend:Int, angle:Float):SheetState {
		if (!Math.isFinite(angle))
			throw new SheetDiagnosis("sheet.invalid-state", "State angles must be finite", ["bend:" + bend]);
		angles.set(bend, angle);
		return this;
	}
	public function angle(bend:Int):Float {
		var value = angles.get(bend);
		if (value == null)
			throw new SheetDiagnosis("sheet.invalid-state", "State omits a bend", ["bend:" + bend]);
		return value;
	}
	public function ids():Array<Int> {
		return [for (id in angles.keys()) id];
	}
}
