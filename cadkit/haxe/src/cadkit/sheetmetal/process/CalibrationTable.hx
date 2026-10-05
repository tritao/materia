package cadkit.sheetmetal.process;

import cadkit.sheetmetal.SheetDiagnosis;

/** Immutable measurements with exact lot, thickness and tool matching; no nearest-entry fallback. */
class CalibrationTable {
	final data:Array<CalibrationEntry>;
	public function new(entries:Array<CalibrationEntry>) {
		data = entries.copy();
		for (i in 0...data.length) for (j in 0...i) {
			var a = data[i]; var b = data[j];
			if (a.lot == b.lot && a.thickness == b.thickness && a.punchId == b.punchId && a.dieId == b.dieId
				&& Math.max(a.firstAngle, b.firstAngle) < Math.min(a.lastAngle, b.lastAngle))
				throw new SheetDiagnosis("sheet.ambiguous-calibration", "Calibration angle bands overlap for the same lot, thickness and tools", [a.lot, a.punchId, a.dieId]);
		}
	}
	public function entries():Array<CalibrationEntry> { return data.copy(); }
	public function find(lot:String, thickness:Float, tooling:ProcessTooling, angle:Float):Null<CalibrationEntry> {
		if (lot == null || lot.length == 0 || !Math.isFinite(thickness) || thickness <= 0
			|| !Math.isFinite(angle) || Math.abs(angle) <= 0 || Math.abs(angle) >= Math.PI)
			throw new SheetDiagnosis("sheet.invalid-calibration-key", "Calibration lookup needs an explicit lot, positive thickness and a nonzero bend angle below 180 degrees");
		var magnitude = Math.abs(angle);
		for (entry in data)
			if (entry.lot == lot && entry.thickness == thickness && entry.punchId == tooling.punchId && entry.dieId == tooling.dieId
				&& magnitude >= entry.firstAngle && magnitude < entry.lastAngle) return entry;
		return null;
	}
}
