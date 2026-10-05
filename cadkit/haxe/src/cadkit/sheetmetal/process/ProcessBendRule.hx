package cadkit.sheetmetal.process;

import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;

/** Immutable air-bending setup; geometry and allowances follow its current stock and measurements. */
class ProcessBendRule {
	public final tooling:ProcessTooling;
	public final calibration:CalibrationTable;
	public final lot:Null<String>;
	final process:BendProcess;

	public function new(tooling:ProcessTooling, ?calibration:CalibrationTable, ?lot:String) {
		if (lot != null && lot.length == 0)
			throw new SheetDiagnosis("sheet.invalid-calibration-key", "A process lot must be explicit or omitted");
		this.tooling = tooling;
		this.calibration = calibration == null ? new CalibrationTable([]) : calibration;
		this.lot = lot;
		process = new AirBending(this.calibration);
	}
	public function predict(material:SheetMaterial, angle:Float, length:Float):BendProcessResult {
		return process.predict(material, tooling, angle, length, lot);
	}
	public function allowance(material:SheetMaterial, radius:Float, angle:Float, length:Float):Float {
		if (angle == 0) return 0;
		var result = predict(material, angle, length);
		if (Math.abs(radius - result.innerRadius) > 1e-8)
			throw new SheetDiagnosis("sheet.process-radius-mismatch", "Authored inner radius " + radius
				+ " differs from the tooling's released radius " + result.innerRadius, [tooling.punchId, tooling.dieId]);
		return result.bendAllowance;
	}
	public function record():ProcessRuleRecord {
		return {tooling: {punchId: tooling.punchId, dieId: tooling.dieId,
			punchRadius: tooling.punchRadius, punchAngle: tooling.punchAngle, vWidth: tooling.vWidth,
			dieAngle: tooling.dieAngle, shoulderRadius: tooling.shoulderRadius}, lot: lot,
			calibration: [for (entry in calibration.entries()) {lot: entry.lot, thickness: entry.thickness,
				punchId: entry.punchId, dieId: entry.dieId, firstAngle: entry.firstAngle,
				lastAngle: entry.lastAngle, depthOffset: entry.depthOffset, innerRadius: entry.innerRadius}]};
	}
	public static function fromRecord(record:ProcessRuleRecord):ProcessBendRule {
		var tools = record.tooling;
		return new ProcessBendRule(new ProcessTooling(tools.punchId, tools.dieId, tools.punchRadius,
			tools.punchAngle, tools.vWidth, tools.dieAngle, tools.shoulderRadius),
			new CalibrationTable([for (entry in record.calibration) new CalibrationEntry(entry.lot,
				entry.thickness, entry.punchId, entry.dieId, entry.firstAngle, entry.lastAngle,
				entry.depthOffset, entry.innerRadius)]), record.lot);
	}
}
