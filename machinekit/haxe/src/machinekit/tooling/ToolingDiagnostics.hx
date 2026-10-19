package machinekit.tooling;

import machinekit.assembly.Diagnostics;
import materia.tooling.ToolingValidation;

/** Use the same code/subject/severity vocabulary as machinekit.design. */
class ToolingDiagnostics {
	public static function fromValidation(records:Array<ToolingValidation>):Diagnostics {
		var result = new Diagnostics();
		for (record in records)
			if (record.status != "pass")
				for (id in record.affectedIds) {
					if (record.status == "fail" || record.status == "unsupported")
						result.error(record.rule, id, record.message);
					else
						result.warning(record.rule, id, record.message);
				}
		return result;
	}
}
