package materia.tooling;

import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

typedef ToolingValidation = {
	var rule:String;
	var affectedIds:Array<String>;
	var status:ToolingCheckStatus;
	var severity:ToolingSeverity;
	var message:String;
	var evidence:Array<String>;
}

typedef FabricationReadiness = {
	var ready:Bool;
	var blockingRules:Array<String>;
}

/** Qualification is separate from successful planning and CAD reconstruction. */
class ToolingReadiness {
	public static function pending(rule:String, ids:Array<String>, message:String):ToolingValidation
		return {
			rule: rule,
			affectedIds: ids.copy(),
			status: Pending,
			severity: Warning,
			message: message,
			evidence: []
		};

	public static function evaluate(checks:Array<ToolingValidation>):FabricationReadiness {
		var blocked:Array<String> = [];
		if (checks == null || checks.length == 0)
			throw "Missing fabrication validation";
		for (check in checks) {
			if (check.rule == null || check.rule.length == 0 || check.affectedIds == null || check.affectedIds.length == 0)
				throw "Invalid tooling validation";
			for (id in check.affectedIds)
				ToolingIds.validate(id);
			if (check.status != Pass && check.status != Fail && check.status != Pending && check.status != Unsupported)
				throw "Invalid tooling validation status";
			var expected = check.status == Pass ? Info : check.status == Pending ? Warning : Error;
			if (check.severity != expected)
				throw "Invalid tooling validation severity/status consistency";
			if (check.status != Pass && blocked.indexOf(check.rule) < 0)
				blocked.push(check.rule);
		}
		blocked.sort(Reflect.compare);
		return {ready: blocked.length == 0, blockingRules: blocked};
	}
}
