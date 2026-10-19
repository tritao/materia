package materia.tooling;

enum abstract ToolingCheckStatus(String) to String {
	var Pass = "pass";
	var Fail = "fail";
	var Pending = "pending";
	var Unsupported = "unsupported";
}

enum abstract ToolingSeverity(String) to String {
	var Info = "info";
	var Warning = "warning";
	var Error = "error";
}

/** Stable rule and subject IDs let later CAD and process checks share one report. */
typedef ToolingDiagnostic = {
	var ruleId:String;
	var subjectId:String;
	var severity:ToolingSeverity;
	var status:ToolingCheckStatus;
	var message:String;
	var evidence:Array<String>;
}
