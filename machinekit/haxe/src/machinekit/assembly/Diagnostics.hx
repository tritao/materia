package machinekit.assembly;

enum abstract DiagnosticSeverity(String) to String {
	var Error = "error";
	var Warning = "warning";
}

typedef Diagnostic = {
	var severity:DiagnosticSeverity;
	var code:String;
	var subject:String;
	var message:String;
}

/** Ordered validation findings; callers can inspect all errors before acting. */
class Diagnostics {
	public final items:Array<Diagnostic> = [];

	public function new() {}

	public function add(severity:DiagnosticSeverity, code:String, subject:String, message:String):Void
		items.push({severity: severity, code: code, subject: subject, message: message});

	public function error(code:String, subject:String, message:String):Void add(Error, code, subject, message);
	public function warning(code:String, subject:String, message:String):Void add(Warning, code, subject, message);
	public function hasErrors():Bool return Lambda.exists(items, item -> item.severity == Error);
	public function warnings():Array<String> return [for (item in items) if (item.severity == Warning) item.message];
	public function throwIfErrors():Void {
		var errors = [for (item in items) if (item.severity == Error) item.message];
		if (errors.length > 0) throw errors.join("\n");
	}
}
