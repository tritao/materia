package cadkit;

/** Checked failure raised by a CadKit result-policy wrapper. */
class CadKitError {
	public final status:Dynamic;
	public final operation:String;
	public final diagnostic:Null<String>;

	public function new(status:Dynamic, operation:String, diagnostic:Null<String>) {
		this.status = status;
		this.operation = operation;
		this.diagnostic = diagnostic;
	}

	public function toString():String {
		return diagnostic == null || diagnostic.length == 0
			? operation + " failed"
			: operation + " failed: " + diagnostic;
	}
}
