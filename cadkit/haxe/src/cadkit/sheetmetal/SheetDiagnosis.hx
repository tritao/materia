package cadkit.sheetmetal;

/** A sheet failure names the rule and the geometry that failed it. */
class SheetDiagnosis {
	public final code:String;
	public final message:String;
	public final elements:Array<String>;

	public function new(code:String, message:String, ?elements:Array<String>) {
		this.code = code;
		this.message = message;
		this.elements = elements == null ? [] : elements.copy();
	}

	public function toString():String {
		return code + ": " + message + (elements.length == 0 ? "" : " [" + elements.join(", ") + "]");
	}
}
