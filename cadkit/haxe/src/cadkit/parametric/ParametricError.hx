package cadkit.parametric;

/** User-facing validation or document-graph failure. */
class ParametricError {
	public final message:String;

	public function new(message:String) {
		this.message = message;
	}

	public function toString():String {
		return message;
	}
}
