package cadkit.parametric;

/** User-facing validation or document-graph failure. */
class ParametricError {
	public final message:String;
	public final referenceState:Null<ReferenceState>;

	public function new(message:String, ?referenceState:ReferenceState) {
		this.message = message;
		this.referenceState = referenceState;
	}

	public function toString():String {
		return message;
	}
}
