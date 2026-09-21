package cadkit;

import cadkit.SelectionErrorKind;

/** A selector could not produce the requested deterministic result. */
class SelectionError {
	public final kind:SelectionErrorKind;
	public final message:String;

	public function new(kind:SelectionErrorKind, message:String) {
		this.kind = kind;
		this.message = message;
	}

	public function toString():String {
		return message;
	}
}
