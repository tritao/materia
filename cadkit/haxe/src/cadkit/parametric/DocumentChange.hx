package cadkit.parametric;

/** Non-scalar document edit participating in transactions and undo/redo. */
interface DocumentChange {
	public function undo():Void;
	public function redo():Void;
}
