package cadkit.parametric;

/** Undoable selection of the document's primary output feature. */
class OutputSelectionChange implements DocumentChange {
	private final document:Document;
	private final before:Null<Feature>;
	private final after:Null<Feature>;

	public function new(document:Document, before:Null<Feature>, after:Null<Feature>) {
		this.document = document;
		this.before = before;
		this.after = after;
	}

	public function undo():Void
		document.restoreOutputSelection(before);

	public function redo():Void
		document.restoreOutputSelection(after);
}
