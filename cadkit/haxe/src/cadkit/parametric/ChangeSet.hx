package cadkit.parametric;

import cadkit.parametric.ParameterChange;

/** Undoable group of parameter changes. */
class ChangeSet {
	public final changes:Array<ParameterChange>;
	public final documentChanges:Array<DocumentChange>;

	public function new(changes:Array<ParameterChange>, ?documentChanges:Array<DocumentChange>) {
		this.changes = changes;
		this.documentChanges = documentChanges == null ? [] : documentChanges;
	}
}
