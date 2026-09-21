package cadkit.parametric;

import cadkit.parametric.ParameterChange;

/** Undoable group of parameter changes. */
class ChangeSet {
	public final changes:Array<ParameterChange>;

	public function new(changes:Array<ParameterChange>) {
		this.changes = changes;
	}
}
