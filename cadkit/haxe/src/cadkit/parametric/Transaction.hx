package cadkit.parametric;

import cadkit.parametric.ParametricError;
import cadkit.parametric.Document;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterChange;

/** Explicit grouping boundary for parameter edits. */
class Transaction {
	private static var nextIdentity:Int = 1;

	public final identity:Int;
	public final changes:Array<ParameterChange>;

	private final document:Document;
	private var finished:Bool;

	public function new(document:Document) {
		identity = nextIdentity;
		nextIdentity++;
		this.document = document;
		this.changes = [];
		this.finished = false;
	}

	public function commit():Void {
		if (finished)
			throw new ParametricError("transaction is already finished");
		finished = true;
		document.commitTransaction(this);
	}

	public function cancel():Void {
		if (finished)
			throw new ParametricError("transaction is already finished");
		finished = true;
		document.cancelTransaction(this);
	}

	public function record(parameter:Parameter, oldValue:Float):Void {
		for (change in changes) {
			if (change.parameter == parameter) {
				change.newValue = parameter.value;
				return;
			}
		}
		changes.push(new ParameterChange(parameter, oldValue, parameter.value));
	}
}
