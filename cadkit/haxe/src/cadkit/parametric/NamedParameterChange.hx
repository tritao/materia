package cadkit.parametric;

class NamedParameterChange implements DocumentChange {
	private final parameter:NamedParameter;
	private final oldValue:Float;
	private final newValue:Float;

	public function new(parameter:NamedParameter, oldValue:Float, newValue:Float) {
		this.parameter = parameter;
		this.oldValue = oldValue;
		this.newValue = newValue;
	}

	public function undo():Void {
		parameter.restoreStored(oldValue);
	}

	public function redo():Void {
		parameter.restoreStored(newValue);
	}
}
