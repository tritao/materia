package cadkit.parametric;

class ParameterExpressionChange implements DocumentChange {
	private final parameter:NamedParameter;
	private final oldExpression:Null<ParameterExpression>;
	private final newExpression:Null<ParameterExpression>;
	private final oldValue:Float;
	private final newValue:Float;

	public function new(parameter:NamedParameter, oldExpression:Null<ParameterExpression>, newExpression:Null<ParameterExpression>,
		oldValue:Float, newValue:Float) {
		this.parameter = parameter;
		this.oldExpression = oldExpression;
		this.newExpression = newExpression;
		this.oldValue = oldValue;
		this.newValue = newValue;
	}

	public function undo():Void {
		parameter.replaceExpression(oldExpression);
		parameter.synchronize(oldValue);
	}

	public function redo():Void {
		parameter.replaceExpression(newExpression);
		parameter.synchronize(newValue);
	}
}
